#!/usr/bin/env python3
"""
Generates pins_gen.inc and pin_mapping_gen.inc from the STM32CubeMX MCU
database (XML files shipped with STM32CubeMX), covering every MCU of every
STM32 family whose CMSIS device header uses the "clean" <subline>xx compile
macro scheme (one macro per silicon die, independent of package/flash size -
the same scheme already used by STM32G431xx/STM32H573xx elsewhere in this
repo).

Families deliberately EXCLUDED: F0, F1, F3, L0, L1 (their CMSIS headers use
flash-density-suffixed macros, e.g. STM32F103xB, which cannot be derived
reliably from the MCU XML alone) and N6/U3 (no public cmsis_device_* header
was reachable to verify their macro scheme against). All of these families
also use a different GPIO/RCC register layout in the F1/F3/L1 case, which
gpio.hh (MODER/OTYPER/OSPEEDR/PUPDR/BSRR + RCC_AHB2ENR-style enable) does not
model anyway.

Structure:
  - pins_gen.inc / pin_mapping_gen.inc are now thin dispatchers:
    #if defined(<macro>) / #include "generated/pins/<macro>.inc" / #elif ...
  - generated/pins/<macro>.inc: physical GPIO pins of that exact die, as
    Pxnn identifiers (union across every package sharing that macro).
  - generated/mapping/<macro>.inc: PinAfMapping table for those pins, for
    every alternate-function-muxed peripheral/signal the MCU database has
    (no curated subset).
  - generated/fixed_signals/<macro>.inc: PinFixedSignal table for signals
    that are physically wired to a pin but not AF-muxed (ADC/COMP/OPAMP/DAC
    channels, RTC_TAMP, SYS_WKUP, RCC_LSCO, UCPD_FRSTX, ...) - sourced from
    the per-MCU XML's <Pin><Signal> list rather than the GPIO IP Modes file.
  - generated/peripherals_present/<macro>.inc: which Peripheral enum values
    this exact chip actually has, from the per-MCU XML's top-level
    <IP InstanceName="..."/> list.
  - generated/peripherals_gen.inc, generated/signals_gen.inc: every distinct
    Peripheral/Signal enumerator found across the AF and fixed-signal tables,
    included by pin_mapping.hh's Peripheral/Signal enum definitions. A signal
    with no '_' in its name (e.g. EVENTOUT, CEC, AUDIOCLK) becomes its own
    Peripheral with Signal::None, since it IS the function, not a sub-signal
    of one.
  - EXTI line is NOT generated - it always equals the pin number (0-15) on
    every STM32 family (verified against the DB), so pin_mapping.hh computes
    it with plain arithmetic instead.

Each family's macro whitelist below was read verbatim from the real
STMicroelectronics/cmsis_device_<family> stm32<family>xx.h master header
(GitHub) - or, for G4/H5, from the STM32CubeMX-downloaded firmware package
cached locally - not guessed from the XML naming pattern. Where a subline
has package- or flash-specific macros instead of a plain "xx" (e.g.
STM32F410Tx/Cx/Rx, STM32H745xx vs STM32H745xG), every real variant is listed
so the matcher below can disambiguate per exact part.

Run from anywhere:
    python generate_pin_files.py
"""
import re
import xml.etree.ElementTree as ET
from pathlib import Path

MCU_DB_DIR = Path(r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeMX\db\mcu")
OUT_DIR = Path(__file__).resolve().parent
GENERATED_DIR = OUT_DIR / "generated"

# family glob prefix -> verbatim macro whitelist (from the family's real CMSIS
# device master header). 'x'/'X' in a macro is a wildcard (flash size/package).
FAMILY_MACROS: dict[str, list[str]] = {
    "STM32C0": ["STM32C011xx", "STM32C031xx", "STM32C051xx", "STM32C071xx", "STM32C091xx", "STM32C092xx"],
    "STM32F2": ["STM32F205xx", "STM32F215xx", "STM32F207xx", "STM32F217xx"],
    "STM32F4": [
        "STM32F405xx", "STM32F415xx", "STM32F407xx", "STM32F417xx", "STM32F427xx", "STM32F437xx",
        "STM32F429xx", "STM32F439xx", "STM32F401xC", "STM32F401xE", "STM32F410Tx", "STM32F410Cx",
        "STM32F410Rx", "STM32F411xE", "STM32F446xx", "STM32F469xx", "STM32F479xx", "STM32F412Cx",
        "STM32F412Zx", "STM32F412Rx", "STM32F412Vx", "STM32F413xx", "STM32F423xx",
    ],
    "STM32F7": [
        "STM32F722xx", "STM32F723xx", "STM32F732xx", "STM32F733xx", "STM32F756xx", "STM32F746xx",
        "STM32F745xx", "STM32F765xx", "STM32F767xx", "STM32F769xx", "STM32F777xx", "STM32F779xx",
        "STM32F730xx", "STM32F750xx",
    ],
    "STM32G0": [
        "STM32G0B1xx", "STM32G0C1xx", "STM32G0B0xx", "STM32G071xx", "STM32G081xx", "STM32G070xx",
        "STM32G031xx", "STM32G041xx", "STM32G030xx", "STM32G051xx", "STM32G061xx", "STM32G050xx",
    ],
    "STM32G4": [
        "STM32G431xx", "STM32G441xx", "STM32G471xx", "STM32G473xx", "STM32G483xx", "STM32G474xx",
        "STM32G484xx", "STM32G491xx", "STM32G4A1xx", "STM32GBK1CB", "STM32G411xB", "STM32G411xC",
        "STM32G414xx",
    ],
    "STM32H5": [
        "STM32H5F5xx", "STM32H5F4xx", "STM32H5E5xx", "STM32H5E4xx", "STM32H573xx", "STM32H563xx",
        "STM32H562xx", "STM32H503xx", "STM32H523xx", "STM32H533xx",
    ],
    "STM32H7": [
        "STM32H743xx", "STM32H753xx", "STM32H750xx", "STM32H742xx", "STM32H745xx", "STM32H745xG",
        "STM32H755xx", "STM32H747xx", "STM32H747xG", "STM32H757xx", "STM32H7B0xx", "STM32H7B0xxQ",
        "STM32H7A3xx", "STM32H7B3xx", "STM32H7A3xxQ", "STM32H7B3xxQ", "STM32H735xx", "STM32H733xx",
        "STM32H730xx", "STM32H730xxQ", "STM32H725xx", "STM32H723xx",
        # STM32H7RS subfamily (separate cmsis_device_h7rs header, same STM32H7* filename prefix)
        "STM32H7R3xx", "STM32H7R7xx", "STM32H7S3xx", "STM32H7S7xx",
    ],
    "STM32L4": [
        "STM32L412xx", "STM32L422xx", "STM32L431xx", "STM32L432xx", "STM32L433xx", "STM32L442xx",
        "STM32L443xx", "STM32L451xx", "STM32L452xx", "STM32L462xx", "STM32L471xx", "STM32L475xx",
        "STM32L476xx", "STM32L485xx", "STM32L486xx", "STM32L496xx", "STM32L4A6xx", "STM32L4P5xx",
        "STM32L4Q5xx", "STM32L4R5xx", "STM32L4R7xx", "STM32L4R9xx", "STM32L4S5xx", "STM32L4S7xx",
        "STM32L4S9xx",
    ],
    "STM32L5": ["STM32L552xx", "STM32L562xx"],
    "STM32U0": ["STM32U073xx", "STM32U083xx", "STM32U031xx"],
    "STM32U5": [
        "STM32U575xx", "STM32U585xx", "STM32U595xx", "STM32U599xx", "STM32U5A5xx", "STM32U5A9xx",
        "STM32U5F9xx", "STM32U5G9xx", "STM32U5F7xx", "STM32U5G7xx", "STM32U535xx", "STM32U545xx",
    ],
}

PIN_NAME_RE = re.compile(r"^P([A-Z])(\d{1,2})")
GPIO_AF_INDEX_RE = re.compile(r"^GPIO_AF(\d+)_")


def strip_namespaces(root: ET.Element) -> ET.Element:
    for el in root.iter():
        if "}" in el.tag:
            el.tag = el.tag.split("}", 1)[1]
    return root


def parse_xml(path: Path) -> ET.Element:
    return strip_namespaces(ET.parse(path).getroot())


def pin_sort_key(pin_id: str):
    m = PIN_NAME_RE.match(pin_id)
    return (m.group(1), int(m.group(2)))


def expand_refname(ref_name: str) -> list[str]:
    """'STM32F401C(B-C)Ux' -> ['STM32F401CBUx', 'STM32F401CCUx'] (one concrete
    RefName per flash/option letter the DB groups together with '(A-B-...)')."""
    m = re.search(r"\(([^)]*)\)", ref_name)
    if not m:
        return [ref_name]
    prefix, suffix = ref_name[: m.start()], ref_name[m.end():]
    return [prefix + option + suffix for option in m.group(1).split("-")]


SUBLINE_RE = re.compile(r"^(STM32[A-Z0-9]{4})")


def fallback_macro(ref_name: str) -> str:
    """Best-effort macro for a RefName that matched nothing in the family's
    verified whitelist: <subline>xx, e.g. 'STM32G411C6Tx' -> 'STM32G411xx'.
    Used only so that no MCU is ever dropped from the output - if this exact
    macro isn't what the real CMSIS header defines for that part, the branch
    simply stays unreachable dead code until corrected, rather than silently
    missing pin data."""
    m = SUBLINE_RE.match(ref_name)
    subline = m.group(1) if m else ref_name[:9]
    return subline + "xx"


def match_macro(ref_name: str, whitelist: list[str]) -> str | None:
    """Match an MCU RefName (e.g. 'STM32G431R(6-8-B)Tx') against a family's
    verbatim macro whitelist. 'x'/'X' in a macro is a wildcard; every other
    character must match literally. Picks the candidate with the most
    literal (non-wildcard) characters matched; refuses on a tie."""
    candidates = []
    for macro in whitelist:
        if len(macro) > len(ref_name):
            continue
        score = 0
        ok = True
        for mc, rc in zip(macro, ref_name):
            if mc in "xX":
                continue
            if mc.upper() != rc.upper():
                ok = False
                break
            score += 1
        if ok:
            candidates.append((score, macro))
    if not candidates:
        return None
    candidates.sort(key=lambda c: c[0], reverse=True)
    top_score = candidates[0][0]
    top = {m for s, m in candidates if s == top_score}
    if len(top) != 1:
        return None
    return top.pop()


def physical_pins(mcu_root: ET.Element) -> set[str]:
    """Every bonded-out general purpose I/O pin, as a Pxnn identifier (e.g. PA00, PG10)."""
    pins = set()
    for pin in mcu_root.findall("Pin"):
        if pin.get("Type") != "I/O":
            continue
        raw_name = pin.get("Name", "").split("-", 1)[0]  # "PG10-NRST" -> "PG10"
        m = PIN_NAME_RE.match(raw_name)
        if not m:
            continue  # dedicated pin (USB_DP, PDR_ON, ...), not a generic Pxx GPIO
        port, num = m.group(1), int(m.group(2))
        pins.add(f"P{port}{num:02d}")
    return pins


def gpio_modes_path(mcu_root: ET.Element) -> Path | None:
    gpio_ip = mcu_root.find("./IP[@Name='GPIO']")
    if gpio_ip is None:
        return None
    version = gpio_ip.get("Version")
    path = MCU_DB_DIR / "IP" / f"GPIO-{version}_Modes.xml"
    return path if path.exists() else None


TIM_INSTANCE_RE = re.compile(r"^TIM(\d+)$")  # matched against the raw (unsuffixed) InstanceName
TIM_ENUM_RE = re.compile(r"^TIM(\d+)_$")  # matched against enum_identifier()-suffixed Peripheral names
CH_MAIN_RE = re.compile(r"^CH(\d+)_$")
CH_COMPLEMENTARY_RE = re.compile(r"^CH(\d+)N_$")
SEMAPHORE_CLAUSE_RE = re.compile(r"(&\s*!?Semaphore_\w+(?:\$\w+)?)|(!?Semaphore_\w+(?:\$\w+)?\s*&)")
SAFE_BOOL_EXPR_RE = re.compile(r"^[\sA-Za-z()]+$")


def eval_ip_condition(expr: str, ip_number: int) -> bool | None:
    """Evaluate a CubeMX RefParameter <Condition Expression="..."/> for one
    concrete $IpNumber (e.g. TIM2 -> 2). Only understands '$IpNumber=N',
    '|', '&', '!' and parens; any 'Semaphore_...' clause (a GPIO-pin-
    availability flag CubeMX resolves internally, not derivable here) is
    stripped out first, since we only want the "default"/no-special-mode
    capability. Returns None if the expression can't be evaluated at all
    (e.g. it was Semaphore-only) - the caller then simply doesn't count it
    as a match rather than guessing."""
    stripped = SEMAPHORE_CLAUSE_RE.sub("", expr).strip()
    if not stripped:
        return None
    py_expr = re.sub(r"\$IpNumber=(\d+)", lambda m: "True" if int(m.group(1)) == ip_number else "False", stripped)
    py_expr = py_expr.replace("|", " or ").replace("&", " and ").replace("!", " not ")
    if not SAFE_BOOL_EXPR_RE.match(py_expr.replace("True", "").replace("False", "").replace("or", "").replace("and", "").replace("not", "")):
        return None
    try:
        return bool(eval(py_expr, {"__builtins__": {}}, {}))
    except Exception:
        return None


def read_max_for_instance(modes_root: ET.Element, param_names: list[str], ip_number: int, default: int) -> int:
    """Largest RefParameter Max="..." among the entries (across any of
    param_names, since ARR is called 'Period' in older IP schemas and
    'PeriodNoDither' in newer ones) whose Condition evaluates True for this
    exact timer instance number - or 'default' if none apply/match."""
    matches = []
    for param_name in param_names:
        for rp in modes_root.findall(f"./RefParameter[@Name='{param_name}']"):
            max_attr = rp.get("Max")
            if max_attr is None:
                continue
            cond = rp.find("Condition")
            if cond is None:
                matches.append(int(max_attr))
                continue
            if eval_ip_condition(cond.get("Expression", ""), ip_number) is True:
                matches.append(int(max_attr))
    return max(matches) if matches else default


def timer_register_widths(
    mcu_root: ET.Element, tim_modes_cache: dict[Path, ET.Element]
) -> dict[str, tuple[int, int, int, str]]:
    """Sanitized TIM instance name -> (maxPrescaler, maxAutoReload, maxRepetitionCounter,
    rawInstanceName), read from the family's shared timer-IP capability file
    (keyed internally by $IpNumber, one file covers every general-purpose/
    advanced timer of the family). rawInstanceName (e.g. 'TIM2', unsuffixed)
    is carried through so the generated code can emit the literal CMSIS/HAL
    names (TIM2, __HAL_RCC_TIM2_CLK_ENABLE) - see render_timer_capabilities_file."""
    widths: dict[str, tuple[int, int, int, str]] = {}
    for ip in mcu_root.findall("IP"):
        instance = ip.get("InstanceName", "")
        m = TIM_INSTANCE_RE.match(instance)
        name, version = ip.get("Name"), ip.get("Version")
        if not m or not name or not version:
            continue
        ip_number = int(m.group(1))
        modes_path = MCU_DB_DIR / "IP" / f"{name}-{version}_Modes.xml"
        if modes_path not in tim_modes_cache:
            if not modes_path.exists():
                continue
            tim_modes_cache[modes_path] = parse_xml(modes_path)
        modes_root = tim_modes_cache[modes_path]
        max_psc = read_max_for_instance(modes_root, ["Prescaler"], ip_number, default=65535)
        max_arr = read_max_for_instance(modes_root, ["PeriodNoDither", "Period"], ip_number, default=65535)
        max_rcr = read_max_for_instance(modes_root, ["RepetitionCounter"], ip_number, default=0)
        widths[enum_identifier(instance)] = (max_psc, max_arr, max_rcr, instance)
    return widths


def sanitize_identifier(token: str) -> str:
    """A handful of signal names aren't valid C++ identifiers as-is (e.g. the
    dual-function debug pins 'JTCK-SWCLK', 'JTMS-SWDIO', 'JTDO-SWO')."""
    token = re.sub(r"[^A-Za-z0-9_]", "_", token)
    if token[:1].isdigit():
        token = f"_{token}"
    return token


def enum_identifier(token: str) -> str:
    """Every generated Peripheral/Signal enumerator gets a trailing '_':
    plain names like 'ADC1', 'TIM2', 'CRS' are *also* preprocessor macros in
    the real CMSIS device header (e.g. '#define ADC1 ((ADC_TypeDef*)ADC1_BASE)'),
    and the preprocessor rewrites them wherever they appear as bare text -
    including right after 'Peripheral::' - no matter how the enum scopes it.
    A trailing underscore keeps the name recognizable while guaranteeing no
    such macro will ever match it token-for-token."""
    return sanitize_identifier(token) + "_"


def split_signal_name(signal_name: str) -> tuple[str, str]:
    """'TIM2_CH1' -> ('TIM2_', 'CH1_'). A handful of signals have no '_' at
    all (e.g. 'EVENTOUT', 'CEC', 'AUDIOCLK') - those become their own
    Peripheral with Signal::None, since they're both the peripheral and the
    function. 'None' is the hand-written sentinel in pin_mapping.hh and is
    deliberately left unsuffixed - it isn't a database-derived token and
    doesn't collide with anything."""
    if "_" in signal_name:
        peripheral, signal = signal_name.split("_", 1)
        return enum_identifier(peripheral), enum_identifier(signal)
    return enum_identifier(signal_name), "None"


def fixed_signals_from_mcu(mcu_root: ET.Element) -> dict[str, set[tuple[str, str]]]:
    """Pxnn identifier -> set of (peripheral, signal) for every fixed-wired
    (non-AF-muxed) signal on that exact pin - ADC/COMP/OPAMP/DAC channels,
    RTC_TAMP/SYS_WKUP/RCC_LSCO/UCPD_FRSTX, ... These come straight from the
    per-MCU XML's <Pin><Signal Name="..."/></Pin> list, not from the GPIO IP
    Modes file (they have no GPIO_AF - the routing isn't mux'd, it's just
    physically wired to that pin). The bare 'GPIO' signal every I/O pin has
    is skipped - it's not a peripheral, it's the base capability marker."""
    table: dict[str, set[tuple[str, str]]] = {}
    for pin in mcu_root.findall("Pin"):
        raw_name = pin.get("Name", "").split("-", 1)[0]
        m = PIN_NAME_RE.match(raw_name)
        if not m:
            continue
        pin_id = f"P{m.group(1)}{int(m.group(2)):02d}"
        for signal in pin.findall("Signal"):
            name = signal.get("Name", "")
            if name == "GPIO" or not name:
                continue
            peripheral, sig = split_signal_name(name)
            table.setdefault(pin_id, set()).add((peripheral, sig))
    return table


def ip_instance_names(mcu_root: ET.Element) -> set[str]:
    """Every peripheral instance this exact chip physically has (ADC1, TIM8,
    I2C3, ...), from the per-MCU XML's top-level <IP InstanceName="..."/>
    list, turned into the same enumerator spelling as Peripheral enum values
    (see enum_identifier) so the two can be cross-referenced directly."""
    return {
        enum_identifier(ip.get("InstanceName"))
        for ip in mcu_root.findall("IP")
        if ip.get("InstanceName")
    }


def af_table_from_modes(modes_root: ET.Element) -> dict[str, set[tuple[int, str, str]]]:
    """Pxnn identifier -> set of (af_index, peripheral, signal), for every
    alternate-function-muxed signal found (no peripheral/signal filtering -
    every peripheral and signal that exists in the database is modelled)."""
    table: dict[str, set[tuple[int, str, str]]] = {}
    for gpio_pin in modes_root.findall("GPIO_Pin"):
        raw_name = gpio_pin.get("Name", "")
        m = PIN_NAME_RE.match(raw_name)
        if not m:
            continue
        pin_id = f"P{m.group(1)}{int(m.group(2)):02d}"

        for pin_signal in gpio_pin.findall("PinSignal"):
            af_param = pin_signal.find("./SpecificParameter[@Name='GPIO_AF']/PossibleValue")
            if af_param is None or af_param.text is None:
                continue  # not an AF-mux'd signal (e.g. plain GPIO/EXTI mode)
            af_match = GPIO_AF_INDEX_RE.match(af_param.text)
            if not af_match:
                continue
            peripheral, signal = split_signal_name(pin_signal.get("Name", ""))
            table.setdefault(pin_id, set()).add((int(af_match.group(1)), peripheral, signal))
    return table


def render_pins_file(pins: list[str]) -> str:
    return "\n".join(f"  {pin}," for pin in pins) + "\n"


def render_mapping_file(pins: list[str], table: dict[str, set[tuple[int, str, str]]]) -> str:
    """One row per (pin, af, peripheral, signal). Deliberately NOT one row per
    AF slot with a single signal - real hardware sometimes muxes more than
    one signal onto the same AF number (e.g. USART2_CTS and USART2_NSS often
    share one AF, since CTS is only used in async mode and NSS only in sync
    mode), so an af-indexed array with exactly one slot per number would
    silently drop one of them."""
    lines = []
    for pin in pins:
        for af, peripheral, signal in sorted(table.get(pin, set())):
            lines.append(f"        PinAlternateFunction{{gpio::Pin::{pin}, {af}, P::{peripheral}, S::{signal}}},")
    return "\n".join(lines) + "\n"


def timer_channel_info(af_table: dict[str, set[tuple[int, str, str]]]) -> dict[str, tuple[int, bool]]:
    """From the already-collected AF table (peripheral/signal pairs actually
    exposed on some pin of this macro): TIMx -> (channelCount, hasComplementaryChannels).
    channelCount is the highest CHn actually wired to a pin (0 if TIMx has no
    OC channels at all, e.g. basic timers TIM6/TIM7)."""
    signals_by_peripheral: dict[str, set[str]] = {}
    for entries in af_table.values():
        for _af, peripheral, signal in entries:
            signals_by_peripheral.setdefault(peripheral, set()).add(signal)

    info: dict[str, tuple[int, bool]] = {}
    for peripheral, signals in signals_by_peripheral.items():
        if not TIM_ENUM_RE.match(peripheral):
            continue
        max_channel = 0
        has_complementary = False
        for signal in signals:
            if (m := CH_COMPLEMENTARY_RE.match(signal)) is not None:
                has_complementary = True
                max_channel = max(max_channel, int(m.group(1)))
            elif (m := CH_MAIN_RE.match(signal)) is not None:
                max_channel = max(max_channel, int(m.group(1)))
        info[peripheral] = (max_channel, has_complementary)
    return info


def render_timer_capabilities_file(
    register_widths: dict[str, tuple[int, int, int, str]], channel_info: dict[str, tuple[int, bool]]
) -> str:
    """'instance'/'enableClock' are generated as the literal CMSIS/HAL names
    (TIM2, __HAL_RCC_TIM2_CLK_ENABLE), not raw addresses or register offsets:
    TIM2_BASE etc. can differ between the secure and non-secure alias on
    TrustZone parts (STM32H5/L5/U5 - confirmed for H5 by comparing
    stm32h5xx.h's TIM1_BASE_S vs TIM1_BASE_NS), so the only value that's
    guaranteed correct for whichever world this translation unit is actually
    built for is whatever the already-included HAL header resolves 'TIM2' to
    - baking in a numeric constant here could silently pick the wrong one."""
    lines = []
    for peripheral in sorted(register_widths):
        max_psc, max_arr, max_rcr, raw_instance = register_widths[peripheral]
        channel_count, has_complementary = channel_info.get(peripheral, (0, False))
        comp = "true" if has_complementary else "false"
        lines.append(
            f"  TimerCapabilities{{P::{peripheral}, {max_psc}u, {max_arr}u, {max_rcr}u, {channel_count}, {comp}, "
            f"{raw_instance}, +[]{{ __HAL_RCC_{raw_instance}_CLK_ENABLE(); }}}},"
        )
    return "\n".join(lines) + "\n"


def render_fixed_signals_file(pins: list[str], table: dict[str, set[tuple[str, str]]]) -> str:
    lines = []
    for pin in pins:
        for peripheral, signal in sorted(table.get(pin, set())):
            lines.append(f"        PinFixedSignal{{gpio::Pin::{pin}, P::{peripheral}, S::{signal}}},")
    return "\n".join(lines) + "\n"


def render_available_peripherals_file(instance_names: set[str], known_peripherals: set[str]) -> str:
    lines = [f"  P::{name}," for name in sorted(instance_names & known_peripherals)]
    return "\n".join(lines) + "\n"


def render_dispatcher(macros: list[str], subdir: str) -> str:
    lines = []
    for i, macro in enumerate(sorted(macros)):
        lines.append(f"#if defined({macro})" if i == 0 else f"#elif defined({macro})")
        lines.append(f'#include "generated/{subdir}/{macro}.inc"')
    lines.append("#else")
    lines.append(f'#error "unknown/unsupported STM32 part - add a branch here"')
    lines.append("#endif")
    return "\n".join(lines) + "\n"


def render_enum_file(names: list[str]) -> str:
    return "\n".join(f"  {name}," for name in names) + "\n"


def main() -> None:
    (GENERATED_DIR / "pins").mkdir(parents=True, exist_ok=True)
    (GENERATED_DIR / "mapping").mkdir(parents=True, exist_ok=True)
    (GENERATED_DIR / "fixed_signals").mkdir(parents=True, exist_ok=True)
    (GENERATED_DIR / "peripherals_present").mkdir(parents=True, exist_ok=True)
    (GENERATED_DIR / "timer_capabilities").mkdir(parents=True, exist_ok=True)

    modes_cache: dict[Path, dict[str, set[tuple[int, str, str]]]] = {}
    tim_modes_cache: dict[Path, ET.Element] = {}
    macro_pins: dict[str, set[str]] = {}
    macro_af: dict[str, dict[str, set[tuple[int, str, str]]]] = {}
    macro_fixed: dict[str, dict[str, set[tuple[str, str]]]] = {}
    macro_ip: dict[str, set[str]] = {}
    macro_tim_regs: dict[str, dict[str, tuple[int, int, int, str]]] = {}

    unmatched = []
    for family_prefix, whitelist in sorted(FAMILY_MACROS.items()):
        xml_files = sorted(MCU_DB_DIR.glob(f"{family_prefix}*.xml"))
        for xml_path in xml_files:
            mcu_root = parse_xml(xml_path)
            ref_name = mcu_root.get("RefName", xml_path.stem)
            candidates = expand_refname(ref_name)
            matched_macros = {
                macro
                for candidate in candidates
                if (macro := match_macro(candidate, whitelist)) is not None
            }
            if not matched_macros:
                # Nothing in the verified whitelist matched - fall back to a
                # best-effort macro rather than dropping this MCU entirely.
                matched_macros = {fallback_macro(c) for c in candidates}
                unmatched.append((family_prefix, xml_path.name, ref_name, sorted(matched_macros)))

            pins = physical_pins(mcu_root)
            fixed = fixed_signals_from_mcu(mcu_root)
            ip_names = ip_instance_names(mcu_root)
            tim_regs = timer_register_widths(mcu_root, tim_modes_cache)
            modes_path = gpio_modes_path(mcu_root)
            family_table = {}
            if modes_path is not None:
                if modes_path not in modes_cache:
                    modes_cache[modes_path] = af_table_from_modes(parse_xml(modes_path))
                family_table = modes_cache[modes_path]

            for macro in matched_macros:
                macro_pins.setdefault(macro, set()).update(pins)
                dest = macro_af.setdefault(macro, {})
                for pin_id, entries in family_table.items():
                    dest.setdefault(pin_id, set()).update(entries)
                fixed_dest = macro_fixed.setdefault(macro, {})
                for pin_id, entries in fixed.items():
                    fixed_dest.setdefault(pin_id, set()).update(entries)
                macro_ip.setdefault(macro, set()).update(ip_names)
                macro_tim_regs.setdefault(macro, {}).update(tim_regs)

    # The per-MCU XML's <Pin><Signal> list (source of macro_fixed) catalogs
    # EVERY signal available on a pin, AF-muxed or not, undifferentiated -
    # e.g. PA0 on STM32C011 lists both 'ADC1_IN0' (truly fixed-wired) and
    # 'USART2_CTS' (AF-muxed, AF1) side by side. Drop anything that's also in
    # the AF table for that pin - fixed_signals should only be signals with
    # NO alternate-function number at all (ADC/COMP/OPAMP/DAC channels,
    # RTC_TAMP, SYS_WKUP, RCC_LSCO, UCPD_FRSTX, ...).
    for macro, fixed_table in macro_fixed.items():
        af_table_for_macro = macro_af.get(macro, {})
        for pin_id in list(fixed_table.keys()):
            af_signals = {(peripheral, signal) for _af, peripheral, signal in af_table_for_macro.get(pin_id, set())}
            fixed_table[pin_id] -= af_signals
            if not fixed_table[pin_id]:
                del fixed_table[pin_id]

    # Discover every peripheral/signal actually used, across every macro and
    # every table (AF-muxed, fixed-wired, and timer instances that have no
    # GPIO pins at all - e.g. basic timers TIM6/TIM7), and emit them as the
    # complete Peripheral/Signal enums (no curated subset).
    peripherals: set[str] = set()
    signals: set[str] = set()
    for table in macro_af.values():
        for entries in table.values():
            for _af, peripheral, signal in entries:
                peripherals.add(peripheral)
                signals.add(signal)
    for table in macro_fixed.values():
        for entries in table.values():
            for peripheral, signal in entries:
                peripherals.add(peripheral)
                signals.add(signal)
    for tim_regs in macro_tim_regs.values():
        peripherals.update(tim_regs.keys())
    signals.discard("None")  # "None" is the enum's fixed first entry, written by hand in pin_mapping.hh

    (GENERATED_DIR / "peripherals_gen.inc").write_text(
        render_enum_file(sorted(peripherals)), encoding="utf-8", newline="\n"
    )
    (GENERATED_DIR / "signals_gen.inc").write_text(
        render_enum_file(sorted(signals)), encoding="utf-8", newline="\n"
    )

    for macro, pins in sorted(macro_pins.items()):
        pin_list = sorted(pins, key=pin_sort_key)
        (GENERATED_DIR / "pins" / f"{macro}.inc").write_text(
            render_pins_file(pin_list), encoding="utf-8", newline="\n"
        )
        (GENERATED_DIR / "mapping" / f"{macro}.inc").write_text(
            render_mapping_file(pin_list, macro_af.get(macro, {})), encoding="utf-8", newline="\n"
        )
        (GENERATED_DIR / "fixed_signals" / f"{macro}.inc").write_text(
            render_fixed_signals_file(pin_list, macro_fixed.get(macro, {})), encoding="utf-8", newline="\n"
        )
        (GENERATED_DIR / "peripherals_present" / f"{macro}.inc").write_text(
            render_available_peripherals_file(macro_ip.get(macro, set()), peripherals), encoding="utf-8", newline="\n"
        )
        (GENERATED_DIR / "timer_capabilities" / f"{macro}.inc").write_text(
            render_timer_capabilities_file(
                macro_tim_regs.get(macro, {}), timer_channel_info(macro_af.get(macro, {}))
            ),
            encoding="utf-8", newline="\n",
        )

    (OUT_DIR / "pins_gen.inc").write_text(
        render_dispatcher(list(macro_pins.keys()), "pins"), encoding="utf-8", newline="\n"
    )
    (OUT_DIR / "pin_mapping_gen.inc").write_text(
        render_dispatcher(list(macro_pins.keys()), "mapping"), encoding="utf-8", newline="\n"
    )
    (OUT_DIR / "pin_fixed_signals_gen.inc").write_text(
        render_dispatcher(list(macro_pins.keys()), "fixed_signals"), encoding="utf-8", newline="\n"
    )
    (OUT_DIR / "available_peripherals_gen.inc").write_text(
        render_dispatcher(list(macro_pins.keys()), "peripherals_present"), encoding="utf-8", newline="\n"
    )
    (OUT_DIR / "timer_capabilities_gen.inc").write_text(
        render_dispatcher(list(macro_pins.keys()), "timer_capabilities"), encoding="utf-8", newline="\n"
    )

    print(f"Generated {len(macro_pins)} macro variants from {sum(len(list(MCU_DB_DIR.glob(f'{p}*.xml'))) for p in FAMILY_MACROS)} MCU XML files")
    print(f"Discovered {len(peripherals)} peripherals and {len(signals)} signals across all of them")
    if unmatched:
        print(f"{len(unmatched)} MCU XML files used a best-effort fallback macro (none of the family's verified macros matched):")
        for family_prefix, filename, ref_name, fallback_macros in unmatched:
            print(f"  [{family_prefix}] {filename} (RefName={ref_name}) -> {', '.join(fallback_macros)}")


if __name__ == "__main__":
    main()
