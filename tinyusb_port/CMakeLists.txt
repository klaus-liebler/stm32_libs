cmake_minimum_required(VERSION 3.22)

# Duenne CMake-Huelle um das TinyUSB-Submodule (stm32_libs/tinyusb, verschachteltes Submodule
# auf hathach/tinyusb) -- TinyUSBs eigenes Build-System ist auf deren Beispiel-Projektlayout
# (hw/bsp/<board>) zugeschnitten und passt nicht zu diesem CMake-Projekt, daher hier nur die fuer
# STM32H5/USB_DRD_FS + Device-CDC/MSC/NCM tatsaechlich benoetigten Quellen von Hand aufgefuehrt
# (analog zum expliziten Dateilisten-Stil von usb_pd/tmc2209 statt eines GLOBs).
set(TINYUSB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../tinyusb)

add_library(tinyusb_port STATIC
    ${TINYUSB_DIR}/src/tusb.c
    ${TINYUSB_DIR}/src/common/tusb_fifo.c
    ${TINYUSB_DIR}/src/device/usbd.c
    ${TINYUSB_DIR}/src/class/cdc/cdc_device.c
    ${TINYUSB_DIR}/src/class/msc/msc_device.c
    ${TINYUSB_DIR}/src/class/net/ncm_device.c
    ${TINYUSB_DIR}/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
    ${TINYUSB_DIR}/src/portable/st/stm32_fsdev/fsdev_common.c
)

target_include_directories(tinyusb_port PUBLIC
    ${TINYUSB_DIR}/src
)

# tusb_config.h ist projektspezifisch (liegt im konsumierenden Firmware-Projekt, z.B.
# Core/Inc/tusb_config.h) -- TinyUSBs eigene Quellen (dieses Target) brauchen ihn beim
# Kompilieren ebenso wie die Consumer-Seite. Das konsumierende Projekt-CMakeLists.txt muss
# deshalb nach add_subdirectory(...tinyusb_port...) per
#   target_include_directories(tinyusb_port PRIVATE <Pfad-zu-tusb_config.h>)
# den passenden Include-Pfad ergaenzen.

# Link gegen stm32cubemx (HAL/CMSIS, liefert stm32h5xx.h fuer fsdev_stm32.h) --
# gleiches Muster wie common_stm32/tmc2209/usb_pd.
target_link_libraries(tinyusb_port PUBLIC stm32cubemx)

target_compile_definitions(tinyusb_port PUBLIC
    CFG_TUSB_MCU=OPT_MCU_STM32H5
    CFG_TUSB_OS=OPT_OS_THREADX
)
