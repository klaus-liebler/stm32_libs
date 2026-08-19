#include "hal_header_selector.h"
#include "log.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

//#define LOG_USE_UNICODE
static struct {
  void *udata;
  log_LockFn lock;
  int level;
  bool quiet;
} L;

static log_ExtraSinkFn extra_sink = NULL;

void log_set_extra_sink(log_ExtraSinkFn fn) {
  extra_sink = fn;
}


#if defined(LOG_USE_UNICODE)
static const char *level_strings[] = {
  u8"· T", u8"◇ D", u8"ℹ I ", u8"⚠ W ", u8"● E", u8"✗ F"
};
#else
static const char *level_strings[] = {
  "TRC", "DBG", "INF", "WRN", "ERR", "FTL"
};
#endif

static const char *level_colors[] = {
  "\x1b[94m", "\x1b[36m", "\x1b[32m", "\x1b[33m", "\x1b[31m", "\x1b[35m"
};


void log_set_lock(log_LockFn fn) {
  L.lock = fn;
}


void log_set_level(int level) {
  L.level = level;
}


void log_set_quiet(bool enable) {
  L.quiet = enable;
}


// Breite (sichtbare Zeichen, ohne ANSI-Codes) des Praefixes unten: 6 (Tick) + 1 (Space) +
// 3 (Level) + 1 (Space) + 8 (Datei) + 1 (':') + 3 (Zeile) + 1 (':') + 1 (Space) = 25 -- die
// Einrueckung von log_log_ml() (with_prefix=false) richtet sich optisch danach aus.
#define LOG_PREFIX_WIDTH 25

// Schreibt eine einzelne Zeile: Praefix (Zeitstempel/Level/Datei:Zeile) nur wenn with_prefix,
// sonst eine Einrueckung in Praefixbreite stattdessen (fuer LOG_ML()-Folgezeilen). Gemeinsamer
// Kern von log_log() und den log_log_ml_*()-Funktionen unten; sperrt/entsperrt selbst NICHT (die
// Aufrufer erledigen das, je nach Funktion unterschiedlich, s.u.). va_copy() statt eines zweiten
// va_start(), da 'ap' hier schon eine laufende va_list ist, nicht der Aufrufer selbst Besitzer
// von '...' (anders als im alten log_log(), das direkt in der variadischen Funktion stand).
static void log_write_line(int level, char const* file, int line, bool with_prefix, char const* fmt, va_list ap) {
  if (with_prefix) {
    // Ab 100000 (>99999, also ab 100s Laufzeit) auf Sekunden umschalten (letzte drei Stellen/ms
    // weglassen) -- haelt die Tick-Spalte trotz wachsender Laufzeit in der 6-stelligen Breite
    // lesbar, statt irgendwann ueber die Spaltenbreite hinauszuwachsen.
    unsigned long tick = (unsigned long)HAL_GetTick();
    if (tick > 99999UL) {
      tick /= 1000UL;
    }
    fprintf(stdout, "\x1b[0m%6lu\x1b[0m %s%s\x1b[0m \x1b[90m%-8.8s:%03d:\x1b[0m ", tick, level_colors[level], level_strings[level], file, line);
  } else {
    fprintf(stdout, "%*s", LOG_PREFIX_WIDTH, "");
  }
  va_list ap_body;
  va_copy(ap_body, ap);
  vfprintf(stdout, fmt, ap_body);
  va_end(ap_body);
  fprintf(stdout, "\r\n");
  fflush(stdout);
  if (extra_sink) {
    // Eigener vsnprintf()-Durchlauf noetig (nicht dieselbe va_list wie oben wiederverwenden --
    // nach dem Verbrauch durch vfprintf() ist sie erschoepft), um dem Sink den reinen,
    // unformatierten Nachrichtentext ohne Zeitstempel-/Level-Praefix zu uebergeben (das Praefix
    // ist fuer eine Netzwerk-Senke wie WebSocket-LogMessage irrelevant, die Zeitstempel/Level
    // bereits selbst im Nachrichtenkopf mitschickt).
    char buf[256];
    va_list ap2;
    va_copy(ap2, ap);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap2);
    va_end(ap2);
    if (n > 0) {
      size_t written = (size_t)n >= sizeof(buf) ? sizeof(buf) - 1 : (size_t)n;
      extra_sink(level, buf, written);
    }
  }
}

void log_log(int level, char const* file, int line, char const* fmt, ...) {
  if (L.quiet || level < L.level) return;
  va_list ap;
  va_start(ap, fmt);
  if (L.lock) { L.lock(true); }
  log_write_line(level, file, line, true, fmt, ap);
  if (L.lock) { L.lock(false); }
  va_end(ap);
}

// --- Mehrzeilige Bloecke (LOG_INFO_ML/LOG_ML/LOG_ML_END, s. log.h) -----------------------------
// ml_active/ml_level sind nur waehrend eines offenen Blocks gueltig -- ein Aufruf von log_log_ml()
// / log_log_ml_end() OHNE vorausgehenden erfolgreichen log_log_ml_begin() ist ein No-op (kein
// Absturz). Dieses einfache Design geht davon aus, dass ML-Bloecke nicht von zwei UNABHAENGIGEN
// Aufrufstellen gleichzeitig auf unterschiedlichen Log-Leveln offen gehalten werden (fuer die
// beiden aktuellen Aufrufstellen dieses Projekts -- App::greeting(), scan_i2c_bus(), beide
// sequenziell und auf demselben Level -- zutreffend; bei weiteren, echt nebenlaeufigen
// ML-Nutzern waere ein Thread-Besitzer-Feld statt eines einfachen bool noetig).
static bool ml_active = false;
static int ml_level = LOG_INFO;

void log_log_ml_begin(int level, char const* file, int line, char const* fmt, ...) {
  if (L.quiet || level < L.level) return;
  if (L.lock) { L.lock(true); }
  if (ml_active) {
    // Nur erreichbar, wenn DERSELBE Thread ein vorheriges LOG_ML_END() vergessen hat (ThreadX'
    // TX_MUTEX erlaubt rekursives get() durch den Besitzer-Thread) -- ein ANDERER Thread haette
    // hier stattdessen bis zum echten log_log_ml_end() blockiert. Alten Block zwangsweise
    // schliessen (abschliessende Leerzeile, einmal wieder entsperren -- das rekursiv genommene
    // Lock bleibt fuer den neuen Block unten bestehen), dann normal fortfahren.
    fprintf(stdout, "\r\n");
    fflush(stdout);
    if (L.lock) { L.lock(false); }
  }
  ml_active = true;
  ml_level = level;
  va_list ap;
  va_start(ap, fmt);
  log_write_line(level, file, line, true, fmt, ap);
  va_end(ap);
}

void log_log_ml(char const* fmt, ...) {
  if (!ml_active) return;
  va_list ap;
  va_start(ap, fmt);
  log_write_line(ml_level, NULL, 0, false, fmt, ap);
  va_end(ap);
}

void log_log_ml_end(void) {
  if (!ml_active) return;
  fprintf(stdout, "\r\n");
  fflush(stdout);
  ml_active = false;
  if (L.lock) { L.lock(false); }
}
