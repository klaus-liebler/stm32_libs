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


void log_log(int level, char const* file, int line, char const* fmt, ...) {

  if (L.quiet || level < L.level) return;
  va_list ap;
  va_start(ap, fmt);
  if (L.lock) { L.lock(true); }
  fprintf(stdout, "\x1b[0m%6lu\x1b[0m %s%s\x1b[0m \x1b[90m%-8.8s:%03d:\x1b[0m ", (unsigned long)HAL_GetTick(), level_colors[level], level_strings[level], file, line);
  vfprintf(stdout, fmt, ap);
  fprintf(stdout, "\r\n");
  fflush(stdout);
  if (extra_sink) {
    // Eigener vsnprintf()-Durchlauf noetig (nicht denselben 'ap' wie oben wiederverwenden --
    // ein va_list ist nach vfprintf() verbraucht, ein zweiter va_start()/va_copy() ist Pflicht),
    // um dem Sink den reinen, unformatierten Nachrichtentext ohne Zeitstempel-/Level-Praefix zu
    // uebergeben (das Praefix ist fuer eine Netzwerk-Senke wie WebSocket-LogMessage irrelevant,
    // die Zeitstempel/Level bereits selbst im Nachrichtenkopf mitschickt).
    char buf[256];
    va_list ap2;
    va_start(ap2, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap2);
    va_end(ap2);
    if (n > 0) {
      size_t written = (size_t)n >= sizeof(buf) ? sizeof(buf) - 1 : (size_t)n;
      extra_sink(level, buf, written);
    }
  }
  if (L.lock) { L.lock(false); }
  va_end(ap);
}
