#pragma once

#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>


//to avoid intellisense errors, see https://github.com/microsoft/vscode-cpptools/issues/11164
#if __INTELLISENSE__
#define __FILE_NAME__  __FILE__
#endif

#ifdef __cplusplus
 extern "C" {
#endif


typedef void (*log_LockFn)(bool lock);
// Zusaetzliche, optionale Log-Senke neben der eingebauten stdout/UART-Ausgabe -- bekommt bei
// jedem log_log()-Aufruf (der den Level-/quiet-Filter passiert) den bereits fertig formatierten
// Nachrichtentext (ohne Zeitstempel/Level/Datei:Zeile-Praefix, ohne Zeilenumbruch) uebergeben.
// Bewusst generisch gehalten (kein Wissen ueber Netzwerk/WebSocket/etc. in dieser Bibliothek) --
// ein aufrufendes Projekt kann hierueber z.B. Logzeilen an verbundene Netzwerk-Clients spiegeln.
// Wird SYNCHRON und mit gehaltenem log_lock() aufgerufen: muss daher schnell/nicht-blockierend
// sein und darf selbst keine log_*()-Funktion aufrufen (Deadlock-Gefahr).
typedef void (*log_ExtraSinkFn)(int level, char const* text, size_t len);

enum { LOG_TRACE, LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_FATAL };

#define log_trace(...) log_log(LOG_TRACE, __FILE_NAME__, __LINE__, __VA_ARGS__)
#define log_debug(...) log_log(LOG_DEBUG, __FILE_NAME__, __LINE__, __VA_ARGS__)
#define log_info(...)  log_log(LOG_INFO,  __FILE_NAME__, __LINE__, __VA_ARGS__)
#define log_warn(...)  log_log(LOG_WARN,  __FILE_NAME__, __LINE__, __VA_ARGS__)
#define log_error(...) log_log(LOG_ERROR, __FILE_NAME__, __LINE__, __VA_ARGS__)
#define log_fatal(...) log_log(LOG_FATAL, __FILE_NAME__, __LINE__, __VA_ARGS__)

// Mehrzeiliger Block (ML = multi-line): fuer Ausgaben, die logisch zusammengehoeren (Boot-Banner,
// I2C-Scan-Ergebnisse) und nicht durch die Log-Zeile eines anderen Threads durchmischt werden
// duerfen. LOG_INFO_ML(...) druckt die erste Zeile MIT dem ueblichen Praefix (Zeitstempel/Level/
// Datei:Zeile) und haelt danach das Log-Lock offen; LOG_ML(...) druckt beliebig viele
// Folgezeilen OHNE Praefix, nur eingerueckt; LOG_ML_END() (ohne Parameter) druckt eine
// abschliessende Leerzeile und gibt das Lock wieder frei. LOG_ML()/LOG_ML_END() ohne
// vorausgehendes LOG_INFO_ML() sind No-ops, kein Absturz. Nicht verschachteln (ein zweites
// LOG_INFO_ML() vom selben Thread ohne vorheriges LOG_ML_END() schliesst den alten Block
// automatisch, s. log.c) -- fuer Details/Einschraenkungen s. log_log_ml_begin() in log.c.
#define LOG_INFO_ML(...) log_log_ml_begin(LOG_INFO, __FILE_NAME__, __LINE__, __VA_ARGS__)
#define LOG_ML(...) log_log_ml(__VA_ARGS__)
#define LOG_ML_END() log_log_ml_end()

void log_log(int level, char const* file, int line, char const* fmt, ...);
void log_log_ml_begin(int level, char const* file, int line, char const* fmt, ...);
void log_log_ml(char const* fmt, ...);
void log_log_ml_end(void);
void log_set_lock(log_LockFn);
void log_set_level(int level);
void log_set_quiet(bool enable);
void log_set_extra_sink(log_ExtraSinkFn fn);

#ifdef __cplusplus
}
#endif

