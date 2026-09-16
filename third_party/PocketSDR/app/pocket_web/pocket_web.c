//
//  Pocket SDR C AP -  GNSS Receiver Server with Web UI.
//
//  The receiver is not started by this AP. Its configuration and its whole
//  lifecycle are controlled from the Web UI in a browser. The settings of the
//  last session are restored at start and saved at exit.
//
//  Author:
//  T.TAKASU
//
//  History:
//  2026-08-07  1.0  separated from pocket_trk.c
//
#include <signal.h>
#include "pocket_sdr.h"

// constants and macros ---------------------------------------------------------
#define PROG_NAME  "pocket_web" // program name
#define TRACE_LEVEL 3           // debug trace level
#define INI_FILE   "pocket_web.ini" // settings file
#define DEF_ADDR   "127.0.0.1"  // address to show for the default bind address
#define LOOP_CYC   100          // main loop cycle (ms)
#define EXIT_WAIT  400          // max wait cycles (10 ms) for device close

// usage text ------------------------------------------------------------------
static const char *usage_text[] = {
    "Usage: pocket_web [-web [addr:]port] [-html dir] [-ini file] [-start]",
    "       [-debug file]",
    NULL
};

// interrupt and shutdown completion flags -------------------------------------
static volatile uint8_t intr = 0;
static volatile uint8_t done = 0;

// signal handler --------------------------------------------------------------
static void sig_func(int sig)
{
    intr = 1;
    signal(sig, sig_func);
}

#ifdef WIN32

// console control handler -----------------------------------------------------
//   The C runtime raises SIGINT only for Ctrl-C and Ctrl-Break. Console close,
//   logoff and shutdown terminate the process as soon as the handler returns,
//   so wait here until the main thread has closed the SDR device.
static BOOL WINAPI ctrl_func(DWORD type)
{
    intr = 1;
    for (int i = 0; i < EXIT_WAIT && !done; i++) {
        sdr_sleep_msec(10);
    }
    return TRUE;
}
#endif // WIN32

// print version ---------------------------------------------------------------
static void print_ver(void)
{
    printf("%s ver.%s\n", PROG_NAME, sdr_get_ver());
    exit(0);
}

// show usage ------------------------------------------------------------------
static void show_usage(void)
{
    for (int i = 0; usage_text[i]; i++) {
        printf("%s\n", usage_text[i]);
    }
    exit(0);
}

// main ------------------------------------------------------------------------
// Command spec: see doc/command_ref.md.
int main(int argc, char **argv)
{
    sdr_web_cfg_t cfg;
    sdr_web_t *web;
    sdr_rcv_t *rcv;
    char addr[64] = "";
    const char *html_dir = "", *ini_file = INI_FILE, *debug_file = "";
    int port = 0, start = 0;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-web") && i + 1 < argc) {
            const char *str = argv[++i], *p = strrchr(str, ':');
            if (p) {
                snprintf(addr, sizeof(addr), "%.*s", (int)(p - str), str);
                port = atoi(p + 1);
            } else {
                port = atoi(str);
            }
        } else if (!strcmp(argv[i], "-html") && i + 1 < argc) {
            html_dir = argv[++i];
        } else if (!strcmp(argv[i], "-ini") && i + 1 < argc) {
            ini_file = argv[++i];
        } else if (!strcmp(argv[i], "-start")) {
            start = 1;
        } else if (!strcmp(argv[i], "-debug") && i + 1 < argc) {
            debug_file = argv[++i];
        } else if (!strcmp(argv[i], "-v")) {
            print_ver();
        } else {
            show_usage();
        }
    }
    if (port <= 0) {
        show_usage();
    }
    if (*debug_file) {
        traceopen(debug_file);
        tracelevel(TRACE_LEVEL);
    }
    sdr_func_init(""); // also initializes the socket library

    signal(SIGTERM, sig_func);
    signal(SIGINT, sig_func);
#ifdef WIN32
    SetConsoleCtrlHandler(ctrl_func, TRUE); // console close, logoff, shutdown
#else
    signal(SIGPIPE, SIG_IGN);
#endif
    if (!(web = sdr_web_start(NULL, addr, port, html_dir))) {
        fprintf(stderr, "web server start error port=%d\n", port);
        return -1;
    }
    sdr_web_init_cfg(&cfg);
    sdr_web_set_cfg(web, &cfg, ini_file);
    sdr_web_load_cfg(web); // restore the settings of the last session

    if (start && !sdr_web_start_rcv(web)) { // can be retried from the Web UI
        fprintf(stderr, "receiver start error\n");
    }
    printf("Web UI: http://%s:%d/\n", *addr ? addr : DEF_ADDR, port);

    while (!intr) { // the receiver lifecycle belongs to the Web UI
        sdr_sleep_msec(LOOP_CYC);
    }
    if ((rcv = sdr_web_stop(web))) {
        sdr_rcv_close(rcv);
    }
    if (*debug_file) {
        traceclose();
    }
    done = 1; // release the console control handler
    return 0;
}
