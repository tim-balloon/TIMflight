/**
 * @file main.c
 * 
 * Copyright 20nn ********
 */ 


#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <arpa/inet.h> // socket stuff
#include <netinet/in.h> // socket stuff
#include <sys/types.h> // socket types
#include <sys/socket.h> // socket stuff
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <pthread.h> // threads
#include <float.h>
#include <ctype.h>

#include "groundhog.h"
#include "linklist_connect.h"

extern struct TlmReport pilot_report;
extern struct TlmReport bi0_report;
extern struct TlmReport highrate_report;
extern struct TlmReport sbd_report;
extern struct TlmReport evtm_los_report;
extern struct TlmReport evtm_tdrss_report;

void groundhog_write_calspecs(char *fname) {
  channels_write_calspecs(fname, derived_list);
}

int main(int argc, char * argv[]) {
  // set the directory in which to save raw linklist files received from the payload
  snprintf(archive_dir, LINKLIST_MAX_FILENAME_SIZE, "%s", DEFAULT_ARCHIVE_DIR);

  // initialize the main telemetry superframe
  channels_initialize(channel_list);

  // initialize the linklists derived from the superframe
  linklist_t ** ll_list = calloc(MAX_NUM_LINKLIST_FILES, sizeof(linklist_t *));
  load_all_linklists(superframe, DEFAULT_LINKLIST_DIR, ll_list, LL_INCLUDE_ALLFRAME);
  generate_housekeeping_linklist(linklist_find_by_name(ALL_TELEMETRY_NAME, ll_list), ALL_TELEMETRY_NAME);
  linklist_generate_lookup(ll_list);
  write_linklist_format(linklist_find_by_name(ALL_TELEMETRY_NAME, ll_list), \
                        DEFAULT_LINKLIST_DIR ALL_TELEMETRY_NAME ".auto");

  channels_write_calspecs("test.cs", derived_list);

  // TODO(shubh): remove deprecated telemetry links after testing
  int pilot_on = 1;
  int bi0_on = 1;
  int highrate_on = 1;
  int evtm_los_on = 1;
  int evtm_tdrss_on = 1;
  int daemon = 0;

  for (int i = 1; i < argc; i++) {
    // TODO(shubh): remove deprecated telemetry links after testing
    if (strcmp(argv[i], "-no_pilot") == 0) pilot_on = 0;
    else if (strcmp(argv[i], "-no_bi0") == 0) bi0_on = 0;
    else if (strcmp(argv[i], "-no_highrate") == 0) highrate_on = 0;
    else if (strcmp(argv[i], "-no_evtm_los") == 0) evtm_los_on = 0;
    else if (strcmp(argv[i], "-no_evtm_tdrss") == 0) evtm_tdrss_on = 0;
    else if (strcmp(argv[i], "-no_evtm") == 0) evtm_los_on = evtm_tdrss_on = 0;
    else if (strcmp(argv[i], "-pilot_only") == 0) bi0_on = highrate_on = evtm_tdrss_on = evtm_los_on = 0;
    else if (strcmp(argv[i], "-bi0_only") == 0) highrate_on = pilot_on = evtm_tdrss_on = evtm_los_on = 0;
    else if (strcmp(argv[i], "-highrate_only") == 0) pilot_on = bi0_on = evtm_tdrss_on = evtm_los_on = 0;
    else if (strcmp(argv[i], "-evtm_los_only") == 0) pilot_on = bi0_on = highrate_on = evtm_tdrss_on = 0;
    else if (strcmp(argv[i], "-evtm_tdrss_only") == 0) pilot_on = bi0_on = highrate_on = evtm_los_on = 0;
    else if (strcmp(argv[i], "-evtm_only") == 0) pilot_on = bi0_on = highrate_on = 0;
    else if (strcmp(argv[i], "-d") == 0) daemon = 1;
    else if (strcmp(argv[i], "-quiet") == 0) verbose = 0;
    else if (strcmp(argv[i], "-verbose") == 0) verbose = 1;
    else
      blast_fatal("Unrecognized option \"%s\"", argv[i]);
  }

  if (daemon) {
    daemonize();
  }

  // setup pilot receive udp struct
  struct UDPSetup pilot_setup = {"Pilot",
                                 PILOT_ADDR,
                                 PILOT_PORT,
                                 PILOT_MAX_SIZE,
                                 PILOT_MAX_PACKET_SIZE,
                                 PILOT};

  struct UDPSetup pilot_setup2 = {"Pilot 2",
                                 PILOT_ADDR,
                                 PILOT_PORT+1,
                                 PILOT_MAX_SIZE,
                                 PILOT_MAX_PACKET_SIZE,
                                 PILOT};

  struct UDPSetup evtm_los_setup = {"EVTM_LOS",
                                    EVTM_ADDR_LOS,
                                    EVTM_PORT_LOS,
                                    EVTM_MAX_SIZE,
                                    EVTM_MAX_PACKET_SIZE,
                                    LOS_EVTM};

  struct UDPSetup evtm_tdrss_setup = {"EVTM_TDRSS",
                                      EVTM_ADDR_TDRSS,
                                      EVTM_PORT_TDRSS,
                                      EVTM_MAX_SIZE,
                                      EVTM_MAX_PACKET_SIZE,
                                      TDRSS_EVTM};

  // Receiving data from telemetry
  pthread_t pilot_receive_worker[2];
  pthread_t biphase_receive_worker;
  pthread_t highrate_receive_worker;
  pthread_t direct_receive_worker;
  pthread_t evtm_los_receive_worker;
  pthread_t evtm_tdrss_receive_worker;
  // TODO(shubh): remove deprecated telemetry links after testing

  // Serving up data received via telemetry
  pthread_t server_thread;

  if (pilot_on) {
    pthread_create(&pilot_receive_worker[0], NULL, (void *) &udp_receive, (void *) &pilot_setup);
    pthread_create(&pilot_receive_worker[1], NULL, (void *) &udp_receive, (void *) &pilot_setup2);
  }

  // TODO(shubh): remove deprecated telemetry links after testing
  if (bi0_on) {
    pthread_create(&biphase_receive_worker, NULL, (void *) &biphase_receive, NULL);
  }

  // TODO(shubh): remove deprecated telemetry links after testing
  if (highrate_on) {
    pthread_create(&highrate_receive_worker, NULL, (void *) &highrate_receive, (void *) 0);
    pthread_create(&direct_receive_worker, NULL, (void *) &highrate_receive, (void *) 1);
  }

  if (evtm_los_on) {
    pthread_create(&evtm_los_receive_worker, NULL, (void *) &EVTM_udp_receive, (void *) &evtm_los_setup);
  }

  if (evtm_tdrss_on) {
    pthread_create(&evtm_tdrss_receive_worker, NULL, (void *) &EVTM_udp_receive, (void *) &evtm_tdrss_setup);
  }

  // start the server thread for mole clients
  pthread_create(&server_thread, NULL, (void *) &linklist_server, NULL);

  sleep(1);
  printf("\n\n\n");

  char fn_str[1024] = "";
  int len = 0, prev_len = 0;

  // print out the reports
  while (true) {
    snprintf(fn_str, sizeof(fn_str), BLU "    Pilot: %s %s [%" PRIu64 "];" GRN "   BI0: %s %s [%" PRIu64 "];" \
                                     YLW "  Highrate: %s %s [%" PRIu64 "];" RED "  SBD: %s %s [%" PRIu64 "];" \
                                     BLU " EVTM_LOS: %s %s [%" PRIu64 "];" GRN " EVTM_TDRSS: %s %s [%" PRIu64 "];" NOR,
            (pilot_report.ll) ? pilot_report.ll->name : "(NULL)",
            (pilot_report.allframe) ? "AF" : "  ",
            pilot_report.framenum,

            (bi0_report.ll) ? bi0_report.ll->name : "(NULL)",
            (bi0_report.allframe) ? "AF" : "  ",
            bi0_report.framenum,

            (highrate_report.ll) ? highrate_report.ll->name : "(NULL)",
            (highrate_report.allframe) ? "AF" : "  ",
            highrate_report.framenum,

            (sbd_report.ll) ? sbd_report.ll->name : "(NULL)",
            (sbd_report.allframe) ? "AF" : "  ",
            sbd_report.framenum,

            (evtm_los_report.ll) ? evtm_los_report.ll->name : "(NULL)",
            (evtm_los_report.allframe) ? "AF" : "  ",
            evtm_los_report.framenum,

            (evtm_tdrss_report.ll) ? evtm_tdrss_report.ll->name : "(NULL)",
            (evtm_tdrss_report.allframe) ? "AF" : "  ",
            evtm_tdrss_report.framenum);

    // print enough characters to overwrite the previous line
    len = strlen(fn_str);
    for (int i = len; i < prev_len; i++) fn_str[i] = ' ';
    fn_str[(len > prev_len) ? len : prev_len] = '\0'; // terminate
    prev_len = len;

    fprintf(stdout, "%s\r", fn_str);
    fflush(stdout);

    usleep(200000);
  }
  return 0;
}
