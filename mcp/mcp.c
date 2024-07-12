/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2006 University of Toronto
 *
 * This file is part of mcp.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/io.h>
#include <sys/statvfs.h>
#include <stdarg.h>
#include <errno.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <sys/syscall.h>
#include <openssl/md5.h>

#include "phenom/job.h"
#include "phenom/log.h"
#include "phenom/sysutil.h"

#include "chrgctrl.h"
#include "mputs.h"
#include "command_list.h"
#include "command_struct.h"
#include "crc.h"
#include "magnetometer.h"
#include "inclinometer.h"
#include "mcp.h"
#include "pointing_struct.h"
#include "channels_tng.h"
#include "tx.h"
#include "lut.h"
#include "labjack.h"
#include "labjack_functions.h"
#include "multiplexed_labjack.h"

#include "acs.h"
#include "actuators.h"
#include "balance.h"
#include "blast.h"
#include "blast_time.h"
#include "computer_sensors.h"
#include "diskmanager_tng.h"
#include "dsp1760.h"
#include "ec_motors.h"
#include "evtm.h"
#include "framing.h"
#include "gps.h"
#include "csbf_dgps.h"
#include "linklist.h"
#include "linklist_compress.h"
#include "pilot.h"
#include "highrate.h"
#include "bitserver.h"
#include "bi0.h"
#include "biphase_hardware.h"
#include "data_sharing_server.h"
#include "FIFO.h"
#include "logger.h"
#include "motors.h"
#include "store_data.h"
#include "watchdog.h"
#include "xsc_network.h"
#include "xsc_pointing.h"
#include "sip.h"
#include "scheduler_tng.h"
#include "inner_frame_power.h"
#include "outer_frame_power.h"
#include "motor_box_power.h"
#include "socket_utils.h"
#include "gondola_thermometry.h"
#include "star_camera_transmit.h"
#include "star_camera_solutions.h"
#include "star_camera_receive.h"
#include "star_camera_trigger.h"

/* Define global variables */
char* flc_ip[2] = {"192.168.1.3", "192.168.1.4"};

int16_t SouthIAm;
int16_t InCharge = 1;
int16_t InChargeSet = 0;

extern labjack_state_t state[NUM_LABJACKS];

bool shutdown_mcp = false;
bool ready_to_close = false;

void Pointing();
void WatchFIFO(void*);          // commands.c
#ifdef USE_XY_THREAD
void StageBus(void);
#endif

logger_t logger = {0};
uint8_t * logger_buffer = NULL;
struct tm start_time;
int ResetLog = 0;

linklist_t * linklist_array[MAX_NUM_LINKLIST_FILES] = {NULL};
linklist_t * telemetries_linklist[NUM_TELEMETRIES] = {NULL, NULL, NULL, NULL, NULL, NULL};
uint8_t * master_superframe_buffer = NULL;
// struct Fifo * telem_fifo[NUM_TELEMETRIES] = {&pilot_fifo, &bi0_fifo, &highrate_fifo, &sbd_fifo};
struct Fifo * telem_fifo[NUM_TELEMETRIES] = \
                {&pilot_fifo, &bi0_fifo, &highrate_fifo, &sbd_fifo, &evtm_fifo_los, &evtm_fifo_tdrss};
extern linklist_t * ll_hk;

#define MPRINT_BUFFER_SIZE 1024
#define MAX_MPRINT_STRING \
( \
  MPRINT_BUFFER_SIZE /* buffer length */ \
  - 6                /* 2*(marker+space) + EOL + NUL */ \
  - 24               /* date "YYYY-MM-DD HH:MM:SS.mmm " */ \
  - 8                /* thread name "ThName: " */ \
)

#if (TEMPORAL_OFFSET != 0)
#warning TEMPORAL_OFFSET NON-ZERO; FIX FOR FLIGHT
#endif

/* gives system time (in s) */
time_t mcp_systime(time_t *t)
{
    time_t the_time = time(NULL) + TEMPORAL_OFFSET;
    if (t) {
        *t = the_time;
    }
    return the_time;
}

void force_incharge(void) {
    InCharge = 1;
    // used for the test cryostat without watchdog board
}

void close_mcp(int m_code)
{
    fprintf(stderr, "Closing MCP with signal %d\n", m_code);
    shutdown_mcp = true;
    while (!ready_to_close) usleep(10000);
    watchdog_close();
    diskmanager_shutdown();
    closeLogger(&logger);
    ph_sched_stop();
}

/* Polarity crisis: am I north or south? */
/* Right now fc2 == south */
static int AmISouth(int *not_cryo_corner)
{
    char buffer[4];
    *not_cryo_corner = 1;

    if (gethostname(buffer, 3) == -1 && errno != ENAMETOOLONG) {
      berror(err, "System: Unable to get hostname");
    } else if (buffer[0] == 'p') {
      *not_cryo_corner = 0;
      blast_info("System: Cryo Corner Mode Activated\n");
    }

    return ((buffer[0] == 'f') && (buffer[1] == 'c') && (buffer[2] == '2')) ? 1 : 0;
}

void * lj_connection_handler(void *arg) {
    while (!InCharge) {
        sleep(1);
    }
    // LABJACKS
    blast_info("I am now in charge, initializing LJs");
    // initialize the OF and IF pbob Labjacks
    // ordering is OF PBOB, IF PBOB, UNASSIGNED, UNASSIGNED, UNASSIGNED, CommandQueue
    init_labjacks(1, 1, 0, 0, 0, 1);
    // Set the queue to allow new set
    // leaving the queue in here because it will be used in the future.
    CommandData.Labjack_Queue.set_q = 1;
    CommandData.Labjack_Queue.lj_q_on = 0;
    for (int h = 0; h < NUM_LABJACKS; h++) {
        CommandData.Labjack_Queue.which_q[h] = 0;
    }
    // commented out but left in as a model for how to end this thread
    // to prevent seg faults.
    // ph_thread_t *cmd_thread = mult_initialize_labjack_commands(6);
    // ph_thread_join(cmd_thread, NULL);
    return NULL;
}

unsigned int superframe_counter[RATE_END] = {0};



static void mcp_200hz_routines(void)
{
    store_200hz_acs();
    command_motors();
    write_motor_channels_200hz();
    SetGyroMask();
    share_data(RATE_200HZ);
    framing_publish_200hz();
    add_frame_to_superframe(channel_data[RATE_200HZ], RATE_200HZ, master_superframe_buffer,
                            &superframe_counter[RATE_200HZ]);
}

static void mcp_122hz_routines(void) {
    // dummy right now for the loops
    static int dummy = 0;
}

static void mcp_100hz_routines(void)
{
    int i_point = GETREADINDEX(point_index);
    read_100hz_acs();
    PointingData[i_point].recv_shared_data = recv_fast_data();
    Pointing();
    DoSched();
    update_axes_mode();
    store_100hz_acs();
    send_fast_data();
    store_100hz_xsc(0);
    store_100hz_xsc(1);
    write_motor_channels_100hz();
    xsc_control_triggers();
    xsc_decrement_is_new_countdowns(&CommandData.XSC[0].net);
    xsc_decrement_is_new_countdowns(&CommandData.XSC[1].net);
    // write the logs to the frame
    if (logger_buffer) {
        if (ResetLog) {
            resetLogger(&logger);
            ResetLog = 0;
        }
        readLogger(&logger, logger_buffer);
    }

    share_data(RATE_100HZ);
    framing_publish_100hz();
    add_frame_to_superframe(channel_data[RATE_100HZ], RATE_100HZ, master_superframe_buffer,
                            &superframe_counter[RATE_100HZ]);
}
static void mcp_5hz_routines(void)
{
    watchdog_ping();
    // Tickles software WD 2.5x as fast as timeout
    read_5hz_acs();
    store_5hz_acs();
    store_5hz_xsc(0);
    store_5hz_xsc(1);
    write_motor_channels_5hz();
    store_axes_mode_data();
    WriteAux();
    ControlBalance();
    StoreActBus();
    #ifdef USE_XY_THREAD
    StoreStageBus(0);
    #endif
//    PhaseControl();
//    ChargeController();
//    VideoTx();
//    cameraFields();

    share_data(RATE_5HZ);
    framing_publish_5hz();
    add_frame_to_superframe(channel_data[RATE_5HZ], RATE_5HZ, master_superframe_buffer,
                            &superframe_counter[RATE_5HZ]);
}
static void mcp_2hz_routines(void)
{
    if (InCharge) {
      xsc_write_data(0);
      xsc_write_data(1);
    }
}

static void mcp_1hz_routines(void)
{
    force_incharge();
    int ready = !superframe_counter[RATE_488HZ];
    // int ready = 1;
    // int i = 0;
    // for (i = 0; i < RATE_END; i++) ready = ready && !superframe_counter[i];
    if (ready && InCharge) {
        for (int i = 0; i < NUM_TELEMETRIES; i++) {
           memcpy(getFifoWrite(telem_fifo[i]), master_superframe_buffer, superframe->size);
           incrementFifo(telem_fifo[i]);
        }
    }
    // gondola thermometry
    read_thermistors();
    // 4 below log the data from the pbobs and command the relays
    log_of_pbob_analog();
    log_if_pbob_analog();
    of_pbob_commanding();
    if_pbob_commanding();
    motor_pbob_commanding();
    share_superframe(master_superframe_buffer);
    // commented out but will use when we have LJ subsystems again for power
    labjack_choose_execute();
    // printf("InCharge is %d\n", InCharge);
    store_1hz_acs();
    record_motor_status_1hz();
    // blast_store_disk_space();
    xsc_control_heaters();
    store_1hz_xsc(0);
    store_1hz_xsc(1);
    store_charge_controller_data();
    share_data(RATE_1HZ);
    framing_publish_1hz();
    store_data_hk(master_superframe_buffer);
    add_frame_to_superframe(channel_data[RATE_1HZ], RATE_1HZ, master_superframe_buffer,
                            &superframe_counter[RATE_1HZ]);
}

static void *mcp_main_loop(void *m_arg)
{
#define MCP_FREQ 24400
#define MCP_NS_PERIOD (NSEC_PER_SEC / MCP_FREQ)
#define HZ_COUNTER(_freq) (MCP_FREQ / (_freq))


    // Start values indicate the phase (in MCP_FREQ counts) of each channel relative to zero.
    //
    // For 24400 mcp counts per second, there are 50, 100, 122, 244, 4880, 12200, and 24400
    // mcp counts per 488 Hz, 244 Hz, 200 Hz, 100 Hz, 5 Hz, and 1 Hz routine, respectively.
    //
    // Start values are chosen so that all the routines are spaced over the 50 mcp pulses per
    // 488 Hz routine, which is the fastest rate.
    int counter_200hz = 33; // 11;
    int counter_122hz = 28; // TODO(ianlowe13): maybe needs to be changed
    int counter_100hz = 27; // 17;
    int counter_5hz = 20; // 23;
    int counter_2hz = 19; // 30;
    int counter_1hz = 1; // 31;

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    nameThread("Main");

    while (true) {
        int ret;
        const struct timespec interval_ts = { .tv_sec = 0,
                                        .tv_nsec = MCP_NS_PERIOD}; /// 200HZ interval
        /// Set our wakeup time
        ts = timespec_add(ts, interval_ts);
        ret = clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);
        if (ret == EINTR) {
            blast_info("Exiting MCP!");
            break;
        }

        if (ret) {
            blast_err("error while sleeping, code %d (%s)\n", ret, strerror(ret));
            break;
        }

        if (!--counter_1hz) {
            counter_1hz = HZ_COUNTER(1);
            mcp_1hz_routines();

            // only break out of main loop after all data has been written to mqtt
            if (shutdown_mcp) {
                ready_to_close = true;
                blast_info("Main loop is ready for shutdown\n");
                break;
            }
        }
        if (!--counter_2hz) {
            counter_2hz = HZ_COUNTER(2);
            mcp_2hz_routines();
        }
        if (!--counter_5hz) {
            counter_5hz = HZ_COUNTER(5);
            mcp_5hz_routines();
        }
        if (!--counter_100hz) {
            counter_100hz = HZ_COUNTER(100);
            mcp_100hz_routines();
        }
        if (!--counter_122hz) {
            counter_122hz = HZ_COUNTER(122);
            mcp_122hz_routines();
        }
        if (!--counter_200hz) {
            counter_200hz = HZ_COUNTER(200);
            mcp_200hz_routines();
        }
    }

    return NULL;
}

int main(int argc, char *argv[])
{
  ph_thread_t *main_thread = NULL;
  ph_thread_t *act_thread = NULL;
  ph_thread_t *mag_thread = NULL;
  ph_thread_t *inc_thread = NULL;
  ph_thread_t *gps_thread = NULL;
  ph_thread_t *lj_init_thread = NULL;
  ph_thread_t *DiskManagerID = NULL;
  ph_thread_t *bi0_send_worker = NULL;

  pthread_t CommandDatacomm1;
  pthread_t CommandDatacomm2;
  pthread_t CommandDataFIFO;
  pthread_t pilot_send_worker;
  pthread_t evtm_los_send_worker;
  pthread_t evtm_tdrss_send_worker;
  pthread_t highrate_send_worker;
  pthread_t CPU_monitor;
  int use_starcams = 0;

  if (argc == 1) {
    fprintf(stderr, "Must specify file type:\n"
        "p  pointing\n"
        "m  maps\n"
        "c  cryo\n"
        "n  noise\n"
        "x  software test\n"
        "f  flight\n");
    exit(0);
  }

  if (geteuid() != 0) {
      fprintf(stderr, "Sorry!  MCP needs to be run with root privileges.  Try `sudo ./mcp`\n");
      exit(0);
  }
  umask(0);  /* clear umask */


  ph_library_init();
  ph_nbio_init(4);

  /**
   * Begin logging
   */
  char log_file_name[PATH_MAX];
  {
      time_t start_time_s;

      start_time_s = time(&start_time_s);
      gmtime_r(&start_time_s, &start_time);

      snprintf(log_file_name, PATH_MAX, "/data/etc/blast/mcp_%02d-%02d-%02d_%02d:%02d.log",
              start_time.tm_mday, start_time.tm_mon + 1 , start_time.tm_year + 1900,
              start_time.tm_hour, start_time.tm_min);

      openMCElog(log_file_name);
  }

  /* register the output function */
  nameThread("Dummy"); // insert dummy sentinel node first
  nameThread("Scheduling");

  buos_use_func(mputs);

#if (TEMPORAL_OFFSET > 0)
  blast_warn("System: TEMPORAL OFFSET = %i\n", TEMPORAL_OFFSET);
#endif

  bputs(startup, "System: Startup");

  /* Find out whether I'm north or south */
  /* Note that South == fc2 */
  SouthIAm = AmISouth(&use_starcams);

  if (SouthIAm)
    bputs(info, "System: I am South.\n");
  else
    bputs(info, "System: I am not South.\n");

  // populate nios addresses, based off of tx_struct, derived
  channels_initialize(channel_list);

  InitCommandData(); // This should happen before all other threads

  blast_info("Commands: MCP Command List Version: %s", command_list_serial);

  // telemetry logger
	initLogger(&logger, log_file_name, 1);
	logger_buffer = channels_find_by_name("chatter")->var;

//  initialize_blast_comms();
//  initialize_sip_interface();
  initialize_dsp1760_interface();

  pthread_create(&CommandDataFIFO, NULL, (void*)&WatchFIFO, (void*)flc_ip[SouthIAm]);
  pthread_create(&CommandDatacomm1, NULL, (void*)&WatchPort, (void*)0);
  pthread_create(&CommandDatacomm2, NULL, (void*)&WatchPort, (void*)1);

    // TODO(ianlow13): find out if this is important
#ifndef BOLOTEST
  /* Initialize the Ephemeris */
//  ReductionInit("/data/etc/blast/ephem.2000");
  // framing_init(channel_list, derived_list);
  memset(PointingData, 0, 3 * sizeof(struct PointingDataStruct));
#endif


/* blast_info("Initializing Beaglebones from MCP...");
init_beaglebone();
blast_info("Finished initializing Beaglebones..."); */

  // initialize superframe FIFO
  master_superframe_buffer = calloc(1, superframe->size);
  for (int i = 0; i < NUM_TELEMETRIES; i++) { // initialize all fifos
    allocFifo(telem_fifo[i], 3, superframe->size);
  }

  // load all the linklists
  load_all_linklists(superframe, DEFAULT_LINKLIST_DIR, linklist_array, 0);
  generate_housekeeping_linklist(linklist_find_by_name(ALL_TELEMETRY_NAME, linklist_array), ALL_TELEMETRY_NAME);
  linklist_generate_lookup(linklist_array);
  ll_hk = linklist_find_by_name(ALL_TELEMETRY_NAME, linklist_array);

  // TODO(shubh): currently all linklists are set to the pilot linklist for testing purposes.
  // THIS NEEDS TO BE CHANGED: CommandData.pilot_linklist_name -> CommandData.XXXX_linklist_name

  // load the latest linklist into telemetry
  telemetries_linklist[PILOT_TELEMETRY_INDEX] =
      linklist_find_by_name(CommandData.pilot_linklist_name, linklist_array);
  telemetries_linklist[BI0_TELEMETRY_INDEX] =
      linklist_find_by_name(CommandData.pilot_linklist_name, linklist_array);
  telemetries_linklist[HIGHRATE_TELEMETRY_INDEX] =
      linklist_find_by_name(CommandData.pilot_linklist_name, linklist_array);
  telemetries_linklist[SBD_TELEMETRY_INDEX] =
      linklist_find_by_name(CommandData.pilot_linklist_name, linklist_array);
  telemetries_linklist[EVTM_LOS_TELEMETRY_INDEX] =
      linklist_find_by_name(CommandData.pilot_linklist_name, linklist_array);
  telemetries_linklist[EVTM_TDRSS_TELEMETRY_INDEX] =
      linklist_find_by_name(CommandData.pilot_linklist_name, linklist_array);

  struct evtmInfo evtm_los_info = {.telemetries = telemetries_linklist, .evtm_type = EVTM_LOS};
  struct evtmInfo evtm_tdrss_info = {.telemetries = telemetries_linklist, .evtm_type = EVTM_TDRSS};
  pthread_create(&pilot_send_worker, NULL, (void *) &pilot_compress_and_send, (void *) telemetries_linklist);
  pthread_create(&highrate_send_worker, NULL, (void *) &highrate_compress_and_send, (void *) telemetries_linklist);
  pthread_create(&evtm_los_send_worker, NULL, (void *) &EVTM_compress_and_send, (void *) &evtm_los_info);
  pthread_create(&evtm_tdrss_send_worker, NULL, (void *) &EVTM_compress_and_send, (void *) &evtm_tdrss_info);
  bi0_send_worker = ph_thread_spawn((void *) &biphase_writer, (void *) telemetries_linklist);


//  pthread_create(&disk_id, NULL, (void*)&FrameFileWriter, NULL);
  signal(SIGHUP, close_mcp);
  signal(SIGINT, close_mcp);
  signal(SIGTERM, close_mcp);
  signal(SIGPIPE, SIG_IGN);

  DiskManagerID = ph_thread_spawn((void *) &initialize_diskmanager, (void *) NULL);

  InitSched();
  initialize_motors();

// LJ THREAD
  // lj_init_thread = ph_thread_spawn(lj_connection_handler, NULL);
  init_labjacks(1, 1, 0, 0, 0, 1);
  mult_labjack_networking_init(LABJACK_MULT_OF, LABJACK_MAX_AIN, LABJACK_OF_SPP);
  mult_initialize_labjack_commands(LABJACK_MULT_OF);

  pthread_create(&CPU_monitor, NULL, CPU_health, NULL);

// this will need to be changed with the new star cameras
  if (use_starcams) {
       xsc_networking_init(0);
       xsc_networking_init(1);
       xsc_trigger(0, 0);
       xsc_trigger(1, 0);
  }

  // new star cam stuff
  // command setup
  pthread_t sc1_command_thread;
  pthread_t sc2_command_thread;
  pthread_t sc1_trigger_thread;
  pthread_t sc2_trigger_thread;
  struct socketData sc1_command_socket;
  struct socketData sc2_command_socket;
  struct socketData sc1_trigger_socket;
  struct socketData sc2_trigger_socket;
  // image setup
  pthread_t sc1_image_thread;
  pthread_t sc2_image_thread;
  struct socketData sc1_image_socket;
  struct socketData sc2_image_socket;
  // parameters setup
  pthread_t sc1_param_thread;
  pthread_t sc2_param_thread;
  struct socketData sc1_param_socket;
  struct socketData sc2_param_socket;
  // populate the structures with appropriate addresses and ports
  populateSocketData(SC1_IP_ADDR, SC1_RECEIVE_SOLVE_PORT, &sc1_image_socket);
  populateSocketData(SC2_IP_ADDR, SC2_RECEIVE_SOLVE_PORT, &sc2_image_socket);
  populateSocketData(SC1_IP_ADDR, SC1_RECEIVE_PARAM_PORT, &sc1_param_socket);
  populateSocketData(SC2_IP_ADDR, SC2_RECEIVE_PARAM_PORT, &sc2_param_socket);
  // this southiam check should be the logic to decide between ports
  if (SouthIAm) {
    populateSocketData(SC1_IP_ADDR, SC1_COMMAND_PORT_FC2, &sc1_command_socket);
    populateSocketData(SC2_IP_ADDR, SC2_COMMAND_PORT_FC2, &sc2_command_socket);
    populateSocketData(SC1_IP_ADDR, SC1_TRIGGER_PORT_FC2, &sc1_trigger_socket);
    populateSocketData(SC2_IP_ADDR, SC2_TRIGGER_PORT_FC2, &sc2_trigger_socket);
  } else {
    populateSocketData(SC1_IP_ADDR, SC1_COMMAND_PORT_FC1, &sc1_command_socket);
    populateSocketData(SC2_IP_ADDR, SC2_COMMAND_PORT_FC1, &sc2_command_socket);
    populateSocketData(SC1_IP_ADDR, SC1_TRIGGER_PORT_FC1, &sc1_trigger_socket);
    populateSocketData(SC2_IP_ADDR, SC2_TRIGGER_PORT_FC1, &sc2_trigger_socket);
  }
  // lets dispatch these threads now
  // SC1
  pthread_create(&sc1_command_thread, NULL, star_camera_command_thread, (void *) &sc1_command_socket);
  pthread_create(&sc1_image_thread, NULL, image_receive_thread, (void *) &sc1_image_socket);
  pthread_create(&sc1_param_thread, NULL, parameter_receive_thread, (void *) &sc1_param_socket);
  pthread_create(&sc1_trigger_thread, NULL, star_camera_trigger_thread, (void *) &sc1_trigger_socket);
  // SC2 (future)
  // TODO(Ian): when we get sc2 actually create the threads.

  initialize_magnetometer();
  mag_thread = ph_thread_spawn(monitor_magnetometer, NULL);

  initialize_inclinometer();
  inc_thread = ph_thread_spawn(monitor_inclinometer, NULL);

  // This is our (BLAST) GPS, used for timing and position.
  gps_thread = ph_thread_spawn(GPSMonitor, &GPSData);

  // This is CSBF's GPS, used for timing, position, and azimuth.
  StartDGPSmonitors();

  // pthread_create(&sensors_id, NULL, (void*)&SensorReader, NULL);
  // pthread_create(&compression_id, NULL, (void*)&CompressionWriter, NULL);
#ifndef USE_XY_THREAD
  // for now put ActBus inside ifndef so that only one of Actbus thread and XYbus thread run
  act_thread = ph_thread_spawn(ActuatorBus, NULL);
#endif
// Turns on software WD 2, which reboots the FC if not tickled
// TODO(evanmayer): FIXME: Don't want this for testing but MUST BE UNCOMMENTED FOR FLIGHT
// initialize_watchdog(SOFTWARE_WATCHDOG_TIMEOUT);

  startChrgCtrl(0);
  startChrgCtrl(1);

//  initialize the data sharing server
  data_sharing_init(linklist_array);

  main_thread = ph_thread_spawn(mcp_main_loop, NULL);
#ifdef USE_XY_THREAD // define should be set in mcp.h
  ph_thread_t *xy_thread = ph_thread_spawn(StageBus, NULL);
#endif
  ph_sched_run();

  blast_info("Joining main thread.");
  ph_thread_join(main_thread, NULL);

  return(0);
}
