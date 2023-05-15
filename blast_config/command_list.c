
/* command_list.c: BLAST command specification file
 *
 * This software is copyright (C) 2002-20010 University of Toronto
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* !XXX!!XXX!!XXX!!XXX!!XXX!! BIG ALL CAPS WARNING !!XXX!!XXX!!XXX!!XXX!!XXX!!
 *
 * IF YOU ADD, MODIFY, OR DELETE *ANY* COMMANDS IN THIS FILE YOU *MUST*
 * RECOMPILE AND REINSTALL BLASTCMD ON ARWEN/WIDOW/!
 *
 * !XXX!!XXX!!XXX!!XXX!!XXX!! BIG ALL CAPS WARNING !!XXX!!XXX!!XXX!!XXX!!XXX!!
 */

#include "include/command_list.h"

const char *command_list_serial = "$Revision: 5.2 $";

const char *GroupNames[N_GROUPS] = {
                                    [GRPOS_POINT] = "Pointing Modes",
                                    [GRPOS_BAL] = "Balance",
                                    [GRPOS_HWPR] =  "Waveplate Rotator",
                                    [GRPOS_TRIM] = "Pointing Sensor Trims",
                                    [GRPOS_ELECT] = "Aux. Electronics",
                                    [GRPOS_BIAS] = "Bias",
                                    [GRPOS_ROACH] = "ROACH Commands",
                                    [GRPOS_VETO] = "Pointing Sensor Vetos",
                                    [GRPOS_ACT] = "Actuators",
                                    [GRPOS_XSC_HOUSE] = "XSC Housekeeping",
                                    [GRPOS_XSC_MODE] = "XSC Mode Settings",
                                    [GRPOS_XSC_PARAM] = "XSC Solving Parameters",
                                    [GRPOS_MOTOR] =  "Pointing Motors",
                                    [GRPOS_CRYO] = "Cryo Control",
                                    [GRPOS_POWER] = "Subsystem Power",
                                    [GRPOS_LOCK] = "Lock Motor",
                                    [GRPOS_TELEM] =  "Telemetry",
                                    [GRPOS_MISC] = "Miscellaneous",
                                    [GRPOS_FOCUS] = "Focus",
                                    [GRPOS_PSS] = "PSS",
  };

#define LINKLIST_SELECT "Linklist", 0, 64, 'i', "NONE", {linklist_names}

const char *downlink_names[] = {"Pilot", "Bi0", "Highrate", "SBD", 0};
const char *pilot_target_names[] = {"highbay", "gollum", "smeagol", "galadriel", 0};
const char *disable_enable[] = {"Disable", "Enable", 0};
const char *internal_external[] = {"Internal", "External", 0};
const char *stream_types[] = {"$ALL_VNA_SWEEPS", "$ALL_TARG_SWEEPS", "$ALL_IQ_DATA",
                              "$ALL_DF_DATA", "$ALL_LAMP_DATA", "$ALL_NOISE_COMP",
                              "$ALL_BB_FREQS", "$ALL_DS_VNA", 0};
const char *linklist_names[] = {0};


// echoes as string; makes enum name the command name string
#define COMMAND(x) (int)x, #x

struct scom scommands[xyzzy + 1] = {
    /* POWER SYSTEMS */
    // OF PBOB
    {COMMAND(fc1_on), "Turning on power to FC1", GR_POWER},
    {COMMAND(fc1_off), "Turning off power to FC1", GR_POWER},
    {COMMAND(fc2_on), "Turning on power to FC2", GR_POWER},
    {COMMAND(fc2_off), "Turning off power to FC2", GR_POWER},
    {COMMAND(motor_lj_on), "Turning on power to motor power LJ", GR_POWER},
    {COMMAND(motor_lj_off), "Turning off power to motor power LJ", GR_POWER},
    {COMMAND(of_relay_5_on), "Turning on power to OF relay 5", GR_POWER},
    {COMMAND(of_relay_5_off), "Turning off power to OF relay 5", GR_POWER},
    {COMMAND(of_inc_on), "Turning on power to OF inclinometer", GR_POWER},
    {COMMAND(of_inc_off), "Turning off power to OF inclinometer", GR_POWER},
    {COMMAND(mag_on), "Turning on power to magnetometers", GR_POWER},
    {COMMAND(mag_off), "Turning off power to magnetometers", GR_POWER},
    {COMMAND(therm_on), "Turning on power to thermistors", GR_POWER},
    {COMMAND(therm_off), "Turning off power to thermistors", GR_POWER},
    {COMMAND(gps_on), "Turning on power to GPS and NTP", GR_POWER},
    {COMMAND(gps_off), "Turning off power to GPS and NTP", GR_POWER},
    {COMMAND(pss_on), "Turning on power to PSS system", GR_POWER},
    {COMMAND(pss_off), "Turning off power to PSS system", GR_POWER},
    // IF PBOB
    {COMMAND(sc1_on), "Turning on power to SC1", GR_POWER},
    {COMMAND(sc1_off), "Turning off power to SC1", GR_POWER},
    {COMMAND(cryo_digital_on), "Turning on power to cryo readout", GR_POWER},
    {COMMAND(cryo_digital_off), "Turning off power to cryo readout", GR_POWER},
    {COMMAND(gyros_on), "Turning on power to gyros", GR_POWER},
    {COMMAND(gyros_off), "Turning off power to gyros", GR_POWER},
    {COMMAND(rfsoc_on), "Turning on power to RFSOC", GR_POWER},
    {COMMAND(rfsoc_off), "Turning off power to RFSOC", GR_POWER},
    {COMMAND(steppers_on), "Turning on power to stepper motors", GR_POWER},
    {COMMAND(steppers_off), "Turning off power to stepper motors", GR_POWER},
    {COMMAND(if_inc_on), "Turning on power to IF inclinometer", GR_POWER},
    {COMMAND(if_inc_off), "Turning off power to IF inclinometer", GR_POWER},
    {COMMAND(sc2_on), "Turning on power to SC2", GR_POWER},
    {COMMAND(sc2_off), "Turning off power to SC2", GR_POWER},
    {COMMAND(cryo_analog_on), "Turning on power to cryo sources", GR_POWER},
    {COMMAND(cryo_analog_off), "Turning off power to cryo sources", GR_POWER},
    {COMMAND(if_relay_10_on), "Turning on power to IF relay 10", GR_POWER},
    {COMMAND(if_relay_10_off), "Turning off power to IF relay 10", GR_POWER},
    /* HOUSEKEEPING */

    /* DETECTORS */

    /* POINTING */
    {COMMAND(antisun), "turn antisolar now", GR_POINT},
    {COMMAND(stop), "servo off of gyros to zero speed now", GR_POINT},
    // Vetoes & Allows
    {COMMAND(elclin_allow), "un-veto elevation clinometer", GR_VETO},
    {COMMAND(elclin_veto), "veto elevation clinometer", GR_VETO},
    {COMMAND(elmotenc_allow), "un-veto elevation motor encoder", GR_VETO},
    {COMMAND(elmotenc_veto), "veto elevation motor encoder", GR_VETO},
    {COMMAND(xsc0_allow), "un-veto star camera 0", GR_VETO},
    {COMMAND(xsc0_veto), "veto star camera 0", GR_VETO},
    {COMMAND(xsc1_allow), "un-veto star camera 1", GR_VETO},
    {COMMAND(xsc1_veto), "veto star camera 1", GR_VETO},
    {COMMAND(dgps_allow), "un-veto CSBF DGPS sensor", GR_VETO},
    {COMMAND(dgps_veto), "veto CSBF DGPS sensor", GR_VETO},
    {COMMAND(ifroll_1_gy_allow), "enable ifroll_1_gy", GR_VETO},
    {COMMAND(ifroll_1_gy_veto), "disable ifroll_1_gy", GR_VETO},
    {COMMAND(ifyaw_1_gy_allow), "enable ifyaw_1_gy", GR_VETO},
    {COMMAND(ifyaw_1_gy_veto), "disable ifyaw_1_gy", GR_VETO},
    {COMMAND(ifel_1_gy_allow), "enable ifel_1_gy", GR_VETO},
    {COMMAND(ifel_1_gy_veto), "disable ifel_1_gy", GR_VETO},
    {COMMAND(ifroll_2_gy_allow), "enable ifroll_2_gy", GR_VETO},
    {COMMAND(ifroll_2_gy_veto), "disable ifroll_2_gy", GR_VETO},
    {COMMAND(ifyaw_2_gy_allow), "enable ifyaw_2_gy", GR_VETO},
    {COMMAND(ifyaw_2_gy_veto), "disable ifyaw_2_gy", GR_VETO},
    {COMMAND(ifel_2_gy_allow), "enable ifel_2_gy", GR_VETO},
    {COMMAND(ifel_2_gy_veto), "disable ifel_2_gy", GR_VETO},
    {COMMAND(mag_allow_fc1), "un-veto magnetometer attached to fc1", GR_VETO},
    {COMMAND(mag_veto_fc1), "veto magnotometer attached to fc1", GR_VETO},
    {COMMAND(mag_allow_fc2), "un-veto magnetometer attached to fc2", GR_VETO},
    {COMMAND(mag_veto_fc2), "veto magnotometer attached to fc2", GR_VETO},
    {COMMAND(pss_allow), "un-veto pss sensor", GR_VETO | GR_PSS},
    {COMMAND(pss_veto), "veto pss sensor", GR_VETO | GR_PSS},
    // Sensors
    {COMMAND(az_auto_gyro), "automatically calculate az gyro offsets", GR_TRIM},
    {COMMAND(el_auto_gyro), "automatically calculate el gyro offset", GR_TRIM},
    {COMMAND(mag_reset), "command a reset of the magnetometer", GR_VETO | GR_TRIM},
    // Trims
    {COMMAND(trim_to_xsc0), "trim coarse sensors to XSC0 (disables autotrim)", GR_TRIM},
    {COMMAND(trim_to_xsc1), "trim coarse sensors to XSC1 (disables autotrim)", GR_TRIM},
    {COMMAND(trim_xsc0_to_xsc1), "trim XSC0 to XSC1", GR_TRIM},
    {COMMAND(trim_xsc1_to_xsc0), "trim XSC1 to XSC0", GR_TRIM},
    {COMMAND(autotrim_off), "disable auto-trim to XSC0/XSC1", GR_TRIM},
    {COMMAND(reset_trims), "reset coarse pointing trims to zero", GR_TRIM},

    /* MOTORS */
    {COMMAND(rw_wake_and_wiggle), "Trigger a wake-and wiggle re-commutation of the reaction wheel motor.", GR_MOTOR},
    // Motor gains
    {COMMAND(az_off), "disable az motors' gains", GR_MOTOR},
    {COMMAND(az_on), "enable az motors' gains", GR_MOTOR},
    {COMMAND(el_off), "disable el motor gains", GR_MOTOR},
    {COMMAND(el_on), "enable el motor gains", GR_MOTOR},
    {COMMAND(force_el_on), "force enable el motors despite the pin being in", CONFIRM | GR_MOTOR},
    // Resets
    {COMMAND(reset_rw), "reset the serial connection to the RW controller", GR_MOTOR},
    {COMMAND(reset_piv), "reset the serial connection to the pivot controller", GR_MOTOR},
    {COMMAND(reset_elev), "reset the serial connection to the elev controller", GR_MOTOR},
    {COMMAND(reset_ethercat), "reset communications with all EtherCat devices", GR_MOTOR},
    {COMMAND(restore_piv), "restore the serial settings for the pivot controller", GR_MOTOR},
    // TODO(ian) remove references to the HWPR
    /* ACTUATORS */
    {COMMAND(actbus_on), "turn on the Actuators, Lock, and HWPR", GR_POWER | GR_LOCK | GR_ACT | GR_HWPR},
    {COMMAND(actbus_off), "turn off the Actuators, Lock, and HWPR", GR_POWER | GR_LOCK | GR_ACT | GR_HWPR | CONFIRM},
    {COMMAND(actbus_cycle), "power cycle the Actuators and Lock", GR_POWER | GR_LOCK | GR_ACT | GR_HWPR | CONFIRM},
    {COMMAND(repoll), "force repoll of the stepper busses (act, lock, HWPR, XY)", GR_LOCK | GR_ACT | GR_HWPR},
    // Shutter
    {COMMAND(shutter_off), "Turn off shutter; shutter will fall open", GR_MISC},
    {COMMAND(shutter_init), "Initialize shutter move parameters", GR_MISC},
    {COMMAND(shutter_reset), "Reset shutter; shutter will open", GR_MISC},
    {COMMAND(shutter_close), "Close shutter (will NOT keep it closed)", GR_MISC},
    {COMMAND(shutter_open), "Open shutter", GR_MISC},
    {COMMAND(shutter_keepopen), "Open shutter and keep open with limit switch", GR_MISC},
    {COMMAND(shutter_keepclosed), "Close shutter and keep closed with limit switch", GR_MISC},
    // Lock pin
    {COMMAND(pin_in), "close lock pin without checking encoder (dangerous)", GR_LOCK | CONFIRM},
    {COMMAND(unlock), "unlock the inner frame", GR_LOCK},
    {COMMAND(lock_off), "turn off the lock motor", GR_LOCK},
    {COMMAND(lock45), "Lock the inner frame at 45 degrees", GR_LOCK},
    // Secondary mirror
    {COMMAND(autofocus_allow), "allow the secondary actuator system temperature correction mode", GR_FOCUS},
    {COMMAND(autofocus_veto), "veto the secondary actuator system temperature correction mode", GR_FOCUS},
    {COMMAND(actuator_stop), "stop all secondary actuators immediately", GR_ACT},
    // Balance system
    {COMMAND(balance_auto), "Put balance system into auto mode", GR_BAL},
    {COMMAND(balance_off),  "Turn off the balance motor", GR_BAL},
    {COMMAND(balance_terminate),  "Drive balance system to lower limit before termination after locking", GR_BAL},

    /* STAR CAMERAS */

    /* MISC */
    // Video transmitters
    {COMMAND(vtx_xsc0), "Setting video transmitter to XSC0", GR_XSC_MODE | GR_TELEM},
    {COMMAND(vtx_xsc1), "Setting video transmitter to XSC1", GR_XSC_MODE | GR_TELEM},
    // XY stage
    {COMMAND(xy_panic), "stop XY stage motors immediately", GR_MISC},
    // BLAST(TIM) Misc
    {COMMAND(reap_fc1), "ask MCP to reap the fc1 watchdog tickle", GR_MISC | CONFIRM},
    {COMMAND(reap_fc2), "ask MCP to reap the fc2 watchdog tickle", GR_MISC | CONFIRM},
    {COMMAND(halt_fc1), "ask MCP to halt fc1", GR_MISC | CONFIRM},
    {COMMAND(halt_fc2), "ask MCP to halt fc2", GR_MISC | CONFIRM},
    {COMMAND(blast_rocks), "the receiver rocks, use the happy schedule file", GR_TELEM},
    {COMMAND(blast_sucks), "the receiver sucks, use the sad schedule file", GR_TELEM},
    {COMMAND(at_float), "tell the scheduler that we're at float (don't run initial float controls)", GR_TELEM},
    {COMMAND(not_at_float), "tell the scheduler that we're not at float", GR_TELEM},
    {COMMAND(gps_sw_reset), "reset gps software", GR_TELEM},
    {COMMAND(gps_stats), "save gps nema + chrony stats to file at /data/etc/blast/gps/stats.txt", GR_TELEM},
    {COMMAND(reset_log), "Read the most recent log (clear cache)", GR_MISC},
    {COMMAND(xyzzy), "nothing happens here", GR_MISC}
};

/* parameter type:
 * i :  parameter is 16 bit unnormalised integer. Max is CMD_I_MAX
 * l :  parameter is 32 bit unnormalised integer. Max is CMD_L_MAX
 * f :  parameter is 16 bit renormalised floating point
 * d :  parameter is 32 bit renormalised floating point
 * s :  parameter is 7-bit character string JOY: actually 32 char long
 */
struct mcom mcommands[plugh + 2] = {
    /* HOUSEKEEPING */

    /* DETECTORS */

    /* NEW STAR CAMERAS */
    // SC1
    {COMMAND(sc1_trim_lat), "Send the commanded Latitude to SC1", GR_XSC_PARAM, 1,
        {
            {"Latitude", -90., 90., 'f', "NONE"}
        }
    },
    {COMMAND(sc1_trim_lon), "Send the commanded Longitude to SC1", GR_XSC_PARAM, 1,
        {
            {"Longitude", -180., 180., 'f', "NONE"}
        }
    },
    {COMMAND(sc1_trim_height), "Send the commanded Altitude to SC1", GR_XSC_PARAM, 1,
        {
            {"Altitude", 0., 50000., 'f', "NONE"}
        }
    },
    {COMMAND(sc1_trim_pos), "Send the commanded position to SC1", GR_XSC_PARAM, 3,
        {
            {"Latitude", -90., 90., 'f', "NONE"},
            {"Longitude", -180., 180., 'f', "NONE"},
            {"Altitude", 0., 50000., 'f', "NONE"}
        }
    },
    {COMMAND(sc1_set_exposure_time), "Set SC1 exposure time (msec)", GR_XSC_PARAM, 1,
        {
            {"Exposure time (msec)", 10., 1000., 'f', "NONE"}
        }
    },
    {COMMAND(sc1_set_logodds), "Set SC1 astrometry logodds", GR_XSC_PARAM, 1,
        {
            {"Logodds", 0., 10000000000., 'f', "NONE"}
        }
    },
    {COMMAND(sc1_set_time_limit), "Set the number of solve attempts SC1 is limited to per photo", GR_XSC_PARAM, 1,
        {
            {"Attempts", 1., 5., 'f', "NONE"}
        }
    },
    {COMMAND(sc1_set_af_parameters), "Set SC1 autofocus start/stop/step", GR_XSC_PARAM, 3,
        {
            {"Start position, refer to current minimum on KST", -3000, 3000, 'i', "NONE"},
            {"End position, refer to current maximum on KST", -3000, 3000, 'i', "NONE"},
            {"Step size, must be integer fraction of end-start", 1, 20, 'i', "NONE"}
        }
    },
    {COMMAND(sc1_set_af_photos), "Set SC1 autofocus photos per position", GR_XSC_PARAM, 1,
        {
            {"Photos", 1, 5, 'i', "NONE"}
        }
    },
    {COMMAND(sc1_set_focus_mode), "Set SC1 focus mode (0 = manual, 1 = auto)", GR_XSC_PARAM, 1,
        {
            {"Focus mode", 0, 1, 'i', "NONE"}
        }
    },
    {COMMAND(sc1_focus_move), "Move SC1 focus to x in encoder units", GR_XSC_PARAM, 1,
        {
            {"Focus position, see current allowed values in KST", -3000, 3000, 'i', "NONE"}
        }
    },
    {COMMAND(sc1_set_focus_inf), "Set SC1 focus to infinity", GR_XSC_PARAM, 1,
        {
            {"Set focus to infinity (1)", 0, 1, 'i', "NONE"}
        }
    },
    {COMMAND(sc1_change_aperture), "Change SC1 aperture by x steps", GR_XSC_PARAM, 1,
        {
            {"How many (+/-) steps?", -10, 10, 'i', "NONE"}
        }
    },
    {COMMAND(sc1_max_aperture), "Maximize SC1 aperture", GR_XSC_PARAM, 1,
        {
            {"Set aperture to max (1)", 0, 1, 'i', "NONE"}
        }
    },
    {COMMAND(sc1_make_static_hp), "Have SC1 remake the static hot pixel map", GR_XSC_PARAM, 1,
        {
            {"Make hot pixel map (1)", 0, 1, 'i', "NONE"}
        }
    },
    {COMMAND(sc1_use_static_hp), "Have SC1 use the most recent static hot pixel map", GR_XSC_PARAM, 1,
        {
            {"Use hot pixel map (1)", 0, 1, 'i', "NONE"}
        }
    },
    {COMMAND(sc1_set_spike_limit), "Set SC1 spike rejection limit (dynamic hot pixels)", GR_XSC_PARAM, 1,
        {
            {"Hot pixel limit (default 3)", 0., 255., 'f', "NONE"}
        }
    },
    {COMMAND(sc1_search_dynamic_hp), "Should SC1 look for dynamic hot pixels", GR_XSC_PARAM, 1,
        {
            {"Look for dynamic hot pixels (1)", 0., 1., 'f', "NONE"}
        }
    },
    {COMMAND(sc1_set_lpf_radius), "Set SC1 low pass filter radius", GR_XSC_PARAM, 1,
        {
            {"LPF radius (px)", 0., 5., 'f', "NONE"}
        }
    },
    {COMMAND(sc1_use_hpf), "Should SC1 high pass filter the image", GR_XSC_PARAM, 1,
        {
            {"Use high pass filter", 0., 1., 'f', "NONE"}
        }
    },
    {COMMAND(sc1_set_hpf_radius), "Set SC1 high pass filter radius", GR_XSC_PARAM, 1,
        {
            {"HPF radius (px)", 0., 50., 'f', "NONE"}
        }
    },
    {COMMAND(sc1_set_border), "Set SC1 image mask", GR_XSC_PARAM, 1,
        {
            {"Border mask size (px)", 0., 50., 'f', "NONE"}
        }
    },
    {COMMAND(sc1_set_unique_spacing), "Set SC1 minimum star spacing", GR_XSC_PARAM, 1,
        {
            {"Spacing (px)", 0., 50., 'f', "NONE"}
        }
    },
    {COMMAND(sc1_set_n_sigma), "Set SC1 minimum detection level above the background", GR_XSC_PARAM, 1,
        {
            {"Detection level (sigma)", 2., 30., 'f', "NONE"}
        }
    },
    // SC2
    {COMMAND(sc2_trim_lat), "Send the commanded Latitude to SC2", GR_XSC_PARAM, 1,
        {
            {"Latitude", -90., 90, 'f', "NONE"}
        }
    },
    {COMMAND(sc2_trim_lon), "Send the commanded Longitude to SC2", GR_XSC_PARAM, 1,
        {
            {"Longitude", -180., 180., 'f', "NONE"}
        }
    },
    {COMMAND(sc2_trim_height), "Send the commanded Altitude to SC2", GR_XSC_PARAM, 1,
        {
            {"Altitude", 0., 50000., 'f', "NONE"}
        }
    },
    {COMMAND(sc2_trim_pos), "Send the commanded position to SC2", GR_XSC_PARAM, 3,
        {
            {"Latitude", -90., 90., 'f', "NONE"},
            {"Longitude", -180., 180., 'f', "NONE"},
            {"Altitude", 0., 50000., 'f', "NONE"}
        }
    },
    {COMMAND(sc2_set_exposure_time), "Set SC2 exposure time (msec)", GR_XSC_PARAM, 1,
        {
            {"Exposure time (msec)", 10., 1000., 'f', "NONE"}
        }
    },
    {COMMAND(sc2_set_logodds), "Set SC2 astrometry logodds", GR_XSC_PARAM, 1,
        {
            {"Logodds", 0., 10000000000., 'f', "NONE"}
        }
    },
    {COMMAND(sc2_set_time_limit), "Set the number of solve attempts SC2 is limited to per photo", GR_XSC_PARAM, 1,
        {
            {"Attempts", 1., 5., 'f', "NONE"}
        }
    },
    {COMMAND(sc2_set_af_parameters), "Set SC2 autofocus start/stop/step", GR_XSC_PARAM, 3,
        {
            {"Start position, refer to current minimum on KST", -3000, 3000, 'i', "NONE"},
            {"End position, refer to current maximum on KST", -3000, 3000, 'i', "NONE"},
            {"Step size, must be integer fraction of end-start", 1, 20, 'i', "NONE"}
        }
    },
    {COMMAND(sc2_set_af_photos), "Set SC2 autofocus photos per position", GR_XSC_PARAM, 1,
        {
            {"Photos", 1, 5, 'i', "NONE"}
        }
    },
    {COMMAND(sc2_set_focus_mode), "Set SC2 focus mode (0 = manual, 1 = auto)", GR_XSC_PARAM, 1,
        {
            {"Focus mode", 0, 1, 'i', "NONE"}
        }
    },
    {COMMAND(sc2_focus_move), "Move SC2 focus to x in encoder units", GR_XSC_PARAM, 1,
        {
            {"Focus position, see current allowed values in KST", -3000, 3000, 'i', "NONE"}
        }
    },
    {COMMAND(sc2_set_focus_inf), "Set SC2 focus to infinity", GR_XSC_PARAM, 1,
        {
            {"Set focus to infinity (1)", 0, 1, 'i', "NONE"}
        }
    },
    {COMMAND(sc2_change_aperture), "Change SC2 aperture by x steps", GR_XSC_PARAM, 1,
        {
            {"How many (+/-) steps?", -10, 10, 'i', "NONE"}
        }
    },
    {COMMAND(sc2_max_aperture), "Maximize SC2 aperture", GR_XSC_PARAM, 1,
        {
            {"Set aperture to max (1)", 0, 1, 'i', "NONE"}
        }
    },
    {COMMAND(sc2_make_static_hp), "Have SC2 remake the static hot pixel map", GR_XSC_PARAM, 1,
        {
            {"Make hot pixel map (1)", 0, 1, 'i', "NONE"}
        }
    },
    {COMMAND(sc2_use_static_hp), "Have SC2 use the most recent static hot pixel map", GR_XSC_PARAM, 1,
        {
            {"Use hot pixel map (1)", 0, 1, 'i', "NONE"}
        }
    },
    {COMMAND(sc2_set_spike_limit), "Set SC2 spike rejection limit (dynamic hot pixels)", GR_XSC_PARAM, 1,
        {
            {"Hot pixel limit (default 3)", 0., 255., 'f', "NONE"}
        }
    },
    {COMMAND(sc2_search_dynamic_hp), "Should SC2 look for dynamic hot pixels", GR_XSC_PARAM, 1,
        {
            {"Look for dynamic hot pixels (1)", 0., 1., 'f', "NONE"}
        }
    },
    {COMMAND(sc2_set_lpf_radius), "Set SC2 low pass filter radius", GR_XSC_PARAM, 1,
        {
            {"LPF radius (px)", 0., 5., 'f', "NONE"}
        }
    },
    {COMMAND(sc2_use_hpf), "Should SC2 high pass filter the image", GR_XSC_PARAM, 1,
        {
            {"Use high pass filter", 0., 1., 'f', "NONE"}
        }
    },
    {COMMAND(sc2_set_hpf_radius), "Set SC2 high pass filter radius", GR_XSC_PARAM, 1,
        {
            {"HPF radius (px)", 0., 50., 'f', "NONE"}
        }
    },
    {COMMAND(sc2_set_border), "Set SC2 image mask", GR_XSC_PARAM, 1,
        {
            {"Border mask size (px)", 0., 50., 'f', "NONE"}
        }
    },
    {COMMAND(sc2_set_unique_spacing), "Set SC2 minimum star spacing", GR_XSC_PARAM, 1,
        {
            {"Spacing (px)", 0., 50., 'f', "NONE"}
        }
    },
    {COMMAND(sc2_set_n_sigma), "Set SC2 minimum detection level above the background", GR_XSC_PARAM, 1,
        {
            {"Detection level (sigma)", 2., 30., 'f', "NONE"}
        }
    },

    /* POINTING */
    {COMMAND(slew_veto), "set the length of the gyro offset slew veto (s)", GR_TRIM, 1,
        {
            {"Slew Veto (s)", 0., 1200., 'f', "SVETO_LEN"}
        }
    },
    // Scans
    {COMMAND(az_scan_accel), "set azimuth scan turnaround acceleration", GR_MOTOR, 1,
        {
            {"Az Acceleration", 0.1, 2.0, 'f', "accel_az"}
        }
    },
    {COMMAND(set_scan_params), "set dither index for next scan", GR_POINT, 1,
        {
            {"Next dither index ", 0, 200, 'i', "next_i_dith"}
        }
    },
    {COMMAND(ra_dec_goto), "track a location RA/Dec", GR_POINT, 2,
        {
            {"RA of Centre (h)",      0, 24, 'f', "RA"},
            {"Dec of Centre (deg)", -90, 90, 'f', "DEC"}
        }
    },
    {COMMAND(az_el_goto), "goto point in azimuth and elevation", GR_POINT, 2,
        {
            {"Azimuth (deg)", -360, 360, 'f', "AZ"},
            {"Elevation (deg)", 4.95,  65, 'f', "EL"}
        }
    },
    {COMMAND(box), "scan an az/el box centred on RA/Dec with el steps", GR_POINT, 7,
        {
            {"RA of Centre (h)",          0, 24, 'd', "RA"},
            {"Dec of Centre (deg)",     -90, 90, 'd', "DEC"},
            {"Az Width (deg on sky)",     0, 90, 'f', "NONE"},
            {"El Height (deg on sky)",    0, 45, 'f', "NONE"},
            {"Az Scan Speed (deg az/s)",  0,  2, 'f', "NONE"},
            {"El Step Size (deg on sky)", 0,  1, 'f', "NONE"},
            {"No. of dither steps",       0, 200, 'i', "n_dith"}
        }
    },
    {COMMAND(quad), "scan a quadrilateral region in RA/Dec (corners must be ordered)", GR_POINT, 11,
        {
            {"RA of Corner 1 (h)",        0, 24, 'f', "NONE"},
            {"Dec of Corner 1 (deg)",   -90, 90, 'f', "NONE"},
            {"RA of Corner 2 (h)",        0, 24, 'f', "NONE"},
            {"Dec of Corner 2 (deg)",   -90, 90, 'f', "NONE"},
            {"RA of Corner 3 (h)",        0, 24, 'f', "NONE"},
            {"Dec of Corner 3 (deg)",   -90, 90, 'f', "NONE"},
            {"RA of Corner 4 (h)",        0, 24, 'f', "NONE"},
            {"Dec of Corner 4 (deg)",   -90, 90, 'f', "NONE"},
            {"Az Scan Speed (deg az/s)",  0,  2, 'f', "NONE"},
            {"El Step Size (deg on sky)", 0,  1, 'f', "NONE"},
            {"No. of dither steps",       0, 200, 'i', "n_dith"}
        }
    },
    {COMMAND(cap), "scan a circle centred on RA/Dec with el steps", GR_POINT, 6,
        {
            {"RA of Centre (h)",          0, 24, 'd', "RA"},
            {"Dec of Centre (deg)",     -90, 90, 'd', "DEC"},
            {"Radius (deg on sky)",       0, 90, 'f', "NONE"},
            {"Az Scan Speed (deg az/s)",  0,  2, 'f', "NONE"},
            {"El Step Size (deg on sky)", 0,  1, 'f', "NONE"},
            {"No. of dither steps",       0, 200, 'i', "n_dith"}
        }
    },
    {COMMAND(el_scan), "scan in azimuth", GR_POINT, 4,
        {
            {"Az centre (deg)",       -180, 360, 'f', "AZ"},
            {"El centre (deg)",         15,  65, 'f', "EL"},
            {"Height (deg on sky)",       0, 360, 'f', "NONE"},
            {"El Scan Speed (deg az/s)", 0,   2, 'f', "NONE"}
        }
    },
    {COMMAND(el_box), "scan an az/el box centred on RA/Dec with az steps", GR_POINT, 7,
        {
            {"RA of Centre (h)",          0, 24, 'd', "RA"},
            {"Dec of Centre (deg)",     -90, 90, 'd', "DEC"},
            {"Az Width (deg on sky)",     0, 90, 'f', "NONE"},
            {"El Height (deg on sky)",    0, 45, 'f', "NONE"},
            {"El Scan Speed (deg az/s)",  0,  2, 'f', "NONE"},
            {"Az Step Size (deg on sky)", 0,  1, 'f', "NONE"},
            {"No. of dither steps",       0, 200, 'i', "n_dith"}
        }
    },
    {COMMAND(az_scan), "scan in azimuth", GR_POINT, 4,
        {
            {"Az centre (deg)",       -180, 360, 'f', "AZ"},
            {"El centre (deg)",         15,  65, 'f', "EL"},
            {"Width (deg on sky)",       0, 360, 'f', "NONE"},
            {"Az Scan Speed (deg az/s)", 0,   2, 'f', "NONE"}
        }
    },
    {COMMAND(cur_mode), "drive motors at constant current", GR_POINT, 3,
        {
            {"Pivot Current (Amps)", -20.0, 20.0, 'f', "0.0"},
            {"RW Current (Amps)", -20.0, 20.0, 'f', "0.0"},
            {"Elevation Current (Amps)", -20.0, 20.0, 'f', "0.0"}
        }
    },
    {COMMAND(drift), "move at constant speed in az and el", GR_POINT, 2,
        {
            {"Az Speed (deg/s on sky)", -2.0, 2.0, 'f', "0.0"},
            {"El Speed (deg/s on sky)", -2.0, 2.0, 'f', "0.0"}
        }
    },
    // Magnetometer
    {COMMAND(mag_cal_fc1), "set fc1 magnetometer calibration", GR_TRIM, 5,
        {
            {"Max X", -20, 20, 'd', "CAL_XMAX_MAG1"},
            {"Min X", -20, 20, 'd', "CAL_XMIN_MAG1"},
            {"Max Y", -20, 20, 'd', "CAL_YMAX_MAG1"},
            {"Min Y", -20, 20, 'd', "CAL_YMIN_MAG1"},
            {"Mag Angle Offset", -180.0, 180.0, 'f', "CAL_ALIGNMENT_MAG1"}
        }
    }, // 10 10 10.5 10.34
    {COMMAND(mag_cal_fc2), "set fc2 magnetometer calibration", GR_TRIM, 5,
        {
            {"Max X", -20, 20, 'd', "CAL_XMAX_MAG2"},
            {"Min X", -20, 20, 'd', "CAL_XMIN_MAG2"},
            {"Max Y", -20, 20, 'd', "CAL_YMAX_MAG2"},
            {"Min Y", -20, 20, 'd', "CAL_YMIN_MAG2"},
            {"Mag Angle Offset", -180.0, 180.0, 'f', "CAL_ALIGNMENT_MAG2"}
        }
    }, // 10 10 10.5 10.34
    // PSS
    {COMMAND(pss_cal_n), "set calibration for PSS N", GR_TRIM | GR_PSS, 5,
        {
            {"PSS number (1-6)",  1, 6, 'i', "NONE"},
            {"Distance offset",  -10.0, 10.0, 'f', "NONE"},
            {"Azimuth offset",   -10.0, 10.0, 'f', "NONE"},
            {"Elevation offset", -10.0, 10.0, 'f', "NONE"},
            {"Roll offset",      -180.0, 180.0, 'f', "NONE"},
        }
      },
    {COMMAND(pss_set_noise), "set pss noise level", GR_TRIM | GR_PSS, 1,
        {
            {"RMS deviation of summed voltages (volts)", 0.0, 10.0, 'f', "NOISE_PSS"},
        }
    },
    {COMMAND(pss_cal_d), "set pss distance calibration (mm)", GR_TRIM | GR_PSS, 6,
        {
            {"Distance offset 1", -10.0, 10.0, 'f', "CAL_D_PSS1"},
            {"Distance offset 2", -10.0, 10.0, 'f', "CAL_D_PSS2"},
            {"Distance offset 3", -10.0, 10.0, 'f', "CAL_D_PSS3"},
            {"Distance offset 4", -10.0, 10.0, 'f', "CAL_D_PSS4"},
            {"Distance offset 5", -10.0, 10.0, 'f', "CAL_D_PSS5"},
            {"Distance offset 6", -10.0, 10.0, 'f', "CAL_D_PSS6"},
        }
    },
    {COMMAND(pss_cal_el), "set pss elevation calibration (deg)", GR_TRIM | GR_PSS, 6,
        {
            {"Elevation offset 1", -10.0, 10.0, 'f', "CAL_EL_PSS1"},
            {"Elevation offset 2", -10.0, 10.0, 'f', "CAL_EL_PSS2"},
            {"Elevation offset 3", -10.0, 10.0, 'f', "CAL_EL_PSS3"},
            {"Elevation offset 4", -10.0, 10.0, 'f', "CAL_EL_PSS4"},
            {"Elevation offset 5", -10.0, 10.0, 'f', "CAL_EL_PSS5"},
            {"Elevation offset 6", -10.0, 10.0, 'f', "CAL_EL_PSS6"},
        }
    },
    {COMMAND(pss_cal_az), "set pss azimuth calibration (deg)", GR_TRIM | GR_PSS, 6,
        {
            {"Azimuth offset 1", -10.0, 10.0, 'f', "CAL_AZ_PSS1"},
            {"Azimuth offset 2", -10.0, 10.0, 'f', "CAL_AZ_PSS2"},
            {"Azimuth offset 3", -10.0, 10.0, 'f', "CAL_AZ_PSS3"},
            {"Azimuth offset 4", -10.0, 10.0, 'f', "CAL_AZ_PSS4"},
            {"Azimuth offset 5", -10.0, 10.0, 'f', "CAL_AZ_PSS5"},
            {"Azimuth offset 6", -10.0, 10.0, 'f', "CAL_AZ_PSS6"},
        }
    },
    {COMMAND(pss_cal_roll), "set pss roll calibration (deg)", GR_TRIM | GR_PSS, 6,
        {
            {"Roll offset 1", -180.0, 180.0, 'f', "CAL_ROLL_PSS1"},
            {"Roll offset 2", -180.0, 180.0, 'f', "CAL_ROLL_PSS2"},
            {"Roll offset 3", -180.0, 180.0, 'f', "CAL_ROLL_PSS3"},
            {"Roll offset 4", -180.0, 180.0, 'f', "CAL_ROLL_PSS4"},
            {"Roll offset 5", -180.0, 180.0, 'f', "CAL_ROLL_PSS5"},
            {"Roll offset 6", -180.0, 180.0, 'f', "CAL_ROLL_PSS6"},
        }
    },
    {COMMAND(pss_cal_array_az), "set pss azimuth calibration for entire array(deg)", GR_TRIM | GR_PSS, 1,
        {
            {"Azimuth offset of array", -20.0, 20.0, 'f', "CAL_AZ_PSS_ARRAY"},
        }
    },
    {COMMAND(pss_set_imin), "set pss minimum current", GR_TRIM | GR_PSS, 1,
        {
            {"I Min", 0.0, 20.0, 'f', "CAL_IMIN_PSS"}
        }
    },
    // Gyros
    {COMMAND(az_gyro_offset), "manually set az gyro offsets", GR_TRIM, 2,
        {
            {"IF Roll Gyro offset (deg/s)", -0.5, 0.5, 'd', "OFFSET_IFROLL_GY"},
            {"IF Yaw Gyro offset (deg/s)", -0.5, 0.5, 'd', "OFFSET_IFYAW_GY"}
        }
    },
    {COMMAND(el_gyro_offset), "manually set el gyro offset", GR_TRIM, 1,
        {
            {"IF Elev Gyro offset (deg/s)", -0.5, 0.5, 'd', "OFFSET_IFEL_GY"},
        }
    },
    // Trims
    {COMMAND(ra_dec_set), "define RA/Dec of current position", GR_TRIM, 2,
        {
            {"Current RA (h)",      0, 24, 'f', "RA"},
            {"Current Dec (deg)", -90, 90, 'f', "DEC"}
        }
    },
    {COMMAND(pos_set), "define Latitude/Longitude of current position", GR_TRIM, 2,
        {
            {"Current Latitude (deg)",      -90, 90, 'f', "LAT"},
            {"Current Longitude (deg)", -360, 360, 'f', "LON"}
        }
    },
    {COMMAND(az_el_trim), "trim sensors to azimuth and elevation", GR_TRIM, 2,
        {
            {"Azimuth (deg)", 0, 360, 'f', "AZ"},
            {"Elevation (deg)", 0, 90, 'f', "EL"}
        }
    },
    {COMMAND(autotrim_to_sc), "enable auto-trim to ISC/OSC", GR_TRIM, 3,
        {
            {"Threshold (sigma)", 0, 10, 'f', "THRESH_ATRIM"},
            {"Good time (s)", 0, CMD_I_MAX, 'i', "TIME_ATRIM"},
            {"Set rate (deg/s)", 0, 30, 'f', "RATE_ATRIM"}
        }
    },

    /* MOTORS */
    {COMMAND(fix_ethercat), "Attempt to fix EC device? (1=yes, 0=no)", GR_MOTOR, 4,
        {
            {"RW", 0, 1, 'i', "NONE"},
            {"El", 0, 1, 'i', "NONE"},
            {"Pivot", 0, 1, 'i', "NONE"},
            {"HWP", 0, 1, 'i', "NONE"},
        }
    },
    {COMMAND(az_gain), "az reaction wheel gains", GR_MOTOR, 4,
        {
            {"Proportional Gain", 0, CMD_I_MAX, 'd', "g_p_az"},
            {"Integral Time",     0, 200, 'd', "g_i_az"},
            {"Derivative Time",     0, 200, 'f', "g_d_az"},
            {"Pointing Gain", 0, CMD_I_MAX, 'd', "g_pt_az"},
        }
    },
    {COMMAND(pivot_gain), "pivot gains", GR_MOTOR, 6,
        {
            {"Set Point (dps)",   -200, 200, 'f', "SET_RW"},
            {"V_err Gain (prop)", 0, CMD_L_MAX, 'd', "G_PE_PIV"},
            {"V_err Integral time", 0, CMD_L_MAX, 'd', "G_IE_PIV"},
            {"V_RW Gain (prop)", 0, CMD_L_MAX, 'd', "G_PV_PIV"},
            {"V_RW Integral time", 0, 200, 'd', "G_IV_PIV"},
            {"Static Friction offset",   0, 100, 'f', "FRICT_OFF_PIV"},
        }
    },
    {COMMAND(el_gain), "elevation motor gains", GR_MOTOR, 6,
        {
            {"Proportional Gain", 0, CMD_L_MAX, 'd', "G_P_EL"},
            {"Integral Time",     0, 200, 'd', "G_I_EL"},
            {"Derivative Time",   0, 200, 'd', "G_D_EL"},
            {"Pointing Gain",     0, CMD_L_MAX, 'd', "G_PT_EL"},
            {"Integral Term Deadband  (mA)",     0, 500, 'f', "G_DB_EL"},
            {"Static Friction offset",   0, 100, 'f', "FRICT_OFF_EL"},
        }
    },
    {COMMAND(motors_verbose), "Set verbosity of motor serial threads (0=norm, 1=verbose, 2= superverbose )", GR_MISC, 3,
        {
            {"Reaction Wheel", 0, 5, 'i', "VERBOSE_RW"},
            {"Elevation", 0, 5, 'i', "VERBOSE_EL"},
            {"Pivot", 0, 5, 'i', "VERBOSE_PIV"}
        }
    },
    /* ACTUATORS */
    {COMMAND(actuator_i), "set the actuator motor currents", GR_ACT, 2,
        {
            {"Move current (%)", 0, 100, 'i', "I_MOVE_ACT"},
            {"Hold current (%)", 0,  50, 'i', "I_HOLD_ACT"}
        }
    },
    {COMMAND(actuator_vel), "set the actuator velocity and acceleration", GR_ACT, 2,
        {
            {"Velocity", 5, 20000, 'i', "VEL_ACT"},
            {"Acceleration", 1, 20, 'i', "ACC_ACT"}
        }
    },
    {COMMAND(actuator_tol), "set the tolerance for servo moves", GR_ACT, 1,
        {
            {"Move tolerance (~um)", 0, 1000, 'i', "TOL_ACT"}
        }
    },
    {COMMAND(actuator_servo), "servo the actuators to absolute positions", GR_ACT, 3,
        {
            {"Actuator Alpha (ENC units)", -15000, 15000, 'i', "ENC_0_ACT"},
            {"Actuator Beta (ENC units)",  -15000, 15000, 'i', "ENC_1_ACT"},
            {"Actuator Gamma (ENC units)", -15000, 15000, 'i', "ENC_2_ACT"}
        }
    },
    {COMMAND(general), "send a general command string to the lock or actuators", GR_ACT | GR_LOCK | GR_HWPR | GR_BAL, 2,
        {
            {"Address (1-10)", 1, 0x2F, 'i', "1.0"},
            {"Command", 0, 32, 's', "NONE"},
        }
    },
    // Lock Pin
    {COMMAND(lock_vel), "set the lock motor velocity and acceleration", GR_LOCK, 2,
        {
            {"Velocity", 5, 500000, 'l', "VEL_LOCK"},
            {"Acceleration", 1, 1000, 'i', "ACC_LOCK"},
        }
    },
    {COMMAND(lock_i), "set the lock motor currents", GR_LOCK, 2,
        {
            {"Move current (%)", 0, 100, 'i', "I_MOVE_LOCK"},
            {"Hold current (%)", 0,  50, 'i', "I_HOLD_LOCK"},
        }
    },
    {COMMAND(lock), "lock inner frame", GR_LOCK | GR_POINT, 1,
        {
            {"Lock Elevation (deg)", 5, 90, 'f', "EL_ENC"}
        }
    },
    // Shutter
    {COMMAND(shutter_i), "set shutter move and hold currents", GR_MISC, 2,
        {
            {"Shutter move current", 0, 42, 'i', "I_MOVE_SHUTTER"},
            {"Shutter hold current", 0, 42, 'i', "I_HOLD_SHUTTER"},
        }
    },
    {COMMAND(shutter_vel), "set shutter velocity and acceleration", GR_MISC, 2,
        {
            {"Shutter velocity", 0, 10000, 'i', "VEL_SHUTTER"},
            {"Shutter acceleration", 0, 20, 'i', "ACC_SHUTTER"},
        }
    },
    // Balance
    {COMMAND(balance_gain), "Set balance system setpoints", GR_BAL, 2,
        {
            {"I_El Balance On (A)",  0, 5, 'f', "I_LEVEL_ON_BAL"},
            {"I_El Balance Off  (A)", 0, 5, 'f', "I_LEVEL_OFF_BAL"},
        }
    },
    {COMMAND(balance_manual), "Manually set balance on", GR_BAL, 1,
        {
            {"dir (pos=1, neg=-1, off=0)",           -1, 1, 'i', "NONE"},
        }
    },
    {COMMAND(balance_vel), "set the balance system velocity and acceleration", GR_BAL, 2,
        {
            {"Velocity", 5, 500000, 'l', "VEL_BAL"},
            {"Acceleration", 1, 1000, 'i', "ACC_BAL"},
        }
    },
    {COMMAND(balance_i), "set the balance system currents", GR_BAL, 2,
        {
            {"Move current (%)", 0, 100, 'i', "I_MOVE_BAL"},
            {"Hold current (%)", 0,  50, 'i', "I_HOLD_BAL"},
        }
    },
    // Secondary mirror
    {COMMAND(set_secondary), "servo the secondary mirror to absolute position", GR_FOCUS, 1,
        {
            {"Position (per FOCUS_SF counts)", -15000, 15000, 'i', "FOCUS_SF"},
        }
    },
    {COMMAND(thermo_param), "set the thermal compensation parameters", GR_FOCUS, 3,
        {
            {"Temp. Spread", 0, 100, 'f', "SPREAD_SF"},
            {"Preferred T Prime", 0, 2, 'i', "PREF_TP_SF"},
            {"Preferred T Second", 0, 2, 'i', "PREF_TS_SF"}
        }
    },
    {COMMAND(focus_offset), "set the in focus position offset relative to the nominal focus", GR_FOCUS, 1,
        {
            {"Offset", -5000, 25000, 'i', "OFFSET_SF"}
        }
    },
    {COMMAND(thermo_gain), "set the secondary actuator system gains", GR_FOCUS, 4,
        {
            {"T. Primary Gain",   1, 1000, 'f', "G_PRIME_SF"},
            {"T. Secondary Gain", 1, 1000, 'f', "G_SECOND_SF"},
            {"Step Size (um)",   10, 1000, 'i', "STEP_SF"},
            {"Step Wait (min)"  , 0, 1500, 'i', "WAIT_SF"},
        }
    },
    {COMMAND(actuator_delta), "offset the actuators to from current position", GR_ACT, 3,
        {
            {"Actuator Alpha", -5000, 5000, 'i', "0"},
            {"Actuator Beta",  -5000, 5000, 'i', "0"},
            {"Actuator Gamma", -5000, 5000, 'i', "0"}
        }
    },
    {COMMAND(delta_secondary), "servo the secondary mirror by a relative amount", GR_FOCUS, 1,
        {
            {"Position (counts)", -1000, 1000, 'i', "0"},
        }
    },
    {COMMAND(act_enc_trim), "manually set encoder and dead reckoning", GR_ACT, 3,
        {
            {"Actuator Alpha (Enc units)", 0, 65536, 'f', "DR_0_ACT"},
            {"Actuator Beta (Enc units)",  0, 65536, 'f', "DR_1_ACT"},
            {"Actuator Gamma (Enc units)", 0, 65536, 'f', "DR_2_ACT"}
        }
    },

    /* TELEMETRY */
    {COMMAND(highrate_bw), "Highrate bandwidth", GR_TELEM, 2,
        {
            {"Bandwidth (kbps)", 0, 500, 'f', "rate_highrate"},
            {"Allframe fraction", 0, 1, 'f', "aff_highrate"}
        }
    },
    {COMMAND(pilot_bw), "pilot bandwidth", GR_TELEM, 2,
        {
            {"Bandwidth (kbps)", 0, 80000, 'f', "rate_pilot"},
            {"Allframe fraction", 0, 1, 'f', "aff_pilot"}
        }
    },
    {COMMAND(biphase_bw), "biphase bandwidth", GR_TELEM, 2,
        {
            {"Bandwidth (kbps)", 1, 2000, 'f', "rate_biphase"},
            {"Allframe fraction", 0, 1, 'f', "aff_biphase"}
        }
    },
    {COMMAND(biphase_clk_speed), "mpsse clock speed", GR_TELEM, 1,
        {
            {"Clock speed (kbps)", 100, 2000, 'i', "mpsse_clock_speed"}
        }
    },
    {COMMAND(highrate_through_tdrss), "Highrate downlink", GR_TELEM, 1,
        {
            {"TDRSS(1) or Iridium(0)", 0, 1, 'i', "NONE"}
        }
    },
    {COMMAND(set_linklists), "change linklists for downlink", GR_TELEM, 2,
        {
            {"Downlink", 0, 3, 'i', "NONE", {downlink_names}},
            {LINKLIST_SELECT}
        }
    },
    {COMMAND(request_file), "Stream a file at full bw over given link", GR_TELEM, 4,
        {
            {"Downlink", 0, 3, 'i', "NONE", {downlink_names}},
            {"File block number", 0, 255, 'i', ""},
            {"Fragment # (1-indexed; 0=>full file)", 0, CMD_L_MAX, 'l', ""},
            {"Absolute file path", 0, 64, 's', ""}
        }
    },
    {COMMAND(request_stream_file), "Stream a file at full bw over given link (compress first!)", GR_TELEM, 4,
        {
            {"Downlink", 0, 3, 'i', "NONE", {downlink_names}},
            {"File block number", 0, 255, 'i', ""},
            {"Fragment # (1-indexed; 0=>full file)", 0, CMD_L_MAX, 'l', ""},
            {"Type", 0, 64, 'l', "NONE", {stream_types}}
        }
    },
    {COMMAND(set_pilot_oth), "Set the pilot target to downlink", GR_TELEM, 1,
        {
            {"Downlink", 0, 3, 'i', "NONE", {pilot_target_names}},
        }
    },

    /* STAR CAMERAS */
    {COMMAND(xsc_is_new_window_period), "Set the time for which commands are valid (centi-seconds)", GR_XSC_PARAM, 2,
        {
            {"Which", 0, 2, 'i', "NONE"},
            {"Window period", 0, 2000, 'i', "NONE"},
        }
    },
    {COMMAND(xsc_offset), "Trim the star camera", GR_XSC_PARAM|GR_TRIM, 3,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
            {"Cross-El trim", -180, 180, 'd', "NONE"},
            {"El trim", -180, 180, 'd', "NONE"},
        }
    },
    {COMMAND(xsc_heaters_off), "Turn off the XSC heater", GR_XSC_HOUSE, 1,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
        }
    },
    {COMMAND(xsc_heaters_on), "Turn on the XSC heater", GR_XSC_HOUSE, 1,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
        }
    },
    {COMMAND(xsc_heaters_auto), "Allow XSC to control its heater", GR_XSC_HOUSE, 1,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
        }
    },
    {COMMAND(xsc_exposure_timing), "xsc exposure time", GR_XSC_PARAM, 4,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"exposure time_cs (cs) [default: 12]", 1, 100, 'l', "NONE"},
            {"grace period (s) (disregards which) [default: 45.0]", 1.0, 100.0, 'f', "NONE"},
            {"post trigger counter_mcp share delay (cs) (disregards which) [default: 200]", 1, 1000, 'l', "NONE"},
        },
    },
    {COMMAND(xsc_multi_trigger), "xsc trigger timing", GR_XSC_PARAM, 3,
        {
            {"num triggers [default: 1]", 0, 16, 'i', "NONE"},
            {"time between triggers (cs) [default: 19]", 1, 100, 'l', "NONE"},
            {"stars readout delay (s) [default: 1.0]", 0.1, 10.0, 'f', "NONE"},
        },
    },
    {COMMAND(xsc_trigger_threshold), "Allow XSC to trigger based on predicted px streaking", GR_XSC_PARAM, 3,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
            {"Enabled", 0, 1, 'i', "NONE"},
            {"Blob streaking limit (pixels", 0, 100, 'f', "NONE"},
        }
    },
    {COMMAND(xsc_scan_force_trigger), "Force XSC to trigger on turnaround (ignore speed)", GR_XSC_PARAM, 2,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
            {"Force the trigger on scan limits", 0, 1, 'i', "NONE"},
        }
    },
    {COMMAND(xsc_quit), "Quit XSC", GR_XSC_HOUSE|CONFIRM, 1,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
        }
    },
    {COMMAND(xsc_shutdown), "Shutdown the XSC computer", GR_XSC_HOUSE|CONFIRM, 2,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
            {"Restart?", 0, 1, 'i', "NONE"}
        }
    },
    {COMMAND(xsc_main_settings), "xsc main settings", GR_XSC_PARAM, 6,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"display_frequency [default: 20.0]", 1.0, 30.0, 'f', "NONE"},
            {"display_fullscreen [default: 1]", 0, 1, 'i', "NONE"},
            {"display_image_only [default: 0]", 0, 1, 'i', "NONE"},
            {"display_solving_filters [default: 0]]", 0, 1, 'i', "NONE"},
            {"display_image_brightness [default: 1.0]", 0.5, 4.0, 'f', "NONE"},
        },
    },
    {COMMAND(xsc_display_zoom), "xsc display zoom", GR_XSC_PARAM, 4,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"center x display pixel [default: 320]", 0, 1000, 'i', "NONE"},
            {"center y display pixel [default: 240]", 0, 1000, 'i', "NONE"},
            {"zoom [default: 1.0]", 1.0, 4.0, 'f', "NONE"},
        },
    },
    {COMMAND(xsc_image_client), "Enable or disable sending images to mcp", GR_XSC_MODE, 2,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
            {"Image client enabled", 0, 1, 'i', "NONE"},
        }
    },
    {COMMAND(xsc_init_focus), "Initialize the focus motor", GR_XSC_MODE, 1,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
        }
    },
    {COMMAND(xsc_get_focus), "Get the absolute focus position", GR_XSC_MODE, 1,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
        }
    },
    {COMMAND(xsc_set_focus), "Set the absolute focus position", GR_XSC_MODE, 2,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
            {"Absolute focus position", 0, 5000, 'i', "NONE"},
        }
    },
    {COMMAND(xsc_stop_focus), "Stop all motion on the focus actuator", GR_XSC_MODE, 1,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
        }
    },
    {COMMAND(xsc_define_focus), "Define the value of the focus at the current position", GR_XSC_MODE, 2,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
            {"Focus value", 0, 5000, 'i', "NONE"},
        }
    },
    {COMMAND(xsc_set_focus_incremental), "Command an incremental step to the focus motor", GR_XSC_MODE, 2,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
            {"Incremental focus steps", -5000, 5000, 'i', "NONE"},
        }
    },
    {COMMAND(xsc_run_autofocus), "xsc run autofocus", GR_XSC_PARAM, 1,
        {
            {"which", 0, 2, 'i', "NONE"},
        },
    },

    {COMMAND(xsc_set_autofocus_range), "xsc set autofocus range", GR_XSC_PARAM, 4,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"focus_search_min", 0, 5000, 'l', "NONE"},
            {"focus_search_max", 0, 5000, 'l', "NONE"},
            {"focus_search_step", 1, 1000, 'l', "NONE"},
        },
    },
    {COMMAND(xsc_abort_autofocus), "xsc abort autofocus", GR_XSC_PARAM, 2,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"still_use_solution", 0, 1, 'i', "NONE"},
        },
    },
    {COMMAND(xsc_autofocus_display_mode), "xsc autofocus display mode", GR_XSC_PARAM, 2,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"mode (0=auto, 1=on, 2=off)", 0, 2, 'i', "NONE"},
        },
    },
    {COMMAND(xsc_init_aperture), "Initialize the aperture motor", GR_XSC_MODE, 1,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
        }
    },
    {COMMAND(xsc_get_aperture), "Get Aperture", GR_XSC_MODE, 1,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
        }
    },
    {COMMAND(xsc_set_aperture), "Set the absolute aperture position", GR_XSC_MODE, 2,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
            {"Absolute aperture position", 0, 1000, 'i', "NONE"},
        }
    },
    {COMMAND(xsc_stop_aperture), "Stop all motion on the aperture actuator", GR_XSC_MODE, 1,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
        }
    },
    {COMMAND(xsc_define_aperture), "Define the value of the aperture at the current position", GR_XSC_MODE, 2,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
            {"Aperture value", 0, 1000, 'i', "NONE"},
        }
    },
    {COMMAND(xsc_get_gain), "xsc get gain", GR_XSC_PARAM, 0,
        {
            {"which", 0, 2, 'i', "NONE"},
        },
    },

    {COMMAND(xsc_set_gain), "xsc set gain", GR_XSC_PARAM, 0,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"position", 0.5, 10.0, 'f', "NONE"},
        },
    },
    {COMMAND(xsc_fake_sky_brightness), "xsc fake sky brightness", GR_XSC_PARAM, 0,
        {
            {"which", 0, 2, 'i', "NONE"},
            {"enabled", 0, 1, 'i', "NONE"},
            {"level_kepsa", 0.0, 10000.0, 'f', "NONE"},
            {"gain_db", -50, 10, 'f', "NONE"},
            {"actual exposure time (s)", 0.01, 10.0, 'f', "NONE"},
            {"simulated exposure time (s)", 0.01, 10.0, 'f', "NONE"},
        },
    },
    {COMMAND(xsc_solver_general), "Solver parameter settings", GR_XSC_PARAM, 3,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
            {"Enable the solver", 0, 1, 'i', "NONE"},
            {"Set the solver timeout period (s)", 5, 600, 'f', "NONE"},
        }
    },
    {COMMAND(xsc_solver_abort), "Abort the solving the current image", GR_XSC_MODE, 1,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"}
        }
    },
    {COMMAND(xsc_selective_mask), "Set the XSC selective mask", GR_XSC_PARAM, 5,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
            {"Enable selective masking", 0, 1, 'i', "NONE"},
            {"Mask field 1", 0, CMD_L_MAX, 'l', "NONE"},
            {"Mask field 2", 0, CMD_L_MAX, 'l', "NONE"},
            {"Mask field 3", 0, CMD_L_MAX, 'l', "NONE"},
        }
    },
    {COMMAND(xsc_blob_finding), "XSC blob finder settings", GR_XSC_PARAM, 5,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
            {"SNR Threshhold", 0.01, 10, 'f', "NONE"},
            {"Max num blobs", 3, 100, 'i', "NONE"},
            {"Enable Robust mode", 0, 1, 'i', "NONE"},
            {"Fitting method (0= none, 1=gaussian, 2=double gaussian)", 0, 2, 'i', "NONE"},
        }
    },
    {COMMAND(xsc_blob_cells), "XSC blob cell settings", GR_XSC_PARAM, 3,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
            {"Cell size (pixels, power of 2)", 4, 512, 'i', "NONE"},
            {"Max num blobs per cell (default 2)", 1, 10, 'i', "NONE"},
        }
    },
    {COMMAND(xsc_pattern_matching), "XSC pattern matching settings", GR_XSC_PARAM, 8,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
            {"Pattern match enable", 0, 1, 'i', "NONE"},
            {"Display Star names", 0, 1, 'i', "NONE"},
            {"Match Tolerance (pixels)", 0.01, 10, 'f', "NONE"},
            {"Platescale min (\"/px)", 6.0, 7.0, 'f', "NONE"},
            {"Platescale max (\"/px)", 6.0, 7.0, 'f', "NONE"},
            {"Use fixed Platescale", 0, 1, 'i', "NONE"},
            {"Fixed Platescale (\"/px)", 6.0, 7.0, 'f', "NONE"},
        }
    },
    {COMMAND(xsc_filter_hor_location), "XSC Horizontal Location Filter", GR_XSC_PARAM, 3,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
            {"Horizontal Limit Enabled", 0, 1, 'i', "NONE"},
            {"Horizontal Radius (degrees)", 0.0, 90.0, 'f', "NONE"},
        }
    },
    {COMMAND(xsc_filter_hor_roll), "XSC Horizontal Roll Limit", GR_XSC_PARAM, 4,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
            {"Horizontal Roll Limit", 0, 1, 'i', "NONE"},
            {"Minimum Horizontal Roll (degrees)", -180.0, 180.0, 'f', "NONE"},
            {"Maximum Horizontal Roll (degrees)", -180.0, 180.0, 'f', "NONE"},
        }
    },
    {COMMAND(xsc_filter_el), "XSC Elevation Limit", GR_XSC_PARAM, 4,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
            {"Elevation Limit", 0, 1, 'i', "NONE"},
            {"Minimum Elevation (degrees)", -90.0, 90.0, 'f', "NONE"},
            {"Maximum Elevation (degrees)", -90.0, 90.0, 'f', "NONE"},
        }
    },
    {COMMAND(xsc_filter_eq_location), "XSC Equatorial Location Filter", GR_XSC_PARAM, 3,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
            {"Equatorial Limit Enabled", 0, 1, 'i', "NONE"},
            {"Equatorial Radius (degrees)", 0.0, 90.0, 'f', "NONE"},
        }
    },
    {COMMAND(xsc_filter_matching), "XSC Matching Filter", GR_XSC_PARAM, 4,
        {
            {"Which camera (0, 1, 2=both)", 0, 2, 'i', "NONE"},
            {"Pointing Error threshold (arc seconds)", 0.01, 120, 'f', "NONE"},
            {"Pixel Error threshold (pixels)", 0.01, 20, 'f', "NONE"},
            {"Minimum stars matched", 4, 30, 'i', "NONE"},
        }
    },

    /* MISC */
    // XY stage
    {COMMAND(xy_goto), "move the X-Y translation stage to absolute position", GR_MISC, 4,
        {
            {"X destination", 0, 8000000, 'l', "X_STAGE"},
            {"Y destination", 0, 8000000, 'l', "Y_STAGE"},
            {"X speed", 0, 100000, 'l', "X_VEL_STAGE"},
            {"Y speed", 0, 100000, 'l', "Y_VEL_STAGE"}
        }
    },
    {COMMAND(xy_jump), "move the X-Y translation stage to relative position", GR_MISC, 4,
        {
            {"X delta", -1000000, 1000000, 'l', "0"},
            {"Y delta", -1000000, 1000000, 'l', "0"},
            {"X speed", 0, 100000, 'l', "X_VEL_STAGE"},
            {"Y speed", 0, 100000, 'l', "Y_VEL_STAGE"}
        }
    },
    {COMMAND(xy_xscan), "scan the X-Y translation stage in X", GR_MISC, 3,
        {
            {"X center", 0, 8000000, 'l', "X_STAGE"},
            {"delta X", 0, 4000000, 'l', "NONE"},
            {"X speed", 0, 1000000, 'l', "X_VEL_STAGE"},
        }
    },
    {COMMAND(xy_yscan), "scan the X-Y translation stage in Y", GR_MISC, 3,
        {
            {"Y center", 0, 8000000, 'l', "Y_STAGE"},
            {"delta Y", 0, 4000000, 'l', "NONE"},
            {"Y speed", 0, 1000000, 'l', "Y_VEL_STAGE"},
        }
    },
    {COMMAND(xy_raster), "raster the X-Y translation stage", GR_MISC, 7,
        {
            {"X center", 0, 10000000, 'l', "X_STAGE"},
            {"X Width", 0, 8000000, 'l', "NONE"},
            {"Y center", 0, 10000000, 'l', "Y_STAGE"},
            {"Y Width", 0, 8000000, 'l', "NONE"},
            {"X Velocity", 0, 5000000, 'l', "X_VEL_STAGE"},
            {"Y Velocity", 0, 5000000, 'l', "Y_VEL_STAGE"},
            {"Step Size", 0, 4000000, 'l', "NONE"},
        }
    },
    // Labjacks
    {COMMAND(set_queue_execute), "command queue changed", GR_CRYO, 1,
        {
            {"Labjack to execute queue", 0, 4, 'i', "LJ"},
        }
    },
    {COMMAND(reconnect_lj), "rebooting labjack cryo 1", GR_CRYO, 1,
        {
            {"Labjack to reconnect", 1, 7, 'i', "NONE"},
        }
    },
    // BLAST Misc
    {COMMAND(params_test), "Do nothing, with all parameter types", GR_MISC, 5,
        {
            {"i", 0, CMD_I_MAX, 'i', "NONE"},
            {"l", 0, CMD_L_MAX, 'l', "NONE"},
            {"f (-100 to +100)", -100, 100, 'f', "NONE"},
            {"d (-100 to +100)", -100, 100, 'd', "NONE"},
            {"s", 0, 32, 's', "NONE"}
        }
    },
    {COMMAND(timeout), "time until schedule mode", GR_TELEM, 1,
        {
            {"Timeout (s)", 2, 65535, 'f', "TIMEOUT"}
        }
    },
    {COMMAND(slot_sched), "set uplinked slot to use for schedule file", GR_TELEM, 1,
        {
            {"Slot #", 0, 250, 'i', "SLOT_SCHED"}
        }
    },
    {COMMAND(plugh), "A hollow voice says \"Plugh\".", GR_MISC, 1,
        {
          {"Plover", 0, CMD_I_MAX, 'i', "PLOVER"}
        }
    }
};

/* validate parameters of an mcom -- called by spidercmd before tranmitting a
 * command and by pcm after decoding one.  Inputs:
 *
 * cmd:         command number
 * [irs]values: mcp-style parsed parameters
 * buflen       size of the err_buffer
 * err_buffer   a place to write the error string
 *
 * Return value:
 *
 *  0:  if parameters are okay; err_buffer ignored.
 *  !0: if parameters are not okay.  In this case a descriptive error message
 *      should be written to err_buffer.
 */
int mcom_validate(enum multiCommand cmd, const int *ivalues,
    const double *rvalues, char svalues[][CMD_STRING_LEN], size_t buflen,
    char *err_buffer)
{
  return 0; /* no checks -- everything passes */
}
