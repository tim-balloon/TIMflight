// TODO(laura): This function doesn't appear to be called by anything in mcp.
// Is this BLASTPol code that Seth overwrote?
//
// int possible_solution(double az, double el, int i_point) {
//   double mag_az, enc_el, d_az;
//
//   // test for insanity
//   if (!finite(az)) return(0);
//   if (!finite(el)) return(0);
//   if (el > 70.0) return (0);
//   if (el < 0.0) return(0);
//
//   mag_az = PointingData[i_point].mag_az;
//
//   if (CommandData.use_mag) {
//     d_az = az - mag_az;
//
//     if (d_az > 180.0) d_az -= 360;
//     if (d_az < -180.0) d_az += 360;
//
//     if (d_az > 30.0) return (0);
//     if (d_az < -30.0) return (0);
//   }
//
//
//   return(1);
// }

// TODO(ianlowe13): remove deprecated, should be updated with new XSC stuff
static xsc_last_trigger_state_t *XSCHasNewSolution(int which)
{
    xsc_last_trigger_state_t *trig_state = NULL;

    // The latest solution isn't good
    if (!XSC_SERVER_DATA(which).channels.image_eq_valid) {
        return NULL;
    }

    // The camera system has just started
    if (XSC_SERVER_DATA(which).channels.image_ctr_stars < 0 || XSC_SERVER_DATA(which).channels.image_ctr_mcp < 0) {
        return NULL;
    }

    // The solution has already been processed
    if (XSC_SERVER_DATA(which).channels.image_ctr_stars == xsc_pointing_state[which].last_solution_stars_counter) {
        return NULL;
    }
    while ((trig_state = xsc_get_trigger_data(which))) {
        if ((XSC_SERVER_DATA(which).channels.image_ctr_mcp == trig_state->counter_mcp)
          && (XSC_SERVER_DATA(which).channels.image_ctr_stars == trig_state->counter_stars)) {
            break;
        }
        blast_dbg("Discarding trigger data with counter_mcp %d", trig_state->counter_mcp);
        blast_dbg("Discarding trigger data with image_ctr_mcp %d", XSC_SERVER_DATA(which).channels.image_ctr_mcp);
        blast_dbg("Discarding trigger data with counter_stars %d", trig_state->counter_stars);
        blast_dbg("Discarding trigger data with image_ctr_stars %d", XSC_SERVER_DATA(which).channels.image_ctr_stars);
        free(trig_state);
    }
    return trig_state;
}

// TODO(ianlowe13): remove deprecated, replace with new XSC stuff
/**
 * @brief Calculate the star camera pointing after incorporating all sensor
 * measurements.
 * @param which int Star camera identifier
 */
static void sc_calculate_full_pointing_estimated_location(int which)
{
    int pointing_read_index = GETREADINDEX(point_index);
    double az = from_degrees(PointingData[pointing_read_index].az);
    double el = from_degrees(PointingData[pointing_read_index].el);
    double sc_az = az - approximate_az_from_cross_el(CommandData.XSC[which].cross_el_trim, el);
    double sc_el = el - CommandData.XSC[which].el_trim;
    double sc_ra_hours = 0.0;
    double sc_dec_deg = 0.0;
    horizontal_to_equatorial(to_degrees(sc_az), to_degrees(sc_el),
                             PointingData[pointing_read_index].lst,
                             PointingData[pointing_read_index].lat, &sc_ra_hours, &sc_dec_deg);
    // Save off data to use as priors for new star camera solve
    PointingData[point_index].estimated_xsc_az_deg[which] = to_degrees(sc_az);
    PointingData[point_index].estimated_xsc_el_deg[which] = to_degrees(sc_el);
    PointingData[point_index].estimated_xsc_ra_hours[which] = sc_ra_hours;
    PointingData[point_index].estimated_xsc_dec_deg[which] = sc_dec_deg;
}