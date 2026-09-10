/**
 * @brief Stores the 100Hz star camera data to the frame
 * 
 * @param which which star camera to store the data from
 */
void store_100hz_xsc(int which)
{
    static bool firsttime[2] = {true, true};
    static int last_blob_counter_stars[2] = {-1, -1};
    static int last_blob_i[2] = {1000, 1000};
    static int intermediate_frame_counter[2] = {0, 0};

    static channel_t* address_xN_ctr_stars[2];
    static channel_t* address_xN_image_ctr_mcp[2];
    static channel_t* address_xN_image_ctr_stars[2];

    static channel_t* address_xN_ctr_mcp;
    static channel_t* address_xN_last_trig_age_cs;
    static channel_t* address_xN_last_trig_ctr_mcp;
    static channel_t* address_xN_last_trig_ctr_stars[2];
    static channel_t* address_xN_predicted_streaking_px[2];
    static channel_t* address_xN_image_blobn_x[2];
    static channel_t* address_xN_image_blobn_y[2];
    static channel_t* address_xN_image_blobn_flux[2];
    static channel_t* address_xN_image_blobn_peak_to_flux[2];

    if (firsttime[which]) {
        firsttime[which] = false;

        if (which == 0) {
            address_xN_ctr_mcp                     = get_xsc_channel("ctr_mcp", 0);
            address_xN_last_trig_age_cs            = get_xsc_channel("last_trig_age_cs", 0);
            address_xN_last_trig_ctr_mcp           = get_xsc_channel("last_trig_ctr_mcp", 0);
        }
        address_xN_predicted_streaking_px[which]   = get_xsc_channel("predicted_streaking_px", which);
        address_xN_ctr_stars[which]                = get_xsc_channel("ctr_stars", which);
        address_xN_image_ctr_stars[which]          = get_xsc_channel("image_ctr_stars", which);
        address_xN_image_ctr_mcp[which]            = get_xsc_channel("image_ctr_mcp", which);
        address_xN_last_trig_ctr_stars[which]      = get_xsc_channel("last_trig_ctr_stars", which);
        address_xN_image_blobn_x[which]            = get_xsc_channel("image_blobn_x", which);
        address_xN_image_blobn_y[which]            = get_xsc_channel("image_blobn_y", which);
        address_xN_image_blobn_flux[which]         = get_xsc_channel("image_blobn_flux", which);
        address_xN_image_blobn_peak_to_flux[which] = get_xsc_channel("image_blobn_peak_to_flux", which);
    }

    if (which == 0) {
        SET_SCALED_VALUE(address_xN_ctr_mcp, xsc_pointing_state[which].counter_mcp);
        SET_SCALED_VALUE(address_xN_last_trig_age_cs, xsc_pointing_state[which].last_trigger.trigger_time);
        SET_SCALED_VALUE(address_xN_last_trig_ctr_mcp, xsc_pointing_state[which].last_trigger.counter_mcp);
    }


    SET_SCALED_VALUE(address_xN_predicted_streaking_px[which], xsc_pointing_state[which].predicted_streaking_px);
    SET_INT32(address_xN_ctr_stars[which], XSC_SERVER_DATA(which).channels.ctr_stars);
    SET_INT32(address_xN_image_ctr_stars[which], XSC_SERVER_DATA(which).channels.image_ctr_stars);
    SET_INT32(address_xN_image_ctr_mcp[which], XSC_SERVER_DATA(which).channels.image_ctr_mcp);
    SET_SCALED_VALUE(address_xN_last_trig_ctr_stars[which],
                     xsc_pointing_state[which].last_trigger.counter_stars);

    if (XSC_SERVER_DATA(which).blobs.counter_stars != last_blob_counter_stars[which] &&
        XSC_SERVER_DATA(which).blobs.counter_stars > 0) {
        last_blob_counter_stars[which] = XSC_SERVER_DATA(which).blobs.counter_stars;
        last_blob_i[which] = 0;
    }
    if (intermediate_frame_counter[which] == 0) {
        if (last_blob_i[which] < XSC_SERVER_DATA(which).blobs.num_blobs
                && last_blob_i[which] < XSC_BLOBS_ARRAY_SIZE) {
            SET_SCALED_VALUE(address_xN_image_blobn_x[which],
                             XSC_SERVER_DATA(which).blobs.blobs[last_blob_i[which]].x);
            SET_SCALED_VALUE(address_xN_image_blobn_y[which],
                             XSC_SERVER_DATA(which).blobs.blobs[last_blob_i[which]].y);
            SET_SCALED_VALUE(address_xN_image_blobn_flux[which],
                             XSC_SERVER_DATA(which).blobs.blobs[last_blob_i[which]].flux);
            SET_SCALED_VALUE(address_xN_image_blobn_peak_to_flux[which],
                             XSC_SERVER_DATA(which).blobs.blobs[last_blob_i[which]].peak_to_flux);
            last_blob_i[which]++;
        } else if (last_blob_i[which] == XSC_SERVER_DATA(which).blobs.num_blobs) {
            SET_SCALED_VALUE(address_xN_image_blobn_x[which], 0);
            SET_SCALED_VALUE(address_xN_image_blobn_y[which], 0);
            SET_SCALED_VALUE(address_xN_image_blobn_flux[which], 0);
            SET_SCALED_VALUE(address_xN_image_blobn_peak_to_flux[which], 0);
        }
    }

    intermediate_frame_counter[which] = (intermediate_frame_counter[which]+1) % 3;
}

/**
 * @brief Stores the 5Hz star camera data to the frame
 * 
 * @param m_which which star camera to store data from
 */
void store_5hz_xsc(int m_which)
{
    static bool firsttime[2] = {true, true};
    static channel_t* address_xN_point_az[2];
    static channel_t* address_xN_point_el[2];
    static channel_t* address_xN_point_var[2];
    static channel_t* address_xN_point_sigma[2];

    int i_point = GETREADINDEX(point_index);

    if (firsttime[m_which]) {
        firsttime[m_which] = false;
        address_xN_point_az[m_which]    = get_xsc_channel("point_az"    , m_which);
        address_xN_point_el[m_which]    = get_xsc_channel("point_el"    , m_which);
        address_xN_point_var[m_which]   = get_xsc_channel("point_var"   , m_which);
        address_xN_point_sigma[m_which] = get_xsc_channel("point_sigma" , m_which);
    }
    SET_SCALED_VALUE(address_xN_point_az[m_which]    , PointingData[i_point].xsc_az[m_which]);
    SET_SCALED_VALUE(address_xN_point_el[m_which]    , PointingData[i_point].xsc_el[m_which]);
    SET_SCALED_VALUE(address_xN_point_var[m_which]   , PointingData[i_point].xsc_var[m_which]);
    SET_SCALED_VALUE(address_xN_point_sigma[m_which] , PointingData[i_point].xsc_sigma[m_which]);
}


/**
 * @brief Stores the 1Hz camera data to the frame
 * 
 * @param m_which which star camera to store data from
 */
void store_1hz_xsc(int m_which)
{
    static bool firsttime[2] = {true, true};

    static channel_t* address_xN_point_az_raw[2];
    static channel_t* address_xN_point_el_raw[2];
    static channel_t* address_xN_point_az_trim[2];
    static channel_t* address_xN_point_el_trim[2];
    static channel_t* address_xN_cd_robust_mode[2];
//    static channel_t* address_xN_num_images_saved[2];

    static channel_t* address_xN_last_trig_lat;
    static channel_t* address_xN_last_trig_lst;

    static channel_t *address_xN_hk_temp_lens[2];
    static channel_t *address_xN_hk_temp_comp[2];
    static channel_t *address_xN_hk_temp_plate[2];
    static channel_t *address_xN_hk_temp_flange[2];
    static channel_t *address_xN_hk_pressure[2];
    static channel_t *address_xN_hk_disk[2];

    static channel_t *address_xN_image_eq_valid[2];
    static channel_t *address_xN_cam_gain_valid[2];
    static channel_t *address_xN_image_hor_valid[2];
    static channel_t *address_xN_image_afocus_metric_valid[2];

    static channel_t *address_xN_stars_run_time[2];
    static channel_t *address_xN_cam_gain_db[2];
    static channel_t *address_xN_lens_focus[2];
    static channel_t *address_xN_lens_aperture[2];

    static channel_t *address_xN_image_num_exposures[2];
    static channel_t *address_xN_image_stats_mean[2];
    static channel_t *address_xN_image_stats_noise[2];
    static channel_t *address_xN_image_stats_gaindb[2];
    static channel_t *address_xN_image_stats_num_px_sat[2];
    static channel_t *address_xN_image_stats_frac_px_sat[2];
    static channel_t *address_xN_image_afocus_metric[2];

    static channel_t *address_xN_image_eq_iplate[2];
    static channel_t *address_xN_image_hor_iplate[2];

    static channel_t *address_xN_image_eq_ra[2];
    static channel_t *address_xN_image_eq_dec[2];
    static channel_t *address_xN_image_eq_roll[2];
    static channel_t *address_xN_image_eq_sigma_ra[2];
    static channel_t *address_xN_image_eq_sigma_dec[2];
    static channel_t *address_xN_image_eq_sigma_roll[2];
    static channel_t *address_xN_image_eq_sigma_pointing[2];
    static channel_t *address_xN_image_hor_az[2];
    static channel_t *address_xN_image_hor_el[2];
    static channel_t *address_xN_image_hor_roll[2];
    static channel_t *address_xN_image_hor_sigma_az[2];
    static channel_t *address_xN_image_hor_sigma_el[2];
    static channel_t *address_xN_image_hor_sigma_roll[2];
    static channel_t *address_xN_image_hor_sigma_pointing[2];

    static channel_t *address_xN_image_num_blobs_found[2];
    static channel_t *address_xN_image_num_blobs_matched[2];

    if (firsttime[m_which]) {
        firsttime[m_which] = false;

        address_xN_image_num_blobs_found[m_which] = get_xsc_channel("image_num_blobs_found", m_which);
        address_xN_image_num_blobs_matched[m_which] = get_xsc_channel("image_num_blobs_matched", m_which);

        address_xN_hk_temp_lens[m_which] = get_xsc_channel("hk_temp_lens", m_which);
        address_xN_hk_temp_comp[m_which] = get_xsc_channel("hk_temp_comp", m_which);
        address_xN_hk_temp_plate[m_which] = get_xsc_channel("hk_temp_plate", m_which);
        address_xN_hk_temp_flange[m_which] = get_xsc_channel("hk_temp_flange", m_which);
        address_xN_hk_pressure[m_which] = get_xsc_channel("hk_pressure", m_which);
        address_xN_hk_disk[m_which] = get_xsc_channel("hk_disk", m_which);

        address_xN_image_eq_valid[m_which] = get_xsc_channel("image_eq_valid", m_which);
        address_xN_cam_gain_valid[m_which] = get_xsc_channel("cam_gain_valid", m_which);
        address_xN_image_hor_valid[m_which] = get_xsc_channel("image_hor_valid", m_which);
        address_xN_image_afocus_metric_valid[m_which] = get_xsc_channel("image_afocus_metric_valid", m_which);

        address_xN_stars_run_time[m_which] = get_xsc_channel("stars_run_time", m_which);
        address_xN_cam_gain_db[m_which] = get_xsc_channel("cam_gain_db", m_which);
        address_xN_lens_focus[m_which] = get_xsc_channel("lens_focus", m_which);
        address_xN_lens_aperture[m_which] = get_xsc_channel("lens_aperture", m_which);

        address_xN_image_num_exposures[m_which] = get_xsc_channel("image_num_exposures", m_which);
        address_xN_image_stats_mean[m_which] = get_xsc_channel("image_stats_mean", m_which);
        address_xN_image_stats_noise[m_which] = get_xsc_channel("image_stats_noise", m_which);
        address_xN_image_stats_gaindb[m_which] = get_xsc_channel("image_stats_gaindb", m_which);
        address_xN_image_stats_num_px_sat[m_which] = get_xsc_channel("image_stats_num_px_sat", m_which);
        address_xN_image_stats_frac_px_sat[m_which] = get_xsc_channel("image_stats_frac_px_sat", m_which);
        address_xN_image_afocus_metric[m_which] = get_xsc_channel("image_afocus_metric", m_which);

        address_xN_image_eq_iplate[m_which] = get_xsc_channel("image_eq_iplate", m_which);
        address_xN_image_hor_iplate[m_which] = get_xsc_channel("image_hor_iplate", m_which);

        address_xN_image_eq_ra[m_which] = get_xsc_channel("image_eq_ra", m_which);
        address_xN_image_eq_dec[m_which] = get_xsc_channel("image_eq_dec", m_which);
        address_xN_image_eq_roll[m_which] = get_xsc_channel("image_eq_roll", m_which);
        address_xN_image_eq_sigma_ra[m_which] = get_xsc_channel("image_eq_sigma_ra", m_which);
        address_xN_image_eq_sigma_dec[m_which] = get_xsc_channel("image_eq_sigma_dec", m_which);
        address_xN_image_eq_sigma_roll[m_which] = get_xsc_channel("image_eq_sigma_roll", m_which);
        address_xN_image_eq_sigma_pointing[m_which] = get_xsc_channel("image_eq_sigma_pointing", m_which);
        address_xN_image_hor_az[m_which] = get_xsc_channel("image_hor_az", m_which);
        address_xN_image_hor_el[m_which] = get_xsc_channel("image_hor_el", m_which);
        address_xN_image_hor_roll[m_which] = get_xsc_channel("image_hor_roll", m_which);
        address_xN_image_hor_sigma_az[m_which] = get_xsc_channel("image_hor_sigma_az", m_which);
        address_xN_image_hor_sigma_el[m_which] = get_xsc_channel("image_hor_sigma_el", m_which);
        address_xN_image_hor_sigma_roll[m_which] = get_xsc_channel("image_hor_sigma_roll", m_which);
        address_xN_image_hor_sigma_pointing[m_which] = get_xsc_channel("image_hor_sigma_pointing", m_which);

        address_xN_point_az_raw[m_which]  = get_xsc_channel("point_az_raw"    , m_which);
        address_xN_point_el_raw[m_which]  = get_xsc_channel("point_el_raw"    , m_which);
        address_xN_point_az_trim[m_which] = get_xsc_channel("point_az_trim"   , m_which);
        address_xN_point_el_trim[m_which] = get_xsc_channel("point_el_trim"   , m_which);
        address_xN_cd_robust_mode[m_which] = get_xsc_channel("cd_robust_mode"   , m_which);
//        address_xN_num_images_saved[m_which] = get_xsc_channel("num_images_saved"   , m_which);
        if (m_which == 0) {
            address_xN_last_trig_lat                = get_xsc_channel("last_trig_lat"        , 0);
            address_xN_last_trig_lst                = get_xsc_channel("last_trig_lst"        , 0);
        }
    }

    SET_SCALED_VALUE(address_xN_image_num_blobs_found[m_which],
                     XSC_SERVER_DATA(m_which).channels.image_num_blobs_found);
    SET_SCALED_VALUE(address_xN_image_num_blobs_matched[m_which],
                     XSC_SERVER_DATA(m_which).channels.image_num_blobs_matched);

    SET_SCALED_VALUE(address_xN_hk_temp_lens[m_which], XSC_SERVER_DATA(m_which).channels.hk_temp_lens);
    SET_SCALED_VALUE(address_xN_hk_temp_comp[m_which], XSC_SERVER_DATA(m_which).channels.hk_temp_comp);
    SET_SCALED_VALUE(address_xN_hk_temp_plate[m_which], XSC_SERVER_DATA(m_which).channels.hk_temp_plate);
    SET_SCALED_VALUE(address_xN_hk_temp_flange[m_which], XSC_SERVER_DATA(m_which).channels.hk_temp_flange);
    SET_SCALED_VALUE(address_xN_hk_pressure[m_which], XSC_SERVER_DATA(m_which).channels.hk_pressure);
    SET_SCALED_VALUE(address_xN_hk_disk[m_which], XSC_SERVER_DATA(m_which).channels.hk_disk);

    SET_SCALED_VALUE(address_xN_image_eq_valid[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_valid);
    SET_SCALED_VALUE(address_xN_cam_gain_valid[m_which], XSC_SERVER_DATA(m_which).channels.cam_gain_valid);
    SET_SCALED_VALUE(address_xN_image_hor_valid[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_valid);
    SET_SCALED_VALUE(address_xN_image_afocus_metric_valid[m_which],
                     XSC_SERVER_DATA(m_which).channels.image_afocus_metric_valid);

    SET_SCALED_VALUE(address_xN_stars_run_time[m_which], XSC_SERVER_DATA(m_which).channels.stars_run_time);
    SET_SCALED_VALUE(address_xN_cam_gain_db[m_which], XSC_SERVER_DATA(m_which).channels.cam_gain_db);
    SET_SCALED_VALUE(address_xN_lens_focus[m_which], XSC_SERVER_DATA(m_which).channels.lens_focus);
    SET_SCALED_VALUE(address_xN_lens_aperture[m_which], XSC_SERVER_DATA(m_which).channels.lens_aperture);

    SET_SCALED_VALUE(address_xN_image_num_exposures[m_which], XSC_SERVER_DATA(m_which).channels.image_num_exposures);
    SET_SCALED_VALUE(address_xN_image_stats_mean[m_which], XSC_SERVER_DATA(m_which).channels.image_stats_mean);
    SET_SCALED_VALUE(address_xN_image_stats_noise[m_which], XSC_SERVER_DATA(m_which).channels.image_stats_noise);
    SET_SCALED_VALUE(address_xN_image_stats_gaindb[m_which], XSC_SERVER_DATA(m_which).channels.image_stats_gaindb);
    SET_SCALED_VALUE(address_xN_image_stats_num_px_sat[m_which],
                     XSC_SERVER_DATA(m_which).channels.image_stats_num_px_sat);
    SET_SCALED_VALUE(address_xN_image_stats_frac_px_sat[m_which],
                     XSC_SERVER_DATA(m_which).channels.image_stats_frac_px_sat);
    SET_SCALED_VALUE(address_xN_image_afocus_metric[m_which], XSC_SERVER_DATA(m_which).channels.image_afocus_metric);

    SET_SCALED_VALUE(address_xN_image_eq_iplate[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_iplate);
    SET_SCALED_VALUE(address_xN_image_hor_iplate[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_iplate);

    SET_SCALED_VALUE(address_xN_image_eq_ra[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_ra);
    SET_SCALED_VALUE(address_xN_image_eq_dec[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_dec);
    SET_SCALED_VALUE(address_xN_image_eq_roll[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_roll);
    SET_SCALED_VALUE(address_xN_image_eq_sigma_ra[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_sigma_ra);
    SET_SCALED_VALUE(address_xN_image_eq_sigma_dec[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_sigma_dec);
    SET_SCALED_VALUE(address_xN_image_eq_sigma_roll[m_which], XSC_SERVER_DATA(m_which).channels.image_eq_sigma_roll);
    SET_SCALED_VALUE(address_xN_image_eq_sigma_pointing[m_which],
                     XSC_SERVER_DATA(m_which).channels.image_eq_sigma_pointing);
    SET_SCALED_VALUE(address_xN_image_hor_az[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_az);
    SET_SCALED_VALUE(address_xN_image_hor_el[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_el);
    SET_SCALED_VALUE(address_xN_image_hor_roll[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_roll);
    SET_SCALED_VALUE(address_xN_image_hor_sigma_az[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_sigma_az);
    SET_SCALED_VALUE(address_xN_image_hor_sigma_el[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_sigma_el);
    SET_SCALED_VALUE(address_xN_image_hor_sigma_roll[m_which], XSC_SERVER_DATA(m_which).channels.image_hor_sigma_roll);
    SET_SCALED_VALUE(address_xN_image_hor_sigma_pointing[m_which],
                     XSC_SERVER_DATA(m_which).channels.image_hor_sigma_pointing);

    SET_SCALED_VALUE(address_xN_point_az_raw[m_which] , xsc_pointing_state[m_which].az);
    SET_SCALED_VALUE(address_xN_point_el_raw[m_which] , xsc_pointing_state[m_which].el);
    SET_SCALED_VALUE(address_xN_point_az_trim[m_which], CommandData.XSC[m_which].cross_el_trim);
    SET_SCALED_VALUE(address_xN_point_el_trim[m_which], CommandData.XSC[m_which].el_trim);
    SET_SCALED_VALUE(address_xN_cd_robust_mode[m_which], CommandData.XSC[m_which].net.solver.robust_mode_enabled);

//    SET_SCALED_VALUE(address_xN_num_images_saved[m_which], images_num_saved[m_which]);
    if (m_which == 0) {
        SET_SCALED_VALUE(address_xN_last_trig_lat       , xsc_pointing_state[m_which].last_trigger.lat);
        SET_VALUE(address_xN_last_trig_lst              , xsc_pointing_state[m_which].last_trigger.lst*SEC2LI);
    }
}