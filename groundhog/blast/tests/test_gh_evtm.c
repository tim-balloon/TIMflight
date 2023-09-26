/**
 * @file test_gh_evtm.c
 * @author Shubh Agrawal (shubh@sas.upenn.edu)
 * @brief This file is part of GroundHog, created for the Terahertz Intensity Mapper (TIM) project.
 * @date 2023-09-26
 * 
 * Copyright (c) 2023 University of Pennsylvania
 * 
 */

#include <setjmp.h>
#include <stdarg.h>
#include <stddef.h>
#include <cmocka.h>
#include <float.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <netinet/in.h> // socket stuff
#include <sys/socket.h> // socket stuff

#include "linklist.h"
#include "linklist_connect.h"
#include "groundhog_funcs.h"
#include "FIFO.h"
#include "blast.h"
#include "derived.h"
#include "bitserver.h"
#include "channels_tng.h"

/**
 * @brief gives a 
 * 
 * @return int 
 */
static int GH_EVTM_setup_unit_tests(void **state) {
    // channels_initialize(channel_list);
    return 0;
}

void test_EVTM_setup_receiver(void **state) {
    assert_true(1);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup_teardown(test_EVTM_setup_receiver, GH_EVTM_setup_unit_tests, NULL),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
