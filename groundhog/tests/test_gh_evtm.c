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
#include "bitserver.h"
#include "blast.h"
#include "channels_tng.h"

void __wrap_bprintf(buos_t l, const char *fmt, ...) {
    char message[BUOS_MAX];
    va_list argptr;

    va_start(argptr, fmt);
    vsnprintf(message, BUOS_MAX, fmt, argptr);
    va_end(argptr);

    bputs(info, message);

    if (l == fatal) {
        check_expected(fmt);
        function_called();
    }
}

int main(void) {
    const struct CMUnitTest tests[] = {
        // cmocka_unit_test_setup_teardown(test_EVTM_setup_config_LOS, EVTM_setup_unit_tests, NULL),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
