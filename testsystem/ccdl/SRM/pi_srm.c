/*
 * Copyright 2017 Razorcat Development GmbH
 */

/**\file Switching relay matrix function implementations of the
 * CCD test step execution engine platform interface.
 *
 * Simplistic implementation for demonstration purposes,
 * WITHOUT ANY ERROR REPORTING MESSAGES!!!
 * @author m.weber
 */

#include "pi_srm.h"
#include <string.h>
#include <ctype.h>
#include <ccd/ccd_runtime_core_interface.h>
#include <stdio.h>
#include <gaapi/api.h>

/***************************************************************************/

/** whether the Switching Relay Matrix is supported here */
#define SRM_SUPPORT 1

//---------------------------------------------------------------------------
/** Meta-data of a pin mapped to a PV in gamma.
 */
typedef struct {
    const char * srmName;
    const char * pinName;
    /** name of the PV that is used to control the relay */
    const char * pvName;
    /** the handle of the PV that is used to control the relay */
    ga_PV pvHandle;
//    gc_DataType type;
} MappedPin;

//---------------------------------------------------------------------------
// Global variables
//---------------------------------------------------------------------------
static MappedPin pins[] = { { .srmName = "srm1", .pinName = "a" }, { .srmName =
        "srm1", .pinName = "b" }, { .srmName = "", .pinName =
        "SRM.ROTARY_ENCODER_A", .pvName = "DO_ROTARY_ENCODER_A" },
        { .srmName = "", .pinName = "SRM.ROTARY_ENCODER_B", .pvName =
                "DO_ROTARY_ENCODER_B" }, { .srmName = "", .pinName =
                "SRM.FLOOR_LEVEL_0_SENSOR+", .pvName =
                "FLOOR_LEVEL_0_SENSOR_PIN" }, { .srmName = "", .pinName =
                "SRM.FLOOR_LEVEL_1_SENSOR+", .pvName =
                "FLOOR_LEVEL_1_SENSOR_PIN" }, { .srmName = "", .pinName =
                "SRM.FLOOR_LEVEL_2_SENSOR+", .pvName =
                "FLOOR_LEVEL_2_SENSOR_PIN" },
//{ .srmName = "", .pinName = "", .pvName="" },
        };

//---------------------------------------------------------------------------
//* internal functions
//---------------------------------------------------------------------------

static void rt_gamma5x_log_internal_err(const char *message, gc_ErrorID errcode) {
    gc_logMessage(gc_LOG_SEVERITY_ERROR, "%s: %s '%d'", message,
            gc_getErrorDescription(errcode), errcode);
}

/**
 * Initialises all known pins, which are hard-coded here.
 *
 * @return 0 on success, -1 on failure.
 */
static int initPins() {
    static char initialized = 0;
    if (!initialized) {
	initialized = 1; 
       for (unsigned int i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
            if (!pins[i].pvName)
                continue;
            gc_ErrorID ga_err;
            // get PV pointer
            pins[i].pvHandle = NULL;
            if (gc_ERROR_SUCCESS.errorID
                    != (ga_err = ga_pv_open(&pins[i].pvHandle, pins[i].pvName,
                            ga_PV_PROPERTY_ACC_NON_BLOCKING))) {
                rt_gamma5x_log_internal_err(__FUNCTION__, ga_err);
//                gc_logMessage(gc_LOG_SEVERITY_ERROR, "pinName: '%s'",
//                        pins[i].pinName);
                return -1; // failure
            }
        }
    }
    return 0; // success
}

/**
 * Initialises the SRM facility. Invoked from the startup code.
 * Errors will be logged.
 * @return 0 on success, -1 on failure.
 */
int rt_gamma5x_init_srm(void) {
    gc_logMessage(gc_LOG_SEVERITY_INFO, "%s() SRM", __FUNCTION__);
    return initPins();
//    return 0; // success
}

/**
 *
 * @return @c MappedPin* on success, @c NULL on failure.
 */
static MappedPin* getPinByName(const char* pinName) {
    for (unsigned int i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
        if (!strcmp(pins[i].pinName, pinName)) {
            return &pins[i];
        }
    }
    return NULL;
}

/**
 *
 * @return 0 on success, -1 on failure.
 */
static int setPvValue(const ga_PV pvHandle, unsigned char value) {
    gc_ErrorID ga_err;
//      rt_gamma5x_log_internal_warning("%s(): val= %g", __FUNCTION__, value);
// set the value object...
    gc_NumericValue nValue;
// assume 8 bit unsigned for now..
    nValue.uint8 = value;
    if (gc_ERROR_SUCCESS.errorID
            != (ga_err = ga_pv_putValue(pvHandle, &nValue))) {
        rt_gamma5x_log_internal_err(__FUNCTION__, ga_err);
        return -1; // failure
    }
    return 0; // success
}

/*-----------------------------------------------------------------------------
 * Interface functions
 *-----------------------------------------------------------------------------*/

int ccdl_te_disconnect(const char * srmName, const char * pinName) {
    initPins();
    MappedPin *pin = getPinByName(pinName);
    if (pin) {
        return setPvValue(pin->pvHandle, 1);
    }
    gc_logMessage(gc_LOG_SEVERITY_ERROR, "%s(): unknown pin '%s'", __FUNCTION__,
            pinName);
    return -1; // failure
}

int ccdl_te_reconnect(const char * srmName, const char * pinName) {
    initPins();
    MappedPin *pin = getPinByName(pinName);
    if (pin) {
        return setPvValue(pin->pvHandle, 0);
    }
    gc_logMessage(gc_LOG_SEVERITY_ERROR, "%s(): unknown pin '%s'", __FUNCTION__,
            pinName);
    return -1; // failure
}

int ccdl_te_shortCircuit(const char * srmName, const char * pin1,
        const char * pin2) {
    gc_logMessage(gc_LOG_SEVERITY_ERROR,
            "%s(): Switching Relay Matrix not supported", __FUNCTION__);
    return -1; // failure
}

int ccdl_te_releaseShortCircuit(const char * srmName, const char * pin1,
        const char * pin2) {
    gc_logMessage(gc_LOG_SEVERITY_ERROR,
            "%s(): Switching Relay Matrix not supported", __FUNCTION__);
    return -1; // failure
}

int ccdl_te_fireSRM(const char * srmName) {
    // nothing to do here, since gamma buffers the new state and will flush
    return 0;
}
