/*************************************************************************
 *
 * Copyright(C) 2010-2016 Razorcat Development GmbH. All Rights Reserved.
 *
 *************************************************************************
 */

/**\file
 *    This header contains declarations for the sample user functions
 *    provided with the CCDL installation package.
 * \brief   Sample user functions header file.
*/

#ifndef USERFUNCTIONS_H_
#define USERFUNCTIONS_H_

#include <ccd/ccd_types.h>
#include <ccd/ccd_test_engine_interface.h>

/* exact test for ccd runtime > 2.0.0 */
/*
#if defined( __CCD_RUNTIME_VERSION_MAJOR__) && \
   __CCD_RUNTIME_VERSION_MAJOR__ > 2  || \
   (__CCD_RUNTIME_VERSION_MAJOR__ == 2 && (__CCD_RUNTIME_VERSION_MINOR__ > 0 || \
      (__CCD_RUNTIME_VERSION_MINOR__ == 0 && \
	 __CCD_RUNTIME_VERSION_MICRO__ > 0)))
*/

/* simple test for ccd runtime >= 2.1.1 */
/*
#if defined( __CCD_RUNTIME_VERSION_MAJOR__)
 */



/******************************************************************************
*
* CCD Language extension functions (USER FUNCTIONS)
*
******************************************************************************/
#define TDL_MEASURESPEED_IMPLEMENTED
typedef struct {
	int counter;
} tdl_measureSpeed_t;

/**
*/
double tdl_measureSpeed (
                        ccdl_StatementReturnType * rp,
                        const char * module,
                        tdl_measureSpeed_t * data
                    );

/******************************************************************************/

#define TDL_ISINMAINTENANCEMODE_IMPLEMENTED
typedef struct {
	int counter;
} tdl_isInMaintenanceMode_t;

/**
*/
double tdl_isInMaintenanceMode (
                        ccdl_StatementReturnType * rp,
                        const char * module,
                        tdl_isInMaintenanceMode_t * data
                    );

/******************************************************************************/

#define TDL_INITRELAYMATRIX_IMPLEMENTED
typedef struct {
	int counter;
} tdl_initRelayMatrix_t;

/**
*/
double tdl_initRelayMatrix (
                        ccdl_StatementReturnType * rp,
                        const char * module,
                        tdl_initRelayMatrix_t * data
                    );

/******************************************************************************/



#endif /* USERFUNCTIONS_H_ */
