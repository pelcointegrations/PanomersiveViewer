//============================================================================
// Copyright (c) 2014-2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#ifndef __PELCO_IMEXTK_NUMERIC_CALIBRATE_H__
#define __PELCO_IMEXTK_NUMERIC_CALIBRATE_H__

#include "CameraInfos.hpp"

typedef struct {
	double optera_pan;
	double optera_tilt;
	double spectra_pan;
	double spectra_tilt;
} pointpair_t;

typedef struct {
	double spectra_H;
	double spectra_X;
	double spectra_Z;
	double optera_tilt_err_X;
	double optera_tilt_err_Z;
	double spectra_tilt_err_X;
	double spectra_tilt_err_Z;
	double spectra_pan_offset;
} vars_t;

// enumeration to specify the index into the variable array for the
// various parameters
typedef enum {
	spectra_H_idx = 0,
	spectra_X_idx = 1,
	spectra_Z_idx = 2,
	optera_tilt_err_X_idx = 3,
	optera_tilt_err_Z_idx = 4,
	spectra_tilt_err_X_idx = 5,
	spectra_tilt_err_Z_idx = 6,
	spectra_pan_offset_idx = 7,
	NUM_VARS = 8
} eVar_idx;

int numeric_calibrate(pointpair_t*                   point_pair,       // (I)   point pair inputs
					  int                            num_point_pairs,  // (I)   # of point pairs
					  CameraInfos::OpteraCameraTypes optera_type,      // (I)   CAMERA_180/270/360
					  bool                           separated_camera, // (I)   optera separated from spectra
					  bool                           mutually_visible, // (I)   optera and spectra can see each other
					  float                          optera_h,         // (I)   height of optera (feet)
					  vars_t*                        var,              // (I/O) unknown vars to be solved for
					  int*                           num_iter,         // (O)   # of iterations solver used
					  double*                        final_err_feet);  // (O)   final error in point pair matching (feet)

cart_3d_point_t rotateCameraToMount(cart_3d_point_t input_pnt, double rot_X, double rot_Y, double rot_Z);
cart_3d_point_t rotateMountToCamera(cart_3d_point_t input_pnt, double rot_X, double rot_Y, double rot_Z);

// Testbench Definitions

typedef struct {
	float optera_H;
	float optera_X;
	float optera_Z;
	float spectra_H;
	float spectra_X;
	float spectra_Z;
	float optera_tilt_err_X;
	float optera_tilt_err_Z;
	float spectra_tilt_err_X;
	float spectra_tilt_err_Z;
} modelParameters;

void test_bench();
void system_model(modelParameters model_params, float target_x, float target_z, pointpair_t &results);

#endif // __PELCO_IMEXTK_NUMERIC_CALIBRATE_H__
