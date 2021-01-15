//============================================================================
// Copyright (c) 2014-2015 Pelco. All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================

#include "stdafx.h"
#include <atlstr.h>
#include <math.h>
#include <stdio.h>   // fprintf
#include <stdlib.h>  // malloc/free
#include <string.h>  // memcpy
#include <windows.h>
#include <ctime>
#include <limits>    // FLT_MAX

// local includes
#include "lbfgs.h"
#include "numeric_calibrate.h"
#include "mycamera.hpp"
#include "CameraInfos.hpp"

char debugMsg[300]; 

//#define _DEBUG_OUTPUT
#ifdef _DEBUG_OUTPUT
#define DOUT( s )            \
{                            \
   OutputDebugStringA( s );  \
}
#else
#define DOUT( s )
#endif


// Sets the bounds for the penalty function
lbfgsfloatval_t bound_radius[NUM_VARS];
lbfgsfloatval_t penalty_scale[NUM_VARS];

// Default values for the delta used in the numerical approximations for the gradient
lbfgsfloatval_t def_delta[NUM_VARS];

// Default mask for which variables we are going to vary.
bool def_vary_mask[NUM_VARS];

#define MAX_POINT_PAIRS 4
typedef struct {
	pointpair_t         point_pair[MAX_POINT_PAIRS];
	bool                vary[NUM_VARS];
	lbfgsfloatval_t     optera_H;
	lbfgsfloatval_t     optera_pan_offset;
	lbfgsfloatval_t     delta[NUM_VARS];
	lbfgsfloatval_t     upper_bound[NUM_VARS];
	lbfgsfloatval_t     lower_bound[NUM_VARS];
    int                 num_point_pairs;
	int					optera_type;
    int                 iter_count;
} lbfgs_context;


void initialize_context_params(CameraInfos::OpteraCameraTypes optera_type, bool separated_camera, bool mutually_visible) {

	// Default param values for Optera 360/270, separated, mutually visible case:

	// Sets the bounds for the penalty function
	bound_radius[0] = 10.0;    // spectra height (feet)
	bound_radius[1] = 125.0;   // spectra X position (feet)
	bound_radius[2] = 125.0;   // spectra Z position (feet)
	bound_radius[3] = 0.05;    // optera tilt error X axis (radians)
	bound_radius[4] = 0.05;    // optera tilt error Z axis (radians)
	bound_radius[5] = 0.05;	   // spectra tilt error X axis (radians)
	bound_radius[6] = 0.05;	   // spectra tilt error Z axis (radians)
	bound_radius[7] = 4.0;     // spectra pan offset (radians)

	// Sets the weights for the penalty function
	penalty_scale[0] = 5.0;    // spectra height
	penalty_scale[1] = 5.0;    // spectra X position
	penalty_scale[2] = 5.0;    // spectra Z position
	penalty_scale[3] = 50.0;   // optera tilt error X axis
	penalty_scale[4] = 50.0;   // optera tilt error Z axis
	penalty_scale[5] = 50.0;   // spectra tilt error X axis
	penalty_scale[6] = 50.0;   // spectra tilt error Z axis
	penalty_scale[7] = 5.0;    // spectra pan offset

	// Default values for the delta used in the numerical approximations for the gradient
	def_delta[0] = 1e-5;    // spectra height
	def_delta[1] = 1e-5;    // spectra X position
	def_delta[2] = 1e-5;    // spectra Z position
	def_delta[3] = 1e-5;    // optera tilt error X axis
	def_delta[4] = 1e-5;    // optera tilt error Z axis
	def_delta[5] = 1e-5;    // spectra tilt error X axis
	def_delta[6] = 1e-5;    // spectra tilt error Z axis
	def_delta[7] = 1e-5;    // spectra pan offset

	// Default mask for which variables we are going to vary.
	def_vary_mask[0] = true;    // spectra height
	def_vary_mask[1] = true;    // spectra X position
	def_vary_mask[2] = true;    // spectra Z position
	def_vary_mask[3] = true;    // optera tilt error X axis
	def_vary_mask[4] = true;    // optera tilt error Z axis
	def_vary_mask[5] = true;    // spectra tilt error X axis
	def_vary_mask[6] = true;    // spectra tilt error Z axis
	def_vary_mask[7] = false;   // spectra pan offset

	// Modifiers for 360/270 Cameras

	if ((optera_type == CameraInfos::CAMERA_360) || (optera_type == CameraInfos::CAMERA_270)) {
		if (!separated_camera) {
			// tighten up spectra position range for co-located case
			bound_radius[1] = 1.0;   // spectra X position (feet)
			bound_radius[2] = 1.0;   // spectra Z position (feet)
		}
		else if (!mutually_visible) {
			// solve for spectra pan offset
			def_vary_mask[7] = true;    // spectra pan offset
		}
	}

	// Modifiers for 180 Cameras:

	if (optera_type == CameraInfos::CAMERA_180) {
		// allow for +/-90 deg tilt on optera & solving for spectra pan offset
		bound_radius[4]  = 1.50;	// optera tilt error Z axis (radians)
		def_vary_mask[7] = true;    // spectra pan offset

		if (!separated_camera) {
			// tighten up spectra position range for co-located case
			bound_radius[1] = 1.0;   // spectra X position (feet)
			bound_radius[2] = 1.0;   // spectra Z position (feet)
		}
	}
}

double errorVal(lbfgs_context* pCtx, lbfgsfloatval_t* in_var, int print_delta, double* max_err) {

	// unknown variables
	double spectra_H		  = in_var[spectra_H_idx];
	double spectra_X          = in_var[spectra_X_idx];
	double spectra_Z          = in_var[spectra_Z_idx];
	double optera_tilt_err_X  = in_var[optera_tilt_err_X_idx];
	double optera_tilt_err_Z  = in_var[optera_tilt_err_Z_idx];
	double spectra_tilt_err_X = in_var[spectra_tilt_err_X_idx];
	double spectra_tilt_err_Z = in_var[spectra_tilt_err_Z_idx];
	double spectra_pan_offset = in_var[spectra_pan_offset_idx];

    double retval = 0.0;

	// Calc target locations for each point pair
	double total_squared_error = 0.0;
	double max = 0.0;
	int i;
	for (i = 0; i < pCtx->num_point_pairs; i++) {

		// OPTERA

		// Calc xyz point at Optera unit sphere in Not - Parallel coordinate space
		float R = 1.0f;
		double optera_tilt = pCtx->point_pair[i].optera_tilt;
		double optera_pan = pCtx->point_pair[i].optera_pan;
		double optera_pan_offset = pCtx->optera_pan_offset;
		cart_3d_point_t camera_pnt, mount_pnt;
		camera_pnt.X = R * cos(optera_tilt) * cos(-optera_pan);  // expecting CCW pan, optera pan is CW
		camera_pnt.Y = R * sin(optera_tilt);
		camera_pnt.Z = -R * cos(optera_tilt) * sin(-optera_pan); // minus(-R) to adjust for right hand coord.system

		// Apply XYZ rotations to transform this point to Parallel coordinate space
		double c2m_rotation_X = optera_tilt_err_X; // tilts are defined mount to camera
		double c2m_rotation_Y = -optera_pan_offset; // optera offset is CW rotation, need CCW for this transform
		double c2m_rotation_Z = optera_tilt_err_Z;
		mount_pnt = rotateCameraToMount(camera_pnt, c2m_rotation_X, c2m_rotation_Y, c2m_rotation_Z);

		// Calc Optera pan / tilt in Parallel coordinate space
		optera_pan = -atan2(mount_pnt.Z, mount_pnt.X);  // minus for right hand coord sys
		optera_tilt = atan2(mount_pnt.Y, sqrt(mount_pnt.X * mount_pnt.X + mount_pnt.Z * mount_pnt.Z));

		// Calc target location from Optera parallel coordinate space
		double optera_d = pCtx->optera_H / sin(-optera_tilt);
		double optera_target_x = optera_d * cos(optera_tilt) * cos(optera_pan);   // expects CCW pan rotation
		double optera_target_z = optera_d * cos(optera_tilt) * -sin(optera_pan);

		// SPECTRA

		// Calc xyz point at Spectra unit sphere in Not-Parallel coordinate space
		R = 1.0f;
		double spectra_tilt = pCtx->point_pair[i].spectra_tilt;
		double spectra_pan = pCtx->point_pair[i].spectra_pan;
		camera_pnt.X = R * cos(spectra_tilt) * cos(-spectra_pan);  // expecting CCW pan, optera pan is CW
		camera_pnt.Y = R * sin(spectra_tilt);
		camera_pnt.Z = -R * cos(spectra_tilt) * sin(-spectra_pan); // minus(-R) to adjust for right hand coord.system

		// Apply XYZ rotations to transform this point to Parallel coordinate space
		c2m_rotation_X = spectra_tilt_err_X; // tilts are defined mount to camera
		c2m_rotation_Z = spectra_tilt_err_Z;
		c2m_rotation_Y = -spectra_pan_offset; // spectra offset is CW rotation, need CCW for this transform
		mount_pnt = rotateCameraToMount(camera_pnt, c2m_rotation_X, c2m_rotation_Y, c2m_rotation_Z);

		// Calc Spectra pan / tilt in Parallel coordinate space
		spectra_pan = -atan2(mount_pnt.Z, mount_pnt.X);  // minus for right hand coord sys
		spectra_tilt = atan2(mount_pnt.Y, sqrt(mount_pnt.X * mount_pnt.X + mount_pnt.Z * mount_pnt.Z));

		// Calc target location from Spectra parallel coordinate space
		double spectra_d = spectra_H / sin(-spectra_tilt);
		double spectra_target_x = spectra_d * cos(spectra_tilt) * cos(spectra_pan) + spectra_X;
		double spectra_target_z = spectra_d * cos(spectra_tilt) * -sin(spectra_pan) + spectra_Z;

		double x_diff = optera_target_x - spectra_target_x;
		double z_diff = optera_target_z - spectra_target_z;
		double squared_error = (x_diff * x_diff) + (z_diff * z_diff);
		total_squared_error += squared_error;

		// If we are looking for statistics about the error compute it
		if ((print_delta) || (max_err)) {
			double err = sqrt(total_squared_error);
			if (print_delta) {
				sprintf_s(debugMsg, sizeof(debugMsg), "point: %d error: %f\n", i, err);
				DOUT(debugMsg);
		     }
		    if (err > max) {
		        max = err;
		    }
		}
	}

	retval = sqrt(total_squared_error);

    // Now add in the penalty values
    double penalty = 0.0;
	int ii;
	for (ii = 0; ii < NUM_VARS; ii++) {
        if (in_var[ii] > pCtx->upper_bound[ii]) {
            double delta = penalty_scale[ii] * (in_var[ii] - pCtx->upper_bound[ii]);
            penalty += delta * delta;
        }

        if (in_var[ii] < pCtx->lower_bound[ii]) {
            double delta = penalty_scale[ii] * (pCtx->lower_bound[ii] - in_var[ii]);
            penalty += delta * delta;
        }
    }

    retval += penalty;

	if (max_err) {
		*max_err = max;
    }

    return retval;
}

/**
 * Callback interface to receive the progress of the optimization process.
 *
 *  The lbfgs() function calls this function for each iteration. Implementing
 *  this function, a client program can store or display the current progress
 *  of the optimization process.
 *
 *  @param  context     The user data sent for lbfgs() function by the client.
 *  @param  x           The current values of variables.
 *  @param  g           The current gradient values of variables.
 *  @param  fx          The current value of the objective function.
 *  @param  xnorm       The Euclidean norm of the variables.
 *  @param  gnorm       The Euclidean norm of the gradients.
 *  @param  step        The line-search step used for this iteration.
 *  @param  n           The number of variables.
 *  @param  k           The iteration count.
 *  @param  ls          The number of evaluations called for this iteration.
 *  @retval int         Zero to continue the optimization process. Returning a
 *                      non-zero value will cancel the optimization process.
 */
int progress(void *context,
             const lbfgsfloatval_t *x,
             const lbfgsfloatval_t *g,
             const lbfgsfloatval_t fx,
             const lbfgsfloatval_t xnorm,
             const lbfgsfloatval_t gnorm,
             const lbfgsfloatval_t step,
             int n,
             int k,
             int ls) {
    (void)g;
    (void)xnorm;
    (void)gnorm;
    (void)n;

    lbfgs_context* pCtx = (lbfgs_context *)context;

	sprintf_s(debugMsg, sizeof(debugMsg), "progress: spectra_height: %6.3f spectra_X: %6.3f spectra_Z: %6.3f optera_tilt_err_X: %6.3f optera_tilt_err_Z: %6.3f spectra_tilt_err_X: %6.3f spectra_tilt_err_Z: %6.3f spectra_pan_offset: %6.3f fx: %6.3f iter_cnt: %d num_evals: %d step: %6.3f\n",
		x[spectra_H_idx], x[spectra_X_idx], x[spectra_Z_idx], RAD2DEG(x[optera_tilt_err_X_idx]), RAD2DEG(x[optera_tilt_err_Z_idx]), RAD2DEG(x[spectra_tilt_err_X_idx]), RAD2DEG(x[spectra_tilt_err_Z_idx]), RAD2DEG(x[spectra_pan_offset_idx]), fx, k, ls, step);
	DOUT(debugMsg);

    //// TODO: implement code to terminate if things are going wrong

    pCtx->iter_count = k;

    return 0;
}


//////////////////////////////////////////////////////////////////////////////////////
/**
 * Callback interface to provide objective function and gradient evaluations.
 *
 *  The lbfgs() function call this function to obtain the values of objective
 *  function and its gradients when needed. A client program must implement
 *  this function to evaluate the values of the objective function and its
 *  gradients, given current values of variables.
 *
 *  @param  instance    The user data sent for lbfgs() function by the client.
 *  @param  x           The current values of variables.
 *  @param  g           The gradient vector. The callback function must compute
 *                      the gradient values for the current variables.
 *  @param  n           The number of variables.
 *  @param  step        The current step of the line search routine.
 *  @retval lbfgsfloatval_t The value of the objective function for the current
 *                          variables.
 */

lbfgsfloatval_t evaluate(void * context, const lbfgsfloatval_t* x, lbfgsfloatval_t* g, const int n,
                const lbfgsfloatval_t step) {

    (void)step;
    int ii;
	double err1, err2;

    lbfgs_context* pCtx = (lbfgs_context *)context;

	lbfgsfloatval_t in_var[NUM_VARS];
    memcpy(in_var, x, sizeof(in_var));

    // For now numerically approximate the gradient
    for (ii = 0; ii < n; ii++) {
        double e = pCtx->delta[ii]/2;
        double orig = in_var[ii];
        in_var[ii] = orig - e;
        err1 = errorVal(pCtx, &in_var[0], 0, NULL);
        in_var[ii] = orig + e;
        err2 = errorVal(pCtx, &in_var[0], 0, NULL);
        in_var[ii] = orig;  // restore original value

        g[ii] = (err2 - err1)/(2 * e);

        // To keep us from changing a variable that we want to be fixed, force
        // that gradient to be 0
        if (pCtx->vary[ii] == false) g[ii] = 0.0;
    }

    return errorVal(pCtx, &in_var[0], 0, NULL);
}

////////////////////////////////////////////////////////////////////////////////////////
//// use scaling, translation and rotation to align sensors to a fixed reference sensor
//// given control point pairs
int numeric_calibrate(pointpair_t*					 point_pair,       // (I)   point pair inputs
					  int							 num_point_pairs,  // (I)   # of point pairs
					  CameraInfos::OpteraCameraTypes optera_type,      // (I)   CAMERA_180/270/360
					  bool                           separated_camera, // (I)   optera separated from spectra
					  bool                           mutually_visible, // (I)   optera and spectra can see each other
					  float							 optera_h,         // (I)   height of optera (feet)
					  vars_t*						 var,              // (I/O) unknown vars to be solved for
					  int*							 num_iter,         // (O)   # of iterations solver used
					  double*						 final_err_feet) { // (O)   final total error in point pair matching (feet)

    lbfgs_context  context;

    int ii;
	lbfgsfloatval_t in_var[NUM_VARS];
	
    // Set the initial parameters
	in_var[spectra_H_idx]			= var->spectra_H;
	in_var[spectra_X_idx]			= var->spectra_X;
	in_var[spectra_Z_idx]			= var->spectra_Z;
	in_var[optera_tilt_err_X_idx]	= var->optera_tilt_err_X;
	in_var[optera_tilt_err_Z_idx]	= var->optera_tilt_err_Z;
	in_var[spectra_tilt_err_X_idx] = var->spectra_tilt_err_X;
	in_var[spectra_tilt_err_Z_idx] = var->spectra_tilt_err_Z;
	in_var[spectra_pan_offset_idx] = var->spectra_pan_offset;

	// initialize context parameters
	initialize_context_params(optera_type, separated_camera, mutually_visible);

	// load context parameters
    memcpy(context.point_pair, point_pair, sizeof(context.point_pair));
    memcpy(context.delta, def_delta, sizeof(context.delta));
    memcpy(context.vary, def_vary_mask, sizeof(context.vary));

	context.optera_H = optera_h;
	context.optera_pan_offset = 0.0f;
	context.num_point_pairs = num_point_pairs;
	context.optera_type = optera_type;
	context.iter_count = 0;

    for (ii = 0; ii < NUM_VARS; ii++) {
        context.upper_bound[ii] = in_var[ii] + bound_radius[ii];
        context.lower_bound[ii] = in_var[ii] - bound_radius[ii];
    }

	// setup the lbfgs options
    lbfgs_parameter_t param;
    lbfgs_parameter_init(&param);
    param.linesearch = LBFGS_LINESEARCH_BACKTRACKING_STRONG_WOLFE;
	param.xtol = 1.0e-12;
	param.ftol = 1.0e-12;
	param.gtol = 0.9;
	param.epsilon = 1e-5; //1e-10
	param.delta = 1e-8;  // if (f' - f)/f < delta then terminate ie: not changing much.
    param.past = 0; // number of iterations in the past for the delta comparison (ie: if 0 then delta not used)
    param.min_step = 1e-20;
    param.max_iterations = 10000;
	param.max_linesearch = 1000;

    int ret = lbfgs(NUM_VARS, &in_var[0], NULL, evaluate, progress, &context, &param);

    // Even if there is an error compute the statistics for the current values
	sprintf_s(debugMsg, sizeof(debugMsg), "numeric_calibrate: spectra_height: %6.3f spectra_X: %7.3f spectra_Z: %7.3f optera_tilt_err_X: %6.3f optera_tilt_err_Z: %6.3f spectra_tilt_err_X: %6.3f spectra_tilt_err_Z: %6.3f spectra_pan_offset: %6.3f\n",
		in_var[spectra_H_idx], in_var[spectra_X_idx], in_var[spectra_Z_idx], RAD2DEG(in_var[optera_tilt_err_X_idx]), RAD2DEG(in_var[optera_tilt_err_Z_idx]), RAD2DEG(in_var[spectra_tilt_err_X_idx]), RAD2DEG(in_var[spectra_tilt_err_Z_idx]), RAD2DEG(in_var[spectra_pan_offset_idx]));
	DOUT(debugMsg);

    // copy over the results
	var->spectra_H          = (double)in_var[spectra_H_idx];            
	var->spectra_X          = (double)in_var[spectra_X_idx];
	var->spectra_Z          = (double)in_var[spectra_Z_idx];
	var->optera_tilt_err_X  = (double)in_var[optera_tilt_err_X_idx];
	var->optera_tilt_err_Z  = (double)in_var[optera_tilt_err_Z_idx];
	var->spectra_tilt_err_X = (double)in_var[spectra_tilt_err_X_idx];
	var->spectra_tilt_err_Z = (double)in_var[spectra_tilt_err_Z_idx];
	var->spectra_pan_offset = (double)in_var[spectra_pan_offset_idx];

    // print out some information about the errors
    double max_err;
    double result = errorVal(&context, &in_var[0], 1, &max_err);
	sprintf_s(debugMsg, sizeof(debugMsg), "numeric_calibrate: max individual point err = %6.3f feet, combined points error = %6.3f lbfgs retval = %d\n", max_err, result, ret);
	DOUT(debugMsg);

	if (final_err_feet) { *final_err_feet = result; }
    if (num_iter)       { *num_iter = context.iter_count; }

    // Filter out some of the underlying errors that may still have given us a good result.
    if ((ret == LBFGSERR_ROUNDING_ERROR) ||
        (ret == LBFGSERR_MINIMUMSTEP) ||
        (ret == LBFGSERR_MAXIMUMLINESEARCH)) {
        ret = 0;
    }

    return ret;
}

////////////////////////////////////////////////////////////////////////////////////////
// Perform a rotation transform from camera point to mount point
cart_3d_point_t rotateCameraToMount(cart_3d_point_t input_pnt, double rot_X, double rot_Y, double rot_Z) {

	cart_3d_point_t output_pnt;

	// T_XZY matrix coef equations
	double T11 = cos(rot_Z) * cos(rot_Y);
	double T12 = -cos(rot_Z) * sin(rot_Y);
	double T13 = sin(rot_Z);

	double T21 = cos(rot_X) * sin(rot_Y) + sin(rot_X) * sin(rot_Z) * cos(rot_Y);
	double T22 = cos(rot_X) * cos(rot_Y) - sin(rot_X) * sin(rot_Z) * sin(rot_Y);
	double T23 = -sin(rot_X) * cos(rot_Z);

	double T31 = sin(rot_X) * sin(rot_Y) - cos(rot_X) * sin(rot_Z) * cos(rot_Y);
	double T32 = sin(rot_X) * cos(rot_Y) + cos(rot_X) * sin(rot_Z) * sin(rot_Y);
	double T33 = cos(rot_X) * cos(rot_Z);

	output_pnt.X = T11 * input_pnt.X + T12 * input_pnt.Z + T13 * input_pnt.Y;
	output_pnt.Z = T21 * input_pnt.X + T22 * input_pnt.Z + T23 * input_pnt.Y;
	output_pnt.Y = T31 * input_pnt.X + T32 * input_pnt.Z + T33 * input_pnt.Y;

	return output_pnt;
}

////////////////////////////////////////////////////////////////////////////////////////
// Perform a rotation transform from mount point to camera point
cart_3d_point_t rotateMountToCamera(cart_3d_point_t input_pnt, double rot_X, double rot_Y, double rot_Z) {

	cart_3d_point_t output_pnt;

	// Inverse T_XZY matrix coef equations
	double T11 = cos(rot_Y) * cos(rot_Z);
	double T12 = cos(rot_Y) * sin(rot_Z) * sin(rot_X) + sin(rot_Y) * cos(rot_X);
	double T13 = -cos(rot_Y) * sin(rot_Z) * cos(rot_X) + sin(rot_Y) * sin(rot_X);

	double T21 = -sin(rot_Y) * cos(rot_Z);
	double T22 = -sin(rot_Y) * sin(rot_Z) * sin(rot_X) + cos(rot_Y) * cos(rot_X);
	double T23 = sin(rot_Y) * sin(rot_Z) * cos(rot_X) + cos(rot_Y) * sin(rot_X);

	double T31 = sin(rot_Z);
	double T32 = -cos(rot_Z) * sin(rot_X);
	double T33 = cos(rot_Z) * cos(rot_X);

	output_pnt.X = T11 * input_pnt.X + T12 * input_pnt.Z + T13 * input_pnt.Y;
	output_pnt.Z = T21 * input_pnt.X + T22 * input_pnt.Z + T23 * input_pnt.Y;
	output_pnt.Y = T31 * input_pnt.X + T32 * input_pnt.Z + T33 * input_pnt.Y;

	return output_pnt;
}

////////////////////////////////////////////////////////////////////////////////////////
//
void test_bench() {

#define NUM_POINT_PAIRS 4	
#define NUM_ITERATIONS 100

	CString str;
	float deg2rad = (float)(PI / 180.0);
	float rad2deg = (float)(180.0 / PI);
	static bool initialized = false;
	if (!initialized) {
		srand((unsigned int)(std::time(NULL)));
		initialized = true;
	}

	// init statics params
	float spectra_H_min = FLT_MAX, spectra_X_min = FLT_MAX, spectra_Z_min = FLT_MAX, optera_tilt_err_X_min = FLT_MAX, optera_tilt_err_Z_min = FLT_MAX;
	float spectra_tilt_err_X_min = FLT_MAX, spectra_tilt_err_Z_min = FLT_MAX, final_err_feet_min = FLT_MAX;
	float spectra_H_max = -99999, spectra_X_max = -99999, spectra_Z_max = -99999, optera_tilt_err_X_max = -99999, optera_tilt_err_Z_max = -99999;
	float spectra_tilt_err_X_max = -99999, spectra_tilt_err_Z_max = -99999, final_err_feet_max = -99999;
	float spectra_H_accum = 0, spectra_X_accum = 0, spectra_Z_accum = 0, optera_tilt_err_X_accum = 0, optera_tilt_err_Z_accum = 0;
	float spectra_tilt_err_X_accum = 0, spectra_tilt_err_Z_accum = 0, final_err_feet_accum = 0;
	int num_iter_min = INT_MAX, num_iter_max = 0;
	INT64 num_iter_accum = 0;

	for (int iter = 0; iter < NUM_ITERATIONS; iter++) {

		// pick random set of target points
		float target_range_min = -200, target_range_max = 200;  // feet
		float target_x[NUM_POINT_PAIRS], target_z[NUM_POINT_PAIRS];
		for (int i = 0; i < NUM_POINT_PAIRS; i++) {
			target_x[i] = (float)(((rand() / double(RAND_MAX)) * (target_range_max - target_range_min)) + target_range_min);
			target_z[i] = (float)(((rand() / double(RAND_MAX)) * (target_range_max - target_range_min)) + target_range_min);
			sprintf_s(debugMsg, sizeof(debugMsg), "Target Points: target_x[%d]: %9.4f, target_z[%d]: %9.4f\n",
				i, target_x[i], i, target_z[i]);
			DOUT(debugMsg);
		}

		// Run the target points through the system model to get corresponding camera pan/tilt angles.
		// system model params:
		modelParameters model_params;
		model_params.optera_H = 13;
		model_params.optera_X = 0;
		model_params.optera_Z = 0;
		model_params.spectra_H = 19;
		model_params.spectra_X = -86.0987f;
		model_params.spectra_Z = 42.274f;
		model_params.optera_tilt_err_X = 0.63848f * deg2rad;
		model_params.optera_tilt_err_Z = 1.5274f * deg2rad;
		model_params.spectra_tilt_err_X = -2.3753f * deg2rad;
		model_params.spectra_tilt_err_Z = 1.85f * deg2rad;
		pointpair_t point_pair[NUM_POINT_PAIRS];
		for (int i = 0; i < NUM_POINT_PAIRS; i++) {
			system_model(model_params, target_x[i], target_z[i], point_pair[i]);
			sprintf_s(debugMsg, sizeof(debugMsg), "System Model Point Pairs: optera_pan, tilt[%d]: %8.5f, %8.5f spectra_pan/tilt[%d]: %8.5f, %8.5f\n",
				i, point_pair[i].optera_pan, point_pair[i].optera_tilt, i, point_pair[i].spectra_pan, point_pair[i].spectra_tilt);
			DOUT(debugMsg);
		}

		// initialize system params to default
		vars_t var;
		float optera_h = 13;
		var.spectra_H = optera_h;
		var.spectra_X = 0;
		var.spectra_Z = 0;
		var.optera_tilt_err_X = 0;
		var.optera_tilt_err_Z = 0;
		var.spectra_tilt_err_X = 0;
		var.spectra_tilt_err_Z = 0;

		// solve for unknown variables
		int num_iter;
		double final_err_feet = 0;
		int status = numeric_calibrate(point_pair,               // (I)   point pair inputs
									   NUM_POINT_PAIRS,          // (I)   # of point pairs
									   CameraInfos::CAMERA_360,  // (I)   CAMERA_180/270/360
									   true,                     // (I)   optera separated from spectra
									   true,                     // (I)   optera and spectra can see each other
									   optera_h,                 // (I)   height of optera (feet)
									   &var,                     // (I/O) unknown vars to be solved for
									   &num_iter,                // (O)   # of iterations solver used
									   &final_err_feet);         // (O)   final total error in point pair matching (feet)

		sprintf_s(debugMsg, sizeof(debugMsg), "Results: spectra_H: %9.5f spectra_X: %9.5f spectra_Z: %9.5f optera_tilt_err_X: %8.5f optera_tilt_err_Z: %8.5f spectra_tilt_err_X: %8.5f spectra_tilt_err_Z: %8.5f num_iter: %4d status: %4d final_err_feet: %8.5f\n",
			var.spectra_H, var.spectra_X, var.spectra_Z, RAD2DEG(var.optera_tilt_err_X), RAD2DEG(var.optera_tilt_err_Z), RAD2DEG(var.spectra_tilt_err_X), RAD2DEG(var.spectra_tilt_err_Z), num_iter, status, final_err_feet);
		DOUT(debugMsg);

		// update min's
		if (var.spectra_H < spectra_H_min) { spectra_H_min = (float)var.spectra_H; }
		if (var.spectra_X < spectra_X_min) { spectra_X_min = (float)var.spectra_X; }
		if (var.spectra_Z < spectra_Z_min) { spectra_Z_min = (float)var.spectra_Z; }
		if (var.optera_tilt_err_X < optera_tilt_err_X_min) { optera_tilt_err_X_min = (float)var.optera_tilt_err_X; }
		if (var.optera_tilt_err_Z < optera_tilt_err_Z_min) { optera_tilt_err_Z_min = (float)var.optera_tilt_err_Z; }
		if (var.spectra_tilt_err_X < spectra_tilt_err_X_min) { spectra_tilt_err_X_min = (float)var.spectra_tilt_err_X; }
		if (var.spectra_tilt_err_Z < spectra_tilt_err_Z_min) { spectra_tilt_err_Z_min = (float)var.spectra_tilt_err_Z; }
		if (num_iter < num_iter_min) { num_iter_min = num_iter; }
		if (final_err_feet < final_err_feet_min) { final_err_feet_min = (float)final_err_feet; }

		// update max's
		if (var.spectra_H > spectra_H_max) { spectra_H_max = (float)var.spectra_H; }
		if (var.spectra_X > spectra_X_max) { spectra_X_max = (float)var.spectra_X; }
		if (var.spectra_Z > spectra_Z_max) { spectra_Z_max = (float)var.spectra_Z; }
		if (var.optera_tilt_err_X > optera_tilt_err_X_max) { optera_tilt_err_X_max = (float)var.optera_tilt_err_X; }
		if (var.optera_tilt_err_Z > optera_tilt_err_Z_max) { optera_tilt_err_Z_max = (float)var.optera_tilt_err_Z; }
		if (var.spectra_tilt_err_X > spectra_tilt_err_X_max) { spectra_tilt_err_X_max = (float)var.spectra_tilt_err_X; }
		if (var.spectra_tilt_err_Z > spectra_tilt_err_Z_max) { spectra_tilt_err_Z_max = (float)var.spectra_tilt_err_Z; }
		if (num_iter > num_iter_max) { num_iter_max = num_iter; }
		if (final_err_feet > final_err_feet_max) { final_err_feet_max = (float)final_err_feet; }

		// update accum's
		spectra_H_accum += (float)var.spectra_H;
		spectra_X_accum += (float)var.spectra_X;
		spectra_Z_accum += (float)var.spectra_Z;
		optera_tilt_err_X_accum += (float)var.optera_tilt_err_X;
		optera_tilt_err_Z_accum += (float)var.optera_tilt_err_Z;
		spectra_tilt_err_X_accum += (float)var.spectra_tilt_err_X;
		spectra_tilt_err_Z_accum += (float)var.spectra_tilt_err_Z;
		num_iter_accum += num_iter;
		final_err_feet_accum += (float)final_err_feet;
	}
	// display stat's
	sprintf_s(debugMsg, sizeof(debugMsg), "  Min's: spectra_H: %9.5f spectra_X: %9.5f spectra_Z: %9.5f optera_tilt_err_X: %8.5f optera_tilt_err_Z: %8.5f spectra_tilt_err_X: %8.5f spectra_tilt_err_Z: %8.5f num_iter: %4d status: xxxx final_err_feet: %8.5f\n",
		spectra_H_min, spectra_X_min, spectra_Z_min, RAD2DEG(optera_tilt_err_X_min), RAD2DEG(optera_tilt_err_Z_min), RAD2DEG(spectra_tilt_err_X_min), RAD2DEG(spectra_tilt_err_Z_min), num_iter_min, final_err_feet_min);
	DOUT(debugMsg);
	sprintf_s(debugMsg, sizeof(debugMsg), "  Max's: spectra_H: %9.5f spectra_X: %9.5f spectra_Z: %9.5f optera_tilt_err_X: %8.5f optera_tilt_err_Z: %8.5f spectra_tilt_err_X: %8.5f spectra_tilt_err_Z: %8.5f num_iter: %4d status: xxxx final_err_feet: %8.5f\n",
		spectra_H_max, spectra_X_max, spectra_Z_max, RAD2DEG(optera_tilt_err_X_max), RAD2DEG(optera_tilt_err_Z_max), RAD2DEG(spectra_tilt_err_X_max), RAD2DEG(spectra_tilt_err_Z_max), num_iter_max, final_err_feet_max);
	DOUT(debugMsg);
	sprintf_s(debugMsg, sizeof(debugMsg), "  Ave's: spectra_H: %9.5f spectra_X: %9.5f spectra_Z: %9.5f optera_tilt_err_X: %8.5f optera_tilt_err_Z: %8.5f spectra_tilt_err_X: %8.5f spectra_tilt_err_Z: %8.5f num_iter: %4d status: xxxx final_err_feet: %8.5f\n",
		spectra_H_accum / NUM_ITERATIONS, spectra_X_accum / NUM_ITERATIONS, spectra_Z_accum / NUM_ITERATIONS, RAD2DEG(optera_tilt_err_X_accum / NUM_ITERATIONS), RAD2DEG(optera_tilt_err_Z_accum / NUM_ITERATIONS), RAD2DEG(spectra_tilt_err_X_accum / NUM_ITERATIONS), RAD2DEG(spectra_tilt_err_Z_accum / NUM_ITERATIONS), num_iter_accum / NUM_ITERATIONS, final_err_feet_accum / NUM_ITERATIONS);
	DOUT(debugMsg);
}

////////////////////////////////////////////////////////////////////////////////////////
// Given a target point and the system params, this function will generate the pan/tilt angles
// for each camera to that point.
void system_model(modelParameters model_params, float target_x, float target_z, pointpair_t &results) {

	// set model parameters
	float optera_h = model_params.optera_H;
	float optera_x = model_params.optera_X;
	float optera_z = model_params.optera_Z;
	float spectra_h = model_params.spectra_H;
	float spectra_x = model_params.spectra_X;
	float spectra_z = model_params.spectra_Z;
	float optera_tilt_err_x = model_params.optera_tilt_err_X;
	float optera_tilt_err_z = model_params.optera_tilt_err_Z;
	float spectra_tilt_err_x = model_params.spectra_tilt_err_X;
	float spectra_tilt_err_z = model_params.spectra_tilt_err_Z;

	// Calculate from Target X, Z to Optera Pan / Tilt radians
	float optera_pan = atan2((target_z - optera_z), (target_x - optera_x));
	float diff_x = target_x - optera_x;
	float diff_z = target_z - optera_z;
	float optera_target_base = sqrt((diff_x * diff_x) + (diff_z * diff_z));
	float optera_tilt = atan(optera_h / optera_target_base);
	float opteraTiltError = atan((tan(optera_tilt_err_z) * cos(-optera_pan)) + (tan(optera_tilt_err_x) * sin(-optera_pan)));
	optera_tilt = -(optera_tilt - opteraTiltError);

	// Calculate from Target X, Z to Spectra Pan / Tilt radians
	float spectra_pan = atan2((target_z - spectra_z), (target_x - spectra_x));
	diff_x = target_x - spectra_x;
	diff_z = target_z - spectra_z;
	float spectra_target_base = sqrt((diff_x * diff_x) + (diff_z * diff_z));
	float spectra_tilt = atan(spectra_h / spectra_target_base);
	float spectraTiltError = atan((tan(spectra_tilt_err_z) * cos(-(spectra_pan))) + (tan(spectra_tilt_err_x) * sin(-(spectra_pan))));
	spectra_tilt = -(spectra_tilt - spectraTiltError);

	// Fill out results
	results.optera_pan = optera_pan;
	results.optera_tilt = optera_tilt;
	results.spectra_pan = spectra_pan;
	results.spectra_tilt = spectra_tilt;
	return;
}