#include "zenith/includes.h"

// Invert a 3x3 matrix
void inverse(const float in[3][3], float out[3][3]) {
    float det = in[0][0] * (in[1][1] * in[2][2] - in[2][1] * in[1][2]) -
                in[0][1] * (in[1][0] * in[2][2] - in[1][2] * in[2][0]) +
                in[0][2] * (in[1][0] * in[2][1] - in[1][1] * in[2][0]);
    float invdet = 1 / det;

    out[0][0] = (in[1][1] * in[2][2] - in[2][1] * in[1][2]) * invdet;
    out[0][1] = (in[0][2] * in[2][1] - in[0][1] * in[2][2]) * invdet;
    out[0][2] = (in[0][1] * in[1][2] - in[0][2] * in[1][1]) * invdet;
    out[1][0] = (in[1][2] * in[2][0] - in[1][0] * in[2][2]) * invdet;
    out[1][1] = (in[0][0] * in[2][2] - in[0][2] * in[2][0]) * invdet;
    out[1][2] = (in[1][0] * in[0][2] - in[0][0] * in[1][2]) * invdet;
    out[2][0] = (in[1][0] * in[2][1] - in[2][0] * in[1][1]) * invdet;
    out[2][1] = (in[2][0] * in[0][1] - in[0][0] * in[2][1]) * invdet;
    out[2][2] = (in[0][0] * in[1][1] - in[1][0] * in[0][1]) * invdet;
}

void notch_snap(const ax_t x_in, const ax_t y_in, ax_t *x_out, ax_t *y_out,
                const calib_results_t *calib_results,
                const stick_config_t *stick_config, int region, float angle) {
    float boundary_angle = calib_results->boundary_angles[region];
    // TODO: probably plenty of room for math optimization here
    // not sure if this method is significantly slowing things down
    float mag = sqrtf(x_in * x_in + y_in * y_in);
    ax_t x_in_ref = calib_results->notch_points_x_in[region];
    ax_t y_in_ref = calib_results->notch_points_y_in[region];
    float mag_ref = sqrtf(x_in_ref * x_in_ref + y_in_ref * y_in_ref);

    // printf("m: %f, mr: %f, mt: %f\n", mag, mag_ref,
    //        stick_config->mag_threshold);

    // Notch snapping:
    // If the angle is within an angle deadzone
    // And the magnitude is over the threshold (don't want near-center
    // values to be snapped)
    // printf("ba: %f; a: %f; r: %d\n", boundary_angle, angle, region);
    if (mag >= (stick_config->mag_threshold * mag_ref)) {
        int next_region = (region == (NUM_NOTCHES - 1)) ? 0 : (region + 1);
        if (region == 0) {
            float last_boundary_angle =
                calib_results->boundary_angles[NUM_NOTCHES - 1];
            if ((boundary_angle - angle + 2 * M_PI) <
                stick_config->angle_deadzones[next_region]) {
                *x_out = mag * cosf(boundary_angle);
                *y_out = mag * sinf(boundary_angle);
            } else if ((angle - last_boundary_angle) <
                       stick_config->angle_deadzones[region]) {
                *x_out = mag * cosf(last_boundary_angle);
                *y_out = mag * sinf(last_boundary_angle);
            }
        } else {
            float last_boundary_angle =
                calib_results->boundary_angles[region - 1];
            if ((boundary_angle - angle) <
                stick_config->angle_deadzones[next_region]) {
                *x_out = mag * cosf(boundary_angle);
                *y_out = mag * sinf(boundary_angle);
            } else if ((angle - last_boundary_angle) <
                       stick_config->angle_deadzones[region]) {
                *x_out = mag * cosf(last_boundary_angle);
                *y_out = mag * sinf(last_boundary_angle);
            }
        }
    }
}

void notch_remap(const ax_t x_in, const ax_t y_in, ax_t *x_out, ax_t *y_out,  
                 const bool gate_limiter_enable,
                 const calib_results_t *calib_results,
                 const stick_config_t *stick_config) {
    // determine the angle between the x unit vector and the current position
    // vector
    float angle = atan2f(y_in, x_in);

    // unwrap the angle based on the first region boundary
    if (angle < calib_results->boundary_angles[0]) {
        angle += M_PI * 2;
    }

    // Start at the last region. Note that the corresponding boundary angle for
    // this is 0. So look at all the other boundary angles in counterclockwise
    // fashion. The first one that is greater than our angle is +1 from the
    // region we're looking in, so save region and break.
    // Otherwise, if none of them hit, we're in the last region.
    int region = 0;
    for (int i = 0; i < NUM_NOTCHES; i++) {
        float boundary_angle = calib_results->boundary_angles[i];
        if (angle < boundary_angle) {
            region = i;
            break;
        }
    }

    ax_t x_in_adj = x_in;
    ax_t y_in_adj = y_in;
    // uint32_t thing = time_us_32();
    notch_snap(x_in, y_in, &x_in_adj, &y_in_adj, calib_results, stick_config,
               region, angle);
    // debug_print("time: %d\n", time_us_32() - thing);



    // Apply the affine transformation using the coefficients found during
    // calibration
    // Note the lack of translation. This is due to assuming center = 0.
    *x_out = calib_results->affine_coeffs[region][0] * x_in_adj +
             calib_results->affine_coeffs[region][1] * y_in_adj;
    *y_out = calib_results->affine_coeffs[region][2] * x_in_adj +
             calib_results->affine_coeffs[region][3] * y_in_adj;

    if (gate_limiter_enable) {
        int next_region = (region == (NUM_NOTCHES - 1)) ? 0 : (region + 1);
        float x0 = stick_config->notch_points_x[region];
        float y0 = stick_config->notch_points_y[region];
        float xf = stick_config->notch_points_x[next_region];
        float yf = stick_config->notch_points_y[next_region];
        float minv = (xf-x0)/(yf-y0);
        float xp = (*y_out - y0) * minv + x0;

        if (fabs(*x_out) >= fabs(xp) && signbit(*x_out) == signbit(xp)) {
            *x_out = xp;
        }
    }
}

// Multiply two 3x3 matrices
void matrix_matrix_mult(const float left[3][3], const float right[3][3],
                        float output[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            output[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                output[i][j] += left[i][k] * right[k][j];
            }
        }
    }
}

void print_mtx(const float matrix[3][3]) {
    int i, j, nrow, ncol;
    nrow = 3;
    ncol = 3;
    debug_print("\n");
    for (i = 0; i < nrow; i++) {
        for (j = 0; j < ncol; j++) {
            debug_print("%.6f, ", matrix[i][j]); // print 6 decimal places
        }
        debug_print("\n");
    }
    debug_print("\n");
}

void notch_calibrate(const ax_t in_points_x[], const ax_t in_points_y[],
                     const ax_t notch_points_x[], const ax_t notch_points_y[],
                     calib_results_t *calib_results) {
    // We always assume that the input and output share a common origin of 0.
    //
    // In the former case, it's because we corrected the center to it in the
    // linearization stage.
    //
    // In the latter case, it's because.. why would you
    // want your controller to be uncentered?

    // Loop through every region
    for (int cur = 0; cur < NUM_NOTCHES; cur++) {
        // debug_print("calibration region %d\n", i);

        float pointsIn[3][3];
        float pointsOut[3][3];

        // wrap around for last region
        int next = (cur + 1) % NUM_NOTCHES;

        pointsIn[0][0] = 0;
        pointsIn[0][1] = in_points_x[cur];
        pointsIn[0][2] = in_points_x[next];
        pointsIn[1][0] = 0;
        pointsIn[1][1] = in_points_y[cur];
        pointsIn[1][2] = in_points_y[next];
        pointsIn[2][0] = 1;
        pointsIn[2][1] = 1;
        pointsIn[2][2] = 1;
        pointsOut[0][0] = 0;
        pointsOut[0][1] = notch_points_x[cur];
        pointsOut[0][2] = notch_points_x[next];
        pointsOut[1][0] = 0;
        pointsOut[1][1] = notch_points_y[cur];
        pointsOut[1][2] = notch_points_y[next];
        pointsOut[2][0] = 1;
        pointsOut[2][1] = 1;
        pointsOut[2][2] = 1;

        float temp[3][3];
        inverse(pointsIn, temp);

        float A[3][3];
        matrix_matrix_mult(pointsOut, temp, A);

        // debug_print("The transform matrix is:\n");
        // print_mtx(A);

        // Because we have assumed the two point regions to share 0 as the
        // center, we will not save the elements of the matrix representing
        // translation (A[0][2] and A[1][2].) Only the top 2x2 will be saved.
        // Recall this is homogenous coordinates so we do not care about the
        // bottom row.

        calib_results->affine_coeffs[cur][0] = A[0][0];
        calib_results->affine_coeffs[cur][1] = A[0][1];
        calib_results->affine_coeffs[cur][2] = A[1][0];
        calib_results->affine_coeffs[cur][3] = A[1][1];

        // The boundary angle is the angle of the "next" line,
        // going counterclockwise around the stick gate
        calib_results->boundary_angles[cur] =
            atan2f(in_points_y[next], in_points_x[next]);
        // unwrap the angles so that the first has the smallest value
        // It's OK if this results in an angle > 2*pi. We are going
        // around the stick in a defined direction when we do the notch remap,
        // so we'll never accidentally trip on a greater angle that does not
        // wrap around.
        if (calib_results->boundary_angles[cur] <
            calib_results->boundary_angles[0]) {
            calib_results->boundary_angles[cur] += M_PI * 2;
        }
        debug_print("Boundary angle for region %d: %f\n", cur,
                    calib_results->boundary_angles[cur]);
    }
}