#include "linearize.h"

ax_t linearize(const ax_t point, const float coefficients[]) {
    return (coefficients[0] * (point * point * point) +
            coefficients[1] * (point * point) + coefficients[2] * point +
            coefficients[3]);
}

void linearize_cal(const ax_t cleaned_points_x[], const ax_t cleaned_points_y[],
                   ax_t linearized_points_x[], ax_t linearized_points_y[],
                   calib_results_t *calib_results) {

    // for readability
    const ax_t *in_x = cleaned_points_x;
    const ax_t *in_y = cleaned_points_y;

    ax_t *out_x = linearized_points_x;
    ax_t *out_y = linearized_points_y;

    double fit_points_x[5];
    double fit_points_y[5];

    fit_points_x[0] = (double)in_x[5];                   // left
    fit_points_x[1] = (double)(in_x[4] + in_x[6]) / 2.0; // left diagonal
    fit_points_x[2] = (double)in_x[0];                   // center
    fit_points_x[3] = (double)(in_x[2] + in_x[8]) / 2.0; // right diagonal
    fit_points_x[4] = (double)in_x[1];                   // right

    fit_points_y[0] = (double)in_y[7];                   // down
    fit_points_y[1] = (double)(in_y[6] + in_y[8]) / 2.0; // down diagonal
    fit_points_y[2] = (double)in_y[0];                   // center
    fit_points_y[3] = (double)(in_y[2] + in_y[4]) / 2.0; // up diagonal
    fit_points_y[4] = (double)in_y[3];                   // up

    for (int i = 0; i < 5; i++) {
        al_debug_print("Fit point %d; (x,y) = (%f, %f)\n", i, fit_points_x[i],
                    fit_points_y[i]);
    }

    double temp_coeffs_x[FIT_ORDER + 1];
    double temp_coeffs_y[FIT_ORDER + 1];

    polyfit(5, fit_points_x, linearize_reference, FIT_ORDER + 1, temp_coeffs_x);
    polyfit(5, fit_points_y, linearize_reference, FIT_ORDER + 1, temp_coeffs_y);

    // Save these coefficients in the calibration results structure
    // Convert down to float in the process
    for (int i = 0; i < (FIT_ORDER + 1); i++) {
        calib_results->fit_coeffs_x[i] = (float)temp_coeffs_x[i];
        calib_results->fit_coeffs_y[i] = (float)temp_coeffs_y[i];
    }

    // Now we will adjust these coordinates to account for error.
    // Curve fitting is not a perfect process i.e. the data we put in is not
    // exactly a cubic.
    // Thus, there will be a small amount of error (R^2 =/= 1)
    // such that when we put in one of our fit points, the data we get out of
    // the fit curve is not the same as the corresponding output fit point.
    //
    // To account for this, we will adjust the constant coefficient (the d in
    // ax^3+bx^2+cx+d) such that we get 0 when we run the curve fit on our
    // center fit points. This doesn't fix the other points, but that's OK.
    //
    // Why do it then? Because when we do notch adjust, we will operate
    // under the assumption that the center is 0. That allows us to skip
    // having to save the translation coordinate to adjust it to 0 in flash.
    calib_results->fit_coeffs_x[3] -=
        linearize(fit_points_x[2], calib_results->fit_coeffs_x);
    calib_results->fit_coeffs_y[3] -=
        linearize(fit_points_y[2], calib_results->fit_coeffs_y);

    // Linearize the calibration points we got in, but we skip in[0],
    // because again, from this point onward, there is an
    // assumption that 0 = center.
    for (int i = 1; i <= NUM_NOTCHES; i++) {
        out_x[i - 1] = linearize(in_x[i], calib_results->fit_coeffs_x);
        out_y[i - 1] = linearize(in_y[i], calib_results->fit_coeffs_y);
    }

    al_debug_print("Center point (confirmation): (x,y) = (%f,%f)\n",
                linearize(in_x[0], calib_results->fit_coeffs_x),
                linearize(in_y[0], calib_results->fit_coeffs_y));
}

// POLYFIT CODE FROM FOLLOWING LIBRARY:

// Description: Simple polynomial fitting functions.
// Author: Henry M. Forson, Melbourne, Florida USA
// Simplified to go in Zenith codebase

//------------------------------------------------------------------------------------
// MIT License
//
// Copyright (c) 2020 Henry M. Forson
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//------------------------------------------------------------------------------------

// Define this to print matrix computation results to terminal (probably not
// very useful)
// #define MATRIX_DEBUGGING

#ifdef MATRIX_DEBUGGING

#include <stdio.h> // printf()

void reallyShowMatrix(double mat[], int rows, int cols) {
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            printf("   %f", mat[r * cols + c]);
        }
        printf("\n");
    }
}

#define showMatrix(x, r, c)                                                    \
    do {                                                                       \
        printf("   @%d: " #x " =\n", __LINE__);                                \
        reallyShowMatrix(x, r, c);                                             \
        printf("\n");                                                          \
    } while (0)
#define polyfit_dbg(fmt, args...) printf(fmt, ##args)
#else // MATRIX_DEBUGGING
#define showMatrix(x, r, c)
#define polyfit_dbg(fmt, args...)
#endif // MATRIX_DEBUGGING

int polyfit(int pointCount, const double *xValues, const double *yValues,
            int coefficientCount, double *coefficientResults) {
    int rVal = 0;
    int degree = coefficientCount - 1;

    // Check that pointCount >= coefficientCount.
    if (pointCount < coefficientCount) {
        return -2;
    }

    // Make the A matrix:
    double pMatA[pointCount][coefficientCount];

    for (int r = 0; r < pointCount; r++) {
        double acc = 1;
        for (int c = 0; c < coefficientCount; c++) {
            // xValues[r] ^ c
            pMatA[r][degree - c] = acc;
            acc *= xValues[r];
        }
    }

    showMatrix(pMatA, pointCount, coefficientCount);

    const double *pMatB = yValues; // 1 col, pointCount rows

    // Make the transpose of matrix A:
    double pMatAT[coefficientCount][pointCount];

    for (int r = 0; r < coefficientCount; r++) {
        for (int c = 0; c < pointCount; c++) {
            pMatAT[r][c] = pMatA[c][r];
        }
    }

    showMatrix(pMatAT, coefficientCount, pointCount);

    // Make the product of matrices AT and A:
    double pMatATA[coefficientCount][coefficientCount];
    for (int i = 0; i < coefficientCount; i++) {
        for (int j = 0; j < coefficientCount; j++) {
            pMatATA[i][j] = 0.0;
            for (int k = 0; k < pointCount; k++) {
                pMatATA[i][j] += pMatAT[i][k] * pMatA[k][j];
            }
        }
    }

    showMatrix(pMatATA, coefficientCount, coefficientCount);

    // Make the product of matrices AT and b:
    double pMatATB[pointCount];

    for (int i = 0; i < coefficientCount; i++) {
        pMatATB[i] = 0.0;
        for (int k = 0; k < pointCount; k++) {
            pMatATB[i] += pMatAT[i][k] * pMatB[k];
        }
    }

    showMatrix(pMatATB, pointCount, 1);

    // Now we need to solve the system of linear equations,
    // (AT)Ax = (AT)b for "x", the coefficients of the polynomial.

    for (int c = 0; c < coefficientCount; c++) {
        int pr = c; // pr is the pivot row.
        double prVal = pMatATA[pr][c];
        // If it's zero, we can't solve the equations.
        if (0.0 == prVal) {
            polyfit_dbg("Unable to solve equations, pr = %d, c = %d.\n", pr, c);
            showMatrix(pMatATA, coefficientCount, coefficientCount);
            rVal = -4;
            break;
        }
        for (int r = 0; r < coefficientCount; r++) {
            if (r != pr) {
                double targetRowVal = pMatATA[r][c];
                double factor = targetRowVal / prVal;
                for (int c2 = 0; c2 < coefficientCount; c2++) {
                    pMatATA[r][c2] -= pMatATA[pr][c2] * factor;
                    polyfit_dbg("c = %d, pr = %d, r = %d, c2=%d, targetRowVal "
                                "= %f, prVal = %f, factor = %f.\n",
                                c, pr, r, c2, targetRowVal, prVal, factor);
                    showMatrix(pMatATA, coefficientCount, coefficientCount);
                }
                pMatATB[r] -= pMatATB[pr] * factor;

                showMatrix(pMatATB, pointCount, 1);
            }
        }
    }
    for (int c = 0; c < coefficientCount; c++) {
        int pr = c;
        // now, pr is the pivot row.
        double prVal = pMatATA[pr][c];
        pMatATA[pr][c] /= prVal;
        pMatATB[pr] /= prVal;
    }

    showMatrix(pMatATA, coefficientCount, coefficientCount);

    showMatrix(pMatATB, pointCount, 1);

    for (int i = 0; i < coefficientCount; i++) {
        coefficientResults[i] = pMatATB[i];
    }

    return rVal;
}