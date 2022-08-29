/*
 * Copyright (C) Anton Naruta && Daniel Hoppener
 * MAVLab Delft University of Technology
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file wls_alloc.c
 * @brief This is an active set algorithm for WLS control allocation
 *
 * This algorithm will find the optimal inputs to produce the least error wrt
 * the control objective, taking into account the weighting matrices on the
 * control objective and the control effort.
 *
 * The algorithm is described in:
 * Prioritized Control Allocation for Quadrotors Subject to Saturation -
 * E.J.J. Smeur, D.C. Höppener, C. de Wagter. In IMAV 2017
 *
 * written by Anton Naruta && Daniel Hoppener 2016
 * MAVLab Delft University of Technology
 */

#include "math/lsq_package/common/solveActiveSet.h"
#include "math/lsq_package/common/size_defines.h"
#include <stdio.h>
/*#include "std.h"*/
#include <inttypes.h>
#include <stdbool.h>
#include <math.h>

#include <string.h>
#include <math.h>
#include <float.h>
#include "math/lsq_package/lib/qr_wrapper.h"
#include "math/lsq_package/lib/qr_updates.h"
#include "math/lsq_package/lib/qr_solve/qr_solve.h"
#include "math/lsq_package/lib/qr_solve/r8lib_min.h"
#include "math/lsq_package/lib/sparse_math.h"

// provide loop feedback
#define WLS_VERBOSE FALSE

//#define debug_qr




/**
 * @brief active set algorithm for control allocation
 *
 * Takes the control objective and max and min inputs from pprz and calculates
 * the inputs that will satisfy most of the control objective, subject to the
 * weighting matrices Wv and Wu
 *
 * @param u The control output vector
 * @param v The control objective
 * @param umin The minimum u vector
 * @param umax The maximum u vector
 * @param B The control effectiveness matrix
 * @param n_u Length of u
 * @param n_v Lenght of v
 * @param u_guess Initial value for u
 * @param W_init Initial working set, if known
 * @param Wv Weighting on different control objectives
 * @param Wu Weighting on different controls
 * @param up Preferred control vector
 * @param gamma_sq Preference of satisfying control objective over desired
 * control vector (sqare root of gamma)
 * @param imax Max number of iterations
 *
 * @return Number of iterations, -1 upon failure
 */
/* int wls_alloc(num_t* u, num_t* v, num_t* umin, num_t* umax, num_t** B,
    num_t* u_guess, num_t* W_init, num_t* Wv, num_t* Wu, num_t* up,
    num_t gamma_sq, int imax) {
      */
void solveActiveSet_qr(const num_t A_col[CA_N_C*CA_N_U], const num_t b[CA_N_C], const num_t umin[CA_N_U],
                    const num_t umax[CA_N_U], const num_t u_guess[CA_N_U], bool updating,
                    num_t xs[CA_N_U], num_t Ws[CA_N_U], const int n_u, const int n_v, num_t *placeholder, num_t *fl)
  {

  // allocate variables, use defaults where parameters are set to 0
  // if(!gamma_sq) gamma_sq = 100000;
  int imax = 0;
  if(!imax) imax = 100;

  int n_c = n_u + n_v;

  //num_t xs[CA_N_U]; memcpy(xs, u_guess, sizeof(num_t)*n_u);
  memcpy(xs, u_guess, sizeof(num_t)*n_u);
  int iter = 0;
  int n_free = 0;

  num_t A[CA_N_C][CA_N_U];
  num_t A_free[CA_N_C][CA_N_U];
  num_t Q[CA_N_C][CA_N_C];
  num_t R[CA_N_C][CA_N_U];

  // Create a pointer array to the rows of A
  // such that we can pass it to a function
  num_t * A_ptr[CA_N_C];
  num_t * Q_ptr[CA_N_C];
  num_t * R_ptr[CA_N_C];
  for(int i = 0; i < n_c; i++) {
    A_ptr[i] = A[i];
    Q_ptr[i] = Q[i];
    R_ptr[i] = R[i];
  }

  int gamma[CA_N_U]; memset(gamma, 0, sizeof(int)*n_u);
  check_limits_tol(n_u, TOL, xs, umin, umax, gamma, 0);
  /*
  for (int i = 0; i < n_u; i++) {
    if (xs[i] <= umin[i]+TOL) {
      gamma[i] = +1;
    } else if (xs[i] >= umax[i]-TOL) {
      gamma[i] = -1;
    }
  }
  */

  int free_index_lookup[CA_N_U]; memset(free_index_lookup, -1, sizeof(int)*n_u);
  int permutation[CA_N_U]; memset(permutation, 0, sizeof(int)*n_u);
  for (int i = 0; i < n_u; i++) {
    if (gamma[i] == 0) {
      free_index_lookup[i] = n_free;
      permutation[n_free] = i;
      n_free++;
    }
  }

  // convert col major input to 2d array, using the permutaiton just found
  for(int i = 0; i < n_c; i++) {
    for(int j = 0; j < n_u; j++) {
      A[i][j] = A_col[i + n_c * permutation[j]];
    }
  }

  // initial factorisation
  qr_wrapper(n_c, n_u, permutation, A_ptr, Q_ptr, R_ptr);
  #ifdef debug_qr
  print_debug(A_ptr, Q_ptr, R_ptr, &n_u, &n_c);
  #endif

  int free_chk = 0;
  num_t q[CA_N_U];
  num_t z[CA_N_U];

  // -------------- Start loop ------------
  while (iter++ < imax) {
    num_t c[CA_N_U];
    for (int i=0; i < n_free; i++) {
      c[i] = 0;
      for (int j=0; j < n_c; j++) {
        c[i] += Q_ptr[j][i]*b[j];
      }
    }

    num_t u_bound_perm[CA_N_U];
    for (int k=0; k < n_u - n_free; k++) {
      if (gamma[permutation[k+n_free]] < 0)
        u_bound_perm[k] = umax[permutation[k+n_free]];
      else if (gamma[permutation[k+n_free]] > 0) 
        u_bound_perm[k] = umin[permutation[k+n_free]];
      else
        printf("Gamma out of bounds for bounded variables");
    }

    for (int i=0; i < n_free; i++) {
      for (int j=0; j < n_u - n_free; j++) {
        c[i] -= R_ptr[i][n_free+j] * u_bound_perm[j];
      }
    }

    backward_tri_solve(n_free, R_ptr, c, q);
    for (int i = 0; i < n_free; i++) {
      z[permutation[i]] = q[i];
    }
    for (int i = n_free; i < n_u; i++) {
      z[permutation[i]] = xs[permutation[i]];
    }

    int n_violated = 0;
    n_violated = check_limits_tol(n_free, TOL, z, umin, umax, gamma, permutation);
    /*
    for (int i = 0; i < n_free; i++) {
      if ((umin[permutation[i]] - TOL > z[permutation[i]]) ||
        (z[permutation[i]] > TOL + umax[permutation[i]])) {
          n_violated++;
        }
    }
    */

    if (!n_violated) {
      // is this the most efficient location TODO
      for (int i = 0; i < n_free; i++) {
        xs[permutation[i]] = z[permutation[i]];
      }

      if (n_free == n_u) {
        // no active constraints, we are optinal and feasible
        break;
      } else {
        // active constraints, check for optimality
        num_t d[CA_N_U];
        for (int i=n_free; i < n_u; i++) {
          d[i] = 0;
          for (int j=0; j < n_c; j++) {
            d[i] += Q_ptr[j][i]*b[j];
          }
        }

        for (int i=n_free; i < n_u; i++) {
          for (int j=i; j < n_u; j++){
            d[i] -= R_ptr[i][j]*xs[permutation[j]];
          }
        }

        num_t lambda_perm[CA_N_U];
        int f_free = 0;
        num_t maxlam = -1e10; // TODO
        for (int i=n_free; i<n_u; i++) {
          lambda_perm[i] = 0;
          for (int j=n_free; j <= i; j++) {
            lambda_perm[i] += R_ptr[j][i]*d[j];
          }
          lambda_perm[i] *= gamma[permutation[i]];
          if (lambda_perm[i] > maxlam) {
            maxlam = lambda_perm[i];
            f_free = i-n_free;
          }
        }

        if (maxlam <= 0) {
          break;
        }

        // free variable
        qr_shift(n_c, n_u, Q_ptr, R_ptr, n_free, n_free+f_free);
        #ifdef debug_qr
        print_debug(A_ptr, Q_ptr, R_ptr, &n_u, &n_c);
        #endif

        gamma[permutation[n_free+f_free]] = 0;
        int last_val = permutation[n_free+f_free];
        for (int i = f_free-1; i >= 0; i--) {
          permutation[n_free+i+1] = permutation[n_free+i];
        }
        permutation[n_free] = last_val;

        n_free++;

      }
    } else {

      num_t a = 1e10;
      int i_a = 0;
      int f_bound = 0;
      int i_s = 0;
      num_t temp;
      int temp_s;
      for (int f=0; f < n_free; f++) {
        int i = permutation[f];
        if (z[i] < umin[i]-TOL) {
          temp = (xs[i] - umin[i]) / (xs[i] - z[i]);
          temp_s = +1;
        } else if (z[i] > umax[i]+TOL) {
          temp = (umax[i] - xs[i]) / (z[i] - xs[i]);
          temp_s = -1;
        } else {
          continue;
        }
        if (temp < a) {
          a = temp;
          i_a = i;
          f_bound = f;
          i_s = temp_s;
        }
      }

      // update xs
      for (int i=0; i<n_u; i++) {
        xs[i] += a * (z[i] - xs[i]);
      }

      qr_shift(n_c, n_u, Q_ptr, R_ptr, n_free-1, f_bound);
      #ifdef debug_qr
      print_debug(A_ptr, Q_ptr, R_ptr, &n_u, &n_c);
      #endif

      gamma[i_a] = i_s;
      int first_val = permutation[f_bound];
      for (int i = 0; i < n_free-f_bound-1; i++) {
        permutation[f_bound+i] = permutation[f_bound+i+1];
      }
      permutation[n_free-1] = first_val;

      n_free--;

    }

  }
  return; // -1;
}

#if WLS_VERBOSE
void print_in_and_outputs(int n_c, int n_free, num_t** A_free_ptr, num_t* d, num_t* p_free) {

  printf("n_c = %d n_free = %d\n", n_c, n_free);

  printf("A_free =\n");
  for(int i = 0; i < n_c; i++) {
    for (int j = 0; j < n_free; j++) {
      printf("%f ", A_free_ptr[i][j]);
    }
    printf("\n");
  }

  printf("d = ");
  for (int j = 0; j < n_c; j++) {
    printf("%f ", d[j]);
  }

  printf("\noutput = ");
  for (int j = 0; j < n_free; j++) {
    printf("%f ", p_free[j]);
  }
  printf("\n\n");
}

void print_final_values(int n_u, int n_v, num_t* u, num_t** B, num_t* v, num_t* umin, num_t* umax) {
  printf("n_u = %d n_v = %d\n", n_u, n_v);

  printf("B =\n");
  for(int i = 0; i < n_v; i++) {
    for (int j = 0; j < n_u; j++) {
      printf("%f ", B[i][j]);
    }
    printf("\n");
  }

  printf("v = ");
  for (int j = 0; j < n_v; j++) {
    printf("%f ", v[j]);
  }

  printf("\nu = ");
  for (int j = 0; j < n_u; j++) {
    printf("%f ", u[j]);
  }
  printf("\n");

  printf("\numin = ");
  for (int j = 0; j < n_u; j++) {
    printf("%f ", umin[j]);
  }
  printf("\n");

  printf("\numax = ");
  for (int j = 0; j < n_u; j++) {
    printf("%f ", umax[j]);
  }
  printf("\n\n");

}
#endif

#ifdef debug_qr
void print_debug(num_t** A_ptr, num_t** Q_ptr, num_t** R_ptr, const int* n_u, const int* n_c) {
  printf("A_c = [");
  for (int i = 0; i < *n_c; i++) {
    for (int j = 0; j < *n_u; j++) {
      printf("%f ", A_ptr[i][j]);
    }
    printf(";\n");
  }
  printf("];\n\n");

  printf("Q_c = [");
  for (int i = 0; i < *n_c; i++) {
    for (int j = 0; j < *n_c; j++) {
      printf("%f ", Q_ptr[i][j]);
    }
    printf(";\n");
  }
  printf("];\n\n");

  printf("R_c = [");
  for (int i = 0; i < *n_c; i++) {
    for (int j = 0; j < *n_u; j++) {
      printf("%f ", R_ptr[i][j]);
    }
    printf(";\n");
  }
  printf("];\n\n");
}
#endif