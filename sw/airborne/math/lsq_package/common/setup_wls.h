#ifndef SETUP_WLS_H
#define SETUP_WLS_H

#include "math/lsq_package/common/size_defines.h"

extern void setup_wls(
    int n_v, int n_u, num_t JG[CA_N_V*CA_N_U], num_t Wv[CA_N_V],
    num_t Wu[CA_N_U], num_t up[CA_N_U], num_t dv[CA_N_V], num_t theta,
    num_t cond_bound, num_t A[CA_N_C*CA_N_U], num_t b[CA_N_C]);

void gamma_estimator(int n, num_t** A2, num_t cond_target, num_t* gamma, num_t* max_sig);
void cond_estimator(int n, num_t** A2, num_t min_diag2, num_t* cond_est, num_t* max_sig);

#endif

