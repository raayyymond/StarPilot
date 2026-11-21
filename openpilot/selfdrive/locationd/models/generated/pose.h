#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_6433275272970939599);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7086127294916974707);
void pose_H_mod_fun(double *state, double *out_4308325429962827123);
void pose_f_fun(double *state, double dt, double *out_6784884137237425870);
void pose_F_fun(double *state, double dt, double *out_1899404572103969321);
void pose_h_4(double *state, double *unused, double *out_8673137181907007987);
void pose_H_4(double *state, double *unused, double *out_536518220118919551);
void pose_h_10(double *state, double *unused, double *out_4049385380413447719);
void pose_H_10(double *state, double *unused, double *out_393587758664493186);
void pose_h_13(double *state, double *unused, double *out_7785508574307562513);
void pose_H_13(double *state, double *unused, double *out_4370273683421443575);
void pose_h_14(double *state, double *unused, double *out_8347004880535916513);
void pose_H_14(double *state, double *unused, double *out_3619306652414291847);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}