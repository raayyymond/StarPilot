#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_2540016638677114272);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4211094856349578415);
void pose_H_mod_fun(double *state, double *out_4654741660211559160);
void pose_f_fun(double *state, double dt, double *out_2028336384740831999);
void pose_F_fun(double *state, double dt, double *out_6068183104054389338);
void pose_h_4(double *state, double *unused, double *out_8463415650667413426);
void pose_H_4(double *state, double *unused, double *out_4847640747440171114);
void pose_h_10(double *state, double *unused, double *out_3885613174414808707);
void pose_H_10(double *state, double *unused, double *out_1183211805543623442);
void pose_h_13(double *state, double *unused, double *out_7810348970534242965);
void pose_H_13(double *state, double *unused, double *out_1013885284137647090);
void pose_h_14(double *state, double *unused, double *out_7656480836221072816);
void pose_H_14(double *state, double *unused, double *out_1764852315144798818);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}