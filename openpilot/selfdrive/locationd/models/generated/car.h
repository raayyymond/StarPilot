#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_8313107496762277149);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8946501874256176213);
void car_H_mod_fun(double *state, double *out_737346834279677204);
void car_f_fun(double *state, double dt, double *out_8361171851120628943);
void car_F_fun(double *state, double dt, double *out_8724633321717680272);
void car_h_25(double *state, double *unused, double *out_915753033439945975);
void car_H_25(double *state, double *unused, double *out_7183991198694182601);
void car_h_24(double *state, double *unused, double *out_8453070157681789700);
void car_H_24(double *state, double *unused, double *out_5011341599688683035);
void car_h_30(double *state, double *unused, double *out_8600545745863313602);
void car_H_30(double *state, double *unused, double *out_8744419916508120388);
void car_h_26(double *state, double *unused, double *out_6743078406888634476);
void car_H_26(double *state, double *unused, double *out_3442487879820126377);
void car_h_27(double *state, double *unused, double *out_4085920234220899670);
void car_H_27(double *state, double *unused, double *out_7527560845401006317);
void car_h_29(double *state, double *unused, double *out_2684914992129451266);
void car_H_29(double *state, double *unused, double *out_8234188572193728204);
void car_h_28(double *state, double *unused, double *out_5564514375222654806);
void car_H_28(double *state, double *unused, double *out_5130156484446292838);
void car_h_31(double *state, double *unused, double *out_4329514555794088687);
void car_H_31(double *state, double *unused, double *out_2816279777586774901);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}