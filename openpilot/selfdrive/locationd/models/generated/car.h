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
void car_err_fun(double *nom_x, double *delta_x, double *out_1839092989791714752);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4081361881616934295);
void car_H_mod_fun(double *state, double *out_6862801087908760350);
void car_f_fun(double *state, double dt, double *out_6900174257588161961);
void car_F_fun(double *state, double dt, double *out_2418882642574552225);
void car_h_25(double *state, double *unused, double *out_8556456949285266220);
void car_H_25(double *state, double *unused, double *out_2330387138003476887);
void car_h_24(double *state, double *unused, double *out_8166148376236623804);
void car_H_24(double *state, double *unused, double *out_4503036737008976453);
void car_h_30(double *state, double *unused, double *out_8399270280934137075);
void car_H_30(double *state, double *unused, double *out_4586303203488139868);
void car_h_26(double *state, double *unused, double *out_7924645227404098547);
void car_H_26(double *state, double *unused, double *out_6071890456877533111);
void car_h_27(double *state, double *unused, double *out_3594076951603972988);
void car_H_27(double *state, double *unused, double *out_2411539891687714957);
void car_h_29(double *state, double *unused, double *out_5794864960125439424);
void car_H_29(double *state, double *unused, double *out_698177164818163924);
void car_h_28(double *state, double *unused, double *out_4576161044939271100);
void car_H_28(double *state, double *unused, double *out_4384221852251366650);
void car_h_31(double *state, double *unused, double *out_8685147780431705209);
void car_H_31(double *state, double *unused, double *out_2299741176126516459);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}