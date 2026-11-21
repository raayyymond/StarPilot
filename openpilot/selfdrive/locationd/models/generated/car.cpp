#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_8313107496762277149) {
   out_8313107496762277149[0] = delta_x[0] + nom_x[0];
   out_8313107496762277149[1] = delta_x[1] + nom_x[1];
   out_8313107496762277149[2] = delta_x[2] + nom_x[2];
   out_8313107496762277149[3] = delta_x[3] + nom_x[3];
   out_8313107496762277149[4] = delta_x[4] + nom_x[4];
   out_8313107496762277149[5] = delta_x[5] + nom_x[5];
   out_8313107496762277149[6] = delta_x[6] + nom_x[6];
   out_8313107496762277149[7] = delta_x[7] + nom_x[7];
   out_8313107496762277149[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8946501874256176213) {
   out_8946501874256176213[0] = -nom_x[0] + true_x[0];
   out_8946501874256176213[1] = -nom_x[1] + true_x[1];
   out_8946501874256176213[2] = -nom_x[2] + true_x[2];
   out_8946501874256176213[3] = -nom_x[3] + true_x[3];
   out_8946501874256176213[4] = -nom_x[4] + true_x[4];
   out_8946501874256176213[5] = -nom_x[5] + true_x[5];
   out_8946501874256176213[6] = -nom_x[6] + true_x[6];
   out_8946501874256176213[7] = -nom_x[7] + true_x[7];
   out_8946501874256176213[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_737346834279677204) {
   out_737346834279677204[0] = 1.0;
   out_737346834279677204[1] = 0.0;
   out_737346834279677204[2] = 0.0;
   out_737346834279677204[3] = 0.0;
   out_737346834279677204[4] = 0.0;
   out_737346834279677204[5] = 0.0;
   out_737346834279677204[6] = 0.0;
   out_737346834279677204[7] = 0.0;
   out_737346834279677204[8] = 0.0;
   out_737346834279677204[9] = 0.0;
   out_737346834279677204[10] = 1.0;
   out_737346834279677204[11] = 0.0;
   out_737346834279677204[12] = 0.0;
   out_737346834279677204[13] = 0.0;
   out_737346834279677204[14] = 0.0;
   out_737346834279677204[15] = 0.0;
   out_737346834279677204[16] = 0.0;
   out_737346834279677204[17] = 0.0;
   out_737346834279677204[18] = 0.0;
   out_737346834279677204[19] = 0.0;
   out_737346834279677204[20] = 1.0;
   out_737346834279677204[21] = 0.0;
   out_737346834279677204[22] = 0.0;
   out_737346834279677204[23] = 0.0;
   out_737346834279677204[24] = 0.0;
   out_737346834279677204[25] = 0.0;
   out_737346834279677204[26] = 0.0;
   out_737346834279677204[27] = 0.0;
   out_737346834279677204[28] = 0.0;
   out_737346834279677204[29] = 0.0;
   out_737346834279677204[30] = 1.0;
   out_737346834279677204[31] = 0.0;
   out_737346834279677204[32] = 0.0;
   out_737346834279677204[33] = 0.0;
   out_737346834279677204[34] = 0.0;
   out_737346834279677204[35] = 0.0;
   out_737346834279677204[36] = 0.0;
   out_737346834279677204[37] = 0.0;
   out_737346834279677204[38] = 0.0;
   out_737346834279677204[39] = 0.0;
   out_737346834279677204[40] = 1.0;
   out_737346834279677204[41] = 0.0;
   out_737346834279677204[42] = 0.0;
   out_737346834279677204[43] = 0.0;
   out_737346834279677204[44] = 0.0;
   out_737346834279677204[45] = 0.0;
   out_737346834279677204[46] = 0.0;
   out_737346834279677204[47] = 0.0;
   out_737346834279677204[48] = 0.0;
   out_737346834279677204[49] = 0.0;
   out_737346834279677204[50] = 1.0;
   out_737346834279677204[51] = 0.0;
   out_737346834279677204[52] = 0.0;
   out_737346834279677204[53] = 0.0;
   out_737346834279677204[54] = 0.0;
   out_737346834279677204[55] = 0.0;
   out_737346834279677204[56] = 0.0;
   out_737346834279677204[57] = 0.0;
   out_737346834279677204[58] = 0.0;
   out_737346834279677204[59] = 0.0;
   out_737346834279677204[60] = 1.0;
   out_737346834279677204[61] = 0.0;
   out_737346834279677204[62] = 0.0;
   out_737346834279677204[63] = 0.0;
   out_737346834279677204[64] = 0.0;
   out_737346834279677204[65] = 0.0;
   out_737346834279677204[66] = 0.0;
   out_737346834279677204[67] = 0.0;
   out_737346834279677204[68] = 0.0;
   out_737346834279677204[69] = 0.0;
   out_737346834279677204[70] = 1.0;
   out_737346834279677204[71] = 0.0;
   out_737346834279677204[72] = 0.0;
   out_737346834279677204[73] = 0.0;
   out_737346834279677204[74] = 0.0;
   out_737346834279677204[75] = 0.0;
   out_737346834279677204[76] = 0.0;
   out_737346834279677204[77] = 0.0;
   out_737346834279677204[78] = 0.0;
   out_737346834279677204[79] = 0.0;
   out_737346834279677204[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8361171851120628943) {
   out_8361171851120628943[0] = state[0];
   out_8361171851120628943[1] = state[1];
   out_8361171851120628943[2] = state[2];
   out_8361171851120628943[3] = state[3];
   out_8361171851120628943[4] = state[4];
   out_8361171851120628943[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8361171851120628943[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8361171851120628943[7] = state[7];
   out_8361171851120628943[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8724633321717680272) {
   out_8724633321717680272[0] = 1;
   out_8724633321717680272[1] = 0;
   out_8724633321717680272[2] = 0;
   out_8724633321717680272[3] = 0;
   out_8724633321717680272[4] = 0;
   out_8724633321717680272[5] = 0;
   out_8724633321717680272[6] = 0;
   out_8724633321717680272[7] = 0;
   out_8724633321717680272[8] = 0;
   out_8724633321717680272[9] = 0;
   out_8724633321717680272[10] = 1;
   out_8724633321717680272[11] = 0;
   out_8724633321717680272[12] = 0;
   out_8724633321717680272[13] = 0;
   out_8724633321717680272[14] = 0;
   out_8724633321717680272[15] = 0;
   out_8724633321717680272[16] = 0;
   out_8724633321717680272[17] = 0;
   out_8724633321717680272[18] = 0;
   out_8724633321717680272[19] = 0;
   out_8724633321717680272[20] = 1;
   out_8724633321717680272[21] = 0;
   out_8724633321717680272[22] = 0;
   out_8724633321717680272[23] = 0;
   out_8724633321717680272[24] = 0;
   out_8724633321717680272[25] = 0;
   out_8724633321717680272[26] = 0;
   out_8724633321717680272[27] = 0;
   out_8724633321717680272[28] = 0;
   out_8724633321717680272[29] = 0;
   out_8724633321717680272[30] = 1;
   out_8724633321717680272[31] = 0;
   out_8724633321717680272[32] = 0;
   out_8724633321717680272[33] = 0;
   out_8724633321717680272[34] = 0;
   out_8724633321717680272[35] = 0;
   out_8724633321717680272[36] = 0;
   out_8724633321717680272[37] = 0;
   out_8724633321717680272[38] = 0;
   out_8724633321717680272[39] = 0;
   out_8724633321717680272[40] = 1;
   out_8724633321717680272[41] = 0;
   out_8724633321717680272[42] = 0;
   out_8724633321717680272[43] = 0;
   out_8724633321717680272[44] = 0;
   out_8724633321717680272[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8724633321717680272[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8724633321717680272[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8724633321717680272[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8724633321717680272[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8724633321717680272[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8724633321717680272[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8724633321717680272[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8724633321717680272[53] = -9.8100000000000005*dt;
   out_8724633321717680272[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8724633321717680272[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8724633321717680272[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8724633321717680272[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8724633321717680272[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8724633321717680272[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8724633321717680272[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8724633321717680272[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8724633321717680272[62] = 0;
   out_8724633321717680272[63] = 0;
   out_8724633321717680272[64] = 0;
   out_8724633321717680272[65] = 0;
   out_8724633321717680272[66] = 0;
   out_8724633321717680272[67] = 0;
   out_8724633321717680272[68] = 0;
   out_8724633321717680272[69] = 0;
   out_8724633321717680272[70] = 1;
   out_8724633321717680272[71] = 0;
   out_8724633321717680272[72] = 0;
   out_8724633321717680272[73] = 0;
   out_8724633321717680272[74] = 0;
   out_8724633321717680272[75] = 0;
   out_8724633321717680272[76] = 0;
   out_8724633321717680272[77] = 0;
   out_8724633321717680272[78] = 0;
   out_8724633321717680272[79] = 0;
   out_8724633321717680272[80] = 1;
}
void h_25(double *state, double *unused, double *out_915753033439945975) {
   out_915753033439945975[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7183991198694182601) {
   out_7183991198694182601[0] = 0;
   out_7183991198694182601[1] = 0;
   out_7183991198694182601[2] = 0;
   out_7183991198694182601[3] = 0;
   out_7183991198694182601[4] = 0;
   out_7183991198694182601[5] = 0;
   out_7183991198694182601[6] = 1;
   out_7183991198694182601[7] = 0;
   out_7183991198694182601[8] = 0;
}
void h_24(double *state, double *unused, double *out_8453070157681789700) {
   out_8453070157681789700[0] = state[4];
   out_8453070157681789700[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5011341599688683035) {
   out_5011341599688683035[0] = 0;
   out_5011341599688683035[1] = 0;
   out_5011341599688683035[2] = 0;
   out_5011341599688683035[3] = 0;
   out_5011341599688683035[4] = 1;
   out_5011341599688683035[5] = 0;
   out_5011341599688683035[6] = 0;
   out_5011341599688683035[7] = 0;
   out_5011341599688683035[8] = 0;
   out_5011341599688683035[9] = 0;
   out_5011341599688683035[10] = 0;
   out_5011341599688683035[11] = 0;
   out_5011341599688683035[12] = 0;
   out_5011341599688683035[13] = 0;
   out_5011341599688683035[14] = 1;
   out_5011341599688683035[15] = 0;
   out_5011341599688683035[16] = 0;
   out_5011341599688683035[17] = 0;
}
void h_30(double *state, double *unused, double *out_8600545745863313602) {
   out_8600545745863313602[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8744419916508120388) {
   out_8744419916508120388[0] = 0;
   out_8744419916508120388[1] = 0;
   out_8744419916508120388[2] = 0;
   out_8744419916508120388[3] = 0;
   out_8744419916508120388[4] = 1;
   out_8744419916508120388[5] = 0;
   out_8744419916508120388[6] = 0;
   out_8744419916508120388[7] = 0;
   out_8744419916508120388[8] = 0;
}
void h_26(double *state, double *unused, double *out_6743078406888634476) {
   out_6743078406888634476[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3442487879820126377) {
   out_3442487879820126377[0] = 0;
   out_3442487879820126377[1] = 0;
   out_3442487879820126377[2] = 0;
   out_3442487879820126377[3] = 0;
   out_3442487879820126377[4] = 0;
   out_3442487879820126377[5] = 0;
   out_3442487879820126377[6] = 0;
   out_3442487879820126377[7] = 1;
   out_3442487879820126377[8] = 0;
}
void h_27(double *state, double *unused, double *out_4085920234220899670) {
   out_4085920234220899670[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7527560845401006317) {
   out_7527560845401006317[0] = 0;
   out_7527560845401006317[1] = 0;
   out_7527560845401006317[2] = 0;
   out_7527560845401006317[3] = 1;
   out_7527560845401006317[4] = 0;
   out_7527560845401006317[5] = 0;
   out_7527560845401006317[6] = 0;
   out_7527560845401006317[7] = 0;
   out_7527560845401006317[8] = 0;
}
void h_29(double *state, double *unused, double *out_2684914992129451266) {
   out_2684914992129451266[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8234188572193728204) {
   out_8234188572193728204[0] = 0;
   out_8234188572193728204[1] = 1;
   out_8234188572193728204[2] = 0;
   out_8234188572193728204[3] = 0;
   out_8234188572193728204[4] = 0;
   out_8234188572193728204[5] = 0;
   out_8234188572193728204[6] = 0;
   out_8234188572193728204[7] = 0;
   out_8234188572193728204[8] = 0;
}
void h_28(double *state, double *unused, double *out_5564514375222654806) {
   out_5564514375222654806[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5130156484446292838) {
   out_5130156484446292838[0] = 1;
   out_5130156484446292838[1] = 0;
   out_5130156484446292838[2] = 0;
   out_5130156484446292838[3] = 0;
   out_5130156484446292838[4] = 0;
   out_5130156484446292838[5] = 0;
   out_5130156484446292838[6] = 0;
   out_5130156484446292838[7] = 0;
   out_5130156484446292838[8] = 0;
}
void h_31(double *state, double *unused, double *out_4329514555794088687) {
   out_4329514555794088687[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2816279777586774901) {
   out_2816279777586774901[0] = 0;
   out_2816279777586774901[1] = 0;
   out_2816279777586774901[2] = 0;
   out_2816279777586774901[3] = 0;
   out_2816279777586774901[4] = 0;
   out_2816279777586774901[5] = 0;
   out_2816279777586774901[6] = 0;
   out_2816279777586774901[7] = 0;
   out_2816279777586774901[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_8313107496762277149) {
  err_fun(nom_x, delta_x, out_8313107496762277149);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8946501874256176213) {
  inv_err_fun(nom_x, true_x, out_8946501874256176213);
}
void car_H_mod_fun(double *state, double *out_737346834279677204) {
  H_mod_fun(state, out_737346834279677204);
}
void car_f_fun(double *state, double dt, double *out_8361171851120628943) {
  f_fun(state,  dt, out_8361171851120628943);
}
void car_F_fun(double *state, double dt, double *out_8724633321717680272) {
  F_fun(state,  dt, out_8724633321717680272);
}
void car_h_25(double *state, double *unused, double *out_915753033439945975) {
  h_25(state, unused, out_915753033439945975);
}
void car_H_25(double *state, double *unused, double *out_7183991198694182601) {
  H_25(state, unused, out_7183991198694182601);
}
void car_h_24(double *state, double *unused, double *out_8453070157681789700) {
  h_24(state, unused, out_8453070157681789700);
}
void car_H_24(double *state, double *unused, double *out_5011341599688683035) {
  H_24(state, unused, out_5011341599688683035);
}
void car_h_30(double *state, double *unused, double *out_8600545745863313602) {
  h_30(state, unused, out_8600545745863313602);
}
void car_H_30(double *state, double *unused, double *out_8744419916508120388) {
  H_30(state, unused, out_8744419916508120388);
}
void car_h_26(double *state, double *unused, double *out_6743078406888634476) {
  h_26(state, unused, out_6743078406888634476);
}
void car_H_26(double *state, double *unused, double *out_3442487879820126377) {
  H_26(state, unused, out_3442487879820126377);
}
void car_h_27(double *state, double *unused, double *out_4085920234220899670) {
  h_27(state, unused, out_4085920234220899670);
}
void car_H_27(double *state, double *unused, double *out_7527560845401006317) {
  H_27(state, unused, out_7527560845401006317);
}
void car_h_29(double *state, double *unused, double *out_2684914992129451266) {
  h_29(state, unused, out_2684914992129451266);
}
void car_H_29(double *state, double *unused, double *out_8234188572193728204) {
  H_29(state, unused, out_8234188572193728204);
}
void car_h_28(double *state, double *unused, double *out_5564514375222654806) {
  h_28(state, unused, out_5564514375222654806);
}
void car_H_28(double *state, double *unused, double *out_5130156484446292838) {
  H_28(state, unused, out_5130156484446292838);
}
void car_h_31(double *state, double *unused, double *out_4329514555794088687) {
  h_31(state, unused, out_4329514555794088687);
}
void car_H_31(double *state, double *unused, double *out_2816279777586774901) {
  H_31(state, unused, out_2816279777586774901);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
