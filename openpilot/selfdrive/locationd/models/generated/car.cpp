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
void err_fun(double *nom_x, double *delta_x, double *out_1839092989791714752) {
   out_1839092989791714752[0] = delta_x[0] + nom_x[0];
   out_1839092989791714752[1] = delta_x[1] + nom_x[1];
   out_1839092989791714752[2] = delta_x[2] + nom_x[2];
   out_1839092989791714752[3] = delta_x[3] + nom_x[3];
   out_1839092989791714752[4] = delta_x[4] + nom_x[4];
   out_1839092989791714752[5] = delta_x[5] + nom_x[5];
   out_1839092989791714752[6] = delta_x[6] + nom_x[6];
   out_1839092989791714752[7] = delta_x[7] + nom_x[7];
   out_1839092989791714752[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4081361881616934295) {
   out_4081361881616934295[0] = -nom_x[0] + true_x[0];
   out_4081361881616934295[1] = -nom_x[1] + true_x[1];
   out_4081361881616934295[2] = -nom_x[2] + true_x[2];
   out_4081361881616934295[3] = -nom_x[3] + true_x[3];
   out_4081361881616934295[4] = -nom_x[4] + true_x[4];
   out_4081361881616934295[5] = -nom_x[5] + true_x[5];
   out_4081361881616934295[6] = -nom_x[6] + true_x[6];
   out_4081361881616934295[7] = -nom_x[7] + true_x[7];
   out_4081361881616934295[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6862801087908760350) {
   out_6862801087908760350[0] = 1.0;
   out_6862801087908760350[1] = 0.0;
   out_6862801087908760350[2] = 0.0;
   out_6862801087908760350[3] = 0.0;
   out_6862801087908760350[4] = 0.0;
   out_6862801087908760350[5] = 0.0;
   out_6862801087908760350[6] = 0.0;
   out_6862801087908760350[7] = 0.0;
   out_6862801087908760350[8] = 0.0;
   out_6862801087908760350[9] = 0.0;
   out_6862801087908760350[10] = 1.0;
   out_6862801087908760350[11] = 0.0;
   out_6862801087908760350[12] = 0.0;
   out_6862801087908760350[13] = 0.0;
   out_6862801087908760350[14] = 0.0;
   out_6862801087908760350[15] = 0.0;
   out_6862801087908760350[16] = 0.0;
   out_6862801087908760350[17] = 0.0;
   out_6862801087908760350[18] = 0.0;
   out_6862801087908760350[19] = 0.0;
   out_6862801087908760350[20] = 1.0;
   out_6862801087908760350[21] = 0.0;
   out_6862801087908760350[22] = 0.0;
   out_6862801087908760350[23] = 0.0;
   out_6862801087908760350[24] = 0.0;
   out_6862801087908760350[25] = 0.0;
   out_6862801087908760350[26] = 0.0;
   out_6862801087908760350[27] = 0.0;
   out_6862801087908760350[28] = 0.0;
   out_6862801087908760350[29] = 0.0;
   out_6862801087908760350[30] = 1.0;
   out_6862801087908760350[31] = 0.0;
   out_6862801087908760350[32] = 0.0;
   out_6862801087908760350[33] = 0.0;
   out_6862801087908760350[34] = 0.0;
   out_6862801087908760350[35] = 0.0;
   out_6862801087908760350[36] = 0.0;
   out_6862801087908760350[37] = 0.0;
   out_6862801087908760350[38] = 0.0;
   out_6862801087908760350[39] = 0.0;
   out_6862801087908760350[40] = 1.0;
   out_6862801087908760350[41] = 0.0;
   out_6862801087908760350[42] = 0.0;
   out_6862801087908760350[43] = 0.0;
   out_6862801087908760350[44] = 0.0;
   out_6862801087908760350[45] = 0.0;
   out_6862801087908760350[46] = 0.0;
   out_6862801087908760350[47] = 0.0;
   out_6862801087908760350[48] = 0.0;
   out_6862801087908760350[49] = 0.0;
   out_6862801087908760350[50] = 1.0;
   out_6862801087908760350[51] = 0.0;
   out_6862801087908760350[52] = 0.0;
   out_6862801087908760350[53] = 0.0;
   out_6862801087908760350[54] = 0.0;
   out_6862801087908760350[55] = 0.0;
   out_6862801087908760350[56] = 0.0;
   out_6862801087908760350[57] = 0.0;
   out_6862801087908760350[58] = 0.0;
   out_6862801087908760350[59] = 0.0;
   out_6862801087908760350[60] = 1.0;
   out_6862801087908760350[61] = 0.0;
   out_6862801087908760350[62] = 0.0;
   out_6862801087908760350[63] = 0.0;
   out_6862801087908760350[64] = 0.0;
   out_6862801087908760350[65] = 0.0;
   out_6862801087908760350[66] = 0.0;
   out_6862801087908760350[67] = 0.0;
   out_6862801087908760350[68] = 0.0;
   out_6862801087908760350[69] = 0.0;
   out_6862801087908760350[70] = 1.0;
   out_6862801087908760350[71] = 0.0;
   out_6862801087908760350[72] = 0.0;
   out_6862801087908760350[73] = 0.0;
   out_6862801087908760350[74] = 0.0;
   out_6862801087908760350[75] = 0.0;
   out_6862801087908760350[76] = 0.0;
   out_6862801087908760350[77] = 0.0;
   out_6862801087908760350[78] = 0.0;
   out_6862801087908760350[79] = 0.0;
   out_6862801087908760350[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6900174257588161961) {
   out_6900174257588161961[0] = state[0];
   out_6900174257588161961[1] = state[1];
   out_6900174257588161961[2] = state[2];
   out_6900174257588161961[3] = state[3];
   out_6900174257588161961[4] = state[4];
   out_6900174257588161961[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6900174257588161961[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6900174257588161961[7] = state[7];
   out_6900174257588161961[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2418882642574552225) {
   out_2418882642574552225[0] = 1;
   out_2418882642574552225[1] = 0;
   out_2418882642574552225[2] = 0;
   out_2418882642574552225[3] = 0;
   out_2418882642574552225[4] = 0;
   out_2418882642574552225[5] = 0;
   out_2418882642574552225[6] = 0;
   out_2418882642574552225[7] = 0;
   out_2418882642574552225[8] = 0;
   out_2418882642574552225[9] = 0;
   out_2418882642574552225[10] = 1;
   out_2418882642574552225[11] = 0;
   out_2418882642574552225[12] = 0;
   out_2418882642574552225[13] = 0;
   out_2418882642574552225[14] = 0;
   out_2418882642574552225[15] = 0;
   out_2418882642574552225[16] = 0;
   out_2418882642574552225[17] = 0;
   out_2418882642574552225[18] = 0;
   out_2418882642574552225[19] = 0;
   out_2418882642574552225[20] = 1;
   out_2418882642574552225[21] = 0;
   out_2418882642574552225[22] = 0;
   out_2418882642574552225[23] = 0;
   out_2418882642574552225[24] = 0;
   out_2418882642574552225[25] = 0;
   out_2418882642574552225[26] = 0;
   out_2418882642574552225[27] = 0;
   out_2418882642574552225[28] = 0;
   out_2418882642574552225[29] = 0;
   out_2418882642574552225[30] = 1;
   out_2418882642574552225[31] = 0;
   out_2418882642574552225[32] = 0;
   out_2418882642574552225[33] = 0;
   out_2418882642574552225[34] = 0;
   out_2418882642574552225[35] = 0;
   out_2418882642574552225[36] = 0;
   out_2418882642574552225[37] = 0;
   out_2418882642574552225[38] = 0;
   out_2418882642574552225[39] = 0;
   out_2418882642574552225[40] = 1;
   out_2418882642574552225[41] = 0;
   out_2418882642574552225[42] = 0;
   out_2418882642574552225[43] = 0;
   out_2418882642574552225[44] = 0;
   out_2418882642574552225[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2418882642574552225[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2418882642574552225[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2418882642574552225[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2418882642574552225[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2418882642574552225[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2418882642574552225[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2418882642574552225[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2418882642574552225[53] = -9.8100000000000005*dt;
   out_2418882642574552225[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2418882642574552225[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2418882642574552225[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2418882642574552225[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2418882642574552225[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2418882642574552225[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2418882642574552225[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2418882642574552225[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2418882642574552225[62] = 0;
   out_2418882642574552225[63] = 0;
   out_2418882642574552225[64] = 0;
   out_2418882642574552225[65] = 0;
   out_2418882642574552225[66] = 0;
   out_2418882642574552225[67] = 0;
   out_2418882642574552225[68] = 0;
   out_2418882642574552225[69] = 0;
   out_2418882642574552225[70] = 1;
   out_2418882642574552225[71] = 0;
   out_2418882642574552225[72] = 0;
   out_2418882642574552225[73] = 0;
   out_2418882642574552225[74] = 0;
   out_2418882642574552225[75] = 0;
   out_2418882642574552225[76] = 0;
   out_2418882642574552225[77] = 0;
   out_2418882642574552225[78] = 0;
   out_2418882642574552225[79] = 0;
   out_2418882642574552225[80] = 1;
}
void h_25(double *state, double *unused, double *out_8556456949285266220) {
   out_8556456949285266220[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2330387138003476887) {
   out_2330387138003476887[0] = 0;
   out_2330387138003476887[1] = 0;
   out_2330387138003476887[2] = 0;
   out_2330387138003476887[3] = 0;
   out_2330387138003476887[4] = 0;
   out_2330387138003476887[5] = 0;
   out_2330387138003476887[6] = 1;
   out_2330387138003476887[7] = 0;
   out_2330387138003476887[8] = 0;
}
void h_24(double *state, double *unused, double *out_8166148376236623804) {
   out_8166148376236623804[0] = state[4];
   out_8166148376236623804[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4503036737008976453) {
   out_4503036737008976453[0] = 0;
   out_4503036737008976453[1] = 0;
   out_4503036737008976453[2] = 0;
   out_4503036737008976453[3] = 0;
   out_4503036737008976453[4] = 1;
   out_4503036737008976453[5] = 0;
   out_4503036737008976453[6] = 0;
   out_4503036737008976453[7] = 0;
   out_4503036737008976453[8] = 0;
   out_4503036737008976453[9] = 0;
   out_4503036737008976453[10] = 0;
   out_4503036737008976453[11] = 0;
   out_4503036737008976453[12] = 0;
   out_4503036737008976453[13] = 0;
   out_4503036737008976453[14] = 1;
   out_4503036737008976453[15] = 0;
   out_4503036737008976453[16] = 0;
   out_4503036737008976453[17] = 0;
}
void h_30(double *state, double *unused, double *out_8399270280934137075) {
   out_8399270280934137075[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4586303203488139868) {
   out_4586303203488139868[0] = 0;
   out_4586303203488139868[1] = 0;
   out_4586303203488139868[2] = 0;
   out_4586303203488139868[3] = 0;
   out_4586303203488139868[4] = 1;
   out_4586303203488139868[5] = 0;
   out_4586303203488139868[6] = 0;
   out_4586303203488139868[7] = 0;
   out_4586303203488139868[8] = 0;
}
void h_26(double *state, double *unused, double *out_7924645227404098547) {
   out_7924645227404098547[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6071890456877533111) {
   out_6071890456877533111[0] = 0;
   out_6071890456877533111[1] = 0;
   out_6071890456877533111[2] = 0;
   out_6071890456877533111[3] = 0;
   out_6071890456877533111[4] = 0;
   out_6071890456877533111[5] = 0;
   out_6071890456877533111[6] = 0;
   out_6071890456877533111[7] = 1;
   out_6071890456877533111[8] = 0;
}
void h_27(double *state, double *unused, double *out_3594076951603972988) {
   out_3594076951603972988[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2411539891687714957) {
   out_2411539891687714957[0] = 0;
   out_2411539891687714957[1] = 0;
   out_2411539891687714957[2] = 0;
   out_2411539891687714957[3] = 1;
   out_2411539891687714957[4] = 0;
   out_2411539891687714957[5] = 0;
   out_2411539891687714957[6] = 0;
   out_2411539891687714957[7] = 0;
   out_2411539891687714957[8] = 0;
}
void h_29(double *state, double *unused, double *out_5794864960125439424) {
   out_5794864960125439424[0] = state[1];
}
void H_29(double *state, double *unused, double *out_698177164818163924) {
   out_698177164818163924[0] = 0;
   out_698177164818163924[1] = 1;
   out_698177164818163924[2] = 0;
   out_698177164818163924[3] = 0;
   out_698177164818163924[4] = 0;
   out_698177164818163924[5] = 0;
   out_698177164818163924[6] = 0;
   out_698177164818163924[7] = 0;
   out_698177164818163924[8] = 0;
}
void h_28(double *state, double *unused, double *out_4576161044939271100) {
   out_4576161044939271100[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4384221852251366650) {
   out_4384221852251366650[0] = 1;
   out_4384221852251366650[1] = 0;
   out_4384221852251366650[2] = 0;
   out_4384221852251366650[3] = 0;
   out_4384221852251366650[4] = 0;
   out_4384221852251366650[5] = 0;
   out_4384221852251366650[6] = 0;
   out_4384221852251366650[7] = 0;
   out_4384221852251366650[8] = 0;
}
void h_31(double *state, double *unused, double *out_8685147780431705209) {
   out_8685147780431705209[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2299741176126516459) {
   out_2299741176126516459[0] = 0;
   out_2299741176126516459[1] = 0;
   out_2299741176126516459[2] = 0;
   out_2299741176126516459[3] = 0;
   out_2299741176126516459[4] = 0;
   out_2299741176126516459[5] = 0;
   out_2299741176126516459[6] = 0;
   out_2299741176126516459[7] = 0;
   out_2299741176126516459[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_1839092989791714752) {
  err_fun(nom_x, delta_x, out_1839092989791714752);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4081361881616934295) {
  inv_err_fun(nom_x, true_x, out_4081361881616934295);
}
void car_H_mod_fun(double *state, double *out_6862801087908760350) {
  H_mod_fun(state, out_6862801087908760350);
}
void car_f_fun(double *state, double dt, double *out_6900174257588161961) {
  f_fun(state,  dt, out_6900174257588161961);
}
void car_F_fun(double *state, double dt, double *out_2418882642574552225) {
  F_fun(state,  dt, out_2418882642574552225);
}
void car_h_25(double *state, double *unused, double *out_8556456949285266220) {
  h_25(state, unused, out_8556456949285266220);
}
void car_H_25(double *state, double *unused, double *out_2330387138003476887) {
  H_25(state, unused, out_2330387138003476887);
}
void car_h_24(double *state, double *unused, double *out_8166148376236623804) {
  h_24(state, unused, out_8166148376236623804);
}
void car_H_24(double *state, double *unused, double *out_4503036737008976453) {
  H_24(state, unused, out_4503036737008976453);
}
void car_h_30(double *state, double *unused, double *out_8399270280934137075) {
  h_30(state, unused, out_8399270280934137075);
}
void car_H_30(double *state, double *unused, double *out_4586303203488139868) {
  H_30(state, unused, out_4586303203488139868);
}
void car_h_26(double *state, double *unused, double *out_7924645227404098547) {
  h_26(state, unused, out_7924645227404098547);
}
void car_H_26(double *state, double *unused, double *out_6071890456877533111) {
  H_26(state, unused, out_6071890456877533111);
}
void car_h_27(double *state, double *unused, double *out_3594076951603972988) {
  h_27(state, unused, out_3594076951603972988);
}
void car_H_27(double *state, double *unused, double *out_2411539891687714957) {
  H_27(state, unused, out_2411539891687714957);
}
void car_h_29(double *state, double *unused, double *out_5794864960125439424) {
  h_29(state, unused, out_5794864960125439424);
}
void car_H_29(double *state, double *unused, double *out_698177164818163924) {
  H_29(state, unused, out_698177164818163924);
}
void car_h_28(double *state, double *unused, double *out_4576161044939271100) {
  h_28(state, unused, out_4576161044939271100);
}
void car_H_28(double *state, double *unused, double *out_4384221852251366650) {
  H_28(state, unused, out_4384221852251366650);
}
void car_h_31(double *state, double *unused, double *out_8685147780431705209) {
  h_31(state, unused, out_8685147780431705209);
}
void car_H_31(double *state, double *unused, double *out_2299741176126516459) {
  H_31(state, unused, out_2299741176126516459);
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
