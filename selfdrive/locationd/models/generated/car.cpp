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
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_8353448122821896198) {
   out_8353448122821896198[0] = delta_x[0] + nom_x[0];
   out_8353448122821896198[1] = delta_x[1] + nom_x[1];
   out_8353448122821896198[2] = delta_x[2] + nom_x[2];
   out_8353448122821896198[3] = delta_x[3] + nom_x[3];
   out_8353448122821896198[4] = delta_x[4] + nom_x[4];
   out_8353448122821896198[5] = delta_x[5] + nom_x[5];
   out_8353448122821896198[6] = delta_x[6] + nom_x[6];
   out_8353448122821896198[7] = delta_x[7] + nom_x[7];
   out_8353448122821896198[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5467468061494965852) {
   out_5467468061494965852[0] = -nom_x[0] + true_x[0];
   out_5467468061494965852[1] = -nom_x[1] + true_x[1];
   out_5467468061494965852[2] = -nom_x[2] + true_x[2];
   out_5467468061494965852[3] = -nom_x[3] + true_x[3];
   out_5467468061494965852[4] = -nom_x[4] + true_x[4];
   out_5467468061494965852[5] = -nom_x[5] + true_x[5];
   out_5467468061494965852[6] = -nom_x[6] + true_x[6];
   out_5467468061494965852[7] = -nom_x[7] + true_x[7];
   out_5467468061494965852[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3378346308799840044) {
   out_3378346308799840044[0] = 1.0;
   out_3378346308799840044[1] = 0;
   out_3378346308799840044[2] = 0;
   out_3378346308799840044[3] = 0;
   out_3378346308799840044[4] = 0;
   out_3378346308799840044[5] = 0;
   out_3378346308799840044[6] = 0;
   out_3378346308799840044[7] = 0;
   out_3378346308799840044[8] = 0;
   out_3378346308799840044[9] = 0;
   out_3378346308799840044[10] = 1.0;
   out_3378346308799840044[11] = 0;
   out_3378346308799840044[12] = 0;
   out_3378346308799840044[13] = 0;
   out_3378346308799840044[14] = 0;
   out_3378346308799840044[15] = 0;
   out_3378346308799840044[16] = 0;
   out_3378346308799840044[17] = 0;
   out_3378346308799840044[18] = 0;
   out_3378346308799840044[19] = 0;
   out_3378346308799840044[20] = 1.0;
   out_3378346308799840044[21] = 0;
   out_3378346308799840044[22] = 0;
   out_3378346308799840044[23] = 0;
   out_3378346308799840044[24] = 0;
   out_3378346308799840044[25] = 0;
   out_3378346308799840044[26] = 0;
   out_3378346308799840044[27] = 0;
   out_3378346308799840044[28] = 0;
   out_3378346308799840044[29] = 0;
   out_3378346308799840044[30] = 1.0;
   out_3378346308799840044[31] = 0;
   out_3378346308799840044[32] = 0;
   out_3378346308799840044[33] = 0;
   out_3378346308799840044[34] = 0;
   out_3378346308799840044[35] = 0;
   out_3378346308799840044[36] = 0;
   out_3378346308799840044[37] = 0;
   out_3378346308799840044[38] = 0;
   out_3378346308799840044[39] = 0;
   out_3378346308799840044[40] = 1.0;
   out_3378346308799840044[41] = 0;
   out_3378346308799840044[42] = 0;
   out_3378346308799840044[43] = 0;
   out_3378346308799840044[44] = 0;
   out_3378346308799840044[45] = 0;
   out_3378346308799840044[46] = 0;
   out_3378346308799840044[47] = 0;
   out_3378346308799840044[48] = 0;
   out_3378346308799840044[49] = 0;
   out_3378346308799840044[50] = 1.0;
   out_3378346308799840044[51] = 0;
   out_3378346308799840044[52] = 0;
   out_3378346308799840044[53] = 0;
   out_3378346308799840044[54] = 0;
   out_3378346308799840044[55] = 0;
   out_3378346308799840044[56] = 0;
   out_3378346308799840044[57] = 0;
   out_3378346308799840044[58] = 0;
   out_3378346308799840044[59] = 0;
   out_3378346308799840044[60] = 1.0;
   out_3378346308799840044[61] = 0;
   out_3378346308799840044[62] = 0;
   out_3378346308799840044[63] = 0;
   out_3378346308799840044[64] = 0;
   out_3378346308799840044[65] = 0;
   out_3378346308799840044[66] = 0;
   out_3378346308799840044[67] = 0;
   out_3378346308799840044[68] = 0;
   out_3378346308799840044[69] = 0;
   out_3378346308799840044[70] = 1.0;
   out_3378346308799840044[71] = 0;
   out_3378346308799840044[72] = 0;
   out_3378346308799840044[73] = 0;
   out_3378346308799840044[74] = 0;
   out_3378346308799840044[75] = 0;
   out_3378346308799840044[76] = 0;
   out_3378346308799840044[77] = 0;
   out_3378346308799840044[78] = 0;
   out_3378346308799840044[79] = 0;
   out_3378346308799840044[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4900003475537959945) {
   out_4900003475537959945[0] = state[0];
   out_4900003475537959945[1] = state[1];
   out_4900003475537959945[2] = state[2];
   out_4900003475537959945[3] = state[3];
   out_4900003475537959945[4] = state[4];
   out_4900003475537959945[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4900003475537959945[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4900003475537959945[7] = state[7];
   out_4900003475537959945[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8563448028746898306) {
   out_8563448028746898306[0] = 1;
   out_8563448028746898306[1] = 0;
   out_8563448028746898306[2] = 0;
   out_8563448028746898306[3] = 0;
   out_8563448028746898306[4] = 0;
   out_8563448028746898306[5] = 0;
   out_8563448028746898306[6] = 0;
   out_8563448028746898306[7] = 0;
   out_8563448028746898306[8] = 0;
   out_8563448028746898306[9] = 0;
   out_8563448028746898306[10] = 1;
   out_8563448028746898306[11] = 0;
   out_8563448028746898306[12] = 0;
   out_8563448028746898306[13] = 0;
   out_8563448028746898306[14] = 0;
   out_8563448028746898306[15] = 0;
   out_8563448028746898306[16] = 0;
   out_8563448028746898306[17] = 0;
   out_8563448028746898306[18] = 0;
   out_8563448028746898306[19] = 0;
   out_8563448028746898306[20] = 1;
   out_8563448028746898306[21] = 0;
   out_8563448028746898306[22] = 0;
   out_8563448028746898306[23] = 0;
   out_8563448028746898306[24] = 0;
   out_8563448028746898306[25] = 0;
   out_8563448028746898306[26] = 0;
   out_8563448028746898306[27] = 0;
   out_8563448028746898306[28] = 0;
   out_8563448028746898306[29] = 0;
   out_8563448028746898306[30] = 1;
   out_8563448028746898306[31] = 0;
   out_8563448028746898306[32] = 0;
   out_8563448028746898306[33] = 0;
   out_8563448028746898306[34] = 0;
   out_8563448028746898306[35] = 0;
   out_8563448028746898306[36] = 0;
   out_8563448028746898306[37] = 0;
   out_8563448028746898306[38] = 0;
   out_8563448028746898306[39] = 0;
   out_8563448028746898306[40] = 1;
   out_8563448028746898306[41] = 0;
   out_8563448028746898306[42] = 0;
   out_8563448028746898306[43] = 0;
   out_8563448028746898306[44] = 0;
   out_8563448028746898306[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8563448028746898306[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8563448028746898306[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8563448028746898306[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8563448028746898306[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8563448028746898306[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8563448028746898306[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8563448028746898306[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8563448028746898306[53] = -9.8000000000000007*dt;
   out_8563448028746898306[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8563448028746898306[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8563448028746898306[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8563448028746898306[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8563448028746898306[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8563448028746898306[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8563448028746898306[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8563448028746898306[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8563448028746898306[62] = 0;
   out_8563448028746898306[63] = 0;
   out_8563448028746898306[64] = 0;
   out_8563448028746898306[65] = 0;
   out_8563448028746898306[66] = 0;
   out_8563448028746898306[67] = 0;
   out_8563448028746898306[68] = 0;
   out_8563448028746898306[69] = 0;
   out_8563448028746898306[70] = 1;
   out_8563448028746898306[71] = 0;
   out_8563448028746898306[72] = 0;
   out_8563448028746898306[73] = 0;
   out_8563448028746898306[74] = 0;
   out_8563448028746898306[75] = 0;
   out_8563448028746898306[76] = 0;
   out_8563448028746898306[77] = 0;
   out_8563448028746898306[78] = 0;
   out_8563448028746898306[79] = 0;
   out_8563448028746898306[80] = 1;
}
void h_25(double *state, double *unused, double *out_2164074651324699666) {
   out_2164074651324699666[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3891080028064799851) {
   out_3891080028064799851[0] = 0;
   out_3891080028064799851[1] = 0;
   out_3891080028064799851[2] = 0;
   out_3891080028064799851[3] = 0;
   out_3891080028064799851[4] = 0;
   out_3891080028064799851[5] = 0;
   out_3891080028064799851[6] = 1;
   out_3891080028064799851[7] = 0;
   out_3891080028064799851[8] = 0;
}
void h_24(double *state, double *unused, double *out_1055683949582273217) {
   out_1055683949582273217[0] = state[4];
   out_1055683949582273217[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1920305602957977114) {
   out_1920305602957977114[0] = 0;
   out_1920305602957977114[1] = 0;
   out_1920305602957977114[2] = 0;
   out_1920305602957977114[3] = 0;
   out_1920305602957977114[4] = 1;
   out_1920305602957977114[5] = 0;
   out_1920305602957977114[6] = 0;
   out_1920305602957977114[7] = 0;
   out_1920305602957977114[8] = 0;
   out_1920305602957977114[9] = 0;
   out_1920305602957977114[10] = 0;
   out_1920305602957977114[11] = 0;
   out_1920305602957977114[12] = 0;
   out_1920305602957977114[13] = 0;
   out_1920305602957977114[14] = 1;
   out_1920305602957977114[15] = 0;
   out_1920305602957977114[16] = 0;
   out_1920305602957977114[17] = 0;
}
void h_30(double *state, double *unused, double *out_1473374876929322684) {
   out_1473374876929322684[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4020418975208039921) {
   out_4020418975208039921[0] = 0;
   out_4020418975208039921[1] = 0;
   out_4020418975208039921[2] = 0;
   out_4020418975208039921[3] = 0;
   out_4020418975208039921[4] = 1;
   out_4020418975208039921[5] = 0;
   out_4020418975208039921[6] = 0;
   out_4020418975208039921[7] = 0;
   out_4020418975208039921[8] = 0;
}
void h_26(double *state, double *unused, double *out_8908518311358023497) {
   out_8908518311358023497[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7632583346938856075) {
   out_7632583346938856075[0] = 0;
   out_7632583346938856075[1] = 0;
   out_7632583346938856075[2] = 0;
   out_7632583346938856075[3] = 0;
   out_7632583346938856075[4] = 0;
   out_7632583346938856075[5] = 0;
   out_7632583346938856075[6] = 0;
   out_7632583346938856075[7] = 1;
   out_7632583346938856075[8] = 0;
}
void h_27(double *state, double *unused, double *out_3854522486060380626) {
   out_3854522486060380626[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6195182287008464832) {
   out_6195182287008464832[0] = 0;
   out_6195182287008464832[1] = 0;
   out_6195182287008464832[2] = 0;
   out_6195182287008464832[3] = 1;
   out_6195182287008464832[4] = 0;
   out_6195182287008464832[5] = 0;
   out_6195182287008464832[6] = 0;
   out_6195182287008464832[7] = 0;
   out_6195182287008464832[8] = 0;
}
void h_29(double *state, double *unused, double *out_906796495704895665) {
   out_906796495704895665[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3510187630893647737) {
   out_3510187630893647737[0] = 0;
   out_3510187630893647737[1] = 1;
   out_3510187630893647737[2] = 0;
   out_3510187630893647737[3] = 0;
   out_3510187630893647737[4] = 0;
   out_3510187630893647737[5] = 0;
   out_3510187630893647737[6] = 0;
   out_3510187630893647737[7] = 0;
   out_3510187630893647737[8] = 0;
}
void h_28(double *state, double *unused, double *out_6144370555670694786) {
   out_6144370555670694786[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5944914742312689614) {
   out_5944914742312689614[0] = 1;
   out_5944914742312689614[1] = 0;
   out_5944914742312689614[2] = 0;
   out_5944914742312689614[3] = 0;
   out_5944914742312689614[4] = 0;
   out_5944914742312689614[5] = 0;
   out_5944914742312689614[6] = 0;
   out_5944914742312689614[7] = 0;
   out_5944914742312689614[8] = 0;
}
void h_31(double *state, double *unused, double *out_8401396771630802765) {
   out_8401396771630802765[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3860434066187839423) {
   out_3860434066187839423[0] = 0;
   out_3860434066187839423[1] = 0;
   out_3860434066187839423[2] = 0;
   out_3860434066187839423[3] = 0;
   out_3860434066187839423[4] = 0;
   out_3860434066187839423[5] = 0;
   out_3860434066187839423[6] = 0;
   out_3860434066187839423[7] = 0;
   out_3860434066187839423[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8353448122821896198) {
  err_fun(nom_x, delta_x, out_8353448122821896198);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5467468061494965852) {
  inv_err_fun(nom_x, true_x, out_5467468061494965852);
}
void car_H_mod_fun(double *state, double *out_3378346308799840044) {
  H_mod_fun(state, out_3378346308799840044);
}
void car_f_fun(double *state, double dt, double *out_4900003475537959945) {
  f_fun(state,  dt, out_4900003475537959945);
}
void car_F_fun(double *state, double dt, double *out_8563448028746898306) {
  F_fun(state,  dt, out_8563448028746898306);
}
void car_h_25(double *state, double *unused, double *out_2164074651324699666) {
  h_25(state, unused, out_2164074651324699666);
}
void car_H_25(double *state, double *unused, double *out_3891080028064799851) {
  H_25(state, unused, out_3891080028064799851);
}
void car_h_24(double *state, double *unused, double *out_1055683949582273217) {
  h_24(state, unused, out_1055683949582273217);
}
void car_H_24(double *state, double *unused, double *out_1920305602957977114) {
  H_24(state, unused, out_1920305602957977114);
}
void car_h_30(double *state, double *unused, double *out_1473374876929322684) {
  h_30(state, unused, out_1473374876929322684);
}
void car_H_30(double *state, double *unused, double *out_4020418975208039921) {
  H_30(state, unused, out_4020418975208039921);
}
void car_h_26(double *state, double *unused, double *out_8908518311358023497) {
  h_26(state, unused, out_8908518311358023497);
}
void car_H_26(double *state, double *unused, double *out_7632583346938856075) {
  H_26(state, unused, out_7632583346938856075);
}
void car_h_27(double *state, double *unused, double *out_3854522486060380626) {
  h_27(state, unused, out_3854522486060380626);
}
void car_H_27(double *state, double *unused, double *out_6195182287008464832) {
  H_27(state, unused, out_6195182287008464832);
}
void car_h_29(double *state, double *unused, double *out_906796495704895665) {
  h_29(state, unused, out_906796495704895665);
}
void car_H_29(double *state, double *unused, double *out_3510187630893647737) {
  H_29(state, unused, out_3510187630893647737);
}
void car_h_28(double *state, double *unused, double *out_6144370555670694786) {
  h_28(state, unused, out_6144370555670694786);
}
void car_H_28(double *state, double *unused, double *out_5944914742312689614) {
  H_28(state, unused, out_5944914742312689614);
}
void car_h_31(double *state, double *unused, double *out_8401396771630802765) {
  h_31(state, unused, out_8401396771630802765);
}
void car_H_31(double *state, double *unused, double *out_3860434066187839423) {
  H_31(state, unused, out_3860434066187839423);
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
