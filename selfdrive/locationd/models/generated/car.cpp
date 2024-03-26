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
void err_fun(double *nom_x, double *delta_x, double *out_7418791751555442547) {
   out_7418791751555442547[0] = delta_x[0] + nom_x[0];
   out_7418791751555442547[1] = delta_x[1] + nom_x[1];
   out_7418791751555442547[2] = delta_x[2] + nom_x[2];
   out_7418791751555442547[3] = delta_x[3] + nom_x[3];
   out_7418791751555442547[4] = delta_x[4] + nom_x[4];
   out_7418791751555442547[5] = delta_x[5] + nom_x[5];
   out_7418791751555442547[6] = delta_x[6] + nom_x[6];
   out_7418791751555442547[7] = delta_x[7] + nom_x[7];
   out_7418791751555442547[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4675497126535007434) {
   out_4675497126535007434[0] = -nom_x[0] + true_x[0];
   out_4675497126535007434[1] = -nom_x[1] + true_x[1];
   out_4675497126535007434[2] = -nom_x[2] + true_x[2];
   out_4675497126535007434[3] = -nom_x[3] + true_x[3];
   out_4675497126535007434[4] = -nom_x[4] + true_x[4];
   out_4675497126535007434[5] = -nom_x[5] + true_x[5];
   out_4675497126535007434[6] = -nom_x[6] + true_x[6];
   out_4675497126535007434[7] = -nom_x[7] + true_x[7];
   out_4675497126535007434[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4383613899688875359) {
   out_4383613899688875359[0] = 1.0;
   out_4383613899688875359[1] = 0;
   out_4383613899688875359[2] = 0;
   out_4383613899688875359[3] = 0;
   out_4383613899688875359[4] = 0;
   out_4383613899688875359[5] = 0;
   out_4383613899688875359[6] = 0;
   out_4383613899688875359[7] = 0;
   out_4383613899688875359[8] = 0;
   out_4383613899688875359[9] = 0;
   out_4383613899688875359[10] = 1.0;
   out_4383613899688875359[11] = 0;
   out_4383613899688875359[12] = 0;
   out_4383613899688875359[13] = 0;
   out_4383613899688875359[14] = 0;
   out_4383613899688875359[15] = 0;
   out_4383613899688875359[16] = 0;
   out_4383613899688875359[17] = 0;
   out_4383613899688875359[18] = 0;
   out_4383613899688875359[19] = 0;
   out_4383613899688875359[20] = 1.0;
   out_4383613899688875359[21] = 0;
   out_4383613899688875359[22] = 0;
   out_4383613899688875359[23] = 0;
   out_4383613899688875359[24] = 0;
   out_4383613899688875359[25] = 0;
   out_4383613899688875359[26] = 0;
   out_4383613899688875359[27] = 0;
   out_4383613899688875359[28] = 0;
   out_4383613899688875359[29] = 0;
   out_4383613899688875359[30] = 1.0;
   out_4383613899688875359[31] = 0;
   out_4383613899688875359[32] = 0;
   out_4383613899688875359[33] = 0;
   out_4383613899688875359[34] = 0;
   out_4383613899688875359[35] = 0;
   out_4383613899688875359[36] = 0;
   out_4383613899688875359[37] = 0;
   out_4383613899688875359[38] = 0;
   out_4383613899688875359[39] = 0;
   out_4383613899688875359[40] = 1.0;
   out_4383613899688875359[41] = 0;
   out_4383613899688875359[42] = 0;
   out_4383613899688875359[43] = 0;
   out_4383613899688875359[44] = 0;
   out_4383613899688875359[45] = 0;
   out_4383613899688875359[46] = 0;
   out_4383613899688875359[47] = 0;
   out_4383613899688875359[48] = 0;
   out_4383613899688875359[49] = 0;
   out_4383613899688875359[50] = 1.0;
   out_4383613899688875359[51] = 0;
   out_4383613899688875359[52] = 0;
   out_4383613899688875359[53] = 0;
   out_4383613899688875359[54] = 0;
   out_4383613899688875359[55] = 0;
   out_4383613899688875359[56] = 0;
   out_4383613899688875359[57] = 0;
   out_4383613899688875359[58] = 0;
   out_4383613899688875359[59] = 0;
   out_4383613899688875359[60] = 1.0;
   out_4383613899688875359[61] = 0;
   out_4383613899688875359[62] = 0;
   out_4383613899688875359[63] = 0;
   out_4383613899688875359[64] = 0;
   out_4383613899688875359[65] = 0;
   out_4383613899688875359[66] = 0;
   out_4383613899688875359[67] = 0;
   out_4383613899688875359[68] = 0;
   out_4383613899688875359[69] = 0;
   out_4383613899688875359[70] = 1.0;
   out_4383613899688875359[71] = 0;
   out_4383613899688875359[72] = 0;
   out_4383613899688875359[73] = 0;
   out_4383613899688875359[74] = 0;
   out_4383613899688875359[75] = 0;
   out_4383613899688875359[76] = 0;
   out_4383613899688875359[77] = 0;
   out_4383613899688875359[78] = 0;
   out_4383613899688875359[79] = 0;
   out_4383613899688875359[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2672769777354396612) {
   out_2672769777354396612[0] = state[0];
   out_2672769777354396612[1] = state[1];
   out_2672769777354396612[2] = state[2];
   out_2672769777354396612[3] = state[3];
   out_2672769777354396612[4] = state[4];
   out_2672769777354396612[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2672769777354396612[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2672769777354396612[7] = state[7];
   out_2672769777354396612[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8981334264948615307) {
   out_8981334264948615307[0] = 1;
   out_8981334264948615307[1] = 0;
   out_8981334264948615307[2] = 0;
   out_8981334264948615307[3] = 0;
   out_8981334264948615307[4] = 0;
   out_8981334264948615307[5] = 0;
   out_8981334264948615307[6] = 0;
   out_8981334264948615307[7] = 0;
   out_8981334264948615307[8] = 0;
   out_8981334264948615307[9] = 0;
   out_8981334264948615307[10] = 1;
   out_8981334264948615307[11] = 0;
   out_8981334264948615307[12] = 0;
   out_8981334264948615307[13] = 0;
   out_8981334264948615307[14] = 0;
   out_8981334264948615307[15] = 0;
   out_8981334264948615307[16] = 0;
   out_8981334264948615307[17] = 0;
   out_8981334264948615307[18] = 0;
   out_8981334264948615307[19] = 0;
   out_8981334264948615307[20] = 1;
   out_8981334264948615307[21] = 0;
   out_8981334264948615307[22] = 0;
   out_8981334264948615307[23] = 0;
   out_8981334264948615307[24] = 0;
   out_8981334264948615307[25] = 0;
   out_8981334264948615307[26] = 0;
   out_8981334264948615307[27] = 0;
   out_8981334264948615307[28] = 0;
   out_8981334264948615307[29] = 0;
   out_8981334264948615307[30] = 1;
   out_8981334264948615307[31] = 0;
   out_8981334264948615307[32] = 0;
   out_8981334264948615307[33] = 0;
   out_8981334264948615307[34] = 0;
   out_8981334264948615307[35] = 0;
   out_8981334264948615307[36] = 0;
   out_8981334264948615307[37] = 0;
   out_8981334264948615307[38] = 0;
   out_8981334264948615307[39] = 0;
   out_8981334264948615307[40] = 1;
   out_8981334264948615307[41] = 0;
   out_8981334264948615307[42] = 0;
   out_8981334264948615307[43] = 0;
   out_8981334264948615307[44] = 0;
   out_8981334264948615307[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8981334264948615307[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8981334264948615307[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8981334264948615307[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8981334264948615307[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8981334264948615307[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8981334264948615307[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8981334264948615307[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8981334264948615307[53] = -9.8000000000000007*dt;
   out_8981334264948615307[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8981334264948615307[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8981334264948615307[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8981334264948615307[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8981334264948615307[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8981334264948615307[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8981334264948615307[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8981334264948615307[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8981334264948615307[62] = 0;
   out_8981334264948615307[63] = 0;
   out_8981334264948615307[64] = 0;
   out_8981334264948615307[65] = 0;
   out_8981334264948615307[66] = 0;
   out_8981334264948615307[67] = 0;
   out_8981334264948615307[68] = 0;
   out_8981334264948615307[69] = 0;
   out_8981334264948615307[70] = 1;
   out_8981334264948615307[71] = 0;
   out_8981334264948615307[72] = 0;
   out_8981334264948615307[73] = 0;
   out_8981334264948615307[74] = 0;
   out_8981334264948615307[75] = 0;
   out_8981334264948615307[76] = 0;
   out_8981334264948615307[77] = 0;
   out_8981334264948615307[78] = 0;
   out_8981334264948615307[79] = 0;
   out_8981334264948615307[80] = 1;
}
void h_25(double *state, double *unused, double *out_2323891550179960131) {
   out_2323891550179960131[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3130628365570768536) {
   out_3130628365570768536[0] = 0;
   out_3130628365570768536[1] = 0;
   out_3130628365570768536[2] = 0;
   out_3130628365570768536[3] = 0;
   out_3130628365570768536[4] = 0;
   out_3130628365570768536[5] = 0;
   out_3130628365570768536[6] = 1;
   out_3130628365570768536[7] = 0;
   out_3130628365570768536[8] = 0;
}
void h_24(double *state, double *unused, double *out_3959618100859615665) {
   out_3959618100859615665[0] = state[4];
   out_3959618100859615665[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3527273669924628031) {
   out_3527273669924628031[0] = 0;
   out_3527273669924628031[1] = 0;
   out_3527273669924628031[2] = 0;
   out_3527273669924628031[3] = 0;
   out_3527273669924628031[4] = 1;
   out_3527273669924628031[5] = 0;
   out_3527273669924628031[6] = 0;
   out_3527273669924628031[7] = 0;
   out_3527273669924628031[8] = 0;
   out_3527273669924628031[9] = 0;
   out_3527273669924628031[10] = 0;
   out_3527273669924628031[11] = 0;
   out_3527273669924628031[12] = 0;
   out_3527273669924628031[13] = 0;
   out_3527273669924628031[14] = 1;
   out_3527273669924628031[15] = 0;
   out_3527273669924628031[16] = 0;
   out_3527273669924628031[17] = 0;
}
void h_30(double *state, double *unused, double *out_6422194848459331995) {
   out_6422194848459331995[0] = state[4];
}
void H_30(double *state, double *unused, double *out_612295407063519909) {
   out_612295407063519909[0] = 0;
   out_612295407063519909[1] = 0;
   out_612295407063519909[2] = 0;
   out_612295407063519909[3] = 0;
   out_612295407063519909[4] = 1;
   out_612295407063519909[5] = 0;
   out_612295407063519909[6] = 0;
   out_612295407063519909[7] = 0;
   out_612295407063519909[8] = 0;
}
void h_26(double *state, double *unused, double *out_4132346778849017835) {
   out_4132346778849017835[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6872131684444824760) {
   out_6872131684444824760[0] = 0;
   out_6872131684444824760[1] = 0;
   out_6872131684444824760[2] = 0;
   out_6872131684444824760[3] = 0;
   out_6872131684444824760[4] = 0;
   out_6872131684444824760[5] = 0;
   out_6872131684444824760[6] = 0;
   out_6872131684444824760[7] = 1;
   out_6872131684444824760[8] = 0;
}
void h_27(double *state, double *unused, double *out_6472417914086037009) {
   out_6472417914086037009[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1611298664120423308) {
   out_1611298664120423308[0] = 0;
   out_1611298664120423308[1] = 0;
   out_1611298664120423308[2] = 0;
   out_1611298664120423308[3] = 1;
   out_1611298664120423308[4] = 0;
   out_1611298664120423308[5] = 0;
   out_1611298664120423308[6] = 0;
   out_1611298664120423308[7] = 0;
   out_1611298664120423308[8] = 0;
}
void h_29(double *state, double *unused, double *out_8112642683195012955) {
   out_8112642683195012955[0] = state[1];
}
void H_29(double *state, double *unused, double *out_102064062749127725) {
   out_102064062749127725[0] = 0;
   out_102064062749127725[1] = 1;
   out_102064062749127725[2] = 0;
   out_102064062749127725[3] = 0;
   out_102064062749127725[4] = 0;
   out_102064062749127725[5] = 0;
   out_102064062749127725[6] = 0;
   out_102064062749127725[7] = 0;
   out_102064062749127725[8] = 0;
}
void h_28(double *state, double *unused, double *out_738491572168184502) {
   out_738491572168184502[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5184463079818658299) {
   out_5184463079818658299[0] = 1;
   out_5184463079818658299[1] = 0;
   out_5184463079818658299[2] = 0;
   out_5184463079818658299[3] = 0;
   out_5184463079818658299[4] = 0;
   out_5184463079818658299[5] = 0;
   out_5184463079818658299[6] = 0;
   out_5184463079818658299[7] = 0;
   out_5184463079818658299[8] = 0;
}
void h_31(double *state, double *unused, double *out_8241579872775542300) {
   out_8241579872775542300[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7498339786678176236) {
   out_7498339786678176236[0] = 0;
   out_7498339786678176236[1] = 0;
   out_7498339786678176236[2] = 0;
   out_7498339786678176236[3] = 0;
   out_7498339786678176236[4] = 0;
   out_7498339786678176236[5] = 0;
   out_7498339786678176236[6] = 0;
   out_7498339786678176236[7] = 0;
   out_7498339786678176236[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7418791751555442547) {
  err_fun(nom_x, delta_x, out_7418791751555442547);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4675497126535007434) {
  inv_err_fun(nom_x, true_x, out_4675497126535007434);
}
void car_H_mod_fun(double *state, double *out_4383613899688875359) {
  H_mod_fun(state, out_4383613899688875359);
}
void car_f_fun(double *state, double dt, double *out_2672769777354396612) {
  f_fun(state,  dt, out_2672769777354396612);
}
void car_F_fun(double *state, double dt, double *out_8981334264948615307) {
  F_fun(state,  dt, out_8981334264948615307);
}
void car_h_25(double *state, double *unused, double *out_2323891550179960131) {
  h_25(state, unused, out_2323891550179960131);
}
void car_H_25(double *state, double *unused, double *out_3130628365570768536) {
  H_25(state, unused, out_3130628365570768536);
}
void car_h_24(double *state, double *unused, double *out_3959618100859615665) {
  h_24(state, unused, out_3959618100859615665);
}
void car_H_24(double *state, double *unused, double *out_3527273669924628031) {
  H_24(state, unused, out_3527273669924628031);
}
void car_h_30(double *state, double *unused, double *out_6422194848459331995) {
  h_30(state, unused, out_6422194848459331995);
}
void car_H_30(double *state, double *unused, double *out_612295407063519909) {
  H_30(state, unused, out_612295407063519909);
}
void car_h_26(double *state, double *unused, double *out_4132346778849017835) {
  h_26(state, unused, out_4132346778849017835);
}
void car_H_26(double *state, double *unused, double *out_6872131684444824760) {
  H_26(state, unused, out_6872131684444824760);
}
void car_h_27(double *state, double *unused, double *out_6472417914086037009) {
  h_27(state, unused, out_6472417914086037009);
}
void car_H_27(double *state, double *unused, double *out_1611298664120423308) {
  H_27(state, unused, out_1611298664120423308);
}
void car_h_29(double *state, double *unused, double *out_8112642683195012955) {
  h_29(state, unused, out_8112642683195012955);
}
void car_H_29(double *state, double *unused, double *out_102064062749127725) {
  H_29(state, unused, out_102064062749127725);
}
void car_h_28(double *state, double *unused, double *out_738491572168184502) {
  h_28(state, unused, out_738491572168184502);
}
void car_H_28(double *state, double *unused, double *out_5184463079818658299) {
  H_28(state, unused, out_5184463079818658299);
}
void car_h_31(double *state, double *unused, double *out_8241579872775542300) {
  h_31(state, unused, out_8241579872775542300);
}
void car_H_31(double *state, double *unused, double *out_7498339786678176236) {
  H_31(state, unused, out_7498339786678176236);
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
