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
void car_err_fun(double *nom_x, double *delta_x, double *out_7418791751555442547);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4675497126535007434);
void car_H_mod_fun(double *state, double *out_4383613899688875359);
void car_f_fun(double *state, double dt, double *out_2672769777354396612);
void car_F_fun(double *state, double dt, double *out_8981334264948615307);
void car_h_25(double *state, double *unused, double *out_2323891550179960131);
void car_H_25(double *state, double *unused, double *out_3130628365570768536);
void car_h_24(double *state, double *unused, double *out_3959618100859615665);
void car_H_24(double *state, double *unused, double *out_3527273669924628031);
void car_h_30(double *state, double *unused, double *out_6422194848459331995);
void car_H_30(double *state, double *unused, double *out_612295407063519909);
void car_h_26(double *state, double *unused, double *out_4132346778849017835);
void car_H_26(double *state, double *unused, double *out_6872131684444824760);
void car_h_27(double *state, double *unused, double *out_6472417914086037009);
void car_H_27(double *state, double *unused, double *out_1611298664120423308);
void car_h_29(double *state, double *unused, double *out_8112642683195012955);
void car_H_29(double *state, double *unused, double *out_102064062749127725);
void car_h_28(double *state, double *unused, double *out_738491572168184502);
void car_H_28(double *state, double *unused, double *out_5184463079818658299);
void car_h_31(double *state, double *unused, double *out_8241579872775542300);
void car_H_31(double *state, double *unused, double *out_7498339786678176236);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}