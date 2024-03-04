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
void car_err_fun(double *nom_x, double *delta_x, double *out_8353448122821896198);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5467468061494965852);
void car_H_mod_fun(double *state, double *out_3378346308799840044);
void car_f_fun(double *state, double dt, double *out_4900003475537959945);
void car_F_fun(double *state, double dt, double *out_8563448028746898306);
void car_h_25(double *state, double *unused, double *out_2164074651324699666);
void car_H_25(double *state, double *unused, double *out_3891080028064799851);
void car_h_24(double *state, double *unused, double *out_1055683949582273217);
void car_H_24(double *state, double *unused, double *out_1920305602957977114);
void car_h_30(double *state, double *unused, double *out_1473374876929322684);
void car_H_30(double *state, double *unused, double *out_4020418975208039921);
void car_h_26(double *state, double *unused, double *out_8908518311358023497);
void car_H_26(double *state, double *unused, double *out_7632583346938856075);
void car_h_27(double *state, double *unused, double *out_3854522486060380626);
void car_H_27(double *state, double *unused, double *out_6195182287008464832);
void car_h_29(double *state, double *unused, double *out_906796495704895665);
void car_H_29(double *state, double *unused, double *out_3510187630893647737);
void car_h_28(double *state, double *unused, double *out_6144370555670694786);
void car_H_28(double *state, double *unused, double *out_5944914742312689614);
void car_h_31(double *state, double *unused, double *out_8401396771630802765);
void car_H_31(double *state, double *unused, double *out_3860434066187839423);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}