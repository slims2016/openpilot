#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_8490845275103024580);
void live_err_fun(double *nom_x, double *delta_x, double *out_1062821592924566878);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5806722205139150385);
void live_H_mod_fun(double *state, double *out_1066851157255734461);
void live_f_fun(double *state, double dt, double *out_1160212763761334837);
void live_F_fun(double *state, double dt, double *out_7746574922297630244);
void live_h_4(double *state, double *unused, double *out_3676912514419079076);
void live_H_4(double *state, double *unused, double *out_2326848626800457721);
void live_h_9(double *state, double *unused, double *out_4062813202222779713);
void live_H_9(double *state, double *unused, double *out_8832676511644646425);
void live_h_10(double *state, double *unused, double *out_7221679492932465247);
void live_H_10(double *state, double *unused, double *out_1516364932739886309);
void live_h_12(double *state, double *unused, double *out_1271193297187319263);
void live_H_12(double *state, double *unused, double *out_4054409750242275275);
void live_h_35(double *state, double *unused, double *out_6052933428093829024);
void live_H_35(double *state, double *unused, double *out_8354876006552118391);
void live_h_32(double *state, double *unused, double *out_8348184778349246700);
void live_H_32(double *state, double *unused, double *out_8667497701439563419);
void live_h_13(double *state, double *unused, double *out_3319014371751258535);
void live_H_13(double *state, double *unused, double *out_7894801291823691667);
void live_h_14(double *state, double *unused, double *out_4062813202222779713);
void live_H_14(double *state, double *unused, double *out_8832676511644646425);
void live_h_33(double *state, double *unused, double *out_7699026587773598503);
void live_H_33(double *state, double *unused, double *out_2556647096262772090);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}