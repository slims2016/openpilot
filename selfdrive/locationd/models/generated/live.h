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
void live_H(double *in_vec, double *out_8945704586164018613);
void live_err_fun(double *nom_x, double *delta_x, double *out_8370097340763738192);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1561714653313430456);
void live_H_mod_fun(double *state, double *out_619228651928883812);
void live_f_fun(double *state, double dt, double *out_7848720022958012510);
void live_F_fun(double *state, double dt, double *out_350033378085274248);
void live_h_4(double *state, double *unused, double *out_6806577712686989055);
void live_H_4(double *state, double *unused, double *out_5600576627277883540);
void live_h_9(double *state, double *unused, double *out_8013092661027487392);
void live_H_9(double *state, double *unused, double *out_5558948511167220606);
void live_h_10(double *state, double *unused, double *out_3244487682903101191);
void live_H_10(double *state, double *unused, double *out_8030959225542197370);
void live_h_12(double *state, double *unused, double *out_2391008662114364333);
void live_H_12(double *state, double *unused, double *out_7826711038399706281);
void live_h_35(double *state, double *unused, double *out_7297975473935584261);
void live_H_35(double *state, double *unused, double *out_8967238684650490916);
void live_h_32(double *state, double *unused, double *out_5218610459212142195);
void live_H_32(double *state, double *unused, double *out_9098403469591231385);
void live_h_13(double *state, double *unused, double *out_7462688905416857194);
void live_H_13(double *state, double *unused, double *out_6584715849578254444);
void live_h_14(double *state, double *unused, double *out_8013092661027487392);
void live_H_14(double *state, double *unused, double *out_5558948511167220606);
void live_h_33(double *state, double *unused, double *out_298940871667614646);
void live_H_33(double *state, double *unused, double *out_6328948384420203096);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}