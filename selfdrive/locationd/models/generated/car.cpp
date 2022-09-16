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
 *                      Code generated with SymPy 1.10.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5911337756844115743) {
   out_5911337756844115743[0] = delta_x[0] + nom_x[0];
   out_5911337756844115743[1] = delta_x[1] + nom_x[1];
   out_5911337756844115743[2] = delta_x[2] + nom_x[2];
   out_5911337756844115743[3] = delta_x[3] + nom_x[3];
   out_5911337756844115743[4] = delta_x[4] + nom_x[4];
   out_5911337756844115743[5] = delta_x[5] + nom_x[5];
   out_5911337756844115743[6] = delta_x[6] + nom_x[6];
   out_5911337756844115743[7] = delta_x[7] + nom_x[7];
   out_5911337756844115743[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5485660850319610450) {
   out_5485660850319610450[0] = -nom_x[0] + true_x[0];
   out_5485660850319610450[1] = -nom_x[1] + true_x[1];
   out_5485660850319610450[2] = -nom_x[2] + true_x[2];
   out_5485660850319610450[3] = -nom_x[3] + true_x[3];
   out_5485660850319610450[4] = -nom_x[4] + true_x[4];
   out_5485660850319610450[5] = -nom_x[5] + true_x[5];
   out_5485660850319610450[6] = -nom_x[6] + true_x[6];
   out_5485660850319610450[7] = -nom_x[7] + true_x[7];
   out_5485660850319610450[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8085107026912875082) {
   out_8085107026912875082[0] = 1.0;
   out_8085107026912875082[1] = 0;
   out_8085107026912875082[2] = 0;
   out_8085107026912875082[3] = 0;
   out_8085107026912875082[4] = 0;
   out_8085107026912875082[5] = 0;
   out_8085107026912875082[6] = 0;
   out_8085107026912875082[7] = 0;
   out_8085107026912875082[8] = 0;
   out_8085107026912875082[9] = 0;
   out_8085107026912875082[10] = 1.0;
   out_8085107026912875082[11] = 0;
   out_8085107026912875082[12] = 0;
   out_8085107026912875082[13] = 0;
   out_8085107026912875082[14] = 0;
   out_8085107026912875082[15] = 0;
   out_8085107026912875082[16] = 0;
   out_8085107026912875082[17] = 0;
   out_8085107026912875082[18] = 0;
   out_8085107026912875082[19] = 0;
   out_8085107026912875082[20] = 1.0;
   out_8085107026912875082[21] = 0;
   out_8085107026912875082[22] = 0;
   out_8085107026912875082[23] = 0;
   out_8085107026912875082[24] = 0;
   out_8085107026912875082[25] = 0;
   out_8085107026912875082[26] = 0;
   out_8085107026912875082[27] = 0;
   out_8085107026912875082[28] = 0;
   out_8085107026912875082[29] = 0;
   out_8085107026912875082[30] = 1.0;
   out_8085107026912875082[31] = 0;
   out_8085107026912875082[32] = 0;
   out_8085107026912875082[33] = 0;
   out_8085107026912875082[34] = 0;
   out_8085107026912875082[35] = 0;
   out_8085107026912875082[36] = 0;
   out_8085107026912875082[37] = 0;
   out_8085107026912875082[38] = 0;
   out_8085107026912875082[39] = 0;
   out_8085107026912875082[40] = 1.0;
   out_8085107026912875082[41] = 0;
   out_8085107026912875082[42] = 0;
   out_8085107026912875082[43] = 0;
   out_8085107026912875082[44] = 0;
   out_8085107026912875082[45] = 0;
   out_8085107026912875082[46] = 0;
   out_8085107026912875082[47] = 0;
   out_8085107026912875082[48] = 0;
   out_8085107026912875082[49] = 0;
   out_8085107026912875082[50] = 1.0;
   out_8085107026912875082[51] = 0;
   out_8085107026912875082[52] = 0;
   out_8085107026912875082[53] = 0;
   out_8085107026912875082[54] = 0;
   out_8085107026912875082[55] = 0;
   out_8085107026912875082[56] = 0;
   out_8085107026912875082[57] = 0;
   out_8085107026912875082[58] = 0;
   out_8085107026912875082[59] = 0;
   out_8085107026912875082[60] = 1.0;
   out_8085107026912875082[61] = 0;
   out_8085107026912875082[62] = 0;
   out_8085107026912875082[63] = 0;
   out_8085107026912875082[64] = 0;
   out_8085107026912875082[65] = 0;
   out_8085107026912875082[66] = 0;
   out_8085107026912875082[67] = 0;
   out_8085107026912875082[68] = 0;
   out_8085107026912875082[69] = 0;
   out_8085107026912875082[70] = 1.0;
   out_8085107026912875082[71] = 0;
   out_8085107026912875082[72] = 0;
   out_8085107026912875082[73] = 0;
   out_8085107026912875082[74] = 0;
   out_8085107026912875082[75] = 0;
   out_8085107026912875082[76] = 0;
   out_8085107026912875082[77] = 0;
   out_8085107026912875082[78] = 0;
   out_8085107026912875082[79] = 0;
   out_8085107026912875082[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5894352474097094406) {
   out_5894352474097094406[0] = state[0];
   out_5894352474097094406[1] = state[1];
   out_5894352474097094406[2] = state[2];
   out_5894352474097094406[3] = state[3];
   out_5894352474097094406[4] = state[4];
   out_5894352474097094406[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5894352474097094406[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5894352474097094406[7] = state[7];
   out_5894352474097094406[8] = state[8];
}
void F_fun(double *state, double dt, double *out_817662128394459820) {
   out_817662128394459820[0] = 1;
   out_817662128394459820[1] = 0;
   out_817662128394459820[2] = 0;
   out_817662128394459820[3] = 0;
   out_817662128394459820[4] = 0;
   out_817662128394459820[5] = 0;
   out_817662128394459820[6] = 0;
   out_817662128394459820[7] = 0;
   out_817662128394459820[8] = 0;
   out_817662128394459820[9] = 0;
   out_817662128394459820[10] = 1;
   out_817662128394459820[11] = 0;
   out_817662128394459820[12] = 0;
   out_817662128394459820[13] = 0;
   out_817662128394459820[14] = 0;
   out_817662128394459820[15] = 0;
   out_817662128394459820[16] = 0;
   out_817662128394459820[17] = 0;
   out_817662128394459820[18] = 0;
   out_817662128394459820[19] = 0;
   out_817662128394459820[20] = 1;
   out_817662128394459820[21] = 0;
   out_817662128394459820[22] = 0;
   out_817662128394459820[23] = 0;
   out_817662128394459820[24] = 0;
   out_817662128394459820[25] = 0;
   out_817662128394459820[26] = 0;
   out_817662128394459820[27] = 0;
   out_817662128394459820[28] = 0;
   out_817662128394459820[29] = 0;
   out_817662128394459820[30] = 1;
   out_817662128394459820[31] = 0;
   out_817662128394459820[32] = 0;
   out_817662128394459820[33] = 0;
   out_817662128394459820[34] = 0;
   out_817662128394459820[35] = 0;
   out_817662128394459820[36] = 0;
   out_817662128394459820[37] = 0;
   out_817662128394459820[38] = 0;
   out_817662128394459820[39] = 0;
   out_817662128394459820[40] = 1;
   out_817662128394459820[41] = 0;
   out_817662128394459820[42] = 0;
   out_817662128394459820[43] = 0;
   out_817662128394459820[44] = 0;
   out_817662128394459820[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_817662128394459820[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_817662128394459820[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_817662128394459820[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_817662128394459820[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_817662128394459820[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_817662128394459820[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_817662128394459820[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_817662128394459820[53] = -9.8000000000000007*dt;
   out_817662128394459820[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_817662128394459820[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_817662128394459820[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_817662128394459820[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_817662128394459820[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_817662128394459820[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_817662128394459820[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_817662128394459820[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_817662128394459820[62] = 0;
   out_817662128394459820[63] = 0;
   out_817662128394459820[64] = 0;
   out_817662128394459820[65] = 0;
   out_817662128394459820[66] = 0;
   out_817662128394459820[67] = 0;
   out_817662128394459820[68] = 0;
   out_817662128394459820[69] = 0;
   out_817662128394459820[70] = 1;
   out_817662128394459820[71] = 0;
   out_817662128394459820[72] = 0;
   out_817662128394459820[73] = 0;
   out_817662128394459820[74] = 0;
   out_817662128394459820[75] = 0;
   out_817662128394459820[76] = 0;
   out_817662128394459820[77] = 0;
   out_817662128394459820[78] = 0;
   out_817662128394459820[79] = 0;
   out_817662128394459820[80] = 1;
}
void h_25(double *state, double *unused, double *out_3753649504837398471) {
   out_3753649504837398471[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7477841514035474124) {
   out_7477841514035474124[0] = 0;
   out_7477841514035474124[1] = 0;
   out_7477841514035474124[2] = 0;
   out_7477841514035474124[3] = 0;
   out_7477841514035474124[4] = 0;
   out_7477841514035474124[5] = 0;
   out_7477841514035474124[6] = 1;
   out_7477841514035474124[7] = 0;
   out_7477841514035474124[8] = 0;
}
void h_24(double *state, double *unused, double *out_5015391030761451135) {
   out_5015391030761451135[0] = state[4];
   out_5015391030761451135[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5305191915029974558) {
   out_5305191915029974558[0] = 0;
   out_5305191915029974558[1] = 0;
   out_5305191915029974558[2] = 0;
   out_5305191915029974558[3] = 0;
   out_5305191915029974558[4] = 1;
   out_5305191915029974558[5] = 0;
   out_5305191915029974558[6] = 0;
   out_5305191915029974558[7] = 0;
   out_5305191915029974558[8] = 0;
   out_5305191915029974558[9] = 0;
   out_5305191915029974558[10] = 0;
   out_5305191915029974558[11] = 0;
   out_5305191915029974558[12] = 0;
   out_5305191915029974558[13] = 0;
   out_5305191915029974558[14] = 1;
   out_5305191915029974558[15] = 0;
   out_5305191915029974558[16] = 0;
   out_5305191915029974558[17] = 0;
}
void h_30(double *state, double *unused, double *out_7922259342521802209) {
   out_7922259342521802209[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2950145183907865926) {
   out_2950145183907865926[0] = 0;
   out_2950145183907865926[1] = 0;
   out_2950145183907865926[2] = 0;
   out_2950145183907865926[3] = 0;
   out_2950145183907865926[4] = 1;
   out_2950145183907865926[5] = 0;
   out_2950145183907865926[6] = 0;
   out_2950145183907865926[7] = 0;
   out_2950145183907865926[8] = 0;
}
void h_26(double *state, double *unused, double *out_2920586074860956564) {
   out_2920586074860956564[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3736338195161417900) {
   out_3736338195161417900[0] = 0;
   out_3736338195161417900[1] = 0;
   out_3736338195161417900[2] = 0;
   out_3736338195161417900[3] = 0;
   out_3736338195161417900[4] = 0;
   out_3736338195161417900[5] = 0;
   out_3736338195161417900[6] = 0;
   out_3736338195161417900[7] = 1;
   out_3736338195161417900[8] = 0;
}
void h_27(double *state, double *unused, double *out_4199319549773751608) {
   out_4199319549773751608[0] = state[3];
}
void H_27(double *state, double *unused, double *out_775381872107441015) {
   out_775381872107441015[0] = 0;
   out_775381872107441015[1] = 0;
   out_775381872107441015[2] = 0;
   out_775381872107441015[3] = 1;
   out_775381872107441015[4] = 0;
   out_775381872107441015[5] = 0;
   out_775381872107441015[6] = 0;
   out_775381872107441015[7] = 0;
   out_775381872107441015[8] = 0;
}
void h_29(double *state, double *unused, double *out_2164274428528064253) {
   out_2164274428528064253[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3460376528222258110) {
   out_3460376528222258110[0] = 0;
   out_3460376528222258110[1] = 1;
   out_3460376528222258110[2] = 0;
   out_3460376528222258110[3] = 0;
   out_3460376528222258110[4] = 0;
   out_3460376528222258110[5] = 0;
   out_3460376528222258110[6] = 0;
   out_3460376528222258110[7] = 0;
   out_3460376528222258110[8] = 0;
}
void h_28(double *state, double *unused, double *out_8040266736455178953) {
   out_8040266736455178953[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5424006799787584361) {
   out_5424006799787584361[0] = 1;
   out_5424006799787584361[1] = 0;
   out_5424006799787584361[2] = 0;
   out_5424006799787584361[3] = 0;
   out_5424006799787584361[4] = 0;
   out_5424006799787584361[5] = 0;
   out_5424006799787584361[6] = 0;
   out_5424006799787584361[7] = 0;
   out_5424006799787584361[8] = 0;
}
void h_31(double *state, double *unused, double *out_1661961508788404114) {
   out_1661961508788404114[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3110130092928066424) {
   out_3110130092928066424[0] = 0;
   out_3110130092928066424[1] = 0;
   out_3110130092928066424[2] = 0;
   out_3110130092928066424[3] = 0;
   out_3110130092928066424[4] = 0;
   out_3110130092928066424[5] = 0;
   out_3110130092928066424[6] = 0;
   out_3110130092928066424[7] = 0;
   out_3110130092928066424[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5911337756844115743) {
  err_fun(nom_x, delta_x, out_5911337756844115743);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5485660850319610450) {
  inv_err_fun(nom_x, true_x, out_5485660850319610450);
}
void car_H_mod_fun(double *state, double *out_8085107026912875082) {
  H_mod_fun(state, out_8085107026912875082);
}
void car_f_fun(double *state, double dt, double *out_5894352474097094406) {
  f_fun(state,  dt, out_5894352474097094406);
}
void car_F_fun(double *state, double dt, double *out_817662128394459820) {
  F_fun(state,  dt, out_817662128394459820);
}
void car_h_25(double *state, double *unused, double *out_3753649504837398471) {
  h_25(state, unused, out_3753649504837398471);
}
void car_H_25(double *state, double *unused, double *out_7477841514035474124) {
  H_25(state, unused, out_7477841514035474124);
}
void car_h_24(double *state, double *unused, double *out_5015391030761451135) {
  h_24(state, unused, out_5015391030761451135);
}
void car_H_24(double *state, double *unused, double *out_5305191915029974558) {
  H_24(state, unused, out_5305191915029974558);
}
void car_h_30(double *state, double *unused, double *out_7922259342521802209) {
  h_30(state, unused, out_7922259342521802209);
}
void car_H_30(double *state, double *unused, double *out_2950145183907865926) {
  H_30(state, unused, out_2950145183907865926);
}
void car_h_26(double *state, double *unused, double *out_2920586074860956564) {
  h_26(state, unused, out_2920586074860956564);
}
void car_H_26(double *state, double *unused, double *out_3736338195161417900) {
  H_26(state, unused, out_3736338195161417900);
}
void car_h_27(double *state, double *unused, double *out_4199319549773751608) {
  h_27(state, unused, out_4199319549773751608);
}
void car_H_27(double *state, double *unused, double *out_775381872107441015) {
  H_27(state, unused, out_775381872107441015);
}
void car_h_29(double *state, double *unused, double *out_2164274428528064253) {
  h_29(state, unused, out_2164274428528064253);
}
void car_H_29(double *state, double *unused, double *out_3460376528222258110) {
  H_29(state, unused, out_3460376528222258110);
}
void car_h_28(double *state, double *unused, double *out_8040266736455178953) {
  h_28(state, unused, out_8040266736455178953);
}
void car_H_28(double *state, double *unused, double *out_5424006799787584361) {
  H_28(state, unused, out_5424006799787584361);
}
void car_h_31(double *state, double *unused, double *out_1661961508788404114) {
  h_31(state, unused, out_1661961508788404114);
}
void car_H_31(double *state, double *unused, double *out_3110130092928066424) {
  H_31(state, unused, out_3110130092928066424);
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

ekf_init(car);
