#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2540016638677114272) {
   out_2540016638677114272[0] = delta_x[0] + nom_x[0];
   out_2540016638677114272[1] = delta_x[1] + nom_x[1];
   out_2540016638677114272[2] = delta_x[2] + nom_x[2];
   out_2540016638677114272[3] = delta_x[3] + nom_x[3];
   out_2540016638677114272[4] = delta_x[4] + nom_x[4];
   out_2540016638677114272[5] = delta_x[5] + nom_x[5];
   out_2540016638677114272[6] = delta_x[6] + nom_x[6];
   out_2540016638677114272[7] = delta_x[7] + nom_x[7];
   out_2540016638677114272[8] = delta_x[8] + nom_x[8];
   out_2540016638677114272[9] = delta_x[9] + nom_x[9];
   out_2540016638677114272[10] = delta_x[10] + nom_x[10];
   out_2540016638677114272[11] = delta_x[11] + nom_x[11];
   out_2540016638677114272[12] = delta_x[12] + nom_x[12];
   out_2540016638677114272[13] = delta_x[13] + nom_x[13];
   out_2540016638677114272[14] = delta_x[14] + nom_x[14];
   out_2540016638677114272[15] = delta_x[15] + nom_x[15];
   out_2540016638677114272[16] = delta_x[16] + nom_x[16];
   out_2540016638677114272[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4211094856349578415) {
   out_4211094856349578415[0] = -nom_x[0] + true_x[0];
   out_4211094856349578415[1] = -nom_x[1] + true_x[1];
   out_4211094856349578415[2] = -nom_x[2] + true_x[2];
   out_4211094856349578415[3] = -nom_x[3] + true_x[3];
   out_4211094856349578415[4] = -nom_x[4] + true_x[4];
   out_4211094856349578415[5] = -nom_x[5] + true_x[5];
   out_4211094856349578415[6] = -nom_x[6] + true_x[6];
   out_4211094856349578415[7] = -nom_x[7] + true_x[7];
   out_4211094856349578415[8] = -nom_x[8] + true_x[8];
   out_4211094856349578415[9] = -nom_x[9] + true_x[9];
   out_4211094856349578415[10] = -nom_x[10] + true_x[10];
   out_4211094856349578415[11] = -nom_x[11] + true_x[11];
   out_4211094856349578415[12] = -nom_x[12] + true_x[12];
   out_4211094856349578415[13] = -nom_x[13] + true_x[13];
   out_4211094856349578415[14] = -nom_x[14] + true_x[14];
   out_4211094856349578415[15] = -nom_x[15] + true_x[15];
   out_4211094856349578415[16] = -nom_x[16] + true_x[16];
   out_4211094856349578415[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_4654741660211559160) {
   out_4654741660211559160[0] = 1.0;
   out_4654741660211559160[1] = 0.0;
   out_4654741660211559160[2] = 0.0;
   out_4654741660211559160[3] = 0.0;
   out_4654741660211559160[4] = 0.0;
   out_4654741660211559160[5] = 0.0;
   out_4654741660211559160[6] = 0.0;
   out_4654741660211559160[7] = 0.0;
   out_4654741660211559160[8] = 0.0;
   out_4654741660211559160[9] = 0.0;
   out_4654741660211559160[10] = 0.0;
   out_4654741660211559160[11] = 0.0;
   out_4654741660211559160[12] = 0.0;
   out_4654741660211559160[13] = 0.0;
   out_4654741660211559160[14] = 0.0;
   out_4654741660211559160[15] = 0.0;
   out_4654741660211559160[16] = 0.0;
   out_4654741660211559160[17] = 0.0;
   out_4654741660211559160[18] = 0.0;
   out_4654741660211559160[19] = 1.0;
   out_4654741660211559160[20] = 0.0;
   out_4654741660211559160[21] = 0.0;
   out_4654741660211559160[22] = 0.0;
   out_4654741660211559160[23] = 0.0;
   out_4654741660211559160[24] = 0.0;
   out_4654741660211559160[25] = 0.0;
   out_4654741660211559160[26] = 0.0;
   out_4654741660211559160[27] = 0.0;
   out_4654741660211559160[28] = 0.0;
   out_4654741660211559160[29] = 0.0;
   out_4654741660211559160[30] = 0.0;
   out_4654741660211559160[31] = 0.0;
   out_4654741660211559160[32] = 0.0;
   out_4654741660211559160[33] = 0.0;
   out_4654741660211559160[34] = 0.0;
   out_4654741660211559160[35] = 0.0;
   out_4654741660211559160[36] = 0.0;
   out_4654741660211559160[37] = 0.0;
   out_4654741660211559160[38] = 1.0;
   out_4654741660211559160[39] = 0.0;
   out_4654741660211559160[40] = 0.0;
   out_4654741660211559160[41] = 0.0;
   out_4654741660211559160[42] = 0.0;
   out_4654741660211559160[43] = 0.0;
   out_4654741660211559160[44] = 0.0;
   out_4654741660211559160[45] = 0.0;
   out_4654741660211559160[46] = 0.0;
   out_4654741660211559160[47] = 0.0;
   out_4654741660211559160[48] = 0.0;
   out_4654741660211559160[49] = 0.0;
   out_4654741660211559160[50] = 0.0;
   out_4654741660211559160[51] = 0.0;
   out_4654741660211559160[52] = 0.0;
   out_4654741660211559160[53] = 0.0;
   out_4654741660211559160[54] = 0.0;
   out_4654741660211559160[55] = 0.0;
   out_4654741660211559160[56] = 0.0;
   out_4654741660211559160[57] = 1.0;
   out_4654741660211559160[58] = 0.0;
   out_4654741660211559160[59] = 0.0;
   out_4654741660211559160[60] = 0.0;
   out_4654741660211559160[61] = 0.0;
   out_4654741660211559160[62] = 0.0;
   out_4654741660211559160[63] = 0.0;
   out_4654741660211559160[64] = 0.0;
   out_4654741660211559160[65] = 0.0;
   out_4654741660211559160[66] = 0.0;
   out_4654741660211559160[67] = 0.0;
   out_4654741660211559160[68] = 0.0;
   out_4654741660211559160[69] = 0.0;
   out_4654741660211559160[70] = 0.0;
   out_4654741660211559160[71] = 0.0;
   out_4654741660211559160[72] = 0.0;
   out_4654741660211559160[73] = 0.0;
   out_4654741660211559160[74] = 0.0;
   out_4654741660211559160[75] = 0.0;
   out_4654741660211559160[76] = 1.0;
   out_4654741660211559160[77] = 0.0;
   out_4654741660211559160[78] = 0.0;
   out_4654741660211559160[79] = 0.0;
   out_4654741660211559160[80] = 0.0;
   out_4654741660211559160[81] = 0.0;
   out_4654741660211559160[82] = 0.0;
   out_4654741660211559160[83] = 0.0;
   out_4654741660211559160[84] = 0.0;
   out_4654741660211559160[85] = 0.0;
   out_4654741660211559160[86] = 0.0;
   out_4654741660211559160[87] = 0.0;
   out_4654741660211559160[88] = 0.0;
   out_4654741660211559160[89] = 0.0;
   out_4654741660211559160[90] = 0.0;
   out_4654741660211559160[91] = 0.0;
   out_4654741660211559160[92] = 0.0;
   out_4654741660211559160[93] = 0.0;
   out_4654741660211559160[94] = 0.0;
   out_4654741660211559160[95] = 1.0;
   out_4654741660211559160[96] = 0.0;
   out_4654741660211559160[97] = 0.0;
   out_4654741660211559160[98] = 0.0;
   out_4654741660211559160[99] = 0.0;
   out_4654741660211559160[100] = 0.0;
   out_4654741660211559160[101] = 0.0;
   out_4654741660211559160[102] = 0.0;
   out_4654741660211559160[103] = 0.0;
   out_4654741660211559160[104] = 0.0;
   out_4654741660211559160[105] = 0.0;
   out_4654741660211559160[106] = 0.0;
   out_4654741660211559160[107] = 0.0;
   out_4654741660211559160[108] = 0.0;
   out_4654741660211559160[109] = 0.0;
   out_4654741660211559160[110] = 0.0;
   out_4654741660211559160[111] = 0.0;
   out_4654741660211559160[112] = 0.0;
   out_4654741660211559160[113] = 0.0;
   out_4654741660211559160[114] = 1.0;
   out_4654741660211559160[115] = 0.0;
   out_4654741660211559160[116] = 0.0;
   out_4654741660211559160[117] = 0.0;
   out_4654741660211559160[118] = 0.0;
   out_4654741660211559160[119] = 0.0;
   out_4654741660211559160[120] = 0.0;
   out_4654741660211559160[121] = 0.0;
   out_4654741660211559160[122] = 0.0;
   out_4654741660211559160[123] = 0.0;
   out_4654741660211559160[124] = 0.0;
   out_4654741660211559160[125] = 0.0;
   out_4654741660211559160[126] = 0.0;
   out_4654741660211559160[127] = 0.0;
   out_4654741660211559160[128] = 0.0;
   out_4654741660211559160[129] = 0.0;
   out_4654741660211559160[130] = 0.0;
   out_4654741660211559160[131] = 0.0;
   out_4654741660211559160[132] = 0.0;
   out_4654741660211559160[133] = 1.0;
   out_4654741660211559160[134] = 0.0;
   out_4654741660211559160[135] = 0.0;
   out_4654741660211559160[136] = 0.0;
   out_4654741660211559160[137] = 0.0;
   out_4654741660211559160[138] = 0.0;
   out_4654741660211559160[139] = 0.0;
   out_4654741660211559160[140] = 0.0;
   out_4654741660211559160[141] = 0.0;
   out_4654741660211559160[142] = 0.0;
   out_4654741660211559160[143] = 0.0;
   out_4654741660211559160[144] = 0.0;
   out_4654741660211559160[145] = 0.0;
   out_4654741660211559160[146] = 0.0;
   out_4654741660211559160[147] = 0.0;
   out_4654741660211559160[148] = 0.0;
   out_4654741660211559160[149] = 0.0;
   out_4654741660211559160[150] = 0.0;
   out_4654741660211559160[151] = 0.0;
   out_4654741660211559160[152] = 1.0;
   out_4654741660211559160[153] = 0.0;
   out_4654741660211559160[154] = 0.0;
   out_4654741660211559160[155] = 0.0;
   out_4654741660211559160[156] = 0.0;
   out_4654741660211559160[157] = 0.0;
   out_4654741660211559160[158] = 0.0;
   out_4654741660211559160[159] = 0.0;
   out_4654741660211559160[160] = 0.0;
   out_4654741660211559160[161] = 0.0;
   out_4654741660211559160[162] = 0.0;
   out_4654741660211559160[163] = 0.0;
   out_4654741660211559160[164] = 0.0;
   out_4654741660211559160[165] = 0.0;
   out_4654741660211559160[166] = 0.0;
   out_4654741660211559160[167] = 0.0;
   out_4654741660211559160[168] = 0.0;
   out_4654741660211559160[169] = 0.0;
   out_4654741660211559160[170] = 0.0;
   out_4654741660211559160[171] = 1.0;
   out_4654741660211559160[172] = 0.0;
   out_4654741660211559160[173] = 0.0;
   out_4654741660211559160[174] = 0.0;
   out_4654741660211559160[175] = 0.0;
   out_4654741660211559160[176] = 0.0;
   out_4654741660211559160[177] = 0.0;
   out_4654741660211559160[178] = 0.0;
   out_4654741660211559160[179] = 0.0;
   out_4654741660211559160[180] = 0.0;
   out_4654741660211559160[181] = 0.0;
   out_4654741660211559160[182] = 0.0;
   out_4654741660211559160[183] = 0.0;
   out_4654741660211559160[184] = 0.0;
   out_4654741660211559160[185] = 0.0;
   out_4654741660211559160[186] = 0.0;
   out_4654741660211559160[187] = 0.0;
   out_4654741660211559160[188] = 0.0;
   out_4654741660211559160[189] = 0.0;
   out_4654741660211559160[190] = 1.0;
   out_4654741660211559160[191] = 0.0;
   out_4654741660211559160[192] = 0.0;
   out_4654741660211559160[193] = 0.0;
   out_4654741660211559160[194] = 0.0;
   out_4654741660211559160[195] = 0.0;
   out_4654741660211559160[196] = 0.0;
   out_4654741660211559160[197] = 0.0;
   out_4654741660211559160[198] = 0.0;
   out_4654741660211559160[199] = 0.0;
   out_4654741660211559160[200] = 0.0;
   out_4654741660211559160[201] = 0.0;
   out_4654741660211559160[202] = 0.0;
   out_4654741660211559160[203] = 0.0;
   out_4654741660211559160[204] = 0.0;
   out_4654741660211559160[205] = 0.0;
   out_4654741660211559160[206] = 0.0;
   out_4654741660211559160[207] = 0.0;
   out_4654741660211559160[208] = 0.0;
   out_4654741660211559160[209] = 1.0;
   out_4654741660211559160[210] = 0.0;
   out_4654741660211559160[211] = 0.0;
   out_4654741660211559160[212] = 0.0;
   out_4654741660211559160[213] = 0.0;
   out_4654741660211559160[214] = 0.0;
   out_4654741660211559160[215] = 0.0;
   out_4654741660211559160[216] = 0.0;
   out_4654741660211559160[217] = 0.0;
   out_4654741660211559160[218] = 0.0;
   out_4654741660211559160[219] = 0.0;
   out_4654741660211559160[220] = 0.0;
   out_4654741660211559160[221] = 0.0;
   out_4654741660211559160[222] = 0.0;
   out_4654741660211559160[223] = 0.0;
   out_4654741660211559160[224] = 0.0;
   out_4654741660211559160[225] = 0.0;
   out_4654741660211559160[226] = 0.0;
   out_4654741660211559160[227] = 0.0;
   out_4654741660211559160[228] = 1.0;
   out_4654741660211559160[229] = 0.0;
   out_4654741660211559160[230] = 0.0;
   out_4654741660211559160[231] = 0.0;
   out_4654741660211559160[232] = 0.0;
   out_4654741660211559160[233] = 0.0;
   out_4654741660211559160[234] = 0.0;
   out_4654741660211559160[235] = 0.0;
   out_4654741660211559160[236] = 0.0;
   out_4654741660211559160[237] = 0.0;
   out_4654741660211559160[238] = 0.0;
   out_4654741660211559160[239] = 0.0;
   out_4654741660211559160[240] = 0.0;
   out_4654741660211559160[241] = 0.0;
   out_4654741660211559160[242] = 0.0;
   out_4654741660211559160[243] = 0.0;
   out_4654741660211559160[244] = 0.0;
   out_4654741660211559160[245] = 0.0;
   out_4654741660211559160[246] = 0.0;
   out_4654741660211559160[247] = 1.0;
   out_4654741660211559160[248] = 0.0;
   out_4654741660211559160[249] = 0.0;
   out_4654741660211559160[250] = 0.0;
   out_4654741660211559160[251] = 0.0;
   out_4654741660211559160[252] = 0.0;
   out_4654741660211559160[253] = 0.0;
   out_4654741660211559160[254] = 0.0;
   out_4654741660211559160[255] = 0.0;
   out_4654741660211559160[256] = 0.0;
   out_4654741660211559160[257] = 0.0;
   out_4654741660211559160[258] = 0.0;
   out_4654741660211559160[259] = 0.0;
   out_4654741660211559160[260] = 0.0;
   out_4654741660211559160[261] = 0.0;
   out_4654741660211559160[262] = 0.0;
   out_4654741660211559160[263] = 0.0;
   out_4654741660211559160[264] = 0.0;
   out_4654741660211559160[265] = 0.0;
   out_4654741660211559160[266] = 1.0;
   out_4654741660211559160[267] = 0.0;
   out_4654741660211559160[268] = 0.0;
   out_4654741660211559160[269] = 0.0;
   out_4654741660211559160[270] = 0.0;
   out_4654741660211559160[271] = 0.0;
   out_4654741660211559160[272] = 0.0;
   out_4654741660211559160[273] = 0.0;
   out_4654741660211559160[274] = 0.0;
   out_4654741660211559160[275] = 0.0;
   out_4654741660211559160[276] = 0.0;
   out_4654741660211559160[277] = 0.0;
   out_4654741660211559160[278] = 0.0;
   out_4654741660211559160[279] = 0.0;
   out_4654741660211559160[280] = 0.0;
   out_4654741660211559160[281] = 0.0;
   out_4654741660211559160[282] = 0.0;
   out_4654741660211559160[283] = 0.0;
   out_4654741660211559160[284] = 0.0;
   out_4654741660211559160[285] = 1.0;
   out_4654741660211559160[286] = 0.0;
   out_4654741660211559160[287] = 0.0;
   out_4654741660211559160[288] = 0.0;
   out_4654741660211559160[289] = 0.0;
   out_4654741660211559160[290] = 0.0;
   out_4654741660211559160[291] = 0.0;
   out_4654741660211559160[292] = 0.0;
   out_4654741660211559160[293] = 0.0;
   out_4654741660211559160[294] = 0.0;
   out_4654741660211559160[295] = 0.0;
   out_4654741660211559160[296] = 0.0;
   out_4654741660211559160[297] = 0.0;
   out_4654741660211559160[298] = 0.0;
   out_4654741660211559160[299] = 0.0;
   out_4654741660211559160[300] = 0.0;
   out_4654741660211559160[301] = 0.0;
   out_4654741660211559160[302] = 0.0;
   out_4654741660211559160[303] = 0.0;
   out_4654741660211559160[304] = 1.0;
   out_4654741660211559160[305] = 0.0;
   out_4654741660211559160[306] = 0.0;
   out_4654741660211559160[307] = 0.0;
   out_4654741660211559160[308] = 0.0;
   out_4654741660211559160[309] = 0.0;
   out_4654741660211559160[310] = 0.0;
   out_4654741660211559160[311] = 0.0;
   out_4654741660211559160[312] = 0.0;
   out_4654741660211559160[313] = 0.0;
   out_4654741660211559160[314] = 0.0;
   out_4654741660211559160[315] = 0.0;
   out_4654741660211559160[316] = 0.0;
   out_4654741660211559160[317] = 0.0;
   out_4654741660211559160[318] = 0.0;
   out_4654741660211559160[319] = 0.0;
   out_4654741660211559160[320] = 0.0;
   out_4654741660211559160[321] = 0.0;
   out_4654741660211559160[322] = 0.0;
   out_4654741660211559160[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_2028336384740831999) {
   out_2028336384740831999[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_2028336384740831999[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_2028336384740831999[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_2028336384740831999[3] = dt*state[12] + state[3];
   out_2028336384740831999[4] = dt*state[13] + state[4];
   out_2028336384740831999[5] = dt*state[14] + state[5];
   out_2028336384740831999[6] = state[6];
   out_2028336384740831999[7] = state[7];
   out_2028336384740831999[8] = state[8];
   out_2028336384740831999[9] = state[9];
   out_2028336384740831999[10] = state[10];
   out_2028336384740831999[11] = state[11];
   out_2028336384740831999[12] = state[12];
   out_2028336384740831999[13] = state[13];
   out_2028336384740831999[14] = state[14];
   out_2028336384740831999[15] = state[15];
   out_2028336384740831999[16] = state[16];
   out_2028336384740831999[17] = state[17];
}
void F_fun(double *state, double dt, double *out_6068183104054389338) {
   out_6068183104054389338[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6068183104054389338[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6068183104054389338[2] = 0;
   out_6068183104054389338[3] = 0;
   out_6068183104054389338[4] = 0;
   out_6068183104054389338[5] = 0;
   out_6068183104054389338[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6068183104054389338[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6068183104054389338[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6068183104054389338[9] = 0;
   out_6068183104054389338[10] = 0;
   out_6068183104054389338[11] = 0;
   out_6068183104054389338[12] = 0;
   out_6068183104054389338[13] = 0;
   out_6068183104054389338[14] = 0;
   out_6068183104054389338[15] = 0;
   out_6068183104054389338[16] = 0;
   out_6068183104054389338[17] = 0;
   out_6068183104054389338[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6068183104054389338[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6068183104054389338[20] = 0;
   out_6068183104054389338[21] = 0;
   out_6068183104054389338[22] = 0;
   out_6068183104054389338[23] = 0;
   out_6068183104054389338[24] = 0;
   out_6068183104054389338[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6068183104054389338[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6068183104054389338[27] = 0;
   out_6068183104054389338[28] = 0;
   out_6068183104054389338[29] = 0;
   out_6068183104054389338[30] = 0;
   out_6068183104054389338[31] = 0;
   out_6068183104054389338[32] = 0;
   out_6068183104054389338[33] = 0;
   out_6068183104054389338[34] = 0;
   out_6068183104054389338[35] = 0;
   out_6068183104054389338[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6068183104054389338[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6068183104054389338[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6068183104054389338[39] = 0;
   out_6068183104054389338[40] = 0;
   out_6068183104054389338[41] = 0;
   out_6068183104054389338[42] = 0;
   out_6068183104054389338[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6068183104054389338[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6068183104054389338[45] = 0;
   out_6068183104054389338[46] = 0;
   out_6068183104054389338[47] = 0;
   out_6068183104054389338[48] = 0;
   out_6068183104054389338[49] = 0;
   out_6068183104054389338[50] = 0;
   out_6068183104054389338[51] = 0;
   out_6068183104054389338[52] = 0;
   out_6068183104054389338[53] = 0;
   out_6068183104054389338[54] = 0;
   out_6068183104054389338[55] = 0;
   out_6068183104054389338[56] = 0;
   out_6068183104054389338[57] = 1;
   out_6068183104054389338[58] = 0;
   out_6068183104054389338[59] = 0;
   out_6068183104054389338[60] = 0;
   out_6068183104054389338[61] = 0;
   out_6068183104054389338[62] = 0;
   out_6068183104054389338[63] = 0;
   out_6068183104054389338[64] = 0;
   out_6068183104054389338[65] = 0;
   out_6068183104054389338[66] = dt;
   out_6068183104054389338[67] = 0;
   out_6068183104054389338[68] = 0;
   out_6068183104054389338[69] = 0;
   out_6068183104054389338[70] = 0;
   out_6068183104054389338[71] = 0;
   out_6068183104054389338[72] = 0;
   out_6068183104054389338[73] = 0;
   out_6068183104054389338[74] = 0;
   out_6068183104054389338[75] = 0;
   out_6068183104054389338[76] = 1;
   out_6068183104054389338[77] = 0;
   out_6068183104054389338[78] = 0;
   out_6068183104054389338[79] = 0;
   out_6068183104054389338[80] = 0;
   out_6068183104054389338[81] = 0;
   out_6068183104054389338[82] = 0;
   out_6068183104054389338[83] = 0;
   out_6068183104054389338[84] = 0;
   out_6068183104054389338[85] = dt;
   out_6068183104054389338[86] = 0;
   out_6068183104054389338[87] = 0;
   out_6068183104054389338[88] = 0;
   out_6068183104054389338[89] = 0;
   out_6068183104054389338[90] = 0;
   out_6068183104054389338[91] = 0;
   out_6068183104054389338[92] = 0;
   out_6068183104054389338[93] = 0;
   out_6068183104054389338[94] = 0;
   out_6068183104054389338[95] = 1;
   out_6068183104054389338[96] = 0;
   out_6068183104054389338[97] = 0;
   out_6068183104054389338[98] = 0;
   out_6068183104054389338[99] = 0;
   out_6068183104054389338[100] = 0;
   out_6068183104054389338[101] = 0;
   out_6068183104054389338[102] = 0;
   out_6068183104054389338[103] = 0;
   out_6068183104054389338[104] = dt;
   out_6068183104054389338[105] = 0;
   out_6068183104054389338[106] = 0;
   out_6068183104054389338[107] = 0;
   out_6068183104054389338[108] = 0;
   out_6068183104054389338[109] = 0;
   out_6068183104054389338[110] = 0;
   out_6068183104054389338[111] = 0;
   out_6068183104054389338[112] = 0;
   out_6068183104054389338[113] = 0;
   out_6068183104054389338[114] = 1;
   out_6068183104054389338[115] = 0;
   out_6068183104054389338[116] = 0;
   out_6068183104054389338[117] = 0;
   out_6068183104054389338[118] = 0;
   out_6068183104054389338[119] = 0;
   out_6068183104054389338[120] = 0;
   out_6068183104054389338[121] = 0;
   out_6068183104054389338[122] = 0;
   out_6068183104054389338[123] = 0;
   out_6068183104054389338[124] = 0;
   out_6068183104054389338[125] = 0;
   out_6068183104054389338[126] = 0;
   out_6068183104054389338[127] = 0;
   out_6068183104054389338[128] = 0;
   out_6068183104054389338[129] = 0;
   out_6068183104054389338[130] = 0;
   out_6068183104054389338[131] = 0;
   out_6068183104054389338[132] = 0;
   out_6068183104054389338[133] = 1;
   out_6068183104054389338[134] = 0;
   out_6068183104054389338[135] = 0;
   out_6068183104054389338[136] = 0;
   out_6068183104054389338[137] = 0;
   out_6068183104054389338[138] = 0;
   out_6068183104054389338[139] = 0;
   out_6068183104054389338[140] = 0;
   out_6068183104054389338[141] = 0;
   out_6068183104054389338[142] = 0;
   out_6068183104054389338[143] = 0;
   out_6068183104054389338[144] = 0;
   out_6068183104054389338[145] = 0;
   out_6068183104054389338[146] = 0;
   out_6068183104054389338[147] = 0;
   out_6068183104054389338[148] = 0;
   out_6068183104054389338[149] = 0;
   out_6068183104054389338[150] = 0;
   out_6068183104054389338[151] = 0;
   out_6068183104054389338[152] = 1;
   out_6068183104054389338[153] = 0;
   out_6068183104054389338[154] = 0;
   out_6068183104054389338[155] = 0;
   out_6068183104054389338[156] = 0;
   out_6068183104054389338[157] = 0;
   out_6068183104054389338[158] = 0;
   out_6068183104054389338[159] = 0;
   out_6068183104054389338[160] = 0;
   out_6068183104054389338[161] = 0;
   out_6068183104054389338[162] = 0;
   out_6068183104054389338[163] = 0;
   out_6068183104054389338[164] = 0;
   out_6068183104054389338[165] = 0;
   out_6068183104054389338[166] = 0;
   out_6068183104054389338[167] = 0;
   out_6068183104054389338[168] = 0;
   out_6068183104054389338[169] = 0;
   out_6068183104054389338[170] = 0;
   out_6068183104054389338[171] = 1;
   out_6068183104054389338[172] = 0;
   out_6068183104054389338[173] = 0;
   out_6068183104054389338[174] = 0;
   out_6068183104054389338[175] = 0;
   out_6068183104054389338[176] = 0;
   out_6068183104054389338[177] = 0;
   out_6068183104054389338[178] = 0;
   out_6068183104054389338[179] = 0;
   out_6068183104054389338[180] = 0;
   out_6068183104054389338[181] = 0;
   out_6068183104054389338[182] = 0;
   out_6068183104054389338[183] = 0;
   out_6068183104054389338[184] = 0;
   out_6068183104054389338[185] = 0;
   out_6068183104054389338[186] = 0;
   out_6068183104054389338[187] = 0;
   out_6068183104054389338[188] = 0;
   out_6068183104054389338[189] = 0;
   out_6068183104054389338[190] = 1;
   out_6068183104054389338[191] = 0;
   out_6068183104054389338[192] = 0;
   out_6068183104054389338[193] = 0;
   out_6068183104054389338[194] = 0;
   out_6068183104054389338[195] = 0;
   out_6068183104054389338[196] = 0;
   out_6068183104054389338[197] = 0;
   out_6068183104054389338[198] = 0;
   out_6068183104054389338[199] = 0;
   out_6068183104054389338[200] = 0;
   out_6068183104054389338[201] = 0;
   out_6068183104054389338[202] = 0;
   out_6068183104054389338[203] = 0;
   out_6068183104054389338[204] = 0;
   out_6068183104054389338[205] = 0;
   out_6068183104054389338[206] = 0;
   out_6068183104054389338[207] = 0;
   out_6068183104054389338[208] = 0;
   out_6068183104054389338[209] = 1;
   out_6068183104054389338[210] = 0;
   out_6068183104054389338[211] = 0;
   out_6068183104054389338[212] = 0;
   out_6068183104054389338[213] = 0;
   out_6068183104054389338[214] = 0;
   out_6068183104054389338[215] = 0;
   out_6068183104054389338[216] = 0;
   out_6068183104054389338[217] = 0;
   out_6068183104054389338[218] = 0;
   out_6068183104054389338[219] = 0;
   out_6068183104054389338[220] = 0;
   out_6068183104054389338[221] = 0;
   out_6068183104054389338[222] = 0;
   out_6068183104054389338[223] = 0;
   out_6068183104054389338[224] = 0;
   out_6068183104054389338[225] = 0;
   out_6068183104054389338[226] = 0;
   out_6068183104054389338[227] = 0;
   out_6068183104054389338[228] = 1;
   out_6068183104054389338[229] = 0;
   out_6068183104054389338[230] = 0;
   out_6068183104054389338[231] = 0;
   out_6068183104054389338[232] = 0;
   out_6068183104054389338[233] = 0;
   out_6068183104054389338[234] = 0;
   out_6068183104054389338[235] = 0;
   out_6068183104054389338[236] = 0;
   out_6068183104054389338[237] = 0;
   out_6068183104054389338[238] = 0;
   out_6068183104054389338[239] = 0;
   out_6068183104054389338[240] = 0;
   out_6068183104054389338[241] = 0;
   out_6068183104054389338[242] = 0;
   out_6068183104054389338[243] = 0;
   out_6068183104054389338[244] = 0;
   out_6068183104054389338[245] = 0;
   out_6068183104054389338[246] = 0;
   out_6068183104054389338[247] = 1;
   out_6068183104054389338[248] = 0;
   out_6068183104054389338[249] = 0;
   out_6068183104054389338[250] = 0;
   out_6068183104054389338[251] = 0;
   out_6068183104054389338[252] = 0;
   out_6068183104054389338[253] = 0;
   out_6068183104054389338[254] = 0;
   out_6068183104054389338[255] = 0;
   out_6068183104054389338[256] = 0;
   out_6068183104054389338[257] = 0;
   out_6068183104054389338[258] = 0;
   out_6068183104054389338[259] = 0;
   out_6068183104054389338[260] = 0;
   out_6068183104054389338[261] = 0;
   out_6068183104054389338[262] = 0;
   out_6068183104054389338[263] = 0;
   out_6068183104054389338[264] = 0;
   out_6068183104054389338[265] = 0;
   out_6068183104054389338[266] = 1;
   out_6068183104054389338[267] = 0;
   out_6068183104054389338[268] = 0;
   out_6068183104054389338[269] = 0;
   out_6068183104054389338[270] = 0;
   out_6068183104054389338[271] = 0;
   out_6068183104054389338[272] = 0;
   out_6068183104054389338[273] = 0;
   out_6068183104054389338[274] = 0;
   out_6068183104054389338[275] = 0;
   out_6068183104054389338[276] = 0;
   out_6068183104054389338[277] = 0;
   out_6068183104054389338[278] = 0;
   out_6068183104054389338[279] = 0;
   out_6068183104054389338[280] = 0;
   out_6068183104054389338[281] = 0;
   out_6068183104054389338[282] = 0;
   out_6068183104054389338[283] = 0;
   out_6068183104054389338[284] = 0;
   out_6068183104054389338[285] = 1;
   out_6068183104054389338[286] = 0;
   out_6068183104054389338[287] = 0;
   out_6068183104054389338[288] = 0;
   out_6068183104054389338[289] = 0;
   out_6068183104054389338[290] = 0;
   out_6068183104054389338[291] = 0;
   out_6068183104054389338[292] = 0;
   out_6068183104054389338[293] = 0;
   out_6068183104054389338[294] = 0;
   out_6068183104054389338[295] = 0;
   out_6068183104054389338[296] = 0;
   out_6068183104054389338[297] = 0;
   out_6068183104054389338[298] = 0;
   out_6068183104054389338[299] = 0;
   out_6068183104054389338[300] = 0;
   out_6068183104054389338[301] = 0;
   out_6068183104054389338[302] = 0;
   out_6068183104054389338[303] = 0;
   out_6068183104054389338[304] = 1;
   out_6068183104054389338[305] = 0;
   out_6068183104054389338[306] = 0;
   out_6068183104054389338[307] = 0;
   out_6068183104054389338[308] = 0;
   out_6068183104054389338[309] = 0;
   out_6068183104054389338[310] = 0;
   out_6068183104054389338[311] = 0;
   out_6068183104054389338[312] = 0;
   out_6068183104054389338[313] = 0;
   out_6068183104054389338[314] = 0;
   out_6068183104054389338[315] = 0;
   out_6068183104054389338[316] = 0;
   out_6068183104054389338[317] = 0;
   out_6068183104054389338[318] = 0;
   out_6068183104054389338[319] = 0;
   out_6068183104054389338[320] = 0;
   out_6068183104054389338[321] = 0;
   out_6068183104054389338[322] = 0;
   out_6068183104054389338[323] = 1;
}
void h_4(double *state, double *unused, double *out_8463415650667413426) {
   out_8463415650667413426[0] = state[6] + state[9];
   out_8463415650667413426[1] = state[7] + state[10];
   out_8463415650667413426[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_4847640747440171114) {
   out_4847640747440171114[0] = 0;
   out_4847640747440171114[1] = 0;
   out_4847640747440171114[2] = 0;
   out_4847640747440171114[3] = 0;
   out_4847640747440171114[4] = 0;
   out_4847640747440171114[5] = 0;
   out_4847640747440171114[6] = 1;
   out_4847640747440171114[7] = 0;
   out_4847640747440171114[8] = 0;
   out_4847640747440171114[9] = 1;
   out_4847640747440171114[10] = 0;
   out_4847640747440171114[11] = 0;
   out_4847640747440171114[12] = 0;
   out_4847640747440171114[13] = 0;
   out_4847640747440171114[14] = 0;
   out_4847640747440171114[15] = 0;
   out_4847640747440171114[16] = 0;
   out_4847640747440171114[17] = 0;
   out_4847640747440171114[18] = 0;
   out_4847640747440171114[19] = 0;
   out_4847640747440171114[20] = 0;
   out_4847640747440171114[21] = 0;
   out_4847640747440171114[22] = 0;
   out_4847640747440171114[23] = 0;
   out_4847640747440171114[24] = 0;
   out_4847640747440171114[25] = 1;
   out_4847640747440171114[26] = 0;
   out_4847640747440171114[27] = 0;
   out_4847640747440171114[28] = 1;
   out_4847640747440171114[29] = 0;
   out_4847640747440171114[30] = 0;
   out_4847640747440171114[31] = 0;
   out_4847640747440171114[32] = 0;
   out_4847640747440171114[33] = 0;
   out_4847640747440171114[34] = 0;
   out_4847640747440171114[35] = 0;
   out_4847640747440171114[36] = 0;
   out_4847640747440171114[37] = 0;
   out_4847640747440171114[38] = 0;
   out_4847640747440171114[39] = 0;
   out_4847640747440171114[40] = 0;
   out_4847640747440171114[41] = 0;
   out_4847640747440171114[42] = 0;
   out_4847640747440171114[43] = 0;
   out_4847640747440171114[44] = 1;
   out_4847640747440171114[45] = 0;
   out_4847640747440171114[46] = 0;
   out_4847640747440171114[47] = 1;
   out_4847640747440171114[48] = 0;
   out_4847640747440171114[49] = 0;
   out_4847640747440171114[50] = 0;
   out_4847640747440171114[51] = 0;
   out_4847640747440171114[52] = 0;
   out_4847640747440171114[53] = 0;
}
void h_10(double *state, double *unused, double *out_3885613174414808707) {
   out_3885613174414808707[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_3885613174414808707[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_3885613174414808707[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_1183211805543623442) {
   out_1183211805543623442[0] = 0;
   out_1183211805543623442[1] = 9.8100000000000005*cos(state[1]);
   out_1183211805543623442[2] = 0;
   out_1183211805543623442[3] = 0;
   out_1183211805543623442[4] = -state[8];
   out_1183211805543623442[5] = state[7];
   out_1183211805543623442[6] = 0;
   out_1183211805543623442[7] = state[5];
   out_1183211805543623442[8] = -state[4];
   out_1183211805543623442[9] = 0;
   out_1183211805543623442[10] = 0;
   out_1183211805543623442[11] = 0;
   out_1183211805543623442[12] = 1;
   out_1183211805543623442[13] = 0;
   out_1183211805543623442[14] = 0;
   out_1183211805543623442[15] = 1;
   out_1183211805543623442[16] = 0;
   out_1183211805543623442[17] = 0;
   out_1183211805543623442[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_1183211805543623442[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_1183211805543623442[20] = 0;
   out_1183211805543623442[21] = state[8];
   out_1183211805543623442[22] = 0;
   out_1183211805543623442[23] = -state[6];
   out_1183211805543623442[24] = -state[5];
   out_1183211805543623442[25] = 0;
   out_1183211805543623442[26] = state[3];
   out_1183211805543623442[27] = 0;
   out_1183211805543623442[28] = 0;
   out_1183211805543623442[29] = 0;
   out_1183211805543623442[30] = 0;
   out_1183211805543623442[31] = 1;
   out_1183211805543623442[32] = 0;
   out_1183211805543623442[33] = 0;
   out_1183211805543623442[34] = 1;
   out_1183211805543623442[35] = 0;
   out_1183211805543623442[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_1183211805543623442[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_1183211805543623442[38] = 0;
   out_1183211805543623442[39] = -state[7];
   out_1183211805543623442[40] = state[6];
   out_1183211805543623442[41] = 0;
   out_1183211805543623442[42] = state[4];
   out_1183211805543623442[43] = -state[3];
   out_1183211805543623442[44] = 0;
   out_1183211805543623442[45] = 0;
   out_1183211805543623442[46] = 0;
   out_1183211805543623442[47] = 0;
   out_1183211805543623442[48] = 0;
   out_1183211805543623442[49] = 0;
   out_1183211805543623442[50] = 1;
   out_1183211805543623442[51] = 0;
   out_1183211805543623442[52] = 0;
   out_1183211805543623442[53] = 1;
}
void h_13(double *state, double *unused, double *out_7810348970534242965) {
   out_7810348970534242965[0] = state[3];
   out_7810348970534242965[1] = state[4];
   out_7810348970534242965[2] = state[5];
}
void H_13(double *state, double *unused, double *out_1013885284137647090) {
   out_1013885284137647090[0] = 0;
   out_1013885284137647090[1] = 0;
   out_1013885284137647090[2] = 0;
   out_1013885284137647090[3] = 1;
   out_1013885284137647090[4] = 0;
   out_1013885284137647090[5] = 0;
   out_1013885284137647090[6] = 0;
   out_1013885284137647090[7] = 0;
   out_1013885284137647090[8] = 0;
   out_1013885284137647090[9] = 0;
   out_1013885284137647090[10] = 0;
   out_1013885284137647090[11] = 0;
   out_1013885284137647090[12] = 0;
   out_1013885284137647090[13] = 0;
   out_1013885284137647090[14] = 0;
   out_1013885284137647090[15] = 0;
   out_1013885284137647090[16] = 0;
   out_1013885284137647090[17] = 0;
   out_1013885284137647090[18] = 0;
   out_1013885284137647090[19] = 0;
   out_1013885284137647090[20] = 0;
   out_1013885284137647090[21] = 0;
   out_1013885284137647090[22] = 1;
   out_1013885284137647090[23] = 0;
   out_1013885284137647090[24] = 0;
   out_1013885284137647090[25] = 0;
   out_1013885284137647090[26] = 0;
   out_1013885284137647090[27] = 0;
   out_1013885284137647090[28] = 0;
   out_1013885284137647090[29] = 0;
   out_1013885284137647090[30] = 0;
   out_1013885284137647090[31] = 0;
   out_1013885284137647090[32] = 0;
   out_1013885284137647090[33] = 0;
   out_1013885284137647090[34] = 0;
   out_1013885284137647090[35] = 0;
   out_1013885284137647090[36] = 0;
   out_1013885284137647090[37] = 0;
   out_1013885284137647090[38] = 0;
   out_1013885284137647090[39] = 0;
   out_1013885284137647090[40] = 0;
   out_1013885284137647090[41] = 1;
   out_1013885284137647090[42] = 0;
   out_1013885284137647090[43] = 0;
   out_1013885284137647090[44] = 0;
   out_1013885284137647090[45] = 0;
   out_1013885284137647090[46] = 0;
   out_1013885284137647090[47] = 0;
   out_1013885284137647090[48] = 0;
   out_1013885284137647090[49] = 0;
   out_1013885284137647090[50] = 0;
   out_1013885284137647090[51] = 0;
   out_1013885284137647090[52] = 0;
   out_1013885284137647090[53] = 0;
}
void h_14(double *state, double *unused, double *out_7656480836221072816) {
   out_7656480836221072816[0] = state[6];
   out_7656480836221072816[1] = state[7];
   out_7656480836221072816[2] = state[8];
}
void H_14(double *state, double *unused, double *out_1764852315144798818) {
   out_1764852315144798818[0] = 0;
   out_1764852315144798818[1] = 0;
   out_1764852315144798818[2] = 0;
   out_1764852315144798818[3] = 0;
   out_1764852315144798818[4] = 0;
   out_1764852315144798818[5] = 0;
   out_1764852315144798818[6] = 1;
   out_1764852315144798818[7] = 0;
   out_1764852315144798818[8] = 0;
   out_1764852315144798818[9] = 0;
   out_1764852315144798818[10] = 0;
   out_1764852315144798818[11] = 0;
   out_1764852315144798818[12] = 0;
   out_1764852315144798818[13] = 0;
   out_1764852315144798818[14] = 0;
   out_1764852315144798818[15] = 0;
   out_1764852315144798818[16] = 0;
   out_1764852315144798818[17] = 0;
   out_1764852315144798818[18] = 0;
   out_1764852315144798818[19] = 0;
   out_1764852315144798818[20] = 0;
   out_1764852315144798818[21] = 0;
   out_1764852315144798818[22] = 0;
   out_1764852315144798818[23] = 0;
   out_1764852315144798818[24] = 0;
   out_1764852315144798818[25] = 1;
   out_1764852315144798818[26] = 0;
   out_1764852315144798818[27] = 0;
   out_1764852315144798818[28] = 0;
   out_1764852315144798818[29] = 0;
   out_1764852315144798818[30] = 0;
   out_1764852315144798818[31] = 0;
   out_1764852315144798818[32] = 0;
   out_1764852315144798818[33] = 0;
   out_1764852315144798818[34] = 0;
   out_1764852315144798818[35] = 0;
   out_1764852315144798818[36] = 0;
   out_1764852315144798818[37] = 0;
   out_1764852315144798818[38] = 0;
   out_1764852315144798818[39] = 0;
   out_1764852315144798818[40] = 0;
   out_1764852315144798818[41] = 0;
   out_1764852315144798818[42] = 0;
   out_1764852315144798818[43] = 0;
   out_1764852315144798818[44] = 1;
   out_1764852315144798818[45] = 0;
   out_1764852315144798818[46] = 0;
   out_1764852315144798818[47] = 0;
   out_1764852315144798818[48] = 0;
   out_1764852315144798818[49] = 0;
   out_1764852315144798818[50] = 0;
   out_1764852315144798818[51] = 0;
   out_1764852315144798818[52] = 0;
   out_1764852315144798818[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_2540016638677114272) {
  err_fun(nom_x, delta_x, out_2540016638677114272);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4211094856349578415) {
  inv_err_fun(nom_x, true_x, out_4211094856349578415);
}
void pose_H_mod_fun(double *state, double *out_4654741660211559160) {
  H_mod_fun(state, out_4654741660211559160);
}
void pose_f_fun(double *state, double dt, double *out_2028336384740831999) {
  f_fun(state,  dt, out_2028336384740831999);
}
void pose_F_fun(double *state, double dt, double *out_6068183104054389338) {
  F_fun(state,  dt, out_6068183104054389338);
}
void pose_h_4(double *state, double *unused, double *out_8463415650667413426) {
  h_4(state, unused, out_8463415650667413426);
}
void pose_H_4(double *state, double *unused, double *out_4847640747440171114) {
  H_4(state, unused, out_4847640747440171114);
}
void pose_h_10(double *state, double *unused, double *out_3885613174414808707) {
  h_10(state, unused, out_3885613174414808707);
}
void pose_H_10(double *state, double *unused, double *out_1183211805543623442) {
  H_10(state, unused, out_1183211805543623442);
}
void pose_h_13(double *state, double *unused, double *out_7810348970534242965) {
  h_13(state, unused, out_7810348970534242965);
}
void pose_H_13(double *state, double *unused, double *out_1013885284137647090) {
  H_13(state, unused, out_1013885284137647090);
}
void pose_h_14(double *state, double *unused, double *out_7656480836221072816) {
  h_14(state, unused, out_7656480836221072816);
}
void pose_H_14(double *state, double *unused, double *out_1764852315144798818) {
  H_14(state, unused, out_1764852315144798818);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
