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
void err_fun(double *nom_x, double *delta_x, double *out_6433275272970939599) {
   out_6433275272970939599[0] = delta_x[0] + nom_x[0];
   out_6433275272970939599[1] = delta_x[1] + nom_x[1];
   out_6433275272970939599[2] = delta_x[2] + nom_x[2];
   out_6433275272970939599[3] = delta_x[3] + nom_x[3];
   out_6433275272970939599[4] = delta_x[4] + nom_x[4];
   out_6433275272970939599[5] = delta_x[5] + nom_x[5];
   out_6433275272970939599[6] = delta_x[6] + nom_x[6];
   out_6433275272970939599[7] = delta_x[7] + nom_x[7];
   out_6433275272970939599[8] = delta_x[8] + nom_x[8];
   out_6433275272970939599[9] = delta_x[9] + nom_x[9];
   out_6433275272970939599[10] = delta_x[10] + nom_x[10];
   out_6433275272970939599[11] = delta_x[11] + nom_x[11];
   out_6433275272970939599[12] = delta_x[12] + nom_x[12];
   out_6433275272970939599[13] = delta_x[13] + nom_x[13];
   out_6433275272970939599[14] = delta_x[14] + nom_x[14];
   out_6433275272970939599[15] = delta_x[15] + nom_x[15];
   out_6433275272970939599[16] = delta_x[16] + nom_x[16];
   out_6433275272970939599[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7086127294916974707) {
   out_7086127294916974707[0] = -nom_x[0] + true_x[0];
   out_7086127294916974707[1] = -nom_x[1] + true_x[1];
   out_7086127294916974707[2] = -nom_x[2] + true_x[2];
   out_7086127294916974707[3] = -nom_x[3] + true_x[3];
   out_7086127294916974707[4] = -nom_x[4] + true_x[4];
   out_7086127294916974707[5] = -nom_x[5] + true_x[5];
   out_7086127294916974707[6] = -nom_x[6] + true_x[6];
   out_7086127294916974707[7] = -nom_x[7] + true_x[7];
   out_7086127294916974707[8] = -nom_x[8] + true_x[8];
   out_7086127294916974707[9] = -nom_x[9] + true_x[9];
   out_7086127294916974707[10] = -nom_x[10] + true_x[10];
   out_7086127294916974707[11] = -nom_x[11] + true_x[11];
   out_7086127294916974707[12] = -nom_x[12] + true_x[12];
   out_7086127294916974707[13] = -nom_x[13] + true_x[13];
   out_7086127294916974707[14] = -nom_x[14] + true_x[14];
   out_7086127294916974707[15] = -nom_x[15] + true_x[15];
   out_7086127294916974707[16] = -nom_x[16] + true_x[16];
   out_7086127294916974707[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_4308325429962827123) {
   out_4308325429962827123[0] = 1.0;
   out_4308325429962827123[1] = 0.0;
   out_4308325429962827123[2] = 0.0;
   out_4308325429962827123[3] = 0.0;
   out_4308325429962827123[4] = 0.0;
   out_4308325429962827123[5] = 0.0;
   out_4308325429962827123[6] = 0.0;
   out_4308325429962827123[7] = 0.0;
   out_4308325429962827123[8] = 0.0;
   out_4308325429962827123[9] = 0.0;
   out_4308325429962827123[10] = 0.0;
   out_4308325429962827123[11] = 0.0;
   out_4308325429962827123[12] = 0.0;
   out_4308325429962827123[13] = 0.0;
   out_4308325429962827123[14] = 0.0;
   out_4308325429962827123[15] = 0.0;
   out_4308325429962827123[16] = 0.0;
   out_4308325429962827123[17] = 0.0;
   out_4308325429962827123[18] = 0.0;
   out_4308325429962827123[19] = 1.0;
   out_4308325429962827123[20] = 0.0;
   out_4308325429962827123[21] = 0.0;
   out_4308325429962827123[22] = 0.0;
   out_4308325429962827123[23] = 0.0;
   out_4308325429962827123[24] = 0.0;
   out_4308325429962827123[25] = 0.0;
   out_4308325429962827123[26] = 0.0;
   out_4308325429962827123[27] = 0.0;
   out_4308325429962827123[28] = 0.0;
   out_4308325429962827123[29] = 0.0;
   out_4308325429962827123[30] = 0.0;
   out_4308325429962827123[31] = 0.0;
   out_4308325429962827123[32] = 0.0;
   out_4308325429962827123[33] = 0.0;
   out_4308325429962827123[34] = 0.0;
   out_4308325429962827123[35] = 0.0;
   out_4308325429962827123[36] = 0.0;
   out_4308325429962827123[37] = 0.0;
   out_4308325429962827123[38] = 1.0;
   out_4308325429962827123[39] = 0.0;
   out_4308325429962827123[40] = 0.0;
   out_4308325429962827123[41] = 0.0;
   out_4308325429962827123[42] = 0.0;
   out_4308325429962827123[43] = 0.0;
   out_4308325429962827123[44] = 0.0;
   out_4308325429962827123[45] = 0.0;
   out_4308325429962827123[46] = 0.0;
   out_4308325429962827123[47] = 0.0;
   out_4308325429962827123[48] = 0.0;
   out_4308325429962827123[49] = 0.0;
   out_4308325429962827123[50] = 0.0;
   out_4308325429962827123[51] = 0.0;
   out_4308325429962827123[52] = 0.0;
   out_4308325429962827123[53] = 0.0;
   out_4308325429962827123[54] = 0.0;
   out_4308325429962827123[55] = 0.0;
   out_4308325429962827123[56] = 0.0;
   out_4308325429962827123[57] = 1.0;
   out_4308325429962827123[58] = 0.0;
   out_4308325429962827123[59] = 0.0;
   out_4308325429962827123[60] = 0.0;
   out_4308325429962827123[61] = 0.0;
   out_4308325429962827123[62] = 0.0;
   out_4308325429962827123[63] = 0.0;
   out_4308325429962827123[64] = 0.0;
   out_4308325429962827123[65] = 0.0;
   out_4308325429962827123[66] = 0.0;
   out_4308325429962827123[67] = 0.0;
   out_4308325429962827123[68] = 0.0;
   out_4308325429962827123[69] = 0.0;
   out_4308325429962827123[70] = 0.0;
   out_4308325429962827123[71] = 0.0;
   out_4308325429962827123[72] = 0.0;
   out_4308325429962827123[73] = 0.0;
   out_4308325429962827123[74] = 0.0;
   out_4308325429962827123[75] = 0.0;
   out_4308325429962827123[76] = 1.0;
   out_4308325429962827123[77] = 0.0;
   out_4308325429962827123[78] = 0.0;
   out_4308325429962827123[79] = 0.0;
   out_4308325429962827123[80] = 0.0;
   out_4308325429962827123[81] = 0.0;
   out_4308325429962827123[82] = 0.0;
   out_4308325429962827123[83] = 0.0;
   out_4308325429962827123[84] = 0.0;
   out_4308325429962827123[85] = 0.0;
   out_4308325429962827123[86] = 0.0;
   out_4308325429962827123[87] = 0.0;
   out_4308325429962827123[88] = 0.0;
   out_4308325429962827123[89] = 0.0;
   out_4308325429962827123[90] = 0.0;
   out_4308325429962827123[91] = 0.0;
   out_4308325429962827123[92] = 0.0;
   out_4308325429962827123[93] = 0.0;
   out_4308325429962827123[94] = 0.0;
   out_4308325429962827123[95] = 1.0;
   out_4308325429962827123[96] = 0.0;
   out_4308325429962827123[97] = 0.0;
   out_4308325429962827123[98] = 0.0;
   out_4308325429962827123[99] = 0.0;
   out_4308325429962827123[100] = 0.0;
   out_4308325429962827123[101] = 0.0;
   out_4308325429962827123[102] = 0.0;
   out_4308325429962827123[103] = 0.0;
   out_4308325429962827123[104] = 0.0;
   out_4308325429962827123[105] = 0.0;
   out_4308325429962827123[106] = 0.0;
   out_4308325429962827123[107] = 0.0;
   out_4308325429962827123[108] = 0.0;
   out_4308325429962827123[109] = 0.0;
   out_4308325429962827123[110] = 0.0;
   out_4308325429962827123[111] = 0.0;
   out_4308325429962827123[112] = 0.0;
   out_4308325429962827123[113] = 0.0;
   out_4308325429962827123[114] = 1.0;
   out_4308325429962827123[115] = 0.0;
   out_4308325429962827123[116] = 0.0;
   out_4308325429962827123[117] = 0.0;
   out_4308325429962827123[118] = 0.0;
   out_4308325429962827123[119] = 0.0;
   out_4308325429962827123[120] = 0.0;
   out_4308325429962827123[121] = 0.0;
   out_4308325429962827123[122] = 0.0;
   out_4308325429962827123[123] = 0.0;
   out_4308325429962827123[124] = 0.0;
   out_4308325429962827123[125] = 0.0;
   out_4308325429962827123[126] = 0.0;
   out_4308325429962827123[127] = 0.0;
   out_4308325429962827123[128] = 0.0;
   out_4308325429962827123[129] = 0.0;
   out_4308325429962827123[130] = 0.0;
   out_4308325429962827123[131] = 0.0;
   out_4308325429962827123[132] = 0.0;
   out_4308325429962827123[133] = 1.0;
   out_4308325429962827123[134] = 0.0;
   out_4308325429962827123[135] = 0.0;
   out_4308325429962827123[136] = 0.0;
   out_4308325429962827123[137] = 0.0;
   out_4308325429962827123[138] = 0.0;
   out_4308325429962827123[139] = 0.0;
   out_4308325429962827123[140] = 0.0;
   out_4308325429962827123[141] = 0.0;
   out_4308325429962827123[142] = 0.0;
   out_4308325429962827123[143] = 0.0;
   out_4308325429962827123[144] = 0.0;
   out_4308325429962827123[145] = 0.0;
   out_4308325429962827123[146] = 0.0;
   out_4308325429962827123[147] = 0.0;
   out_4308325429962827123[148] = 0.0;
   out_4308325429962827123[149] = 0.0;
   out_4308325429962827123[150] = 0.0;
   out_4308325429962827123[151] = 0.0;
   out_4308325429962827123[152] = 1.0;
   out_4308325429962827123[153] = 0.0;
   out_4308325429962827123[154] = 0.0;
   out_4308325429962827123[155] = 0.0;
   out_4308325429962827123[156] = 0.0;
   out_4308325429962827123[157] = 0.0;
   out_4308325429962827123[158] = 0.0;
   out_4308325429962827123[159] = 0.0;
   out_4308325429962827123[160] = 0.0;
   out_4308325429962827123[161] = 0.0;
   out_4308325429962827123[162] = 0.0;
   out_4308325429962827123[163] = 0.0;
   out_4308325429962827123[164] = 0.0;
   out_4308325429962827123[165] = 0.0;
   out_4308325429962827123[166] = 0.0;
   out_4308325429962827123[167] = 0.0;
   out_4308325429962827123[168] = 0.0;
   out_4308325429962827123[169] = 0.0;
   out_4308325429962827123[170] = 0.0;
   out_4308325429962827123[171] = 1.0;
   out_4308325429962827123[172] = 0.0;
   out_4308325429962827123[173] = 0.0;
   out_4308325429962827123[174] = 0.0;
   out_4308325429962827123[175] = 0.0;
   out_4308325429962827123[176] = 0.0;
   out_4308325429962827123[177] = 0.0;
   out_4308325429962827123[178] = 0.0;
   out_4308325429962827123[179] = 0.0;
   out_4308325429962827123[180] = 0.0;
   out_4308325429962827123[181] = 0.0;
   out_4308325429962827123[182] = 0.0;
   out_4308325429962827123[183] = 0.0;
   out_4308325429962827123[184] = 0.0;
   out_4308325429962827123[185] = 0.0;
   out_4308325429962827123[186] = 0.0;
   out_4308325429962827123[187] = 0.0;
   out_4308325429962827123[188] = 0.0;
   out_4308325429962827123[189] = 0.0;
   out_4308325429962827123[190] = 1.0;
   out_4308325429962827123[191] = 0.0;
   out_4308325429962827123[192] = 0.0;
   out_4308325429962827123[193] = 0.0;
   out_4308325429962827123[194] = 0.0;
   out_4308325429962827123[195] = 0.0;
   out_4308325429962827123[196] = 0.0;
   out_4308325429962827123[197] = 0.0;
   out_4308325429962827123[198] = 0.0;
   out_4308325429962827123[199] = 0.0;
   out_4308325429962827123[200] = 0.0;
   out_4308325429962827123[201] = 0.0;
   out_4308325429962827123[202] = 0.0;
   out_4308325429962827123[203] = 0.0;
   out_4308325429962827123[204] = 0.0;
   out_4308325429962827123[205] = 0.0;
   out_4308325429962827123[206] = 0.0;
   out_4308325429962827123[207] = 0.0;
   out_4308325429962827123[208] = 0.0;
   out_4308325429962827123[209] = 1.0;
   out_4308325429962827123[210] = 0.0;
   out_4308325429962827123[211] = 0.0;
   out_4308325429962827123[212] = 0.0;
   out_4308325429962827123[213] = 0.0;
   out_4308325429962827123[214] = 0.0;
   out_4308325429962827123[215] = 0.0;
   out_4308325429962827123[216] = 0.0;
   out_4308325429962827123[217] = 0.0;
   out_4308325429962827123[218] = 0.0;
   out_4308325429962827123[219] = 0.0;
   out_4308325429962827123[220] = 0.0;
   out_4308325429962827123[221] = 0.0;
   out_4308325429962827123[222] = 0.0;
   out_4308325429962827123[223] = 0.0;
   out_4308325429962827123[224] = 0.0;
   out_4308325429962827123[225] = 0.0;
   out_4308325429962827123[226] = 0.0;
   out_4308325429962827123[227] = 0.0;
   out_4308325429962827123[228] = 1.0;
   out_4308325429962827123[229] = 0.0;
   out_4308325429962827123[230] = 0.0;
   out_4308325429962827123[231] = 0.0;
   out_4308325429962827123[232] = 0.0;
   out_4308325429962827123[233] = 0.0;
   out_4308325429962827123[234] = 0.0;
   out_4308325429962827123[235] = 0.0;
   out_4308325429962827123[236] = 0.0;
   out_4308325429962827123[237] = 0.0;
   out_4308325429962827123[238] = 0.0;
   out_4308325429962827123[239] = 0.0;
   out_4308325429962827123[240] = 0.0;
   out_4308325429962827123[241] = 0.0;
   out_4308325429962827123[242] = 0.0;
   out_4308325429962827123[243] = 0.0;
   out_4308325429962827123[244] = 0.0;
   out_4308325429962827123[245] = 0.0;
   out_4308325429962827123[246] = 0.0;
   out_4308325429962827123[247] = 1.0;
   out_4308325429962827123[248] = 0.0;
   out_4308325429962827123[249] = 0.0;
   out_4308325429962827123[250] = 0.0;
   out_4308325429962827123[251] = 0.0;
   out_4308325429962827123[252] = 0.0;
   out_4308325429962827123[253] = 0.0;
   out_4308325429962827123[254] = 0.0;
   out_4308325429962827123[255] = 0.0;
   out_4308325429962827123[256] = 0.0;
   out_4308325429962827123[257] = 0.0;
   out_4308325429962827123[258] = 0.0;
   out_4308325429962827123[259] = 0.0;
   out_4308325429962827123[260] = 0.0;
   out_4308325429962827123[261] = 0.0;
   out_4308325429962827123[262] = 0.0;
   out_4308325429962827123[263] = 0.0;
   out_4308325429962827123[264] = 0.0;
   out_4308325429962827123[265] = 0.0;
   out_4308325429962827123[266] = 1.0;
   out_4308325429962827123[267] = 0.0;
   out_4308325429962827123[268] = 0.0;
   out_4308325429962827123[269] = 0.0;
   out_4308325429962827123[270] = 0.0;
   out_4308325429962827123[271] = 0.0;
   out_4308325429962827123[272] = 0.0;
   out_4308325429962827123[273] = 0.0;
   out_4308325429962827123[274] = 0.0;
   out_4308325429962827123[275] = 0.0;
   out_4308325429962827123[276] = 0.0;
   out_4308325429962827123[277] = 0.0;
   out_4308325429962827123[278] = 0.0;
   out_4308325429962827123[279] = 0.0;
   out_4308325429962827123[280] = 0.0;
   out_4308325429962827123[281] = 0.0;
   out_4308325429962827123[282] = 0.0;
   out_4308325429962827123[283] = 0.0;
   out_4308325429962827123[284] = 0.0;
   out_4308325429962827123[285] = 1.0;
   out_4308325429962827123[286] = 0.0;
   out_4308325429962827123[287] = 0.0;
   out_4308325429962827123[288] = 0.0;
   out_4308325429962827123[289] = 0.0;
   out_4308325429962827123[290] = 0.0;
   out_4308325429962827123[291] = 0.0;
   out_4308325429962827123[292] = 0.0;
   out_4308325429962827123[293] = 0.0;
   out_4308325429962827123[294] = 0.0;
   out_4308325429962827123[295] = 0.0;
   out_4308325429962827123[296] = 0.0;
   out_4308325429962827123[297] = 0.0;
   out_4308325429962827123[298] = 0.0;
   out_4308325429962827123[299] = 0.0;
   out_4308325429962827123[300] = 0.0;
   out_4308325429962827123[301] = 0.0;
   out_4308325429962827123[302] = 0.0;
   out_4308325429962827123[303] = 0.0;
   out_4308325429962827123[304] = 1.0;
   out_4308325429962827123[305] = 0.0;
   out_4308325429962827123[306] = 0.0;
   out_4308325429962827123[307] = 0.0;
   out_4308325429962827123[308] = 0.0;
   out_4308325429962827123[309] = 0.0;
   out_4308325429962827123[310] = 0.0;
   out_4308325429962827123[311] = 0.0;
   out_4308325429962827123[312] = 0.0;
   out_4308325429962827123[313] = 0.0;
   out_4308325429962827123[314] = 0.0;
   out_4308325429962827123[315] = 0.0;
   out_4308325429962827123[316] = 0.0;
   out_4308325429962827123[317] = 0.0;
   out_4308325429962827123[318] = 0.0;
   out_4308325429962827123[319] = 0.0;
   out_4308325429962827123[320] = 0.0;
   out_4308325429962827123[321] = 0.0;
   out_4308325429962827123[322] = 0.0;
   out_4308325429962827123[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_6784884137237425870) {
   out_6784884137237425870[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_6784884137237425870[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_6784884137237425870[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_6784884137237425870[3] = dt*state[12] + state[3];
   out_6784884137237425870[4] = dt*state[13] + state[4];
   out_6784884137237425870[5] = dt*state[14] + state[5];
   out_6784884137237425870[6] = state[6];
   out_6784884137237425870[7] = state[7];
   out_6784884137237425870[8] = state[8];
   out_6784884137237425870[9] = state[9];
   out_6784884137237425870[10] = state[10];
   out_6784884137237425870[11] = state[11];
   out_6784884137237425870[12] = state[12];
   out_6784884137237425870[13] = state[13];
   out_6784884137237425870[14] = state[14];
   out_6784884137237425870[15] = state[15];
   out_6784884137237425870[16] = state[16];
   out_6784884137237425870[17] = state[17];
}
void F_fun(double *state, double dt, double *out_1899404572103969321) {
   out_1899404572103969321[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1899404572103969321[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1899404572103969321[2] = 0;
   out_1899404572103969321[3] = 0;
   out_1899404572103969321[4] = 0;
   out_1899404572103969321[5] = 0;
   out_1899404572103969321[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1899404572103969321[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1899404572103969321[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1899404572103969321[9] = 0;
   out_1899404572103969321[10] = 0;
   out_1899404572103969321[11] = 0;
   out_1899404572103969321[12] = 0;
   out_1899404572103969321[13] = 0;
   out_1899404572103969321[14] = 0;
   out_1899404572103969321[15] = 0;
   out_1899404572103969321[16] = 0;
   out_1899404572103969321[17] = 0;
   out_1899404572103969321[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1899404572103969321[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1899404572103969321[20] = 0;
   out_1899404572103969321[21] = 0;
   out_1899404572103969321[22] = 0;
   out_1899404572103969321[23] = 0;
   out_1899404572103969321[24] = 0;
   out_1899404572103969321[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1899404572103969321[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1899404572103969321[27] = 0;
   out_1899404572103969321[28] = 0;
   out_1899404572103969321[29] = 0;
   out_1899404572103969321[30] = 0;
   out_1899404572103969321[31] = 0;
   out_1899404572103969321[32] = 0;
   out_1899404572103969321[33] = 0;
   out_1899404572103969321[34] = 0;
   out_1899404572103969321[35] = 0;
   out_1899404572103969321[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1899404572103969321[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1899404572103969321[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1899404572103969321[39] = 0;
   out_1899404572103969321[40] = 0;
   out_1899404572103969321[41] = 0;
   out_1899404572103969321[42] = 0;
   out_1899404572103969321[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1899404572103969321[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1899404572103969321[45] = 0;
   out_1899404572103969321[46] = 0;
   out_1899404572103969321[47] = 0;
   out_1899404572103969321[48] = 0;
   out_1899404572103969321[49] = 0;
   out_1899404572103969321[50] = 0;
   out_1899404572103969321[51] = 0;
   out_1899404572103969321[52] = 0;
   out_1899404572103969321[53] = 0;
   out_1899404572103969321[54] = 0;
   out_1899404572103969321[55] = 0;
   out_1899404572103969321[56] = 0;
   out_1899404572103969321[57] = 1;
   out_1899404572103969321[58] = 0;
   out_1899404572103969321[59] = 0;
   out_1899404572103969321[60] = 0;
   out_1899404572103969321[61] = 0;
   out_1899404572103969321[62] = 0;
   out_1899404572103969321[63] = 0;
   out_1899404572103969321[64] = 0;
   out_1899404572103969321[65] = 0;
   out_1899404572103969321[66] = dt;
   out_1899404572103969321[67] = 0;
   out_1899404572103969321[68] = 0;
   out_1899404572103969321[69] = 0;
   out_1899404572103969321[70] = 0;
   out_1899404572103969321[71] = 0;
   out_1899404572103969321[72] = 0;
   out_1899404572103969321[73] = 0;
   out_1899404572103969321[74] = 0;
   out_1899404572103969321[75] = 0;
   out_1899404572103969321[76] = 1;
   out_1899404572103969321[77] = 0;
   out_1899404572103969321[78] = 0;
   out_1899404572103969321[79] = 0;
   out_1899404572103969321[80] = 0;
   out_1899404572103969321[81] = 0;
   out_1899404572103969321[82] = 0;
   out_1899404572103969321[83] = 0;
   out_1899404572103969321[84] = 0;
   out_1899404572103969321[85] = dt;
   out_1899404572103969321[86] = 0;
   out_1899404572103969321[87] = 0;
   out_1899404572103969321[88] = 0;
   out_1899404572103969321[89] = 0;
   out_1899404572103969321[90] = 0;
   out_1899404572103969321[91] = 0;
   out_1899404572103969321[92] = 0;
   out_1899404572103969321[93] = 0;
   out_1899404572103969321[94] = 0;
   out_1899404572103969321[95] = 1;
   out_1899404572103969321[96] = 0;
   out_1899404572103969321[97] = 0;
   out_1899404572103969321[98] = 0;
   out_1899404572103969321[99] = 0;
   out_1899404572103969321[100] = 0;
   out_1899404572103969321[101] = 0;
   out_1899404572103969321[102] = 0;
   out_1899404572103969321[103] = 0;
   out_1899404572103969321[104] = dt;
   out_1899404572103969321[105] = 0;
   out_1899404572103969321[106] = 0;
   out_1899404572103969321[107] = 0;
   out_1899404572103969321[108] = 0;
   out_1899404572103969321[109] = 0;
   out_1899404572103969321[110] = 0;
   out_1899404572103969321[111] = 0;
   out_1899404572103969321[112] = 0;
   out_1899404572103969321[113] = 0;
   out_1899404572103969321[114] = 1;
   out_1899404572103969321[115] = 0;
   out_1899404572103969321[116] = 0;
   out_1899404572103969321[117] = 0;
   out_1899404572103969321[118] = 0;
   out_1899404572103969321[119] = 0;
   out_1899404572103969321[120] = 0;
   out_1899404572103969321[121] = 0;
   out_1899404572103969321[122] = 0;
   out_1899404572103969321[123] = 0;
   out_1899404572103969321[124] = 0;
   out_1899404572103969321[125] = 0;
   out_1899404572103969321[126] = 0;
   out_1899404572103969321[127] = 0;
   out_1899404572103969321[128] = 0;
   out_1899404572103969321[129] = 0;
   out_1899404572103969321[130] = 0;
   out_1899404572103969321[131] = 0;
   out_1899404572103969321[132] = 0;
   out_1899404572103969321[133] = 1;
   out_1899404572103969321[134] = 0;
   out_1899404572103969321[135] = 0;
   out_1899404572103969321[136] = 0;
   out_1899404572103969321[137] = 0;
   out_1899404572103969321[138] = 0;
   out_1899404572103969321[139] = 0;
   out_1899404572103969321[140] = 0;
   out_1899404572103969321[141] = 0;
   out_1899404572103969321[142] = 0;
   out_1899404572103969321[143] = 0;
   out_1899404572103969321[144] = 0;
   out_1899404572103969321[145] = 0;
   out_1899404572103969321[146] = 0;
   out_1899404572103969321[147] = 0;
   out_1899404572103969321[148] = 0;
   out_1899404572103969321[149] = 0;
   out_1899404572103969321[150] = 0;
   out_1899404572103969321[151] = 0;
   out_1899404572103969321[152] = 1;
   out_1899404572103969321[153] = 0;
   out_1899404572103969321[154] = 0;
   out_1899404572103969321[155] = 0;
   out_1899404572103969321[156] = 0;
   out_1899404572103969321[157] = 0;
   out_1899404572103969321[158] = 0;
   out_1899404572103969321[159] = 0;
   out_1899404572103969321[160] = 0;
   out_1899404572103969321[161] = 0;
   out_1899404572103969321[162] = 0;
   out_1899404572103969321[163] = 0;
   out_1899404572103969321[164] = 0;
   out_1899404572103969321[165] = 0;
   out_1899404572103969321[166] = 0;
   out_1899404572103969321[167] = 0;
   out_1899404572103969321[168] = 0;
   out_1899404572103969321[169] = 0;
   out_1899404572103969321[170] = 0;
   out_1899404572103969321[171] = 1;
   out_1899404572103969321[172] = 0;
   out_1899404572103969321[173] = 0;
   out_1899404572103969321[174] = 0;
   out_1899404572103969321[175] = 0;
   out_1899404572103969321[176] = 0;
   out_1899404572103969321[177] = 0;
   out_1899404572103969321[178] = 0;
   out_1899404572103969321[179] = 0;
   out_1899404572103969321[180] = 0;
   out_1899404572103969321[181] = 0;
   out_1899404572103969321[182] = 0;
   out_1899404572103969321[183] = 0;
   out_1899404572103969321[184] = 0;
   out_1899404572103969321[185] = 0;
   out_1899404572103969321[186] = 0;
   out_1899404572103969321[187] = 0;
   out_1899404572103969321[188] = 0;
   out_1899404572103969321[189] = 0;
   out_1899404572103969321[190] = 1;
   out_1899404572103969321[191] = 0;
   out_1899404572103969321[192] = 0;
   out_1899404572103969321[193] = 0;
   out_1899404572103969321[194] = 0;
   out_1899404572103969321[195] = 0;
   out_1899404572103969321[196] = 0;
   out_1899404572103969321[197] = 0;
   out_1899404572103969321[198] = 0;
   out_1899404572103969321[199] = 0;
   out_1899404572103969321[200] = 0;
   out_1899404572103969321[201] = 0;
   out_1899404572103969321[202] = 0;
   out_1899404572103969321[203] = 0;
   out_1899404572103969321[204] = 0;
   out_1899404572103969321[205] = 0;
   out_1899404572103969321[206] = 0;
   out_1899404572103969321[207] = 0;
   out_1899404572103969321[208] = 0;
   out_1899404572103969321[209] = 1;
   out_1899404572103969321[210] = 0;
   out_1899404572103969321[211] = 0;
   out_1899404572103969321[212] = 0;
   out_1899404572103969321[213] = 0;
   out_1899404572103969321[214] = 0;
   out_1899404572103969321[215] = 0;
   out_1899404572103969321[216] = 0;
   out_1899404572103969321[217] = 0;
   out_1899404572103969321[218] = 0;
   out_1899404572103969321[219] = 0;
   out_1899404572103969321[220] = 0;
   out_1899404572103969321[221] = 0;
   out_1899404572103969321[222] = 0;
   out_1899404572103969321[223] = 0;
   out_1899404572103969321[224] = 0;
   out_1899404572103969321[225] = 0;
   out_1899404572103969321[226] = 0;
   out_1899404572103969321[227] = 0;
   out_1899404572103969321[228] = 1;
   out_1899404572103969321[229] = 0;
   out_1899404572103969321[230] = 0;
   out_1899404572103969321[231] = 0;
   out_1899404572103969321[232] = 0;
   out_1899404572103969321[233] = 0;
   out_1899404572103969321[234] = 0;
   out_1899404572103969321[235] = 0;
   out_1899404572103969321[236] = 0;
   out_1899404572103969321[237] = 0;
   out_1899404572103969321[238] = 0;
   out_1899404572103969321[239] = 0;
   out_1899404572103969321[240] = 0;
   out_1899404572103969321[241] = 0;
   out_1899404572103969321[242] = 0;
   out_1899404572103969321[243] = 0;
   out_1899404572103969321[244] = 0;
   out_1899404572103969321[245] = 0;
   out_1899404572103969321[246] = 0;
   out_1899404572103969321[247] = 1;
   out_1899404572103969321[248] = 0;
   out_1899404572103969321[249] = 0;
   out_1899404572103969321[250] = 0;
   out_1899404572103969321[251] = 0;
   out_1899404572103969321[252] = 0;
   out_1899404572103969321[253] = 0;
   out_1899404572103969321[254] = 0;
   out_1899404572103969321[255] = 0;
   out_1899404572103969321[256] = 0;
   out_1899404572103969321[257] = 0;
   out_1899404572103969321[258] = 0;
   out_1899404572103969321[259] = 0;
   out_1899404572103969321[260] = 0;
   out_1899404572103969321[261] = 0;
   out_1899404572103969321[262] = 0;
   out_1899404572103969321[263] = 0;
   out_1899404572103969321[264] = 0;
   out_1899404572103969321[265] = 0;
   out_1899404572103969321[266] = 1;
   out_1899404572103969321[267] = 0;
   out_1899404572103969321[268] = 0;
   out_1899404572103969321[269] = 0;
   out_1899404572103969321[270] = 0;
   out_1899404572103969321[271] = 0;
   out_1899404572103969321[272] = 0;
   out_1899404572103969321[273] = 0;
   out_1899404572103969321[274] = 0;
   out_1899404572103969321[275] = 0;
   out_1899404572103969321[276] = 0;
   out_1899404572103969321[277] = 0;
   out_1899404572103969321[278] = 0;
   out_1899404572103969321[279] = 0;
   out_1899404572103969321[280] = 0;
   out_1899404572103969321[281] = 0;
   out_1899404572103969321[282] = 0;
   out_1899404572103969321[283] = 0;
   out_1899404572103969321[284] = 0;
   out_1899404572103969321[285] = 1;
   out_1899404572103969321[286] = 0;
   out_1899404572103969321[287] = 0;
   out_1899404572103969321[288] = 0;
   out_1899404572103969321[289] = 0;
   out_1899404572103969321[290] = 0;
   out_1899404572103969321[291] = 0;
   out_1899404572103969321[292] = 0;
   out_1899404572103969321[293] = 0;
   out_1899404572103969321[294] = 0;
   out_1899404572103969321[295] = 0;
   out_1899404572103969321[296] = 0;
   out_1899404572103969321[297] = 0;
   out_1899404572103969321[298] = 0;
   out_1899404572103969321[299] = 0;
   out_1899404572103969321[300] = 0;
   out_1899404572103969321[301] = 0;
   out_1899404572103969321[302] = 0;
   out_1899404572103969321[303] = 0;
   out_1899404572103969321[304] = 1;
   out_1899404572103969321[305] = 0;
   out_1899404572103969321[306] = 0;
   out_1899404572103969321[307] = 0;
   out_1899404572103969321[308] = 0;
   out_1899404572103969321[309] = 0;
   out_1899404572103969321[310] = 0;
   out_1899404572103969321[311] = 0;
   out_1899404572103969321[312] = 0;
   out_1899404572103969321[313] = 0;
   out_1899404572103969321[314] = 0;
   out_1899404572103969321[315] = 0;
   out_1899404572103969321[316] = 0;
   out_1899404572103969321[317] = 0;
   out_1899404572103969321[318] = 0;
   out_1899404572103969321[319] = 0;
   out_1899404572103969321[320] = 0;
   out_1899404572103969321[321] = 0;
   out_1899404572103969321[322] = 0;
   out_1899404572103969321[323] = 1;
}
void h_4(double *state, double *unused, double *out_8673137181907007987) {
   out_8673137181907007987[0] = state[6] + state[9];
   out_8673137181907007987[1] = state[7] + state[10];
   out_8673137181907007987[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_536518220118919551) {
   out_536518220118919551[0] = 0;
   out_536518220118919551[1] = 0;
   out_536518220118919551[2] = 0;
   out_536518220118919551[3] = 0;
   out_536518220118919551[4] = 0;
   out_536518220118919551[5] = 0;
   out_536518220118919551[6] = 1;
   out_536518220118919551[7] = 0;
   out_536518220118919551[8] = 0;
   out_536518220118919551[9] = 1;
   out_536518220118919551[10] = 0;
   out_536518220118919551[11] = 0;
   out_536518220118919551[12] = 0;
   out_536518220118919551[13] = 0;
   out_536518220118919551[14] = 0;
   out_536518220118919551[15] = 0;
   out_536518220118919551[16] = 0;
   out_536518220118919551[17] = 0;
   out_536518220118919551[18] = 0;
   out_536518220118919551[19] = 0;
   out_536518220118919551[20] = 0;
   out_536518220118919551[21] = 0;
   out_536518220118919551[22] = 0;
   out_536518220118919551[23] = 0;
   out_536518220118919551[24] = 0;
   out_536518220118919551[25] = 1;
   out_536518220118919551[26] = 0;
   out_536518220118919551[27] = 0;
   out_536518220118919551[28] = 1;
   out_536518220118919551[29] = 0;
   out_536518220118919551[30] = 0;
   out_536518220118919551[31] = 0;
   out_536518220118919551[32] = 0;
   out_536518220118919551[33] = 0;
   out_536518220118919551[34] = 0;
   out_536518220118919551[35] = 0;
   out_536518220118919551[36] = 0;
   out_536518220118919551[37] = 0;
   out_536518220118919551[38] = 0;
   out_536518220118919551[39] = 0;
   out_536518220118919551[40] = 0;
   out_536518220118919551[41] = 0;
   out_536518220118919551[42] = 0;
   out_536518220118919551[43] = 0;
   out_536518220118919551[44] = 1;
   out_536518220118919551[45] = 0;
   out_536518220118919551[46] = 0;
   out_536518220118919551[47] = 1;
   out_536518220118919551[48] = 0;
   out_536518220118919551[49] = 0;
   out_536518220118919551[50] = 0;
   out_536518220118919551[51] = 0;
   out_536518220118919551[52] = 0;
   out_536518220118919551[53] = 0;
}
void h_10(double *state, double *unused, double *out_4049385380413447719) {
   out_4049385380413447719[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_4049385380413447719[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_4049385380413447719[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_393587758664493186) {
   out_393587758664493186[0] = 0;
   out_393587758664493186[1] = 9.8100000000000005*cos(state[1]);
   out_393587758664493186[2] = 0;
   out_393587758664493186[3] = 0;
   out_393587758664493186[4] = -state[8];
   out_393587758664493186[5] = state[7];
   out_393587758664493186[6] = 0;
   out_393587758664493186[7] = state[5];
   out_393587758664493186[8] = -state[4];
   out_393587758664493186[9] = 0;
   out_393587758664493186[10] = 0;
   out_393587758664493186[11] = 0;
   out_393587758664493186[12] = 1;
   out_393587758664493186[13] = 0;
   out_393587758664493186[14] = 0;
   out_393587758664493186[15] = 1;
   out_393587758664493186[16] = 0;
   out_393587758664493186[17] = 0;
   out_393587758664493186[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_393587758664493186[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_393587758664493186[20] = 0;
   out_393587758664493186[21] = state[8];
   out_393587758664493186[22] = 0;
   out_393587758664493186[23] = -state[6];
   out_393587758664493186[24] = -state[5];
   out_393587758664493186[25] = 0;
   out_393587758664493186[26] = state[3];
   out_393587758664493186[27] = 0;
   out_393587758664493186[28] = 0;
   out_393587758664493186[29] = 0;
   out_393587758664493186[30] = 0;
   out_393587758664493186[31] = 1;
   out_393587758664493186[32] = 0;
   out_393587758664493186[33] = 0;
   out_393587758664493186[34] = 1;
   out_393587758664493186[35] = 0;
   out_393587758664493186[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_393587758664493186[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_393587758664493186[38] = 0;
   out_393587758664493186[39] = -state[7];
   out_393587758664493186[40] = state[6];
   out_393587758664493186[41] = 0;
   out_393587758664493186[42] = state[4];
   out_393587758664493186[43] = -state[3];
   out_393587758664493186[44] = 0;
   out_393587758664493186[45] = 0;
   out_393587758664493186[46] = 0;
   out_393587758664493186[47] = 0;
   out_393587758664493186[48] = 0;
   out_393587758664493186[49] = 0;
   out_393587758664493186[50] = 1;
   out_393587758664493186[51] = 0;
   out_393587758664493186[52] = 0;
   out_393587758664493186[53] = 1;
}
void h_13(double *state, double *unused, double *out_7785508574307562513) {
   out_7785508574307562513[0] = state[3];
   out_7785508574307562513[1] = state[4];
   out_7785508574307562513[2] = state[5];
}
void H_13(double *state, double *unused, double *out_4370273683421443575) {
   out_4370273683421443575[0] = 0;
   out_4370273683421443575[1] = 0;
   out_4370273683421443575[2] = 0;
   out_4370273683421443575[3] = 1;
   out_4370273683421443575[4] = 0;
   out_4370273683421443575[5] = 0;
   out_4370273683421443575[6] = 0;
   out_4370273683421443575[7] = 0;
   out_4370273683421443575[8] = 0;
   out_4370273683421443575[9] = 0;
   out_4370273683421443575[10] = 0;
   out_4370273683421443575[11] = 0;
   out_4370273683421443575[12] = 0;
   out_4370273683421443575[13] = 0;
   out_4370273683421443575[14] = 0;
   out_4370273683421443575[15] = 0;
   out_4370273683421443575[16] = 0;
   out_4370273683421443575[17] = 0;
   out_4370273683421443575[18] = 0;
   out_4370273683421443575[19] = 0;
   out_4370273683421443575[20] = 0;
   out_4370273683421443575[21] = 0;
   out_4370273683421443575[22] = 1;
   out_4370273683421443575[23] = 0;
   out_4370273683421443575[24] = 0;
   out_4370273683421443575[25] = 0;
   out_4370273683421443575[26] = 0;
   out_4370273683421443575[27] = 0;
   out_4370273683421443575[28] = 0;
   out_4370273683421443575[29] = 0;
   out_4370273683421443575[30] = 0;
   out_4370273683421443575[31] = 0;
   out_4370273683421443575[32] = 0;
   out_4370273683421443575[33] = 0;
   out_4370273683421443575[34] = 0;
   out_4370273683421443575[35] = 0;
   out_4370273683421443575[36] = 0;
   out_4370273683421443575[37] = 0;
   out_4370273683421443575[38] = 0;
   out_4370273683421443575[39] = 0;
   out_4370273683421443575[40] = 0;
   out_4370273683421443575[41] = 1;
   out_4370273683421443575[42] = 0;
   out_4370273683421443575[43] = 0;
   out_4370273683421443575[44] = 0;
   out_4370273683421443575[45] = 0;
   out_4370273683421443575[46] = 0;
   out_4370273683421443575[47] = 0;
   out_4370273683421443575[48] = 0;
   out_4370273683421443575[49] = 0;
   out_4370273683421443575[50] = 0;
   out_4370273683421443575[51] = 0;
   out_4370273683421443575[52] = 0;
   out_4370273683421443575[53] = 0;
}
void h_14(double *state, double *unused, double *out_8347004880535916513) {
   out_8347004880535916513[0] = state[6];
   out_8347004880535916513[1] = state[7];
   out_8347004880535916513[2] = state[8];
}
void H_14(double *state, double *unused, double *out_3619306652414291847) {
   out_3619306652414291847[0] = 0;
   out_3619306652414291847[1] = 0;
   out_3619306652414291847[2] = 0;
   out_3619306652414291847[3] = 0;
   out_3619306652414291847[4] = 0;
   out_3619306652414291847[5] = 0;
   out_3619306652414291847[6] = 1;
   out_3619306652414291847[7] = 0;
   out_3619306652414291847[8] = 0;
   out_3619306652414291847[9] = 0;
   out_3619306652414291847[10] = 0;
   out_3619306652414291847[11] = 0;
   out_3619306652414291847[12] = 0;
   out_3619306652414291847[13] = 0;
   out_3619306652414291847[14] = 0;
   out_3619306652414291847[15] = 0;
   out_3619306652414291847[16] = 0;
   out_3619306652414291847[17] = 0;
   out_3619306652414291847[18] = 0;
   out_3619306652414291847[19] = 0;
   out_3619306652414291847[20] = 0;
   out_3619306652414291847[21] = 0;
   out_3619306652414291847[22] = 0;
   out_3619306652414291847[23] = 0;
   out_3619306652414291847[24] = 0;
   out_3619306652414291847[25] = 1;
   out_3619306652414291847[26] = 0;
   out_3619306652414291847[27] = 0;
   out_3619306652414291847[28] = 0;
   out_3619306652414291847[29] = 0;
   out_3619306652414291847[30] = 0;
   out_3619306652414291847[31] = 0;
   out_3619306652414291847[32] = 0;
   out_3619306652414291847[33] = 0;
   out_3619306652414291847[34] = 0;
   out_3619306652414291847[35] = 0;
   out_3619306652414291847[36] = 0;
   out_3619306652414291847[37] = 0;
   out_3619306652414291847[38] = 0;
   out_3619306652414291847[39] = 0;
   out_3619306652414291847[40] = 0;
   out_3619306652414291847[41] = 0;
   out_3619306652414291847[42] = 0;
   out_3619306652414291847[43] = 0;
   out_3619306652414291847[44] = 1;
   out_3619306652414291847[45] = 0;
   out_3619306652414291847[46] = 0;
   out_3619306652414291847[47] = 0;
   out_3619306652414291847[48] = 0;
   out_3619306652414291847[49] = 0;
   out_3619306652414291847[50] = 0;
   out_3619306652414291847[51] = 0;
   out_3619306652414291847[52] = 0;
   out_3619306652414291847[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_6433275272970939599) {
  err_fun(nom_x, delta_x, out_6433275272970939599);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7086127294916974707) {
  inv_err_fun(nom_x, true_x, out_7086127294916974707);
}
void pose_H_mod_fun(double *state, double *out_4308325429962827123) {
  H_mod_fun(state, out_4308325429962827123);
}
void pose_f_fun(double *state, double dt, double *out_6784884137237425870) {
  f_fun(state,  dt, out_6784884137237425870);
}
void pose_F_fun(double *state, double dt, double *out_1899404572103969321) {
  F_fun(state,  dt, out_1899404572103969321);
}
void pose_h_4(double *state, double *unused, double *out_8673137181907007987) {
  h_4(state, unused, out_8673137181907007987);
}
void pose_H_4(double *state, double *unused, double *out_536518220118919551) {
  H_4(state, unused, out_536518220118919551);
}
void pose_h_10(double *state, double *unused, double *out_4049385380413447719) {
  h_10(state, unused, out_4049385380413447719);
}
void pose_H_10(double *state, double *unused, double *out_393587758664493186) {
  H_10(state, unused, out_393587758664493186);
}
void pose_h_13(double *state, double *unused, double *out_7785508574307562513) {
  h_13(state, unused, out_7785508574307562513);
}
void pose_H_13(double *state, double *unused, double *out_4370273683421443575) {
  H_13(state, unused, out_4370273683421443575);
}
void pose_h_14(double *state, double *unused, double *out_8347004880535916513) {
  h_14(state, unused, out_8347004880535916513);
}
void pose_H_14(double *state, double *unused, double *out_3619306652414291847) {
  H_14(state, unused, out_3619306652414291847);
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
