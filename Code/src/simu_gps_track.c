/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file simu_gps_track.c
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief  This file simulates a GPS navigation track
 *
 ******************************************************************************/


#include "simu_gps_track.h"
#include "time_keeper.h"
#include "print_util.h"

#define TRACK 2

#if TRACK == 0
	#define SIZE_MAX_TABLE 302
	void simu_gps_track_w_track(void);
#elif TRACK == 1
	#define SIZE_MAX_TABLE 205
	void simu_gps_track_square_track(void);
#elif TRACK == 2
	#define SIZE_MAX_TABLE 106
	void simu_gps_track_jumping_track(void);
#elif TRACK == 3
	#define SIZE_MAX_TABLE 400
	void simu_gps_track_circular_track(void);
#else
	#error "Unknown track scenario"
#endif

mavlink_global_position_int_t simu_track_table[SIZE_MAX_TABLE];

task_return_t simu_gps_track_pack_msg(simu_gps_track_t* simu_gps_track)
{
	if (simu_gps_track->i >= SIZE_MAX_TABLE)
	{
		simu_gps_track->i = 0;
	}

	mavlink_message_t msg;

	mavlink_msg_global_position_int_pack(	1,
											simu_gps_track->mavlink_stream->compid,
											&msg,
											time_keeper_get_millis(),
											simu_track_table[simu_gps_track->i].lat,
											simu_track_table[simu_gps_track->i].lon,
											simu_track_table[simu_gps_track->i].alt*1000,
											-simu_track_table[simu_gps_track->i].relative_alt*1000,
											simu_track_table[simu_gps_track->i].vx,
											simu_track_table[simu_gps_track->i].vy,
											simu_track_table[simu_gps_track->i].vz,
											simu_track_table[simu_gps_track->i].hdg);

	mavlink_stream_send(simu_gps_track->mavlink_stream,&msg);

	neighbors_selection_read_message_from_neighbors(simu_gps_track->neighbors, simu_gps_track->mavlink_stream->sysid, &msg);

	simu_gps_track->i+= MSG_PERIOD_SEC * 4;

	return TASK_RUN_SUCCESS;
}

void simu_gps_track_send_neighbor_heartbeat(simu_gps_track_t* simu_gps_track)
{
	mavlink_message_t msg;

	mavlink_msg_heartbeat_pack(	1,
								simu_gps_track->mavlink_stream->compid,
								&msg,
								simu_gps_track->state->autopilot_type,
								simu_gps_track->state->autopilot_name,
								MAV_MODE_GPS_NAVIGATION,
								0,
								MAV_STATE_ACTIVE);

	mavlink_stream_send(simu_gps_track->mavlink_stream,&msg);
}

void simu_gps_track_send_msg(simu_gps_track_t* simu_gps_track, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	if (simu_gps_track->i >= SIZE_MAX_TABLE)
	{
		simu_gps_track->i = 0;
	}

	mavlink_msg_global_position_int_pack(	1,
											mavlink_stream->compid,
											msg,
											time_keeper_get_millis(),
											simu_track_table[simu_gps_track->i].lat,
											simu_track_table[simu_gps_track->i].lon,
											simu_track_table[simu_gps_track->i].alt*1000,
											-simu_track_table[simu_gps_track->i].relative_alt*1000,
											simu_track_table[simu_gps_track->i].vx,
											simu_track_table[simu_gps_track->i].vy,
											simu_track_table[simu_gps_track->i].vz,
											simu_track_table[simu_gps_track->i].hdg);

	simu_gps_track->i+= MSG_PERIOD_SEC * 4;
}

void simu_gps_track_init(simu_gps_track_t* simu_gps_track, neighbors_t* neighbors, const mavlink_stream_t* mavlink_stream, const state_t* state)
{
	simu_gps_track->mavlink_stream = mavlink_stream;
	simu_gps_track->neighbors = neighbors;
	simu_gps_track->state = state;

	simu_gps_track->i = 0;

	#if TRACK == 0
		simu_gps_track_w_track();
	#elif TRACK == 1
		simu_gps_track_square_track();
	#elif TRACK == 2
		simu_gps_track_jumping_track();
	#elif TRACK == 3
		simu_gps_track_circular_track();
	#endif
}

#if TRACK == 0

void simu_gps_track_w_track(void)
{
	simu_track_table[0].lat = 465183178;
	simu_track_table[0].lon = 65660100;
	simu_track_table[0].alt = 407;
	simu_track_table[0].relative_alt = -7;
	simu_track_table[0].vx = 1;
	simu_track_table[0].vy = -0;
	simu_track_table[0].vz = 0;
	simu_track_table[0].hdg = 160;

	simu_track_table[1].lat = 465183178;
	simu_track_table[1].lon = 65660100;
	simu_track_table[1].alt = 407;
	simu_track_table[1].relative_alt = -7;
	simu_track_table[1].vx = 1;
	simu_track_table[1].vy = -0;
	simu_track_table[1].vz = 1;
	simu_track_table[1].hdg = 169;

	simu_track_table[2].lat = 465183179;
	simu_track_table[2].lon = 65660100;
	simu_track_table[2].alt = 407;
	simu_track_table[2].relative_alt = -7;
	simu_track_table[2].vx = 9;
	simu_track_table[2].vy = 9;
	simu_track_table[2].vz = 12;
	simu_track_table[2].hdg = 224;

	simu_track_table[3].lat = 465183185;
	simu_track_table[3].lon = 65660109;
	simu_track_table[3].alt = 407;
	simu_track_table[3].relative_alt = -7;
	simu_track_table[3].vx = 50;
	simu_track_table[3].vy = 48;
	simu_track_table[3].vz = 23;
	simu_track_table[3].hdg = 224;

	simu_track_table[4].lat = 465183204;
	simu_track_table[4].lon = 65660134;
	simu_track_table[4].alt = 407;
	simu_track_table[4].relative_alt = -7;
	simu_track_table[4].vx = 126;
	simu_track_table[4].vy = 111;
	simu_track_table[4].vz = 40;
	simu_track_table[4].hdg = 221;

	simu_track_table[5].lat = 465183243;
	simu_track_table[5].lon = 65660182;
	simu_track_table[5].alt = 407;
	simu_track_table[5].relative_alt = -7;
	simu_track_table[5].vx = 208;
	simu_track_table[5].vy = 170;
	simu_track_table[5].vz = 50;
	simu_track_table[5].hdg = 219;

	simu_track_table[6].lat = 465183297;
	simu_track_table[6].lon = 65660243;
	simu_track_table[6].alt = 407;
	simu_track_table[6].relative_alt = -7;
	simu_track_table[6].vx = 263;
	simu_track_table[6].vy = 202;
	simu_track_table[6].vz = 46;
	simu_track_table[6].hdg = 217;

	simu_track_table[7].lat = 465183362;
	simu_track_table[7].lon = 65660313;
	simu_track_table[7].alt = 407;
	simu_track_table[7].relative_alt = -7;
	simu_track_table[7].vx = 285;
	simu_track_table[7].vy = 207;
	simu_track_table[7].vz = 32;
	simu_track_table[7].hdg = 216;

	simu_track_table[8].lat = 465183429;
	simu_track_table[8].lon = 65660382;
	simu_track_table[8].alt = 407;
	simu_track_table[8].relative_alt = -7;
	simu_track_table[8].vx = 279;
	simu_track_table[8].vy = 194;
	simu_track_table[8].vz = 20;
	simu_track_table[8].hdg = 215;

	simu_track_table[9].lat = 465183494;
	simu_track_table[9].lon = 65660446;
	simu_track_table[9].alt = 407;
	simu_track_table[9].relative_alt = -7;
	simu_track_table[9].vx = 264;
	simu_track_table[9].vy = 179;
	simu_track_table[9].vz = 12;
	simu_track_table[9].hdg = 214;

	simu_track_table[10].lat = 465183555;
	simu_track_table[10].lon = 65660506;
	simu_track_table[10].alt = 407;
	simu_track_table[10].relative_alt = -7;
	simu_track_table[10].vx = 251;
	simu_track_table[10].vy = 168;
	simu_track_table[10].vz = 8;
	simu_track_table[10].hdg = 214;

	simu_track_table[11].lat = 465183617;
	simu_track_table[11].lon = 65660565;
	simu_track_table[11].alt = 407;
	simu_track_table[11].relative_alt = -7;
	simu_track_table[11].vx = 247;
	simu_track_table[11].vy = 165;
	simu_track_table[11].vz = 8;
	simu_track_table[11].hdg = 214;

	simu_track_table[12].lat = 465183675;
	simu_track_table[12].lon = 65660622;
	simu_track_table[12].alt = 407;
	simu_track_table[12].relative_alt = -7;
	simu_track_table[12].vx = 249;
	simu_track_table[12].vy = 168;
	simu_track_table[12].vz = 9;
	simu_track_table[12].hdg = 214;

	simu_track_table[13].lat = 465183737;
	simu_track_table[13].lon = 65660682;
	simu_track_table[13].alt = 406;
	simu_track_table[13].relative_alt = -6;
	simu_track_table[13].vx = 255;
	simu_track_table[13].vy = 173;
	simu_track_table[13].vz = 9;
	simu_track_table[13].hdg = 214;

	simu_track_table[14].lat = 465183799;
	simu_track_table[14].lon = 65660742;
	simu_track_table[14].alt = 406;
	simu_track_table[14].relative_alt = -6;
	simu_track_table[14].vx = 259;
	simu_track_table[14].vy = 175;
	simu_track_table[14].vz = 8;
	simu_track_table[14].hdg = 214;

	simu_track_table[15].lat = 465183862;
	simu_track_table[15].lon = 65660803;
	simu_track_table[15].alt = 406;
	simu_track_table[15].relative_alt = -6;
	simu_track_table[15].vx = 258;
	simu_track_table[15].vy = 174;
	simu_track_table[15].vz = 6;
	simu_track_table[15].hdg = 214;

	simu_track_table[16].lat = 465183924;
	simu_track_table[16].lon = 65660863;
	simu_track_table[16].alt = 406;
	simu_track_table[16].relative_alt = -6;
	simu_track_table[16].vx = 257;
	simu_track_table[16].vy = 171;
	simu_track_table[16].vz = 3;
	simu_track_table[16].hdg = 214;

	simu_track_table[17].lat = 465183986;
	simu_track_table[17].lon = 65660922;
	simu_track_table[17].alt = 406;
	simu_track_table[17].relative_alt = -6;
	simu_track_table[17].vx = 253;
	simu_track_table[17].vy = 168;
	simu_track_table[17].vz = 1;
	simu_track_table[17].hdg = 213;

	simu_track_table[18].lat = 465184046;
	simu_track_table[18].lon = 65660980;
	simu_track_table[18].alt = 406;
	simu_track_table[18].relative_alt = -6;
	simu_track_table[18].vx = 251;
	simu_track_table[18].vy = 165;
	simu_track_table[18].vz = -1;
	simu_track_table[18].hdg = 213;

	simu_track_table[19].lat = 465184107;
	simu_track_table[19].lon = 65661038;
	simu_track_table[19].alt = 406;
	simu_track_table[19].relative_alt = -6;
	simu_track_table[19].vx = 249;
	simu_track_table[19].vy = 164;
	simu_track_table[19].vz = -2;
	simu_track_table[19].hdg = 213;

	simu_track_table[20].lat = 465184166;
	simu_track_table[20].lon = 65661094;
	simu_track_table[20].alt = 406;
	simu_track_table[20].relative_alt = -6;
	simu_track_table[20].vx = 248;
	simu_track_table[20].vy = 163;
	simu_track_table[20].vz = -4;
	simu_track_table[20].hdg = 213;

	simu_track_table[21].lat = 465184227;
	simu_track_table[21].lon = 65661152;
	simu_track_table[21].alt = 406;
	simu_track_table[21].relative_alt = -6;
	simu_track_table[21].vx = 247;
	simu_track_table[21].vy = 163;
	simu_track_table[21].vz = -5;
	simu_track_table[21].hdg = 213;

	simu_track_table[22].lat = 465184286;
	simu_track_table[22].lon = 65661208;
	simu_track_table[22].alt = 406;
	simu_track_table[22].relative_alt = -6;
	simu_track_table[22].vx = 246;
	simu_track_table[22].vy = 163;
	simu_track_table[22].vz = -7;
	simu_track_table[22].hdg = 213;

	simu_track_table[23].lat = 465184346;
	simu_track_table[23].lon = 65661265;
	simu_track_table[23].alt = 406;
	simu_track_table[23].relative_alt = -6;
	simu_track_table[23].vx = 246;
	simu_track_table[23].vy = 163;
	simu_track_table[23].vz = -9;
	simu_track_table[23].hdg = 214;

	simu_track_table[24].lat = 465184404;
	simu_track_table[24].lon = 65661321;
	simu_track_table[24].alt = 407;
	simu_track_table[24].relative_alt = -7;
	simu_track_table[24].vx = 245;
	simu_track_table[24].vy = 162;
	simu_track_table[24].vz = -11;
	simu_track_table[24].hdg = 214;

	simu_track_table[25].lat = 465184465;
	simu_track_table[25].lon = 65661378;
	simu_track_table[25].alt = 407;
	simu_track_table[25].relative_alt = -7;
	simu_track_table[25].vx = 244;
	simu_track_table[25].vy = 162;
	simu_track_table[25].vz = -14;
	simu_track_table[25].hdg = 214;

	simu_track_table[26].lat = 465184522;
	simu_track_table[26].lon = 65661434;
	simu_track_table[26].alt = 407;
	simu_track_table[26].relative_alt = -7;
	simu_track_table[26].vx = 243;
	simu_track_table[26].vy = 161;
	simu_track_table[26].vz = -16;
	simu_track_table[26].hdg = 214;

	simu_track_table[27].lat = 465184581;
	simu_track_table[27].lon = 65661490;
	simu_track_table[27].alt = 407;
	simu_track_table[27].relative_alt = -7;
	simu_track_table[27].vx = 242;
	simu_track_table[27].vy = 161;
	simu_track_table[27].vz = -19;
	simu_track_table[27].hdg = 214;

	simu_track_table[28].lat = 465184639;
	simu_track_table[28].lon = 65661546;
	simu_track_table[28].alt = 407;
	simu_track_table[28].relative_alt = -7;
	simu_track_table[28].vx = 240;
	simu_track_table[28].vy = 160;
	simu_track_table[28].vz = -22;
	simu_track_table[28].hdg = 214;

	simu_track_table[29].lat = 465184697;
	simu_track_table[29].lon = 65661601;
	simu_track_table[29].alt = 407;
	simu_track_table[29].relative_alt = -7;
	simu_track_table[29].vx = 238;
	simu_track_table[29].vy = 159;
	simu_track_table[29].vz = -25;
	simu_track_table[29].hdg = 214;

	simu_track_table[30].lat = 465184754;
	simu_track_table[30].lon = 65661657;
	simu_track_table[30].alt = 407;
	simu_track_table[30].relative_alt = -7;
	simu_track_table[30].vx = 237;
	simu_track_table[30].vy = 158;
	simu_track_table[30].vz = -28;
	simu_track_table[30].hdg = 214;

	simu_track_table[31].lat = 465184811;
	simu_track_table[31].lon = 65661712;
	simu_track_table[31].alt = 407;
	simu_track_table[31].relative_alt = -7;
	simu_track_table[31].vx = 236;
	simu_track_table[31].vy = 158;
	simu_track_table[31].vz = -32;
	simu_track_table[31].hdg = 214;

	simu_track_table[32].lat = 465184867;
	simu_track_table[32].lon = 65661766;
	simu_track_table[32].alt = 407;
	simu_track_table[32].relative_alt = -7;
	simu_track_table[32].vx = 235;
	simu_track_table[32].vy = 157;
	simu_track_table[32].vz = -33;
	simu_track_table[32].hdg = 214;

	simu_track_table[33].lat = 465184924;
	simu_track_table[33].lon = 65661821;
	simu_track_table[33].alt = 407;
	simu_track_table[33].relative_alt = -7;
	simu_track_table[33].vx = 231;
	simu_track_table[33].vy = 155;
	simu_track_table[33].vz = -34;
	simu_track_table[33].hdg = 214;

	simu_track_table[34].lat = 465184978;
	simu_track_table[34].lon = 65661873;
	simu_track_table[34].alt = 407;
	simu_track_table[34].relative_alt = -7;
	simu_track_table[34].vx = 223;
	simu_track_table[34].vy = 150;
	simu_track_table[34].vz = -36;
	simu_track_table[34].hdg = 214;

	simu_track_table[35].lat = 465185030;
	simu_track_table[35].lon = 65661925;
	simu_track_table[35].alt = 407;
	simu_track_table[35].relative_alt = -7;
	simu_track_table[35].vx = 210;
	simu_track_table[35].vy = 141;
	simu_track_table[35].vz = -37;
	simu_track_table[35].hdg = 214;

	simu_track_table[36].lat = 465185079;
	simu_track_table[36].lon = 65661972;
	simu_track_table[36].alt = 407;
	simu_track_table[36].relative_alt = -7;
	simu_track_table[36].vx = 194;
	simu_track_table[36].vy = 130;
	simu_track_table[36].vz = -38;
	simu_track_table[36].hdg = 214;

	simu_track_table[37].lat = 465185124;
	simu_track_table[37].lon = 65662016;
	simu_track_table[37].alt = 407;
	simu_track_table[37].relative_alt = -7;
	simu_track_table[37].vx = 176;
	simu_track_table[37].vy = 119;
	simu_track_table[37].vz = -38;
	simu_track_table[37].hdg = 214;

	simu_track_table[38].lat = 465185164;
	simu_track_table[38].lon = 65662055;
	simu_track_table[38].alt = 408;
	simu_track_table[38].relative_alt = -8;
	simu_track_table[38].vx = 160;
	simu_track_table[38].vy = 108;
	simu_track_table[38].vz = -28;
	simu_track_table[38].hdg = 214;

	simu_track_table[39].lat = 465185201;
	simu_track_table[39].lon = 65662091;
	simu_track_table[39].alt = 408;
	simu_track_table[39].relative_alt = -8;
	simu_track_table[39].vx = 135;
	simu_track_table[39].vy = 94;
	simu_track_table[39].vz = 10;
	simu_track_table[39].hdg = 215;

	simu_track_table[40].lat = 465185228;
	simu_track_table[40].lon = 65662120;
	simu_track_table[40].alt = 407;
	simu_track_table[40].relative_alt = -7;
	simu_track_table[40].vx = 82;
	simu_track_table[40].vy = 68;
	simu_track_table[40].vz = 26;
	simu_track_table[40].hdg = 220;

	simu_track_table[41].lat = 465185240;
	simu_track_table[41].lon = 65662139;
	simu_track_table[41].alt = 407;
	simu_track_table[41].relative_alt = -7;
	simu_track_table[41].vx = 10;
	simu_track_table[41].vy = 35;
	simu_track_table[41].vz = 37;
	simu_track_table[41].hdg = 253;

	simu_track_table[42].lat = 465185236;
	simu_track_table[42].lon = 65662147;
	simu_track_table[42].alt = 407;
	simu_track_table[42].relative_alt = -7;
	simu_track_table[42].vx = -54;
	simu_track_table[42].vy = 9;
	simu_track_table[42].vz = 42;
	simu_track_table[42].hdg = 351;

	simu_track_table[43].lat = 465185219;
	simu_track_table[43].lon = 65662150;
	simu_track_table[43].alt = 407;
	simu_track_table[43].relative_alt = -7;
	simu_track_table[43].vx = -103;
	simu_track_table[43].vy = 4;
	simu_track_table[43].vz = 39;
	simu_track_table[43].hdg = 358;

	simu_track_table[44].lat = 465185191;
	simu_track_table[44].lon = 65662155;
	simu_track_table[44].alt = 407;
	simu_track_table[44].relative_alt = -7;
	simu_track_table[44].vx = -143;
	simu_track_table[44].vy = 30;
	simu_track_table[44].vz = 33;
	simu_track_table[44].hdg = 348;

	simu_track_table[45].lat = 465185152;
	simu_track_table[45].lon = 65662173;
	simu_track_table[45].alt = 407;
	simu_track_table[45].relative_alt = -7;
	simu_track_table[45].vx = -187;
	simu_track_table[45].vy = 83;
	simu_track_table[45].vz = 36;
	simu_track_table[45].hdg = 336;

	simu_track_table[46].lat = 465185104;
	simu_track_table[46].lon = 65662210;
	simu_track_table[46].alt = 407;
	simu_track_table[46].relative_alt = -7;
	simu_track_table[46].vx = -228;
	simu_track_table[46].vy = 142;
	simu_track_table[46].vz = 39;
	simu_track_table[46].hdg = 328;

	simu_track_table[47].lat = 465185045;
	simu_track_table[47].lon = 65662266;
	simu_track_table[47].alt = 407;
	simu_track_table[47].relative_alt = -7;
	simu_track_table[47].vx = -255;
	simu_track_table[47].vy = 188;
	simu_track_table[47].vz = 35;
	simu_track_table[47].hdg = 324;

	simu_track_table[48].lat = 465184985;
	simu_track_table[48].lon = 65662332;
	simu_track_table[48].alt = 407;
	simu_track_table[48].relative_alt = -7;
	simu_track_table[48].vx = -261;
	simu_track_table[48].vy = 209;
	simu_track_table[48].vz = 26;
	simu_track_table[48].hdg = 321;

	simu_track_table[49].lat = 465184922;
	simu_track_table[49].lon = 65662404;
	simu_track_table[49].alt = 407;
	simu_track_table[49].relative_alt = -7;
	simu_track_table[49].vx = -254;
	simu_track_table[49].vy = 213;
	simu_track_table[49].vz = 17;
	simu_track_table[49].hdg = 320;

	simu_track_table[50].lat = 465184862;
	simu_track_table[50].lon = 65662476;
	simu_track_table[50].alt = 407;
	simu_track_table[50].relative_alt = -7;
	simu_track_table[50].vx = -243;
	simu_track_table[50].vy = 207;
	simu_track_table[50].vz = 11;
	simu_track_table[50].hdg = 320;

	simu_track_table[51].lat = 465184804;
	simu_track_table[51].lon = 65662545;
	simu_track_table[51].alt = 407;
	simu_track_table[51].relative_alt = -7;
	simu_track_table[51].vx = -234;
	simu_track_table[51].vy = 200;
	simu_track_table[51].vz = 7;
	simu_track_table[51].hdg = 319;

	simu_track_table[52].lat = 465184748;
	simu_track_table[52].lon = 65662612;
	simu_track_table[52].alt = 407;
	simu_track_table[52].relative_alt = -7;
	simu_track_table[52].vx = -230;
	simu_track_table[52].vy = 196;
	simu_track_table[52].vz = 6;
	simu_track_table[52].hdg = 319;

	simu_track_table[53].lat = 465184691;
	simu_track_table[53].lon = 65662681;
	simu_track_table[53].alt = 407;
	simu_track_table[53].relative_alt = -7;
	simu_track_table[53].vx = -231;
	simu_track_table[53].vy = 197;
	simu_track_table[53].vz = 6;
	simu_track_table[53].hdg = 320;

	simu_track_table[54].lat = 465184636;
	simu_track_table[54].lon = 65662747;
	simu_track_table[54].alt = 407;
	simu_track_table[54].relative_alt = -7;
	simu_track_table[54].vx = -233;
	simu_track_table[54].vy = 199;
	simu_track_table[54].vz = 6;
	simu_track_table[54].hdg = 319;

	simu_track_table[55].lat = 465184578;
	simu_track_table[55].lon = 65662817;
	simu_track_table[55].alt = 407;
	simu_track_table[55].relative_alt = -7;
	simu_track_table[55].vx = -235;
	simu_track_table[55].vy = 202;
	simu_track_table[55].vz = 4;
	simu_track_table[55].hdg = 319;

	simu_track_table[56].lat = 465184522;
	simu_track_table[56].lon = 65662884;
	simu_track_table[56].alt = 407;
	simu_track_table[56].relative_alt = -7;
	simu_track_table[56].vx = -234;
	simu_track_table[56].vy = 202;
	simu_track_table[56].vz = 2;
	simu_track_table[56].hdg = 319;

	simu_track_table[57].lat = 465184464;
	simu_track_table[57].lon = 65662954;
	simu_track_table[57].alt = 407;
	simu_track_table[57].relative_alt = -7;
	simu_track_table[57].vx = -233;
	simu_track_table[57].vy = 202;
	simu_track_table[57].vz = -0;
	simu_track_table[57].hdg = 319;

	simu_track_table[58].lat = 465184408;
	simu_track_table[58].lon = 65663022;
	simu_track_table[58].alt = 407;
	simu_track_table[58].relative_alt = -7;
	simu_track_table[58].vx = -230;
	simu_track_table[58].vy = 199;
	simu_track_table[58].vz = -2;
	simu_track_table[58].hdg = 319;

	simu_track_table[59].lat = 465184353;
	simu_track_table[59].lon = 65663089;
	simu_track_table[59].alt = 407;
	simu_track_table[59].relative_alt = -7;
	simu_track_table[59].vx = -227;
	simu_track_table[59].vy = 197;
	simu_track_table[59].vz = -4;
	simu_track_table[59].hdg = 319;

	simu_track_table[60].lat = 465184298;
	simu_track_table[60].lon = 65663155;
	simu_track_table[60].alt = 407;
	simu_track_table[60].relative_alt = -7;
	simu_track_table[60].vx = -225;
	simu_track_table[60].vy = 195;
	simu_track_table[60].vz = -6;
	simu_track_table[60].hdg = 319;

	simu_track_table[61].lat = 465184243;
	simu_track_table[61].lon = 65663221;
	simu_track_table[61].alt = 407;
	simu_track_table[61].relative_alt = -7;
	simu_track_table[61].vx = -224;
	simu_track_table[61].vy = 194;
	simu_track_table[61].vz = -8;
	simu_track_table[61].hdg = 319;

	simu_track_table[62].lat = 465184190;
	simu_track_table[62].lon = 65663286;
	simu_track_table[62].alt = 407;
	simu_track_table[62].relative_alt = -7;
	simu_track_table[62].vx = -223;
	simu_track_table[62].vy = 193;
	simu_track_table[62].vz = -10;
	simu_track_table[62].hdg = 319;

	simu_track_table[63].lat = 465184135;
	simu_track_table[63].lon = 65663351;
	simu_track_table[63].alt = 407;
	simu_track_table[63].relative_alt = -7;
	simu_track_table[63].vx = -223;
	simu_track_table[63].vy = 193;
	simu_track_table[63].vz = -12;
	simu_track_table[63].hdg = 319;

	simu_track_table[64].lat = 465184082;
	simu_track_table[64].lon = 65663416;
	simu_track_table[64].alt = 407;
	simu_track_table[64].relative_alt = -7;
	simu_track_table[64].vx = -222;
	simu_track_table[64].vy = 192;
	simu_track_table[64].vz = -15;
	simu_track_table[64].hdg = 319;

	simu_track_table[65].lat = 465184028;
	simu_track_table[65].lon = 65663480;
	simu_track_table[65].alt = 407;
	simu_track_table[65].relative_alt = -7;
	simu_track_table[65].vx = -220;
	simu_track_table[65].vy = 191;
	simu_track_table[65].vz = -17;
	simu_track_table[65].hdg = 319;

	simu_track_table[66].lat = 465183975;
	simu_track_table[66].lon = 65663544;
	simu_track_table[66].alt = 407;
	simu_track_table[66].relative_alt = -7;
	simu_track_table[66].vx = -219;
	simu_track_table[66].vy = 190;
	simu_track_table[66].vz = -20;
	simu_track_table[66].hdg = 319;

	simu_track_table[67].lat = 465183923;
	simu_track_table[67].lon = 65663608;
	simu_track_table[67].alt = 407;
	simu_track_table[67].relative_alt = -7;
	simu_track_table[67].vx = -218;
	simu_track_table[67].vy = 190;
	simu_track_table[67].vz = -23;
	simu_track_table[67].hdg = 319;

	simu_track_table[68].lat = 465183871;
	simu_track_table[68].lon = 65663670;
	simu_track_table[68].alt = 407;
	simu_track_table[68].relative_alt = -7;
	simu_track_table[68].vx = -217;
	simu_track_table[68].vy = 189;
	simu_track_table[68].vz = -26;
	simu_track_table[68].hdg = 319;

	simu_track_table[69].lat = 465183818;
	simu_track_table[69].lon = 65663734;
	simu_track_table[69].alt = 407;
	simu_track_table[69].relative_alt = -7;
	simu_track_table[69].vx = -215;
	simu_track_table[69].vy = 188;
	simu_track_table[69].vz = -30;
	simu_track_table[69].hdg = 319;

	simu_track_table[70].lat = 465183768;
	simu_track_table[70].lon = 65663796;
	simu_track_table[70].alt = 407;
	simu_track_table[70].relative_alt = -7;
	simu_track_table[70].vx = -214;
	simu_track_table[70].vy = 188;
	simu_track_table[70].vz = -33;
	simu_track_table[70].hdg = 319;

	simu_track_table[71].lat = 465183716;
	simu_track_table[71].lon = 65663859;
	simu_track_table[71].alt = 407;
	simu_track_table[71].relative_alt = -7;
	simu_track_table[71].vx = -210;
	simu_track_table[71].vy = 185;
	simu_track_table[71].vz = -34;
	simu_track_table[71].hdg = 319;

	simu_track_table[72].lat = 465183666;
	simu_track_table[72].lon = 65663919;
	simu_track_table[72].alt = 407;
	simu_track_table[72].relative_alt = -7;
	simu_track_table[72].vx = -205;
	simu_track_table[72].vy = 181;
	simu_track_table[72].vz = -35;
	simu_track_table[72].hdg = 318;

	simu_track_table[73].lat = 465183618;
	simu_track_table[73].lon = 65663978;
	simu_track_table[73].alt = 407;
	simu_track_table[73].relative_alt = -7;
	simu_track_table[73].vx = -195;
	simu_track_table[73].vy = 173;
	simu_track_table[73].vz = -36;
	simu_track_table[73].hdg = 318;

	simu_track_table[74].lat = 465183574;
	simu_track_table[74].lon = 65664033;
	simu_track_table[74].alt = 407;
	simu_track_table[74].relative_alt = -7;
	simu_track_table[74].vx = -181;
	simu_track_table[74].vy = 163;
	simu_track_table[74].vz = -37;
	simu_track_table[74].hdg = 318;

	simu_track_table[75].lat = 465183531;
	simu_track_table[75].lon = 65664085;
	simu_track_table[75].alt = 408;
	simu_track_table[75].relative_alt = -8;
	simu_track_table[75].vx = -165;
	simu_track_table[75].vy = 149;
	simu_track_table[75].vz = -38;
	simu_track_table[75].hdg = 318;

	simu_track_table[76].lat = 465183494;
	simu_track_table[76].lon = 65664131;
	simu_track_table[76].alt = 408;
	simu_track_table[76].relative_alt = -8;
	simu_track_table[76].vx = -150;
	simu_track_table[76].vy = 136;
	simu_track_table[76].vz = -28;
	simu_track_table[76].hdg = 318;

	simu_track_table[77].lat = 465183460;
	simu_track_table[77].lon = 65664174;
	simu_track_table[77].alt = 408;
	simu_track_table[77].relative_alt = -8;
	simu_track_table[77].vx = -126;
	simu_track_table[77].vy = 119;
	simu_track_table[77].vz = 10;
	simu_track_table[77].hdg = 316;

	simu_track_table[78].lat = 465183436;
	simu_track_table[78].lon = 65664206;
	simu_track_table[78].alt = 408;
	simu_track_table[78].relative_alt = -8;
	simu_track_table[78].vx = -71;
	simu_track_table[78].vy = 84;
	simu_track_table[78].vz = 26;
	simu_track_table[78].hdg = 310;

	simu_track_table[79].lat = 465183427;
	simu_track_table[79].lon = 65664225;
	simu_track_table[79].alt = 407;
	simu_track_table[79].relative_alt = -7;
	simu_track_table[79].vx = 4;
	simu_track_table[79].vy = 36;
	simu_track_table[79].vz = 39;
	simu_track_table[79].hdg = 263;

	simu_track_table[80].lat = 465183435;
	simu_track_table[80].lon = 65664228;
	simu_track_table[80].alt = 407;
	simu_track_table[80].relative_alt = -7;
	simu_track_table[80].vx = 77;
	simu_track_table[80].vy = -4;
	simu_track_table[80].vz = 47;
	simu_track_table[80].hdg = 177;

	simu_track_table[81].lat = 465183460;
	simu_track_table[81].lon = 65664221;
	simu_track_table[81].alt = 407;
	simu_track_table[81].relative_alt = -7;
	simu_track_table[81].vx = 134;
	simu_track_table[81].vy = -21;
	simu_track_table[81].vz = 46;
	simu_track_table[81].hdg = 171;

	simu_track_table[82].lat = 465183496;
	simu_track_table[82].lon = 65664212;
	simu_track_table[82].alt = 407;
	simu_track_table[82].relative_alt = -7;
	simu_track_table[82].vx = 185;
	simu_track_table[82].vy = -7;
	simu_track_table[82].vz = 42;
	simu_track_table[82].hdg = 178;

	simu_track_table[83].lat = 465183547;
	simu_track_table[83].lon = 65664212;
	simu_track_table[83].alt = 407;
	simu_track_table[83].relative_alt = -7;
	simu_track_table[83].vx = 239;
	simu_track_table[83].vy = 29;
	simu_track_table[83].vz = 45;
	simu_track_table[83].hdg = 187;

	simu_track_table[84].lat = 465183608;
	simu_track_table[84].lon = 65664223;
	simu_track_table[84].alt = 407;
	simu_track_table[84].relative_alt = -7;
	simu_track_table[84].vx = 284;
	simu_track_table[84].vy = 67;
	simu_track_table[84].vz = 45;
	simu_track_table[84].hdg = 193;

	simu_track_table[85].lat = 465183679;
	simu_track_table[85].lon = 65664246;
	simu_track_table[85].alt = 407;
	simu_track_table[85].relative_alt = -7;
	simu_track_table[85].vx = 309;
	simu_track_table[85].vy = 95;
	simu_track_table[85].vz = 40;
	simu_track_table[85].hdg = 197;

	simu_track_table[86].lat = 465183754;
	simu_track_table[86].lon = 65664276;
	simu_track_table[86].alt = 407;
	simu_track_table[86].relative_alt = -7;
	simu_track_table[86].vx = 313;
	simu_track_table[86].vy = 109;
	simu_track_table[86].vz = 32;
	simu_track_table[86].hdg = 199;

	simu_track_table[87].lat = 465183825;
	simu_track_table[87].lon = 65664308;
	simu_track_table[87].alt = 407;
	simu_track_table[87].relative_alt = -7;
	simu_track_table[87].vx = 301;
	simu_track_table[87].vy = 111;
	simu_track_table[87].vz = 26;
	simu_track_table[87].hdg = 200;

	simu_track_table[88].lat = 465183895;
	simu_track_table[88].lon = 65664340;
	simu_track_table[88].alt = 407;
	simu_track_table[88].relative_alt = -7;
	simu_track_table[88].vx = 289;
	simu_track_table[88].vy = 108;
	simu_track_table[88].vz = 22;
	simu_track_table[88].hdg = 200;

	simu_track_table[89].lat = 465183967;
	simu_track_table[89].lon = 65664373;
	simu_track_table[89].alt = 407;
	simu_track_table[89].relative_alt = -7;
	simu_track_table[89].vx = 283;
	simu_track_table[89].vy = 105;
	simu_track_table[89].vz = 20;
	simu_track_table[89].hdg = 200;

	simu_track_table[90].lat = 465184035;
	simu_track_table[90].lon = 65664404;
	simu_track_table[90].alt = 406;
	simu_track_table[90].relative_alt = -6;
	simu_track_table[90].vx = 283;
	simu_track_table[90].vy = 104;
	simu_track_table[90].vz = 21;
	simu_track_table[90].hdg = 200;

	simu_track_table[91].lat = 465184107;
	simu_track_table[91].lon = 65664437;
	simu_track_table[91].alt = 406;
	simu_track_table[91].relative_alt = -6;
	simu_track_table[91].vx = 288;
	simu_track_table[91].vy = 106;
	simu_track_table[91].vz = 21;
	simu_track_table[91].hdg = 200;

	simu_track_table[92].lat = 465184177;
	simu_track_table[92].lon = 65664470;
	simu_track_table[92].alt = 406;
	simu_track_table[92].relative_alt = -6;
	simu_track_table[92].vx = 293;
	simu_track_table[92].vy = 109;
	simu_track_table[92].vz = 21;
	simu_track_table[92].hdg = 200;

	simu_track_table[93].lat = 465184251;
	simu_track_table[93].lon = 65664505;
	simu_track_table[93].alt = 406;
	simu_track_table[93].relative_alt = -6;
	simu_track_table[93].vx = 295;
	simu_track_table[93].vy = 112;
	simu_track_table[93].vz = 20;
	simu_track_table[93].hdg = 201;

	simu_track_table[94].lat = 465184324;
	simu_track_table[94].lon = 65664540;
	simu_track_table[94].alt = 406;
	simu_track_table[94].relative_alt = -6;
	simu_track_table[94].vx = 294;
	simu_track_table[94].vy = 113;
	simu_track_table[94].vz = 18;
	simu_track_table[94].hdg = 201;

	simu_track_table[95].lat = 465184396;
	simu_track_table[95].lon = 65664576;
	simu_track_table[95].alt = 406;
	simu_track_table[95].relative_alt = -6;
	simu_track_table[95].vx = 290;
	simu_track_table[95].vy = 113;
	simu_track_table[95].vz = 17;
	simu_track_table[95].hdg = 201;

	simu_track_table[96].lat = 465184467;
	simu_track_table[96].lon = 65664612;
	simu_track_table[96].alt = 406;
	simu_track_table[96].relative_alt = -6;
	simu_track_table[96].vx = 285;
	simu_track_table[96].vy = 111;
	simu_track_table[96].vz = 15;
	simu_track_table[96].hdg = 201;

	simu_track_table[97].lat = 465184538;
	simu_track_table[97].lon = 65664648;
	simu_track_table[97].alt = 406;
	simu_track_table[97].relative_alt = -6;
	simu_track_table[97].vx = 282;
	simu_track_table[97].vy = 110;
	simu_track_table[97].vz = 15;
	simu_track_table[97].hdg = 201;

	simu_track_table[98].lat = 465184606;
	simu_track_table[98].lon = 65664682;
	simu_track_table[98].alt = 406;
	simu_track_table[98].relative_alt = -6;
	simu_track_table[98].vx = 281;
	simu_track_table[98].vy = 110;
	simu_track_table[98].vz = 14;
	simu_track_table[98].hdg = 201;

	simu_track_table[99].lat = 465184676;
	simu_track_table[99].lon = 65664718;
	simu_track_table[99].alt = 406;
	simu_track_table[99].relative_alt = -6;
	simu_track_table[99].vx = 281;
	simu_track_table[99].vy = 109;
	simu_track_table[99].vz = 13;
	simu_track_table[99].hdg = 201;

	simu_track_table[100].lat = 465184745;
	simu_track_table[100].lon = 65664752;
	simu_track_table[100].alt = 406;
	simu_track_table[100].relative_alt = -6;
	simu_track_table[100].vx = 281;
	simu_track_table[100].vy = 109;
	simu_track_table[100].vz = 13;
	simu_track_table[100].hdg = 201;

	simu_track_table[101].lat = 465184814;
	simu_track_table[101].lon = 65664787;
	simu_track_table[101].alt = 406;
	simu_track_table[101].relative_alt = -6;
	simu_track_table[101].vx = 279;
	simu_track_table[101].vy = 109;
	simu_track_table[101].vz = 12;
	simu_track_table[101].hdg = 201;

	simu_track_table[102].lat = 465184882;
	simu_track_table[102].lon = 65664822;
	simu_track_table[102].alt = 406;
	simu_track_table[102].relative_alt = -6;
	simu_track_table[102].vx = 279;
	simu_track_table[102].vy = 109;
	simu_track_table[102].vz = 11;
	simu_track_table[102].hdg = 201;

	simu_track_table[103].lat = 465184950;
	simu_track_table[103].lon = 65664857;
	simu_track_table[103].alt = 406;
	simu_track_table[103].relative_alt = -6;
	simu_track_table[103].vx = 278;
	simu_track_table[103].vy = 109;
	simu_track_table[103].vz = 10;
	simu_track_table[103].hdg = 201;

	simu_track_table[104].lat = 465185016;
	simu_track_table[104].lon = 65664891;
	simu_track_table[104].alt = 406;
	simu_track_table[104].relative_alt = -6;
	simu_track_table[104].vx = 278;
	simu_track_table[104].vy = 109;
	simu_track_table[104].vz = 9;
	simu_track_table[104].hdg = 201;

	simu_track_table[105].lat = 465185085;
	simu_track_table[105].lon = 65664927;
	simu_track_table[105].alt = 406;
	simu_track_table[105].relative_alt = -6;
	simu_track_table[105].vx = 277;
	simu_track_table[105].vy = 109;
	simu_track_table[105].vz = 8;
	simu_track_table[105].hdg = 201;

	simu_track_table[106].lat = 465185150;
	simu_track_table[106].lon = 65664960;
	simu_track_table[106].alt = 406;
	simu_track_table[106].relative_alt = -6;
	simu_track_table[106].vx = 276;
	simu_track_table[106].vy = 108;
	simu_track_table[106].vz = 7;
	simu_track_table[106].hdg = 201;

	simu_track_table[107].lat = 465185217;
	simu_track_table[107].lon = 65664995;
	simu_track_table[107].alt = 406;
	simu_track_table[107].relative_alt = -6;
	simu_track_table[107].vx = 275;
	simu_track_table[107].vy = 108;
	simu_track_table[107].vz = 6;
	simu_track_table[107].hdg = 201;

	simu_track_table[108].lat = 465185283;
	simu_track_table[108].lon = 65665029;
	simu_track_table[108].alt = 406;
	simu_track_table[108].relative_alt = -6;
	simu_track_table[108].vx = 274;
	simu_track_table[108].vy = 108;
	simu_track_table[108].vz = 4;
	simu_track_table[108].hdg = 201;

	simu_track_table[109].lat = 465185348;
	simu_track_table[109].lon = 65665063;
	simu_track_table[109].alt = 406;
	simu_track_table[109].relative_alt = -6;
	simu_track_table[109].vx = 274;
	simu_track_table[109].vy = 108;
	simu_track_table[109].vz = 3;
	simu_track_table[109].hdg = 202;

	simu_track_table[110].lat = 465185413;
	simu_track_table[110].lon = 65665097;
	simu_track_table[110].alt = 406;
	simu_track_table[110].relative_alt = -6;
	simu_track_table[110].vx = 272;
	simu_track_table[110].vy = 108;
	simu_track_table[110].vz = 2;
	simu_track_table[110].hdg = 202;

	simu_track_table[111].lat = 465185478;
	simu_track_table[111].lon = 65665131;
	simu_track_table[111].alt = 406;
	simu_track_table[111].relative_alt = -6;
	simu_track_table[111].vx = 272;
	simu_track_table[111].vy = 108;
	simu_track_table[111].vz = 1;
	simu_track_table[111].hdg = 202;

	simu_track_table[112].lat = 465185542;
	simu_track_table[112].lon = 65665165;
	simu_track_table[112].alt = 406;
	simu_track_table[112].relative_alt = -6;
	simu_track_table[112].vx = 272;
	simu_track_table[112].vy = 108;
	simu_track_table[112].vz = -0;
	simu_track_table[112].hdg = 202;

	simu_track_table[113].lat = 465185607;
	simu_track_table[113].lon = 65665199;
	simu_track_table[113].alt = 406;
	simu_track_table[113].relative_alt = -6;
	simu_track_table[113].vx = 272;
	simu_track_table[113].vy = 108;
	simu_track_table[113].vz = -2;
	simu_track_table[113].hdg = 202;

	simu_track_table[114].lat = 465185671;
	simu_track_table[114].lon = 65665233;
	simu_track_table[114].alt = 406;
	simu_track_table[114].relative_alt = -6;
	simu_track_table[114].vx = 271;
	simu_track_table[114].vy = 108;
	simu_track_table[114].vz = -3;
	simu_track_table[114].hdg = 202;

	simu_track_table[115].lat = 465185734;
	simu_track_table[115].lon = 65665267;
	simu_track_table[115].alt = 406;
	simu_track_table[115].relative_alt = -6;
	simu_track_table[115].vx = 269;
	simu_track_table[115].vy = 108;
	simu_track_table[115].vz = -5;
	simu_track_table[115].hdg = 202;

	simu_track_table[116].lat = 465185797;
	simu_track_table[116].lon = 65665301;
	simu_track_table[116].alt = 406;
	simu_track_table[116].relative_alt = -6;
	simu_track_table[116].vx = 269;
	simu_track_table[116].vy = 108;
	simu_track_table[116].vz = -6;
	simu_track_table[116].hdg = 202;

	simu_track_table[117].lat = 465185860;
	simu_track_table[117].lon = 65665334;
	simu_track_table[117].alt = 406;
	simu_track_table[117].relative_alt = -6;
	simu_track_table[117].vx = 269;
	simu_track_table[117].vy = 108;
	simu_track_table[117].vz = -8;
	simu_track_table[117].hdg = 202;

	simu_track_table[118].lat = 465185922;
	simu_track_table[118].lon = 65665368;
	simu_track_table[118].alt = 406;
	simu_track_table[118].relative_alt = -6;
	simu_track_table[118].vx = 269;
	simu_track_table[118].vy = 108;
	simu_track_table[118].vz = -9;
	simu_track_table[118].hdg = 202;

	simu_track_table[119].lat = 465185986;
	simu_track_table[119].lon = 65665402;
	simu_track_table[119].alt = 406;
	simu_track_table[119].relative_alt = -6;
	simu_track_table[119].vx = 268;
	simu_track_table[119].vy = 108;
	simu_track_table[119].vz = -11;
	simu_track_table[119].hdg = 202;

	simu_track_table[120].lat = 465186047;
	simu_track_table[120].lon = 65665435;
	simu_track_table[120].alt = 406;
	simu_track_table[120].relative_alt = -6;
	simu_track_table[120].vx = 267;
	simu_track_table[120].vy = 108;
	simu_track_table[120].vz = -13;
	simu_track_table[120].hdg = 202;

	simu_track_table[121].lat = 465186109;
	simu_track_table[121].lon = 65665469;
	simu_track_table[121].alt = 406;
	simu_track_table[121].relative_alt = -6;
	simu_track_table[121].vx = 267;
	simu_track_table[121].vy = 108;
	simu_track_table[121].vz = -15;
	simu_track_table[121].hdg = 202;

	simu_track_table[122].lat = 465186170;
	simu_track_table[122].lon = 65665503;
	simu_track_table[122].alt = 406;
	simu_track_table[122].relative_alt = -6;
	simu_track_table[122].vx = 266;
	simu_track_table[122].vy = 108;
	simu_track_table[122].vz = -17;
	simu_track_table[122].hdg = 202;

	simu_track_table[123].lat = 465186231;
	simu_track_table[123].lon = 65665537;
	simu_track_table[123].alt = 406;
	simu_track_table[123].relative_alt = -6;
	simu_track_table[123].vx = 264;
	simu_track_table[123].vy = 108;
	simu_track_table[123].vz = -19;
	simu_track_table[123].hdg = 202;

	simu_track_table[124].lat = 465186290;
	simu_track_table[124].lon = 65665570;
	simu_track_table[124].alt = 406;
	simu_track_table[124].relative_alt = -6;
	simu_track_table[124].vx = 263;
	simu_track_table[124].vy = 108;
	simu_track_table[124].vz = -21;
	simu_track_table[124].hdg = 202;

	simu_track_table[125].lat = 465186351;
	simu_track_table[125].lon = 65665604;
	simu_track_table[125].alt = 406;
	simu_track_table[125].relative_alt = -6;
	simu_track_table[125].vx = 262;
	simu_track_table[125].vy = 108;
	simu_track_table[125].vz = -24;
	simu_track_table[125].hdg = 202;

	simu_track_table[126].lat = 465186410;
	simu_track_table[126].lon = 65665637;
	simu_track_table[126].alt = 406;
	simu_track_table[126].relative_alt = -6;
	simu_track_table[126].vx = 261;
	simu_track_table[126].vy = 108;
	simu_track_table[126].vz = -26;
	simu_track_table[126].hdg = 202;

	simu_track_table[127].lat = 465186470;
	simu_track_table[127].lon = 65665670;
	simu_track_table[127].alt = 406;
	simu_track_table[127].relative_alt = -6;
	simu_track_table[127].vx = 261;
	simu_track_table[127].vy = 108;
	simu_track_table[127].vz = -29;
	simu_track_table[127].hdg = 203;

	simu_track_table[128].lat = 465186528;
	simu_track_table[128].lon = 65665704;
	simu_track_table[128].alt = 406;
	simu_track_table[128].relative_alt = -6;
	simu_track_table[128].vx = 260;
	simu_track_table[128].vy = 108;
	simu_track_table[128].vz = -32;
	simu_track_table[128].hdg = 203;

	simu_track_table[129].lat = 465186587;
	simu_track_table[129].lon = 65665737;
	simu_track_table[129].alt = 407;
	simu_track_table[129].relative_alt = -7;
	simu_track_table[129].vx = 257;
	simu_track_table[129].vy = 107;
	simu_track_table[129].vz = -34;
	simu_track_table[129].hdg = 203;

	simu_track_table[130].lat = 465186644;
	simu_track_table[130].lon = 65665770;
	simu_track_table[130].alt = 407;
	simu_track_table[130].relative_alt = -7;
	simu_track_table[130].vx = 254;
	simu_track_table[130].vy = 106;
	simu_track_table[130].vz = -34;
	simu_track_table[130].hdg = 203;

	simu_track_table[131].lat = 465186701;
	simu_track_table[131].lon = 65665803;
	simu_track_table[131].alt = 407;
	simu_track_table[131].relative_alt = -7;
	simu_track_table[131].vx = 248;
	simu_track_table[131].vy = 104;
	simu_track_table[131].vz = -35;
	simu_track_table[131].hdg = 203;

	simu_track_table[132].lat = 465186754;
	simu_track_table[132].lon = 65665834;
	simu_track_table[132].alt = 407;
	simu_track_table[132].relative_alt = -7;
	simu_track_table[132].vx = 238;
	simu_track_table[132].vy = 101;
	simu_track_table[132].vz = -36;
	simu_track_table[132].hdg = 203;

	simu_track_table[133].lat = 465186807;
	simu_track_table[133].lon = 65665864;
	simu_track_table[133].alt = 407;
	simu_track_table[133].relative_alt = -7;
	simu_track_table[133].vx = 225;
	simu_track_table[133].vy = 96;
	simu_track_table[133].vz = -37;
	simu_track_table[133].hdg = 203;

	simu_track_table[134].lat = 465186854;
	simu_track_table[134].lon = 65665892;
	simu_track_table[134].alt = 407;
	simu_track_table[134].relative_alt = -7;
	simu_track_table[134].vx = 211;
	simu_track_table[134].vy = 90;
	simu_track_table[134].vz = -37;
	simu_track_table[134].hdg = 203;

	simu_track_table[135].lat = 465186900;
	simu_track_table[135].lon = 65665918;
	simu_track_table[135].alt = 407;
	simu_track_table[135].relative_alt = -7;
	simu_track_table[135].vx = 196;
	simu_track_table[135].vy = 84;
	simu_track_table[135].vz = -37;
	simu_track_table[135].hdg = 203;

	simu_track_table[136].lat = 465186940;
	simu_track_table[136].lon = 65665942;
	simu_track_table[136].alt = 407;
	simu_track_table[136].relative_alt = -7;
	simu_track_table[136].vx = 183;
	simu_track_table[136].vy = 79;
	simu_track_table[136].vz = -37;
	simu_track_table[136].hdg = 203;

	simu_track_table[137].lat = 465186979;
	simu_track_table[137].lon = 65665965;
	simu_track_table[137].alt = 407;
	simu_track_table[137].relative_alt = -7;
	simu_track_table[137].vx = 170;
	simu_track_table[137].vy = 74;
	simu_track_table[137].vz = -36;
	simu_track_table[137].hdg = 204;

	simu_track_table[138].lat = 465187013;
	simu_track_table[138].lon = 65665986;
	simu_track_table[138].alt = 407;
	simu_track_table[138].relative_alt = -7;
	simu_track_table[138].vx = 159;
	simu_track_table[138].vy = 70;
	simu_track_table[138].vz = -36;
	simu_track_table[138].hdg = 204;

	simu_track_table[139].lat = 465187046;
	simu_track_table[139].lon = 65666006;
	simu_track_table[139].alt = 407;
	simu_track_table[139].relative_alt = -7;
	simu_track_table[139].vx = 148;
	simu_track_table[139].vy = 65;
	simu_track_table[139].vz = -20;
	simu_track_table[139].hdg = 204;

	simu_track_table[140].lat = 465187074;
	simu_track_table[140].lon = 65666022;
	simu_track_table[140].alt = 407;
	simu_track_table[140].relative_alt = -7;
	simu_track_table[140].vx = 128;
	simu_track_table[140].vy = 49;
	simu_track_table[140].vz = 14;
	simu_track_table[140].hdg = 201;

	simu_track_table[141].lat = 465187096;
	simu_track_table[141].lon = 65666028;
	simu_track_table[141].alt = 407;
	simu_track_table[141].relative_alt = -7;
	simu_track_table[141].vx = 80;
	simu_track_table[141].vy = 3;
	simu_track_table[141].vz = 30;
	simu_track_table[141].hdg = 182;

	simu_track_table[142].lat = 465187104;
	simu_track_table[142].lon = 65666016;
	simu_track_table[142].alt = 407;
	simu_track_table[142].relative_alt = -7;
	simu_track_table[142].vx = 19;
	simu_track_table[142].vy = -62;
	simu_track_table[142].vz = 44;
	simu_track_table[142].hdg = 107;

	simu_track_table[143].lat = 465187097;
	simu_track_table[143].lon = 65665979;
	simu_track_table[143].alt = 407;
	simu_track_table[143].relative_alt = -7;
	simu_track_table[143].vx = -35;
	simu_track_table[143].vy = -133;
	simu_track_table[143].vz = 50;
	simu_track_table[143].hdg = 75;

	simu_track_table[144].lat = 465187081;
	simu_track_table[144].lon = 65665919;
	simu_track_table[144].alt = 407;
	simu_track_table[144].relative_alt = -7;
	simu_track_table[144].vx = -62;
	simu_track_table[144].vy = -202;
	simu_track_table[144].vz = 52;
	simu_track_table[144].hdg = 73;

	simu_track_table[145].lat = 465187062;
	simu_track_table[145].lon = 65665836;
	simu_track_table[145].alt = 407;
	simu_track_table[145].relative_alt = -7;
	simu_track_table[145].vx = -59;
	simu_track_table[145].vy = -268;
	simu_track_table[145].vz = 52;
	simu_track_table[145].hdg = 78;

	simu_track_table[146].lat = 465187045;
	simu_track_table[146].lon = 65665733;
	simu_track_table[146].alt = 407;
	simu_track_table[146].relative_alt = -7;
	simu_track_table[146].vx = -37;
	simu_track_table[146].vy = -314;
	simu_track_table[146].vz = 48;
	simu_track_table[146].hdg = 83;

	simu_track_table[147].lat = 465187033;
	simu_track_table[147].lon = 65665615;
	simu_track_table[147].alt = 407;
	simu_track_table[147].relative_alt = -7;
	simu_track_table[147].vx = -12;
	simu_track_table[147].vy = -336;
	simu_track_table[147].vz = 41;
	simu_track_table[147].hdg = 88;

	simu_track_table[148].lat = 465187026;
	simu_track_table[148].lon = 65665497;
	simu_track_table[148].alt = 406;
	simu_track_table[148].relative_alt = -6;
	simu_track_table[148].vx = 5;
	simu_track_table[148].vy = -335;
	simu_track_table[148].vz = 32;
	simu_track_table[148].hdg = 91;

	simu_track_table[149].lat = 465187021;
	simu_track_table[149].lon = 65665378;
	simu_track_table[149].alt = 406;
	simu_track_table[149].relative_alt = -6;
	simu_track_table[149].vx = 12;
	simu_track_table[149].vy = -322;
	simu_track_table[149].vz = 26;
	simu_track_table[149].hdg = 92;

	simu_track_table[150].lat = 465187017;
	simu_track_table[150].lon = 65665265;
	simu_track_table[150].alt = 406;
	simu_track_table[150].relative_alt = -6;
	simu_track_table[150].vx = 9;
	simu_track_table[150].vy = -308;
	simu_track_table[150].vz = 23;
	simu_track_table[150].hdg = 92;

	simu_track_table[151].lat = 465187012;
	simu_track_table[151].lon = 65665155;
	simu_track_table[151].alt = 406;
	simu_track_table[151].relative_alt = -6;
	simu_track_table[151].vx = 4;
	simu_track_table[151].vy = -299;
	simu_track_table[151].vz = 22;
	simu_track_table[151].hdg = 91;

	simu_track_table[152].lat = 465187006;
	simu_track_table[152].lon = 65665048;
	simu_track_table[152].alt = 406;
	simu_track_table[152].relative_alt = -6;
	simu_track_table[152].vx = -0;
	simu_track_table[152].vy = -298;
	simu_track_table[152].vz = 23;
	simu_track_table[152].hdg = 90;

	simu_track_table[153].lat = 465187000;
	simu_track_table[153].lon = 65664938;
	simu_track_table[153].alt = 406;
	simu_track_table[153].relative_alt = -6;
	simu_track_table[153].vx = -1;
	simu_track_table[153].vy = -302;
	simu_track_table[153].vz = 24;
	simu_track_table[153].hdg = 90;

	simu_track_table[154].lat = 465186994;
	simu_track_table[154].lon = 65664831;
	simu_track_table[154].alt = 406;
	simu_track_table[154].relative_alt = -6;
	simu_track_table[154].vx = 2;
	simu_track_table[154].vy = -306;
	simu_track_table[154].vz = 24;
	simu_track_table[154].hdg = 90;

	simu_track_table[155].lat = 465186988;
	simu_track_table[155].lon = 65664720;
	simu_track_table[155].alt = 406;
	simu_track_table[155].relative_alt = -6;
	simu_track_table[155].vx = 7;
	simu_track_table[155].vy = -308;
	simu_track_table[155].vz = 23;
	simu_track_table[155].hdg = 91;

	simu_track_table[156].lat = 465186984;
	simu_track_table[156].lon = 65664612;
	simu_track_table[156].alt = 406;
	simu_track_table[156].relative_alt = -6;
	simu_track_table[156].vx = 11;
	simu_track_table[156].vy = -308;
	simu_track_table[156].vz = 22;
	simu_track_table[156].hdg = 92;

	simu_track_table[157].lat = 465186981;
	simu_track_table[157].lon = 65664502;
	simu_track_table[157].alt = 406;
	simu_track_table[157].relative_alt = -6;
	simu_track_table[157].vx = 15;
	simu_track_table[157].vy = -304;
	simu_track_table[157].vz = 21;
	simu_track_table[157].hdg = 93;

	simu_track_table[158].lat = 465186979;
	simu_track_table[158].lon = 65664395;
	simu_track_table[158].alt = 406;
	simu_track_table[158].relative_alt = -6;
	simu_track_table[158].vx = 17;
	simu_track_table[158].vy = -301;
	simu_track_table[158].vz = 20;
	simu_track_table[158].hdg = 93;

	simu_track_table[159].lat = 465186977;
	simu_track_table[159].lon = 65664289;
	simu_track_table[159].alt = 406;
	simu_track_table[159].relative_alt = -6;
	simu_track_table[159].vx = 17;
	simu_track_table[159].vy = -299;
	simu_track_table[159].vz = 20;
	simu_track_table[159].hdg = 93;

	simu_track_table[160].lat = 465186975;
	simu_track_table[160].lon = 65664186;
	simu_track_table[160].alt = 406;
	simu_track_table[160].relative_alt = -6;
	simu_track_table[160].vx = 17;
	simu_track_table[160].vy = -298;
	simu_track_table[160].vz = 19;
	simu_track_table[160].hdg = 93;

	simu_track_table[161].lat = 465186973;
	simu_track_table[161].lon = 65664080;
	simu_track_table[161].alt = 406;
	simu_track_table[161].relative_alt = -6;
	simu_track_table[161].vx = 18;
	simu_track_table[161].vy = -298;
	simu_track_table[161].vz = 19;
	simu_track_table[161].hdg = 93;

	simu_track_table[162].lat = 465186972;
	simu_track_table[162].lon = 65663978;
	simu_track_table[162].alt = 406;
	simu_track_table[162].relative_alt = -6;
	simu_track_table[162].vx = 18;
	simu_track_table[162].vy = -297;
	simu_track_table[162].vz = 18;
	simu_track_table[162].hdg = 94;

	simu_track_table[163].lat = 465186971;
	simu_track_table[163].lon = 65663874;
	simu_track_table[163].alt = 406;
	simu_track_table[163].relative_alt = -6;
	simu_track_table[163].vx = 19;
	simu_track_table[163].vy = -298;
	simu_track_table[163].vz = 18;
	simu_track_table[163].hdg = 94;

	simu_track_table[164].lat = 465186970;
	simu_track_table[164].lon = 65663772;
	simu_track_table[164].alt = 406;
	simu_track_table[164].relative_alt = -6;
	simu_track_table[164].vx = 20;
	simu_track_table[164].vy = -297;
	simu_track_table[164].vz = 17;
	simu_track_table[164].hdg = 94;

	simu_track_table[165].lat = 465186969;
	simu_track_table[165].lon = 65663670;
	simu_track_table[165].alt = 406;
	simu_track_table[165].relative_alt = -6;
	simu_track_table[165].vx = 21;
	simu_track_table[165].vy = -295;
	simu_track_table[165].vz = 16;
	simu_track_table[165].hdg = 94;

	simu_track_table[166].lat = 465186969;
	simu_track_table[166].lon = 65663570;
	simu_track_table[166].alt = 406;
	simu_track_table[166].relative_alt = -6;
	simu_track_table[166].vx = 22;
	simu_track_table[166].vy = -294;
	simu_track_table[166].vz = 16;
	simu_track_table[166].hdg = 94;

	simu_track_table[167].lat = 465186969;
	simu_track_table[167].lon = 65663470;
	simu_track_table[167].alt = 405;
	simu_track_table[167].relative_alt = -5;
	simu_track_table[167].vx = 22;
	simu_track_table[167].vy = -294;
	simu_track_table[167].vz = 15;
	simu_track_table[167].hdg = 94;

	simu_track_table[168].lat = 465186969;
	simu_track_table[168].lon = 65663371;
	simu_track_table[168].alt = 405;
	simu_track_table[168].relative_alt = -5;
	simu_track_table[168].vx = 23;
	simu_track_table[168].vy = -294;
	simu_track_table[168].vz = 15;
	simu_track_table[168].hdg = 94;

	simu_track_table[169].lat = 465186969;
	simu_track_table[169].lon = 65663270;
	simu_track_table[169].alt = 405;
	simu_track_table[169].relative_alt = -5;
	simu_track_table[169].vx = 23;
	simu_track_table[169].vy = -294;
	simu_track_table[169].vz = 14;
	simu_track_table[169].hdg = 94;

	simu_track_table[170].lat = 465186969;
	simu_track_table[170].lon = 65663173;
	simu_track_table[170].alt = 405;
	simu_track_table[170].relative_alt = -5;
	simu_track_table[170].vx = 23;
	simu_track_table[170].vy = -294;
	simu_track_table[170].vz = 13;
	simu_track_table[170].hdg = 95;

	simu_track_table[171].lat = 465186970;
	simu_track_table[171].lon = 65663074;
	simu_track_table[171].alt = 405;
	simu_track_table[171].relative_alt = -5;
	simu_track_table[171].vx = 24;
	simu_track_table[171].vy = -294;
	simu_track_table[171].vz = 12;
	simu_track_table[171].hdg = 95;

	simu_track_table[172].lat = 465186971;
	simu_track_table[172].lon = 65662976;
	simu_track_table[172].alt = 405;
	simu_track_table[172].relative_alt = -5;
	simu_track_table[172].vx = 24;
	simu_track_table[172].vy = -293;
	simu_track_table[172].vz = 12;
	simu_track_table[172].hdg = 95;

	simu_track_table[173].lat = 465186972;
	simu_track_table[173].lon = 65662879;
	simu_track_table[173].alt = 405;
	simu_track_table[173].relative_alt = -5;
	simu_track_table[173].vx = 25;
	simu_track_table[173].vy = -292;
	simu_track_table[173].vz = 11;
	simu_track_table[173].hdg = 95;

	simu_track_table[174].lat = 465186973;
	simu_track_table[174].lon = 65662783;
	simu_track_table[174].alt = 405;
	simu_track_table[174].relative_alt = -5;
	simu_track_table[174].vx = 25;
	simu_track_table[174].vy = -291;
	simu_track_table[174].vz = 10;
	simu_track_table[174].hdg = 95;

	simu_track_table[175].lat = 465186974;
	simu_track_table[175].lon = 65662686;
	simu_track_table[175].alt = 405;
	simu_track_table[175].relative_alt = -5;
	simu_track_table[175].vx = 25;
	simu_track_table[175].vy = -291;
	simu_track_table[175].vz = 9;
	simu_track_table[175].hdg = 95;

	simu_track_table[176].lat = 465186975;
	simu_track_table[176].lon = 65662592;
	simu_track_table[176].alt = 405;
	simu_track_table[176].relative_alt = -5;
	simu_track_table[176].vx = 26;
	simu_track_table[176].vy = -291;
	simu_track_table[176].vz = 9;
	simu_track_table[176].hdg = 95;

	simu_track_table[177].lat = 465186977;
	simu_track_table[177].lon = 65662496;
	simu_track_table[177].alt = 405;
	simu_track_table[177].relative_alt = -5;
	simu_track_table[177].vx = 26;
	simu_track_table[177].vy = -292;
	simu_track_table[177].vz = 8;
	simu_track_table[177].hdg = 95;

	simu_track_table[178].lat = 465186979;
	simu_track_table[178].lon = 65662401;
	simu_track_table[178].alt = 405;
	simu_track_table[178].relative_alt = -5;
	simu_track_table[178].vx = 26;
	simu_track_table[178].vy = -291;
	simu_track_table[178].vz = 7;
	simu_track_table[178].hdg = 95;

	simu_track_table[179].lat = 465186980;
	simu_track_table[179].lon = 65662307;
	simu_track_table[179].alt = 405;
	simu_track_table[179].relative_alt = -5;
	simu_track_table[179].vx = 27;
	simu_track_table[179].vy = -290;
	simu_track_table[179].vz = 6;
	simu_track_table[179].hdg = 95;

	simu_track_table[180].lat = 465186982;
	simu_track_table[180].lon = 65662213;
	simu_track_table[180].alt = 405;
	simu_track_table[180].relative_alt = -5;
	simu_track_table[180].vx = 27;
	simu_track_table[180].vy = -290;
	simu_track_table[180].vz = 5;
	simu_track_table[180].hdg = 95;

	simu_track_table[181].lat = 465186985;
	simu_track_table[181].lon = 65662120;
	simu_track_table[181].alt = 405;
	simu_track_table[181].relative_alt = -5;
	simu_track_table[181].vx = 27;
	simu_track_table[181].vy = -290;
	simu_track_table[181].vz = 4;
	simu_track_table[181].hdg = 95;

	simu_track_table[182].lat = 465186987;
	simu_track_table[182].lon = 65662028;
	simu_track_table[182].alt = 405;
	simu_track_table[182].relative_alt = -5;
	simu_track_table[182].vx = 28;
	simu_track_table[182].vy = -290;
	simu_track_table[182].vz = 3;
	simu_track_table[182].hdg = 95;

	simu_track_table[183].lat = 465186989;
	simu_track_table[183].lon = 65661933;
	simu_track_table[183].alt = 405;
	simu_track_table[183].relative_alt = -5;
	simu_track_table[183].vx = 28;
	simu_track_table[183].vy = -290;
	simu_track_table[183].vz = 2;
	simu_track_table[183].hdg = 96;

	simu_track_table[184].lat = 465186992;
	simu_track_table[184].lon = 65661843;
	simu_track_table[184].alt = 405;
	simu_track_table[184].relative_alt = -5;
	simu_track_table[184].vx = 29;
	simu_track_table[184].vy = -289;
	simu_track_table[184].vz = 1;
	simu_track_table[184].hdg = 96;

	simu_track_table[185].lat = 465186995;
	simu_track_table[185].lon = 65661750;
	simu_track_table[185].alt = 405;
	simu_track_table[185].relative_alt = -5;
	simu_track_table[185].vx = 29;
	simu_track_table[185].vy = -289;
	simu_track_table[185].vz = 0;
	simu_track_table[185].hdg = 96;

	simu_track_table[186].lat = 465186998;
	simu_track_table[186].lon = 65661659;
	simu_track_table[186].alt = 405;
	simu_track_table[186].relative_alt = -5;
	simu_track_table[186].vx = 29;
	simu_track_table[186].vy = -289;
	simu_track_table[186].vz = -1;
	simu_track_table[186].hdg = 96;

	simu_track_table[187].lat = 465187001;
	simu_track_table[187].lon = 65661568;
	simu_track_table[187].alt = 405;
	simu_track_table[187].relative_alt = -5;
	simu_track_table[187].vx = 30;
	simu_track_table[187].vy = -288;
	simu_track_table[187].vz = -2;
	simu_track_table[187].hdg = 96;

	simu_track_table[188].lat = 465187004;
	simu_track_table[188].lon = 65661478;
	simu_track_table[188].alt = 405;
	simu_track_table[188].relative_alt = -5;
	simu_track_table[188].vx = 30;
	simu_track_table[188].vy = -287;
	simu_track_table[188].vz = -3;
	simu_track_table[188].hdg = 96;

	simu_track_table[189].lat = 465187008;
	simu_track_table[189].lon = 65661387;
	simu_track_table[189].alt = 405;
	simu_track_table[189].relative_alt = -5;
	simu_track_table[189].vx = 30;
	simu_track_table[189].vy = -287;
	simu_track_table[189].vz = -4;
	simu_track_table[189].hdg = 96;

	simu_track_table[190].lat = 465187011;
	simu_track_table[190].lon = 65661299;
	simu_track_table[190].alt = 405;
	simu_track_table[190].relative_alt = -5;
	simu_track_table[190].vx = 31;
	simu_track_table[190].vy = -287;
	simu_track_table[190].vz = -5;
	simu_track_table[190].hdg = 96;

	simu_track_table[191].lat = 465187015;
	simu_track_table[191].lon = 65661209;
	simu_track_table[191].alt = 405;
	simu_track_table[191].relative_alt = -5;
	simu_track_table[191].vx = 31;
	simu_track_table[191].vy = -287;
	simu_track_table[191].vz = -6;
	simu_track_table[191].hdg = 96;

	simu_track_table[192].lat = 465187019;
	simu_track_table[192].lon = 65661120;
	simu_track_table[192].alt = 405;
	simu_track_table[192].relative_alt = -5;
	simu_track_table[192].vx = 31;
	simu_track_table[192].vy = -286;
	simu_track_table[192].vz = -8;
	simu_track_table[192].hdg = 96;

	simu_track_table[193].lat = 465187023;
	simu_track_table[193].lon = 65661032;
	simu_track_table[193].alt = 405;
	simu_track_table[193].relative_alt = -5;
	simu_track_table[193].vx = 32;
	simu_track_table[193].vy = -285;
	simu_track_table[193].vz = -9;
	simu_track_table[193].hdg = 96;

	simu_track_table[194].lat = 465187027;
	simu_track_table[194].lon = 65660944;
	simu_track_table[194].alt = 405;
	simu_track_table[194].relative_alt = -5;
	simu_track_table[194].vx = 32;
	simu_track_table[194].vy = -284;
	simu_track_table[194].vz = -10;
	simu_track_table[194].hdg = 96;

	simu_track_table[195].lat = 465187031;
	simu_track_table[195].lon = 65660856;
	simu_track_table[195].alt = 405;
	simu_track_table[195].relative_alt = -5;
	simu_track_table[195].vx = 32;
	simu_track_table[195].vy = -284;
	simu_track_table[195].vz = -12;
	simu_track_table[195].hdg = 97;

	simu_track_table[196].lat = 465187035;
	simu_track_table[196].lon = 65660770;
	simu_track_table[196].alt = 405;
	simu_track_table[196].relative_alt = -5;
	simu_track_table[196].vx = 33;
	simu_track_table[196].vy = -284;
	simu_track_table[196].vz = -13;
	simu_track_table[196].hdg = 97;

	simu_track_table[197].lat = 465187040;
	simu_track_table[197].lon = 65660682;
	simu_track_table[197].alt = 405;
	simu_track_table[197].relative_alt = -5;
	simu_track_table[197].vx = 33;
	simu_track_table[197].vy = -284;
	simu_track_table[197].vz = -14;
	simu_track_table[197].hdg = 97;

	simu_track_table[198].lat = 465187045;
	simu_track_table[198].lon = 65660597;
	simu_track_table[198].alt = 405;
	simu_track_table[198].relative_alt = -5;
	simu_track_table[198].vx = 33;
	simu_track_table[198].vy = -283;
	simu_track_table[198].vz = -16;
	simu_track_table[198].hdg = 97;

	simu_track_table[199].lat = 465187050;
	simu_track_table[199].lon = 65660510;
	simu_track_table[199].alt = 405;
	simu_track_table[199].relative_alt = -5;
	simu_track_table[199].vx = 34;
	simu_track_table[199].vy = -283;
	simu_track_table[199].vz = -17;
	simu_track_table[199].hdg = 97;

	simu_track_table[200].lat = 465187055;
	simu_track_table[200].lon = 65660425;
	simu_track_table[200].alt = 405;
	simu_track_table[200].relative_alt = -5;
	simu_track_table[200].vx = 34;
	simu_track_table[200].vy = -282;
	simu_track_table[200].vz = -19;
	simu_track_table[200].hdg = 97;

	simu_track_table[201].lat = 465187060;
	simu_track_table[201].lon = 65660340;
	simu_track_table[201].alt = 406;
	simu_track_table[201].relative_alt = -6;
	simu_track_table[201].vx = 34;
	simu_track_table[201].vy = -281;
	simu_track_table[201].vz = -21;
	simu_track_table[201].hdg = 97;

	simu_track_table[202].lat = 465187065;
	simu_track_table[202].lon = 65660257;
	simu_track_table[202].alt = 406;
	simu_track_table[202].relative_alt = -6;
	simu_track_table[202].vx = 35;
	simu_track_table[202].vy = -279;
	simu_track_table[202].vz = -22;
	simu_track_table[202].hdg = 97;

	simu_track_table[203].lat = 465187070;
	simu_track_table[203].lon = 65660171;
	simu_track_table[203].alt = 406;
	simu_track_table[203].relative_alt = -6;
	simu_track_table[203].vx = 35;
	simu_track_table[203].vy = -279;
	simu_track_table[203].vz = -24;
	simu_track_table[203].hdg = 97;

	simu_track_table[204].lat = 465187076;
	simu_track_table[204].lon = 65660089;
	simu_track_table[204].alt = 406;
	simu_track_table[204].relative_alt = -6;
	simu_track_table[204].vx = 35;
	simu_track_table[204].vy = -278;
	simu_track_table[204].vz = -26;
	simu_track_table[204].hdg = 97;

	simu_track_table[205].lat = 465187082;
	simu_track_table[205].lon = 65660005;
	simu_track_table[205].alt = 406;
	simu_track_table[205].relative_alt = -6;
	simu_track_table[205].vx = 35;
	simu_track_table[205].vy = -277;
	simu_track_table[205].vz = -28;
	simu_track_table[205].hdg = 97;

	simu_track_table[206].lat = 465187087;
	simu_track_table[206].lon = 65659924;
	simu_track_table[206].alt = 406;
	simu_track_table[206].relative_alt = -6;
	simu_track_table[206].vx = 36;
	simu_track_table[206].vy = -277;
	simu_track_table[206].vz = -30;
	simu_track_table[206].hdg = 97;

	simu_track_table[207].lat = 465187093;
	simu_track_table[207].lon = 65659841;
	simu_track_table[207].alt = 406;
	simu_track_table[207].relative_alt = -6;
	simu_track_table[207].vx = 36;
	simu_track_table[207].vy = -276;
	simu_track_table[207].vz = -32;
	simu_track_table[207].hdg = 97;

	simu_track_table[208].lat = 465187099;
	simu_track_table[208].lon = 65659760;
	simu_track_table[208].alt = 406;
	simu_track_table[208].relative_alt = -6;
	simu_track_table[208].vx = 36;
	simu_track_table[208].vy = -275;
	simu_track_table[208].vz = -34;
	simu_track_table[208].hdg = 97;

	simu_track_table[209].lat = 465187105;
	simu_track_table[209].lon = 65659679;
	simu_track_table[209].alt = 406;
	simu_track_table[209].relative_alt = -6;
	simu_track_table[209].vx = 36;
	simu_track_table[209].vy = -272;
	simu_track_table[209].vz = -34;
	simu_track_table[209].hdg = 98;

	simu_track_table[210].lat = 465187111;
	simu_track_table[210].lon = 65659601;
	simu_track_table[210].alt = 406;
	simu_track_table[210].relative_alt = -6;
	simu_track_table[210].vx = 36;
	simu_track_table[210].vy = -266;
	simu_track_table[210].vz = -35;
	simu_track_table[210].hdg = 98;

	simu_track_table[211].lat = 465187117;
	simu_track_table[211].lon = 65659523;
	simu_track_table[211].alt = 406;
	simu_track_table[211].relative_alt = -6;
	simu_track_table[211].vx = 35;
	simu_track_table[211].vy = -258;
	simu_track_table[211].vz = -35;
	simu_track_table[211].hdg = 98;

	simu_track_table[212].lat = 465187123;
	simu_track_table[212].lon = 65659451;
	simu_track_table[212].alt = 406;
	simu_track_table[212].relative_alt = -6;
	simu_track_table[212].vx = 34;
	simu_track_table[212].vy = -248;
	simu_track_table[212].vz = -36;
	simu_track_table[212].hdg = 98;

	simu_track_table[213].lat = 465187129;
	simu_track_table[213].lon = 65659381;
	simu_track_table[213].alt = 406;
	simu_track_table[213].relative_alt = -6;
	simu_track_table[213].vx = 33;
	simu_track_table[213].vy = -237;
	simu_track_table[213].vz = -36;
	simu_track_table[213].hdg = 98;

	simu_track_table[214].lat = 465187134;
	simu_track_table[214].lon = 65659315;
	simu_track_table[214].alt = 407;
	simu_track_table[214].relative_alt = -7;
	simu_track_table[214].vx = 32;
	simu_track_table[214].vy = -226;
	simu_track_table[214].vz = -36;
	simu_track_table[214].hdg = 98;

	simu_track_table[215].lat = 465187139;
	simu_track_table[215].lon = 65659253;
	simu_track_table[215].alt = 407;
	simu_track_table[215].relative_alt = -7;
	simu_track_table[215].vx = 30;
	simu_track_table[215].vy = -215;
	simu_track_table[215].vz = -36;
	simu_track_table[215].hdg = 98;

	simu_track_table[216].lat = 465187144;
	simu_track_table[216].lon = 65659195;
	simu_track_table[216].alt = 407;
	simu_track_table[216].relative_alt = -7;
	simu_track_table[216].vx = 29;
	simu_track_table[216].vy = -205;
	simu_track_table[216].vz = -35;
	simu_track_table[216].hdg = 98;

	simu_track_table[217].lat = 465187149;
	simu_track_table[217].lon = 65659140;
	simu_track_table[217].alt = 407;
	simu_track_table[217].relative_alt = -7;
	simu_track_table[217].vx = 28;
	simu_track_table[217].vy = -196;
	simu_track_table[217].vz = -35;
	simu_track_table[217].hdg = 98;

	simu_track_table[218].lat = 465187154;
	simu_track_table[218].lon = 65659089;
	simu_track_table[218].alt = 407;
	simu_track_table[218].relative_alt = -7;
	simu_track_table[218].vx = 27;
	simu_track_table[218].vy = -188;
	simu_track_table[218].vz = -35;
	simu_track_table[218].hdg = 98;

	simu_track_table[219].lat = 465187158;
	simu_track_table[219].lon = 65659040;
	simu_track_table[219].alt = 407;
	simu_track_table[219].relative_alt = -7;
	simu_track_table[219].vx = 26;
	simu_track_table[219].vy = -179;
	simu_track_table[219].vz = -35;
	simu_track_table[219].hdg = 98;

	simu_track_table[220].lat = 465187162;
	simu_track_table[220].lon = 65658995;
	simu_track_table[220].alt = 407;
	simu_track_table[220].relative_alt = -7;
	simu_track_table[220].vx = 25;
	simu_track_table[220].vy = -171;
	simu_track_table[220].vz = -34;
	simu_track_table[220].hdg = 98;

	simu_track_table[221].lat = 465187166;
	simu_track_table[221].lon = 65658952;
	simu_track_table[221].alt = 407;
	simu_track_table[221].relative_alt = -7;
	simu_track_table[221].vx = 24;
	simu_track_table[221].vy = -164;
	simu_track_table[221].vz = -34;
	simu_track_table[221].hdg = 98;

	simu_track_table[222].lat = 465187170;
	simu_track_table[222].lon = 65658912;
	simu_track_table[222].alt = 407;
	simu_track_table[222].relative_alt = -7;
	simu_track_table[222].vx = 23;
	simu_track_table[222].vy = -156;
	simu_track_table[222].vz = -34;
	simu_track_table[222].hdg = 98;

	simu_track_table[223].lat = 465187174;
	simu_track_table[223].lon = 65658875;
	simu_track_table[223].alt = 407;
	simu_track_table[223].relative_alt = -7;
	simu_track_table[223].vx = 22;
	simu_track_table[223].vy = -149;
	simu_track_table[223].vz = -34;
	simu_track_table[223].hdg = 98;

	simu_track_table[224].lat = 465187177;
	simu_track_table[224].lon = 65658841;
	simu_track_table[224].alt = 407;
	simu_track_table[224].relative_alt = -7;
	simu_track_table[224].vx = 19;
	simu_track_table[224].vy = -139;
	simu_track_table[224].vz = 1;
	simu_track_table[224].hdg = 98;

	simu_track_table[225].lat = 465187178;
	simu_track_table[225].lon = 65658813;
	simu_track_table[225].alt = 407;
	simu_track_table[225].relative_alt = -7;
	simu_track_table[225].vx = 2;
	simu_track_table[225].vy = -104;
	simu_track_table[225].vz = 23;
	simu_track_table[225].hdg = 91;

	simu_track_table[226].lat = 465187174;
	simu_track_table[226].lon = 65658802;
	simu_track_table[226].alt = 407;
	simu_track_table[226].relative_alt = -7;
	simu_track_table[226].vx = -31;
	simu_track_table[226].vy = -40;
	simu_track_table[226].vz = 36;
	simu_track_table[226].hdg = 53;

	simu_track_table[227].lat = 465187161;
	simu_track_table[227].lon = 65658815;
	simu_track_table[227].alt = 407;
	simu_track_table[227].relative_alt = -7;
	simu_track_table[227].vx = -75;
	simu_track_table[227].vy = 33;
	simu_track_table[227].vz = 46;
	simu_track_table[227].hdg = 337;

	simu_track_table[228].lat = 465187137;
	simu_track_table[228].lon = 65658850;
	simu_track_table[228].alt = 407;
	simu_track_table[228].relative_alt = -7;
	simu_track_table[228].vx = -124;
	simu_track_table[228].vy = 89;
	simu_track_table[228].vz = 47;
	simu_track_table[228].hdg = 324;

	simu_track_table[229].lat = 465187100;
	simu_track_table[229].lon = 65658901;
	simu_track_table[229].alt = 407;
	simu_track_table[229].relative_alt = -7;
	simu_track_table[229].vx = -185;
	simu_track_table[229].vy = 119;
	simu_track_table[229].vz = 47;
	simu_track_table[229].hdg = 327;

	simu_track_table[230].lat = 465187048;
	simu_track_table[230].lon = 65658959;
	simu_track_table[230].alt = 407;
	simu_track_table[230].relative_alt = -7;
	simu_track_table[230].vx = -251;
	simu_track_table[230].vy = 124;
	simu_track_table[230].vz = 48;
	simu_track_table[230].hdg = 334;

	simu_track_table[231].lat = 465186982;
	simu_track_table[231].lon = 65659016;
	simu_track_table[231].alt = 407;
	simu_track_table[231].relative_alt = -7;
	simu_track_table[231].vx = -302;
	simu_track_table[231].vy = 113;
	simu_track_table[231].vz = 44;
	simu_track_table[231].hdg = 339;

	simu_track_table[232].lat = 465186908;
	simu_track_table[232].lon = 65659068;
	simu_track_table[232].alt = 406;
	simu_track_table[232].relative_alt = -6;
	simu_track_table[232].vx = -324;
	simu_track_table[232].vy = 95;
	simu_track_table[232].vz = 35;
	simu_track_table[232].hdg = 344;

	simu_track_table[233].lat = 465186828;
	simu_track_table[233].lon = 65659116;
	simu_track_table[233].alt = 406;
	simu_track_table[233].relative_alt = -6;
	simu_track_table[233].vx = -324;
	simu_track_table[233].vy = 79;
	simu_track_table[233].vz = 26;
	simu_track_table[233].hdg = 346;

	simu_track_table[234].lat = 465186753;
	simu_track_table[234].lon = 65659159;
	simu_track_table[234].alt = 406;
	simu_track_table[234].relative_alt = -6;
	simu_track_table[234].vx = -312;
	simu_track_table[234].vy = 70;
	simu_track_table[234].vz = 20;
	simu_track_table[234].hdg = 347;

	simu_track_table[235].lat = 465186678;
	simu_track_table[235].lon = 65659200;
	simu_track_table[235].alt = 406;
	simu_track_table[235].relative_alt = -6;
	simu_track_table[235].vx = -300;
	simu_track_table[235].vy = 67;
	simu_track_table[235].vz = 17;
	simu_track_table[235].hdg = 347;

	simu_track_table[236].lat = 465186607;
	simu_track_table[236].lon = 65659240;
	simu_track_table[236].alt = 406;
	simu_track_table[236].relative_alt = -6;
	simu_track_table[236].vx = -292;
	simu_track_table[236].vy = 67;
	simu_track_table[236].vz = 17;
	simu_track_table[236].hdg = 347;

	simu_track_table[237].lat = 465186536;
	simu_track_table[237].lon = 65659280;
	simu_track_table[237].alt = 406;
	simu_track_table[237].relative_alt = -6;
	simu_track_table[237].vx = -292;
	simu_track_table[237].vy = 67;
	simu_track_table[237].vz = 17;
	simu_track_table[237].hdg = 347;

	simu_track_table[238].lat = 465186466;
	simu_track_table[238].lon = 65659319;
	simu_track_table[238].alt = 406;
	simu_track_table[238].relative_alt = -6;
	simu_track_table[238].vx = -295;
	simu_track_table[238].vy = 66;
	simu_track_table[238].vz = 18;
	simu_track_table[238].hdg = 347;

	simu_track_table[239].lat = 465186393;
	simu_track_table[239].lon = 65659358;
	simu_track_table[239].alt = 406;
	simu_track_table[239].relative_alt = -6;
	simu_track_table[239].vx = -300;
	simu_track_table[239].vy = 62;
	simu_track_table[239].vz = 18;
	simu_track_table[239].hdg = 348;

	simu_track_table[240].lat = 465186322;
	simu_track_table[240].lon = 65659394;
	simu_track_table[240].alt = 406;
	simu_track_table[240].relative_alt = -6;
	simu_track_table[240].vx = -303;
	simu_track_table[240].vy = 58;
	simu_track_table[240].vz = 17;
	simu_track_table[240].hdg = 349;

	simu_track_table[241].lat = 465186249;
	simu_track_table[241].lon = 65659429;
	simu_track_table[241].alt = 406;
	simu_track_table[241].relative_alt = -6;
	simu_track_table[241].vx = -302;
	simu_track_table[241].vy = 53;
	simu_track_table[241].vz = 15;
	simu_track_table[241].hdg = 350;

	simu_track_table[242].lat = 465186179;
	simu_track_table[242].lon = 65659461;
	simu_track_table[242].alt = 406;
	simu_track_table[242].relative_alt = -6;
	simu_track_table[242].vx = -300;
	simu_track_table[242].vy = 48;
	simu_track_table[242].vz = 14;
	simu_track_table[242].hdg = 351;

	simu_track_table[243].lat = 465186107;
	simu_track_table[243].lon = 65659492;
	simu_track_table[243].alt = 406;
	simu_track_table[243].relative_alt = -6;
	simu_track_table[243].vx = -298;
	simu_track_table[243].vy = 45;
	simu_track_table[243].vz = 13;
	simu_track_table[243].hdg = 351;

	simu_track_table[244].lat = 465186037;
	simu_track_table[244].lon = 65659521;
	simu_track_table[244].alt = 406;
	simu_track_table[244].relative_alt = -6;
	simu_track_table[244].vx = -296;
	simu_track_table[244].vy = 43;
	simu_track_table[244].vz = 12;
	simu_track_table[244].hdg = 352;

	simu_track_table[245].lat = 465185967;
	simu_track_table[245].lon = 65659550;
	simu_track_table[245].alt = 406;
	simu_track_table[245].relative_alt = -6;
	simu_track_table[245].vx = -295;
	simu_track_table[245].vy = 41;
	simu_track_table[245].vz = 11;
	simu_track_table[245].hdg = 352;

	simu_track_table[246].lat = 465185899;
	simu_track_table[246].lon = 65659577;
	simu_track_table[246].alt = 406;
	simu_track_table[246].relative_alt = -6;
	simu_track_table[246].vx = -293;
	simu_track_table[246].vy = 40;
	simu_track_table[246].vz = 11;
	simu_track_table[246].hdg = 352;

	simu_track_table[247].lat = 465185830;
	simu_track_table[247].lon = 65659604;
	simu_track_table[247].alt = 406;
	simu_track_table[247].relative_alt = -6;
	simu_track_table[247].vx = -293;
	simu_track_table[247].vy = 38;
	simu_track_table[247].vz = 10;
	simu_track_table[247].hdg = 353;

	simu_track_table[248].lat = 465185763;
	simu_track_table[248].lon = 65659629;
	simu_track_table[248].alt = 406;
	simu_track_table[248].relative_alt = -6;
	simu_track_table[248].vx = -294;
	simu_track_table[248].vy = 37;
	simu_track_table[248].vz = 9;
	simu_track_table[248].hdg = 353;

	simu_track_table[249].lat = 465185694;
	simu_track_table[249].lon = 65659653;
	simu_track_table[249].alt = 406;
	simu_track_table[249].relative_alt = -6;
	simu_track_table[249].vx = -294;
	simu_track_table[249].vy = 35;
	simu_track_table[249].vz = 8;
	simu_track_table[249].hdg = 353;

	simu_track_table[250].lat = 465185626;
	simu_track_table[250].lon = 65659677;
	simu_track_table[250].alt = 406;
	simu_track_table[250].relative_alt = -6;
	simu_track_table[250].vx = -294;
	simu_track_table[250].vy = 34;
	simu_track_table[250].vz = 7;
	simu_track_table[250].hdg = 353;

	simu_track_table[251].lat = 465185559;
	simu_track_table[251].lon = 65659700;
	simu_track_table[251].alt = 406;
	simu_track_table[251].relative_alt = -6;
	simu_track_table[251].vx = -293;
	simu_track_table[251].vy = 32;
	simu_track_table[251].vz = 6;
	simu_track_table[251].hdg = 354;

	simu_track_table[252].lat = 465185493;
	simu_track_table[252].lon = 65659722;
	simu_track_table[252].alt = 406;
	simu_track_table[252].relative_alt = -6;
	simu_track_table[252].vx = -291;
	simu_track_table[252].vy = 31;
	simu_track_table[252].vz = 5;
	simu_track_table[252].hdg = 354;

	simu_track_table[253].lat = 465185426;
	simu_track_table[253].lon = 65659743;
	simu_track_table[253].alt = 406;
	simu_track_table[253].relative_alt = -6;
	simu_track_table[253].vx = -291;
	simu_track_table[253].vy = 30;
	simu_track_table[253].vz = 4;
	simu_track_table[253].hdg = 354;

	simu_track_table[254].lat = 465185361;
	simu_track_table[254].lon = 65659763;
	simu_track_table[254].alt = 406;
	simu_track_table[254].relative_alt = -6;
	simu_track_table[254].vx = -290;
	simu_track_table[254].vy = 29;
	simu_track_table[254].vz = 3;
	simu_track_table[254].hdg = 354;

	simu_track_table[255].lat = 465185294;
	simu_track_table[255].lon = 65659783;
	simu_track_table[255].alt = 406;
	simu_track_table[255].relative_alt = -6;
	simu_track_table[255].vx = -290;
	simu_track_table[255].vy = 28;
	simu_track_table[255].vz = 2;
	simu_track_table[255].hdg = 354;

	simu_track_table[256].lat = 465185230;
	simu_track_table[256].lon = 65659802;
	simu_track_table[256].alt = 406;
	simu_track_table[256].relative_alt = -6;
	simu_track_table[256].vx = -290;
	simu_track_table[256].vy = 27;
	simu_track_table[256].vz = 0;
	simu_track_table[256].hdg = 355;

	simu_track_table[257].lat = 465185164;
	simu_track_table[257].lon = 65659821;
	simu_track_table[257].alt = 406;
	simu_track_table[257].relative_alt = -6;
	simu_track_table[257].vx = -290;
	simu_track_table[257].vy = 26;
	simu_track_table[257].vz = -1;
	simu_track_table[257].hdg = 355;

	simu_track_table[258].lat = 465185100;
	simu_track_table[258].lon = 65659838;
	simu_track_table[258].alt = 406;
	simu_track_table[258].relative_alt = -6;
	simu_track_table[258].vx = -289;
	simu_track_table[258].vy = 25;
	simu_track_table[258].vz = -2;
	simu_track_table[258].hdg = 355;

	simu_track_table[259].lat = 465185035;
	simu_track_table[259].lon = 65659856;
	simu_track_table[259].alt = 406;
	simu_track_table[259].relative_alt = -6;
	simu_track_table[259].vx = -289;
	simu_track_table[259].vy = 24;
	simu_track_table[259].vz = -4;
	simu_track_table[259].hdg = 355;

	simu_track_table[260].lat = 465184972;
	simu_track_table[260].lon = 65659872;
	simu_track_table[260].alt = 406;
	simu_track_table[260].relative_alt = -6;
	simu_track_table[260].vx = -288;
	simu_track_table[260].vy = 23;
	simu_track_table[260].vz = -5;
	simu_track_table[260].hdg = 355;

	simu_track_table[261].lat = 465184907;
	simu_track_table[261].lon = 65659888;
	simu_track_table[261].alt = 406;
	simu_track_table[261].relative_alt = -6;
	simu_track_table[261].vx = -287;
	simu_track_table[261].vy = 22;
	simu_track_table[261].vz = -6;
	simu_track_table[261].hdg = 356;

	simu_track_table[262].lat = 465184845;
	simu_track_table[262].lon = 65659903;
	simu_track_table[262].alt = 406;
	simu_track_table[262].relative_alt = -6;
	simu_track_table[262].vx = -287;
	simu_track_table[262].vy = 22;
	simu_track_table[262].vz = -8;
	simu_track_table[262].hdg = 356;

	simu_track_table[263].lat = 465184782;
	simu_track_table[263].lon = 65659917;
	simu_track_table[263].alt = 406;
	simu_track_table[263].relative_alt = -6;
	simu_track_table[263].vx = -286;
	simu_track_table[263].vy = 21;
	simu_track_table[263].vz = -9;
	simu_track_table[263].hdg = 356;

	simu_track_table[264].lat = 465184719;
	simu_track_table[264].lon = 65659931;
	simu_track_table[264].alt = 406;
	simu_track_table[264].relative_alt = -6;
	simu_track_table[264].vx = -286;
	simu_track_table[264].vy = 20;
	simu_track_table[264].vz = -11;
	simu_track_table[264].hdg = 356;

	simu_track_table[265].lat = 465184657;
	simu_track_table[265].lon = 65659945;
	simu_track_table[265].alt = 406;
	simu_track_table[265].relative_alt = -6;
	simu_track_table[265].vx = -286;
	simu_track_table[265].vy = 19;
	simu_track_table[265].vz = -12;
	simu_track_table[265].hdg = 356;

	simu_track_table[266].lat = 465184595;
	simu_track_table[266].lon = 65659957;
	simu_track_table[266].alt = 406;
	simu_track_table[266].relative_alt = -6;
	simu_track_table[266].vx = -285;
	simu_track_table[266].vy = 18;
	simu_track_table[266].vz = -14;
	simu_track_table[266].hdg = 356;

	simu_track_table[267].lat = 465184533;
	simu_track_table[267].lon = 65659970;
	simu_track_table[267].alt = 406;
	simu_track_table[267].relative_alt = -6;
	simu_track_table[267].vx = -285;
	simu_track_table[267].vy = 17;
	simu_track_table[267].vz = -16;
	simu_track_table[267].hdg = 357;

	simu_track_table[268].lat = 465184472;
	simu_track_table[268].lon = 65659981;
	simu_track_table[268].alt = 406;
	simu_track_table[268].relative_alt = -6;
	simu_track_table[268].vx = -284;
	simu_track_table[268].vy = 16;
	simu_track_table[268].vz = -18;
	simu_track_table[268].hdg = 357;

	simu_track_table[269].lat = 465184410;
	simu_track_table[269].lon = 65659992;
	simu_track_table[269].alt = 406;
	simu_track_table[269].relative_alt = -6;
	simu_track_table[269].vx = -283;
	simu_track_table[269].vy = 15;
	simu_track_table[269].vz = -20;
	simu_track_table[269].hdg = 357;

	simu_track_table[270].lat = 465184351;
	simu_track_table[270].lon = 65660002;
	simu_track_table[270].alt = 406;
	simu_track_table[270].relative_alt = -6;
	simu_track_table[270].vx = -282;
	simu_track_table[270].vy = 15;
	simu_track_table[270].vz = -22;
	simu_track_table[270].hdg = 357;

	simu_track_table[271].lat = 465184290;
	simu_track_table[271].lon = 65660012;
	simu_track_table[271].alt = 406;
	simu_track_table[271].relative_alt = -6;
	simu_track_table[271].vx = -282;
	simu_track_table[271].vy = 14;
	simu_track_table[271].vz = -24;
	simu_track_table[271].hdg = 357;

	simu_track_table[272].lat = 465184231;
	simu_track_table[272].lon = 65660021;
	simu_track_table[272].alt = 406;
	simu_track_table[272].relative_alt = -6;
	simu_track_table[272].vx = -281;
	simu_track_table[272].vy = 13;
	simu_track_table[272].vz = -27;
	simu_track_table[272].hdg = 357;

	simu_track_table[273].lat = 465184171;
	simu_track_table[273].lon = 65660030;
	simu_track_table[273].alt = 406;
	simu_track_table[273].relative_alt = -6;
	simu_track_table[273].vx = -279;
	simu_track_table[273].vy = 12;
	simu_track_table[273].vz = -29;
	simu_track_table[273].hdg = 358;

	simu_track_table[274].lat = 465184113;
	simu_track_table[274].lon = 65660038;
	simu_track_table[274].alt = 406;
	simu_track_table[274].relative_alt = -6;
	simu_track_table[274].vx = -278;
	simu_track_table[274].vy = 11;
	simu_track_table[274].vz = -31;
	simu_track_table[274].hdg = 358;

	simu_track_table[275].lat = 465184054;
	simu_track_table[275].lon = 65660045;
	simu_track_table[275].alt = 406;
	simu_track_table[275].relative_alt = -6;
	simu_track_table[275].vx = -277;
	simu_track_table[275].vy = 10;
	simu_track_table[275].vz = -33;
	simu_track_table[275].hdg = 358;

	simu_track_table[276].lat = 465183997;
	simu_track_table[276].lon = 65660052;
	simu_track_table[276].alt = 407;
	simu_track_table[276].relative_alt = -7;
	simu_track_table[276].vx = -274;
	simu_track_table[276].vy = 9;
	simu_track_table[276].vz = -34;
	simu_track_table[276].hdg = 358;

	simu_track_table[277].lat = 465183940;
	simu_track_table[277].lon = 65660058;
	simu_track_table[277].alt = 407;
	simu_track_table[277].relative_alt = -7;
	simu_track_table[277].vx = -269;
	simu_track_table[277].vy = 8;
	simu_track_table[277].vz = -35;
	simu_track_table[277].hdg = 358;

	simu_track_table[278].lat = 465183886;
	simu_track_table[278].lon = 65660064;
	simu_track_table[278].alt = 407;
	simu_track_table[278].relative_alt = -7;
	simu_track_table[278].vx = -260;
	simu_track_table[278].vy = 7;
	simu_track_table[278].vz = -36;
	simu_track_table[278].hdg = 358;

	simu_track_table[279].lat = 465183833;
	simu_track_table[279].lon = 65660069;
	simu_track_table[279].alt = 407;
	simu_track_table[279].relative_alt = -7;
	simu_track_table[279].vx = -248;
	simu_track_table[279].vy = 6;
	simu_track_table[279].vz = -36;
	simu_track_table[279].hdg = 359;

	simu_track_table[280].lat = 465183784;
	simu_track_table[280].lon = 65660073;
	simu_track_table[280].alt = 407;
	simu_track_table[280].relative_alt = -7;
	simu_track_table[280].vx = -234;
	simu_track_table[280].vy = 5;
	simu_track_table[280].vz = -37;
	simu_track_table[280].hdg = 359;

	simu_track_table[281].lat = 465183738;
	simu_track_table[281].lon = 65660077;
	simu_track_table[281].alt = 407;
	simu_track_table[281].relative_alt = -7;
	simu_track_table[281].vx = -220;
	simu_track_table[281].vy = 4;
	simu_track_table[281].vz = -36;
	simu_track_table[281].hdg = 359;

	simu_track_table[282].lat = 465183696;
	simu_track_table[282].lon = 65660080;
	simu_track_table[282].alt = 407;
	simu_track_table[282].relative_alt = -7;
	simu_track_table[282].vx = -208;
	simu_track_table[282].vy = 3;
	simu_track_table[282].vz = -36;
	simu_track_table[282].hdg = 359;

	simu_track_table[283].lat = 465183656;
	simu_track_table[283].lon = 65660083;
	simu_track_table[283].alt = 407;
	simu_track_table[283].relative_alt = -7;
	simu_track_table[283].vx = -196;
	simu_track_table[283].vy = 2;
	simu_track_table[283].vz = -36;
	simu_track_table[283].hdg = 359;

	simu_track_table[284].lat = 465183620;
	simu_track_table[284].lon = 65660085;
	simu_track_table[284].alt = 407;
	simu_track_table[284].relative_alt = -7;
	simu_track_table[284].vx = -185;
	simu_track_table[284].vy = 1;
	simu_track_table[284].vz = -35;
	simu_track_table[284].hdg = 360;

	simu_track_table[285].lat = 465183586;
	simu_track_table[285].lon = 65660086;
	simu_track_table[285].alt = 407;
	simu_track_table[285].relative_alt = -7;
	simu_track_table[285].vx = -175;
	simu_track_table[285].vy = 1;
	simu_track_table[285].vz = -35;
	simu_track_table[285].hdg = 360;

	simu_track_table[286].lat = 465183555;
	simu_track_table[286].lon = 65660088;
	simu_track_table[286].alt = 407;
	simu_track_table[286].relative_alt = -7;
	simu_track_table[286].vx = -164;
	simu_track_table[286].vy = 0;
	simu_track_table[286].vz = -29;
	simu_track_table[286].hdg = 360;

	simu_track_table[287].lat = 465183527;
	simu_track_table[287].lon = 65660088;
	simu_track_table[287].alt = 407;
	simu_track_table[287].relative_alt = -7;
	simu_track_table[287].vx = -147;
	simu_track_table[287].vy = -1;
	simu_track_table[287].vz = -9;
	simu_track_table[287].hdg = 0;

	simu_track_table[288].lat = 465183504;
	simu_track_table[288].lon = 65660088;
	simu_track_table[288].alt = 407;
	simu_track_table[288].relative_alt = -7;
	simu_track_table[288].vx = -113;
	simu_track_table[288].vy = -3;
	simu_track_table[288].vz = -0;
	simu_track_table[288].hdg = 2;

	simu_track_table[289].lat = 465183491;
	simu_track_table[289].lon = 65660087;
	simu_track_table[289].alt = 407;
	simu_track_table[289].relative_alt = -7;
	simu_track_table[289].vx = -72;
	simu_track_table[289].vy = -7;
	simu_track_table[289].vz = 3;
	simu_track_table[289].hdg = 6;

	simu_track_table[290].lat = 465183488;
	simu_track_table[290].lon = 65660084;
	simu_track_table[290].alt = 407;
	simu_track_table[290].relative_alt = -7;
	simu_track_table[290].vx = -36;
	simu_track_table[290].vy = -10;
	simu_track_table[290].vz = 4;
	simu_track_table[290].hdg = 15;

	simu_track_table[291].lat = 465183491;
	simu_track_table[291].lon = 65660081;
	simu_track_table[291].alt = 407;
	simu_track_table[291].relative_alt = -7;
	simu_track_table[291].vx = -17;
	simu_track_table[291].vy = -8;
	simu_track_table[291].vz = 4;
	simu_track_table[291].hdg = 26;

	simu_track_table[292].lat = 465183497;
	simu_track_table[292].lon = 65660079;
	simu_track_table[292].alt = 407;
	simu_track_table[292].relative_alt = -7;
	simu_track_table[292].vx = -14;
	simu_track_table[292].vy = -3;
	simu_track_table[292].vz = 3;
	simu_track_table[292].hdg = 10;

	simu_track_table[293].lat = 465183503;
	simu_track_table[293].lon = 65660079;
	simu_track_table[293].alt = 407;
	simu_track_table[293].relative_alt = -7;
	simu_track_table[293].vx = -23;
	simu_track_table[293].vy = 5;
	simu_track_table[293].vz = 3;
	simu_track_table[293].hdg = 348;

	simu_track_table[294].lat = 465183506;
	simu_track_table[294].lon = 65660080;
	simu_track_table[294].alt = 407;
	simu_track_table[294].relative_alt = -7;
	simu_track_table[294].vx = -35;
	simu_track_table[294].vy = 10;
	simu_track_table[294].vz = 4;
	simu_track_table[294].hdg = 345;

	simu_track_table[295].lat = 465183507;
	simu_track_table[295].lon = 65660082;
	simu_track_table[295].alt = 407;
	simu_track_table[295].relative_alt = -7;
	simu_track_table[295].vx = -47;
	simu_track_table[295].vy = 10;
	simu_track_table[295].vz = 4;
	simu_track_table[295].hdg = 348;

	simu_track_table[296].lat = 465183505;
	simu_track_table[296].lon = 65660084;
	simu_track_table[296].alt = 407;
	simu_track_table[296].relative_alt = -7;
	simu_track_table[296].vx = -55;
	simu_track_table[296].vy = 6;
	simu_track_table[296].vz = 4;
	simu_track_table[296].hdg = 353;

	simu_track_table[297].lat = 465183501;
	simu_track_table[297].lon = 65660084;
	simu_track_table[297].alt = 407;
	simu_track_table[297].relative_alt = -7;
	simu_track_table[297].vx = -58;
	simu_track_table[297].vy = 1;
	simu_track_table[297].vz = 4;
	simu_track_table[297].hdg = 359;

	simu_track_table[298].lat = 465183497;
	simu_track_table[298].lon = 65660082;
	simu_track_table[298].alt = 407;
	simu_track_table[298].relative_alt = -7;
	simu_track_table[298].vx = -59;
	simu_track_table[298].vy = -3;
	simu_track_table[298].vz = 4;
	simu_track_table[298].hdg = 2;

	simu_track_table[299].lat = 465183492;
	simu_track_table[299].lon = 65660079;
	simu_track_table[299].alt = 407;
	simu_track_table[299].relative_alt = -7;
	simu_track_table[299].vx = -59;
	simu_track_table[299].vy = -4;
	simu_track_table[299].vz = 3;
	simu_track_table[299].hdg = 4;

	simu_track_table[300].lat = 465183487;
	simu_track_table[300].lon = 65660076;
	simu_track_table[300].alt = 407;
	simu_track_table[300].relative_alt = -7;
	simu_track_table[300].vx = -59;
	simu_track_table[300].vy = -3;
	simu_track_table[300].vz = 3;
	simu_track_table[300].hdg = 3;

	simu_track_table[301].lat = 465183481;
	simu_track_table[301].lon = 65660073;
	simu_track_table[301].alt = 407;
	simu_track_table[301].relative_alt = -7;
	simu_track_table[301].vx = -60;
	simu_track_table[301].vy = -1;
	simu_track_table[301].vz = 3;
	simu_track_table[301].hdg = 1;
}

#elif TRACK == 1

void simu_gps_track_square_track(void)
{
	simu_track_table[0].lat = 465182996;
	simu_track_table[0].lon = 65662445;
	simu_track_table[0].alt = 408;
	simu_track_table[0].relative_alt = -8;
	simu_track_table[0].vx = 29;
	simu_track_table[0].vy = 2;
	simu_track_table[0].vz = 1;
	simu_track_table[0].hdg = 184;

	simu_track_table[1].lat = 465183000;
	simu_track_table[1].lon = 65662445;
	simu_track_table[1].alt = 408;
	simu_track_table[1].relative_alt = -8;
	simu_track_table[1].vx = 34;
	simu_track_table[1].vy = 2;
	simu_track_table[1].vz = -3;
	simu_track_table[1].hdg = 183;

	simu_track_table[2].lat = 465183006;
	simu_track_table[2].lon = 65662443;
	simu_track_table[2].alt = 408;
	simu_track_table[2].relative_alt = -8;
	simu_track_table[2].vx = 46;
	simu_track_table[2].vy = -19;
	simu_track_table[2].vz = -14;
	simu_track_table[2].hdg = 158;

	simu_track_table[3].lat = 465183014;
	simu_track_table[3].lon = 65662428;
	simu_track_table[3].alt = 408;
	simu_track_table[3].relative_alt = -8;
	simu_track_table[3].vx = 66;
	simu_track_table[3].vy = -75;
	simu_track_table[3].vz = -21;
	simu_track_table[3].hdg = 131;

	simu_track_table[4].lat = 465183028;
	simu_track_table[4].lon = 65662392;
	simu_track_table[4].alt = 408;
	simu_track_table[4].relative_alt = -8;
	simu_track_table[4].vx = 93;
	simu_track_table[4].vy = -152;
	simu_track_table[4].vz = -26;
	simu_track_table[4].hdg = 121;

	simu_track_table[5].lat = 465183046;
	simu_track_table[5].lon = 65662332;
	simu_track_table[5].alt = 408;
	simu_track_table[5].relative_alt = -8;
	simu_track_table[5].vx = 116;
	simu_track_table[5].vy = -215;
	simu_track_table[5].vz = -30;
	simu_track_table[5].hdg = 118;

	simu_track_table[6].lat = 465183070;
	simu_track_table[6].lon = 65662259;
	simu_track_table[6].alt = 408;
	simu_track_table[6].relative_alt = -8;
	simu_track_table[6].vx = 136;
	simu_track_table[6].vy = -246;
	simu_track_table[6].vz = -35;
	simu_track_table[6].hdg = 119;

	simu_track_table[7].lat = 465183095;
	simu_track_table[7].lon = 65662184;
	simu_track_table[7].alt = 408;
	simu_track_table[7].relative_alt = -8;
	simu_track_table[7].vx = 145;
	simu_track_table[7].vy = -249;
	simu_track_table[7].vz = -42;
	simu_track_table[7].hdg = 120;

	simu_track_table[8].lat = 465183122;
	simu_track_table[8].lon = 65662111;
	simu_track_table[8].alt = 408;
	simu_track_table[8].relative_alt = -8;
	simu_track_table[8].vx = 147;
	simu_track_table[8].vy = -235;
	simu_track_table[8].vz = -46;
	simu_track_table[8].hdg = 122;

	simu_track_table[9].lat = 465183146;
	simu_track_table[9].lon = 65662045;
	simu_track_table[9].alt = 408;
	simu_track_table[9].relative_alt = -8;
	simu_track_table[9].vx = 143;
	simu_track_table[9].vy = -219;
	simu_track_table[9].vz = -48;
	simu_track_table[9].hdg = 123;

	simu_track_table[10].lat = 465183170;
	simu_track_table[10].lon = 65661980;
	simu_track_table[10].alt = 409;
	simu_track_table[10].relative_alt = -9;
	simu_track_table[10].vx = 139;
	simu_track_table[10].vy = -213;
	simu_track_table[10].vz = -48;
	simu_track_table[10].hdg = 123;

	simu_track_table[11].lat = 465183191;
	simu_track_table[11].lon = 65661917;
	simu_track_table[11].alt = 409;
	simu_track_table[11].relative_alt = -9;
	simu_track_table[11].vx = 134;
	simu_track_table[11].vy = -213;
	simu_track_table[11].vz = -47;
	simu_track_table[11].hdg = 122;

	simu_track_table[12].lat = 465183212;
	simu_track_table[12].lon = 65661850;
	simu_track_table[12].alt = 409;
	simu_track_table[12].relative_alt = -9;
	simu_track_table[12].vx = 134;
	simu_track_table[12].vy = -220;
	simu_track_table[12].vz = -46;
	simu_track_table[12].hdg = 121;

	simu_track_table[13].lat = 465183232;
	simu_track_table[13].lon = 65661784;
	simu_track_table[13].alt = 409;
	simu_track_table[13].relative_alt = -9;
	simu_track_table[13].vx = 133;
	simu_track_table[13].vy = -223;
	simu_track_table[13].vz = -46;
	simu_track_table[13].hdg = 121;

	simu_track_table[14].lat = 465183253;
	simu_track_table[14].lon = 65661715;
	simu_track_table[14].alt = 409;
	simu_track_table[14].relative_alt = -9;
	simu_track_table[14].vx = 134;
	simu_track_table[14].vy = -225;
	simu_track_table[14].vz = -46;
	simu_track_table[14].hdg = 121;

	simu_track_table[15].lat = 465183272;
	simu_track_table[15].lon = 65661647;
	simu_track_table[15].alt = 409;
	simu_track_table[15].relative_alt = -9;
	simu_track_table[15].vx = 134;
	simu_track_table[15].vy = -221;
	simu_track_table[15].vz = -47;
	simu_track_table[15].hdg = 121;

	simu_track_table[16].lat = 465183293;
	simu_track_table[16].lon = 65661579;
	simu_track_table[16].alt = 409;
	simu_track_table[16].relative_alt = -9;
	simu_track_table[16].vx = 134;
	simu_track_table[16].vy = -216;
	simu_track_table[16].vz = -47;
	simu_track_table[16].hdg = 122;

	simu_track_table[17].lat = 465183313;
	simu_track_table[17].lon = 65661515;
	simu_track_table[17].alt = 409;
	simu_track_table[17].relative_alt = -9;
	simu_track_table[17].vx = 133;
	simu_track_table[17].vy = -209;
	simu_track_table[17].vz = -48;
	simu_track_table[17].hdg = 122;

	simu_track_table[18].lat = 465183333;
	simu_track_table[18].lon = 65661450;
	simu_track_table[18].alt = 409;
	simu_track_table[18].relative_alt = -9;
	simu_track_table[18].vx = 132;
	simu_track_table[18].vy = -205;
	simu_track_table[18].vz = -47;
	simu_track_table[18].hdg = 123;

	simu_track_table[19].lat = 465183353;
	simu_track_table[19].lon = 65661387;
	simu_track_table[19].alt = 410;
	simu_track_table[19].relative_alt = -10;
	simu_track_table[19].vx = 131;
	simu_track_table[19].vy = -201;
	simu_track_table[19].vz = -47;
	simu_track_table[19].hdg = 123;

	simu_track_table[20].lat = 465183373;
	simu_track_table[20].lon = 65661324;
	simu_track_table[20].alt = 410;
	simu_track_table[20].relative_alt = -10;
	simu_track_table[20].vx = 131;
	simu_track_table[20].vy = -200;
	simu_track_table[20].vz = -47;
	simu_track_table[20].hdg = 123;

	simu_track_table[21].lat = 465183392;
	simu_track_table[21].lon = 65661264;
	simu_track_table[21].alt = 410;
	simu_track_table[21].relative_alt = -10;
	simu_track_table[21].vx = 130;
	simu_track_table[21].vy = -196;
	simu_track_table[21].vz = -47;
	simu_track_table[21].hdg = 124;

	simu_track_table[22].lat = 465183412;
	simu_track_table[22].lon = 65661202;
	simu_track_table[22].alt = 410;
	simu_track_table[22].relative_alt = -10;
	simu_track_table[22].vx = 131;
	simu_track_table[22].vy = -195;
	simu_track_table[22].vz = -47;
	simu_track_table[22].hdg = 124;

	simu_track_table[23].lat = 465183433;
	simu_track_table[23].lon = 65661138;
	simu_track_table[23].alt = 410;
	simu_track_table[23].relative_alt = -10;
	simu_track_table[23].vx = 131;
	simu_track_table[23].vy = -193;
	simu_track_table[23].vz = -47;
	simu_track_table[23].hdg = 124;

	simu_track_table[24].lat = 465183454;
	simu_track_table[24].lon = 65661078;
	simu_track_table[24].alt = 410;
	simu_track_table[24].relative_alt = -10;
	simu_track_table[24].vx = 131;
	simu_track_table[24].vy = -190;
	simu_track_table[24].vz = -47;
	simu_track_table[24].hdg = 125;

	simu_track_table[25].lat = 465183475;
	simu_track_table[25].lon = 65661016;
	simu_track_table[25].alt = 410;
	simu_track_table[25].relative_alt = -10;
	simu_track_table[25].vx = 131;
	simu_track_table[25].vy = -187;
	simu_track_table[25].vz = -47;
	simu_track_table[25].hdg = 125;

	simu_track_table[26].lat = 465183495;
	simu_track_table[26].lon = 65660958;
	simu_track_table[26].alt = 410;
	simu_track_table[26].relative_alt = -10;
	simu_track_table[26].vx = 131;
	simu_track_table[26].vy = -184;
	simu_track_table[26].vz = -47;
	simu_track_table[26].hdg = 125;

	simu_track_table[27].lat = 465183516;
	simu_track_table[27].lon = 65660899;
	simu_track_table[27].alt = 411;
	simu_track_table[27].relative_alt = -11;
	simu_track_table[27].vx = 130;
	simu_track_table[27].vy = -180;
	simu_track_table[27].vz = -47;
	simu_track_table[27].hdg = 126;

	simu_track_table[28].lat = 465183537;
	simu_track_table[28].lon = 65660843;
	simu_track_table[28].alt = 411;
	simu_track_table[28].relative_alt = -11;
	simu_track_table[28].vx = 129;
	simu_track_table[28].vy = -177;
	simu_track_table[28].vz = -47;
	simu_track_table[28].hdg = 126;

	simu_track_table[29].lat = 465183557;
	simu_track_table[29].lon = 65660787;
	simu_track_table[29].alt = 411;
	simu_track_table[29].relative_alt = -11;
	simu_track_table[29].vx = 128;
	simu_track_table[29].vy = -174;
	simu_track_table[29].vz = -47;
	simu_track_table[29].hdg = 126;

	simu_track_table[30].lat = 465183578;
	simu_track_table[30].lon = 65660733;
	simu_track_table[30].alt = 411;
	simu_track_table[30].relative_alt = -11;
	simu_track_table[30].vx = 128;
	simu_track_table[30].vy = -172;
	simu_track_table[30].vz = -47;
	simu_track_table[30].hdg = 127;

	simu_track_table[31].lat = 465183598;
	simu_track_table[31].lon = 65660678;
	simu_track_table[31].alt = 411;
	simu_track_table[31].relative_alt = -11;
	simu_track_table[31].vx = 127;
	simu_track_table[31].vy = -169;
	simu_track_table[31].vz = -47;
	simu_track_table[31].hdg = 127;

	simu_track_table[32].lat = 465183619;
	simu_track_table[32].lon = 65660626;
	simu_track_table[32].alt = 411;
	simu_track_table[32].relative_alt = -11;
	simu_track_table[32].vx = 126;
	simu_track_table[32].vy = -167;
	simu_track_table[32].vz = -47;
	simu_track_table[32].hdg = 127;

	simu_track_table[33].lat = 465183639;
	simu_track_table[33].lon = 65660573;
	simu_track_table[33].alt = 411;
	simu_track_table[33].relative_alt = -11;
	simu_track_table[33].vx = 126;
	simu_track_table[33].vy = -164;
	simu_track_table[33].vz = -47;
	simu_track_table[33].hdg = 127;

	simu_track_table[34].lat = 465183660;
	simu_track_table[34].lon = 65660522;
	simu_track_table[34].alt = 411;
	simu_track_table[34].relative_alt = -11;
	simu_track_table[34].vx = 125;
	simu_track_table[34].vy = -162;
	simu_track_table[34].vz = -47;
	simu_track_table[34].hdg = 128;

	simu_track_table[35].lat = 465183680;
	simu_track_table[35].lon = 65660472;
	simu_track_table[35].alt = 412;
	simu_track_table[35].relative_alt = -12;
	simu_track_table[35].vx = 124;
	simu_track_table[35].vy = -160;
	simu_track_table[35].vz = -47;
	simu_track_table[35].hdg = 128;

	simu_track_table[36].lat = 465183701;
	simu_track_table[36].lon = 65660422;
	simu_track_table[36].alt = 412;
	simu_track_table[36].relative_alt = -12;
	simu_track_table[36].vx = 124;
	simu_track_table[36].vy = -157;
	simu_track_table[36].vz = -47;
	simu_track_table[36].hdg = 128;

	simu_track_table[37].lat = 465183721;
	simu_track_table[37].lon = 65660373;
	simu_track_table[37].alt = 412;
	simu_track_table[37].relative_alt = -12;
	simu_track_table[37].vx = 123;
	simu_track_table[37].vy = -155;
	simu_track_table[37].vz = -47;
	simu_track_table[37].hdg = 128;

	simu_track_table[38].lat = 465183741;
	simu_track_table[38].lon = 65660324;
	simu_track_table[38].alt = 412;
	simu_track_table[38].relative_alt = -12;
	simu_track_table[38].vx = 122;
	simu_track_table[38].vy = -153;
	simu_track_table[38].vz = -47;
	simu_track_table[38].hdg = 129;

	simu_track_table[39].lat = 465183762;
	simu_track_table[39].lon = 65660277;
	simu_track_table[39].alt = 412;
	simu_track_table[39].relative_alt = -12;
	simu_track_table[39].vx = 121;
	simu_track_table[39].vy = -150;
	simu_track_table[39].vz = -47;
	simu_track_table[39].hdg = 129;

	simu_track_table[40].lat = 465183782;
	simu_track_table[40].lon = 65660230;
	simu_track_table[40].alt = 412;
	simu_track_table[40].relative_alt = -12;
	simu_track_table[40].vx = 120;
	simu_track_table[40].vy = -148;
	simu_track_table[40].vz = -47;
	simu_track_table[40].hdg = 129;

	simu_track_table[41].lat = 465183803;
	simu_track_table[41].lon = 65660183;
	simu_track_table[41].alt = 412;
	simu_track_table[41].relative_alt = -12;
	simu_track_table[41].vx = 119;
	simu_track_table[41].vy = -146;
	simu_track_table[41].vz = -47;
	simu_track_table[41].hdg = 129;

	simu_track_table[42].lat = 465183823;
	simu_track_table[42].lon = 65660138;
	simu_track_table[42].alt = 412;
	simu_track_table[42].relative_alt = -12;
	simu_track_table[42].vx = 118;
	simu_track_table[42].vy = -144;
	simu_track_table[42].vz = -47;
	simu_track_table[42].hdg = 129;

	simu_track_table[43].lat = 465183843;
	simu_track_table[43].lon = 65660093;
	simu_track_table[43].alt = 412;
	simu_track_table[43].relative_alt = -12;
	simu_track_table[43].vx = 117;
	simu_track_table[43].vy = -141;
	simu_track_table[43].vz = -47;
	simu_track_table[43].hdg = 130;

	simu_track_table[44].lat = 465183864;
	simu_track_table[44].lon = 65660048;
	simu_track_table[44].alt = 413;
	simu_track_table[44].relative_alt = -13;
	simu_track_table[44].vx = 116;
	simu_track_table[44].vy = -139;
	simu_track_table[44].vz = -47;
	simu_track_table[44].hdg = 130;

	simu_track_table[45].lat = 465183884;
	simu_track_table[45].lon = 65660005;
	simu_track_table[45].alt = 413;
	simu_track_table[45].relative_alt = -13;
	simu_track_table[45].vx = 115;
	simu_track_table[45].vy = -137;
	simu_track_table[45].vz = -47;
	simu_track_table[45].hdg = 130;

	simu_track_table[46].lat = 465183904;
	simu_track_table[46].lon = 65659962;
	simu_track_table[46].alt = 413;
	simu_track_table[46].relative_alt = -13;
	simu_track_table[46].vx = 114;
	simu_track_table[46].vy = -135;
	simu_track_table[46].vz = -47;
	simu_track_table[46].hdg = 130;

	simu_track_table[47].lat = 465183924;
	simu_track_table[47].lon = 65659919;
	simu_track_table[47].alt = 413;
	simu_track_table[47].relative_alt = -13;
	simu_track_table[47].vx = 113;
	simu_track_table[47].vy = -132;
	simu_track_table[47].vz = -47;
	simu_track_table[47].hdg = 130;

	simu_track_table[48].lat = 465183944;
	simu_track_table[48].lon = 65659878;
	simu_track_table[48].alt = 413;
	simu_track_table[48].relative_alt = -13;
	simu_track_table[48].vx = 112;
	simu_track_table[48].vy = -130;
	simu_track_table[48].vz = -47;
	simu_track_table[48].hdg = 131;

	simu_track_table[49].lat = 465183964;
	simu_track_table[49].lon = 65659837;
	simu_track_table[49].alt = 413;
	simu_track_table[49].relative_alt = -13;
	simu_track_table[49].vx = 110;
	simu_track_table[49].vy = -128;
	simu_track_table[49].vz = -47;
	simu_track_table[49].hdg = 131;

	simu_track_table[50].lat = 465183984;
	simu_track_table[50].lon = 65659797;
	simu_track_table[50].alt = 413;
	simu_track_table[50].relative_alt = -13;
	simu_track_table[50].vx = 109;
	simu_track_table[50].vy = -125;
	simu_track_table[50].vz = -47;
	simu_track_table[50].hdg = 131;

	simu_track_table[51].lat = 465184003;
	simu_track_table[51].lon = 65659758;
	simu_track_table[51].alt = 413;
	simu_track_table[51].relative_alt = -13;
	simu_track_table[51].vx = 108;
	simu_track_table[51].vy = -123;
	simu_track_table[51].vz = -47;
	simu_track_table[51].hdg = 131;

	simu_track_table[52].lat = 465184023;
	simu_track_table[52].lon = 65659719;
	simu_track_table[52].alt = 414;
	simu_track_table[52].relative_alt = -14;
	simu_track_table[52].vx = 106;
	simu_track_table[52].vy = -121;
	simu_track_table[52].vz = -47;
	simu_track_table[52].hdg = 131;

	simu_track_table[53].lat = 465184042;
	simu_track_table[53].lon = 65659681;
	simu_track_table[53].alt = 414;
	simu_track_table[53].relative_alt = -14;
	simu_track_table[53].vx = 105;
	simu_track_table[53].vy = -118;
	simu_track_table[53].vz = -47;
	simu_track_table[53].hdg = 132;

	simu_track_table[54].lat = 465184061;
	simu_track_table[54].lon = 65659644;
	simu_track_table[54].alt = 414;
	simu_track_table[54].relative_alt = -14;
	simu_track_table[54].vx = 104;
	simu_track_table[54].vy = -116;
	simu_track_table[54].vz = -47;
	simu_track_table[54].hdg = 132;

	simu_track_table[55].lat = 465184080;
	simu_track_table[55].lon = 65659607;
	simu_track_table[55].alt = 414;
	simu_track_table[55].relative_alt = -14;
	simu_track_table[55].vx = 102;
	simu_track_table[55].vy = -114;
	simu_track_table[55].vz = -47;
	simu_track_table[55].hdg = 132;

	simu_track_table[56].lat = 465184099;
	simu_track_table[56].lon = 65659572;
	simu_track_table[56].alt = 414;
	simu_track_table[56].relative_alt = -14;
	simu_track_table[56].vx = 101;
	simu_track_table[56].vy = -111;
	simu_track_table[56].vz = -47;
	simu_track_table[56].hdg = 132;

	simu_track_table[57].lat = 465184117;
	simu_track_table[57].lon = 65659537;
	simu_track_table[57].alt = 414;
	simu_track_table[57].relative_alt = -14;
	simu_track_table[57].vx = 99;
	simu_track_table[57].vy = -109;
	simu_track_table[57].vz = -47;
	simu_track_table[57].hdg = 132;

	simu_track_table[58].lat = 465184136;
	simu_track_table[58].lon = 65659503;
	simu_track_table[58].alt = 414;
	simu_track_table[58].relative_alt = -14;
	simu_track_table[58].vx = 98;
	simu_track_table[58].vy = -106;
	simu_track_table[58].vz = -47;
	simu_track_table[58].hdg = 133;

	simu_track_table[59].lat = 465184154;
	simu_track_table[59].lon = 65659469;
	simu_track_table[59].alt = 414;
	simu_track_table[59].relative_alt = -14;
	simu_track_table[59].vx = 96;
	simu_track_table[59].vy = -104;
	simu_track_table[59].vz = -47;
	simu_track_table[59].hdg = 133;

	simu_track_table[60].lat = 465184172;
	simu_track_table[60].lon = 65659436;
	simu_track_table[60].alt = 414;
	simu_track_table[60].relative_alt = -14;
	simu_track_table[60].vx = 95;
	simu_track_table[60].vy = -102;
	simu_track_table[60].vz = -47;
	simu_track_table[60].hdg = 133;

	simu_track_table[61].lat = 465184190;
	simu_track_table[61].lon = 65659404;
	simu_track_table[61].alt = 415;
	simu_track_table[61].relative_alt = -15;
	simu_track_table[61].vx = 93;
	simu_track_table[61].vy = -99;
	simu_track_table[61].vz = -47;
	simu_track_table[61].hdg = 133;

	simu_track_table[62].lat = 465184207;
	simu_track_table[62].lon = 65659373;
	simu_track_table[62].alt = 415;
	simu_track_table[62].relative_alt = -15;
	simu_track_table[62].vx = 91;
	simu_track_table[62].vy = -97;
	simu_track_table[62].vz = -47;
	simu_track_table[62].hdg = 133;

	simu_track_table[63].lat = 465184225;
	simu_track_table[63].lon = 65659343;
	simu_track_table[63].alt = 415;
	simu_track_table[63].relative_alt = -15;
	simu_track_table[63].vx = 90;
	simu_track_table[63].vy = -95;
	simu_track_table[63].vz = -47;
	simu_track_table[63].hdg = 133;

	simu_track_table[64].lat = 465184242;
	simu_track_table[64].lon = 65659313;
	simu_track_table[64].alt = 415;
	simu_track_table[64].relative_alt = -15;
	simu_track_table[64].vx = 88;
	simu_track_table[64].vy = -92;
	simu_track_table[64].vz = -47;
	simu_track_table[64].hdg = 134;

	simu_track_table[65].lat = 465184259;
	simu_track_table[65].lon = 65659284;
	simu_track_table[65].alt = 415;
	simu_track_table[65].relative_alt = -15;
	simu_track_table[65].vx = 86;
	simu_track_table[65].vy = -90;
	simu_track_table[65].vz = -47;
	simu_track_table[65].hdg = 134;

	simu_track_table[66].lat = 465184275;
	simu_track_table[66].lon = 65659256;
	simu_track_table[66].alt = 415;
	simu_track_table[66].relative_alt = -15;
	simu_track_table[66].vx = 84;
	simu_track_table[66].vy = -88;
	simu_track_table[66].vz = -47;
	simu_track_table[66].hdg = 134;

	simu_track_table[67].lat = 465184292;
	simu_track_table[67].lon = 65659228;
	simu_track_table[67].alt = 415;
	simu_track_table[67].relative_alt = -15;
	simu_track_table[67].vx = 83;
	simu_track_table[67].vy = -85;
	simu_track_table[67].vz = -47;
	simu_track_table[67].hdg = 134;

	simu_track_table[68].lat = 465184308;
	simu_track_table[68].lon = 65659202;
	simu_track_table[68].alt = 415;
	simu_track_table[68].relative_alt = -15;
	simu_track_table[68].vx = 81;
	simu_track_table[68].vy = -83;
	simu_track_table[68].vz = -47;
	simu_track_table[68].hdg = 134;

	simu_track_table[69].lat = 465184324;
	simu_track_table[69].lon = 65659176;
	simu_track_table[69].alt = 415;
	simu_track_table[69].relative_alt = -15;
	simu_track_table[69].vx = 79;
	simu_track_table[69].vy = -81;
	simu_track_table[69].vz = -47;
	simu_track_table[69].hdg = 134;

	simu_track_table[70].lat = 465184339;
	simu_track_table[70].lon = 65659150;
	simu_track_table[70].alt = 416;
	simu_track_table[70].relative_alt = -16;
	simu_track_table[70].vx = 77;
	simu_track_table[70].vy = -78;
	simu_track_table[70].vz = -47;
	simu_track_table[70].hdg = 135;

	simu_track_table[71].lat = 465184354;
	simu_track_table[71].lon = 65659126;
	simu_track_table[71].alt = 416;
	simu_track_table[71].relative_alt = -16;
	simu_track_table[71].vx = 75;
	simu_track_table[71].vy = -76;
	simu_track_table[71].vz = -47;
	simu_track_table[71].hdg = 135;

	simu_track_table[72].lat = 465184369;
	simu_track_table[72].lon = 65659102;
	simu_track_table[72].alt = 416;
	simu_track_table[72].relative_alt = -16;
	simu_track_table[72].vx = 73;
	simu_track_table[72].vy = -74;
	simu_track_table[72].vz = -46;
	simu_track_table[72].hdg = 135;

	simu_track_table[73].lat = 465184383;
	simu_track_table[73].lon = 65659079;
	simu_track_table[73].alt = 416;
	simu_track_table[73].relative_alt = -16;
	simu_track_table[73].vx = 71;
	simu_track_table[73].vy = -71;
	simu_track_table[73].vz = -46;
	simu_track_table[73].hdg = 135;

	simu_track_table[74].lat = 465184397;
	simu_track_table[74].lon = 65659057;
	simu_track_table[74].alt = 416;
	simu_track_table[74].relative_alt = -16;
	simu_track_table[74].vx = 69;
	simu_track_table[74].vy = -69;
	simu_track_table[74].vz = -46;
	simu_track_table[74].hdg = 135;

	simu_track_table[75].lat = 465184411;
	simu_track_table[75].lon = 65659036;
	simu_track_table[75].alt = 416;
	simu_track_table[75].relative_alt = -16;
	simu_track_table[75].vx = 67;
	simu_track_table[75].vy = -67;
	simu_track_table[75].vz = -46;
	simu_track_table[75].hdg = 135;

	simu_track_table[76].lat = 465184425;
	simu_track_table[76].lon = 65659015;
	simu_track_table[76].alt = 416;
	simu_track_table[76].relative_alt = -16;
	simu_track_table[76].vx = 65;
	simu_track_table[76].vy = -65;
	simu_track_table[76].vz = -43;
	simu_track_table[76].hdg = 135;

	simu_track_table[77].lat = 465184438;
	simu_track_table[77].lon = 65658995;
	simu_track_table[77].alt = 416;
	simu_track_table[77].relative_alt = -16;
	simu_track_table[77].vx = 62;
	simu_track_table[77].vy = -58;
	simu_track_table[77].vz = 2;
	simu_track_table[77].hdg = 137;

	simu_track_table[78].lat = 465184450;
	simu_track_table[78].lon = 65658981;
	simu_track_table[78].alt = 416;
	simu_track_table[78].relative_alt = -16;
	simu_track_table[78].vx = 55;
	simu_track_table[78].vy = -29;
	simu_track_table[78].vz = 18;
	simu_track_table[78].hdg = 152;

	simu_track_table[79].lat = 465184460;
	simu_track_table[79].lon = 65658980;
	simu_track_table[79].alt = 416;
	simu_track_table[79].relative_alt = -16;
	simu_track_table[79].vx = 45;
	simu_track_table[79].vy = 20;
	simu_track_table[79].vz = 25;
	simu_track_table[79].hdg = 204;

	simu_track_table[80].lat = 465184468;
	simu_track_table[80].lon = 65658997;
	simu_track_table[80].alt = 416;
	simu_track_table[80].relative_alt = -16;
	simu_track_table[80].vx = 39;
	simu_track_table[80].vy = 76;
	simu_track_table[80].vz = 26;
	simu_track_table[80].hdg = 243;

	simu_track_table[81].lat = 465184476;
	simu_track_table[81].lon = 65659033;
	simu_track_table[81].alt = 416;
	simu_track_table[81].relative_alt = -16;
	simu_track_table[81].vx = 46;
	simu_track_table[81].vy = 130;
	simu_track_table[81].vz = 19;
	simu_track_table[81].hdg = 251;

	simu_track_table[82].lat = 465184489;
	simu_track_table[82].lon = 65659087;
	simu_track_table[82].alt = 416;
	simu_track_table[82].relative_alt = -16;
	simu_track_table[82].vx = 77;
	simu_track_table[82].vy = 184;
	simu_track_table[82].vz = 18;
	simu_track_table[82].hdg = 247;

	simu_track_table[83].lat = 465184510;
	simu_track_table[83].lon = 65659159;
	simu_track_table[83].alt = 416;
	simu_track_table[83].relative_alt = -16;
	simu_track_table[83].vx = 125;
	simu_track_table[83].vy = 231;
	simu_track_table[83].vz = 20;
	simu_track_table[83].hdg = 242;

	simu_track_table[84].lat = 465184543;
	simu_track_table[84].lon = 65659245;
	simu_track_table[84].alt = 416;
	simu_track_table[84].relative_alt = -16;
	simu_track_table[84].vx = 168;
	simu_track_table[84].vy = 261;
	simu_track_table[84].vz = 18;
	simu_track_table[84].hdg = 237;

	simu_track_table[85].lat = 465184584;
	simu_track_table[85].lon = 65659339;
	simu_track_table[85].alt = 416;
	simu_track_table[85].relative_alt = -16;
	simu_track_table[85].vx = 194;
	simu_track_table[85].vy = 271;
	simu_track_table[85].vz = 11;
	simu_track_table[85].hdg = 234;

	simu_track_table[86].lat = 465184629;
	simu_track_table[86].lon = 65659433;
	simu_track_table[86].alt = 416;
	simu_track_table[86].relative_alt = -16;
	simu_track_table[86].vx = 202;
	simu_track_table[86].vy = 266;
	simu_track_table[86].vz = 3;
	simu_track_table[86].hdg = 233;

	simu_track_table[87].lat = 465184675;
	simu_track_table[87].lon = 65659526;
	simu_track_table[87].alt = 416;
	simu_track_table[87].relative_alt = -16;
	simu_track_table[87].vx = 197;
	simu_track_table[87].vy = 256;
	simu_track_table[87].vz = -2;
	simu_track_table[87].hdg = 232;

	simu_track_table[88].lat = 465184719;
	simu_track_table[88].lon = 65659616;
	simu_track_table[88].alt = 416;
	simu_track_table[88].relative_alt = -16;
	simu_track_table[88].vx = 188;
	simu_track_table[88].vy = 248;
	simu_track_table[88].vz = -5;
	simu_track_table[88].hdg = 233;

	simu_track_table[89].lat = 465184762;
	simu_track_table[89].lon = 65659705;
	simu_track_table[89].alt = 416;
	simu_track_table[89].relative_alt = -16;
	simu_track_table[89].vx = 182;
	simu_track_table[89].vy = 245;
	simu_track_table[89].vz = -6;
	simu_track_table[89].hdg = 233;

	simu_track_table[90].lat = 465184805;
	simu_track_table[90].lon = 65659793;
	simu_track_table[90].alt = 416;
	simu_track_table[90].relative_alt = -16;
	simu_track_table[90].vx = 180;
	simu_track_table[90].vy = 246;
	simu_track_table[90].vz = -6;
	simu_track_table[90].hdg = 234;

	simu_track_table[91].lat = 465184847;
	simu_track_table[91].lon = 65659882;
	simu_track_table[91].alt = 416;
	simu_track_table[91].relative_alt = -16;
	simu_track_table[91].vx = 182;
	simu_track_table[91].vy = 248;
	simu_track_table[91].vz = -6;
	simu_track_table[91].hdg = 234;

	simu_track_table[92].lat = 465184890;
	simu_track_table[92].lon = 65659972;
	simu_track_table[92].alt = 416;
	simu_track_table[92].relative_alt = -16;
	simu_track_table[92].vx = 185;
	simu_track_table[92].vy = 249;
	simu_track_table[92].vz = -7;
	simu_track_table[92].hdg = 233;

	simu_track_table[93].lat = 465184933;
	simu_track_table[93].lon = 65660061;
	simu_track_table[93].alt = 416;
	simu_track_table[93].relative_alt = -16;
	simu_track_table[93].vx = 187;
	simu_track_table[93].vy = 248;
	simu_track_table[93].vz = -8;
	simu_track_table[93].hdg = 233;

	simu_track_table[94].lat = 465184977;
	simu_track_table[94].lon = 65660150;
	simu_track_table[94].alt = 416;
	simu_track_table[94].relative_alt = -16;
	simu_track_table[94].vx = 187;
	simu_track_table[94].vy = 246;
	simu_track_table[94].vz = -10;
	simu_track_table[94].hdg = 233;

	simu_track_table[95].lat = 465185020;
	simu_track_table[95].lon = 65660239;
	simu_track_table[95].alt = 416;
	simu_track_table[95].relative_alt = -16;
	simu_track_table[95].vx = 186;
	simu_track_table[95].vy = 243;
	simu_track_table[95].vz = -12;
	simu_track_table[95].hdg = 233;

	simu_track_table[96].lat = 465185064;
	simu_track_table[96].lon = 65660326;
	simu_track_table[96].alt = 416;
	simu_track_table[96].relative_alt = -16;
	simu_track_table[96].vx = 184;
	simu_track_table[96].vy = 241;
	simu_track_table[96].vz = -13;
	simu_track_table[96].hdg = 233;

	simu_track_table[97].lat = 465185106;
	simu_track_table[97].lon = 65660413;
	simu_track_table[97].alt = 416;
	simu_track_table[97].relative_alt = -16;
	simu_track_table[97].vx = 183;
	simu_track_table[97].vy = 239;
	simu_track_table[97].vz = -15;
	simu_track_table[97].hdg = 233;

	simu_track_table[98].lat = 465185149;
	simu_track_table[98].lon = 65660500;
	simu_track_table[98].alt = 416;
	simu_track_table[98].relative_alt = -16;
	simu_track_table[98].vx = 182;
	simu_track_table[98].vy = 237;
	simu_track_table[98].vz = -16;
	simu_track_table[98].hdg = 233;

	simu_track_table[99].lat = 465185191;
	simu_track_table[99].lon = 65660586;
	simu_track_table[99].alt = 416;
	simu_track_table[99].relative_alt = -16;
	simu_track_table[99].vx = 181;
	simu_track_table[99].vy = 237;
	simu_track_table[99].vz = -18;
	simu_track_table[99].hdg = 233;

	simu_track_table[100].lat = 465185233;
	simu_track_table[100].lon = 65660671;
	simu_track_table[100].alt = 416;
	simu_track_table[100].relative_alt = -16;
	simu_track_table[100].vx = 181;
	simu_track_table[100].vy = 236;
	simu_track_table[100].vz = -19;
	simu_track_table[100].hdg = 232;

	simu_track_table[101].lat = 465185275;
	simu_track_table[101].lon = 65660757;
	simu_track_table[101].alt = 416;
	simu_track_table[101].relative_alt = -16;
	simu_track_table[101].vx = 181;
	simu_track_table[101].vy = 235;
	simu_track_table[101].vz = -21;
	simu_track_table[101].hdg = 232;

	simu_track_table[102].lat = 465185317;
	simu_track_table[102].lon = 65660842;
	simu_track_table[102].alt = 416;
	simu_track_table[102].relative_alt = -16;
	simu_track_table[102].vx = 181;
	simu_track_table[102].vy = 234;
	simu_track_table[102].vz = -23;
	simu_track_table[102].hdg = 232;

	simu_track_table[103].lat = 465185359;
	simu_track_table[103].lon = 65660927;
	simu_track_table[103].alt = 416;
	simu_track_table[103].relative_alt = -16;
	simu_track_table[103].vx = 181;
	simu_track_table[103].vy = 233;
	simu_track_table[103].vz = -25;
	simu_track_table[103].hdg = 232;

	simu_track_table[104].lat = 465185401;
	simu_track_table[104].lon = 65661012;
	simu_track_table[104].alt = 416;
	simu_track_table[104].relative_alt = -16;
	simu_track_table[104].vx = 180;
	simu_track_table[104].vy = 231;
	simu_track_table[104].vz = -27;
	simu_track_table[104].hdg = 232;

	simu_track_table[105].lat = 465185443;
	simu_track_table[105].lon = 65661096;
	simu_track_table[105].alt = 417;
	simu_track_table[105].relative_alt = -17;
	simu_track_table[105].vx = 180;
	simu_track_table[105].vy = 230;
	simu_track_table[105].vz = -29;
	simu_track_table[105].hdg = 232;

	simu_track_table[106].lat = 465185484;
	simu_track_table[106].lon = 65661179;
	simu_track_table[106].alt = 417;
	simu_track_table[106].relative_alt = -17;
	simu_track_table[106].vx = 180;
	simu_track_table[106].vy = 229;
	simu_track_table[106].vz = -31;
	simu_track_table[106].hdg = 232;

	simu_track_table[107].lat = 465185526;
	simu_track_table[107].lon = 65661263;
	simu_track_table[107].alt = 417;
	simu_track_table[107].relative_alt = -17;
	simu_track_table[107].vx = 180;
	simu_track_table[107].vy = 228;
	simu_track_table[107].vz = -34;
	simu_track_table[107].hdg = 232;

	simu_track_table[108].lat = 465185567;
	simu_track_table[108].lon = 65661346;
	simu_track_table[108].alt = 417;
	simu_track_table[108].relative_alt = -17;
	simu_track_table[108].vx = 180;
	simu_track_table[108].vy = 227;
	simu_track_table[108].vz = -36;
	simu_track_table[108].hdg = 232;

	simu_track_table[109].lat = 465185609;
	simu_track_table[109].lon = 65661428;
	simu_track_table[109].alt = 417;
	simu_track_table[109].relative_alt = -17;
	simu_track_table[109].vx = 180;
	simu_track_table[109].vy = 225;
	simu_track_table[109].vz = -39;
	simu_track_table[109].hdg = 231;

	simu_track_table[110].lat = 465185650;
	simu_track_table[110].lon = 65661510;
	simu_track_table[110].alt = 417;
	simu_track_table[110].relative_alt = -17;
	simu_track_table[110].vx = 180;
	simu_track_table[110].vy = 224;
	simu_track_table[110].vz = -42;
	simu_track_table[110].hdg = 231;

	simu_track_table[111].lat = 465185691;
	simu_track_table[111].lon = 65661592;
	simu_track_table[111].alt = 417;
	simu_track_table[111].relative_alt = -17;
	simu_track_table[111].vx = 180;
	simu_track_table[111].vy = 222;
	simu_track_table[111].vz = -46;
	simu_track_table[111].hdg = 231;

	simu_track_table[112].lat = 465185732;
	simu_track_table[112].lon = 65661672;
	simu_track_table[112].alt = 417;
	simu_track_table[112].relative_alt = -17;
	simu_track_table[112].vx = 179;
	simu_track_table[112].vy = 219;
	simu_track_table[112].vz = -47;
	simu_track_table[112].hdg = 231;

	simu_track_table[113].lat = 465185773;
	simu_track_table[113].lon = 65661752;
	simu_track_table[113].alt = 417;
	simu_track_table[113].relative_alt = -17;
	simu_track_table[113].vx = 177;
	simu_track_table[113].vy = 215;
	simu_track_table[113].vz = -48;
	simu_track_table[113].hdg = 230;

	simu_track_table[114].lat = 465185813;
	simu_track_table[114].lon = 65661829;
	simu_track_table[114].alt = 417;
	simu_track_table[114].relative_alt = -17;
	simu_track_table[114].vx = 172;
	simu_track_table[114].vy = 206;
	simu_track_table[114].vz = -49;
	simu_track_table[114].hdg = 230;

	simu_track_table[115].lat = 465185852;
	simu_track_table[115].lon = 65661903;
	simu_track_table[115].alt = 418;
	simu_track_table[115].relative_alt = -18;
	simu_track_table[115].vx = 163;
	simu_track_table[115].vy = 193;
	simu_track_table[115].vz = -50;
	simu_track_table[115].hdg = 230;

	simu_track_table[116].lat = 465185888;
	simu_track_table[116].lon = 65661972;
	simu_track_table[116].alt = 418;
	simu_track_table[116].relative_alt = -18;
	simu_track_table[116].vx = 152;
	simu_track_table[116].vy = 177;
	simu_track_table[116].vz = -50;
	simu_track_table[116].hdg = 229;

	simu_track_table[117].lat = 465185922;
	simu_track_table[117].lon = 65662036;
	simu_track_table[117].alt = 418;
	simu_track_table[117].relative_alt = -18;
	simu_track_table[117].vx = 140;
	simu_track_table[117].vy = 164;
	simu_track_table[117].vz = 7;
	simu_track_table[117].hdg = 230;

	simu_track_table[118].lat = 465185950;
	simu_track_table[118].lon = 65662094;
	simu_track_table[118].alt = 418;
	simu_track_table[118].relative_alt = -18;
	simu_track_table[118].vx = 101;
	simu_track_table[118].vy = 145;
	simu_track_table[118].vz = 41;
	simu_track_table[118].hdg = 235;

	simu_track_table[119].lat = 465185965;
	simu_track_table[119].lon = 65662145;
	simu_track_table[119].alt = 418;
	simu_track_table[119].relative_alt = -18;
	simu_track_table[119].vx = 28;
	simu_track_table[119].vy = 124;
	simu_track_table[119].vz = 60;
	simu_track_table[119].hdg = 257;

	simu_track_table[120].lat = 465185961;
	simu_track_table[120].lon = 65662191;
	simu_track_table[120].alt = 417;
	simu_track_table[120].relative_alt = -17;
	simu_track_table[120].vx = -59;
	simu_track_table[120].vy = 116;
	simu_track_table[120].vz = 85;
	simu_track_table[120].hdg = 297;

	simu_track_table[121].lat = 465185937;
	simu_track_table[121].lon = 65662238;
	simu_track_table[121].alt = 417;
	simu_track_table[121].relative_alt = -17;
	simu_track_table[121].vx = -141;
	simu_track_table[121].vy = 141;
	simu_track_table[121].vz = 98;
	simu_track_table[121].hdg = 315;

	simu_track_table[122].lat = 465185897;
	simu_track_table[122].lon = 65662299;
	simu_track_table[122].alt = 417;
	simu_track_table[122].relative_alt = -17;
	simu_track_table[122].vx = -192;
	simu_track_table[122].vy = 198;
	simu_track_table[122].vz = 104;
	simu_track_table[122].hdg = 314;

	simu_track_table[123].lat = 465185849;
	simu_track_table[123].lon = 65662381;
	simu_track_table[123].alt = 417;
	simu_track_table[123].relative_alt = -17;
	simu_track_table[123].vx = -198;
	simu_track_table[123].vy = 265;
	simu_track_table[123].vz = 104;
	simu_track_table[123].hdg = 307;

	simu_track_table[124].lat = 465185803;
	simu_track_table[124].lon = 65662483;
	simu_track_table[124].alt = 416;
	simu_track_table[124].relative_alt = -16;
	simu_track_table[124].vx = -172;
	simu_track_table[124].vy = 314;
	simu_track_table[124].vz = 98;
	simu_track_table[124].hdg = 299;

	simu_track_table[125].lat = 465185764;
	simu_track_table[125].lon = 65662597;
	simu_track_table[125].alt = 416;
	simu_track_table[125].relative_alt = -16;
	simu_track_table[125].vx = -132;
	simu_track_table[125].vy = 330;
	simu_track_table[125].vz = 90;
	simu_track_table[125].hdg = 292;

	simu_track_table[126].lat = 465185733;
	simu_track_table[126].lon = 65662712;
	simu_track_table[126].alt = 416;
	simu_track_table[126].relative_alt = -16;
	simu_track_table[126].vx = -99;
	simu_track_table[126].vy = 318;
	simu_track_table[126].vz = 82;
	simu_track_table[126].hdg = 287;

	simu_track_table[127].lat = 465185708;
	simu_track_table[127].lon = 65662822;
	simu_track_table[127].alt = 416;
	simu_track_table[127].relative_alt = -16;
	simu_track_table[127].vx = -84;
	simu_track_table[127].vy = 294;
	simu_track_table[127].vz = 78;
	simu_track_table[127].hdg = 286;

	simu_track_table[128].lat = 465185684;
	simu_track_table[128].lon = 65662923;
	simu_track_table[128].alt = 416;
	simu_track_table[128].relative_alt = -16;
	simu_track_table[128].vx = -87;
	simu_track_table[128].vy = 270;
	simu_track_table[128].vz = 78;
	simu_track_table[128].hdg = 288;

	simu_track_table[129].lat = 465185659;
	simu_track_table[129].lon = 65663019;
	simu_track_table[129].alt = 415;
	simu_track_table[129].relative_alt = -15;
	simu_track_table[129].vx = -101;
	simu_track_table[129].vy = 257;
	simu_track_table[129].vz = 79;
	simu_track_table[129].hdg = 291;

	simu_track_table[130].lat = 465185630;
	simu_track_table[130].lon = 65663112;
	simu_track_table[130].alt = 415;
	simu_track_table[130].relative_alt = -15;
	simu_track_table[130].vx = -117;
	simu_track_table[130].vy = 256;
	simu_track_table[130].vz = 81;
	simu_track_table[130].hdg = 295;

	simu_track_table[131].lat = 465185597;
	simu_track_table[131].lon = 65663205;
	simu_track_table[131].alt = 415;
	simu_track_table[131].relative_alt = -15;
	simu_track_table[131].vx = -128;
	simu_track_table[131].vy = 264;
	simu_track_table[131].vz = 83;
	simu_track_table[131].hdg = 296;

	simu_track_table[132].lat = 465185564;
	simu_track_table[132].lon = 65663302;
	simu_track_table[132].alt = 415;
	simu_track_table[132].relative_alt = -15;
	simu_track_table[132].vx = -130;
	simu_track_table[132].vy = 275;
	simu_track_table[132].vz = 83;
	simu_track_table[132].hdg = 295;

	simu_track_table[133].lat = 465185530;
	simu_track_table[133].lon = 65663402;
	simu_track_table[133].alt = 414;
	simu_track_table[133].relative_alt = -14;
	simu_track_table[133].vx = -123;
	simu_track_table[133].vy = 283;
	simu_track_table[133].vz = 82;
	simu_track_table[133].hdg = 294;

	simu_track_table[134].lat = 465185499;
	simu_track_table[134].lon = 65663503;
	simu_track_table[134].alt = 414;
	simu_track_table[134].relative_alt = -14;
	simu_track_table[134].vx = -114;
	simu_track_table[134].vy = 286;
	simu_track_table[134].vz = 80;
	simu_track_table[134].hdg = 292;

	simu_track_table[135].lat = 465185469;
	simu_track_table[135].lon = 65663605;
	simu_track_table[135].alt = 414;
	simu_track_table[135].relative_alt = -14;
	simu_track_table[135].vx = -105;
	simu_track_table[135].vy = 284;
	simu_track_table[135].vz = 79;
	simu_track_table[135].hdg = 290;

	simu_track_table[136].lat = 465185441;
	simu_track_table[136].lon = 65663705;
	simu_track_table[136].alt = 414;
	simu_track_table[136].relative_alt = -14;
	simu_track_table[136].vx = -100;
	simu_track_table[136].vy = 279;
	simu_track_table[136].vz = 78;
	simu_track_table[136].hdg = 290;

	simu_track_table[137].lat = 465185414;
	simu_track_table[137].lon = 65663804;
	simu_track_table[137].alt = 414;
	simu_track_table[137].relative_alt = -14;
	simu_track_table[137].vx = -100;
	simu_track_table[137].vy = 274;
	simu_track_table[137].vz = 78;
	simu_track_table[137].hdg = 290;

	simu_track_table[138].lat = 465185387;
	simu_track_table[138].lon = 65663900;
	simu_track_table[138].alt = 414;
	simu_track_table[138].relative_alt = -14;
	simu_track_table[138].vx = -103;
	simu_track_table[138].vy = 271;
	simu_track_table[138].vz = 78;
	simu_track_table[138].hdg = 291;

	simu_track_table[139].lat = 465185358;
	simu_track_table[139].lon = 65663996;
	simu_track_table[139].alt = 413;
	simu_track_table[139].relative_alt = -13;
	simu_track_table[139].vx = -107;
	simu_track_table[139].vy = 271;
	simu_track_table[139].vz = 78;
	simu_track_table[139].hdg = 292;

	simu_track_table[140].lat = 465185329;
	simu_track_table[140].lon = 65664092;
	simu_track_table[140].alt = 413;
	simu_track_table[140].relative_alt = -13;
	simu_track_table[140].vx = -109;
	simu_track_table[140].vy = 272;
	simu_track_table[140].vz = 78;
	simu_track_table[140].hdg = 292;

	simu_track_table[141].lat = 465185300;
	simu_track_table[141].lon = 65664188;
	simu_track_table[141].alt = 413;
	simu_track_table[141].relative_alt = -13;
	simu_track_table[141].vx = -110;
	simu_track_table[141].vy = 274;
	simu_track_table[141].vz = 77;
	simu_track_table[141].hdg = 292;

	simu_track_table[142].lat = 465185270;
	simu_track_table[142].lon = 65664285;
	simu_track_table[142].alt = 413;
	simu_track_table[142].relative_alt = -13;
	simu_track_table[142].vx = -109;
	simu_track_table[142].vy = 276;
	simu_track_table[142].vz = 76;
	simu_track_table[142].hdg = 292;

	simu_track_table[143].lat = 465185241;
	simu_track_table[143].lon = 65664382;
	simu_track_table[143].alt = 413;
	simu_track_table[143].relative_alt = -13;
	simu_track_table[143].vx = -107;
	simu_track_table[143].vy = 277;
	simu_track_table[143].vz = 76;
	simu_track_table[143].hdg = 291;

	simu_track_table[144].lat = 465185213;
	simu_track_table[144].lon = 65664479;
	simu_track_table[144].alt = 412;
	simu_track_table[144].relative_alt = -12;
	simu_track_table[144].vx = -104;
	simu_track_table[144].vy = 277;
	simu_track_table[144].vz = 75;
	simu_track_table[144].hdg = 291;

	simu_track_table[145].lat = 465185185;
	simu_track_table[145].lon = 65664576;
	simu_track_table[145].alt = 412;
	simu_track_table[145].relative_alt = -12;
	simu_track_table[145].vx = -103;
	simu_track_table[145].vy = 276;
	simu_track_table[145].vz = 74;
	simu_track_table[145].hdg = 290;

	simu_track_table[146].lat = 465185157;
	simu_track_table[146].lon = 65664672;
	simu_track_table[146].alt = 412;
	simu_track_table[146].relative_alt = -12;
	simu_track_table[146].vx = -102;
	simu_track_table[146].vy = 276;
	simu_track_table[146].vz = 73;
	simu_track_table[146].hdg = 290;

	simu_track_table[147].lat = 465185129;
	simu_track_table[147].lon = 65664768;
	simu_track_table[147].alt = 412;
	simu_track_table[147].relative_alt = -12;
	simu_track_table[147].vx = -102;
	simu_track_table[147].vy = 275;
	simu_track_table[147].vz = 72;
	simu_track_table[147].hdg = 290;

	simu_track_table[148].lat = 465185102;
	simu_track_table[148].lon = 65664863;
	simu_track_table[148].alt = 412;
	simu_track_table[148].relative_alt = -12;
	simu_track_table[148].vx = -102;
	simu_track_table[148].vy = 276;
	simu_track_table[148].vz = 71;
	simu_track_table[148].hdg = 290;

	simu_track_table[149].lat = 465185074;
	simu_track_table[149].lon = 65664959;
	simu_track_table[149].alt = 411;
	simu_track_table[149].relative_alt = -11;
	simu_track_table[149].vx = -102;
	simu_track_table[149].vy = 276;
	simu_track_table[149].vz = 70;
	simu_track_table[149].hdg = 290;

	simu_track_table[150].lat = 465185046;
	simu_track_table[150].lon = 65665054;
	simu_track_table[150].alt = 411;
	simu_track_table[150].relative_alt = -11;
	simu_track_table[150].vx = -102;
	simu_track_table[150].vy = 277;
	simu_track_table[150].vz = 69;
	simu_track_table[150].hdg = 290;

	simu_track_table[151].lat = 465185019;
	simu_track_table[151].lon = 65665150;
	simu_track_table[151].alt = 411;
	simu_track_table[151].relative_alt = -11;
	simu_track_table[151].vx = -101;
	simu_track_table[151].vy = 278;
	simu_track_table[151].vz = 68;
	simu_track_table[151].hdg = 290;

	simu_track_table[152].lat = 465184992;
	simu_track_table[152].lon = 65665246;
	simu_track_table[152].alt = 411;
	simu_track_table[152].relative_alt = -11;
	simu_track_table[152].vx = -99;
	simu_track_table[152].vy = 279;
	simu_track_table[152].vz = 66;
	simu_track_table[152].hdg = 290;

	simu_track_table[153].lat = 465184965;
	simu_track_table[153].lon = 65665342;
	simu_track_table[153].alt = 411;
	simu_track_table[153].relative_alt = -11;
	simu_track_table[153].vx = -97;
	simu_track_table[153].vy = 280;
	simu_track_table[153].vz = 64;
	simu_track_table[153].hdg = 289;

	simu_track_table[154].lat = 465184938;
	simu_track_table[154].lon = 65665438;
	simu_track_table[154].alt = 411;
	simu_track_table[154].relative_alt = -11;
	simu_track_table[154].vx = -96;
	simu_track_table[154].vy = 280;
	simu_track_table[154].vz = 62;
	simu_track_table[154].hdg = 289;

	simu_track_table[155].lat = 465184913;
	simu_track_table[155].lon = 65665534;
	simu_track_table[155].alt = 410;
	simu_track_table[155].relative_alt = -10;
	simu_track_table[155].vx = -93;
	simu_track_table[155].vy = 281;
	simu_track_table[155].vz = 54;
	simu_track_table[155].hdg = 288;

	simu_track_table[156].lat = 465184888;
	simu_track_table[156].lon = 65665628;
	simu_track_table[156].alt = 410;
	simu_track_table[156].relative_alt = -10;
	simu_track_table[156].vx = -87;
	simu_track_table[156].vy = 266;
	simu_track_table[156].vz = 40;
	simu_track_table[156].hdg = 288;

	simu_track_table[157].lat = 465184865;
	simu_track_table[157].lon = 65665711;
	simu_track_table[157].alt = 410;
	simu_track_table[157].relative_alt = -10;
	simu_track_table[157].vx = -67;
	simu_track_table[157].vy = 209;
	simu_track_table[157].vz = 33;
	simu_track_table[157].hdg = 288;

	simu_track_table[158].lat = 465184849;
	simu_track_table[158].lon = 65665768;
	simu_track_table[158].alt = 410;
	simu_track_table[158].relative_alt = -10;
	simu_track_table[158].vx = -37;
	simu_track_table[158].vy = 109;
	simu_track_table[158].vz = 36;
	simu_track_table[158].hdg = 289;

	simu_track_table[159].lat = 465184840;
	simu_track_table[159].lon = 65665788;
	simu_track_table[159].alt = 410;
	simu_track_table[159].relative_alt = -10;
	simu_track_table[159].vx = -9;
	simu_track_table[159].vy = -0;
	simu_track_table[159].vz = 42;
	simu_track_table[159].hdg = 1;

	simu_track_table[160].lat = 465184836;
	simu_track_table[160].lon = 65665775;
	simu_track_table[160].alt = 410;
	simu_track_table[160].relative_alt = -10;
	simu_track_table[160].vx = 6;
	simu_track_table[160].vy = -81;
	simu_track_table[160].vz = 43;
	simu_track_table[160].hdg = 94;

	simu_track_table[161].lat = 465184833;
	simu_track_table[161].lon = 65665741;
	simu_track_table[161].alt = 410;
	simu_track_table[161].relative_alt = -10;
	simu_track_table[161].vx = -1;
	simu_track_table[161].vy = -119;
	simu_track_table[161].vz = 40;
	simu_track_table[161].hdg = 90;

	simu_track_table[162].lat = 465184827;
	simu_track_table[162].lon = 65665698;
	simu_track_table[162].alt = 410;
	simu_track_table[162].relative_alt = -10;
	simu_track_table[162].vx = -27;
	simu_track_table[162].vy = -127;
	simu_track_table[162].vz = 37;
	simu_track_table[162].hdg = 78;

	simu_track_table[163].lat = 465184812;
	simu_track_table[163].lon = 65665651;
	simu_track_table[163].alt = 410;
	simu_track_table[163].relative_alt = -10;
	simu_track_table[163].vx = -71;
	simu_track_table[163].vy = -141;
	simu_track_table[163].vz = 38;
	simu_track_table[163].hdg = 63;

	simu_track_table[164].lat = 465184787;
	simu_track_table[164].lon = 65665594;
	simu_track_table[164].alt = 410;
	simu_track_table[164].relative_alt = -10;
	simu_track_table[164].vx = -121;
	simu_track_table[164].vy = -181;
	simu_track_table[164].vz = 43;
	simu_track_table[164].hdg = 56;

	simu_track_table[165].lat = 465184751;
	simu_track_table[165].lon = 65665522;
	simu_track_table[165].alt = 410;
	simu_track_table[165].relative_alt = -10;
	simu_track_table[165].vx = -164;
	simu_track_table[165].vy = -230;
	simu_track_table[165].vz = 46;
	simu_track_table[165].hdg = 55;

	simu_track_table[166].lat = 465184707;
	simu_track_table[166].lon = 65665434;
	simu_track_table[166].alt = 409;
	simu_track_table[166].relative_alt = -9;
	simu_track_table[166].vx = -193;
	simu_track_table[166].vy = -266;
	simu_track_table[166].vz = 44;
	simu_track_table[166].hdg = 54;

	simu_track_table[167].lat = 465184658;
	simu_track_table[167].lon = 65665337;
	simu_track_table[167].alt = 409;
	simu_track_table[167].relative_alt = -9;
	simu_track_table[167].vx = -205;
	simu_track_table[167].vy = -280;
	simu_track_table[167].vz = 37;
	simu_track_table[167].hdg = 54;

	simu_track_table[168].lat = 465184607;
	simu_track_table[168].lon = 65665239;
	simu_track_table[168].alt = 409;
	simu_track_table[168].relative_alt = -9;
	simu_track_table[168].vx = -206;
	simu_track_table[168].vy = -274;
	simu_track_table[168].vz = 29;
	simu_track_table[168].hdg = 53;

	simu_track_table[169].lat = 465184556;
	simu_track_table[169].lon = 65665143;
	simu_track_table[169].alt = 409;
	simu_track_table[169].relative_alt = -9;
	simu_track_table[169].vx = -201;
	simu_track_table[169].vy = -260;
	simu_track_table[169].vz = 24;
	simu_track_table[169].hdg = 52;

	simu_track_table[170].lat = 465184507;
	simu_track_table[170].lon = 65665051;
	simu_track_table[170].alt = 409;
	simu_track_table[170].relative_alt = -9;
	simu_track_table[170].vx = -196;
	simu_track_table[170].vy = -247;
	simu_track_table[170].vz = 21;
	simu_track_table[170].hdg = 52;

	simu_track_table[171].lat = 465184458;
	simu_track_table[171].lon = 65664963;
	simu_track_table[171].alt = 409;
	simu_track_table[171].relative_alt = -9;
	simu_track_table[171].vx = -192;
	simu_track_table[171].vy = -240;
	simu_track_table[171].vz = 21;
	simu_track_table[171].hdg = 51;

	simu_track_table[172].lat = 465184411;
	simu_track_table[172].lon = 65664876;
	simu_track_table[172].alt = 409;
	simu_track_table[172].relative_alt = -9;
	simu_track_table[172].vx = -191;
	simu_track_table[172].vy = -239;
	simu_track_table[172].vz = 21;
	simu_track_table[172].hdg = 51;

	simu_track_table[173].lat = 465184363;
	simu_track_table[173].lon = 65664789;
	simu_track_table[173].alt = 409;
	simu_track_table[173].relative_alt = -9;
	simu_track_table[173].vx = -191;
	simu_track_table[173].vy = -243;
	simu_track_table[173].vz = 21;
	simu_track_table[173].hdg = 52;

	simu_track_table[174].lat = 465184316;
	simu_track_table[174].lon = 65664700;
	simu_track_table[174].alt = 409;
	simu_track_table[174].relative_alt = -9;
	simu_track_table[174].vx = -192;
	simu_track_table[174].vy = -247;
	simu_track_table[174].vz = 21;
	simu_track_table[174].hdg = 52;

	simu_track_table[175].lat = 465184269;
	simu_track_table[175].lon = 65664611;
	simu_track_table[175].alt = 409;
	simu_track_table[175].relative_alt = -9;
	simu_track_table[175].vx = -191;
	simu_track_table[175].vy = -249;
	simu_track_table[175].vz = 19;
	simu_track_table[175].hdg = 52;

	simu_track_table[176].lat = 465184223;
	simu_track_table[176].lon = 65664522;
	simu_track_table[176].alt = 409;
	simu_track_table[176].relative_alt = -9;
	simu_track_table[176].vx = -190;
	simu_track_table[176].vy = -248;
	simu_track_table[176].vz = 18;
	simu_track_table[176].hdg = 53;

	simu_track_table[177].lat = 465184177;
	simu_track_table[177].lon = 65664433;
	simu_track_table[177].alt = 409;
	simu_track_table[177].relative_alt = -9;
	simu_track_table[177].vx = -188;
	simu_track_table[177].vy = -246;
	simu_track_table[177].vz = 16;
	simu_track_table[177].hdg = 53;

	simu_track_table[178].lat = 465184132;
	simu_track_table[178].lon = 65664345;
	simu_track_table[178].alt = 409;
	simu_track_table[178].relative_alt = -9;
	simu_track_table[178].vx = -186;
	simu_track_table[178].vy = -242;
	simu_track_table[178].vz = 15;
	simu_track_table[178].hdg = 53;

	simu_track_table[179].lat = 465184087;
	simu_track_table[179].lon = 65664259;
	simu_track_table[179].alt = 409;
	simu_track_table[179].relative_alt = -9;
	simu_track_table[179].vx = -184;
	simu_track_table[179].vy = -240;
	simu_track_table[179].vz = 13;
	simu_track_table[179].hdg = 53;

	simu_track_table[180].lat = 465184043;
	simu_track_table[180].lon = 65664173;
	simu_track_table[180].alt = 409;
	simu_track_table[180].relative_alt = -9;
	simu_track_table[180].vx = -183;
	simu_track_table[180].vy = -239;
	simu_track_table[180].vz = 12;
	simu_track_table[180].hdg = 53;

	simu_track_table[181].lat = 465183999;
	simu_track_table[181].lon = 65664088;
	simu_track_table[181].alt = 409;
	simu_track_table[181].relative_alt = -9;
	simu_track_table[181].vx = -182;
	simu_track_table[181].vy = -238;
	simu_track_table[181].vz = 11;
	simu_track_table[181].hdg = 53;

	simu_track_table[182].lat = 465183956;
	simu_track_table[182].lon = 65664003;
	simu_track_table[182].alt = 409;
	simu_track_table[182].relative_alt = -9;
	simu_track_table[182].vx = -182;
	simu_track_table[182].vy = -238;
	simu_track_table[182].vz = 10;
	simu_track_table[182].hdg = 53;

	simu_track_table[183].lat = 465183913;
	simu_track_table[183].lon = 65663918;
	simu_track_table[183].alt = 409;
	simu_track_table[183].relative_alt = -9;
	simu_track_table[183].vx = -181;
	simu_track_table[183].vy = -238;
	simu_track_table[183].vz = 8;
	simu_track_table[183].hdg = 53;

	simu_track_table[184].lat = 465183871;
	simu_track_table[184].lon = 65663834;
	simu_track_table[184].alt = 409;
	simu_track_table[184].relative_alt = -9;
	simu_track_table[184].vx = -181;
	simu_track_table[184].vy = -238;
	simu_track_table[184].vz = 7;
	simu_track_table[184].hdg = 53;

	simu_track_table[185].lat = 465183828;
	simu_track_table[185].lon = 65663750;
	simu_track_table[185].alt = 409;
	simu_track_table[185].relative_alt = -9;
	simu_track_table[185].vx = -181;
	simu_track_table[185].vy = -237;
	simu_track_table[185].vz = 5;
	simu_track_table[185].hdg = 53;

	simu_track_table[186].lat = 465183786;
	simu_track_table[186].lon = 65663666;
	simu_track_table[186].alt = 409;
	simu_track_table[186].relative_alt = -9;
	simu_track_table[186].vx = -180;
	simu_track_table[186].vy = -236;
	simu_track_table[186].vz = 3;
	simu_track_table[186].hdg = 53;

	simu_track_table[187].lat = 465183744;
	simu_track_table[187].lon = 65663583;
	simu_track_table[187].alt = 409;
	simu_track_table[187].relative_alt = -9;
	simu_track_table[187].vx = -180;
	simu_track_table[187].vy = -235;
	simu_track_table[187].vz = 1;
	simu_track_table[187].hdg = 53;

	simu_track_table[188].lat = 465183703;
	simu_track_table[188].lon = 65663501;
	simu_track_table[188].alt = 409;
	simu_track_table[188].relative_alt = -9;
	simu_track_table[188].vx = -180;
	simu_track_table[188].vy = -234;
	simu_track_table[188].vz = -2;
	simu_track_table[188].hdg = 52;

	simu_track_table[189].lat = 465183661;
	simu_track_table[189].lon = 65663418;
	simu_track_table[189].alt = 409;
	simu_track_table[189].relative_alt = -9;
	simu_track_table[189].vx = -180;
	simu_track_table[189].vy = -233;
	simu_track_table[189].vz = -4;
	simu_track_table[189].hdg = 52;

	simu_track_table[190].lat = 465183621;
	simu_track_table[190].lon = 65663337;
	simu_track_table[190].alt = 409;
	simu_track_table[190].relative_alt = -9;
	simu_track_table[190].vx = -180;
	simu_track_table[190].vy = -232;
	simu_track_table[190].vz = -7;
	simu_track_table[190].hdg = 52;

	simu_track_table[191].lat = 465183579;
	simu_track_table[191].lon = 65663256;
	simu_track_table[191].alt = 409;
	simu_track_table[191].relative_alt = -9;
	simu_track_table[191].vx = -180;
	simu_track_table[191].vy = -231;
	simu_track_table[191].vz = -10;
	simu_track_table[191].hdg = 52;

	simu_track_table[192].lat = 465183539;
	simu_track_table[192].lon = 65663175;
	simu_track_table[192].alt = 409;
	simu_track_table[192].relative_alt = -9;
	simu_track_table[192].vx = -181;
	simu_track_table[192].vy = -230;
	simu_track_table[192].vz = -13;
	simu_track_table[192].hdg = 52;

	simu_track_table[193].lat = 465183498;
	simu_track_table[193].lon = 65663095;
	simu_track_table[193].alt = 409;
	simu_track_table[193].relative_alt = -9;
	simu_track_table[193].vx = -181;
	simu_track_table[193].vy = -229;
	simu_track_table[193].vz = -17;
	simu_track_table[193].hdg = 52;

	simu_track_table[194].lat = 465183457;
	simu_track_table[194].lon = 65663016;
	simu_track_table[194].alt = 409;
	simu_track_table[194].relative_alt = -9;
	simu_track_table[194].vx = -181;
	simu_track_table[194].vy = -227;
	simu_track_table[194].vz = -22;
	simu_track_table[194].hdg = 51;

	simu_track_table[195].lat = 465183417;
	simu_track_table[195].lon = 65662937;
	simu_track_table[195].alt = 409;
	simu_track_table[195].relative_alt = -9;
	simu_track_table[195].vx = -181;
	simu_track_table[195].vy = -225;
	simu_track_table[195].vz = -26;
	simu_track_table[195].hdg = 51;

	simu_track_table[196].lat = 465183376;
	simu_track_table[196].lon = 65662860;
	simu_track_table[196].alt = 409;
	simu_track_table[196].relative_alt = -9;
	simu_track_table[196].vx = -177;
	simu_track_table[196].vy = -218;
	simu_track_table[196].vz = -6;
	simu_track_table[196].hdg = 51;

	simu_track_table[197].lat = 465183340;
	simu_track_table[197].lon = 65662789;
	simu_track_table[197].alt = 409;
	simu_track_table[197].relative_alt = -9;
	simu_track_table[197].vx = -150;
	simu_track_table[197].vy = -185;
	simu_track_table[197].vz = 3;
	simu_track_table[197].hdg = 51;

	simu_track_table[198].lat = 465183312;
	simu_track_table[198].lon = 65662735;
	simu_track_table[198].alt = 409;
	simu_track_table[198].relative_alt = -9;
	simu_track_table[198].vx = -100;
	simu_track_table[198].vy = -118;
	simu_track_table[198].vz = 11;
	simu_track_table[198].hdg = 50;

	simu_track_table[199].lat = 465183297;
	simu_track_table[199].lon = 65662705;
	simu_track_table[199].alt = 409;
	simu_track_table[199].relative_alt = -9;
	simu_track_table[199].vx = -46;
	simu_track_table[199].vy = -43;
	simu_track_table[199].vz = 17;
	simu_track_table[199].hdg = 43;

	simu_track_table[200].lat = 465183293;
	simu_track_table[200].lon = 65662698;
	simu_track_table[200].alt = 409;
	simu_track_table[200].relative_alt = -9;
	simu_track_table[200].vx = -6;
	simu_track_table[200].vy = 9;
	simu_track_table[200].vz = 17;
	simu_track_table[200].hdg = 301;

	simu_track_table[201].lat = 465183297;
	simu_track_table[201].lon = 65662704;
	simu_track_table[201].alt = 409;
	simu_track_table[201].relative_alt = -9;
	simu_track_table[201].vx = 14;
	simu_track_table[201].vy = 24;
	simu_track_table[201].vz = 13;
	simu_track_table[201].hdg = 240;

	simu_track_table[202].lat = 465183304;
	simu_track_table[202].lon = 65662710;
	simu_track_table[202].alt = 409;
	simu_track_table[202].relative_alt = -9;
	simu_track_table[202].vx = 15;
	simu_track_table[202].vy = 9;
	simu_track_table[202].vz = 12;
	simu_track_table[202].hdg = 210;

	simu_track_table[203].lat = 465183311;
	simu_track_table[203].lon = 65662710;
	simu_track_table[203].alt = 409;
	simu_track_table[203].relative_alt = -9;
	simu_track_table[203].vx = 4;
	simu_track_table[203].vy = -18;
	simu_track_table[203].vz = 12;
	simu_track_table[203].hdg = 103;

	simu_track_table[204].lat = 465183314;
	simu_track_table[204].lon = 65662702;
	simu_track_table[204].alt = 409;
	simu_track_table[204].relative_alt = -9;
	simu_track_table[204].vx = -12;
	simu_track_table[204].vy = -39;
	simu_track_table[204].vz = 12;
	simu_track_table[204].hdg = 73;
}

#elif TRACK == 2

void simu_gps_track_jumping_track(void)
{
	simu_track_table[0].lat = 465184241;
	simu_track_table[0].lon = 65659046;
	simu_track_table[0].alt = 428;
	simu_track_table[0].relative_alt = -28;
	simu_track_table[0].vx = 2;
	simu_track_table[0].vy = -1;
	simu_track_table[0].vz = -0;
	simu_track_table[0].hdg = 162;

	simu_track_table[1].lat = 465184242;
	simu_track_table[1].lon = 65659045;
	simu_track_table[1].alt = 428;
	simu_track_table[1].relative_alt = -28;
	simu_track_table[1].vx = 8;
	simu_track_table[1].vy = -1;
	simu_track_table[1].vz = 3;
	simu_track_table[1].hdg = 175;

	simu_track_table[2].lat = 465184248;
	simu_track_table[2].lon = 65659045;
	simu_track_table[2].alt = 428;
	simu_track_table[2].relative_alt = -28;
	simu_track_table[2].vx = 51;
	simu_track_table[2].vy = -1;
	simu_track_table[2].vz = 8;
	simu_track_table[2].hdg = 179;

	simu_track_table[3].lat = 465184270;
	simu_track_table[3].lon = 65659045;
	simu_track_table[3].alt = 428;
	simu_track_table[3].relative_alt = -28;
	simu_track_table[3].vx = 138;
	simu_track_table[3].vy = -1;
	simu_track_table[3].vz = 21;
	simu_track_table[3].hdg = 180;

	simu_track_table[4].lat = 465184316;
	simu_track_table[4].lon = 65659045;
	simu_track_table[4].alt = 428;
	simu_track_table[4].relative_alt = -28;
	simu_track_table[4].vx = 249;
	simu_track_table[4].vy = -1;
	simu_track_table[4].vz = 31;
	simu_track_table[4].hdg = 180;

	simu_track_table[5].lat = 465184386;
	simu_track_table[5].lon = 65659044;
	simu_track_table[5].alt = 428;
	simu_track_table[5].relative_alt = -28;
	simu_track_table[5].vx = 330;
	simu_track_table[5].vy = -1;
	simu_track_table[5].vz = 26;
	simu_track_table[5].hdg = 180;

	simu_track_table[6].lat = 465184466;
	simu_track_table[6].lon = 65659044;
	simu_track_table[6].alt = 428;
	simu_track_table[6].relative_alt = -28;
	simu_track_table[6].vx = 358;
	simu_track_table[6].vy = -1;
	simu_track_table[6].vz = 13;
	simu_track_table[6].hdg = 180;

	simu_track_table[7].lat = 465184547;
	simu_track_table[7].lon = 65659043;
	simu_track_table[7].alt = 428;
	simu_track_table[7].relative_alt = -28;
	simu_track_table[7].vx = 341;
	simu_track_table[7].vy = -1;
	simu_track_table[7].vz = -0;
	simu_track_table[7].hdg = 180;

	simu_track_table[8].lat = 465184623;
	simu_track_table[8].lon = 65659043;
	simu_track_table[8].alt = 428;
	simu_track_table[8].relative_alt = -28;
	simu_track_table[8].vx = 309;
	simu_track_table[8].vy = -1;
	simu_track_table[8].vz = -9;
	simu_track_table[8].hdg = 180;

	simu_track_table[9].lat = 465184697;
	simu_track_table[9].lon = 65659043;
	simu_track_table[9].alt = 428;
	simu_track_table[9].relative_alt = -28;
	simu_track_table[9].vx = 285;
	simu_track_table[9].vy = -1;
	simu_track_table[9].vz = -12;
	simu_track_table[9].hdg = 180;

	simu_track_table[10].lat = 465184766;
	simu_track_table[10].lon = 65659042;
	simu_track_table[10].alt = 428;
	simu_track_table[10].relative_alt = -28;
	simu_track_table[10].vx = 280;
	simu_track_table[10].vy = -1;
	simu_track_table[10].vz = -13;
	simu_track_table[10].hdg = 180;

	simu_track_table[11].lat = 465184838;
	simu_track_table[11].lon = 65659042;
	simu_track_table[11].alt = 428;
	simu_track_table[11].relative_alt = -28;
	simu_track_table[11].vx = 289;
	simu_track_table[11].vy = -0;
	simu_track_table[11].vz = -12;
	simu_track_table[11].hdg = 180;

	simu_track_table[12].lat = 465184913;
	simu_track_table[12].lon = 65659042;
	simu_track_table[12].alt = 428;
	simu_track_table[12].relative_alt = -28;
	simu_track_table[12].vx = 302;
	simu_track_table[12].vy = -0;
	simu_track_table[12].vz = -13;
	simu_track_table[12].hdg = 180;

	simu_track_table[13].lat = 465184991;
	simu_track_table[13].lon = 65659041;
	simu_track_table[13].alt = 428;
	simu_track_table[13].relative_alt = -28;
	simu_track_table[13].vx = 312;
	simu_track_table[13].vy = -0;
	simu_track_table[13].vz = -16;
	simu_track_table[13].hdg = 180;

	simu_track_table[14].lat = 465185071;
	simu_track_table[14].lon = 65659041;
	simu_track_table[14].alt = 428;
	simu_track_table[14].relative_alt = -28;
	simu_track_table[14].vx = 314;
	simu_track_table[14].vy = -0;
	simu_track_table[14].vz = -20;
	simu_track_table[14].hdg = 180;

	simu_track_table[15].lat = 465185152;
	simu_track_table[15].lon = 65659041;
	simu_track_table[15].alt = 428;
	simu_track_table[15].relative_alt = -28;
	simu_track_table[15].vx = 310;
	simu_track_table[15].vy = -0;
	simu_track_table[15].vz = -26;
	simu_track_table[15].hdg = 180;

	simu_track_table[16].lat = 465185232;
	simu_track_table[16].lon = 65659041;
	simu_track_table[16].alt = 428;
	simu_track_table[16].relative_alt = -28;
	simu_track_table[16].vx = 305;
	simu_track_table[16].vy = -0;
	simu_track_table[16].vz = -32;
	simu_track_table[16].hdg = 180;

	simu_track_table[17].lat = 465185310;
	simu_track_table[17].lon = 65659040;
	simu_track_table[17].alt = 428;
	simu_track_table[17].relative_alt = -28;
	simu_track_table[17].vx = 299;
	simu_track_table[17].vy = 0;
	simu_track_table[17].vz = -37;
	simu_track_table[17].hdg = 180;

	simu_track_table[18].lat = 465185388;
	simu_track_table[18].lon = 65659040;
	simu_track_table[18].alt = 428;
	simu_track_table[18].relative_alt = -28;
	simu_track_table[18].vx = 294;
	simu_track_table[18].vy = 0;
	simu_track_table[18].vz = -43;
	simu_track_table[18].hdg = 180;

	simu_track_table[19].lat = 465185465;
	simu_track_table[19].lon = 65659040;
	simu_track_table[19].alt = 428;
	simu_track_table[19].relative_alt = -28;
	simu_track_table[19].vx = 292;
	simu_track_table[19].vy = 0;
	simu_track_table[19].vz = -47;
	simu_track_table[19].hdg = 180;

	simu_track_table[20].lat = 465185541;
	simu_track_table[20].lon = 65659040;
	simu_track_table[20].alt = 428;
	simu_track_table[20].relative_alt = -28;
	simu_track_table[20].vx = 286;
	simu_track_table[20].vy = 3;
	simu_track_table[20].vz = -7;
	simu_track_table[20].hdg = 181;

	simu_track_table[21].lat = 465185613;
	simu_track_table[21].lon = 65659043;
	simu_track_table[21].alt = 428;
	simu_track_table[21].relative_alt = -28;
	simu_track_table[21].vx = 250;
	simu_track_table[21].vy = 24;
	simu_track_table[21].vz = 13;
	simu_track_table[21].hdg = 185;

	simu_track_table[22].lat = 465185671;
	simu_track_table[22].lon = 65659058;
	simu_track_table[22].alt = 428;
	simu_track_table[22].relative_alt = -28;
	simu_track_table[22].vx = 169;
	simu_track_table[22].vy = 71;
	simu_track_table[22].vz = 25;
	simu_track_table[22].hdg = 203;

	simu_track_table[23].lat = 465185708;
	simu_track_table[23].lon = 65659092;
	simu_track_table[23].alt = 428;
	simu_track_table[23].relative_alt = -28;
	simu_track_table[23].vx = 65;
	simu_track_table[23].vy = 135;
	simu_track_table[23].vz = 36;
	simu_track_table[23].hdg = 244;

	simu_track_table[24].lat = 465185721;
	simu_track_table[24].lon = 65659149;
	simu_track_table[24].alt = 428;
	simu_track_table[24].relative_alt = -28;
	simu_track_table[24].vx = -19;
	simu_track_table[24].vy = 209;
	simu_track_table[24].vz = 44;
	simu_track_table[24].hdg = 275;

	simu_track_table[25].lat = 465185720;
	simu_track_table[25].lon = 65659231;
	simu_track_table[25].alt = 428;
	simu_track_table[25].relative_alt = -28;
	simu_track_table[25].vx = -54;
	simu_track_table[25].vy = 278;
	simu_track_table[25].vz = 45;
	simu_track_table[25].hdg = 281;

	simu_track_table[26].lat = 465185715;
	simu_track_table[26].lon = 65659333;
	simu_track_table[26].alt = 428;
	simu_track_table[26].relative_alt = -28;
	simu_track_table[26].vx = -43;
	simu_track_table[26].vy = 326;
	simu_track_table[26].vz = 41;
	simu_track_table[26].hdg = 278;

	simu_track_table[27].lat = 465185715;
	simu_track_table[27].lon = 65659448;
	simu_track_table[27].alt = 428;
	simu_track_table[27].relative_alt = -28;
	simu_track_table[27].vx = -9;
	simu_track_table[27].vy = 346;
	simu_track_table[27].vz = 34;
	simu_track_table[27].hdg = 272;

	simu_track_table[28].lat = 465185722;
	simu_track_table[28].lon = 65659566;
	simu_track_table[28].alt = 428;
	simu_track_table[28].relative_alt = -28;
	simu_track_table[28].vx = 22;
	simu_track_table[28].vy = 341;
	simu_track_table[28].vz = 27;
	simu_track_table[28].hdg = 266;

	simu_track_table[29].lat = 465185734;
	simu_track_table[29].lon = 65659681;
	simu_track_table[29].alt = 428;
	simu_track_table[29].relative_alt = -28;
	simu_track_table[29].vx = 37;
	simu_track_table[29].vy = 322;
	simu_track_table[29].vz = 22;
	simu_track_table[29].hdg = 263;

	simu_track_table[30].lat = 465185748;
	simu_track_table[30].lon = 65659789;
	simu_track_table[30].alt = 428;
	simu_track_table[30].relative_alt = -28;
	simu_track_table[30].vx = 34;
	simu_track_table[30].vy = 303;
	simu_track_table[30].vz = 19;
	simu_track_table[30].hdg = 264;

	simu_track_table[31].lat = 465185760;
	simu_track_table[31].lon = 65659893;
	simu_track_table[31].alt = 428;
	simu_track_table[31].relative_alt = -28;
	simu_track_table[31].vx = 20;
	simu_track_table[31].vy = 292;
	simu_track_table[31].vz = 19;
	simu_track_table[31].hdg = 266;

	simu_track_table[32].lat = 465185768;
	simu_track_table[32].lon = 65659994;
	simu_track_table[32].alt = 427;
	simu_track_table[32].relative_alt = -27;
	simu_track_table[32].vx = 5;
	simu_track_table[32].vy = 291;
	simu_track_table[32].vz = 20;
	simu_track_table[32].hdg = 269;

	simu_track_table[33].lat = 465185773;
	simu_track_table[33].lon = 65660097;
	simu_track_table[33].alt = 427;
	simu_track_table[33].relative_alt = -27;
	simu_track_table[33].vx = -5;
	simu_track_table[33].vy = 296;
	simu_track_table[33].vz = 21;
	simu_track_table[33].hdg = 271;

	simu_track_table[34].lat = 465185777;
	simu_track_table[34].lon = 65660200;
	simu_track_table[34].alt = 427;
	simu_track_table[34].relative_alt = -27;
	simu_track_table[34].vx = -7;
	simu_track_table[34].vy = 302;
	simu_track_table[34].vz = 21;
	simu_track_table[34].hdg = 271;

	simu_track_table[35].lat = 465185781;
	simu_track_table[35].lon = 65660306;
	simu_track_table[35].alt = 427;
	simu_track_table[35].relative_alt = -27;
	simu_track_table[35].vx = -3;
	simu_track_table[35].vy = 306;
	simu_track_table[35].vz = 21;
	simu_track_table[35].hdg = 271;

	simu_track_table[36].lat = 465185785;
	simu_track_table[36].lon = 65660412;
	simu_track_table[36].alt = 427;
	simu_track_table[36].relative_alt = -27;
	simu_track_table[36].vx = 3;
	simu_track_table[36].vy = 307;
	simu_track_table[36].vz = 20;
	simu_track_table[36].hdg = 269;

	simu_track_table[37].lat = 465185791;
	simu_track_table[37].lon = 65660518;
	simu_track_table[37].alt = 427;
	simu_track_table[37].relative_alt = -27;
	simu_track_table[37].vx = 7;
	simu_track_table[37].vy = 305;
	simu_track_table[37].vz = 19;
	simu_track_table[37].hdg = 269;

	simu_track_table[38].lat = 465185797;
	simu_track_table[38].lon = 65660621;
	simu_track_table[38].alt = 427;
	simu_track_table[38].relative_alt = -27;
	simu_track_table[38].vx = 9;
	simu_track_table[38].vy = 301;
	simu_track_table[38].vz = 18;
	simu_track_table[38].hdg = 268;

	simu_track_table[39].lat = 465185803;
	simu_track_table[39].lon = 65660727;
	simu_track_table[39].alt = 427;
	simu_track_table[39].relative_alt = -27;
	simu_track_table[39].vx = 9;
	simu_track_table[39].vy = 299;
	simu_track_table[39].vz = 17;
	simu_track_table[39].hdg = 268;

	simu_track_table[40].lat = 465185809;
	simu_track_table[40].lon = 65660828;
	simu_track_table[40].alt = 427;
	simu_track_table[40].relative_alt = -27;
	simu_track_table[40].vx = 7;
	simu_track_table[40].vy = 298;
	simu_track_table[40].vz = 17;
	simu_track_table[40].hdg = 269;

	simu_track_table[41].lat = 465185814;
	simu_track_table[41].lon = 65660932;
	simu_track_table[41].alt = 427;
	simu_track_table[41].relative_alt = -27;
	simu_track_table[41].vx = 5;
	simu_track_table[41].vy = 298;
	simu_track_table[41].vz = 17;
	simu_track_table[41].hdg = 269;

	simu_track_table[42].lat = 465185819;
	simu_track_table[42].lon = 65661033;
	simu_track_table[42].alt = 427;
	simu_track_table[42].relative_alt = -27;
	simu_track_table[42].vx = 3;
	simu_track_table[42].vy = 299;
	simu_track_table[42].vz = 17;
	simu_track_table[42].hdg = 269;

	simu_track_table[43].lat = 465185823;
	simu_track_table[43].lon = 65661138;
	simu_track_table[43].alt = 427;
	simu_track_table[43].relative_alt = -27;
	simu_track_table[43].vx = 3;
	simu_track_table[43].vy = 299;
	simu_track_table[43].vz = 16;
	simu_track_table[43].hdg = 269;

	simu_track_table[44].lat = 465185827;
	simu_track_table[44].lon = 65661238;
	simu_track_table[44].alt = 427;
	simu_track_table[44].relative_alt = -27;
	simu_track_table[44].vx = 3;
	simu_track_table[44].vy = 299;
	simu_track_table[44].vz = 16;
	simu_track_table[44].hdg = 269;

	simu_track_table[45].lat = 465185831;
	simu_track_table[45].lon = 65661342;
	simu_track_table[45].alt = 427;
	simu_track_table[45].relative_alt = -27;
	simu_track_table[45].vx = 4;
	simu_track_table[45].vy = 299;
	simu_track_table[45].vz = 15;
	simu_track_table[45].hdg = 269;

	simu_track_table[46].lat = 465185834;
	simu_track_table[46].lon = 65661443;
	simu_track_table[46].alt = 427;
	simu_track_table[46].relative_alt = -27;
	simu_track_table[46].vx = 4;
	simu_track_table[46].vy = 298;
	simu_track_table[46].vz = 14;
	simu_track_table[46].hdg = 269;

	simu_track_table[47].lat = 465185838;
	simu_track_table[47].lon = 65661546;
	simu_track_table[47].alt = 427;
	simu_track_table[47].relative_alt = -27;
	simu_track_table[47].vx = 4;
	simu_track_table[47].vy = 298;
	simu_track_table[47].vz = 14;
	simu_track_table[47].hdg = 269;

	simu_track_table[48].lat = 465185842;
	simu_track_table[48].lon = 65661645;
	simu_track_table[48].alt = 427;
	simu_track_table[48].relative_alt = -27;
	simu_track_table[48].vx = 3;
	simu_track_table[48].vy = 297;
	simu_track_table[48].vz = 13;
	simu_track_table[48].hdg = 269;

	simu_track_table[49].lat = 465185845;
	simu_track_table[49].lon = 65661749;
	simu_track_table[49].alt = 427;
	simu_track_table[49].relative_alt = -27;
	simu_track_table[49].vx = 3;
	simu_track_table[49].vy = 298;
	simu_track_table[49].vz = 13;
	simu_track_table[49].hdg = 269;

	simu_track_table[50].lat = 465185848;
	simu_track_table[50].lon = 65661848;
	simu_track_table[50].alt = 427;
	simu_track_table[50].relative_alt = -27;
	simu_track_table[50].vx = 2;
	simu_track_table[50].vy = 298;
	simu_track_table[50].vz = 12;
	simu_track_table[50].hdg = 270;

	simu_track_table[51].lat = 465185850;
	simu_track_table[51].lon = 65661951;
	simu_track_table[51].alt = 427;
	simu_track_table[51].relative_alt = -27;
	simu_track_table[51].vx = 2;
	simu_track_table[51].vy = 297;
	simu_track_table[51].vz = 12;
	simu_track_table[51].hdg = 270;

	simu_track_table[52].lat = 465185853;
	simu_track_table[52].lon = 65662049;
	simu_track_table[52].alt = 427;
	simu_track_table[52].relative_alt = -27;
	simu_track_table[52].vx = 2;
	simu_track_table[52].vy = 297;
	simu_track_table[52].vz = 11;
	simu_track_table[52].hdg = 270;

	simu_track_table[53].lat = 465185855;
	simu_track_table[53].lon = 65662152;
	simu_track_table[53].alt = 427;
	simu_track_table[53].relative_alt = -27;
	simu_track_table[53].vx = 1;
	simu_track_table[53].vy = 297;
	simu_track_table[53].vz = 10;
	simu_track_table[53].hdg = 270;

	simu_track_table[54].lat = 465185857;
	simu_track_table[54].lon = 65662250;
	simu_track_table[54].alt = 427;
	simu_track_table[54].relative_alt = -27;
	simu_track_table[54].vx = 1;
	simu_track_table[54].vy = 297;
	simu_track_table[54].vz = 10;
	simu_track_table[54].hdg = 270;

	simu_track_table[55].lat = 465185859;
	simu_track_table[55].lon = 65662352;
	simu_track_table[55].alt = 427;
	simu_track_table[55].relative_alt = -27;
	simu_track_table[55].vx = 1;
	simu_track_table[55].vy = 297;
	simu_track_table[55].vz = 9;
	simu_track_table[55].hdg = 270;

	simu_track_table[56].lat = 465185860;
	simu_track_table[56].lon = 65662450;
	simu_track_table[56].alt = 427;
	simu_track_table[56].relative_alt = -27;
	simu_track_table[56].vx = 1;
	simu_track_table[56].vy = 297;
	simu_track_table[56].vz = 8;
	simu_track_table[56].hdg = 270;

	simu_track_table[57].lat = 465185862;
	simu_track_table[57].lon = 65662551;
	simu_track_table[57].alt = 427;
	simu_track_table[57].relative_alt = -27;
	simu_track_table[57].vx = 1;
	simu_track_table[57].vy = 296;
	simu_track_table[57].vz = 8;
	simu_track_table[57].hdg = 270;

	simu_track_table[58].lat = 465185863;
	simu_track_table[58].lon = 65662649;
	simu_track_table[58].alt = 427;
	simu_track_table[58].relative_alt = -27;
	simu_track_table[58].vx = 1;
	simu_track_table[58].vy = 296;
	simu_track_table[58].vz = 7;
	simu_track_table[58].hdg = 270;

	simu_track_table[59].lat = 465185864;
	simu_track_table[59].lon = 65662750;
	simu_track_table[59].alt = 427;
	simu_track_table[59].relative_alt = -27;
	simu_track_table[59].vx = 1;
	simu_track_table[59].vy = 296;
	simu_track_table[59].vz = 6;
	simu_track_table[59].hdg = 270;

	simu_track_table[60].lat = 465185865;
	simu_track_table[60].lon = 65662847;
	simu_track_table[60].alt = 427;
	simu_track_table[60].relative_alt = -27;
	simu_track_table[60].vx = 1;
	simu_track_table[60].vy = 296;
	simu_track_table[60].vz = 5;
	simu_track_table[60].hdg = 270;

	simu_track_table[61].lat = 465185866;
	simu_track_table[61].lon = 65662948;
	simu_track_table[61].alt = 427;
	simu_track_table[61].relative_alt = -27;
	simu_track_table[61].vx = 1;
	simu_track_table[61].vy = 296;
	simu_track_table[61].vz = 5;
	simu_track_table[61].hdg = 270;

	simu_track_table[62].lat = 465185867;
	simu_track_table[62].lon = 65663045;
	simu_track_table[62].alt = 426;
	simu_track_table[62].relative_alt = -26;
	simu_track_table[62].vx = 1;
	simu_track_table[62].vy = 296;
	simu_track_table[62].vz = 4;
	simu_track_table[62].hdg = 270;

	simu_track_table[63].lat = 465185867;
	simu_track_table[63].lon = 65663145;
	simu_track_table[63].alt = 426;
	simu_track_table[63].relative_alt = -26;
	simu_track_table[63].vx = 0;
	simu_track_table[63].vy = 296;
	simu_track_table[63].vz = 3;
	simu_track_table[63].hdg = 270;

	simu_track_table[64].lat = 465185868;
	simu_track_table[64].lon = 65663242;
	simu_track_table[64].alt = 426;
	simu_track_table[64].relative_alt = -26;
	simu_track_table[64].vx = 0;
	simu_track_table[64].vy = 295;
	simu_track_table[64].vz = 2;
	simu_track_table[64].hdg = 270;

	simu_track_table[65].lat = 465185868;
	simu_track_table[65].lon = 65663342;
	simu_track_table[65].alt = 426;
	simu_track_table[65].relative_alt = -26;
	simu_track_table[65].vx = 1;
	simu_track_table[65].vy = 295;
	simu_track_table[65].vz = 1;
	simu_track_table[65].hdg = 270;

	simu_track_table[66].lat = 465185868;
	simu_track_table[66].lon = 65663438;
	simu_track_table[66].alt = 426;
	simu_track_table[66].relative_alt = -26;
	simu_track_table[66].vx = 1;
	simu_track_table[66].vy = 295;
	simu_track_table[66].vz = 0;
	simu_track_table[66].hdg = 270;

	simu_track_table[67].lat = 465185868;
	simu_track_table[67].lon = 65663538;
	simu_track_table[67].alt = 426;
	simu_track_table[67].relative_alt = -26;
	simu_track_table[67].vx = 1;
	simu_track_table[67].vy = 295;
	simu_track_table[67].vz = -1;
	simu_track_table[67].hdg = 270;

	simu_track_table[68].lat = 465185868;
	simu_track_table[68].lon = 65663634;
	simu_track_table[68].alt = 426;
	simu_track_table[68].relative_alt = -26;
	simu_track_table[68].vx = 1;
	simu_track_table[68].vy = 295;
	simu_track_table[68].vz = -1;
	simu_track_table[68].hdg = 270;

	simu_track_table[69].lat = 465185868;
	simu_track_table[69].lon = 65663733;
	simu_track_table[69].alt = 426;
	simu_track_table[69].relative_alt = -26;
	simu_track_table[69].vx = 1;
	simu_track_table[69].vy = 295;
	simu_track_table[69].vz = -2;
	simu_track_table[69].hdg = 270;

	simu_track_table[70].lat = 465185868;
	simu_track_table[70].lon = 65663828;
	simu_track_table[70].alt = 426;
	simu_track_table[70].relative_alt = -26;
	simu_track_table[70].vx = 1;
	simu_track_table[70].vy = 294;
	simu_track_table[70].vz = -4;
	simu_track_table[70].hdg = 270;

	simu_track_table[71].lat = 465185868;
	simu_track_table[71].lon = 65663927;
	simu_track_table[71].alt = 427;
	simu_track_table[71].relative_alt = -27;
	simu_track_table[71].vx = 1;
	simu_track_table[71].vy = 294;
	simu_track_table[71].vz = -5;
	simu_track_table[71].hdg = 270;

	simu_track_table[72].lat = 465185867;
	simu_track_table[72].lon = 65664022;
	simu_track_table[72].alt = 427;
	simu_track_table[72].relative_alt = -27;
	simu_track_table[72].vx = 1;
	simu_track_table[72].vy = 294;
	simu_track_table[72].vz = -6;
	simu_track_table[72].hdg = 270;

	simu_track_table[73].lat = 465185867;
	simu_track_table[73].lon = 65664121;
	simu_track_table[73].alt = 427;
	simu_track_table[73].relative_alt = -27;
	simu_track_table[73].vx = 1;
	simu_track_table[73].vy = 294;
	simu_track_table[73].vz = -7;
	simu_track_table[73].hdg = 270;

	simu_track_table[74].lat = 465185866;
	simu_track_table[74].lon = 65664215;
	simu_track_table[74].alt = 427;
	simu_track_table[74].relative_alt = -27;
	simu_track_table[74].vx = 2;
	simu_track_table[74].vy = 293;
	simu_track_table[74].vz = -8;
	simu_track_table[74].hdg = 270;

	simu_track_table[75].lat = 465185866;
	simu_track_table[75].lon = 65664313;
	simu_track_table[75].alt = 427;
	simu_track_table[75].relative_alt = -27;
	simu_track_table[75].vx = 2;
	simu_track_table[75].vy = 293;
	simu_track_table[75].vz = -9;
	simu_track_table[75].hdg = 270;

	simu_track_table[76].lat = 465185865;
	simu_track_table[76].lon = 65664408;
	simu_track_table[76].alt = 427;
	simu_track_table[76].relative_alt = -27;
	simu_track_table[76].vx = 2;
	simu_track_table[76].vy = 293;
	simu_track_table[76].vz = -11;
	simu_track_table[76].hdg = 270;

	simu_track_table[77].lat = 465185865;
	simu_track_table[77].lon = 65664505;
	simu_track_table[77].alt = 427;
	simu_track_table[77].relative_alt = -27;
	simu_track_table[77].vx = 2;
	simu_track_table[77].vy = 292;
	simu_track_table[77].vz = -12;
	simu_track_table[77].hdg = 270;

	simu_track_table[78].lat = 465185864;
	simu_track_table[78].lon = 65664599;
	simu_track_table[78].alt = 427;
	simu_track_table[78].relative_alt = -27;
	simu_track_table[78].vx = 3;
	simu_track_table[78].vy = 292;
	simu_track_table[78].vz = -13;
	simu_track_table[78].hdg = 269;

	simu_track_table[79].lat = 465185863;
	simu_track_table[79].lon = 65664696;
	simu_track_table[79].alt = 427;
	simu_track_table[79].relative_alt = -27;
	simu_track_table[79].vx = 3;
	simu_track_table[79].vy = 291;
	simu_track_table[79].vz = -15;
	simu_track_table[79].hdg = 269;

	simu_track_table[80].lat = 465185862;
	simu_track_table[80].lon = 65664789;
	simu_track_table[80].alt = 427;
	simu_track_table[80].relative_alt = -27;
	simu_track_table[80].vx = 3;
	simu_track_table[80].vy = 291;
	simu_track_table[80].vz = -16;
	simu_track_table[80].hdg = 269;

	simu_track_table[81].lat = 465185862;
	simu_track_table[81].lon = 65664886;
	simu_track_table[81].alt = 427;
	simu_track_table[81].relative_alt = -27;
	simu_track_table[81].vx = 4;
	simu_track_table[81].vy = 291;
	simu_track_table[81].vz = -18;
	simu_track_table[81].hdg = 269;

	simu_track_table[82].lat = 465185861;
	simu_track_table[82].lon = 65664979;
	simu_track_table[82].alt = 427;
	simu_track_table[82].relative_alt = -27;
	simu_track_table[82].vx = 4;
	simu_track_table[82].vy = 291;
	simu_track_table[82].vz = -20;
	simu_track_table[82].hdg = 269;

	simu_track_table[83].lat = 465185860;
	simu_track_table[83].lon = 65665075;
	simu_track_table[83].alt = 427;
	simu_track_table[83].relative_alt = -27;
	simu_track_table[83].vx = 5;
	simu_track_table[83].vy = 290;
	simu_track_table[83].vz = -22;
	simu_track_table[83].hdg = 269;

	simu_track_table[84].lat = 465185860;
	simu_track_table[84].lon = 65665168;
	simu_track_table[84].alt = 427;
	simu_track_table[84].relative_alt = -27;
	simu_track_table[84].vx = 5;
	simu_track_table[84].vy = 289;
	simu_track_table[84].vz = -24;
	simu_track_table[84].hdg = 269;

	simu_track_table[85].lat = 465185859;
	simu_track_table[85].lon = 65665263;
	simu_track_table[85].alt = 427;
	simu_track_table[85].relative_alt = -27;
	simu_track_table[85].vx = 6;
	simu_track_table[85].vy = 289;
	simu_track_table[85].vz = -26;
	simu_track_table[85].hdg = 269;

	simu_track_table[86].lat = 465185858;
	simu_track_table[86].lon = 65665355;
	simu_track_table[86].alt = 427;
	simu_track_table[86].relative_alt = -27;
	simu_track_table[86].vx = 6;
	simu_track_table[86].vy = 288;
	simu_track_table[86].vz = -28;
	simu_track_table[86].hdg = 269;

	simu_track_table[87].lat = 465185858;
	simu_track_table[87].lon = 65665449;
	simu_track_table[87].alt = 427;
	simu_track_table[87].relative_alt = -27;
	simu_track_table[87].vx = 7;
	simu_track_table[87].vy = 287;
	simu_track_table[87].vz = -30;
	simu_track_table[87].hdg = 269;

	simu_track_table[88].lat = 465185858;
	simu_track_table[88].lon = 65665540;
	simu_track_table[88].alt = 427;
	simu_track_table[88].relative_alt = -27;
	simu_track_table[88].vx = 8;
	simu_track_table[88].vy = 286;
	simu_track_table[88].vz = -33;
	simu_track_table[88].hdg = 268;

	simu_track_table[89].lat = 465185857;
	simu_track_table[89].lon = 65665634;
	simu_track_table[89].alt = 427;
	simu_track_table[89].relative_alt = -27;
	simu_track_table[89].vx = 8;
	simu_track_table[89].vy = 285;
	simu_track_table[89].vz = -35;
	simu_track_table[89].hdg = 268;

	simu_track_table[90].lat = 465185857;
	simu_track_table[90].lon = 65665725;
	simu_track_table[90].alt = 427;
	simu_track_table[90].relative_alt = -27;
	simu_track_table[90].vx = 9;
	simu_track_table[90].vy = 284;
	simu_track_table[90].vz = -38;
	simu_track_table[90].hdg = 268;

	simu_track_table[91].lat = 465185857;
	simu_track_table[91].lon = 65665818;
	simu_track_table[91].alt = 428;
	simu_track_table[91].relative_alt = -28;
	simu_track_table[91].vx = 10;
	simu_track_table[91].vy = 283;
	simu_track_table[91].vz = -42;
	simu_track_table[91].hdg = 268;

	simu_track_table[92].lat = 465185857;
	simu_track_table[92].lon = 65665907;
	simu_track_table[92].alt = 428;
	simu_track_table[92].relative_alt = -28;
	simu_track_table[92].vx = 11;
	simu_track_table[92].vy = 282;
	simu_track_table[92].vz = -45;
	simu_track_table[92].hdg = 268;

	simu_track_table[93].lat = 465185857;
	simu_track_table[93].lon = 65665999;
	simu_track_table[93].alt = 428;
	simu_track_table[93].relative_alt = -28;
	simu_track_table[93].vx = 12;
	simu_track_table[93].vy = 280;
	simu_track_table[93].vz = -47;
	simu_track_table[93].hdg = 267;

	simu_track_table[94].lat = 465185858;
	simu_track_table[94].lon = 65666087;
	simu_track_table[94].alt = 428;
	simu_track_table[94].relative_alt = -28;
	simu_track_table[94].vx = 13;
	simu_track_table[94].vy = 276;
	simu_track_table[94].vz = -48;
	simu_track_table[94].hdg = 267;

	simu_track_table[95].lat = 465185859;
	simu_track_table[95].lon = 65666175;
	simu_track_table[95].alt = 428;
	simu_track_table[95].relative_alt = -28;
	simu_track_table[95].vx = 15;
	simu_track_table[95].vy = 266;
	simu_track_table[95].vz = -48;
	simu_track_table[95].hdg = 267;

	simu_track_table[96].lat = 465185859;
	simu_track_table[96].lon = 65666257;
	simu_track_table[96].alt = 428;
	simu_track_table[96].relative_alt = -28;
	simu_track_table[96].vx = 15;
	simu_track_table[96].vy = 252;
	simu_track_table[96].vz = -31;
	simu_track_table[96].hdg = 267;

	simu_track_table[97].lat = 465185860;
	simu_track_table[97].lon = 65666333;
	simu_track_table[97].alt = 428;
	simu_track_table[97].relative_alt = -28;
	simu_track_table[97].vx = 13;
	simu_track_table[97].vy = 214;
	simu_track_table[97].vz = -11;
	simu_track_table[97].hdg = 267;

	simu_track_table[98].lat = 465185860;
	simu_track_table[98].lon = 65666389;
	simu_track_table[98].alt = 428;
	simu_track_table[98].relative_alt = -28;
	simu_track_table[98].vx = 7;
	simu_track_table[98].vy = 149;
	simu_track_table[98].vz = -1;
	simu_track_table[98].hdg = 267;

	simu_track_table[99].lat = 465185858;
	simu_track_table[99].lon = 65666423;
	simu_track_table[99].alt = 428;
	simu_track_table[99].relative_alt = -28;
	simu_track_table[99].vx = -1;
	simu_track_table[99].vy = 75;
	simu_track_table[99].vz = 5;
	simu_track_table[99].hdg = 271;

	simu_track_table[100].lat = 465185855;
	simu_track_table[100].lon = 65666434;
	simu_track_table[100].alt = 428;
	simu_track_table[100].relative_alt = -28;
	simu_track_table[100].vx = -6;
	simu_track_table[100].vy = 22;
	simu_track_table[100].vz = 5;
	simu_track_table[100].hdg = 284;

	simu_track_table[101].lat = 465185851;
	simu_track_table[101].lon = 65666431;
	simu_track_table[101].alt = 428;
	simu_track_table[101].relative_alt = -28;
	simu_track_table[101].vx = -5;
	simu_track_table[101].vy = -2;
	simu_track_table[101].vz = 4;
	simu_track_table[101].hdg = 20;

	simu_track_table[102].lat = 465185848;
	simu_track_table[102].lon = 65666425;
	simu_track_table[102].alt = 428;
	simu_track_table[102].relative_alt = -28;
	simu_track_table[102].vx = 1;
	simu_track_table[102].vy = 2;
	simu_track_table[102].vz = 3;
	simu_track_table[102].hdg = 257;

	simu_track_table[103].lat = 465185846;
	simu_track_table[103].lon = 65666421;
	simu_track_table[103].alt = 428;
	simu_track_table[103].relative_alt = -28;
	simu_track_table[103].vx = 6;
	simu_track_table[103].vy = 20;
	simu_track_table[103].vz = 3;
	simu_track_table[103].hdg = 252;

	simu_track_table[104].lat = 465185845;
	simu_track_table[104].lon = 65666423;
	simu_track_table[104].alt = 428;
	simu_track_table[104].relative_alt = -28;
	simu_track_table[104].vx = 10;
	simu_track_table[104].vy = 38;
	simu_track_table[104].vz = 3;
	simu_track_table[104].hdg = 255;

	simu_track_table[105].lat = 465185844;
	simu_track_table[105].lon = 65666430;
	simu_track_table[105].alt = 428;
	simu_track_table[105].relative_alt = -28;
	simu_track_table[105].vx = 11;
	simu_track_table[105].vy = 49;
	simu_track_table[105].vz = 4;
	simu_track_table[105].hdg = 257;
}

#elif TRACK == 3

void simu_gps_track_circular_track(void)
{
	simu_track_table[0].lat = 465186827;
	simu_track_table[0].lon = 65658272;
	simu_track_table[0].alt = 428;
	simu_track_table[0].relative_alt = -28;
	simu_track_table[0].vx = 4;
	simu_track_table[0].vy = 1;
	simu_track_table[0].vz = -0;
	simu_track_table[0].hdg = 189;

	simu_track_table[1].lat = 465186828;
	simu_track_table[1].lon = 65658272;
	simu_track_table[1].alt = 428;
	simu_track_table[1].relative_alt = -28;
	simu_track_table[1].vx = 5;
	simu_track_table[1].vy = 2;
	simu_track_table[1].vz = 8;
	simu_track_table[1].hdg = 204;

	simu_track_table[2].lat = 465186830;
	simu_track_table[2].lon = 65658276;
	simu_track_table[2].alt = 428;
	simu_track_table[2].relative_alt = -28;
	simu_track_table[2].vx = 22;
	simu_track_table[2].vy = 27;
	simu_track_table[2].vz = 18;
	simu_track_table[2].hdg = 231;

	simu_track_table[3].lat = 465186840;
	simu_track_table[3].lon = 65658295;
	simu_track_table[3].alt = 428;
	simu_track_table[3].relative_alt = -28;
	simu_track_table[3].vx = 67;
	simu_track_table[3].vy = 97;
	simu_track_table[3].vz = 29;
	simu_track_table[3].hdg = 235;

	simu_track_table[4].lat = 465186861;
	simu_track_table[4].lon = 65658340;
	simu_track_table[4].alt = 428;
	simu_track_table[4].relative_alt = -28;
	simu_track_table[4].vx = 123;
	simu_track_table[4].vy = 185;
	simu_track_table[4].vz = 40;
	simu_track_table[4].hdg = 236;

	simu_track_table[5].lat = 465186895;
	simu_track_table[5].lon = 65658415;
	simu_track_table[5].alt = 428;
	simu_track_table[5].relative_alt = -28;
	simu_track_table[5].vx = 172;
	simu_track_table[5].vy = 254;
	simu_track_table[5].vz = 41;
	simu_track_table[5].hdg = 236;

	simu_track_table[6].lat = 465186937;
	simu_track_table[6].lon = 65658504;
	simu_track_table[6].alt = 427;
	simu_track_table[6].relative_alt = -27;
	simu_track_table[6].vx = 199;
	simu_track_table[6].vy = 284;
	simu_track_table[6].vz = 32;
	simu_track_table[6].hdg = 235;

	simu_track_table[7].lat = 465186986;
	simu_track_table[7].lon = 65658602;
	simu_track_table[7].alt = 427;
	simu_track_table[7].relative_alt = -27;
	simu_track_table[7].vx = 206;
	simu_track_table[7].vy = 282;
	simu_track_table[7].vz = 21;
	simu_track_table[7].hdg = 234;

	simu_track_table[8].lat = 465187033;
	simu_track_table[8].lon = 65658694;
	simu_track_table[8].alt = 427;
	simu_track_table[8].relative_alt = -27;
	simu_track_table[8].vx = 200;
	simu_track_table[8].vy = 264;
	simu_track_table[8].vz = 13;
	simu_track_table[8].hdg = 233;

	simu_track_table[9].lat = 465187080;
	simu_track_table[9].lon = 65658783;
	simu_track_table[9].alt = 427;
	simu_track_table[9].relative_alt = -27;
	simu_track_table[9].vx = 190;
	simu_track_table[9].vy = 247;
	simu_track_table[9].vz = 9;
	simu_track_table[9].hdg = 232;

	simu_track_table[10].lat = 465187124;
	simu_track_table[10].lon = 65658865;
	simu_track_table[10].alt = 427;
	simu_track_table[10].relative_alt = -27;
	simu_track_table[10].vx = 183;
	simu_track_table[10].vy = 239;
	simu_track_table[10].vz = 8;
	simu_track_table[10].hdg = 233;

	simu_track_table[11].lat = 465187168;
	simu_track_table[11].lon = 65658950;
	simu_track_table[11].alt = 427;
	simu_track_table[11].relative_alt = -27;
	simu_track_table[11].vx = 179;
	simu_track_table[11].vy = 240;
	simu_track_table[11].vz = 8;
	simu_track_table[11].hdg = 233;

	simu_track_table[12].lat = 465187210;
	simu_track_table[12].lon = 65659033;
	simu_track_table[12].alt = 427;
	simu_track_table[12].relative_alt = -27;
	simu_track_table[12].vx = 180;
	simu_track_table[12].vy = 246;
	simu_track_table[12].vz = 9;
	simu_track_table[12].hdg = 234;

	simu_track_table[13].lat = 465187254;
	simu_track_table[13].lon = 65659120;
	simu_track_table[13].alt = 427;
	simu_track_table[13].relative_alt = -27;
	simu_track_table[13].vx = 182;
	simu_track_table[13].vy = 252;
	simu_track_table[13].vz = 9;
	simu_track_table[13].hdg = 234;

	simu_track_table[14].lat = 465187297;
	simu_track_table[14].lon = 65659207;
	simu_track_table[14].alt = 427;
	simu_track_table[14].relative_alt = -27;
	simu_track_table[14].vx = 183;
	simu_track_table[14].vy = 255;
	simu_track_table[14].vz = 8;
	simu_track_table[14].hdg = 234;

	simu_track_table[15].lat = 465187341;
	simu_track_table[15].lon = 65659297;
	simu_track_table[15].alt = 427;
	simu_track_table[15].relative_alt = -27;
	simu_track_table[15].vx = 183;
	simu_track_table[15].vy = 254;
	simu_track_table[15].vz = 6;
	simu_track_table[15].hdg = 234;

	simu_track_table[16].lat = 465187384;
	simu_track_table[16].lon = 65659383;
	simu_track_table[16].alt = 427;
	simu_track_table[16].relative_alt = -27;
	simu_track_table[16].vx = 181;
	simu_track_table[16].vy = 250;
	simu_track_table[16].vz = 4;
	simu_track_table[16].hdg = 234;

	simu_track_table[17].lat = 465187428;
	simu_track_table[17].lon = 65659471;
	simu_track_table[17].alt = 427;
	simu_track_table[17].relative_alt = -27;
	simu_track_table[17].vx = 179;
	simu_track_table[17].vy = 246;
	simu_track_table[17].vz = 3;
	simu_track_table[17].hdg = 234;

	simu_track_table[18].lat = 465187470;
	simu_track_table[18].lon = 65659554;
	simu_track_table[18].alt = 427;
	simu_track_table[18].relative_alt = -27;
	simu_track_table[18].vx = 177;
	simu_track_table[18].vy = 243;
	simu_track_table[18].vz = 1;
	simu_track_table[18].hdg = 234;

	simu_track_table[19].lat = 465187513;
	simu_track_table[19].lon = 65659640;
	simu_track_table[19].alt = 427;
	simu_track_table[19].relative_alt = -27;
	simu_track_table[19].vx = 176;
	simu_track_table[19].vy = 242;
	simu_track_table[19].vz = 1;
	simu_track_table[19].hdg = 234;

	simu_track_table[20].lat = 465187554;
	simu_track_table[20].lon = 65659723;
	simu_track_table[20].alt = 427;
	simu_track_table[20].relative_alt = -27;
	simu_track_table[20].vx = 175;
	simu_track_table[20].vy = 242;
	simu_track_table[20].vz = -0;
	simu_track_table[20].hdg = 234;

	simu_track_table[21].lat = 465187596;
	simu_track_table[21].lon = 65659808;
	simu_track_table[21].alt = 427;
	simu_track_table[21].relative_alt = -27;
	simu_track_table[21].vx = 174;
	simu_track_table[21].vy = 242;
	simu_track_table[21].vz = -1;
	simu_track_table[21].hdg = 234;

	simu_track_table[22].lat = 465187637;
	simu_track_table[22].lon = 65659891;
	simu_track_table[22].alt = 427;
	simu_track_table[22].relative_alt = -27;
	simu_track_table[22].vx = 174;
	simu_track_table[22].vy = 242;
	simu_track_table[22].vz = -3;
	simu_track_table[22].hdg = 234;

	simu_track_table[23].lat = 465187679;
	simu_track_table[23].lon = 65659977;
	simu_track_table[23].alt = 427;
	simu_track_table[23].relative_alt = -27;
	simu_track_table[23].vx = 174;
	simu_track_table[23].vy = 242;
	simu_track_table[23].vz = -4;
	simu_track_table[23].hdg = 234;

	simu_track_table[24].lat = 465187720;
	simu_track_table[24].lon = 65660059;
	simu_track_table[24].alt = 427;
	simu_track_table[24].relative_alt = -27;
	simu_track_table[24].vx = 173;
	simu_track_table[24].vy = 242;
	simu_track_table[24].vz = -6;
	simu_track_table[24].hdg = 234;

	simu_track_table[25].lat = 465187762;
	simu_track_table[25].lon = 65660144;
	simu_track_table[25].alt = 427;
	simu_track_table[25].relative_alt = -27;
	simu_track_table[25].vx = 173;
	simu_track_table[25].vy = 241;
	simu_track_table[25].vz = -7;
	simu_track_table[25].hdg = 234;

	simu_track_table[26].lat = 465187802;
	simu_track_table[26].lon = 65660226;
	simu_track_table[26].alt = 427;
	simu_track_table[26].relative_alt = -27;
	simu_track_table[26].vx = 172;
	simu_track_table[26].vy = 240;
	simu_track_table[26].vz = -9;
	simu_track_table[26].hdg = 234;

	simu_track_table[27].lat = 465187844;
	simu_track_table[27].lon = 65660311;
	simu_track_table[27].alt = 427;
	simu_track_table[27].relative_alt = -27;
	simu_track_table[27].vx = 172;
	simu_track_table[27].vy = 239;
	simu_track_table[27].vz = -11;
	simu_track_table[27].hdg = 234;

	simu_track_table[28].lat = 465187884;
	simu_track_table[28].lon = 65660393;
	simu_track_table[28].alt = 427;
	simu_track_table[28].relative_alt = -27;
	simu_track_table[28].vx = 171;
	simu_track_table[28].vy = 239;
	simu_track_table[28].vz = -13;
	simu_track_table[28].hdg = 234;

	simu_track_table[29].lat = 465187925;
	simu_track_table[29].lon = 65660477;
	simu_track_table[29].alt = 427;
	simu_track_table[29].relative_alt = -27;
	simu_track_table[29].vx = 171;
	simu_track_table[29].vy = 239;
	simu_track_table[29].vz = -15;
	simu_track_table[29].hdg = 234;

	simu_track_table[30].lat = 465187965;
	simu_track_table[30].lon = 65660558;
	simu_track_table[30].alt = 427;
	simu_track_table[30].relative_alt = -27;
	simu_track_table[30].vx = 171;
	simu_track_table[30].vy = 238;
	simu_track_table[30].vz = -17;
	simu_track_table[30].hdg = 234;

	simu_track_table[31].lat = 465188006;
	simu_track_table[31].lon = 65660643;
	simu_track_table[31].alt = 427;
	simu_track_table[31].relative_alt = -27;
	simu_track_table[31].vx = 170;
	simu_track_table[31].vy = 238;
	simu_track_table[31].vz = -19;
	simu_track_table[31].hdg = 234;

	simu_track_table[32].lat = 465188045;
	simu_track_table[32].lon = 65660724;
	simu_track_table[32].alt = 427;
	simu_track_table[32].relative_alt = -27;
	simu_track_table[32].vx = 170;
	simu_track_table[32].vy = 237;
	simu_track_table[32].vz = -21;
	simu_track_table[32].hdg = 234;

	simu_track_table[33].lat = 465188086;
	simu_track_table[33].lon = 65660807;
	simu_track_table[33].alt = 428;
	simu_track_table[33].relative_alt = -28;
	simu_track_table[33].vx = 170;
	simu_track_table[33].vy = 237;
	simu_track_table[33].vz = -24;
	simu_track_table[33].hdg = 234;

	simu_track_table[34].lat = 465188125;
	simu_track_table[34].lon = 65660888;
	simu_track_table[34].alt = 428;
	simu_track_table[34].relative_alt = -28;
	simu_track_table[34].vx = 169;
	simu_track_table[34].vy = 236;
	simu_track_table[34].vz = -27;
	simu_track_table[34].hdg = 234;

	simu_track_table[35].lat = 465188166;
	simu_track_table[35].lon = 65660971;
	simu_track_table[35].alt = 428;
	simu_track_table[35].relative_alt = -28;
	simu_track_table[35].vx = 169;
	simu_track_table[35].vy = 235;
	simu_track_table[35].vz = -30;
	simu_track_table[35].hdg = 234;

	simu_track_table[36].lat = 465188205;
	simu_track_table[36].lon = 65661051;
	simu_track_table[36].alt = 428;
	simu_track_table[36].relative_alt = -28;
	simu_track_table[36].vx = 168;
	simu_track_table[36].vy = 234;
	simu_track_table[36].vz = -34;
	simu_track_table[36].hdg = 234;

	simu_track_table[37].lat = 465188245;
	simu_track_table[37].lon = 65661133;
	simu_track_table[37].alt = 428;
	simu_track_table[37].relative_alt = -28;
	simu_track_table[37].vx = 168;
	simu_track_table[37].vy = 233;
	simu_track_table[37].vz = -38;
	simu_track_table[37].hdg = 234;

	simu_track_table[38].lat = 465188284;
	simu_track_table[38].lon = 65661212;
	simu_track_table[38].alt = 428;
	simu_track_table[38].relative_alt = -28;
	simu_track_table[38].vx = 167;
	simu_track_table[38].vy = 232;
	simu_track_table[38].vz = -42;
	simu_track_table[38].hdg = 234;

	simu_track_table[39].lat = 465188324;
	simu_track_table[39].lon = 65661294;
	simu_track_table[39].alt = 428;
	simu_track_table[39].relative_alt = -28;
	simu_track_table[39].vx = 166;
	simu_track_table[39].vy = 230;
	simu_track_table[39].vz = -46;
	simu_track_table[39].hdg = 234;

	simu_track_table[40].lat = 465188362;
	simu_track_table[40].lon = 65661372;
	simu_track_table[40].alt = 428;
	simu_track_table[40].relative_alt = -28;
	simu_track_table[40].vx = 164;
	simu_track_table[40].vy = 227;
	simu_track_table[40].vz = -39;
	simu_track_table[40].hdg = 234;

	simu_track_table[41].lat = 465188400;
	simu_track_table[41].lon = 65661451;
	simu_track_table[41].alt = 428;
	simu_track_table[41].relative_alt = -28;
	simu_track_table[41].vx = 156;
	simu_track_table[41].vy = 221;
	simu_track_table[41].vz = 64;
	simu_track_table[41].hdg = 235;

	simu_track_table[42].lat = 465188433;
	simu_track_table[42].lon = 65661524;
	simu_track_table[42].alt = 428;
	simu_track_table[42].relative_alt = -28;
	simu_track_table[42].vx = 121;
	simu_track_table[42].vy = 206;
	simu_track_table[42].vz = 97;
	simu_track_table[42].hdg = 240;

	simu_track_table[43].lat = 465188455;
	simu_track_table[43].lon = 65661593;
	simu_track_table[43].alt = 428;
	simu_track_table[43].relative_alt = -28;
	simu_track_table[43].vx = 56;
	simu_track_table[43].vy = 184;
	simu_track_table[43].vz = 107;
	simu_track_table[43].hdg = 253;

	simu_track_table[44].lat = 465188460;
	simu_track_table[44].lon = 65661653;
	simu_track_table[44].alt = 427;
	simu_track_table[44].relative_alt = -27;
	simu_track_table[44].vx = -14;
	simu_track_table[44].vy = 163;
	simu_track_table[44].vz = 114;
	simu_track_table[44].hdg = 275;

	simu_track_table[45].lat = 465188450;
	simu_track_table[45].lon = 65661708;
	simu_track_table[45].alt = 427;
	simu_track_table[45].relative_alt = -27;
	simu_track_table[45].vx = -66;
	simu_track_table[45].vy = 148;
	simu_track_table[45].vz = 116;
	simu_track_table[45].hdg = 294;

	simu_track_table[46].lat = 465188433;
	simu_track_table[46].lon = 65661758;
	simu_track_table[46].alt = 427;
	simu_track_table[46].relative_alt = -27;
	simu_track_table[46].vx = -82;
	simu_track_table[46].vy = 146;
	simu_track_table[46].vz = 115;
	simu_track_table[46].hdg = 299;

	simu_track_table[47].lat = 465188414;
	simu_track_table[47].lon = 65661811;
	simu_track_table[47].alt = 426;
	simu_track_table[47].relative_alt = -26;
	simu_track_table[47].vx = -64;
	simu_track_table[47].vy = 157;
	simu_track_table[47].vz = 115;
	simu_track_table[47].hdg = 292;

	simu_track_table[48].lat = 465188402;
	simu_track_table[48].lon = 65661866;
	simu_track_table[48].alt = 426;
	simu_track_table[48].relative_alt = -26;
	simu_track_table[48].vx = -25;
	simu_track_table[48].vy = 172;
	simu_track_table[48].vz = 117;
	simu_track_table[48].hdg = 278;

	simu_track_table[49].lat = 465188399;
	simu_track_table[49].lon = 65661928;
	simu_track_table[49].alt = 426;
	simu_track_table[49].relative_alt = -26;
	simu_track_table[49].vx = 17;
	simu_track_table[49].vy = 184;
	simu_track_table[49].vz = 117;
	simu_track_table[49].hdg = 265;

	simu_track_table[50].lat = 465188405;
	simu_track_table[50].lon = 65661991;
	simu_track_table[50].alt = 426;
	simu_track_table[50].relative_alt = -26;
	simu_track_table[50].vx = 46;
	simu_track_table[50].vy = 190;
	simu_track_table[50].vz = 117;
	simu_track_table[50].hdg = 256;

	simu_track_table[51].lat = 465188415;
	simu_track_table[51].lon = 65662058;
	simu_track_table[51].alt = 425;
	simu_track_table[51].relative_alt = -25;
	simu_track_table[51].vx = 56;
	simu_track_table[51].vy = 190;
	simu_track_table[51].vz = 115;
	simu_track_table[51].hdg = 254;

	simu_track_table[52].lat = 465188425;
	simu_track_table[52].lon = 65662121;
	simu_track_table[52].alt = 425;
	simu_track_table[52].relative_alt = -25;
	simu_track_table[52].vx = 47;
	simu_track_table[52].vy = 187;
	simu_track_table[52].vz = 114;
	simu_track_table[52].hdg = 256;

	simu_track_table[53].lat = 465188433;
	simu_track_table[53].lon = 65662186;
	simu_track_table[53].alt = 425;
	simu_track_table[53].relative_alt = -25;
	simu_track_table[53].vx = 28;
	simu_track_table[53].vy = 184;
	simu_track_table[53].vz = 114;
	simu_track_table[53].hdg = 261;

	simu_track_table[54].lat = 465188436;
	simu_track_table[54].lon = 65662248;
	simu_track_table[54].alt = 424;
	simu_track_table[54].relative_alt = -24;
	simu_track_table[54].vx = 9;
	simu_track_table[54].vy = 181;
	simu_track_table[54].vz = 114;
	simu_track_table[54].hdg = 267;

	simu_track_table[55].lat = 465188435;
	simu_track_table[55].lon = 65662311;
	simu_track_table[55].alt = 424;
	simu_track_table[55].relative_alt = -24;
	simu_track_table[55].vx = -3;
	simu_track_table[55].vy = 180;
	simu_track_table[55].vz = 114;
	simu_track_table[55].hdg = 271;

	simu_track_table[56].lat = 465188433;
	simu_track_table[56].lon = 65662372;
	simu_track_table[56].alt = 424;
	simu_track_table[56].relative_alt = -24;
	simu_track_table[56].vx = -5;
	simu_track_table[56].vy = 181;
	simu_track_table[56].vz = 114;
	simu_track_table[56].hdg = 272;

	simu_track_table[57].lat = 465188431;
	simu_track_table[57].lon = 65662436;
	simu_track_table[57].alt = 424;
	simu_track_table[57].relative_alt = -24;
	simu_track_table[57].vx = 2;
	simu_track_table[57].vy = 184;
	simu_track_table[57].vz = 114;
	simu_track_table[57].hdg = 269;

	simu_track_table[58].lat = 465188431;
	simu_track_table[58].lon = 65662498;
	simu_track_table[58].alt = 423;
	simu_track_table[58].relative_alt = -23;
	simu_track_table[58].vx = 14;
	simu_track_table[58].vy = 186;
	simu_track_table[58].vz = 114;
	simu_track_table[58].hdg = 266;

	simu_track_table[59].lat = 465188434;
	simu_track_table[59].lon = 65662563;
	simu_track_table[59].alt = 423;
	simu_track_table[59].relative_alt = -23;
	simu_track_table[59].vx = 25;
	simu_track_table[59].vy = 188;
	simu_track_table[59].vz = 114;
	simu_track_table[59].hdg = 263;

	simu_track_table[60].lat = 465188438;
	simu_track_table[60].lon = 65662627;
	simu_track_table[60].alt = 423;
	simu_track_table[60].relative_alt = -23;
	simu_track_table[60].vx = 31;
	simu_track_table[60].vy = 189;
	simu_track_table[60].vz = 114;
	simu_track_table[60].hdg = 261;

	simu_track_table[61].lat = 465188444;
	simu_track_table[61].lon = 65662692;
	simu_track_table[61].alt = 422;
	simu_track_table[61].relative_alt = -22;
	simu_track_table[61].vx = 32;
	simu_track_table[61].vy = 189;
	simu_track_table[61].vz = 114;
	simu_track_table[61].hdg = 261;

	simu_track_table[62].lat = 465188450;
	simu_track_table[62].lon = 65662756;
	simu_track_table[62].alt = 422;
	simu_track_table[62].relative_alt = -22;
	simu_track_table[62].vx = 28;
	simu_track_table[62].vy = 189;
	simu_track_table[62].vz = 114;
	simu_track_table[62].hdg = 262;

	simu_track_table[63].lat = 465188454;
	simu_track_table[63].lon = 65662821;
	simu_track_table[63].alt = 422;
	simu_track_table[63].relative_alt = -22;
	simu_track_table[63].vx = 22;
	simu_track_table[63].vy = 188;
	simu_track_table[63].vz = 114;
	simu_track_table[63].hdg = 263;

	simu_track_table[64].lat = 465188457;
	simu_track_table[64].lon = 65662884;
	simu_track_table[64].alt = 422;
	simu_track_table[64].relative_alt = -22;
	simu_track_table[64].vx = 16;
	simu_track_table[64].vy = 188;
	simu_track_table[64].vz = 114;
	simu_track_table[64].hdg = 265;

	simu_track_table[65].lat = 465188458;
	simu_track_table[65].lon = 65662949;
	simu_track_table[65].alt = 421;
	simu_track_table[65].relative_alt = -21;
	simu_track_table[65].vx = 13;
	simu_track_table[65].vy = 188;
	simu_track_table[65].vz = 114;
	simu_track_table[65].hdg = 266;

	simu_track_table[66].lat = 465188460;
	simu_track_table[66].lon = 65663012;
	simu_track_table[66].alt = 421;
	simu_track_table[66].relative_alt = -21;
	simu_track_table[66].vx = 13;
	simu_track_table[66].vy = 189;
	simu_track_table[66].vz = 114;
	simu_track_table[66].hdg = 266;

	simu_track_table[67].lat = 465188462;
	simu_track_table[67].lon = 65663077;
	simu_track_table[67].alt = 421;
	simu_track_table[67].relative_alt = -21;
	simu_track_table[67].vx = 15;
	simu_track_table[67].vy = 190;
	simu_track_table[67].vz = 114;
	simu_track_table[67].hdg = 265;

	simu_track_table[68].lat = 465188464;
	simu_track_table[68].lon = 65663141;
	simu_track_table[68].alt = 420;
	simu_track_table[68].relative_alt = -20;
	simu_track_table[68].vx = 19;
	simu_track_table[68].vy = 191;
	simu_track_table[68].vz = 114;
	simu_track_table[68].hdg = 264;

	simu_track_table[69].lat = 465188467;
	simu_track_table[69].lon = 65663206;
	simu_track_table[69].alt = 420;
	simu_track_table[69].relative_alt = -20;
	simu_track_table[69].vx = 22;
	simu_track_table[69].vy = 192;
	simu_track_table[69].vz = 114;
	simu_track_table[69].hdg = 264;

	simu_track_table[70].lat = 465188470;
	simu_track_table[70].lon = 65663270;
	simu_track_table[70].alt = 420;
	simu_track_table[70].relative_alt = -20;
	simu_track_table[70].vx = 23;
	simu_track_table[70].vy = 192;
	simu_track_table[70].vz = 114;
	simu_track_table[70].hdg = 263;

	simu_track_table[71].lat = 465188474;
	simu_track_table[71].lon = 65663336;
	simu_track_table[71].alt = 420;
	simu_track_table[71].relative_alt = -20;
	simu_track_table[71].vx = 23;
	simu_track_table[71].vy = 193;
	simu_track_table[71].vz = 114;
	simu_track_table[71].hdg = 263;

	simu_track_table[72].lat = 465188478;
	simu_track_table[72].lon = 65663400;
	simu_track_table[72].alt = 419;
	simu_track_table[72].relative_alt = -19;
	simu_track_table[72].vx = 22;
	simu_track_table[72].vy = 193;
	simu_track_table[72].vz = 114;
	simu_track_table[72].hdg = 263;

	simu_track_table[73].lat = 465188481;
	simu_track_table[73].lon = 65663466;
	simu_track_table[73].alt = 419;
	simu_track_table[73].relative_alt = -19;
	simu_track_table[73].vx = 21;
	simu_track_table[73].vy = 193;
	simu_track_table[73].vz = 114;
	simu_track_table[73].hdg = 264;

	simu_track_table[74].lat = 465188484;
	simu_track_table[74].lon = 65663530;
	simu_track_table[74].alt = 419;
	simu_track_table[74].relative_alt = -19;
	simu_track_table[74].vx = 19;
	simu_track_table[74].vy = 194;
	simu_track_table[74].vz = 114;
	simu_track_table[74].hdg = 264;

	simu_track_table[75].lat = 465188487;
	simu_track_table[75].lon = 65663597;
	simu_track_table[75].alt = 418;
	simu_track_table[75].relative_alt = -18;
	simu_track_table[75].vx = 19;
	simu_track_table[75].vy = 195;
	simu_track_table[75].vz = 114;
	simu_track_table[75].hdg = 264;

	simu_track_table[76].lat = 465188489;
	simu_track_table[76].lon = 65663661;
	simu_track_table[76].alt = 418;
	simu_track_table[76].relative_alt = -18;
	simu_track_table[76].vx = 19;
	simu_track_table[76].vy = 195;
	simu_track_table[76].vz = 114;
	simu_track_table[76].hdg = 264;

	simu_track_table[77].lat = 465188492;
	simu_track_table[77].lon = 65663728;
	simu_track_table[77].alt = 418;
	simu_track_table[77].relative_alt = -18;
	simu_track_table[77].vx = 20;
	simu_track_table[77].vy = 196;
	simu_track_table[77].vz = 114;
	simu_track_table[77].hdg = 264;

	simu_track_table[78].lat = 465188495;
	simu_track_table[78].lon = 65663792;
	simu_track_table[78].alt = 418;
	simu_track_table[78].relative_alt = -18;
	simu_track_table[78].vx = 21;
	simu_track_table[78].vy = 197;
	simu_track_table[78].vz = 114;
	simu_track_table[78].hdg = 264;

	simu_track_table[79].lat = 465188499;
	simu_track_table[79].lon = 65663859;
	simu_track_table[79].alt = 417;
	simu_track_table[79].relative_alt = -17;
	simu_track_table[79].vx = 22;
	simu_track_table[79].vy = 198;
	simu_track_table[79].vz = 114;
	simu_track_table[79].hdg = 264;

	simu_track_table[80].lat = 465188502;
	simu_track_table[80].lon = 65663924;
	simu_track_table[80].alt = 417;
	simu_track_table[80].relative_alt = -17;
	simu_track_table[80].vx = 23;
	simu_track_table[80].vy = 199;
	simu_track_table[80].vz = 114;
	simu_track_table[80].hdg = 263;

	simu_track_table[81].lat = 465188506;
	simu_track_table[81].lon = 65663992;
	simu_track_table[81].alt = 417;
	simu_track_table[81].relative_alt = -17;
	simu_track_table[81].vx = 23;
	simu_track_table[81].vy = 199;
	simu_track_table[81].vz = 114;
	simu_track_table[81].hdg = 263;

	simu_track_table[82].lat = 465188509;
	simu_track_table[82].lon = 65664057;
	simu_track_table[82].alt = 416;
	simu_track_table[82].relative_alt = -16;
	simu_track_table[82].vx = 23;
	simu_track_table[82].vy = 200;
	simu_track_table[82].vz = 114;
	simu_track_table[82].hdg = 263;

	simu_track_table[83].lat = 465188513;
	simu_track_table[83].lon = 65664126;
	simu_track_table[83].alt = 416;
	simu_track_table[83].relative_alt = -16;
	simu_track_table[83].vx = 23;
	simu_track_table[83].vy = 201;
	simu_track_table[83].vz = 114;
	simu_track_table[83].hdg = 263;

	simu_track_table[84].lat = 465188517;
	simu_track_table[84].lon = 65664191;
	simu_track_table[84].alt = 416;
	simu_track_table[84].relative_alt = -16;
	simu_track_table[84].vx = 23;
	simu_track_table[84].vy = 202;
	simu_track_table[84].vz = 114;
	simu_track_table[84].hdg = 263;

	simu_track_table[85].lat = 465188520;
	simu_track_table[85].lon = 65664260;
	simu_track_table[85].alt = 416;
	simu_track_table[85].relative_alt = -16;
	simu_track_table[85].vx = 23;
	simu_track_table[85].vy = 203;
	simu_track_table[85].vz = 114;
	simu_track_table[85].hdg = 263;

	simu_track_table[86].lat = 465188524;
	simu_track_table[86].lon = 65664326;
	simu_track_table[86].alt = 415;
	simu_track_table[86].relative_alt = -15;
	simu_track_table[86].vx = 24;
	simu_track_table[86].vy = 204;
	simu_track_table[86].vz = 114;
	simu_track_table[86].hdg = 263;

	simu_track_table[87].lat = 465188528;
	simu_track_table[87].lon = 65664396;
	simu_track_table[87].alt = 415;
	simu_track_table[87].relative_alt = -15;
	simu_track_table[87].vx = 24;
	simu_track_table[87].vy = 206;
	simu_track_table[87].vz = 114;
	simu_track_table[87].hdg = 263;

	simu_track_table[88].lat = 465188532;
	simu_track_table[88].lon = 65664463;
	simu_track_table[88].alt = 415;
	simu_track_table[88].relative_alt = -15;
	simu_track_table[88].vx = 25;
	simu_track_table[88].vy = 207;
	simu_track_table[88].vz = 114;
	simu_track_table[88].hdg = 263;

	simu_track_table[89].lat = 465188536;
	simu_track_table[89].lon = 65664533;
	simu_track_table[89].alt = 415;
	simu_track_table[89].relative_alt = -15;
	simu_track_table[89].vx = 26;
	simu_track_table[89].vy = 208;
	simu_track_table[89].vz = 114;
	simu_track_table[89].hdg = 263;

	simu_track_table[90].lat = 465188540;
	simu_track_table[90].lon = 65664601;
	simu_track_table[90].alt = 414;
	simu_track_table[90].relative_alt = -14;
	simu_track_table[90].vx = 26;
	simu_track_table[90].vy = 210;
	simu_track_table[90].vz = 114;
	simu_track_table[90].hdg = 263;

	simu_track_table[91].lat = 465188544;
	simu_track_table[91].lon = 65664672;
	simu_track_table[91].alt = 414;
	simu_track_table[91].relative_alt = -14;
	simu_track_table[91].vx = 27;
	simu_track_table[91].vy = 211;
	simu_track_table[91].vz = 114;
	simu_track_table[91].hdg = 263;

	simu_track_table[92].lat = 465188549;
	simu_track_table[92].lon = 65664741;
	simu_track_table[92].alt = 414;
	simu_track_table[92].relative_alt = -14;
	simu_track_table[92].vx = 28;
	simu_track_table[92].vy = 213;
	simu_track_table[92].vz = 114;
	simu_track_table[92].hdg = 263;

	simu_track_table[93].lat = 465188554;
	simu_track_table[93].lon = 65664813;
	simu_track_table[93].alt = 413;
	simu_track_table[93].relative_alt = -13;
	simu_track_table[93].vx = 28;
	simu_track_table[93].vy = 215;
	simu_track_table[93].vz = 114;
	simu_track_table[93].hdg = 263;

	simu_track_table[94].lat = 465188558;
	simu_track_table[94].lon = 65664883;
	simu_track_table[94].alt = 413;
	simu_track_table[94].relative_alt = -13;
	simu_track_table[94].vx = 29;
	simu_track_table[94].vy = 217;
	simu_track_table[94].vz = 114;
	simu_track_table[94].hdg = 262;

	simu_track_table[95].lat = 465188564;
	simu_track_table[95].lon = 65664956;
	simu_track_table[95].alt = 413;
	simu_track_table[95].relative_alt = -13;
	simu_track_table[95].vx = 29;
	simu_track_table[95].vy = 219;
	simu_track_table[95].vz = 114;
	simu_track_table[95].hdg = 262;

	simu_track_table[96].lat = 465188569;
	simu_track_table[96].lon = 65665027;
	simu_track_table[96].alt = 413;
	simu_track_table[96].relative_alt = -13;
	simu_track_table[96].vx = 30;
	simu_track_table[96].vy = 222;
	simu_track_table[96].vz = 114;
	simu_track_table[96].hdg = 262;

	simu_track_table[97].lat = 465188574;
	simu_track_table[97].lon = 65665102;
	simu_track_table[97].alt = 412;
	simu_track_table[97].relative_alt = -12;
	simu_track_table[97].vx = 31;
	simu_track_table[97].vy = 224;
	simu_track_table[97].vz = 114;
	simu_track_table[97].hdg = 262;

	simu_track_table[98].lat = 465188580;
	simu_track_table[98].lon = 65665176;
	simu_track_table[98].alt = 412;
	simu_track_table[98].relative_alt = -12;
	simu_track_table[98].vx = 33;
	simu_track_table[98].vy = 228;
	simu_track_table[98].vz = 112;
	simu_track_table[98].hdg = 262;

	simu_track_table[99].lat = 465188585;
	simu_track_table[99].lon = 65665253;
	simu_track_table[99].alt = 412;
	simu_track_table[99].relative_alt = -12;
	simu_track_table[99].vx = 25;
	simu_track_table[99].vy = 234;
	simu_track_table[99].vz = 70;
	simu_track_table[99].hdg = 264;

	simu_track_table[100].lat = 465188585;
	simu_track_table[100].lon = 65665331;
	simu_track_table[100].alt = 412;
	simu_track_table[100].relative_alt = -12;
	simu_track_table[100].vx = -16;
	simu_track_table[100].vy = 246;
	simu_track_table[100].vz = 58;
	simu_track_table[100].hdg = 274;

	simu_track_table[101].lat = 465188573;
	simu_track_table[101].lon = 65665415;
	simu_track_table[101].alt = 411;
	simu_track_table[101].relative_alt = -11;
	simu_track_table[101].vx = -84;
	simu_track_table[101].vy = 258;
	simu_track_table[101].vz = 57;
	simu_track_table[101].hdg = 288;

	simu_track_table[102].lat = 465188545;
	simu_track_table[102].lon = 65665501;
	simu_track_table[102].alt = 411;
	simu_track_table[102].relative_alt = -11;
	simu_track_table[102].vx = -148;
	simu_track_table[102].vy = 267;
	simu_track_table[102].vz = 57;
	simu_track_table[102].hdg = 299;

	simu_track_table[103].lat = 465188503;
	simu_track_table[103].lon = 65665591;
	simu_track_table[103].alt = 411;
	simu_track_table[103].relative_alt = -11;
	simu_track_table[103].vx = -189;
	simu_track_table[103].vy = 272;
	simu_track_table[103].vz = 53;
	simu_track_table[103].hdg = 305;

	simu_track_table[104].lat = 465188457;
	simu_track_table[104].lon = 65665680;
	simu_track_table[104].alt = 411;
	simu_track_table[104].relative_alt = -11;
	simu_track_table[104].vx = -198;
	simu_track_table[104].vy = 273;
	simu_track_table[104].vz = 47;
	simu_track_table[104].hdg = 306;

	simu_track_table[105].lat = 465188409;
	simu_track_table[105].lon = 65665773;
	simu_track_table[105].alt = 411;
	simu_track_table[105].relative_alt = -11;
	simu_track_table[105].vx = -185;
	simu_track_table[105].vy = 273;
	simu_track_table[105].vz = 42;
	simu_track_table[105].hdg = 304;

	simu_track_table[106].lat = 465188366;
	simu_track_table[106].lon = 65665862;
	simu_track_table[106].alt = 411;
	simu_track_table[106].relative_alt = -11;
	simu_track_table[106].vx = -164;
	simu_track_table[106].vy = 273;
	simu_track_table[106].vz = 39;
	simu_track_table[106].hdg = 301;

	simu_track_table[107].lat = 465188326;
	simu_track_table[107].lon = 65665953;
	simu_track_table[107].alt = 411;
	simu_track_table[107].relative_alt = -11;
	simu_track_table[107].vx = -146;
	simu_track_table[107].vy = 271;
	simu_track_table[107].vz = 37;
	simu_track_table[107].hdg = 298;

	simu_track_table[108].lat = 465188291;
	simu_track_table[108].lon = 65666042;
	simu_track_table[108].alt = 411;
	simu_track_table[108].relative_alt = -11;
	simu_track_table[108].vx = -136;
	simu_track_table[108].vy = 270;
	simu_track_table[108].vz = 37;
	simu_track_table[108].hdg = 297;

	simu_track_table[109].lat = 465188255;
	simu_track_table[109].lon = 65666133;
	simu_track_table[109].alt = 411;
	simu_track_table[109].relative_alt = -11;
	simu_track_table[109].vx = -136;
	simu_track_table[109].vy = 269;
	simu_track_table[109].vz = 37;
	simu_track_table[109].hdg = 297;

	simu_track_table[110].lat = 465188221;
	simu_track_table[110].lon = 65666220;
	simu_track_table[110].alt = 410;
	simu_track_table[110].relative_alt = -10;
	simu_track_table[110].vx = -141;
	simu_track_table[110].vy = 269;
	simu_track_table[110].vz = 37;
	simu_track_table[110].hdg = 298;

	simu_track_table[111].lat = 465188184;
	simu_track_table[111].lon = 65666311;
	simu_track_table[111].alt = 410;
	simu_track_table[111].relative_alt = -10;
	simu_track_table[111].vx = -146;
	simu_track_table[111].vy = 269;
	simu_track_table[111].vz = 37;
	simu_track_table[111].hdg = 299;

	simu_track_table[112].lat = 465188147;
	simu_track_table[112].lon = 65666398;
	simu_track_table[112].alt = 410;
	simu_track_table[112].relative_alt = -10;
	simu_track_table[112].vx = -149;
	simu_track_table[112].vy = 269;
	simu_track_table[112].vz = 36;
	simu_track_table[112].hdg = 299;

	simu_track_table[113].lat = 465188109;
	simu_track_table[113].lon = 65666489;
	simu_track_table[113].alt = 410;
	simu_track_table[113].relative_alt = -10;
	simu_track_table[113].vx = -148;
	simu_track_table[113].vy = 269;
	simu_track_table[113].vz = 35;
	simu_track_table[113].hdg = 299;

	simu_track_table[114].lat = 465188072;
	simu_track_table[114].lon = 65666576;
	simu_track_table[114].alt = 410;
	simu_track_table[114].relative_alt = -10;
	simu_track_table[114].vx = -144;
	simu_track_table[114].vy = 269;
	simu_track_table[114].vz = 34;
	simu_track_table[114].hdg = 298;

	simu_track_table[115].lat = 465188036;
	simu_track_table[115].lon = 65666666;
	simu_track_table[115].alt = 410;
	simu_track_table[115].relative_alt = -10;
	simu_track_table[115].vx = -139;
	simu_track_table[115].vy = 268;
	simu_track_table[115].vz = 33;
	simu_track_table[115].hdg = 297;

	simu_track_table[116].lat = 465188001;
	simu_track_table[116].lon = 65666754;
	simu_track_table[116].alt = 410;
	simu_track_table[116].relative_alt = -10;
	simu_track_table[116].vx = -136;
	simu_track_table[116].vy = 269;
	simu_track_table[116].vz = 32;
	simu_track_table[116].hdg = 297;

	simu_track_table[117].lat = 465187967;
	simu_track_table[117].lon = 65666842;
	simu_track_table[117].alt = 410;
	simu_track_table[117].relative_alt = -10;
	simu_track_table[117].vx = -133;
	simu_track_table[117].vy = 269;
	simu_track_table[117].vz = 31;
	simu_track_table[117].hdg = 296;

	simu_track_table[118].lat = 465187933;
	simu_track_table[118].lon = 65666931;
	simu_track_table[118].alt = 410;
	simu_track_table[118].relative_alt = -10;
	simu_track_table[118].vx = -132;
	simu_track_table[118].vy = 269;
	simu_track_table[118].vz = 30;
	simu_track_table[118].hdg = 296;

	simu_track_table[119].lat = 465187900;
	simu_track_table[119].lon = 65667019;
	simu_track_table[119].alt = 410;
	simu_track_table[119].relative_alt = -10;
	simu_track_table[119].vx = -132;
	simu_track_table[119].vy = 269;
	simu_track_table[119].vz = 29;
	simu_track_table[119].hdg = 296;

	simu_track_table[120].lat = 465187866;
	simu_track_table[120].lon = 65667107;
	simu_track_table[120].alt = 410;
	simu_track_table[120].relative_alt = -10;
	simu_track_table[120].vx = -132;
	simu_track_table[120].vy = 269;
	simu_track_table[120].vz = 28;
	simu_track_table[120].hdg = 296;

	simu_track_table[121].lat = 465187833;
	simu_track_table[121].lon = 65667195;
	simu_track_table[121].alt = 410;
	simu_track_table[121].relative_alt = -10;
	simu_track_table[121].vx = -132;
	simu_track_table[121].vy = 269;
	simu_track_table[121].vz = 27;
	simu_track_table[121].hdg = 296;

	simu_track_table[122].lat = 465187800;
	simu_track_table[122].lon = 65667283;
	simu_track_table[122].alt = 409;
	simu_track_table[122].relative_alt = -9;
	simu_track_table[122].vx = -132;
	simu_track_table[122].vy = 269;
	simu_track_table[122].vz = 25;
	simu_track_table[122].hdg = 296;

	simu_track_table[123].lat = 465187767;
	simu_track_table[123].lon = 65667371;
	simu_track_table[123].alt = 409;
	simu_track_table[123].relative_alt = -9;
	simu_track_table[123].vx = -131;
	simu_track_table[123].vy = 269;
	simu_track_table[123].vz = 24;
	simu_track_table[123].hdg = 296;

	simu_track_table[124].lat = 465187734;
	simu_track_table[124].lon = 65667459;
	simu_track_table[124].alt = 409;
	simu_track_table[124].relative_alt = -9;
	simu_track_table[124].vx = -130;
	simu_track_table[124].vy = 269;
	simu_track_table[124].vz = 23;
	simu_track_table[124].hdg = 296;

	simu_track_table[125].lat = 465187702;
	simu_track_table[125].lon = 65667547;
	simu_track_table[125].alt = 409;
	simu_track_table[125].relative_alt = -9;
	simu_track_table[125].vx = -129;
	simu_track_table[125].vy = 269;
	simu_track_table[125].vz = 21;
	simu_track_table[125].hdg = 296;

	simu_track_table[126].lat = 465187669;
	simu_track_table[126].lon = 65667635;
	simu_track_table[126].alt = 409;
	simu_track_table[126].relative_alt = -9;
	simu_track_table[126].vx = -128;
	simu_track_table[126].vy = 270;
	simu_track_table[126].vz = 19;
	simu_track_table[126].hdg = 295;

	simu_track_table[127].lat = 465187637;
	simu_track_table[127].lon = 65667723;
	simu_track_table[127].alt = 409;
	simu_track_table[127].relative_alt = -9;
	simu_track_table[127].vx = -127;
	simu_track_table[127].vy = 270;
	simu_track_table[127].vz = 18;
	simu_track_table[127].hdg = 295;

	simu_track_table[128].lat = 465187606;
	simu_track_table[128].lon = 65667811;
	simu_track_table[128].alt = 409;
	simu_track_table[128].relative_alt = -9;
	simu_track_table[128].vx = -126;
	simu_track_table[128].vy = 270;
	simu_track_table[128].vz = 16;
	simu_track_table[128].hdg = 295;

	simu_track_table[129].lat = 465187574;
	simu_track_table[129].lon = 65667899;
	simu_track_table[129].alt = 409;
	simu_track_table[129].relative_alt = -9;
	simu_track_table[129].vx = -126;
	simu_track_table[129].vy = 270;
	simu_track_table[129].vz = 14;
	simu_track_table[129].hdg = 295;

	simu_track_table[130].lat = 465187543;
	simu_track_table[130].lon = 65667987;
	simu_track_table[130].alt = 409;
	simu_track_table[130].relative_alt = -9;
	simu_track_table[130].vx = -125;
	simu_track_table[130].vy = 270;
	simu_track_table[130].vz = 11;
	simu_track_table[130].hdg = 295;

	simu_track_table[131].lat = 465187512;
	simu_track_table[131].lon = 65668075;
	simu_track_table[131].alt = 409;
	simu_track_table[131].relative_alt = -9;
	simu_track_table[131].vx = -124;
	simu_track_table[131].vy = 270;
	simu_track_table[131].vz = 9;
	simu_track_table[131].hdg = 295;

	simu_track_table[132].lat = 465187481;
	simu_track_table[132].lon = 65668162;
	simu_track_table[132].alt = 409;
	simu_track_table[132].relative_alt = -9;
	simu_track_table[132].vx = -123;
	simu_track_table[132].vy = 270;
	simu_track_table[132].vz = 6;
	simu_track_table[132].hdg = 294;

	simu_track_table[133].lat = 465187451;
	simu_track_table[133].lon = 65668250;
	simu_track_table[133].alt = 409;
	simu_track_table[133].relative_alt = -9;
	simu_track_table[133].vx = -121;
	simu_track_table[133].vy = 271;
	simu_track_table[133].vz = 3;
	simu_track_table[133].hdg = 294;

	simu_track_table[134].lat = 465187421;
	simu_track_table[134].lon = 65668338;
	simu_track_table[134].alt = 409;
	simu_track_table[134].relative_alt = -9;
	simu_track_table[134].vx = -120;
	simu_track_table[134].vy = 271;
	simu_track_table[134].vz = -0;
	simu_track_table[134].hdg = 294;

	simu_track_table[135].lat = 465187391;
	simu_track_table[135].lon = 65668425;
	simu_track_table[135].alt = 409;
	simu_track_table[135].relative_alt = -9;
	simu_track_table[135].vx = -119;
	simu_track_table[135].vy = 271;
	simu_track_table[135].vz = -4;
	simu_track_table[135].hdg = 294;

	simu_track_table[136].lat = 465187362;
	simu_track_table[136].lon = 65668513;
	simu_track_table[136].alt = 409;
	simu_track_table[136].relative_alt = -9;
	simu_track_table[136].vx = -117;
	simu_track_table[136].vy = 271;
	simu_track_table[136].vz = -9;
	simu_track_table[136].hdg = 293;

	simu_track_table[137].lat = 465187333;
	simu_track_table[137].lon = 65668600;
	simu_track_table[137].alt = 409;
	simu_track_table[137].relative_alt = -9;
	simu_track_table[137].vx = -115;
	simu_track_table[137].vy = 266;
	simu_track_table[137].vz = -28;
	simu_track_table[137].hdg = 293;

	simu_track_table[138].lat = 465187305;
	simu_track_table[138].lon = 65668681;
	simu_track_table[138].alt = 409;
	simu_track_table[138].relative_alt = -9;
	simu_track_table[138].vx = -109;
	simu_track_table[138].vy = 229;
	simu_track_table[138].vz = -43;
	simu_track_table[138].hdg = 296;

	simu_track_table[139].lat = 465187279;
	simu_track_table[139].lon = 65668742;
	simu_track_table[139].alt = 409;
	simu_track_table[139].relative_alt = -9;
	simu_track_table[139].vx = -95;
	simu_track_table[139].vy = 147;
	simu_track_table[139].vz = -47;
	simu_track_table[139].hdg = 303;

	simu_track_table[140].lat = 465187257;
	simu_track_table[140].lon = 65668773;
	simu_track_table[140].alt = 409;
	simu_track_table[140].relative_alt = -9;
	simu_track_table[140].vx = -76;
	simu_track_table[140].vy = 54;
	simu_track_table[140].vz = -45;
	simu_track_table[140].hdg = 325;

	simu_track_table[141].lat = 465187240;
	simu_track_table[141].lon = 65668776;
	simu_track_table[141].alt = 410;
	simu_track_table[141].relative_alt = -10;
	simu_track_table[141].vx = -62;
	simu_track_table[141].vy = -14;
	simu_track_table[141].vz = -43;
	simu_track_table[141].hdg = 13;

	simu_track_table[142].lat = 465187224;
	simu_track_table[142].lon = 65668761;
	simu_track_table[142].alt = 410;
	simu_track_table[142].relative_alt = -10;
	simu_track_table[142].vx = -55;
	simu_track_table[142].vy = -45;
	simu_track_table[142].vz = -44;
	simu_track_table[142].hdg = 39;

	simu_track_table[143].lat = 465187210;
	simu_track_table[143].lon = 65668741;
	simu_track_table[143].alt = 410;
	simu_track_table[143].relative_alt = -10;
	simu_track_table[143].vx = -57;
	simu_track_table[143].vy = -45;
	simu_track_table[143].vz = -45;
	simu_track_table[143].hdg = 38;

	simu_track_table[144].lat = 465187195;
	simu_track_table[144].lon = 65668722;
	simu_track_table[144].alt = 410;
	simu_track_table[144].relative_alt = -10;
	simu_track_table[144].vx = -62;
	simu_track_table[144].vy = -31;
	simu_track_table[144].vz = -46;
	simu_track_table[144].hdg = 26;

	simu_track_table[145].lat = 465187178;
	simu_track_table[145].lon = 65668707;
	simu_track_table[145].alt = 410;
	simu_track_table[145].relative_alt = -10;
	simu_track_table[145].vx = -65;
	simu_track_table[145].vy = -16;
	simu_track_table[145].vz = -46;
	simu_track_table[145].hdg = 14;

	simu_track_table[146].lat = 465187162;
	simu_track_table[146].lon = 65668695;
	simu_track_table[146].alt = 410;
	simu_track_table[146].relative_alt = -10;
	simu_track_table[146].vx = -63;
	simu_track_table[146].vy = -8;
	simu_track_table[146].vz = -46;
	simu_track_table[146].hdg = 7;

	simu_track_table[147].lat = 465187147;
	simu_track_table[147].lon = 65668686;
	simu_track_table[147].alt = 410;
	simu_track_table[147].relative_alt = -10;
	simu_track_table[147].vx = -59;
	simu_track_table[147].vy = -6;
	simu_track_table[147].vz = -46;
	simu_track_table[147].hdg = 6;

	simu_track_table[148].lat = 465187133;
	simu_track_table[148].lon = 65668676;
	simu_track_table[148].alt = 410;
	simu_track_table[148].relative_alt = -10;
	simu_track_table[148].vx = -54;
	simu_track_table[148].vy = -7;
	simu_track_table[148].vz = -46;
	simu_track_table[148].hdg = 7;

	simu_track_table[149].lat = 465187120;
	simu_track_table[149].lon = 65668666;
	simu_track_table[149].alt = 410;
	simu_track_table[149].relative_alt = -10;
	simu_track_table[149].vx = -52;
	simu_track_table[149].vy = -8;
	simu_track_table[149].vz = -46;
	simu_track_table[149].hdg = 9;

	simu_track_table[150].lat = 465187108;
	simu_track_table[150].lon = 65668656;
	simu_track_table[150].alt = 411;
	simu_track_table[150].relative_alt = -11;
	simu_track_table[150].vx = -52;
	simu_track_table[150].vy = -8;
	simu_track_table[150].vz = -46;
	simu_track_table[150].hdg = 8;

	simu_track_table[151].lat = 465187095;
	simu_track_table[151].lon = 65668646;
	simu_track_table[151].alt = 411;
	simu_track_table[151].relative_alt = -11;
	simu_track_table[151].vx = -54;
	simu_track_table[151].vy = -6;
	simu_track_table[151].vz = -46;
	simu_track_table[151].hdg = 6;

	simu_track_table[152].lat = 465187082;
	simu_track_table[152].lon = 65668637;
	simu_track_table[152].alt = 411;
	simu_track_table[152].relative_alt = -11;
	simu_track_table[152].vx = -56;
	simu_track_table[152].vy = -2;
	simu_track_table[152].vz = -46;
	simu_track_table[152].hdg = 2;

	simu_track_table[153].lat = 465187068;
	simu_track_table[153].lon = 65668630;
	simu_track_table[153].alt = 411;
	simu_track_table[153].relative_alt = -11;
	simu_track_table[153].vx = -58;
	simu_track_table[153].vy = 1;
	simu_track_table[153].vz = -46;
	simu_track_table[153].hdg = 359;

	simu_track_table[154].lat = 465187055;
	simu_track_table[154].lon = 65668623;
	simu_track_table[154].alt = 411;
	simu_track_table[154].relative_alt = -11;
	simu_track_table[154].vx = -59;
	simu_track_table[154].vy = 3;
	simu_track_table[154].vz = -46;
	simu_track_table[154].hdg = 357;

	simu_track_table[155].lat = 465187041;
	simu_track_table[155].lon = 65668617;
	simu_track_table[155].alt = 411;
	simu_track_table[155].relative_alt = -11;
	simu_track_table[155].vx = -59;
	simu_track_table[155].vy = 5;
	simu_track_table[155].vz = -46;
	simu_track_table[155].hdg = 355;

	simu_track_table[156].lat = 465187028;
	simu_track_table[156].lon = 65668612;
	simu_track_table[156].alt = 411;
	simu_track_table[156].relative_alt = -11;
	simu_track_table[156].vx = -60;
	simu_track_table[156].vy = 6;
	simu_track_table[156].vz = -46;
	simu_track_table[156].hdg = 354;

	simu_track_table[157].lat = 465187014;
	simu_track_table[157].lon = 65668607;
	simu_track_table[157].alt = 411;
	simu_track_table[157].relative_alt = -11;
	simu_track_table[157].vx = -60;
	simu_track_table[157].vy = 7;
	simu_track_table[157].vz = -46;
	simu_track_table[157].hdg = 354;

	simu_track_table[158].lat = 465187001;
	simu_track_table[158].lon = 65668603;
	simu_track_table[158].alt = 411;
	simu_track_table[158].relative_alt = -11;
	simu_track_table[158].vx = -60;
	simu_track_table[158].vy = 7;
	simu_track_table[158].vz = -46;
	simu_track_table[158].hdg = 353;

	simu_track_table[159].lat = 465186988;
	simu_track_table[159].lon = 65668599;
	simu_track_table[159].alt = 412;
	simu_track_table[159].relative_alt = -12;
	simu_track_table[159].vx = -60;
	simu_track_table[159].vy = 8;
	simu_track_table[159].vz = -46;
	simu_track_table[159].hdg = 352;

	simu_track_table[160].lat = 465186974;
	simu_track_table[160].lon = 65668595;
	simu_track_table[160].alt = 412;
	simu_track_table[160].relative_alt = -12;
	simu_track_table[160].vx = -61;
	simu_track_table[160].vy = 9;
	simu_track_table[160].vz = -46;
	simu_track_table[160].hdg = 352;

	simu_track_table[161].lat = 465186961;
	simu_track_table[161].lon = 65668591;
	simu_track_table[161].alt = 412;
	simu_track_table[161].relative_alt = -12;
	simu_track_table[161].vx = -61;
	simu_track_table[161].vy = 9;
	simu_track_table[161].vz = -46;
	simu_track_table[161].hdg = 351;

	simu_track_table[162].lat = 465186948;
	simu_track_table[162].lon = 65668588;
	simu_track_table[162].alt = 412;
	simu_track_table[162].relative_alt = -12;
	simu_track_table[162].vx = -61;
	simu_track_table[162].vy = 10;
	simu_track_table[162].vz = -46;
	simu_track_table[162].hdg = 351;

	simu_track_table[163].lat = 465186935;
	simu_track_table[163].lon = 65668585;
	simu_track_table[163].alt = 412;
	simu_track_table[163].relative_alt = -12;
	simu_track_table[163].vx = -61;
	simu_track_table[163].vy = 10;
	simu_track_table[163].vz = -46;
	simu_track_table[163].hdg = 350;

	simu_track_table[164].lat = 465186922;
	simu_track_table[164].lon = 65668582;
	simu_track_table[164].alt = 412;
	simu_track_table[164].relative_alt = -12;
	simu_track_table[164].vx = -61;
	simu_track_table[164].vy = 10;
	simu_track_table[164].vz = -46;
	simu_track_table[164].hdg = 350;

	simu_track_table[165].lat = 465186909;
	simu_track_table[165].lon = 65668580;
	simu_track_table[165].alt = 412;
	simu_track_table[165].relative_alt = -12;
	simu_track_table[165].vx = -60;
	simu_track_table[165].vy = 11;
	simu_track_table[165].vz = -46;
	simu_track_table[165].hdg = 350;

	simu_track_table[166].lat = 465186897;
	simu_track_table[166].lon = 65668577;
	simu_track_table[166].alt = 412;
	simu_track_table[166].relative_alt = -12;
	simu_track_table[166].vx = -60;
	simu_track_table[166].vy = 11;
	simu_track_table[166].vz = -46;
	simu_track_table[166].hdg = 350;

	simu_track_table[167].lat = 465186885;
	simu_track_table[167].lon = 65668575;
	simu_track_table[167].alt = 413;
	simu_track_table[167].relative_alt = -13;
	simu_track_table[167].vx = -60;
	simu_track_table[167].vy = 11;
	simu_track_table[167].vz = -46;
	simu_track_table[167].hdg = 350;

	simu_track_table[168].lat = 465186872;
	simu_track_table[168].lon = 65668573;
	simu_track_table[168].alt = 413;
	simu_track_table[168].relative_alt = -13;
	simu_track_table[168].vx = -60;
	simu_track_table[168].vy = 11;
	simu_track_table[168].vz = -46;
	simu_track_table[168].hdg = 349;

	simu_track_table[169].lat = 465186860;
	simu_track_table[169].lon = 65668571;
	simu_track_table[169].alt = 413;
	simu_track_table[169].relative_alt = -13;
	simu_track_table[169].vx = -60;
	simu_track_table[169].vy = 11;
	simu_track_table[169].vz = -46;
	simu_track_table[169].hdg = 349;

	simu_track_table[170].lat = 465186848;
	simu_track_table[170].lon = 65668569;
	simu_track_table[170].alt = 413;
	simu_track_table[170].relative_alt = -13;
	simu_track_table[170].vx = -59;
	simu_track_table[170].vy = 11;
	simu_track_table[170].vz = -46;
	simu_track_table[170].hdg = 349;

	simu_track_table[171].lat = 465186836;
	simu_track_table[171].lon = 65668567;
	simu_track_table[171].alt = 413;
	simu_track_table[171].relative_alt = -13;
	simu_track_table[171].vx = -59;
	simu_track_table[171].vy = 11;
	simu_track_table[171].vz = -46;
	simu_track_table[171].hdg = 349;

	simu_track_table[172].lat = 465186825;
	simu_track_table[172].lon = 65668565;
	simu_track_table[172].alt = 413;
	simu_track_table[172].relative_alt = -13;
	simu_track_table[172].vx = -59;
	simu_track_table[172].vy = 12;
	simu_track_table[172].vz = -46;
	simu_track_table[172].hdg = 349;

	simu_track_table[173].lat = 465186813;
	simu_track_table[173].lon = 65668564;
	simu_track_table[173].alt = 413;
	simu_track_table[173].relative_alt = -13;
	simu_track_table[173].vx = -59;
	simu_track_table[173].vy = 12;
	simu_track_table[173].vz = -46;
	simu_track_table[173].hdg = 349;

	simu_track_table[174].lat = 465186802;
	simu_track_table[174].lon = 65668562;
	simu_track_table[174].alt = 413;
	simu_track_table[174].relative_alt = -13;
	simu_track_table[174].vx = -59;
	simu_track_table[174].vy = 12;
	simu_track_table[174].vz = -46;
	simu_track_table[174].hdg = 349;

	simu_track_table[175].lat = 465186791;
	simu_track_table[175].lon = 65668561;
	simu_track_table[175].alt = 413;
	simu_track_table[175].relative_alt = -13;
	simu_track_table[175].vx = -59;
	simu_track_table[175].vy = 12;
	simu_track_table[175].vz = -46;
	simu_track_table[175].hdg = 349;

	simu_track_table[176].lat = 465186779;
	simu_track_table[176].lon = 65668560;
	simu_track_table[176].alt = 414;
	simu_track_table[176].relative_alt = -14;
	simu_track_table[176].vx = -58;
	simu_track_table[176].vy = 12;
	simu_track_table[176].vz = -46;
	simu_track_table[176].hdg = 349;

	simu_track_table[177].lat = 465186768;
	simu_track_table[177].lon = 65668558;
	simu_track_table[177].alt = 414;
	simu_track_table[177].relative_alt = -14;
	simu_track_table[177].vx = -58;
	simu_track_table[177].vy = 12;
	simu_track_table[177].vz = -46;
	simu_track_table[177].hdg = 348;

	simu_track_table[178].lat = 465186758;
	simu_track_table[178].lon = 65668557;
	simu_track_table[178].alt = 414;
	simu_track_table[178].relative_alt = -14;
	simu_track_table[178].vx = -58;
	simu_track_table[178].vy = 12;
	simu_track_table[178].vz = -46;
	simu_track_table[178].hdg = 348;

	simu_track_table[179].lat = 465186747;
	simu_track_table[179].lon = 65668556;
	simu_track_table[179].alt = 414;
	simu_track_table[179].relative_alt = -14;
	simu_track_table[179].vx = -58;
	simu_track_table[179].vy = 12;
	simu_track_table[179].vz = -46;
	simu_track_table[179].hdg = 348;

	simu_track_table[180].lat = 465186736;
	simu_track_table[180].lon = 65668555;
	simu_track_table[180].alt = 414;
	simu_track_table[180].relative_alt = -14;
	simu_track_table[180].vx = -58;
	simu_track_table[180].vy = 12;
	simu_track_table[180].vz = -46;
	simu_track_table[180].hdg = 348;

	simu_track_table[181].lat = 465186725;
	simu_track_table[181].lon = 65668555;
	simu_track_table[181].alt = 414;
	simu_track_table[181].relative_alt = -14;
	simu_track_table[181].vx = -58;
	simu_track_table[181].vy = 12;
	simu_track_table[181].vz = -46;
	simu_track_table[181].hdg = 348;

	simu_track_table[182].lat = 465186715;
	simu_track_table[182].lon = 65668554;
	simu_track_table[182].alt = 414;
	simu_track_table[182].relative_alt = -14;
	simu_track_table[182].vx = -57;
	simu_track_table[182].vy = 12;
	simu_track_table[182].vz = -46;
	simu_track_table[182].hdg = 348;

	simu_track_table[183].lat = 465186705;
	simu_track_table[183].lon = 65668553;
	simu_track_table[183].alt = 414;
	simu_track_table[183].relative_alt = -14;
	simu_track_table[183].vx = -57;
	simu_track_table[183].vy = 12;
	simu_track_table[183].vz = -46;
	simu_track_table[183].hdg = 348;

	simu_track_table[184].lat = 465186695;
	simu_track_table[184].lon = 65668553;
	simu_track_table[184].alt = 415;
	simu_track_table[184].relative_alt = -15;
	simu_track_table[184].vx = -57;
	simu_track_table[184].vy = 12;
	simu_track_table[184].vz = -46;
	simu_track_table[184].hdg = 348;

	simu_track_table[185].lat = 465186684;
	simu_track_table[185].lon = 65668552;
	simu_track_table[185].alt = 415;
	simu_track_table[185].relative_alt = -15;
	simu_track_table[185].vx = -57;
	simu_track_table[185].vy = 12;
	simu_track_table[185].vz = -46;
	simu_track_table[185].hdg = 348;

	simu_track_table[186].lat = 465186674;
	simu_track_table[186].lon = 65668552;
	simu_track_table[186].alt = 415;
	simu_track_table[186].relative_alt = -15;
	simu_track_table[186].vx = -57;
	simu_track_table[186].vy = 12;
	simu_track_table[186].vz = -46;
	simu_track_table[186].hdg = 348;

	simu_track_table[187].lat = 465186664;
	simu_track_table[187].lon = 65668551;
	simu_track_table[187].alt = 415;
	simu_track_table[187].relative_alt = -15;
	simu_track_table[187].vx = -57;
	simu_track_table[187].vy = 12;
	simu_track_table[187].vz = -46;
	simu_track_table[187].hdg = 348;

	simu_track_table[188].lat = 465186655;
	simu_track_table[188].lon = 65668551;
	simu_track_table[188].alt = 415;
	simu_track_table[188].relative_alt = -15;
	simu_track_table[188].vx = -57;
	simu_track_table[188].vy = 13;
	simu_track_table[188].vz = -46;
	simu_track_table[188].hdg = 348;

	simu_track_table[189].lat = 465186645;
	simu_track_table[189].lon = 65668551;
	simu_track_table[189].alt = 415;
	simu_track_table[189].relative_alt = -15;
	simu_track_table[189].vx = -57;
	simu_track_table[189].vy = 13;
	simu_track_table[189].vz = -46;
	simu_track_table[189].hdg = 347;

	simu_track_table[190].lat = 465186635;
	simu_track_table[190].lon = 65668551;
	simu_track_table[190].alt = 415;
	simu_track_table[190].relative_alt = -15;
	simu_track_table[190].vx = -56;
	simu_track_table[190].vy = 13;
	simu_track_table[190].vz = -46;
	simu_track_table[190].hdg = 347;

	simu_track_table[191].lat = 465186626;
	simu_track_table[191].lon = 65668551;
	simu_track_table[191].alt = 415;
	simu_track_table[191].relative_alt = -15;
	simu_track_table[191].vx = -56;
	simu_track_table[191].vy = 13;
	simu_track_table[191].vz = -46;
	simu_track_table[191].hdg = 347;

	simu_track_table[192].lat = 465186616;
	simu_track_table[192].lon = 65668551;
	simu_track_table[192].alt = 415;
	simu_track_table[192].relative_alt = -15;
	simu_track_table[192].vx = -56;
	simu_track_table[192].vy = 13;
	simu_track_table[192].vz = -46;
	simu_track_table[192].hdg = 347;

	simu_track_table[193].lat = 465186607;
	simu_track_table[193].lon = 65668552;
	simu_track_table[193].alt = 416;
	simu_track_table[193].relative_alt = -16;
	simu_track_table[193].vx = -56;
	simu_track_table[193].vy = 13;
	simu_track_table[193].vz = -46;
	simu_track_table[193].hdg = 347;

	simu_track_table[194].lat = 465186597;
	simu_track_table[194].lon = 65668552;
	simu_track_table[194].alt = 416;
	simu_track_table[194].relative_alt = -16;
	simu_track_table[194].vx = -56;
	simu_track_table[194].vy = 13;
	simu_track_table[194].vz = -46;
	simu_track_table[194].hdg = 347;

	simu_track_table[195].lat = 465186588;
	simu_track_table[195].lon = 65668552;
	simu_track_table[195].alt = 416;
	simu_track_table[195].relative_alt = -16;
	simu_track_table[195].vx = -56;
	simu_track_table[195].vy = 13;
	simu_track_table[195].vz = -46;
	simu_track_table[195].hdg = 347;

	simu_track_table[196].lat = 465186579;
	simu_track_table[196].lon = 65668553;
	simu_track_table[196].alt = 416;
	simu_track_table[196].relative_alt = -16;
	simu_track_table[196].vx = -56;
	simu_track_table[196].vy = 13;
	simu_track_table[196].vz = -46;
	simu_track_table[196].hdg = 347;

	simu_track_table[197].lat = 465186570;
	simu_track_table[197].lon = 65668554;
	simu_track_table[197].alt = 416;
	simu_track_table[197].relative_alt = -16;
	simu_track_table[197].vx = -56;
	simu_track_table[197].vy = 13;
	simu_track_table[197].vz = -46;
	simu_track_table[197].hdg = 347;

	simu_track_table[198].lat = 465186560;
	simu_track_table[198].lon = 65668554;
	simu_track_table[198].alt = 416;
	simu_track_table[198].relative_alt = -16;
	simu_track_table[198].vx = -56;
	simu_track_table[198].vy = 13;
	simu_track_table[198].vz = -46;
	simu_track_table[198].hdg = 347;

	simu_track_table[199].lat = 465186551;
	simu_track_table[199].lon = 65668555;
	simu_track_table[199].alt = 416;
	simu_track_table[199].relative_alt = -16;
	simu_track_table[199].vx = -56;
	simu_track_table[199].vy = 13;
	simu_track_table[199].vz = -46;
	simu_track_table[199].hdg = 347;

	simu_track_table[200].lat = 465186542;
	simu_track_table[200].lon = 65668556;
	simu_track_table[200].alt = 416;
	simu_track_table[200].relative_alt = -16;
	simu_track_table[200].vx = -56;
	simu_track_table[200].vy = 13;
	simu_track_table[200].vz = -46;
	simu_track_table[200].hdg = 347;

	simu_track_table[201].lat = 465186533;
	simu_track_table[201].lon = 65668557;
	simu_track_table[201].alt = 416;
	simu_track_table[201].relative_alt = -16;
	simu_track_table[201].vx = -56;
	simu_track_table[201].vy = 13;
	simu_track_table[201].vz = -46;
	simu_track_table[201].hdg = 347;

	simu_track_table[202].lat = 465186525;
	simu_track_table[202].lon = 65668558;
	simu_track_table[202].alt = 417;
	simu_track_table[202].relative_alt = -17;
	simu_track_table[202].vx = -55;
	simu_track_table[202].vy = 13;
	simu_track_table[202].vz = -46;
	simu_track_table[202].hdg = 347;

	simu_track_table[203].lat = 465186516;
	simu_track_table[203].lon = 65668559;
	simu_track_table[203].alt = 417;
	simu_track_table[203].relative_alt = -17;
	simu_track_table[203].vx = -55;
	simu_track_table[203].vy = 13;
	simu_track_table[203].vz = -46;
	simu_track_table[203].hdg = 346;

	simu_track_table[204].lat = 465186507;
	simu_track_table[204].lon = 65668560;
	simu_track_table[204].alt = 417;
	simu_track_table[204].relative_alt = -17;
	simu_track_table[204].vx = -55;
	simu_track_table[204].vy = 13;
	simu_track_table[204].vz = -46;
	simu_track_table[204].hdg = 346;

	simu_track_table[205].lat = 465186498;
	simu_track_table[205].lon = 65668561;
	simu_track_table[205].alt = 417;
	simu_track_table[205].relative_alt = -17;
	simu_track_table[205].vx = -55;
	simu_track_table[205].vy = 13;
	simu_track_table[205].vz = -46;
	simu_track_table[205].hdg = 346;

	simu_track_table[206].lat = 465186490;
	simu_track_table[206].lon = 65668563;
	simu_track_table[206].alt = 417;
	simu_track_table[206].relative_alt = -17;
	simu_track_table[206].vx = -55;
	simu_track_table[206].vy = 13;
	simu_track_table[206].vz = -46;
	simu_track_table[206].hdg = 346;

	simu_track_table[207].lat = 465186481;
	simu_track_table[207].lon = 65668564;
	simu_track_table[207].alt = 417;
	simu_track_table[207].relative_alt = -17;
	simu_track_table[207].vx = -55;
	simu_track_table[207].vy = 13;
	simu_track_table[207].vz = -46;
	simu_track_table[207].hdg = 346;

	simu_track_table[208].lat = 465186473;
	simu_track_table[208].lon = 65668566;
	simu_track_table[208].alt = 417;
	simu_track_table[208].relative_alt = -17;
	simu_track_table[208].vx = -55;
	simu_track_table[208].vy = 13;
	simu_track_table[208].vz = -46;
	simu_track_table[208].hdg = 346;

	simu_track_table[209].lat = 465186464;
	simu_track_table[209].lon = 65668567;
	simu_track_table[209].alt = 417;
	simu_track_table[209].relative_alt = -17;
	simu_track_table[209].vx = -55;
	simu_track_table[209].vy = 13;
	simu_track_table[209].vz = -46;
	simu_track_table[209].hdg = 346;

	simu_track_table[210].lat = 465186455;
	simu_track_table[210].lon = 65668569;
	simu_track_table[210].alt = 418;
	simu_track_table[210].relative_alt = -18;
	simu_track_table[210].vx = -55;
	simu_track_table[210].vy = 14;
	simu_track_table[210].vz = -46;
	simu_track_table[210].hdg = 346;

	simu_track_table[211].lat = 465186447;
	simu_track_table[211].lon = 65668571;
	simu_track_table[211].alt = 418;
	simu_track_table[211].relative_alt = -18;
	simu_track_table[211].vx = -55;
	simu_track_table[211].vy = 14;
	simu_track_table[211].vz = -46;
	simu_track_table[211].hdg = 346;

	simu_track_table[212].lat = 465186439;
	simu_track_table[212].lon = 65668572;
	simu_track_table[212].alt = 418;
	simu_track_table[212].relative_alt = -18;
	simu_track_table[212].vx = -55;
	simu_track_table[212].vy = 14;
	simu_track_table[212].vz = -46;
	simu_track_table[212].hdg = 346;

	simu_track_table[213].lat = 465186430;
	simu_track_table[213].lon = 65668574;
	simu_track_table[213].alt = 418;
	simu_track_table[213].relative_alt = -18;
	simu_track_table[213].vx = -55;
	simu_track_table[213].vy = 14;
	simu_track_table[213].vz = -46;
	simu_track_table[213].hdg = 346;

	simu_track_table[214].lat = 465186422;
	simu_track_table[214].lon = 65668576;
	simu_track_table[214].alt = 418;
	simu_track_table[214].relative_alt = -18;
	simu_track_table[214].vx = -55;
	simu_track_table[214].vy = 14;
	simu_track_table[214].vz = -46;
	simu_track_table[214].hdg = 346;

	simu_track_table[215].lat = 465186414;
	simu_track_table[215].lon = 65668578;
	simu_track_table[215].alt = 418;
	simu_track_table[215].relative_alt = -18;
	simu_track_table[215].vx = -55;
	simu_track_table[215].vy = 14;
	simu_track_table[215].vz = -46;
	simu_track_table[215].hdg = 346;

	simu_track_table[216].lat = 465186405;
	simu_track_table[216].lon = 65668580;
	simu_track_table[216].alt = 418;
	simu_track_table[216].relative_alt = -18;
	simu_track_table[216].vx = -55;
	simu_track_table[216].vy = 14;
	simu_track_table[216].vz = -46;
	simu_track_table[216].hdg = 346;

	simu_track_table[217].lat = 465186397;
	simu_track_table[217].lon = 65668582;
	simu_track_table[217].alt = 418;
	simu_track_table[217].relative_alt = -18;
	simu_track_table[217].vx = -54;
	simu_track_table[217].vy = 14;
	simu_track_table[217].vz = -46;
	simu_track_table[217].hdg = 346;

	simu_track_table[218].lat = 465186389;
	simu_track_table[218].lon = 65668584;
	simu_track_table[218].alt = 418;
	simu_track_table[218].relative_alt = -18;
	simu_track_table[218].vx = -54;
	simu_track_table[218].vy = 14;
	simu_track_table[218].vz = -46;
	simu_track_table[218].hdg = 346;

	simu_track_table[219].lat = 465186380;
	simu_track_table[219].lon = 65668587;
	simu_track_table[219].alt = 419;
	simu_track_table[219].relative_alt = -19;
	simu_track_table[219].vx = -54;
	simu_track_table[219].vy = 14;
	simu_track_table[219].vz = -46;
	simu_track_table[219].hdg = 346;

	simu_track_table[220].lat = 465186372;
	simu_track_table[220].lon = 65668589;
	simu_track_table[220].alt = 419;
	simu_track_table[220].relative_alt = -19;
	simu_track_table[220].vx = -54;
	simu_track_table[220].vy = 14;
	simu_track_table[220].vz = -46;
	simu_track_table[220].hdg = 346;

	simu_track_table[221].lat = 465186364;
	simu_track_table[221].lon = 65668591;
	simu_track_table[221].alt = 419;
	simu_track_table[221].relative_alt = -19;
	simu_track_table[221].vx = -54;
	simu_track_table[221].vy = 14;
	simu_track_table[221].vz = -46;
	simu_track_table[221].hdg = 346;

	simu_track_table[222].lat = 465186356;
	simu_track_table[222].lon = 65668594;
	simu_track_table[222].alt = 419;
	simu_track_table[222].relative_alt = -19;
	simu_track_table[222].vx = -54;
	simu_track_table[222].vy = 14;
	simu_track_table[222].vz = -46;
	simu_track_table[222].hdg = 346;

	simu_track_table[223].lat = 465186348;
	simu_track_table[223].lon = 65668596;
	simu_track_table[223].alt = 419;
	simu_track_table[223].relative_alt = -19;
	simu_track_table[223].vx = -54;
	simu_track_table[223].vy = 14;
	simu_track_table[223].vz = -46;
	simu_track_table[223].hdg = 346;

	simu_track_table[224].lat = 465186340;
	simu_track_table[224].lon = 65668599;
	simu_track_table[224].alt = 419;
	simu_track_table[224].relative_alt = -19;
	simu_track_table[224].vx = -54;
	simu_track_table[224].vy = 14;
	simu_track_table[224].vz = -46;
	simu_track_table[224].hdg = 346;

	simu_track_table[225].lat = 465186331;
	simu_track_table[225].lon = 65668601;
	simu_track_table[225].alt = 419;
	simu_track_table[225].relative_alt = -19;
	simu_track_table[225].vx = -54;
	simu_track_table[225].vy = 14;
	simu_track_table[225].vz = -46;
	simu_track_table[225].hdg = 346;

	simu_track_table[226].lat = 465186323;
	simu_track_table[226].lon = 65668604;
	simu_track_table[226].alt = 419;
	simu_track_table[226].relative_alt = -19;
	simu_track_table[226].vx = -54;
	simu_track_table[226].vy = 14;
	simu_track_table[226].vz = -46;
	simu_track_table[226].hdg = 346;

	simu_track_table[227].lat = 465186315;
	simu_track_table[227].lon = 65668607;
	simu_track_table[227].alt = 419;
	simu_track_table[227].relative_alt = -19;
	simu_track_table[227].vx = -54;
	simu_track_table[227].vy = 14;
	simu_track_table[227].vz = -46;
	simu_track_table[227].hdg = 346;

	simu_track_table[228].lat = 465186307;
	simu_track_table[228].lon = 65668610;
	simu_track_table[228].alt = 420;
	simu_track_table[228].relative_alt = -20;
	simu_track_table[228].vx = -54;
	simu_track_table[228].vy = 14;
	simu_track_table[228].vz = -46;
	simu_track_table[228].hdg = 346;

	simu_track_table[229].lat = 465186299;
	simu_track_table[229].lon = 65668612;
	simu_track_table[229].alt = 420;
	simu_track_table[229].relative_alt = -20;
	simu_track_table[229].vx = -54;
	simu_track_table[229].vy = 14;
	simu_track_table[229].vz = -46;
	simu_track_table[229].hdg = 346;

	simu_track_table[230].lat = 465186291;
	simu_track_table[230].lon = 65668615;
	simu_track_table[230].alt = 420;
	simu_track_table[230].relative_alt = -20;
	simu_track_table[230].vx = -54;
	simu_track_table[230].vy = 14;
	simu_track_table[230].vz = -46;
	simu_track_table[230].hdg = 346;

	simu_track_table[231].lat = 465186283;
	simu_track_table[231].lon = 65668618;
	simu_track_table[231].alt = 420;
	simu_track_table[231].relative_alt = -20;
	simu_track_table[231].vx = -54;
	simu_track_table[231].vy = 14;
	simu_track_table[231].vz = -46;
	simu_track_table[231].hdg = 346;

	simu_track_table[232].lat = 465186275;
	simu_track_table[232].lon = 65668621;
	simu_track_table[232].alt = 420;
	simu_track_table[232].relative_alt = -20;
	simu_track_table[232].vx = -54;
	simu_track_table[232].vy = 14;
	simu_track_table[232].vz = -46;
	simu_track_table[232].hdg = 346;

	simu_track_table[233].lat = 465186267;
	simu_track_table[233].lon = 65668624;
	simu_track_table[233].alt = 420;
	simu_track_table[233].relative_alt = -20;
	simu_track_table[233].vx = -54;
	simu_track_table[233].vy = 14;
	simu_track_table[233].vz = -46;
	simu_track_table[233].hdg = 346;

	simu_track_table[234].lat = 465186259;
	simu_track_table[234].lon = 65668627;
	simu_track_table[234].alt = 420;
	simu_track_table[234].relative_alt = -20;
	simu_track_table[234].vx = -53;
	simu_track_table[234].vy = 14;
	simu_track_table[234].vz = -46;
	simu_track_table[234].hdg = 346;

	simu_track_table[235].lat = 465186251;
	simu_track_table[235].lon = 65668630;
	simu_track_table[235].alt = 420;
	simu_track_table[235].relative_alt = -20;
	simu_track_table[235].vx = -53;
	simu_track_table[235].vy = 14;
	simu_track_table[235].vz = -46;
	simu_track_table[235].hdg = 346;

	simu_track_table[236].lat = 465186243;
	simu_track_table[236].lon = 65668634;
	simu_track_table[236].alt = 421;
	simu_track_table[236].relative_alt = -21;
	simu_track_table[236].vx = -53;
	simu_track_table[236].vy = 14;
	simu_track_table[236].vz = -46;
	simu_track_table[236].hdg = 346;

	simu_track_table[237].lat = 465186235;
	simu_track_table[237].lon = 65668637;
	simu_track_table[237].alt = 421;
	simu_track_table[237].relative_alt = -21;
	simu_track_table[237].vx = -53;
	simu_track_table[237].vy = 14;
	simu_track_table[237].vz = -46;
	simu_track_table[237].hdg = 346;

	simu_track_table[238].lat = 465186227;
	simu_track_table[238].lon = 65668640;
	simu_track_table[238].alt = 421;
	simu_track_table[238].relative_alt = -21;
	simu_track_table[238].vx = -53;
	simu_track_table[238].vy = 13;
	simu_track_table[238].vz = -46;
	simu_track_table[238].hdg = 346;

	simu_track_table[239].lat = 465186219;
	simu_track_table[239].lon = 65668643;
	simu_track_table[239].alt = 421;
	simu_track_table[239].relative_alt = -21;
	simu_track_table[239].vx = -53;
	simu_track_table[239].vy = 13;
	simu_track_table[239].vz = -46;
	simu_track_table[239].hdg = 346;

	simu_track_table[240].lat = 465186211;
	simu_track_table[240].lon = 65668647;
	simu_track_table[240].alt = 421;
	simu_track_table[240].relative_alt = -21;
	simu_track_table[240].vx = -53;
	simu_track_table[240].vy = 13;
	simu_track_table[240].vz = -46;
	simu_track_table[240].hdg = 346;

	simu_track_table[241].lat = 465186203;
	simu_track_table[241].lon = 65668650;
	simu_track_table[241].alt = 421;
	simu_track_table[241].relative_alt = -21;
	simu_track_table[241].vx = -53;
	simu_track_table[241].vy = 13;
	simu_track_table[241].vz = -46;
	simu_track_table[241].hdg = 346;

	simu_track_table[242].lat = 465186195;
	simu_track_table[242].lon = 65668653;
	simu_track_table[242].alt = 421;
	simu_track_table[242].relative_alt = -21;
	simu_track_table[242].vx = -53;
	simu_track_table[242].vy = 13;
	simu_track_table[242].vz = -46;
	simu_track_table[242].hdg = 346;

	simu_track_table[243].lat = 465186187;
	simu_track_table[243].lon = 65668657;
	simu_track_table[243].alt = 421;
	simu_track_table[243].relative_alt = -21;
	simu_track_table[243].vx = -53;
	simu_track_table[243].vy = 13;
	simu_track_table[243].vz = -46;
	simu_track_table[243].hdg = 346;

	simu_track_table[244].lat = 465186179;
	simu_track_table[244].lon = 65668660;
	simu_track_table[244].alt = 421;
	simu_track_table[244].relative_alt = -21;
	simu_track_table[244].vx = -53;
	simu_track_table[244].vy = 13;
	simu_track_table[244].vz = -46;
	simu_track_table[244].hdg = 346;

	simu_track_table[245].lat = 465186171;
	simu_track_table[245].lon = 65668664;
	simu_track_table[245].alt = 422;
	simu_track_table[245].relative_alt = -22;
	simu_track_table[245].vx = -53;
	simu_track_table[245].vy = 13;
	simu_track_table[245].vz = -46;
	simu_track_table[245].hdg = 346;

	simu_track_table[246].lat = 465186163;
	simu_track_table[246].lon = 65668667;
	simu_track_table[246].alt = 422;
	simu_track_table[246].relative_alt = -22;
	simu_track_table[246].vx = -53;
	simu_track_table[246].vy = 13;
	simu_track_table[246].vz = -46;
	simu_track_table[246].hdg = 346;

	simu_track_table[247].lat = 465186154;
	simu_track_table[247].lon = 65668671;
	simu_track_table[247].alt = 422;
	simu_track_table[247].relative_alt = -22;
	simu_track_table[247].vx = -53;
	simu_track_table[247].vy = 13;
	simu_track_table[247].vz = -46;
	simu_track_table[247].hdg = 346;

	simu_track_table[248].lat = 465186146;
	simu_track_table[248].lon = 65668675;
	simu_track_table[248].alt = 422;
	simu_track_table[248].relative_alt = -22;
	simu_track_table[248].vx = -53;
	simu_track_table[248].vy = 13;
	simu_track_table[248].vz = -46;
	simu_track_table[248].hdg = 346;

	simu_track_table[249].lat = 465186138;
	simu_track_table[249].lon = 65668678;
	simu_track_table[249].alt = 422;
	simu_track_table[249].relative_alt = -22;
	simu_track_table[249].vx = -53;
	simu_track_table[249].vy = 13;
	simu_track_table[249].vz = -46;
	simu_track_table[249].hdg = 346;

	simu_track_table[250].lat = 465186130;
	simu_track_table[250].lon = 65668682;
	simu_track_table[250].alt = 422;
	simu_track_table[250].relative_alt = -22;
	simu_track_table[250].vx = -52;
	simu_track_table[250].vy = 13;
	simu_track_table[250].vz = -46;
	simu_track_table[250].hdg = 346;

	simu_track_table[251].lat = 465186122;
	simu_track_table[251].lon = 65668686;
	simu_track_table[251].alt = 422;
	simu_track_table[251].relative_alt = -22;
	simu_track_table[251].vx = -52;
	simu_track_table[251].vy = 13;
	simu_track_table[251].vz = -46;
	simu_track_table[251].hdg = 346;

	simu_track_table[252].lat = 465186114;
	simu_track_table[252].lon = 65668689;
	simu_track_table[252].alt = 422;
	simu_track_table[252].relative_alt = -22;
	simu_track_table[252].vx = -52;
	simu_track_table[252].vy = 13;
	simu_track_table[252].vz = -46;
	simu_track_table[252].hdg = 346;

	simu_track_table[253].lat = 465186106;
	simu_track_table[253].lon = 65668693;
	simu_track_table[253].alt = 422;
	simu_track_table[253].relative_alt = -22;
	simu_track_table[253].vx = -52;
	simu_track_table[253].vy = 13;
	simu_track_table[253].vz = -46;
	simu_track_table[253].hdg = 346;

	simu_track_table[254].lat = 465186098;
	simu_track_table[254].lon = 65668697;
	simu_track_table[254].alt = 423;
	simu_track_table[254].relative_alt = -23;
	simu_track_table[254].vx = -52;
	simu_track_table[254].vy = 13;
	simu_track_table[254].vz = -46;
	simu_track_table[254].hdg = 346;

	simu_track_table[255].lat = 465186090;
	simu_track_table[255].lon = 65668701;
	simu_track_table[255].alt = 423;
	simu_track_table[255].relative_alt = -23;
	simu_track_table[255].vx = -52;
	simu_track_table[255].vy = 13;
	simu_track_table[255].vz = -46;
	simu_track_table[255].hdg = 346;

	simu_track_table[256].lat = 465186082;
	simu_track_table[256].lon = 65668705;
	simu_track_table[256].alt = 423;
	simu_track_table[256].relative_alt = -23;
	simu_track_table[256].vx = -52;
	simu_track_table[256].vy = 13;
	simu_track_table[256].vz = -46;
	simu_track_table[256].hdg = 346;

	simu_track_table[257].lat = 465186074;
	simu_track_table[257].lon = 65668708;
	simu_track_table[257].alt = 423;
	simu_track_table[257].relative_alt = -23;
	simu_track_table[257].vx = -52;
	simu_track_table[257].vy = 13;
	simu_track_table[257].vz = -46;
	simu_track_table[257].hdg = 346;

	simu_track_table[258].lat = 465186066;
	simu_track_table[258].lon = 65668712;
	simu_track_table[258].alt = 423;
	simu_track_table[258].relative_alt = -23;
	simu_track_table[258].vx = -52;
	simu_track_table[258].vy = 13;
	simu_track_table[258].vz = -46;
	simu_track_table[258].hdg = 346;

	simu_track_table[259].lat = 465186057;
	simu_track_table[259].lon = 65668716;
	simu_track_table[259].alt = 423;
	simu_track_table[259].relative_alt = -23;
	simu_track_table[259].vx = -52;
	simu_track_table[259].vy = 13;
	simu_track_table[259].vz = -46;
	simu_track_table[259].hdg = 346;

	simu_track_table[260].lat = 465186049;
	simu_track_table[260].lon = 65668720;
	simu_track_table[260].alt = 423;
	simu_track_table[260].relative_alt = -23;
	simu_track_table[260].vx = -52;
	simu_track_table[260].vy = 13;
	simu_track_table[260].vz = -46;
	simu_track_table[260].hdg = 346;

	simu_track_table[261].lat = 465186041;
	simu_track_table[261].lon = 65668724;
	simu_track_table[261].alt = 423;
	simu_track_table[261].relative_alt = -23;
	simu_track_table[261].vx = -52;
	simu_track_table[261].vy = 13;
	simu_track_table[261].vz = -46;
	simu_track_table[261].hdg = 346;

	simu_track_table[262].lat = 465186033;
	simu_track_table[262].lon = 65668728;
	simu_track_table[262].alt = 424;
	simu_track_table[262].relative_alt = -24;
	simu_track_table[262].vx = -51;
	simu_track_table[262].vy = 12;
	simu_track_table[262].vz = -46;
	simu_track_table[262].hdg = 346;

	simu_track_table[263].lat = 465186025;
	simu_track_table[263].lon = 65668732;
	simu_track_table[263].alt = 424;
	simu_track_table[263].relative_alt = -24;
	simu_track_table[263].vx = -51;
	simu_track_table[263].vy = 12;
	simu_track_table[263].vz = -46;
	simu_track_table[263].hdg = 346;

	simu_track_table[264].lat = 465186017;
	simu_track_table[264].lon = 65668736;
	simu_track_table[264].alt = 424;
	simu_track_table[264].relative_alt = -24;
	simu_track_table[264].vx = -51;
	simu_track_table[264].vy = 12;
	simu_track_table[264].vz = -46;
	simu_track_table[264].hdg = 346;

	simu_track_table[265].lat = 465186008;
	simu_track_table[265].lon = 65668740;
	simu_track_table[265].alt = 424;
	simu_track_table[265].relative_alt = -24;
	simu_track_table[265].vx = -51;
	simu_track_table[265].vy = 12;
	simu_track_table[265].vz = -46;
	simu_track_table[265].hdg = 347;

	simu_track_table[266].lat = 465186000;
	simu_track_table[266].lon = 65668744;
	simu_track_table[266].alt = 424;
	simu_track_table[266].relative_alt = -24;
	simu_track_table[266].vx = -51;
	simu_track_table[266].vy = 12;
	simu_track_table[266].vz = -46;
	simu_track_table[266].hdg = 347;

	simu_track_table[267].lat = 465185992;
	simu_track_table[267].lon = 65668748;
	simu_track_table[267].alt = 424;
	simu_track_table[267].relative_alt = -24;
	simu_track_table[267].vx = -51;
	simu_track_table[267].vy = 12;
	simu_track_table[267].vz = -46;
	simu_track_table[267].hdg = 347;

	simu_track_table[268].lat = 465185984;
	simu_track_table[268].lon = 65668752;
	simu_track_table[268].alt = 424;
	simu_track_table[268].relative_alt = -24;
	simu_track_table[268].vx = -51;
	simu_track_table[268].vy = 12;
	simu_track_table[268].vz = -46;
	simu_track_table[268].hdg = 347;

	simu_track_table[269].lat = 465185975;
	simu_track_table[269].lon = 65668756;
	simu_track_table[269].alt = 424;
	simu_track_table[269].relative_alt = -24;
	simu_track_table[269].vx = -51;
	simu_track_table[269].vy = 12;
	simu_track_table[269].vz = -46;
	simu_track_table[269].hdg = 347;

	simu_track_table[270].lat = 465185967;
	simu_track_table[270].lon = 65668760;
	simu_track_table[270].alt = 424;
	simu_track_table[270].relative_alt = -24;
	simu_track_table[270].vx = -51;
	simu_track_table[270].vy = 12;
	simu_track_table[270].vz = -46;
	simu_track_table[270].hdg = 347;

	simu_track_table[271].lat = 465185959;
	simu_track_table[271].lon = 65668764;
	simu_track_table[271].alt = 425;
	simu_track_table[271].relative_alt = -25;
	simu_track_table[271].vx = -51;
	simu_track_table[271].vy = 12;
	simu_track_table[271].vz = -46;
	simu_track_table[271].hdg = 347;

	simu_track_table[272].lat = 465185951;
	simu_track_table[272].lon = 65668768;
	simu_track_table[272].alt = 425;
	simu_track_table[272].relative_alt = -25;
	simu_track_table[272].vx = -50;
	simu_track_table[272].vy = 12;
	simu_track_table[272].vz = -46;
	simu_track_table[272].hdg = 347;

	simu_track_table[273].lat = 465185942;
	simu_track_table[273].lon = 65668772;
	simu_track_table[273].alt = 425;
	simu_track_table[273].relative_alt = -25;
	simu_track_table[273].vx = -50;
	simu_track_table[273].vy = 12;
	simu_track_table[273].vz = -46;
	simu_track_table[273].hdg = 347;

	simu_track_table[274].lat = 465185934;
	simu_track_table[274].lon = 65668776;
	simu_track_table[274].alt = 425;
	simu_track_table[274].relative_alt = -25;
	simu_track_table[274].vx = -50;
	simu_track_table[274].vy = 12;
	simu_track_table[274].vz = -46;
	simu_track_table[274].hdg = 347;

	simu_track_table[275].lat = 465185926;
	simu_track_table[275].lon = 65668780;
	simu_track_table[275].alt = 425;
	simu_track_table[275].relative_alt = -25;
	simu_track_table[275].vx = -50;
	simu_track_table[275].vy = 12;
	simu_track_table[275].vz = -46;
	simu_track_table[275].hdg = 347;

	simu_track_table[276].lat = 465185917;
	simu_track_table[276].lon = 65668784;
	simu_track_table[276].alt = 425;
	simu_track_table[276].relative_alt = -25;
	simu_track_table[276].vx = -50;
	simu_track_table[276].vy = 12;
	simu_track_table[276].vz = -46;
	simu_track_table[276].hdg = 347;

	simu_track_table[277].lat = 465185909;
	simu_track_table[277].lon = 65668788;
	simu_track_table[277].alt = 425;
	simu_track_table[277].relative_alt = -25;
	simu_track_table[277].vx = -50;
	simu_track_table[277].vy = 11;
	simu_track_table[277].vz = -46;
	simu_track_table[277].hdg = 347;

	simu_track_table[278].lat = 465185901;
	simu_track_table[278].lon = 65668793;
	simu_track_table[278].alt = 425;
	simu_track_table[278].relative_alt = -25;
	simu_track_table[278].vx = -50;
	simu_track_table[278].vy = 11;
	simu_track_table[278].vz = -46;
	simu_track_table[278].hdg = 347;

	simu_track_table[279].lat = 465185892;
	simu_track_table[279].lon = 65668797;
	simu_track_table[279].alt = 425;
	simu_track_table[279].relative_alt = -25;
	simu_track_table[279].vx = -50;
	simu_track_table[279].vy = 11;
	simu_track_table[279].vz = -46;
	simu_track_table[279].hdg = 347;

	simu_track_table[280].lat = 465185884;
	simu_track_table[280].lon = 65668801;
	simu_track_table[280].alt = 426;
	simu_track_table[280].relative_alt = -26;
	simu_track_table[280].vx = -50;
	simu_track_table[280].vy = 11;
	simu_track_table[280].vz = -46;
	simu_track_table[280].hdg = 347;

	simu_track_table[281].lat = 465185876;
	simu_track_table[281].lon = 65668805;
	simu_track_table[281].alt = 426;
	simu_track_table[281].relative_alt = -26;
	simu_track_table[281].vx = -49;
	simu_track_table[281].vy = 11;
	simu_track_table[281].vz = -46;
	simu_track_table[281].hdg = 347;

	simu_track_table[282].lat = 465185867;
	simu_track_table[282].lon = 65668809;
	simu_track_table[282].alt = 426;
	simu_track_table[282].relative_alt = -26;
	simu_track_table[282].vx = -49;
	simu_track_table[282].vy = 11;
	simu_track_table[282].vz = -46;
	simu_track_table[282].hdg = 347;

	simu_track_table[283].lat = 465185859;
	simu_track_table[283].lon = 65668813;
	simu_track_table[283].alt = 426;
	simu_track_table[283].relative_alt = -26;
	simu_track_table[283].vx = -49;
	simu_track_table[283].vy = 11;
	simu_track_table[283].vz = -46;
	simu_track_table[283].hdg = 347;

	simu_track_table[284].lat = 465185850;
	simu_track_table[284].lon = 65668817;
	simu_track_table[284].alt = 426;
	simu_track_table[284].relative_alt = -26;
	simu_track_table[284].vx = -49;
	simu_track_table[284].vy = 11;
	simu_track_table[284].vz = -46;
	simu_track_table[284].hdg = 348;

	simu_track_table[285].lat = 465185842;
	simu_track_table[285].lon = 65668821;
	simu_track_table[285].alt = 426;
	simu_track_table[285].relative_alt = -26;
	simu_track_table[285].vx = -49;
	simu_track_table[285].vy = 11;
	simu_track_table[285].vz = -46;
	simu_track_table[285].hdg = 348;

	simu_track_table[286].lat = 465185834;
	simu_track_table[286].lon = 65668825;
	simu_track_table[286].alt = 426;
	simu_track_table[286].relative_alt = -26;
	simu_track_table[286].vx = -49;
	simu_track_table[286].vy = 11;
	simu_track_table[286].vz = -46;
	simu_track_table[286].hdg = 348;

	simu_track_table[287].lat = 465185825;
	simu_track_table[287].lon = 65668829;
	simu_track_table[287].alt = 426;
	simu_track_table[287].relative_alt = -26;
	simu_track_table[287].vx = -49;
	simu_track_table[287].vy = 11;
	simu_track_table[287].vz = -46;
	simu_track_table[287].hdg = 348;

	simu_track_table[288].lat = 465185817;
	simu_track_table[288].lon = 65668833;
	simu_track_table[288].alt = 427;
	simu_track_table[288].relative_alt = -27;
	simu_track_table[288].vx = -49;
	simu_track_table[288].vy = 11;
	simu_track_table[288].vz = -46;
	simu_track_table[288].hdg = 348;

	simu_track_table[289].lat = 465185808;
	simu_track_table[289].lon = 65668837;
	simu_track_table[289].alt = 427;
	simu_track_table[289].relative_alt = -27;
	simu_track_table[289].vx = -48;
	simu_track_table[289].vy = 10;
	simu_track_table[289].vz = -46;
	simu_track_table[289].hdg = 348;

	simu_track_table[290].lat = 465185800;
	simu_track_table[290].lon = 65668841;
	simu_track_table[290].alt = 427;
	simu_track_table[290].relative_alt = -27;
	simu_track_table[290].vx = -48;
	simu_track_table[290].vy = 10;
	simu_track_table[290].vz = -46;
	simu_track_table[290].hdg = 348;

	simu_track_table[291].lat = 465185792;
	simu_track_table[291].lon = 65668845;
	simu_track_table[291].alt = 427;
	simu_track_table[291].relative_alt = -27;
	simu_track_table[291].vx = -48;
	simu_track_table[291].vy = 10;
	simu_track_table[291].vz = -46;
	simu_track_table[291].hdg = 348;

	simu_track_table[292].lat = 465185783;
	simu_track_table[292].lon = 65668849;
	simu_track_table[292].alt = 427;
	simu_track_table[292].relative_alt = -27;
	simu_track_table[292].vx = -48;
	simu_track_table[292].vy = 10;
	simu_track_table[292].vz = -46;
	simu_track_table[292].hdg = 348;

	simu_track_table[293].lat = 465185775;
	simu_track_table[293].lon = 65668853;
	simu_track_table[293].alt = 427;
	simu_track_table[293].relative_alt = -27;
	simu_track_table[293].vx = -48;
	simu_track_table[293].vy = 10;
	simu_track_table[293].vz = -46;
	simu_track_table[293].hdg = 348;

	simu_track_table[294].lat = 465185766;
	simu_track_table[294].lon = 65668857;
	simu_track_table[294].alt = 427;
	simu_track_table[294].relative_alt = -27;
	simu_track_table[294].vx = -48;
	simu_track_table[294].vy = 10;
	simu_track_table[294].vz = -46;
	simu_track_table[294].hdg = 348;

	simu_track_table[295].lat = 465185758;
	simu_track_table[295].lon = 65668861;
	simu_track_table[295].alt = 427;
	simu_track_table[295].relative_alt = -27;
	simu_track_table[295].vx = -48;
	simu_track_table[295].vy = 10;
	simu_track_table[295].vz = -46;
	simu_track_table[295].hdg = 348;

	simu_track_table[296].lat = 465185749;
	simu_track_table[296].lon = 65668865;
	simu_track_table[296].alt = 427;
	simu_track_table[296].relative_alt = -27;
	simu_track_table[296].vx = -47;
	simu_track_table[296].vy = 10;
	simu_track_table[296].vz = -46;
	simu_track_table[296].hdg = 348;

	simu_track_table[297].lat = 465185741;
	simu_track_table[297].lon = 65668869;
	simu_track_table[297].alt = 428;
	simu_track_table[297].relative_alt = -28;
	simu_track_table[297].vx = -47;
	simu_track_table[297].vy = 10;
	simu_track_table[297].vz = -46;
	simu_track_table[297].hdg = 348;

	simu_track_table[298].lat = 465185732;
	simu_track_table[298].lon = 65668873;
	simu_track_table[298].alt = 428;
	simu_track_table[298].relative_alt = -28;
	simu_track_table[298].vx = -47;
	simu_track_table[298].vy = 10;
	simu_track_table[298].vz = -46;
	simu_track_table[298].hdg = 348;

	simu_track_table[299].lat = 465185724;
	simu_track_table[299].lon = 65668877;
	simu_track_table[299].alt = 428;
	simu_track_table[299].relative_alt = -28;
	simu_track_table[299].vx = -47;
	simu_track_table[299].vy = 10;
	simu_track_table[299].vz = -46;
	simu_track_table[299].hdg = 348;

	simu_track_table[300].lat = 465185715;
	simu_track_table[300].lon = 65668881;
	simu_track_table[300].alt = 428;
	simu_track_table[300].relative_alt = -28;
	simu_track_table[300].vx = -47;
	simu_track_table[300].vy = 9;
	simu_track_table[300].vz = -46;
	simu_track_table[300].hdg = 349;

	simu_track_table[301].lat = 465185707;
	simu_track_table[301].lon = 65668885;
	simu_track_table[301].alt = 428;
	simu_track_table[301].relative_alt = -28;
	simu_track_table[301].vx = -47;
	simu_track_table[301].vy = 9;
	simu_track_table[301].vz = -46;
	simu_track_table[301].hdg = 349;

	simu_track_table[302].lat = 465185698;
	simu_track_table[302].lon = 65668889;
	simu_track_table[302].alt = 428;
	simu_track_table[302].relative_alt = -28;
	simu_track_table[302].vx = -46;
	simu_track_table[302].vy = 9;
	simu_track_table[302].vz = -46;
	simu_track_table[302].hdg = 349;

	simu_track_table[303].lat = 465185690;
	simu_track_table[303].lon = 65668893;
	simu_track_table[303].alt = 428;
	simu_track_table[303].relative_alt = -28;
	simu_track_table[303].vx = -46;
	simu_track_table[303].vy = 9;
	simu_track_table[303].vz = -46;
	simu_track_table[303].hdg = 349;

	simu_track_table[304].lat = 465185682;
	simu_track_table[304].lon = 65668896;
	simu_track_table[304].alt = 428;
	simu_track_table[304].relative_alt = -28;
	simu_track_table[304].vx = -46;
	simu_track_table[304].vy = 9;
	simu_track_table[304].vz = -46;
	simu_track_table[304].hdg = 349;

	simu_track_table[305].lat = 465185673;
	simu_track_table[305].lon = 65668900;
	simu_track_table[305].alt = 428;
	simu_track_table[305].relative_alt = -28;
	simu_track_table[305].vx = -46;
	simu_track_table[305].vy = 9;
	simu_track_table[305].vz = -46;
	simu_track_table[305].hdg = 349;

	simu_track_table[306].lat = 465185665;
	simu_track_table[306].lon = 65668904;
	simu_track_table[306].alt = 429;
	simu_track_table[306].relative_alt = -29;
	simu_track_table[306].vx = -46;
	simu_track_table[306].vy = 9;
	simu_track_table[306].vz = -46;
	simu_track_table[306].hdg = 349;

	simu_track_table[307].lat = 465185656;
	simu_track_table[307].lon = 65668908;
	simu_track_table[307].alt = 429;
	simu_track_table[307].relative_alt = -29;
	simu_track_table[307].vx = -46;
	simu_track_table[307].vy = 9;
	simu_track_table[307].vz = -46;
	simu_track_table[307].hdg = 349;

	simu_track_table[308].lat = 465185648;
	simu_track_table[308].lon = 65668911;
	simu_track_table[308].alt = 429;
	simu_track_table[308].relative_alt = -29;
	simu_track_table[308].vx = -45;
	simu_track_table[308].vy = 9;
	simu_track_table[308].vz = -46;
	simu_track_table[308].hdg = 349;

	simu_track_table[309].lat = 465185639;
	simu_track_table[309].lon = 65668915;
	simu_track_table[309].alt = 429;
	simu_track_table[309].relative_alt = -29;
	simu_track_table[309].vx = -45;
	simu_track_table[309].vy = 9;
	simu_track_table[309].vz = -46;
	simu_track_table[309].hdg = 349;

	simu_track_table[310].lat = 465185631;
	simu_track_table[310].lon = 65668919;
	simu_track_table[310].alt = 429;
	simu_track_table[310].relative_alt = -29;
	simu_track_table[310].vx = -45;
	simu_track_table[310].vy = 9;
	simu_track_table[310].vz = -46;
	simu_track_table[310].hdg = 349;

	simu_track_table[311].lat = 465185622;
	simu_track_table[311].lon = 65668923;
	simu_track_table[311].alt = 429;
	simu_track_table[311].relative_alt = -29;
	simu_track_table[311].vx = -45;
	simu_track_table[311].vy = 8;
	simu_track_table[311].vz = -46;
	simu_track_table[311].hdg = 349;

	simu_track_table[312].lat = 465185614;
	simu_track_table[312].lon = 65668926;
	simu_track_table[312].alt = 429;
	simu_track_table[312].relative_alt = -29;
	simu_track_table[312].vx = -45;
	simu_track_table[312].vy = 8;
	simu_track_table[312].vz = -46;
	simu_track_table[312].hdg = 349;

	simu_track_table[313].lat = 465185605;
	simu_track_table[313].lon = 65668930;
	simu_track_table[313].alt = 429;
	simu_track_table[313].relative_alt = -29;
	simu_track_table[313].vx = -44;
	simu_track_table[313].vy = 8;
	simu_track_table[313].vz = -46;
	simu_track_table[313].hdg = 349;

	simu_track_table[314].lat = 465185597;
	simu_track_table[314].lon = 65668933;
	simu_track_table[314].alt = 429;
	simu_track_table[314].relative_alt = -29;
	simu_track_table[314].vx = -44;
	simu_track_table[314].vy = 8;
	simu_track_table[314].vz = -46;
	simu_track_table[314].hdg = 350;

	simu_track_table[315].lat = 465185589;
	simu_track_table[315].lon = 65668937;
	simu_track_table[315].alt = 430;
	simu_track_table[315].relative_alt = -30;
	simu_track_table[315].vx = -44;
	simu_track_table[315].vy = 8;
	simu_track_table[315].vz = -46;
	simu_track_table[315].hdg = 350;

	simu_track_table[316].lat = 465185580;
	simu_track_table[316].lon = 65668941;
	simu_track_table[316].alt = 430;
	simu_track_table[316].relative_alt = -30;
	simu_track_table[316].vx = -44;
	simu_track_table[316].vy = 8;
	simu_track_table[316].vz = -46;
	simu_track_table[316].hdg = 350;

	simu_track_table[317].lat = 465185572;
	simu_track_table[317].lon = 65668944;
	simu_track_table[317].alt = 430;
	simu_track_table[317].relative_alt = -30;
	simu_track_table[317].vx = -44;
	simu_track_table[317].vy = 8;
	simu_track_table[317].vz = -46;
	simu_track_table[317].hdg = 350;

	simu_track_table[318].lat = 465185564;
	simu_track_table[318].lon = 65668948;
	simu_track_table[318].alt = 430;
	simu_track_table[318].relative_alt = -30;
	simu_track_table[318].vx = -43;
	simu_track_table[318].vy = 8;
	simu_track_table[318].vz = -46;
	simu_track_table[318].hdg = 350;

	simu_track_table[319].lat = 465185555;
	simu_track_table[319].lon = 65668951;
	simu_track_table[319].alt = 430;
	simu_track_table[319].relative_alt = -30;
	simu_track_table[319].vx = -43;
	simu_track_table[319].vy = 8;
	simu_track_table[319].vz = -46;
	simu_track_table[319].hdg = 350;

	simu_track_table[320].lat = 465185547;
	simu_track_table[320].lon = 65668955;
	simu_track_table[320].alt = 430;
	simu_track_table[320].relative_alt = -30;
	simu_track_table[320].vx = -43;
	simu_track_table[320].vy = 8;
	simu_track_table[320].vz = -46;
	simu_track_table[320].hdg = 350;

	simu_track_table[321].lat = 465185538;
	simu_track_table[321].lon = 65668958;
	simu_track_table[321].alt = 430;
	simu_track_table[321].relative_alt = -30;
	simu_track_table[321].vx = -43;
	simu_track_table[321].vy = 8;
	simu_track_table[321].vz = -46;
	simu_track_table[321].hdg = 350;

	simu_track_table[322].lat = 465185530;
	simu_track_table[322].lon = 65668961;
	simu_track_table[322].alt = 430;
	simu_track_table[322].relative_alt = -30;
	simu_track_table[322].vx = -43;
	simu_track_table[322].vy = 7;
	simu_track_table[322].vz = -46;
	simu_track_table[322].hdg = 350;

	simu_track_table[323].lat = 465185522;
	simu_track_table[323].lon = 65668965;
	simu_track_table[323].alt = 431;
	simu_track_table[323].relative_alt = -31;
	simu_track_table[323].vx = -42;
	simu_track_table[323].vy = 7;
	simu_track_table[323].vz = -46;
	simu_track_table[323].hdg = 350;

	simu_track_table[324].lat = 465185514;
	simu_track_table[324].lon = 65668968;
	simu_track_table[324].alt = 431;
	simu_track_table[324].relative_alt = -31;
	simu_track_table[324].vx = -42;
	simu_track_table[324].vy = 7;
	simu_track_table[324].vz = -46;
	simu_track_table[324].hdg = 350;

	simu_track_table[325].lat = 465185505;
	simu_track_table[325].lon = 65668971;
	simu_track_table[325].alt = 431;
	simu_track_table[325].relative_alt = -31;
	simu_track_table[325].vx = -42;
	simu_track_table[325].vy = 7;
	simu_track_table[325].vz = -46;
	simu_track_table[325].hdg = 350;

	simu_track_table[326].lat = 465185497;
	simu_track_table[326].lon = 65668975;
	simu_track_table[326].alt = 431;
	simu_track_table[326].relative_alt = -31;
	simu_track_table[326].vx = -42;
	simu_track_table[326].vy = 7;
	simu_track_table[326].vz = -46;
	simu_track_table[326].hdg = 350;

	simu_track_table[327].lat = 465185489;
	simu_track_table[327].lon = 65668978;
	simu_track_table[327].alt = 431;
	simu_track_table[327].relative_alt = -31;
	simu_track_table[327].vx = -42;
	simu_track_table[327].vy = 7;
	simu_track_table[327].vz = -46;
	simu_track_table[327].hdg = 351;

	simu_track_table[328].lat = 465185481;
	simu_track_table[328].lon = 65668981;
	simu_track_table[328].alt = 431;
	simu_track_table[328].relative_alt = -31;
	simu_track_table[328].vx = -41;
	simu_track_table[328].vy = 7;
	simu_track_table[328].vz = -46;
	simu_track_table[328].hdg = 351;

	simu_track_table[329].lat = 465185472;
	simu_track_table[329].lon = 65668984;
	simu_track_table[329].alt = 431;
	simu_track_table[329].relative_alt = -31;
	simu_track_table[329].vx = -41;
	simu_track_table[329].vy = 7;
	simu_track_table[329].vz = -46;
	simu_track_table[329].hdg = 351;

	simu_track_table[330].lat = 465185464;
	simu_track_table[330].lon = 65668987;
	simu_track_table[330].alt = 431;
	simu_track_table[330].relative_alt = -31;
	simu_track_table[330].vx = -41;
	simu_track_table[330].vy = 7;
	simu_track_table[330].vz = -46;
	simu_track_table[330].hdg = 351;

	simu_track_table[331].lat = 465185456;
	simu_track_table[331].lon = 65668991;
	simu_track_table[331].alt = 431;
	simu_track_table[331].relative_alt = -31;
	simu_track_table[331].vx = -41;
	simu_track_table[331].vy = 7;
	simu_track_table[331].vz = -46;
	simu_track_table[331].hdg = 351;

	simu_track_table[332].lat = 465185448;
	simu_track_table[332].lon = 65668994;
	simu_track_table[332].alt = 432;
	simu_track_table[332].relative_alt = -32;
	simu_track_table[332].vx = -40;
	simu_track_table[332].vy = 6;
	simu_track_table[332].vz = -46;
	simu_track_table[332].hdg = 351;

	simu_track_table[333].lat = 465185440;
	simu_track_table[333].lon = 65668997;
	simu_track_table[333].alt = 432;
	simu_track_table[333].relative_alt = -32;
	simu_track_table[333].vx = -40;
	simu_track_table[333].vy = 6;
	simu_track_table[333].vz = -46;
	simu_track_table[333].hdg = 351;

	simu_track_table[334].lat = 465185432;
	simu_track_table[334].lon = 65669000;
	simu_track_table[334].alt = 432;
	simu_track_table[334].relative_alt = -32;
	simu_track_table[334].vx = -40;
	simu_track_table[334].vy = 6;
	simu_track_table[334].vz = -46;
	simu_track_table[334].hdg = 351;

	simu_track_table[335].lat = 465185423;
	simu_track_table[335].lon = 65669003;
	simu_track_table[335].alt = 432;
	simu_track_table[335].relative_alt = -32;
	simu_track_table[335].vx = -40;
	simu_track_table[335].vy = 6;
	simu_track_table[335].vz = -46;
	simu_track_table[335].hdg = 351;

	simu_track_table[336].lat = 465185415;
	simu_track_table[336].lon = 65669006;
	simu_track_table[336].alt = 432;
	simu_track_table[336].relative_alt = -32;
	simu_track_table[336].vx = -39;
	simu_track_table[336].vy = 6;
	simu_track_table[336].vz = -46;
	simu_track_table[336].hdg = 351;

	simu_track_table[337].lat = 465185407;
	simu_track_table[337].lon = 65669009;
	simu_track_table[337].alt = 432;
	simu_track_table[337].relative_alt = -32;
	simu_track_table[337].vx = -39;
	simu_track_table[337].vy = 6;
	simu_track_table[337].vz = -46;
	simu_track_table[337].hdg = 351;

	simu_track_table[338].lat = 465185399;
	simu_track_table[338].lon = 65669011;
	simu_track_table[338].alt = 432;
	simu_track_table[338].relative_alt = -32;
	simu_track_table[338].vx = -39;
	simu_track_table[338].vy = 6;
	simu_track_table[338].vz = -46;
	simu_track_table[338].hdg = 351;

	simu_track_table[339].lat = 465185391;
	simu_track_table[339].lon = 65669014;
	simu_track_table[339].alt = 432;
	simu_track_table[339].relative_alt = -32;
	simu_track_table[339].vx = -39;
	simu_track_table[339].vy = 6;
	simu_track_table[339].vz = -46;
	simu_track_table[339].hdg = 351;

	simu_track_table[340].lat = 465185384;
	simu_track_table[340].lon = 65669017;
	simu_track_table[340].alt = 432;
	simu_track_table[340].relative_alt = -32;
	simu_track_table[340].vx = -38;
	simu_track_table[340].vy = 6;
	simu_track_table[340].vz = -46;
	simu_track_table[340].hdg = 352;

	simu_track_table[341].lat = 465185376;
	simu_track_table[341].lon = 65669020;
	simu_track_table[341].alt = 433;
	simu_track_table[341].relative_alt = -33;
	simu_track_table[341].vx = -38;
	simu_track_table[341].vy = 6;
	simu_track_table[341].vz = -46;
	simu_track_table[341].hdg = 352;

	simu_track_table[342].lat = 465185368;
	simu_track_table[342].lon = 65669023;
	simu_track_table[342].alt = 433;
	simu_track_table[342].relative_alt = -33;
	simu_track_table[342].vx = -38;
	simu_track_table[342].vy = 6;
	simu_track_table[342].vz = -46;
	simu_track_table[342].hdg = 352;

	simu_track_table[343].lat = 465185360;
	simu_track_table[343].lon = 65669025;
	simu_track_table[343].alt = 433;
	simu_track_table[343].relative_alt = -33;
	simu_track_table[343].vx = -38;
	simu_track_table[343].vy = 5;
	simu_track_table[343].vz = -46;
	simu_track_table[343].hdg = 352;

	simu_track_table[344].lat = 465185352;
	simu_track_table[344].lon = 65669028;
	simu_track_table[344].alt = 433;
	simu_track_table[344].relative_alt = -33;
	simu_track_table[344].vx = -37;
	simu_track_table[344].vy = 5;
	simu_track_table[344].vz = -46;
	simu_track_table[344].hdg = 352;

	simu_track_table[345].lat = 465185344;
	simu_track_table[345].lon = 65669031;
	simu_track_table[345].alt = 433;
	simu_track_table[345].relative_alt = -33;
	simu_track_table[345].vx = -37;
	simu_track_table[345].vy = 5;
	simu_track_table[345].vz = -46;
	simu_track_table[345].hdg = 352;

	simu_track_table[346].lat = 465185336;
	simu_track_table[346].lon = 65669033;
	simu_track_table[346].alt = 433;
	simu_track_table[346].relative_alt = -33;
	simu_track_table[346].vx = -37;
	simu_track_table[346].vy = 5;
	simu_track_table[346].vz = -46;
	simu_track_table[346].hdg = 352;

	simu_track_table[347].lat = 465185329;
	simu_track_table[347].lon = 65669036;
	simu_track_table[347].alt = 433;
	simu_track_table[347].relative_alt = -33;
	simu_track_table[347].vx = -37;
	simu_track_table[347].vy = 5;
	simu_track_table[347].vz = -46;
	simu_track_table[347].hdg = 352;

	simu_track_table[348].lat = 465185321;
	simu_track_table[348].lon = 65669038;
	simu_track_table[348].alt = 433;
	simu_track_table[348].relative_alt = -33;
	simu_track_table[348].vx = -36;
	simu_track_table[348].vy = 5;
	simu_track_table[348].vz = -46;
	simu_track_table[348].hdg = 352;

	simu_track_table[349].lat = 465185313;
	simu_track_table[349].lon = 65669041;
	simu_track_table[349].alt = 434;
	simu_track_table[349].relative_alt = -34;
	simu_track_table[349].vx = -36;
	simu_track_table[349].vy = 5;
	simu_track_table[349].vz = -46;
	simu_track_table[349].hdg = 352;

	simu_track_table[350].lat = 465185306;
	simu_track_table[350].lon = 65669043;
	simu_track_table[350].alt = 434;
	simu_track_table[350].relative_alt = -34;
	simu_track_table[350].vx = -36;
	simu_track_table[350].vy = 5;
	simu_track_table[350].vz = -46;
	simu_track_table[350].hdg = 352;

	simu_track_table[351].lat = 465185298;
	simu_track_table[351].lon = 65669046;
	simu_track_table[351].alt = 434;
	simu_track_table[351].relative_alt = -34;
	simu_track_table[351].vx = -36;
	simu_track_table[351].vy = 5;
	simu_track_table[351].vz = -46;
	simu_track_table[351].hdg = 352;

	simu_track_table[352].lat = 465185291;
	simu_track_table[352].lon = 65669048;
	simu_track_table[352].alt = 434;
	simu_track_table[352].relative_alt = -34;
	simu_track_table[352].vx = -35;
	simu_track_table[352].vy = 5;
	simu_track_table[352].vz = -46;
	simu_track_table[352].hdg = 353;

	simu_track_table[353].lat = 465185283;
	simu_track_table[353].lon = 65669051;
	simu_track_table[353].alt = 434;
	simu_track_table[353].relative_alt = -34;
	simu_track_table[353].vx = -35;
	simu_track_table[353].vy = 5;
	simu_track_table[353].vz = -46;
	simu_track_table[353].hdg = 353;

	simu_track_table[354].lat = 465185275;
	simu_track_table[354].lon = 65669053;
	simu_track_table[354].alt = 434;
	simu_track_table[354].relative_alt = -34;
	simu_track_table[354].vx = -35;
	simu_track_table[354].vy = 4;
	simu_track_table[354].vz = -46;
	simu_track_table[354].hdg = 353;

	simu_track_table[355].lat = 465185268;
	simu_track_table[355].lon = 65669055;
	simu_track_table[355].alt = 434;
	simu_track_table[355].relative_alt = -34;
	simu_track_table[355].vx = -35;
	simu_track_table[355].vy = 4;
	simu_track_table[355].vz = -46;
	simu_track_table[355].hdg = 353;

	simu_track_table[356].lat = 465185261;
	simu_track_table[356].lon = 65669057;
	simu_track_table[356].alt = 434;
	simu_track_table[356].relative_alt = -34;
	simu_track_table[356].vx = -34;
	simu_track_table[356].vy = 4;
	simu_track_table[356].vz = -46;
	simu_track_table[356].hdg = 353;

	simu_track_table[357].lat = 465185253;
	simu_track_table[357].lon = 65669060;
	simu_track_table[357].alt = 434;
	simu_track_table[357].relative_alt = -34;
	simu_track_table[357].vx = -34;
	simu_track_table[357].vy = 4;
	simu_track_table[357].vz = -46;
	simu_track_table[357].hdg = 353;

	simu_track_table[358].lat = 465185246;
	simu_track_table[358].lon = 65669062;
	simu_track_table[358].alt = 435;
	simu_track_table[358].relative_alt = -35;
	simu_track_table[358].vx = -34;
	simu_track_table[358].vy = 4;
	simu_track_table[358].vz = -46;
	simu_track_table[358].hdg = 353;

	simu_track_table[359].lat = 465185239;
	simu_track_table[359].lon = 65669064;
	simu_track_table[359].alt = 435;
	simu_track_table[359].relative_alt = -35;
	simu_track_table[359].vx = -34;
	simu_track_table[359].vy = 4;
	simu_track_table[359].vz = -46;
	simu_track_table[359].hdg = 353;

	simu_track_table[360].lat = 465185231;
	simu_track_table[360].lon = 65669066;
	simu_track_table[360].alt = 435;
	simu_track_table[360].relative_alt = -35;
	simu_track_table[360].vx = -33;
	simu_track_table[360].vy = 4;
	simu_track_table[360].vz = -46;
	simu_track_table[360].hdg = 353;

	simu_track_table[361].lat = 465185224;
	simu_track_table[361].lon = 65669068;
	simu_track_table[361].alt = 435;
	simu_track_table[361].relative_alt = -35;
	simu_track_table[361].vx = -33;
	simu_track_table[361].vy = 4;
	simu_track_table[361].vz = -46;
	simu_track_table[361].hdg = 353;

	simu_track_table[362].lat = 465185217;
	simu_track_table[362].lon = 65669070;
	simu_track_table[362].alt = 435;
	simu_track_table[362].relative_alt = -35;
	simu_track_table[362].vx = -33;
	simu_track_table[362].vy = 4;
	simu_track_table[362].vz = -46;
	simu_track_table[362].hdg = 353;

	simu_track_table[363].lat = 465185210;
	simu_track_table[363].lon = 65669072;
	simu_track_table[363].alt = 435;
	simu_track_table[363].relative_alt = -35;
	simu_track_table[363].vx = -32;
	simu_track_table[363].vy = 4;
	simu_track_table[363].vz = -46;
	simu_track_table[363].hdg = 353;

	simu_track_table[364].lat = 465185203;
	simu_track_table[364].lon = 65669074;
	simu_track_table[364].alt = 435;
	simu_track_table[364].relative_alt = -35;
	simu_track_table[364].vx = -32;
	simu_track_table[364].vy = 4;
	simu_track_table[364].vz = -46;
	simu_track_table[364].hdg = 354;

	simu_track_table[365].lat = 465185196;
	simu_track_table[365].lon = 65669076;
	simu_track_table[365].alt = 435;
	simu_track_table[365].relative_alt = -35;
	simu_track_table[365].vx = -32;
	simu_track_table[365].vy = 4;
	simu_track_table[365].vz = -46;
	simu_track_table[365].hdg = 354;

	simu_track_table[366].lat = 465185189;
	simu_track_table[366].lon = 65669078;
	simu_track_table[366].alt = 435;
	simu_track_table[366].relative_alt = -35;
	simu_track_table[366].vx = -32;
	simu_track_table[366].vy = 3;
	simu_track_table[366].vz = -46;
	simu_track_table[366].hdg = 354;

	simu_track_table[367].lat = 465185182;
	simu_track_table[367].lon = 65669080;
	simu_track_table[367].alt = 436;
	simu_track_table[367].relative_alt = -36;
	simu_track_table[367].vx = -31;
	simu_track_table[367].vy = 3;
	simu_track_table[367].vz = -46;
	simu_track_table[367].hdg = 354;

	simu_track_table[368].lat = 465185175;
	simu_track_table[368].lon = 65669082;
	simu_track_table[368].alt = 436;
	simu_track_table[368].relative_alt = -36;
	simu_track_table[368].vx = -31;
	simu_track_table[368].vy = 3;
	simu_track_table[368].vz = -46;
	simu_track_table[368].hdg = 354;

	simu_track_table[369].lat = 465185168;
	simu_track_table[369].lon = 65669083;
	simu_track_table[369].alt = 436;
	simu_track_table[369].relative_alt = -36;
	simu_track_table[369].vx = -31;
	simu_track_table[369].vy = 3;
	simu_track_table[369].vz = -46;
	simu_track_table[369].hdg = 354;

	simu_track_table[370].lat = 465185161;
	simu_track_table[370].lon = 65669085;
	simu_track_table[370].alt = 436;
	simu_track_table[370].relative_alt = -36;
	simu_track_table[370].vx = -30;
	simu_track_table[370].vy = 3;
	simu_track_table[370].vz = -46;
	simu_track_table[370].hdg = 354;

	simu_track_table[371].lat = 465185155;
	simu_track_table[371].lon = 65669087;
	simu_track_table[371].alt = 436;
	simu_track_table[371].relative_alt = -36;
	simu_track_table[371].vx = -30;
	simu_track_table[371].vy = 3;
	simu_track_table[371].vz = -46;
	simu_track_table[371].hdg = 354;

	simu_track_table[372].lat = 465185148;
	simu_track_table[372].lon = 65669089;
	simu_track_table[372].alt = 436;
	simu_track_table[372].relative_alt = -36;
	simu_track_table[372].vx = -30;
	simu_track_table[372].vy = 3;
	simu_track_table[372].vz = -46;
	simu_track_table[372].hdg = 354;

	simu_track_table[373].lat = 465185141;
	simu_track_table[373].lon = 65669090;
	simu_track_table[373].alt = 436;
	simu_track_table[373].relative_alt = -36;
	simu_track_table[373].vx = -29;
	simu_track_table[373].vy = 3;
	simu_track_table[373].vz = -46;
	simu_track_table[373].hdg = 354;

	simu_track_table[374].lat = 465185135;
	simu_track_table[374].lon = 65669092;
	simu_track_table[374].alt = 436;
	simu_track_table[374].relative_alt = -36;
	simu_track_table[374].vx = -29;
	simu_track_table[374].vy = 3;
	simu_track_table[374].vz = -46;
	simu_track_table[374].hdg = 354;

	simu_track_table[375].lat = 465185128;
	simu_track_table[375].lon = 65669094;
	simu_track_table[375].alt = 437;
	simu_track_table[375].relative_alt = -37;
	simu_track_table[375].vx = -29;
	simu_track_table[375].vy = 3;
	simu_track_table[375].vz = -46;
	simu_track_table[375].hdg = 354;

	simu_track_table[376].lat = 465185122;
	simu_track_table[376].lon = 65669095;
	simu_track_table[376].alt = 437;
	simu_track_table[376].relative_alt = -37;
	simu_track_table[376].vx = -29;
	simu_track_table[376].vy = 3;
	simu_track_table[376].vz = -46;
	simu_track_table[376].hdg = 355;

	simu_track_table[377].lat = 465185115;
	simu_track_table[377].lon = 65669097;
	simu_track_table[377].alt = 437;
	simu_track_table[377].relative_alt = -37;
	simu_track_table[377].vx = -28;
	simu_track_table[377].vy = 3;
	simu_track_table[377].vz = -46;
	simu_track_table[377].hdg = 355;

	simu_track_table[378].lat = 465185109;
	simu_track_table[378].lon = 65669098;
	simu_track_table[378].alt = 437;
	simu_track_table[378].relative_alt = -37;
	simu_track_table[378].vx = -28;
	simu_track_table[378].vy = 3;
	simu_track_table[378].vz = -46;
	simu_track_table[378].hdg = 355;

	simu_track_table[379].lat = 465185102;
	simu_track_table[379].lon = 65669100;
	simu_track_table[379].alt = 437;
	simu_track_table[379].relative_alt = -37;
	simu_track_table[379].vx = -28;
	simu_track_table[379].vy = 3;
	simu_track_table[379].vz = -46;
	simu_track_table[379].hdg = 355;

	simu_track_table[380].lat = 465185096;
	simu_track_table[380].lon = 65669101;
	simu_track_table[380].alt = 437;
	simu_track_table[380].relative_alt = -37;
	simu_track_table[380].vx = -27;
	simu_track_table[380].vy = 2;
	simu_track_table[380].vz = -46;
	simu_track_table[380].hdg = 355;

	simu_track_table[381].lat = 465185090;
	simu_track_table[381].lon = 65669102;
	simu_track_table[381].alt = 437;
	simu_track_table[381].relative_alt = -37;
	simu_track_table[381].vx = -27;
	simu_track_table[381].vy = 2;
	simu_track_table[381].vz = -46;
	simu_track_table[381].hdg = 355;

	simu_track_table[382].lat = 465185084;
	simu_track_table[382].lon = 65669104;
	simu_track_table[382].alt = 437;
	simu_track_table[382].relative_alt = -37;
	simu_track_table[382].vx = -27;
	simu_track_table[382].vy = 2;
	simu_track_table[382].vz = -46;
	simu_track_table[382].hdg = 355;

	simu_track_table[383].lat = 465185078;
	simu_track_table[383].lon = 65669105;
	simu_track_table[383].alt = 437;
	simu_track_table[383].relative_alt = -37;
	simu_track_table[383].vx = -27;
	simu_track_table[383].vy = 2;
	simu_track_table[383].vz = -46;
	simu_track_table[383].hdg = 355;

	simu_track_table[384].lat = 465185072;
	simu_track_table[384].lon = 65669106;
	simu_track_table[384].alt = 438;
	simu_track_table[384].relative_alt = -38;
	simu_track_table[384].vx = -26;
	simu_track_table[384].vy = 2;
	simu_track_table[384].vz = -46;
	simu_track_table[384].hdg = 355;

	simu_track_table[385].lat = 465185066;
	simu_track_table[385].lon = 65669108;
	simu_track_table[385].alt = 438;
	simu_track_table[385].relative_alt = -38;
	simu_track_table[385].vx = -26;
	simu_track_table[385].vy = 2;
	simu_track_table[385].vz = -46;
	simu_track_table[385].hdg = 355;

	simu_track_table[386].lat = 465185060;
	simu_track_table[386].lon = 65669109;
	simu_track_table[386].alt = 438;
	simu_track_table[386].relative_alt = -38;
	simu_track_table[386].vx = -26;
	simu_track_table[386].vy = 2;
	simu_track_table[386].vz = -46;
	simu_track_table[386].hdg = 355;

	simu_track_table[387].lat = 465185054;
	simu_track_table[387].lon = 65669110;
	simu_track_table[387].alt = 438;
	simu_track_table[387].relative_alt = -38;
	simu_track_table[387].vx = -25;
	simu_track_table[387].vy = 2;
	simu_track_table[387].vz = -46;
	simu_track_table[387].hdg = 355;

	simu_track_table[388].lat = 465185048;
	simu_track_table[388].lon = 65669111;
	simu_track_table[388].alt = 438;
	simu_track_table[388].relative_alt = -38;
	simu_track_table[388].vx = -25;
	simu_track_table[388].vy = 2;
	simu_track_table[388].vz = -46;
	simu_track_table[388].hdg = 356;

	simu_track_table[389].lat = 465185042;
	simu_track_table[389].lon = 65669112;
	simu_track_table[389].alt = 438;
	simu_track_table[389].relative_alt = -38;
	simu_track_table[389].vx = -25;
	simu_track_table[389].vy = 2;
	simu_track_table[389].vz = -46;
	simu_track_table[389].hdg = 356;

	simu_track_table[390].lat = 465185037;
	simu_track_table[390].lon = 65669113;
	simu_track_table[390].alt = 438;
	simu_track_table[390].relative_alt = -38;
	simu_track_table[390].vx = -24;
	simu_track_table[390].vy = 2;
	simu_track_table[390].vz = -46;
	simu_track_table[390].hdg = 356;

	simu_track_table[391].lat = 465185031;
	simu_track_table[391].lon = 65669115;
	simu_track_table[391].alt = 438;
	simu_track_table[391].relative_alt = -38;
	simu_track_table[391].vx = -24;
	simu_track_table[391].vy = 2;
	simu_track_table[391].vz = -46;
	simu_track_table[391].hdg = 356;

	simu_track_table[392].lat = 465185025;
	simu_track_table[392].lon = 65669116;
	simu_track_table[392].alt = 438;
	simu_track_table[392].relative_alt = -38;
	simu_track_table[392].vx = -24;
	simu_track_table[392].vy = 2;
	simu_track_table[392].vz = -46;
	simu_track_table[392].hdg = 356;

	simu_track_table[393].lat = 465185020;
	simu_track_table[393].lon = 65669117;
	simu_track_table[393].alt = 439;
	simu_track_table[393].relative_alt = -39;
	simu_track_table[393].vx = -23;
	simu_track_table[393].vy = 2;
	simu_track_table[393].vz = -46;
	simu_track_table[393].hdg = 356;

	simu_track_table[394].lat = 465185014;
	simu_track_table[394].lon = 65669118;
	simu_track_table[394].alt = 439;
	simu_track_table[394].relative_alt = -39;
	simu_track_table[394].vx = -23;
	simu_track_table[394].vy = 2;
	simu_track_table[394].vz = -46;
	simu_track_table[394].hdg = 356;

	simu_track_table[395].lat = 465185009;
	simu_track_table[395].lon = 65669119;
	simu_track_table[395].alt = 439;
	simu_track_table[395].relative_alt = -39;
	simu_track_table[395].vx = -23;
	simu_track_table[395].vy = 2;
	simu_track_table[395].vz = -46;
	simu_track_table[395].hdg = 356;

	simu_track_table[396].lat = 465185004;
	simu_track_table[396].lon = 65669119;
	simu_track_table[396].alt = 439;
	simu_track_table[396].relative_alt = -39;
	simu_track_table[396].vx = -23;
	simu_track_table[396].vy = 1;
	simu_track_table[396].vz = -46;
	simu_track_table[396].hdg = 356;

	simu_track_table[397].lat = 465184998;
	simu_track_table[397].lon = 65669120;
	simu_track_table[397].alt = 439;
	simu_track_table[397].relative_alt = -39;
	simu_track_table[397].vx = -22;
	simu_track_table[397].vy = 1;
	simu_track_table[397].vz = -46;
	simu_track_table[397].hdg = 356;

	simu_track_table[398].lat = 465184993;
	simu_track_table[398].lon = 65669121;
	simu_track_table[398].alt = 439;
	simu_track_table[398].relative_alt = -39;
	simu_track_table[398].vx = -22;
	simu_track_table[398].vy = 1;
	simu_track_table[398].vz = -46;
	simu_track_table[398].hdg = 356;

	simu_track_table[399].lat = 465184988;
	simu_track_table[399].lon = 65669122;
	simu_track_table[399].alt = 439;
	simu_track_table[399].relative_alt = -39;
	simu_track_table[399].vx = -22;
	simu_track_table[399].vy = 1;
	simu_track_table[399].vz = -46;
	simu_track_table[399].hdg = 356;
}
/*
	simu_track_table[400].lat = 465184983;
	simu_track_table[400].lon = 65669123;
	simu_track_table[400].alt = 439;
	simu_track_table[400].relative_alt = -39;
	simu_track_table[400].vx = -21;
	simu_track_table[400].vy = 1;
	simu_track_table[400].vz = -46;
	simu_track_table[400].hdg = 357;

	simu_track_table[401].lat = 465184978;
	simu_track_table[401].lon = 65669124;
	simu_track_table[401].alt = 440;
	simu_track_table[401].relative_alt = -40;
	simu_track_table[401].vx = -21;
	simu_track_table[401].vy = 1;
	simu_track_table[401].vz = -46;
	simu_track_table[401].hdg = 357;

	simu_track_table[402].lat = 465184973;
	simu_track_table[402].lon = 65669124;
	simu_track_table[402].alt = 440;
	simu_track_table[402].relative_alt = -40;
	simu_track_table[402].vx = -21;
	simu_track_table[402].vy = 1;
	simu_track_table[402].vz = -46;
	simu_track_table[402].hdg = 357;

	simu_track_table[403].lat = 465184968;
	simu_track_table[403].lon = 65669125;
	simu_track_table[403].alt = 440;
	simu_track_table[403].relative_alt = -40;
	simu_track_table[403].vx = -20;
	simu_track_table[403].vy = 1;
	simu_track_table[403].vz = -46;
	simu_track_table[403].hdg = 357;

	simu_track_table[404].lat = 465184963;
	simu_track_table[404].lon = 65669126;
	simu_track_table[404].alt = 440;
	simu_track_table[404].relative_alt = -40;
	simu_track_table[404].vx = -20;
	simu_track_table[404].vy = 1;
	simu_track_table[404].vz = -46;
	simu_track_table[404].hdg = 357;

	simu_track_table[405].lat = 465184958;
	simu_track_table[405].lon = 65669127;
	simu_track_table[405].alt = 440;
	simu_track_table[405].relative_alt = -40;
	simu_track_table[405].vx = -20;
	simu_track_table[405].vy = 1;
	simu_track_table[405].vz = -46;
	simu_track_table[405].hdg = 357;

	simu_track_table[406].lat = 465184954;
	simu_track_table[406].lon = 65669127;
	simu_track_table[406].alt = 440;
	simu_track_table[406].relative_alt = -40;
	simu_track_table[406].vx = -19;
	simu_track_table[406].vy = 1;
	simu_track_table[406].vz = -46;
	simu_track_table[406].hdg = 357;

	simu_track_table[407].lat = 465184949;
	simu_track_table[407].lon = 65669128;
	simu_track_table[407].alt = 440;
	simu_track_table[407].relative_alt = -40;
	simu_track_table[407].vx = -19;
	simu_track_table[407].vy = 1;
	simu_track_table[407].vz = -46;
	simu_track_table[407].hdg = 357;

	simu_track_table[408].lat = 465184944;
	simu_track_table[408].lon = 65669129;
	simu_track_table[408].alt = 440;
	simu_track_table[408].relative_alt = -40;
	simu_track_table[408].vx = -19;
	simu_track_table[408].vy = 1;
	simu_track_table[408].vz = -46;
	simu_track_table[408].hdg = 357;

	simu_track_table[409].lat = 465184940;
	simu_track_table[409].lon = 65669129;
	simu_track_table[409].alt = 440;
	simu_track_table[409].relative_alt = -40;
	simu_track_table[409].vx = -19;
	simu_track_table[409].vy = 1;
	simu_track_table[409].vz = -46;
	simu_track_table[409].hdg = 357;

	simu_track_table[410].lat = 465184935;
	simu_track_table[410].lon = 65669130;
	simu_track_table[410].alt = 441;
	simu_track_table[410].relative_alt = -41;
	simu_track_table[410].vx = -18;
	simu_track_table[410].vy = 1;
	simu_track_table[410].vz = -46;
	simu_track_table[410].hdg = 357;

	simu_track_table[411].lat = 465184931;
	simu_track_table[411].lon = 65669130;
	simu_track_table[411].alt = 441;
	simu_track_table[411].relative_alt = -41;
	simu_track_table[411].vx = -18;
	simu_track_table[411].vy = 1;
	simu_track_table[411].vz = -46;
	simu_track_table[411].hdg = 357;

	simu_track_table[412].lat = 465184927;
	simu_track_table[412].lon = 65669131;
	simu_track_table[412].alt = 441;
	simu_track_table[412].relative_alt = -41;
	simu_track_table[412].vx = -18;
	simu_track_table[412].vy = 1;
	simu_track_table[412].vz = -46;
	simu_track_table[412].hdg = 358;

	simu_track_table[413].lat = 465184922;
	simu_track_table[413].lon = 65669131;
	simu_track_table[413].alt = 441;
	simu_track_table[413].relative_alt = -41;
	simu_track_table[413].vx = -17;
	simu_track_table[413].vy = 1;
	simu_track_table[413].vz = -46;
	simu_track_table[413].hdg = 358;

	simu_track_table[414].lat = 465184918;
	simu_track_table[414].lon = 65669132;
	simu_track_table[414].alt = 441;
	simu_track_table[414].relative_alt = -41;
	simu_track_table[414].vx = -17;
	simu_track_table[414].vy = 1;
	simu_track_table[414].vz = -46;
	simu_track_table[414].hdg = 358;

	simu_track_table[415].lat = 465184914;
	simu_track_table[415].lon = 65669132;
	simu_track_table[415].alt = 441;
	simu_track_table[415].relative_alt = -41;
	simu_track_table[415].vx = -17;
	simu_track_table[415].vy = 1;
	simu_track_table[415].vz = -46;
	simu_track_table[415].hdg = 358;

	simu_track_table[416].lat = 465184910;
	simu_track_table[416].lon = 65669133;
	simu_track_table[416].alt = 441;
	simu_track_table[416].relative_alt = -41;
	simu_track_table[416].vx = -16;
	simu_track_table[416].vy = 1;
	simu_track_table[416].vz = -46;
	simu_track_table[416].hdg = 358;

	simu_track_table[417].lat = 465184906;
	simu_track_table[417].lon = 65669133;
	simu_track_table[417].alt = 441;
	simu_track_table[417].relative_alt = -41;
	simu_track_table[417].vx = -16;
	simu_track_table[417].vy = 1;
	simu_track_table[417].vz = -46;
	simu_track_table[417].hdg = 358;

	simu_track_table[418].lat = 465184902;
	simu_track_table[418].lon = 65669134;
	simu_track_table[418].alt = 441;
	simu_track_table[418].relative_alt = -41;
	simu_track_table[418].vx = -16;
	simu_track_table[418].vy = 1;
	simu_track_table[418].vz = -46;
	simu_track_table[418].hdg = 358;

	simu_track_table[419].lat = 465184898;
	simu_track_table[419].lon = 65669134;
	simu_track_table[419].alt = 442;
	simu_track_table[419].relative_alt = -42;
	simu_track_table[419].vx = -15;
	simu_track_table[419].vy = 1;
	simu_track_table[419].vz = -46;
	simu_track_table[419].hdg = 358;

	simu_track_table[420].lat = 465184894;
	simu_track_table[420].lon = 65669134;
	simu_track_table[420].alt = 442;
	simu_track_table[420].relative_alt = -42;
	simu_track_table[420].vx = -15;
	simu_track_table[420].vy = 1;
	simu_track_table[420].vz = -46;
	simu_track_table[420].hdg = 358;

	simu_track_table[421].lat = 465184891;
	simu_track_table[421].lon = 65669135;
	simu_track_table[421].alt = 442;
	simu_track_table[421].relative_alt = -42;
	simu_track_table[421].vx = -15;
	simu_track_table[421].vy = 0;
	simu_track_table[421].vz = -46;
	simu_track_table[421].hdg = 358;

	simu_track_table[422].lat = 465184887;
	simu_track_table[422].lon = 65669135;
	simu_track_table[422].alt = 442;
	simu_track_table[422].relative_alt = -42;
	simu_track_table[422].vx = -15;
	simu_track_table[422].vy = 0;
	simu_track_table[422].vz = -46;
	simu_track_table[422].hdg = 358;

	simu_track_table[423].lat = 465184883;
	simu_track_table[423].lon = 65669135;
	simu_track_table[423].alt = 442;
	simu_track_table[423].relative_alt = -42;
	simu_track_table[423].vx = -14;
	simu_track_table[423].vy = 0;
	simu_track_table[423].vz = -46;
	simu_track_table[423].hdg = 358;

	simu_track_table[424].lat = 465184880;
	simu_track_table[424].lon = 65669136;
	simu_track_table[424].alt = 442;
	simu_track_table[424].relative_alt = -42;
	simu_track_table[424].vx = -14;
	simu_track_table[424].vy = 0;
	simu_track_table[424].vz = -46;
	simu_track_table[424].hdg = 358;

	simu_track_table[425].lat = 465184876;
	simu_track_table[425].lon = 65669136;
	simu_track_table[425].alt = 442;
	simu_track_table[425].relative_alt = -42;
	simu_track_table[425].vx = -14;
	simu_track_table[425].vy = 0;
	simu_track_table[425].vz = -46;
	simu_track_table[425].hdg = 358;

	simu_track_table[426].lat = 465184873;
	simu_track_table[426].lon = 65669136;
	simu_track_table[426].alt = 442;
	simu_track_table[426].relative_alt = -42;
	simu_track_table[426].vx = -13;
	simu_track_table[426].vy = 0;
	simu_track_table[426].vz = -46;
	simu_track_table[426].hdg = 358;

	simu_track_table[427].lat = 465184870;
	simu_track_table[427].lon = 65669136;
	simu_track_table[427].alt = 443;
	simu_track_table[427].relative_alt = -43;
	simu_track_table[427].vx = -13;
	simu_track_table[427].vy = 0;
	simu_track_table[427].vz = -46;
	simu_track_table[427].hdg = 358;

	simu_track_table[428].lat = 465184867;
	simu_track_table[428].lon = 65669137;
	simu_track_table[428].alt = 443;
	simu_track_table[428].relative_alt = -43;
	simu_track_table[428].vx = -13;
	simu_track_table[428].vy = 0;
	simu_track_table[428].vz = -46;
	simu_track_table[428].hdg = 358;

	simu_track_table[429].lat = 465184863;
	simu_track_table[429].lon = 65669137;
	simu_track_table[429].alt = 443;
	simu_track_table[429].relative_alt = -43;
	simu_track_table[429].vx = -12;
	simu_track_table[429].vy = 0;
	simu_track_table[429].vz = -46;
	simu_track_table[429].hdg = 359;

	simu_track_table[430].lat = 465184860;
	simu_track_table[430].lon = 65669137;
	simu_track_table[430].alt = 443;
	simu_track_table[430].relative_alt = -43;
	simu_track_table[430].vx = -12;
	simu_track_table[430].vy = 0;
	simu_track_table[430].vz = -46;
	simu_track_table[430].hdg = 359;

	simu_track_table[431].lat = 465184857;
	simu_track_table[431].lon = 65669137;
	simu_track_table[431].alt = 443;
	simu_track_table[431].relative_alt = -43;
	simu_track_table[431].vx = -12;
	simu_track_table[431].vy = 0;
	simu_track_table[431].vz = -46;
	simu_track_table[431].hdg = 359;

	simu_track_table[432].lat = 465184854;
	simu_track_table[432].lon = 65669137;
	simu_track_table[432].alt = 443;
	simu_track_table[432].relative_alt = -43;
	simu_track_table[432].vx = -12;
	simu_track_table[432].vy = 0;
	simu_track_table[432].vz = -46;
	simu_track_table[432].hdg = 359;

	simu_track_table[433].lat = 465184851;
	simu_track_table[433].lon = 65669138;
	simu_track_table[433].alt = 443;
	simu_track_table[433].relative_alt = -43;
	simu_track_table[433].vx = -11;
	simu_track_table[433].vy = 0;
	simu_track_table[433].vz = -46;
	simu_track_table[433].hdg = 359;

	simu_track_table[434].lat = 465184849;
	simu_track_table[434].lon = 65669138;
	simu_track_table[434].alt = 443;
	simu_track_table[434].relative_alt = -43;
	simu_track_table[434].vx = -11;
	simu_track_table[434].vy = 0;
	simu_track_table[434].vz = -46;
	simu_track_table[434].hdg = 359;

	simu_track_table[435].lat = 465184846;
	simu_track_table[435].lon = 65669138;
	simu_track_table[435].alt = 443;
	simu_track_table[435].relative_alt = -43;
	simu_track_table[435].vx = -11;
	simu_track_table[435].vy = 0;
	simu_track_table[435].vz = -46;
	simu_track_table[435].hdg = 359;

	simu_track_table[436].lat = 465184843;
	simu_track_table[436].lon = 65669138;
	simu_track_table[436].alt = 444;
	simu_track_table[436].relative_alt = -44;
	simu_track_table[436].vx = -10;
	simu_track_table[436].vy = 0;
	simu_track_table[436].vz = -46;
	simu_track_table[436].hdg = 359;

	simu_track_table[437].lat = 465184840;
	simu_track_table[437].lon = 65669138;
	simu_track_table[437].alt = 444;
	simu_track_table[437].relative_alt = -44;
	simu_track_table[437].vx = -10;
	simu_track_table[437].vy = 0;
	simu_track_table[437].vz = -46;
	simu_track_table[437].hdg = 359;

	simu_track_table[438].lat = 465184838;
	simu_track_table[438].lon = 65669138;
	simu_track_table[438].alt = 444;
	simu_track_table[438].relative_alt = -44;
	simu_track_table[438].vx = -10;
	simu_track_table[438].vy = 0;
	simu_track_table[438].vz = -46;
	simu_track_table[438].hdg = 359;

	simu_track_table[439].lat = 465184835;
	simu_track_table[439].lon = 65669138;
	simu_track_table[439].alt = 444;
	simu_track_table[439].relative_alt = -44;
	simu_track_table[439].vx = -10;
	simu_track_table[439].vy = 0;
	simu_track_table[439].vz = -46;
	simu_track_table[439].hdg = 359;

	simu_track_table[440].lat = 465184833;
	simu_track_table[440].lon = 65669138;
	simu_track_table[440].alt = 444;
	simu_track_table[440].relative_alt = -44;
	simu_track_table[440].vx = -9;
	simu_track_table[440].vy = 0;
	simu_track_table[440].vz = -46;
	simu_track_table[440].hdg = 359;

	simu_track_table[441].lat = 465184830;
	simu_track_table[441].lon = 65669139;
	simu_track_table[441].alt = 444;
	simu_track_table[441].relative_alt = -44;
	simu_track_table[441].vx = -9;
	simu_track_table[441].vy = 0;
	simu_track_table[441].vz = -46;
	simu_track_table[441].hdg = 359;

	simu_track_table[442].lat = 465184828;
	simu_track_table[442].lon = 65669139;
	simu_track_table[442].alt = 444;
	simu_track_table[442].relative_alt = -44;
	simu_track_table[442].vx = -9;
	simu_track_table[442].vy = 0;
	simu_track_table[442].vz = -46;
	simu_track_table[442].hdg = 359;

	simu_track_table[443].lat = 465184826;
	simu_track_table[443].lon = 65669139;
	simu_track_table[443].alt = 444;
	simu_track_table[443].relative_alt = -44;
	simu_track_table[443].vx = -8;
	simu_track_table[443].vy = 0;
	simu_track_table[443].vz = -46;
	simu_track_table[443].hdg = 359;

	simu_track_table[444].lat = 465184824;
	simu_track_table[444].lon = 65669139;
	simu_track_table[444].alt = 444;
	simu_track_table[444].relative_alt = -44;
	simu_track_table[444].vx = -8;
	simu_track_table[444].vy = 0;
	simu_track_table[444].vz = -46;
	simu_track_table[444].hdg = 359;

	simu_track_table[445].lat = 465184822;
	simu_track_table[445].lon = 65669139;
	simu_track_table[445].alt = 445;
	simu_track_table[445].relative_alt = -45;
	simu_track_table[445].vx = -8;
	simu_track_table[445].vy = 0;
	simu_track_table[445].vz = -46;
	simu_track_table[445].hdg = 359;

	simu_track_table[446].lat = 465184819;
	simu_track_table[446].lon = 65669139;
	simu_track_table[446].alt = 445;
	simu_track_table[446].relative_alt = -45;
	simu_track_table[446].vx = -8;
	simu_track_table[446].vy = 0;
	simu_track_table[446].vz = -46;
	simu_track_table[446].hdg = 359;

	simu_track_table[447].lat = 465184817;
	simu_track_table[447].lon = 65669139;
	simu_track_table[447].alt = 445;
	simu_track_table[447].relative_alt = -45;
	simu_track_table[447].vx = -7;
	simu_track_table[447].vy = 0;
	simu_track_table[447].vz = -46;
	simu_track_table[447].hdg = 359;

	simu_track_table[448].lat = 465184816;
	simu_track_table[448].lon = 65669139;
	simu_track_table[448].alt = 445;
	simu_track_table[448].relative_alt = -45;
	simu_track_table[448].vx = -7;
	simu_track_table[448].vy = 0;
	simu_track_table[448].vz = -46;
	simu_track_table[448].hdg = 359;

	simu_track_table[449].lat = 465184814;
	simu_track_table[449].lon = 65669139;
	simu_track_table[449].alt = 445;
	simu_track_table[449].relative_alt = -45;
	simu_track_table[449].vx = -7;
	simu_track_table[449].vy = 0;
	simu_track_table[449].vz = -41;
	simu_track_table[449].hdg = 359;

	simu_track_table[450].lat = 465184811;
	simu_track_table[450].lon = 65669138;
	simu_track_table[450].alt = 445;
	simu_track_table[450].relative_alt = -45;
	simu_track_table[450].vx = -17;
	simu_track_table[450].vy = -11;
	simu_track_table[450].vz = -22;
	simu_track_table[450].hdg = 32;

	simu_track_table[451].lat = 465184802;
	simu_track_table[451].lon = 65669128;
	simu_track_table[451].alt = 445;
	simu_track_table[451].relative_alt = -45;
	simu_track_table[451].vx = -63;
	simu_track_table[451].vy = -56;
	simu_track_table[451].vz = -9;
	simu_track_table[451].hdg = 41;

	simu_track_table[452].lat = 465184780;
	simu_track_table[452].lon = 65669099;
	simu_track_table[452].alt = 445;
	simu_track_table[452].relative_alt = -45;
	simu_track_table[452].vx = -138;
	simu_track_table[452].vy = -120;
	simu_track_table[452].vz = 5;
	simu_track_table[452].hdg = 41;

	simu_track_table[453].lat = 465184739;
	simu_track_table[453].lon = 65669048;
	simu_track_table[453].alt = 445;
	simu_track_table[453].relative_alt = -45;
	simu_track_table[453].vx = -211;
	simu_track_table[453].vy = -177;
	simu_track_table[453].vz = 10;
	simu_track_table[453].hdg = 40;

	simu_track_table[454].lat = 465184685;
	simu_track_table[454].lon = 65668985;
	simu_track_table[454].alt = 445;
	simu_track_table[454].relative_alt = -45;
	simu_track_table[454].vx = -255;
	simu_track_table[454].vy = -205;
	simu_track_table[454].vz = 4;
	simu_track_table[454].hdg = 39;

	simu_track_table[455].lat = 465184622;
	simu_track_table[455].lon = 65668913;
	simu_track_table[455].alt = 445;
	simu_track_table[455].relative_alt = -45;
	simu_track_table[455].vx = -269;
	simu_track_table[455].vy = -207;
	simu_track_table[455].vz = -8;
	simu_track_table[455].hdg = 38;

	simu_track_table[456].lat = 465184560;
	simu_track_table[456].lon = 65668845;
	simu_track_table[456].alt = 445;
	simu_track_table[456].relative_alt = -45;
	simu_track_table[456].vx = -264;
	simu_track_table[456].vy = -196;
	simu_track_table[456].vz = -17;
	simu_track_table[456].hdg = 37;

	simu_track_table[457].lat = 465184497;
	simu_track_table[457].lon = 65668778;
	simu_track_table[457].alt = 445;
	simu_track_table[457].relative_alt = -45;
	simu_track_table[457].vx = -252;
	simu_track_table[457].vy = -183;
	simu_track_table[457].vz = -23;
	simu_track_table[457].hdg = 36;

	simu_track_table[458].lat = 465184439;
	simu_track_table[458].lon = 65668717;
	simu_track_table[458].alt = 445;
	simu_track_table[458].relative_alt = -45;
	simu_track_table[458].vx = -243;
	simu_track_table[458].vy = -176;
	simu_track_table[458].vz = -25;
	simu_track_table[458].hdg = 36;

	simu_track_table[459].lat = 465184380;
	simu_track_table[459].lon = 65668654;
	simu_track_table[459].alt = 445;
	simu_track_table[459].relative_alt = -45;
	simu_track_table[459].vx = -241;
	simu_track_table[459].vy = -175;
	simu_track_table[459].vz = -25;
	simu_track_table[459].hdg = 36;

	simu_track_table[460].lat = 465184322;
	simu_track_table[460].lon = 65668594;
	simu_track_table[460].alt = 445;
	simu_track_table[460].relative_alt = -45;
	simu_track_table[460].vx = -243;
	simu_track_table[460].vy = -178;
	simu_track_table[460].vz = -25;
	simu_track_table[460].hdg = 36;

	simu_track_table[461].lat = 465184262;
	simu_track_table[461].lon = 65668530;
	simu_track_table[461].alt = 445;
	simu_track_table[461].relative_alt = -45;
	simu_track_table[461].vx = -246;
	simu_track_table[461].vy = -180;
	simu_track_table[461].vz = -27;
	simu_track_table[461].hdg = 36;

	simu_track_table[462].lat = 465184204;
	simu_track_table[462].lon = 65668468;
	simu_track_table[462].alt = 446;
	simu_track_table[462].relative_alt = -46;
	simu_track_table[462].vx = -248;
	simu_track_table[462].vy = -181;
	simu_track_table[462].vz = -29;
	simu_track_table[462].hdg = 36;

	simu_track_table[463].lat = 465184143;
	simu_track_table[463].lon = 65668404;
	simu_track_table[463].alt = 446;
	simu_track_table[463].relative_alt = -46;
	simu_track_table[463].vx = -247;
	simu_track_table[463].vy = -179;
	simu_track_table[463].vz = -31;
	simu_track_table[463].hdg = 36;

	simu_track_table[464].lat = 465184085;
	simu_track_table[464].lon = 65668343;
	simu_track_table[464].alt = 446;
	simu_track_table[464].relative_alt = -46;
	simu_track_table[464].vx = -244;
	simu_track_table[464].vy = -175;
	simu_track_table[464].vz = -34;
	simu_track_table[464].hdg = 36;

	simu_track_table[465].lat = 465184025;
	simu_track_table[465].lon = 65668281;
	simu_track_table[465].alt = 446;
	simu_track_table[465].relative_alt = -46;
	simu_track_table[465].vx = -241;
	simu_track_table[465].vy = -172;
	simu_track_table[465].vz = -36;
	simu_track_table[465].hdg = 36;

	simu_track_table[466].lat = 465183968;
	simu_track_table[466].lon = 65668222;
	simu_track_table[466].alt = 446;
	simu_track_table[466].relative_alt = -46;
	simu_track_table[466].vx = -239;
	simu_track_table[466].vy = -170;
	simu_track_table[466].vz = -38;
	simu_track_table[466].hdg = 35;

	simu_track_table[467].lat = 465183910;
	simu_track_table[467].lon = 65668161;
	simu_track_table[467].alt = 446;
	simu_track_table[467].relative_alt = -46;
	simu_track_table[467].vx = -237;
	simu_track_table[467].vy = -168;
	simu_track_table[467].vz = -40;
	simu_track_table[467].hdg = 35;

	simu_track_table[468].lat = 465183854;
	simu_track_table[468].lon = 65668104;
	simu_track_table[468].alt = 446;
	simu_track_table[468].relative_alt = -46;
	simu_track_table[468].vx = -235;
	simu_track_table[468].vy = -167;
	simu_track_table[468].vz = -43;
	simu_track_table[468].hdg = 35;

	simu_track_table[469].lat = 465183796;
	simu_track_table[469].lon = 65668044;
	simu_track_table[469].alt = 446;
	simu_track_table[469].relative_alt = -46;
	simu_track_table[469].vx = -234;
	simu_track_table[469].vy = -166;
	simu_track_table[469].vz = -45;
	simu_track_table[469].hdg = 35;

	simu_track_table[470].lat = 465183741;
	simu_track_table[470].lon = 65667987;
	simu_track_table[470].alt = 446;
	simu_track_table[470].relative_alt = -46;
	simu_track_table[470].vx = -233;
	simu_track_table[470].vy = -166;
	simu_track_table[470].vz = -47;
	simu_track_table[470].hdg = 35;

	simu_track_table[471].lat = 465183684;
	simu_track_table[471].lon = 65667928;
	simu_track_table[471].alt = 446;
	simu_track_table[471].relative_alt = -46;
	simu_track_table[471].vx = -230;
	simu_track_table[471].vy = -164;
	simu_track_table[471].vz = -47;
	simu_track_table[471].hdg = 35;

	simu_track_table[472].lat = 465183630;
	simu_track_table[472].lon = 65667872;
	simu_track_table[472].alt = 447;
	simu_track_table[472].relative_alt = -47;
	simu_track_table[472].vx = -225;
	simu_track_table[472].vy = -160;
	simu_track_table[472].vz = -48;
	simu_track_table[472].hdg = 35;

	simu_track_table[473].lat = 465183576;
	simu_track_table[473].lon = 65667815;
	simu_track_table[473].alt = 447;
	simu_track_table[473].relative_alt = -47;
	simu_track_table[473].vx = -217;
	simu_track_table[473].vy = -154;
	simu_track_table[473].vz = -49;
	simu_track_table[473].hdg = 35;

	simu_track_table[474].lat = 465183526;
	simu_track_table[474].lon = 65667763;
	simu_track_table[474].alt = 447;
	simu_track_table[474].relative_alt = -47;
	simu_track_table[474].vx = -206;
	simu_track_table[474].vy = -146;
	simu_track_table[474].vz = -50;
	simu_track_table[474].hdg = 35;

	simu_track_table[475].lat = 465183477;
	simu_track_table[475].lon = 65667712;
	simu_track_table[475].alt = 447;
	simu_track_table[475].relative_alt = -47;
	simu_track_table[475].vx = -194;
	simu_track_table[475].vy = -138;
	simu_track_table[475].vz = -50;
	simu_track_table[475].hdg = 35;

	simu_track_table[476].lat = 465183432;
	simu_track_table[476].lon = 65667666;
	simu_track_table[476].alt = 447;
	simu_track_table[476].relative_alt = -47;
	simu_track_table[476].vx = -182;
	simu_track_table[476].vy = -129;
	simu_track_table[476].vz = -50;
	simu_track_table[476].hdg = 35;

	simu_track_table[477].lat = 465183389;
	simu_track_table[477].lon = 65667621;
	simu_track_table[477].alt = 447;
	simu_track_table[477].relative_alt = -47;
	simu_track_table[477].vx = -170;
	simu_track_table[477].vy = -121;
	simu_track_table[477].vz = -50;
	simu_track_table[477].hdg = 35;

	simu_track_table[478].lat = 465183350;
	simu_track_table[478].lon = 65667580;
	simu_track_table[478].alt = 447;
	simu_track_table[478].relative_alt = -47;
	simu_track_table[478].vx = -160;
	simu_track_table[478].vy = -113;
	simu_track_table[478].vz = -49;
	simu_track_table[478].hdg = 35;

	simu_track_table[479].lat = 465183312;
	simu_track_table[479].lon = 65667540;
	simu_track_table[479].alt = 447;
	simu_track_table[479].relative_alt = -47;
	simu_track_table[479].vx = -149;
	simu_track_table[479].vy = -106;
	simu_track_table[479].vz = -49;
	simu_track_table[479].hdg = 35;

	simu_track_table[480].lat = 465183278;
	simu_track_table[480].lon = 65667504;
	simu_track_table[480].alt = 447;
	simu_track_table[480].relative_alt = -47;
	simu_track_table[480].vx = -141;
	simu_track_table[480].vy = -101;
	simu_track_table[480].vz = 5;
	simu_track_table[480].hdg = 36;

	simu_track_table[481].lat = 465183246;
	simu_track_table[481].lon = 65667466;
	simu_track_table[481].alt = 447;
	simu_track_table[481].relative_alt = -47;
	simu_track_table[481].vx = -118;
	simu_track_table[481].vy = -110;
	simu_track_table[481].vz = 56;
	simu_track_table[481].hdg = 43;

	simu_track_table[482].lat = 465183223;
	simu_track_table[482].lon = 65667423;
	simu_track_table[482].alt = 447;
	simu_track_table[482].relative_alt = -47;
	simu_track_table[482].vx = -70;
	simu_track_table[482].vy = -148;
	simu_track_table[482].vz = 77;
	simu_track_table[482].hdg = 65;

	simu_track_table[483].lat = 465183212;
	simu_track_table[483].lon = 65667360;
	simu_track_table[483].alt = 447;
	simu_track_table[483].relative_alt = -47;
	simu_track_table[483].vx = -10;
	simu_track_table[483].vy = -215;
	simu_track_table[483].vz = 92;
	simu_track_table[483].hdg = 87;

	simu_track_table[484].lat = 465183215;
	simu_track_table[484].lon = 65667277;
	simu_track_table[484].alt = 447;
	simu_track_table[484].relative_alt = -47;
	simu_track_table[484].vx = 34;
	simu_track_table[484].vy = -284;
	simu_track_table[484].vz = 99;
	simu_track_table[484].hdg = 97;

	simu_track_table[485].lat = 465183225;
	simu_track_table[485].lon = 65667169;
	simu_track_table[485].alt = 446;
	simu_track_table[485].relative_alt = -46;
	simu_track_table[485].vx = 47;
	simu_track_table[485].vy = -333;
	simu_track_table[485].vz = 96;
	simu_track_table[485].hdg = 98;

	simu_track_table[486].lat = 465183234;
	simu_track_table[486].lon = 65667053;
	simu_track_table[486].alt = 446;
	simu_track_table[486].relative_alt = -46;
	simu_track_table[486].vx = 32;
	simu_track_table[486].vy = -352;
	simu_track_table[486].vz = 88;
	simu_track_table[486].hdg = 95;

	simu_track_table[487].lat = 465183239;
	simu_track_table[487].lon = 65666930;
	simu_track_table[487].alt = 446;
	simu_track_table[487].relative_alt = -46;
	simu_track_table[487].vx = 2;
	simu_track_table[487].vy = -343;
	simu_track_table[487].vz = 81;
	simu_track_table[487].hdg = 90;

	simu_track_table[488].lat = 465183237;
	simu_track_table[488].lon = 65666817;
	simu_track_table[488].alt = 446;
	simu_track_table[488].relative_alt = -46;
	simu_track_table[488].vx = -26;
	simu_track_table[488].vy = -320;
	simu_track_table[488].vz = 77;
	simu_track_table[488].hdg = 85;

	simu_track_table[489].lat = 465183230;
	simu_track_table[489].lon = 65666707;
	simu_track_table[489].alt = 446;
	simu_track_table[489].relative_alt = -46;
	simu_track_table[489].vx = -43;
	simu_track_table[489].vy = -296;
	simu_track_table[489].vz = 74;
	simu_track_table[489].hdg = 82;

	simu_track_table[490].lat = 465183221;
	simu_track_table[490].lon = 65666607;
	simu_track_table[490].alt = 445;
	simu_track_table[490].relative_alt = -45;
	simu_track_table[490].vx = -46;
	simu_track_table[490].vy = -282;
	simu_track_table[490].vz = 74;
	simu_track_table[490].hdg = 81;

	simu_track_table[491].lat = 465183212;
	simu_track_table[491].lon = 65666506;
	simu_track_table[491].alt = 445;
	simu_track_table[491].relative_alt = -45;
	simu_track_table[491].vx = -38;
	simu_track_table[491].vy = -279;
	simu_track_table[491].vz = 75;
	simu_track_table[491].hdg = 82;

	simu_track_table[492].lat = 465183206;
	simu_track_table[492].lon = 65666409;
	simu_track_table[492].alt = 445;
	simu_track_table[492].relative_alt = -45;
	simu_track_table[492].vx = -26;
	simu_track_table[492].vy = -286;
	simu_track_table[492].vz = 77;
	simu_track_table[492].hdg = 85;

	simu_track_table[493].lat = 465183202;
	simu_track_table[493].lon = 65666305;
	simu_track_table[493].alt = 445;
	simu_track_table[493].relative_alt = -45;
	simu_track_table[493].vx = -16;
	simu_track_table[493].vy = -296;
	simu_track_table[493].vz = 78;
	simu_track_table[493].hdg = 87;

	simu_track_table[494].lat = 465183200;
	simu_track_table[494].lon = 65666202;
	simu_track_table[494].alt = 445;
	simu_track_table[494].relative_alt = -45;
	simu_track_table[494].vx = -12;
	simu_track_table[494].vy = -304;
	simu_track_table[494].vz = 77;
	simu_track_table[494].hdg = 88;

	simu_track_table[495].lat = 465183198;
	simu_track_table[495].lon = 65666093;
	simu_track_table[495].alt = 444;
	simu_track_table[495].relative_alt = -44;
	simu_track_table[495].vx = -14;
	simu_track_table[495].vy = -306;
	simu_track_table[495].vz = 76;
	simu_track_table[495].hdg = 87;

	simu_track_table[496].lat = 465183195;
	simu_track_table[496].lon = 65665989;
	simu_track_table[496].alt = 444;
	simu_track_table[496].relative_alt = -44;
	simu_track_table[496].vx = -20;
	simu_track_table[496].vy = -304;
	simu_track_table[496].vz = 75;
	simu_track_table[496].hdg = 86;

	simu_track_table[497].lat = 465183191;
	simu_track_table[497].lon = 65665882;
	simu_track_table[497].alt = 444;
	simu_track_table[497].relative_alt = -44;
	simu_track_table[497].vx = -26;
	simu_track_table[497].vy = -299;
	simu_track_table[497].vz = 74;
	simu_track_table[497].hdg = 85;

	simu_track_table[498].lat = 465183186;
	simu_track_table[498].lon = 65665780;
	simu_track_table[498].alt = 444;
	simu_track_table[498].relative_alt = -44;
	simu_track_table[498].vx = -31;
	simu_track_table[498].vy = -294;
	simu_track_table[498].vz = 73;
	simu_track_table[498].hdg = 84;

	simu_track_table[499].lat = 465183179;
	simu_track_table[499].lon = 65665676;
	simu_track_table[499].alt = 444;
	simu_track_table[499].relative_alt = -44;
	simu_track_table[499].vx = -34;
	simu_track_table[499].vy = -290;
	simu_track_table[499].vz = 73;
	simu_track_table[499].hdg = 83;

	simu_track_table[500].lat = 465183173;
	simu_track_table[500].lon = 65665577;
	simu_track_table[500].alt = 444;
	simu_track_table[500].relative_alt = -44;
	simu_track_table[500].vx = -33;
	simu_track_table[500].vy = -290;
	simu_track_table[500].vz = 73;
	simu_track_table[500].hdg = 83;

	simu_track_table[501].lat = 465183167;
	simu_track_table[501].lon = 65665474;
	simu_track_table[501].alt = 443;
	simu_track_table[501].relative_alt = -43;
	simu_track_table[501].vx = -31;
	simu_track_table[501].vy = -291;
	simu_track_table[501].vz = 72;
	simu_track_table[501].hdg = 84;

	simu_track_table[502].lat = 465183161;
	simu_track_table[502].lon = 65665375;
	simu_track_table[502].alt = 443;
	simu_track_table[502].relative_alt = -43;
	simu_track_table[502].vx = -29;
	simu_track_table[502].vy = -293;
	simu_track_table[502].vz = 72;
	simu_track_table[502].hdg = 84;

	simu_track_table[503].lat = 465183156;
	simu_track_table[503].lon = 65665271;
	simu_track_table[503].alt = 443;
	simu_track_table[503].relative_alt = -43;
	simu_track_table[503].vx = -27;
	simu_track_table[503].vy = -295;
	simu_track_table[503].vz = 72;
	simu_track_table[503].hdg = 85;

	simu_track_table[504].lat = 465183151;
	simu_track_table[504].lon = 65665170;
	simu_track_table[504].alt = 443;
	simu_track_table[504].relative_alt = -43;
	simu_track_table[504].vx = -27;
	simu_track_table[504].vy = -296;
	simu_track_table[504].vz = 71;
	simu_track_table[504].hdg = 85;

	simu_track_table[505].lat = 465183147;
	simu_track_table[505].lon = 65665066;
	simu_track_table[505].alt = 443;
	simu_track_table[505].relative_alt = -43;
	simu_track_table[505].vx = -28;
	simu_track_table[505].vy = -296;
	simu_track_table[505].vz = 70;
	simu_track_table[505].hdg = 85;

	simu_track_table[506].lat = 465183142;
	simu_track_table[506].lon = 65664966;
	simu_track_table[506].alt = 443;
	simu_track_table[506].relative_alt = -43;
	simu_track_table[506].vx = -30;
	simu_track_table[506].vy = -295;
	simu_track_table[506].vz = 69;
	simu_track_table[506].hdg = 84;

	simu_track_table[507].lat = 465183136;
	simu_track_table[507].lon = 65664863;
	simu_track_table[507].alt = 442;
	simu_track_table[507].relative_alt = -42;
	simu_track_table[507].vx = -31;
	simu_track_table[507].vy = -294;
	simu_track_table[507].vz = 68;
	simu_track_table[507].hdg = 84;

	simu_track_table[508].lat = 465183131;
	simu_track_table[508].lon = 65664763;
	simu_track_table[508].alt = 442;
	simu_track_table[508].relative_alt = -42;
	simu_track_table[508].vx = -32;
	simu_track_table[508].vy = -293;
	simu_track_table[508].vz = 68;
	simu_track_table[508].hdg = 84;

	simu_track_table[509].lat = 465183125;
	simu_track_table[509].lon = 65664661;
	simu_track_table[509].alt = 442;
	simu_track_table[509].relative_alt = -42;
	simu_track_table[509].vx = -32;
	simu_track_table[509].vy = -293;
	simu_track_table[509].vz = 67;
	simu_track_table[509].hdg = 84;

	simu_track_table[510].lat = 465183119;
	simu_track_table[510].lon = 65664561;
	simu_track_table[510].alt = 442;
	simu_track_table[510].relative_alt = -42;
	simu_track_table[510].vx = -32;
	simu_track_table[510].vy = -294;
	simu_track_table[510].vz = 66;
	simu_track_table[510].hdg = 84;

	simu_track_table[511].lat = 465183113;
	simu_track_table[511].lon = 65664458;
	simu_track_table[511].alt = 442;
	simu_track_table[511].relative_alt = -42;
	simu_track_table[511].vx = -32;
	simu_track_table[511].vy = -294;
	simu_track_table[511].vz = 65;
	simu_track_table[511].hdg = 84;

	simu_track_table[512].lat = 465183108;
	simu_track_table[512].lon = 65664359;
	simu_track_table[512].alt = 442;
	simu_track_table[512].relative_alt = -42;
	simu_track_table[512].vx = -32;
	simu_track_table[512].vy = -295;
	simu_track_table[512].vz = 64;
	simu_track_table[512].hdg = 84;

	simu_track_table[513].lat = 465183102;
	simu_track_table[513].lon = 65664256;
	simu_track_table[513].alt = 441;
	simu_track_table[513].relative_alt = -41;
	simu_track_table[513].vx = -33;
	simu_track_table[513].vy = -295;
	simu_track_table[513].vz = 63;
	simu_track_table[513].hdg = 84;

	simu_track_table[514].lat = 465183097;
	simu_track_table[514].lon = 65664157;
	simu_track_table[514].alt = 441;
	simu_track_table[514].relative_alt = -41;
	simu_track_table[514].vx = -33;
	simu_track_table[514].vy = -295;
	simu_track_table[514].vz = 61;
	simu_track_table[514].hdg = 84;

	simu_track_table[515].lat = 465183091;
	simu_track_table[515].lon = 65664054;
	simu_track_table[515].alt = 441;
	simu_track_table[515].relative_alt = -41;
	simu_track_table[515].vx = -34;
	simu_track_table[515].vy = -295;
	simu_track_table[515].vz = 60;
	simu_track_table[515].hdg = 83;

	simu_track_table[516].lat = 465183085;
	simu_track_table[516].lon = 65663955;
	simu_track_table[516].alt = 441;
	simu_track_table[516].relative_alt = -41;
	simu_track_table[516].vx = -35;
	simu_track_table[516].vy = -295;
	simu_track_table[516].vz = 58;
	simu_track_table[516].hdg = 83;

	simu_track_table[517].lat = 465183079;
	simu_track_table[517].lon = 65663852;
	simu_track_table[517].alt = 441;
	simu_track_table[517].relative_alt = -41;
	simu_track_table[517].vx = -36;
	simu_track_table[517].vy = -295;
	simu_track_table[517].vz = 57;
	simu_track_table[517].hdg = 83;

	simu_track_table[518].lat = 465183072;
	simu_track_table[518].lon = 65663753;
	simu_track_table[518].alt = 441;
	simu_track_table[518].relative_alt = -41;
	simu_track_table[518].vx = -37;
	simu_track_table[518].vy = -296;
	simu_track_table[518].vz = 55;
	simu_track_table[518].hdg = 83;

	simu_track_table[519].lat = 465183066;
	simu_track_table[519].lon = 65663652;
	simu_track_table[519].alt = 440;
	simu_track_table[519].relative_alt = -40;
	simu_track_table[519].vx = -38;
	simu_track_table[519].vy = -296;
	simu_track_table[519].vz = 52;
	simu_track_table[519].hdg = 83;

	simu_track_table[520].lat = 465183059;
	simu_track_table[520].lon = 65663551;
	simu_track_table[520].alt = 440;
	simu_track_table[520].relative_alt = -40;
	simu_track_table[520].vx = -39;
	simu_track_table[520].vy = -296;
	simu_track_table[520].vz = 50;
	simu_track_table[520].hdg = 82;

	simu_track_table[521].lat = 465183052;
	simu_track_table[521].lon = 65663451;
	simu_track_table[521].alt = 440;
	simu_track_table[521].relative_alt = -40;
	simu_track_table[521].vx = -40;
	simu_track_table[521].vy = -295;
	simu_track_table[521].vz = 52;
	simu_track_table[521].hdg = 82;

	simu_track_table[522].lat = 465183045;
	simu_track_table[522].lon = 65663351;
	simu_track_table[522].alt = 440;
	simu_track_table[522].relative_alt = -40;
	simu_track_table[522].vx = -33;
	simu_track_table[522].vy = -294;
	simu_track_table[522].vz = 74;
	simu_track_table[522].hdg = 84;

	simu_track_table[523].lat = 465183043;
	simu_track_table[523].lon = 65663252;
	simu_track_table[523].alt = 440;
	simu_track_table[523].relative_alt = -40;
	simu_track_table[523].vx = 0;
	simu_track_table[523].vy = -289;
	simu_track_table[523].vz = 85;
	simu_track_table[523].hdg = 90;

	simu_track_table[524].lat = 465183051;
	simu_track_table[524].lon = 65663156;
	simu_track_table[524].alt = 440;
	simu_track_table[524].relative_alt = -40;
	simu_track_table[524].vx = 56;
	simu_track_table[524].vy = -280;
	simu_track_table[524].vz = 92;
	simu_track_table[524].hdg = 101;

	simu_track_table[525].lat = 465183073;
	simu_track_table[525].lon = 65663063;
	simu_track_table[525].alt = 439;
	simu_track_table[525].relative_alt = -39;
	simu_track_table[525].vx = 112;
	simu_track_table[525].vy = -270;
	simu_track_table[525].vz = 96;
	simu_track_table[525].hdg = 113;

	simu_track_table[526].lat = 465183105;
	simu_track_table[526].lon = 65662973;
	simu_track_table[526].alt = 439;
	simu_track_table[526].relative_alt = -39;
	simu_track_table[526].vx = 149;
	simu_track_table[526].vy = -262;
	simu_track_table[526].vz = 95;
	simu_track_table[526].hdg = 120;

	simu_track_table[527].lat = 465183144;
	simu_track_table[527].lon = 65662885;
	simu_track_table[527].alt = 439;
	simu_track_table[527].relative_alt = -39;
	simu_track_table[527].vx = 158;
	simu_track_table[527].vy = -261;
	simu_track_table[527].vz = 93;
	simu_track_table[527].hdg = 121;

	simu_track_table[528].lat = 465183182;
	simu_track_table[528].lon = 65662796;
	simu_track_table[528].alt = 439;
	simu_track_table[528].relative_alt = -39;
	simu_track_table[528].vx = 143;
	simu_track_table[528].vy = -266;
	simu_track_table[528].vz = 92;
	simu_track_table[528].hdg = 118;

	simu_track_table[529].lat = 465183216;
	simu_track_table[529].lon = 65662706;
	simu_track_table[529].alt = 438;
	simu_track_table[529].relative_alt = -38;
	simu_track_table[529].vx = 115;
	simu_track_table[529].vy = -273;
	simu_track_table[529].vz = 91;
	simu_track_table[529].hdg = 113;

	simu_track_table[530].lat = 465183244;
	simu_track_table[530].lon = 65662613;
	simu_track_table[530].alt = 438;
	simu_track_table[530].relative_alt = -38;
	simu_track_table[530].vx = 88;
	simu_track_table[530].vy = -279;
	simu_track_table[530].vz = 90;
	simu_track_table[530].hdg = 108;

	simu_track_table[531].lat = 465183266;
	simu_track_table[531].lon = 65662519;
	simu_track_table[531].alt = 438;
	simu_track_table[531].relative_alt = -38;
	simu_track_table[531].vx = 71;
	simu_track_table[531].vy = -282;
	simu_track_table[531].vz = 90;
	simu_track_table[531].hdg = 104;

	simu_track_table[532].lat = 465183286;
	simu_track_table[532].lon = 65662425;
	simu_track_table[532].alt = 438;
	simu_track_table[532].relative_alt = -38;
	simu_track_table[532].vx = 66;
	simu_track_table[532].vy = -283;
	simu_track_table[532].vz = 90;
	simu_track_table[532].hdg = 103;

	simu_track_table[533].lat = 465183306;
	simu_track_table[533].lon = 65662331;
	simu_track_table[533].alt = 438;
	simu_track_table[533].relative_alt = -38;
	simu_track_table[533].vx = 72;
	simu_track_table[533].vy = -281;
	simu_track_table[533].vz = 90;
	simu_track_table[533].hdg = 104;

	simu_track_table[534].lat = 465183327;
	simu_track_table[534].lon = 65662237;
	simu_track_table[534].alt = 437;
	simu_track_table[534].relative_alt = -37;
	simu_track_table[534].vx = 83;
	simu_track_table[534].vy = -280;
	simu_track_table[534].vz = 90;
	simu_track_table[534].hdg = 106;

	simu_track_table[535].lat = 465183351;
	simu_track_table[535].lon = 65662144;
	simu_track_table[535].alt = 437;
	simu_track_table[535].relative_alt = -37;
	simu_track_table[535].vx = 91;
	simu_track_table[535].vy = -278;
	simu_track_table[535].vz = 89;
	simu_track_table[535].hdg = 108;

	simu_track_table[536].lat = 465183377;
	simu_track_table[536].lon = 65662051;
	simu_track_table[536].alt = 437;
	simu_track_table[536].relative_alt = -37;
	simu_track_table[536].vx = 95;
	simu_track_table[536].vy = -278;
	simu_track_table[536].vz = 89;
	simu_track_table[536].hdg = 109;

	simu_track_table[537].lat = 465183402;
	simu_track_table[537].lon = 65661959;
	simu_track_table[537].alt = 437;
	simu_track_table[537].relative_alt = -37;
	simu_track_table[537].vx = 93;
	simu_track_table[537].vy = -278;
	simu_track_table[537].vz = 88;
	simu_track_table[537].hdg = 109;

	simu_track_table[538].lat = 465183427;
	simu_track_table[538].lon = 65661866;
	simu_track_table[538].alt = 436;
	simu_track_table[538].relative_alt = -36;
	simu_track_table[538].vx = 87;
	simu_track_table[538].vy = -279;
	simu_track_table[538].vz = 88;
	simu_track_table[538].hdg = 107;

	simu_track_table[539].lat = 465183450;
	simu_track_table[539].lon = 65661773;
	simu_track_table[539].alt = 436;
	simu_track_table[539].relative_alt = -36;
	simu_track_table[539].vx = 80;
	simu_track_table[539].vy = -281;
	simu_track_table[539].vz = 88;
	simu_track_table[539].hdg = 106;

	simu_track_table[540].lat = 465183472;
	simu_track_table[540].lon = 65661680;
	simu_track_table[540].alt = 436;
	simu_track_table[540].relative_alt = -36;
	simu_track_table[540].vx = 75;
	simu_track_table[540].vy = -282;
	simu_track_table[540].vz = 87;
	simu_track_table[540].hdg = 105;

	simu_track_table[541].lat = 465183493;
	simu_track_table[541].lon = 65661587;
	simu_track_table[541].alt = 436;
	simu_track_table[541].relative_alt = -36;
	simu_track_table[541].vx = 72;
	simu_track_table[541].vy = -283;
	simu_track_table[541].vz = 87;
	simu_track_table[541].hdg = 104;

	simu_track_table[542].lat = 465183513;
	simu_track_table[542].lon = 65661494;
	simu_track_table[542].alt = 436;
	simu_track_table[542].relative_alt = -36;
	simu_track_table[542].vx = 72;
	simu_track_table[542].vy = -283;
	simu_track_table[542].vz = 87;
	simu_track_table[542].hdg = 104;

	simu_track_table[543].lat = 465183533;
	simu_track_table[543].lon = 65661400;
	simu_track_table[543].alt = 435;
	simu_track_table[543].relative_alt = -35;
	simu_track_table[543].vx = 74;
	simu_track_table[543].vy = -282;
	simu_track_table[543].vz = 86;
	simu_track_table[543].hdg = 105;

	simu_track_table[544].lat = 465183554;
	simu_track_table[544].lon = 65661308;
	simu_track_table[544].alt = 435;
	simu_track_table[544].relative_alt = -35;
	simu_track_table[544].vx = 77;
	simu_track_table[544].vy = -282;
	simu_track_table[544].vz = 86;
	simu_track_table[544].hdg = 105;

	simu_track_table[545].lat = 465183576;
	simu_track_table[545].lon = 65661214;
	simu_track_table[545].alt = 435;
	simu_track_table[545].relative_alt = -35;
	simu_track_table[545].vx = 78;
	simu_track_table[545].vy = -282;
	simu_track_table[545].vz = 85;
	simu_track_table[545].hdg = 105;

	simu_track_table[546].lat = 465183597;
	simu_track_table[546].lon = 65661122;
	simu_track_table[546].alt = 435;
	simu_track_table[546].relative_alt = -35;
	simu_track_table[546].vx = 78;
	simu_track_table[546].vy = -282;
	simu_track_table[546].vz = 85;
	simu_track_table[546].hdg = 105;

	simu_track_table[547].lat = 465183619;
	simu_track_table[547].lon = 65661029;
	simu_track_table[547].alt = 435;
	simu_track_table[547].relative_alt = -35;
	simu_track_table[547].vx = 77;
	simu_track_table[547].vy = -283;
	simu_track_table[547].vz = 84;
	simu_track_table[547].hdg = 105;

	simu_track_table[548].lat = 465183640;
	simu_track_table[548].lon = 65660936;
	simu_track_table[548].alt = 434;
	simu_track_table[548].relative_alt = -34;
	simu_track_table[548].vx = 75;
	simu_track_table[548].vy = -283;
	simu_track_table[548].vz = 84;
	simu_track_table[548].hdg = 105;

	simu_track_table[549].lat = 465183660;
	simu_track_table[549].lon = 65660844;
	simu_track_table[549].alt = 434;
	simu_track_table[549].relative_alt = -34;
	simu_track_table[549].vx = 73;
	simu_track_table[549].vy = -284;
	simu_track_table[549].vz = 83;
	simu_track_table[549].hdg = 105;

	simu_track_table[550].lat = 465183681;
	simu_track_table[550].lon = 65660751;
	simu_track_table[550].alt = 434;
	simu_track_table[550].relative_alt = -34;
	simu_track_table[550].vx = 72;
	simu_track_table[550].vy = -284;
	simu_track_table[550].vz = 83;
	simu_track_table[550].hdg = 104;

	simu_track_table[551].lat = 465183701;
	simu_track_table[551].lon = 65660658;
	simu_track_table[551].alt = 434;
	simu_track_table[551].relative_alt = -34;
	simu_track_table[551].vx = 71;
	simu_track_table[551].vy = -284;
	simu_track_table[551].vz = 82;
	simu_track_table[551].hdg = 104;

	simu_track_table[552].lat = 465183720;
	simu_track_table[552].lon = 65660565;
	simu_track_table[552].alt = 433;
	simu_track_table[552].relative_alt = -33;
	simu_track_table[552].vx = 71;
	simu_track_table[552].vy = -285;
	simu_track_table[552].vz = 82;
	simu_track_table[552].hdg = 104;

	simu_track_table[553].lat = 465183740;
	simu_track_table[553].lon = 65660472;
	simu_track_table[553].alt = 433;
	simu_track_table[553].relative_alt = -33;
	simu_track_table[553].vx = 71;
	simu_track_table[553].vy = -285;
	simu_track_table[553].vz = 81;
	simu_track_table[553].hdg = 104;

	simu_track_table[554].lat = 465183760;
	simu_track_table[554].lon = 65660380;
	simu_track_table[554].alt = 433;
	simu_track_table[554].relative_alt = -33;
	simu_track_table[554].vx = 71;
	simu_track_table[554].vy = -285;
	simu_track_table[554].vz = 80;
	simu_track_table[554].hdg = 104;

	simu_track_table[555].lat = 465183780;
	simu_track_table[555].lon = 65660287;
	simu_track_table[555].alt = 433;
	simu_track_table[555].relative_alt = -33;
	simu_track_table[555].vx = 71;
	simu_track_table[555].vy = -286;
	simu_track_table[555].vz = 79;
	simu_track_table[555].hdg = 104;

	simu_track_table[556].lat = 465183799;
	simu_track_table[556].lon = 65660194;
	simu_track_table[556].alt = 433;
	simu_track_table[556].relative_alt = -33;
	simu_track_table[556].vx = 70;
	simu_track_table[556].vy = -286;
	simu_track_table[556].vz = 79;
	simu_track_table[556].hdg = 104;

	simu_track_table[557].lat = 465183819;
	simu_track_table[557].lon = 65660101;
	simu_track_table[557].alt = 432;
	simu_track_table[557].relative_alt = -32;
	simu_track_table[557].vx = 69;
	simu_track_table[557].vy = -286;
	simu_track_table[557].vz = 78;
	simu_track_table[557].hdg = 104;

	simu_track_table[558].lat = 465183838;
	simu_track_table[558].lon = 65660009;
	simu_track_table[558].alt = 432;
	simu_track_table[558].relative_alt = -32;
	simu_track_table[558].vx = 68;
	simu_track_table[558].vy = -287;
	simu_track_table[558].vz = 77;
	simu_track_table[558].hdg = 103;

	simu_track_table[559].lat = 465183856;
	simu_track_table[559].lon = 65659916;
	simu_track_table[559].alt = 432;
	simu_track_table[559].relative_alt = -32;
	simu_track_table[559].vx = 67;
	simu_track_table[559].vy = -287;
	simu_track_table[559].vz = 76;
	simu_track_table[559].hdg = 103;

	simu_track_table[560].lat = 465183875;
	simu_track_table[560].lon = 65659823;
	simu_track_table[560].alt = 432;
	simu_track_table[560].relative_alt = -32;
	simu_track_table[560].vx = 66;
	simu_track_table[560].vy = -288;
	simu_track_table[560].vz = 75;
	simu_track_table[560].hdg = 103;

	simu_track_table[561].lat = 465183893;
	simu_track_table[561].lon = 65659730;
	simu_track_table[561].alt = 432;
	simu_track_table[561].relative_alt = -32;
	simu_track_table[561].vx = 65;
	simu_track_table[561].vy = -288;
	simu_track_table[561].vz = 74;
	simu_track_table[561].hdg = 103;

	simu_track_table[562].lat = 465183911;
	simu_track_table[562].lon = 65659637;
	simu_track_table[562].alt = 432;
	simu_track_table[562].relative_alt = -32;
	simu_track_table[562].vx = 65;
	simu_track_table[562].vy = -289;
	simu_track_table[562].vz = 72;
	simu_track_table[562].hdg = 103;

	simu_track_table[563].lat = 465183929;
	simu_track_table[563].lon = 65659543;
	simu_track_table[563].alt = 431;
	simu_track_table[563].relative_alt = -31;
	simu_track_table[563].vx = 64;
	simu_track_table[563].vy = -289;
	simu_track_table[563].vz = 71;
	simu_track_table[563].hdg = 102;

	simu_track_table[564].lat = 465183947;
	simu_track_table[564].lon = 65659450;
	simu_track_table[564].alt = 431;
	simu_track_table[564].relative_alt = -31;
	simu_track_table[564].vx = 63;
	simu_track_table[564].vy = -290;
	simu_track_table[564].vz = 69;
	simu_track_table[564].hdg = 102;

	simu_track_table[565].lat = 465183964;
	simu_track_table[565].lon = 65659357;
	simu_track_table[565].alt = 431;
	simu_track_table[565].relative_alt = -31;
	simu_track_table[565].vx = 61;
	simu_track_table[565].vy = -290;
	simu_track_table[565].vz = 67;
	simu_track_table[565].hdg = 102;

	simu_track_table[566].lat = 465183981;
	simu_track_table[566].lon = 65659264;
	simu_track_table[566].alt = 431;
	simu_track_table[566].relative_alt = -31;
	simu_track_table[566].vx = 60;
	simu_track_table[566].vy = -291;
	simu_track_table[566].vz = 65;
	simu_track_table[566].hdg = 102;

	simu_track_table[567].lat = 465183997;
	simu_track_table[567].lon = 65659170;
	simu_track_table[567].alt = 431;
	simu_track_table[567].relative_alt = -31;
	simu_track_table[567].vx = 59;
	simu_track_table[567].vy = -292;
	simu_track_table[567].vz = 63;
	simu_track_table[567].hdg = 101;

	simu_track_table[568].lat = 465184014;
	simu_track_table[568].lon = 65659076;
	simu_track_table[568].alt = 431;
	simu_track_table[568].relative_alt = -31;
	simu_track_table[568].vx = 57;
	simu_track_table[568].vy = -292;
	simu_track_table[568].vz = 60;
	simu_track_table[568].hdg = 101;

	simu_track_table[569].lat = 465184029;
	simu_track_table[569].lon = 65658983;
	simu_track_table[569].alt = 430;
	simu_track_table[569].relative_alt = -30;
	simu_track_table[569].vx = 57;
	simu_track_table[569].vy = -288;
	simu_track_table[569].vz = 46;
	simu_track_table[569].hdg = 101;

	simu_track_table[570].lat = 465184046;
	simu_track_table[570].lon = 65658896;
	simu_track_table[570].alt = 430;
	simu_track_table[570].relative_alt = -30;
	simu_track_table[570].vx = 65;
	simu_track_table[570].vy = -247;
	simu_track_table[570].vz = 36;
	simu_track_table[570].hdg = 105;

	simu_track_table[571].lat = 465184065;
	simu_track_table[571].lon = 65658830;
	simu_track_table[571].alt = 430;
	simu_track_table[571].relative_alt = -30;
	simu_track_table[571].vx = 82;
	simu_track_table[571].vy = -158;
	simu_track_table[571].vz = 36;
	simu_track_table[571].hdg = 117;

	simu_track_table[572].lat = 465184089;
	simu_track_table[572].lon = 65658799;
	simu_track_table[572].alt = 430;
	simu_track_table[572].relative_alt = -30;
	simu_track_table[572].vx = 106;
	simu_track_table[572].vy = -47;
	simu_track_table[572].vz = 42;
	simu_track_table[572].hdg = 156;

	simu_track_table[573].lat = 465184121;
	simu_track_table[573].lon = 65658803;
	simu_track_table[573].alt = 430;
	simu_track_table[573].relative_alt = -30;
	simu_track_table[573].vx = 145;
	simu_track_table[573].vy = 43;
	simu_track_table[573].vz = 48;
	simu_track_table[573].hdg = 197;

	simu_track_table[574].lat = 465184163;
	simu_track_table[574].lon = 65658831;
	simu_track_table[574].alt = 430;
	simu_track_table[574].relative_alt = -30;
	simu_track_table[574].vx = 206;
	simu_track_table[574].vy = 86;
	simu_track_table[574].vz = 50;
	simu_track_table[574].hdg = 203;

	simu_track_table[575].lat = 465184221;
	simu_track_table[575].lon = 65658865;
	simu_track_table[575].alt = 430;
	simu_track_table[575].relative_alt = -30;
	simu_track_table[575].vx = 276;
	simu_track_table[575].vy = 71;
	simu_track_table[575].vz = 51;
	simu_track_table[575].hdg = 194;

	simu_track_table[576].lat = 465184294;
	simu_track_table[576].lon = 65658889;
	simu_track_table[576].alt = 430;
	simu_track_table[576].relative_alt = -30;
	simu_track_table[576].vx = 330;
	simu_track_table[576].vy = 21;
	simu_track_table[576].vz = 49;
	simu_track_table[576].hdg = 184;

	simu_track_table[577].lat = 465184376;
	simu_track_table[577].lon = 65658895;
	simu_track_table[577].alt = 429;
	simu_track_table[577].relative_alt = -29;
	simu_track_table[577].vx = 351;
	simu_track_table[577].vy = -32;
	simu_track_table[577].vz = 42;
	simu_track_table[577].hdg = 175;

	simu_track_table[578].lat = 465184460;
	simu_track_table[578].lon = 65658887;
	simu_track_table[578].alt = 429;
	simu_track_table[578].relative_alt = -29;
	simu_track_table[578].vx = 342;
	simu_track_table[578].vy = -66;
	simu_track_table[578].vz = 33;
	simu_track_table[578].hdg = 169;

	simu_track_table[579].lat = 465184541;
	simu_track_table[579].lon = 65658872;
	simu_track_table[579].alt = 429;
	simu_track_table[579].relative_alt = -29;
	simu_track_table[579].vx = 319;
	simu_track_table[579].vy = -77;
	simu_track_table[579].vz = 26;
	simu_track_table[579].hdg = 166;

	simu_track_table[580].lat = 465184617;
	simu_track_table[580].lon = 65658855;
	simu_track_table[580].alt = 429;
	simu_track_table[580].relative_alt = -29;
	simu_track_table[580].vx = 298;
	simu_track_table[580].vy = -70;
	simu_track_table[580].vz = 22;
	simu_track_table[580].hdg = 167;

	simu_track_table[581].lat = 465184689;
	simu_track_table[581].lon = 65658842;
	simu_track_table[581].alt = 429;
	simu_track_table[581].relative_alt = -29;
	simu_track_table[581].vx = 287;
	simu_track_table[581].vy = -57;
	simu_track_table[581].vz = 22;
	simu_track_table[581].hdg = 169;

	simu_track_table[582].lat = 465184759;
	simu_track_table[582].lon = 65658833;
	simu_track_table[582].alt = 429;
	simu_track_table[582].relative_alt = -29;
	simu_track_table[582].vx = 287;
	simu_track_table[582].vy = -47;
	simu_track_table[582].vz = 23;
	simu_track_table[582].hdg = 171;

	simu_track_table[583].lat = 465184830;
	simu_track_table[583].lon = 65658825;
	simu_track_table[583].alt = 429;
	simu_track_table[583].relative_alt = -29;
	simu_track_table[583].vx = 293;
	simu_track_table[583].vy = -43;
	simu_track_table[583].vz = 24;
	simu_track_table[583].hdg = 172;

	simu_track_table[584].lat = 465184903;
	simu_track_table[584].lon = 65658818;
	simu_track_table[584].alt = 429;
	simu_track_table[584].relative_alt = -29;
	simu_track_table[584].vx = 300;
	simu_track_table[584].vy = -46;
	simu_track_table[584].vz = 24;
	simu_track_table[584].hdg = 171;

	simu_track_table[585].lat = 465184976;
	simu_track_table[585].lon = 65658810;
	simu_track_table[585].alt = 429;
	simu_track_table[585].relative_alt = -29;
	simu_track_table[585].vx = 303;
	simu_track_table[585].vy = -53;
	simu_track_table[585].vz = 23;
	simu_track_table[585].hdg = 170;

	simu_track_table[586].lat = 465185050;
	simu_track_table[586].lon = 65658799;
	simu_track_table[586].alt = 429;
	simu_track_table[586].relative_alt = -29;
	simu_track_table[586].vx = 303;
	simu_track_table[586].vy = -60;
	simu_track_table[586].vz = 21;
	simu_track_table[586].hdg = 169;

	simu_track_table[587].lat = 465185123;
	simu_track_table[587].lon = 65658785;
	simu_track_table[587].alt = 429;
	simu_track_table[587].relative_alt = -29;
	simu_track_table[587].vx = 299;
	simu_track_table[587].vy = -64;
	simu_track_table[587].vz = 19;
	simu_track_table[587].hdg = 168;

	simu_track_table[588].lat = 465185194;
	simu_track_table[588].lon = 65658771;
	simu_track_table[588].alt = 429;
	simu_track_table[588].relative_alt = -29;
	simu_track_table[588].vx = 295;
	simu_track_table[588].vy = -66;
	simu_track_table[588].vz = 17;
	simu_track_table[588].hdg = 167;

	simu_track_table[589].lat = 465185265;
	simu_track_table[589].lon = 65658757;
	simu_track_table[589].alt = 429;
	simu_track_table[589].relative_alt = -29;
	simu_track_table[589].vx = 292;
	simu_track_table[589].vy = -66;
	simu_track_table[589].vz = 16;
	simu_track_table[589].hdg = 167;

	simu_track_table[590].lat = 465185335;
	simu_track_table[590].lon = 65658743;
	simu_track_table[590].alt = 429;
	simu_track_table[590].relative_alt = -29;
	simu_track_table[590].vx = 290;
	simu_track_table[590].vy = -65;
	simu_track_table[590].vz = 15;
	simu_track_table[590].hdg = 167;

	simu_track_table[591].lat = 465185405;
	simu_track_table[591].lon = 65658729;
	simu_track_table[591].alt = 429;
	simu_track_table[591].relative_alt = -29;
	simu_track_table[591].vx = 290;
	simu_track_table[591].vy = -64;
	simu_track_table[591].vz = 14;
	simu_track_table[591].hdg = 168;

	simu_track_table[592].lat = 465185474;
	simu_track_table[592].lon = 65658715;
	simu_track_table[592].alt = 429;
	simu_track_table[592].relative_alt = -29;
	simu_track_table[592].vx = 291;
	simu_track_table[592].vy = -64;
	simu_track_table[592].vz = 13;
	simu_track_table[592].hdg = 168;

	simu_track_table[593].lat = 465185543;
	simu_track_table[593].lon = 65658701;
	simu_track_table[593].alt = 429;
	simu_track_table[593].relative_alt = -29;
	simu_track_table[593].vx = 291;
	simu_track_table[593].vy = -65;
	simu_track_table[593].vz = 12;
	simu_track_table[593].hdg = 167;

	simu_track_table[594].lat = 465185612;
	simu_track_table[594].lon = 65658686;
	simu_track_table[594].alt = 429;
	simu_track_table[594].relative_alt = -29;
	simu_track_table[594].vx = 291;
	simu_track_table[594].vy = -66;
	simu_track_table[594].vz = 10;
	simu_track_table[594].hdg = 167;

	simu_track_table[595].lat = 465185681;
	simu_track_table[595].lon = 65658671;
	simu_track_table[595].alt = 429;
	simu_track_table[595].relative_alt = -29;
	simu_track_table[595].vx = 290;
	simu_track_table[595].vy = -68;
	simu_track_table[595].vz = 8;
	simu_track_table[595].hdg = 167;

	simu_track_table[596].lat = 465185749;
	simu_track_table[596].lon = 65658655;
	simu_track_table[596].alt = 429;
	simu_track_table[596].relative_alt = -29;
	simu_track_table[596].vx = 289;
	simu_track_table[596].vy = -69;
	simu_track_table[596].vz = 6;
	simu_track_table[596].hdg = 167;

	simu_track_table[597].lat = 465185817;
	simu_track_table[597].lon = 65658639;
	simu_track_table[597].alt = 429;
	simu_track_table[597].relative_alt = -29;
	simu_track_table[597].vx = 288;
	simu_track_table[597].vy = -70;
	simu_track_table[597].vz = 4;
	simu_track_table[597].hdg = 166;

	simu_track_table[598].lat = 465185885;
	simu_track_table[598].lon = 65658623;
	simu_track_table[598].alt = 429;
	simu_track_table[598].relative_alt = -29;
	simu_track_table[598].vx = 287;
	simu_track_table[598].vy = -71;
	simu_track_table[598].vz = 2;
	simu_track_table[598].hdg = 166;

	simu_track_table[599].lat = 465185952;
	simu_track_table[599].lon = 65658606;
	simu_track_table[599].alt = 429;
	simu_track_table[599].relative_alt = -29;
	simu_track_table[599].vx = 286;
	simu_track_table[599].vy = -72;
	simu_track_table[599].vz = 0;
	simu_track_table[599].hdg = 166;

	simu_track_table[600].lat = 465186019;
	simu_track_table[600].lon = 65658589;
	simu_track_table[600].alt = 429;
	simu_track_table[600].relative_alt = -29;
	simu_track_table[600].vx = 285;
	simu_track_table[600].vy = -73;
	simu_track_table[600].vz = -2;
	simu_track_table[600].hdg = 166;

	simu_track_table[601].lat = 465186085;
	simu_track_table[601].lon = 65658571;
	simu_track_table[601].alt = 429;
	simu_track_table[601].relative_alt = -29;
	simu_track_table[601].vx = 284;
	simu_track_table[601].vy = -75;
	simu_track_table[601].vz = -5;
	simu_track_table[601].hdg = 165;

	simu_track_table[602].lat = 465186151;
	simu_track_table[602].lon = 65658553;
	simu_track_table[602].alt = 429;
	simu_track_table[602].relative_alt = -29;
	simu_track_table[602].vx = 284;
	simu_track_table[602].vy = -76;
	simu_track_table[602].vz = -8;
	simu_track_table[602].hdg = 165;

	simu_track_table[603].lat = 465186217;
	simu_track_table[603].lon = 65658534;
	simu_track_table[603].alt = 429;
	simu_track_table[603].relative_alt = -29;
	simu_track_table[603].vx = 282;
	simu_track_table[603].vy = -78;
	simu_track_table[603].vz = -11;
	simu_track_table[603].hdg = 165;

	simu_track_table[604].lat = 465186282;
	simu_track_table[604].lon = 65658515;
	simu_track_table[604].alt = 429;
	simu_track_table[604].relative_alt = -29;
	simu_track_table[604].vx = 281;
	simu_track_table[604].vy = -79;
	simu_track_table[604].vz = -14;
	simu_track_table[604].hdg = 164;

	simu_track_table[605].lat = 465186347;
	simu_track_table[605].lon = 65658495;
	simu_track_table[605].alt = 429;
	simu_track_table[605].relative_alt = -29;
	simu_track_table[605].vx = 280;
	simu_track_table[605].vy = -81;
	simu_track_table[605].vz = -18;
	simu_track_table[605].hdg = 164;

	simu_track_table[606].lat = 465186411;
	simu_track_table[606].lon = 65658474;
	simu_track_table[606].alt = 429;
	simu_track_table[606].relative_alt = -29;
	simu_track_table[606].vx = 278;
	simu_track_table[606].vy = -83;
	simu_track_table[606].vz = -23;
	simu_track_table[606].hdg = 163;

	simu_track_table[607].lat = 465186474;
	simu_track_table[607].lon = 65658453;
	simu_track_table[607].alt = 429;
	simu_track_table[607].relative_alt = -29;
	simu_track_table[607].vx = 274;
	simu_track_table[607].vy = -84;
	simu_track_table[607].vz = -11;
	simu_track_table[607].hdg = 163;

	simu_track_table[608].lat = 465186535;
	simu_track_table[608].lon = 65658432;
	simu_track_table[608].alt = 429;
	simu_track_table[608].relative_alt = -29;
	simu_track_table[608].vx = 250;
	simu_track_table[608].vy = -78;
	simu_track_table[608].vz = 2;
	simu_track_table[608].hdg = 163;

	simu_track_table[609].lat = 465186586;
	simu_track_table[609].lon = 65658415;
	simu_track_table[609].alt = 429;
	simu_track_table[609].relative_alt = -29;
	simu_track_table[609].vx = 186;
	simu_track_table[609].vy = -60;
	simu_track_table[609].vz = 8;
	simu_track_table[609].hdg = 162;

	simu_track_table[610].lat = 465186618;
	simu_track_table[610].lon = 65658405;
	simu_track_table[610].alt = 429;
	simu_track_table[610].relative_alt = -29;
	simu_track_table[610].vx = 98;
	simu_track_table[610].vy = -34;
	simu_track_table[610].vz = 15;
	simu_track_table[610].hdg = 161;

	simu_track_table[611].lat = 465186631;
	simu_track_table[611].lon = 65658405;
	simu_track_table[611].alt = 429;
	simu_track_table[611].relative_alt = -29;
	simu_track_table[611].vx = 23;
	simu_track_table[611].vy = -9;
	simu_track_table[611].vz = 18;
	simu_track_table[611].hdg = 159;

	simu_track_table[612].lat = 465186629;
	simu_track_table[612].lon = 65658411;
	simu_track_table[612].alt = 429;
	simu_track_table[612].relative_alt = -29;
	simu_track_table[612].vx = -18;
	simu_track_table[612].vy = 6;
	simu_track_table[612].vz = 15;
	simu_track_table[612].hdg = 341;

	simu_track_table[613].lat = 465186622;
	simu_track_table[613].lon = 65658420;
	simu_track_table[613].alt = 429;
	simu_track_table[613].relative_alt = -29;
	simu_track_table[613].vx = -20;
	simu_track_table[613].vy = 9;
	simu_track_table[613].vz = 12;
	simu_track_table[613].hdg = 337;

	simu_track_table[614].lat = 465186617;
	simu_track_table[614].lon = 65658429;
	simu_track_table[614].alt = 429;
	simu_track_table[614].relative_alt = -29;
	simu_track_table[614].vx = 1;
	simu_track_table[614].vy = 3;
	simu_track_table[614].vz = 12;
	simu_track_table[614].hdg = 242;

	simu_track_table[615].lat = 465186616;
	simu_track_table[615].lon = 65658436;
	simu_track_table[615].alt = 429;
	simu_track_table[615].relative_alt = -29;
	simu_track_table[615].vx = 28;
	simu_track_table[615].vy = -7;
	simu_track_table[615].vz = 12;
	simu_track_table[615].hdg = 166;

	simu_track_table[616].lat = 465186620;
	simu_track_table[616].lon = 65658439;
	simu_track_table[616].alt = 429;
	simu_track_table[616].relative_alt = -29;
	simu_track_table[616].vx = 46;
	simu_track_table[616].vy = -16;
	simu_track_table[616].vz = 12;
	simu_track_table[616].hdg = 161;

	simu_track_table[617].lat = 465186627;
	simu_track_table[617].lon = 65658440;
	simu_track_table[617].alt = 429;
	simu_track_table[617].relative_alt = -29;
	simu_track_table[617].vx = 52;
	simu_track_table[617].vy = -20;
	simu_track_table[617].vz = 11;
	simu_track_table[617].hdg = 159;

	simu_track_table[618].lat = 465186634;
	simu_track_table[618].lon = 65658440;
	simu_track_table[618].alt = 428;
	simu_track_table[618].relative_alt = -28;
	simu_track_table[618].vx = 47;
	simu_track_table[618].vy = -22;
	simu_track_table[618].vz = 10;
	simu_track_table[618].hdg = 155;

	simu_track_table[619].lat = 465186640;
	simu_track_table[619].lon = 65658440;
	simu_track_table[619].alt = 428;
	simu_track_table[619].relative_alt = -28;
	simu_track_table[619].vx = 39;
	simu_track_table[619].vy = -21;
	simu_track_table[619].vz = 10;
	simu_track_table[619].hdg = 152;

	simu_track_table[620].lat = 465186643;
	simu_track_table[620].lon = 65658439;
	simu_track_table[620].alt = 428;
	simu_track_table[620].relative_alt = -28;
	simu_track_table[620].vx = 31;
	simu_track_table[620].vy = -20;
	simu_track_table[620].vz = 9;
	simu_track_table[620].hdg = 148;

	simu_track_table[621].lat = 465186645;
	simu_track_table[621].lon = 65658440;
	simu_track_table[621].alt = 428;
	simu_track_table[621].relative_alt = -28;
	simu_track_table[621].vx = 29;
	simu_track_table[621].vy = -19;
	simu_track_table[621].vz = 9;
	simu_track_table[621].hdg = 146;

	simu_track_table[622].lat = 465186647;
	simu_track_table[622].lon = 65658439;
	simu_track_table[622].alt = 428;
	simu_track_table[622].relative_alt = -28;
	simu_track_table[622].vx = 30;
	simu_track_table[622].vy = -20;
	simu_track_table[622].vz = 8;
	simu_track_table[622].hdg = 146;

	simu_track_table[623].lat = 465186649;
	simu_track_table[623].lon = 65658439;
	simu_track_table[623].alt = 428;
	simu_track_table[623].relative_alt = -28;
	simu_track_table[623].vx = 34;
	simu_track_table[623].vy = -21;
	simu_track_table[623].vz = 8;
	simu_track_table[623].hdg = 148;

	simu_track_table[624].lat = 465186652;
	simu_track_table[624].lon = 65658438;
	simu_track_table[624].alt = 428;
	simu_track_table[624].relative_alt = -28;
	simu_track_table[624].vx = 37;
	simu_track_table[624].vy = -23;
	simu_track_table[624].vz = 8;
	simu_track_table[624].hdg = 148;

	simu_track_table[625].lat = 465186655;
	simu_track_table[625].lon = 65658436;
	simu_track_table[625].alt = 428;
	simu_track_table[625].relative_alt = -28;
	simu_track_table[625].vx = 39;
	simu_track_table[625].vy = -24;
	simu_track_table[625].vz = 8;
	simu_track_table[625].hdg = 149;

	simu_track_table[626].lat = 465186659;
	simu_track_table[626].lon = 65658434;
	simu_track_table[626].alt = 428;
	simu_track_table[626].relative_alt = -28;
	simu_track_table[626].vx = 40;
	simu_track_table[626].vy = -24;
	simu_track_table[626].vz = 7;
	simu_track_table[626].hdg = 149;

	simu_track_table[627].lat = 465186663;
	simu_track_table[627].lon = 65658431;
	simu_track_table[627].alt = 428;
	simu_track_table[627].relative_alt = -28;
	simu_track_table[627].vx = 39;
	simu_track_table[627].vy = -24;
	simu_track_table[627].vz = 7;
	simu_track_table[627].hdg = 148;

	simu_track_table[628].lat = 465186666;
	simu_track_table[628].lon = 65658429;
	simu_track_table[628].alt = 428;
	simu_track_table[628].relative_alt = -28;
	simu_track_table[628].vx = 37;
	simu_track_table[628].vy = -24;
	simu_track_table[628].vz = 7;
	simu_track_table[628].hdg = 147;

	simu_track_table[629].lat = 465186668;
	simu_track_table[629].lon = 65658427;
	simu_track_table[629].alt = 428;
	simu_track_table[629].relative_alt = -28;
	simu_track_table[629].vx = 36;
	simu_track_table[629].vy = -23;
	simu_track_table[629].vz = 7;
	simu_track_table[629].hdg = 147;

	simu_track_table[630].lat = 465186671;
	simu_track_table[630].lon = 65658425;
	simu_track_table[630].alt = 428;
	simu_track_table[630].relative_alt = -28;
	simu_track_table[630].vx = 35;
	simu_track_table[630].vy = -23;
	simu_track_table[630].vz = 6;
	simu_track_table[630].hdg = 147;

	simu_track_table[631].lat = 465186673;
	simu_track_table[631].lon = 65658422;
	simu_track_table[631].alt = 428;
	simu_track_table[631].relative_alt = -28;
	simu_track_table[631].vx = 34;
	simu_track_table[631].vy = -23;
	simu_track_table[631].vz = 6;
	simu_track_table[631].hdg = 147;

	simu_track_table[632].lat = 465186675;
	simu_track_table[632].lon = 65658420;
	simu_track_table[632].alt = 428;
	simu_track_table[632].relative_alt = -28;
	simu_track_table[632].vx = 34;
	simu_track_table[632].vy = -22;
	simu_track_table[632].vz = 6;
	simu_track_table[632].hdg = 147;

	simu_track_table[633].lat = 465186677;
	simu_track_table[633].lon = 65658418;
	simu_track_table[633].alt = 428;
	simu_track_table[633].relative_alt = -28;
	simu_track_table[633].vx = 34;
	simu_track_table[633].vy = -22;
	simu_track_table[633].vz = 6;
	simu_track_table[633].hdg = 147;

	simu_track_table[634].lat = 465186679;
	simu_track_table[634].lon = 65658416;
	simu_track_table[634].alt = 428;
	simu_track_table[634].relative_alt = -28;
	simu_track_table[634].vx = 34;
	simu_track_table[634].vy = -22;
	simu_track_table[634].vz = 5;
	simu_track_table[634].hdg = 147;

	simu_track_table[635].lat = 465186680;
	simu_track_table[635].lon = 65658413;
	simu_track_table[635].alt = 428;
	simu_track_table[635].relative_alt = -28;
	simu_track_table[635].vx = 34;
	simu_track_table[635].vy = -22;
	simu_track_table[635].vz = 5;
	simu_track_table[635].hdg = 148;

	simu_track_table[636].lat = 465186682;
	simu_track_table[636].lon = 65658411;
	simu_track_table[636].alt = 428;
	simu_track_table[636].relative_alt = -28;
	simu_track_table[636].vx = 34;
	simu_track_table[636].vy = -21;
	simu_track_table[636].vz = 5;
	simu_track_table[636].hdg = 148;

	simu_track_table[637].lat = 465186683;
	simu_track_table[637].lon = 65658409;
	simu_track_table[637].alt = 428;
	simu_track_table[637].relative_alt = -28;
	simu_track_table[637].vx = 33;
	simu_track_table[637].vy = -21;
	simu_track_table[637].vz = 5;
	simu_track_table[637].hdg = 148;

	simu_track_table[638].lat = 465186685;
	simu_track_table[638].lon = 65658406;
	simu_track_table[638].alt = 428;
	simu_track_table[638].relative_alt = -28;
	simu_track_table[638].vx = 33;
	simu_track_table[638].vy = -20;
	simu_track_table[638].vz = 5;
	simu_track_table[638].hdg = 148;

	simu_track_table[639].lat = 465186686;
	simu_track_table[639].lon = 65658404;
	simu_track_table[639].alt = 428;
	simu_track_table[639].relative_alt = -28;
	simu_track_table[639].vx = 33;
	simu_track_table[639].vy = -20;
	simu_track_table[639].vz = 4;
	simu_track_table[639].hdg = 148;

	simu_track_table[640].lat = 465186687;
	simu_track_table[640].lon = 65658402;
	simu_track_table[640].alt = 428;
	simu_track_table[640].relative_alt = -28;
	simu_track_table[640].vx = 32;
	simu_track_table[640].vy = -20;
	simu_track_table[640].vz = 4;
	simu_track_table[640].hdg = 149;

	simu_track_table[641].lat = 465186687;
	simu_track_table[641].lon = 65658400;
	simu_track_table[641].alt = 428;
	simu_track_table[641].relative_alt = -28;
	simu_track_table[641].vx = 32;
	simu_track_table[641].vy = -19;
	simu_track_table[641].vz = 4;
	simu_track_table[641].hdg = 149;

	simu_track_table[642].lat = 465186688;
	simu_track_table[642].lon = 65658398;
	simu_track_table[642].alt = 428;
	simu_track_table[642].relative_alt = -28;
	simu_track_table[642].vx = 32;
	simu_track_table[642].vy = -19;
	simu_track_table[642].vz = 4;
	simu_track_table[642].hdg = 149;

	simu_track_table[643].lat = 465186689;
	simu_track_table[643].lon = 65658395;
	simu_track_table[643].alt = 428;
	simu_track_table[643].relative_alt = -28;
	simu_track_table[643].vx = 32;
	simu_track_table[643].vy = -19;
	simu_track_table[643].vz = 4;
	simu_track_table[643].hdg = 150;
}
*/
#endif
