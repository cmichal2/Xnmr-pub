/*

Copyright (c) 2025 Carl Michal with contributions of:
Scott Nelson
Phil Eles
Matt Grinder

This file is part of Xnmr.

Xnmr is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free
Software Foundation, either version 3 of the License, or (at your
option) any later version.

Xnmr is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
for more details.

You should have received a copy of the GNU General Public License
along with Xnmr. If not, see <https://www.gnu.org/licenses/>.

*/

#define STANDARD_MODE 0
#define NOISY_MODE 1

int setup_dsp(int sw, int p,double freq,int dgain, double dsp_ph, char force_setup,double rcvr_clk);
int read_fifo(int npts,int *data,int mode,int receiver_model);
void dsp_close_port();
int dsp_request_data();
void dsp_reset_fifo();
