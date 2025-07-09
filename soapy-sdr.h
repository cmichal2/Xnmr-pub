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

int open_sdr();
int prep_sdr(int npts,int64_t *buffer, int first);
int start_sdr_threads();

void deinit_sdrs();
void close_sdrs();
void join_sdrs();
void kill_gains();
int wait_for_sdr_data();
int wait_till_streams_done();
void generate_rx_sdr_events(int first_time);
int check_sync_was_found();
// hong long is our table of sin/cos?
#define TRIGLEN 65536
