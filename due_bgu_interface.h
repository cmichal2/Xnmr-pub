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

void bgu_int_close(int fd);
int bgu_int_open();
int bgu_reset(int fd);
int bgu_write_zero(int fd);
int bgu_get_status(uint16_t *inbuff,uint16_t *zero_count,int fd);
int start_bgu(gradprog_t *ingradprog,int fd);
int end_bgu_thread();
