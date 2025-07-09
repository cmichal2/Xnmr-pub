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

#ifdef MINGW

#define PATH_SEP '\\'
#define DPATH_SEP "\\"
#define HOMEP "HOMEPATH"
#define SYS_PROG_PATH "C:\\Xnmr\\prog\\"
#define COPY_COMM "copy %s %s"
#define ICON_PATH "C:\\Xnmr\\xnmr_buff_icon.png"

#else

#define PATH_SEP '/'
#define DPATH_SEP "/"
#define HOMEP "HOME"
#define SYS_PROG_PATH "/usr/share/Xnmr/prog/"
#define COPY_COMM "cp -p %s %s"
#define ICON_PATH "/usr/share/Xnmr/xnmr_buff_icon.png"

#endif

