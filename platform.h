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

