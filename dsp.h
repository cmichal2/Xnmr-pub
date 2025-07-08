#define STANDARD_MODE 0
#define NOISY_MODE 1

int setup_dsp(int sw, int p,double freq,int dgain, double dsp_ph, char force_setup);
int read_fifo(int npts,int *data,int mode);
void dsp_close_port();
