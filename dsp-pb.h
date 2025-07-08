#define STANDARD_MODE 0
#define NOISY_MODE 1

int setup_dsp(int sw, int p,double freq,int dgain, double dsp_ph, char force_setup,double rcvr_clk);
int read_fifo(int npts,int *data,int mode,int receiver_model);
void dsp_close_port();
int dsp_request_data();
void dsp_reset_fifo();
