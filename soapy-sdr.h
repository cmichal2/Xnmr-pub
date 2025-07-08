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
