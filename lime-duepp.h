void deinit_limes();
int init_limes();
int open_limes();
void close_limes();
int prep_lime(int npts,int64_t *buffer);
int start_lime();
int sync_lime();
void join_limes();
void kill_gains();
int wait_for_lime_data();
int wait_till_streams_done();
// hong long is our table of sin/cos?
#define TRIGLEN 65536
