void bgu_int_close(int fd);
int bgu_int_open();
int bgu_reset(int fd);
int bgu_write_zero(int fd);
int bgu_get_status(uint16_t *inbuff,uint16_t *zero_count,int fd);
int start_bgu(gradprog_t *ingradprog,int fd);
int end_bgu_thread();
