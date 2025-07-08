CC = gcc
CFLAGS = -O2 -Wall -I../
LFLAGS = `gtk-config --libs` 

HDEPALL = ../h_config.h ../p_signals.h ../param_utils.h ../pulse.h ../shm_data.h ../shm_prog.h
ODEPALL = -lxnmr

all: pulse_test timel timel2 bopo single_p excercise one_pulse freq_test


clean:
	rm -f *.o pulse_test core timel timel2 single_p bopo one_pulse
neat:
	rm -f *.o

pulse_test.o: pulse_test.c $(HDEPALL) 
	$(CC) $(CFLAGS) -c pulse_test.c
pulse_test: pulse_test.o 
	$(CC) $(CFLAGS) $(LFLAGS) -o pulse_test pulse_test.o $(ODEPALL) 

single_p.o: single_p.c $(HDEPALL)
	$(CC) $(CFLAGS) -c single_p.c
single_p: single_p.o 
	$(CC) $(CFLAGS) $(LFLAGS) -o single_p single_p.o $(ODEPALL)

timel.o: timel.c $(HDEPALL)
	$(CC) $(CFLAGS) -c timel.c
timel: timel.o 
	$(CC) $(CFLAGS) $(LFLAGS) -o timel timel.o $(ODEPALL)

timel2.o: timel2.c $(HDEPALL)
	$(CC) $(CFLAGS) -c timel2.c
timel2: timel2.o 
	$(CC) $(CFLAGS) $(LFLAGS) -o timel2 timel2.o $(ODEPALL)

bopo.o: bopo.c $(HDEPALL)
	$(CC) $(CFLAGS) -c bopo.c
bopo: bopo.o
	$(CC) $(CFLAGS) $(LFLAGS) -o bopo bopo.o $(ODEPALL)

excercies.o: excercise.c $(HDEPALL)
	$(CC) $(CFLAGS) -c bopo.c
excercise: excercise.o 
	$(CC) $(CFLAGS) $(LFLAGS) -o excercise excercise.o  $(ODEPALL)

one_pulse.o: one_pulse.c $(HDEPALL)
	$(CC) $(CFLAGS) -c one_pulse.c
one_pulse: one_pulse.o 
	$(CC) $(CFLAGS) $(LFLAGS) -o one_pulse one_pulse.o $(ODEPALL)

freq_test.o: freq_test.c $(HDEPALL)
	$(CC) $(CFLAGS)  -c freq_test.c
freq_test: freq_test.o 
	$(CC) $(CFLAGS) $(LFLAGS) -o freq_test freq_test.o $(ODEPALL)




