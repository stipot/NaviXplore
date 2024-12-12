CFLAGS = -I/
LDFLAGS = -L/ -lrcl -lrclc -lgeometry-msgs

all: gps_simulator

gps_simulator:
	gcc -Wall -Werror -Wextra -std=c11 -o gps_simulator gps_simulator_vdummy.c

clean:
	rm gps_simulator

rebuild: clean all