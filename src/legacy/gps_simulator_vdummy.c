#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define EARTH_RADIUS 6371e3;

unsigned char calculate_checksum(const char *sentence) {
  unsigned char checksum = 0;
  for (const char *p = sentence; *p; p++) {
    checksum ^= *p;
  }
  return checksum;
}

void generate_gprmc(char *buffer, double latitude, char lat_dir,
                    double longitude, char lon_dir, double speed,
                    double course) {
  time_t rawtime;
  struct tm *timeinfo;
  char sentence[100];

  time(&rawtime);
  timeinfo = gmtime(&rawtime);

  sprintf(sentence,
          "GPRMC,%02d%02d%02d.000,A,%02d%07.4f,%c,%03d%07.4f,%c,%.1f,%.1f,%02d%"
          "02d%02d,,",
          timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec, (int)latitude,
          (latitude - (int)latitude) * 60.0, lat_dir, (int)longitude,
          (longitude - (int)longitude) * 60.0, lon_dir, speed, course,
          timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year % 100);

  unsigned char checksum = calculate_checksum(sentence);
  sprintf(buffer, "$%s*%02X\r\n", sentence, checksum);
}

int init_serial(const char *portname) {
  int fd = open(portname, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd < 0) {
    perror("Error while opening a serial port");
    return fd;
  }
  struct termios options;
  tcgetattr(fd, &options);

  cfsetispeed(&options, B9600);
  cfsetospeed(&options, B9600);

  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_oflag &= ~OPOST;

  tcsetattr(fd, TCSANOW, &options);
  return fd;
}

int main() {
  const char *portname = "/dev/rfcomm0";
  int fd = init_serial(portname);
  if (fd < 0) {
    return 1;
  }
  ssize_t bytes_written = 0;

  while (1) {
    char nmea_sentence[128];
    double latitude = 55.752593;
    char lat_dir = 'N';
    double longitude = 37.626626;
    char lon_dir = 'E';
    double speed = 0.0;
    double course = 0.0;

    generate_gprmc(nmea_sentence, latitude, lat_dir, longitude, lon_dir, speed,
                   course);
    bytes_written = write(fd, nmea_sentence, strlen(nmea_sentence));
    tcdrain(fd);

    if (bytes_written < 0) {
      if (errno == EAGAIN) {
        printf("Buffer is full\n");
        sleep(10);
        continue;
      } else {
        perror("Failed to write to serial port");
        close(fd);
        return 1;
      }
    } else {
      printf("Sent %s", nmea_sentence);
    }

    sleep(1);
  }
  close(fd);
  return 0;
}
