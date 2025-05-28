# Инструкция по настройке bluetooth соединения между Raspberry Pi 5 и Android смартфоном с передачей координат из ORB-SLAM3

- 
```Shell
sudo raspi-config
```
- Переход в `Interfacing Options` -> `Serial Port`.
- "Would you like a login shell to be accessible over serial?" - **No**.
- "Would you like the serial port hardware to be enabled?" - **Yes**.
- 
```Shell
sudo reboot
```
- 
```Shell
mkdir gps_simulator
cd gps_simulator
touch gps_simulator.c
code .
```
- Установка C/C++ Extension Pack
- Установка Error Lens
- Установка Clang-Format
- 
```Shell
touch .clang-format
```
- Запись в .clang-format:
```
---
BasedOnStyle: Google
```
- Ctrl + Shift + P > Format Document > Выбор расширения Clang-Format
- Вставка кода в gps_simulator.c:
```C
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

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

    double latitude = 55.7558;
    char lat_dir = 'N';
    double longitude = 37.6173;
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
```
- Установка valgrind
```shell
-sudo apt install valgrind
```
- Проверка кода:
```Shell
cppcheck --enable=all --suppress=missingIncludeSystem gps_simulator.c
Ctrl+C
```
- создание файла Makefile
- Код в Makefile:
```Makefile
all: gps_simulator

gps_simulator:
	gcc -Wall -Werror -Wextra -std=c11 -o gps_simulator gps_simulator.c

clean:
	rm gps_simulator

rebuild: clean all
```
- Сборка:
```Shell
make
```
- Проверка исполняемого файла:
```Shell
valgrind --tool=memcheck --leak-check=full --track-origins=yes -s ./gps_simulator.c
Ctrl+C
```
- Запуск файла:
```Shell
./gps_simulator
```
- Установка пакетов для передачи данных Bluetooth:
```Shell
sudo apt-get install bluetooth bluez bluez-tools
```
- Настройка rfcomm.conf:
```Shell
sudo vim /etc/bluetooth/rfcomm.conf
```

```
rfcomm0 {
    bind yes;
    device <MAC-адрес>;
    channel 1;
    comment "Serial Port";
}
```
- Перезапуск служб Bluetooth:
```Shell
sudo systemctl restart bluetooth
```

```Shell
sudo bluetoothctl
```
- Внутри интерактивной оболочки:
```Shell
scan on
pair <MAC-адрес телефона>
trust <MAC-адрес телефона>
quit
```
- Привязка телефона:
```Shell
sudo rfcomm bind 0 D0:97:FE:5A:3A:6C 1
```
- Остановка gps_simulator, смена порта в строке 66 на "/dev/rfcomm0", сборка и перезапуск с правами суперпользователя:
```Shell
make rebuild
sudo ./gps_simulator
```
- Остановка приложения
- 
```Shell
sudo apt-get install minicom -y
sudo vim /etc/systemd/system/dbus-org.bluez.service
```
- В конце строки с “ExecStart” добавление флага ‘-C’.
```
ExecStart=/usr/lib/bluetooth/bluetoothd -C
```
- Добавление новой строки:
```
ExecStartPost=/usr/bin/sdptool add SP
```
- 
```Shell
sudo systemctl daemon-reload
sudo systemctl restart bluetooth.service
sudo rfcomm watch /dev/rfcomm0 1
```
- Установка полной версии приложения Bluetooth GPS на телефон.
- Выбор raspberrypi, "Connect"
- Параметры "Для разработчиков" > Меню "Отладка" > Выбрать приложение для фиктивных местоположений > Bluetooth GPS
- В приложении Bluetooth GPS "Enable Mock GPS Provider"
- 
```Shell
sudo ./gps_simulator
```
- Работает только в Google maps с неточным местоположением
- Просмотр портов rfcomm:
```Shell
sudo rfcomm
```
- Очистка портов rfcomm:
```Shell
sudo rfcomm release 0
```
