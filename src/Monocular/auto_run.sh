#!/bin/bash

# ================================================================================================================
# Скрипт для автозапуска проекта
# ================================================================================================================

CONTAINER_NAME=orb_slam3_container
CURRENT_DIR=$(pwd)
BASH_DIR=$(where bash | head -n 1)

SLAM_CMD='
export DISPLAY=host.docker.internal:0 &&
./Examples/Monocular/camera_carla_transfer_extended \
./Vocabulary/ORBvoc.txt \
./Examples/Monocular/carla_60.yaml \
tcp://host.docker.internal:5555 \
tcp://host.docker.internal:5557
'

show_help() {
  echo "Использование: $0 [ОПЦИИ]"
  echo
  echo "  -h, --help                Показать эту справку и выйти."
  echo
  echo "=== Команда: record ==="
  echo "Собирает монокулярный датасет из симулятора городской среды CARLA Simulator."
  echo "Работает только без команды test."
  echo "Требует запущенного симулятора и установленного Python API для него."
  
  echo "Опции:"
  echo "  --output <СТРОКА>         Директория.              Обязательный."

  echo "  --duration <INT>          Длительность в секундах. Обязательный."

  echo "  -s, --seed <INT>          Заданный сид, если нужно детерминированное поведение."
  echo "                            По-умолчанию - текущее время через команду time.time()."

  echo "  --filterv <СТРОКА>        Модель транспорта. Узнать доступные модели можно в документации CARLA Simulator."
  echo "                            По-умолчанию: \"vehicle.dodge.charger\"."

  echo "  --host <СТРОКА>           IP-адрес CARLA сервера."
  echo "                            Тип: строковый."
  echo "                            По-умолчанию: \"127.0.0.1\"."

  echo "  --rport <INT>              Порт CARLA сервера."
  echo "                            По-умолчанию: 2000."

  echo "  --tm-port <INT>           Порт Traffic Manager. Подробнее про Traffic Manager можно узнать в документации CARLA Simulator."
  echo "                            По-умолчанию: 8000."

  echo "  --preview <ЛОГИЧЕСКИЙ>    Включить окно предпросмотра из камеры."
  echo "                            По-умолчанию: True."

  echo "  --rspeed <FLOAT>           Скорость движения в процентах. 100% - нормальная, 50% - половина."
  echo "                            По-умолчанию: 100.0."
  echo
  echo "=== Команда: test ==="
  echo "Последовательно передаёт собранный ранее набор данных в ORB-SLAM3 через порт zmq."
  echo "Работает только без команды record."
  echo "Опции:"
  echo "  --dataset <СТРОКА>        Директория.              Обязательный."

  echo "  --tport <INT>              Порт для ZMQ PUB сокета."
  echo "                            По-умолчанию: 5555."

  echo "  --tspeed <FLOAT>           Множитель скорости воспроизведения."
  echo "                            По-умолчанию: 1.0."

  echo "  --loop <ЛОГИЧЕСКИЙ>       Зациклить воспроизведение."
  echo "                            По-умолчанию: True."
  exit 0
}

if [ -f "wt_receiver.pid" ]; then
    rm "wt_receiver.pid"
fi

if [ -f "wt_sender.pid" ]; then
    rm "wt_sender.pid"
fi

if [ -f "wt_orb_slam3.pid" ]; then
    rm "wt_orb_slam3.pid"
fi

wt $BASH_DIR -c "
docker exec -i \"$CONTAINER_NAME\" bash -c '$SLAM_CMD' & exit" &
sleep 1
PID=$(docker exec orb_slam3_container ps aux | grep camera_carla_transfer_extended | awk '{print $2}')
sleep 1

echo "Запущен ORB-SLAM3, PID: $PID, ожидание 7 секунд..."
sleep 7

wt $BASH_DIR -c "cd \"$CURRENT_DIR\" && echo \$BASHPID > wt_receiver.pid \
&& ./receive_data.py --data-port 5557 & exit" &
sleep 1
PPID_RECEIVER=$(cat "wt_receiver.pid")
PID_RECEIVER=$(ps aux | awk -v ppid=$PPID_RECEIVER '$2 == ppid && $0 ~ /python/ { print $1 }')
rm "wt_receiver.pid"

wt $BASH_DIR -c "cd \"$CURRENT_DIR\" && echo \$BASHPID > wt_sender.pid \
&& ./test_monocular_dataset.py --dataset carla_dataset_long_60fps/ & exit" &
sleep 1
PPID_SENDER=$(cat "wt_sender.pid")
PID_SENDER=$(ps aux | awk -v ppid=$PPID_SENDER '$2 == ppid && $0 ~ /python/ { print $1 }')
rm "wt_sender.pid"

sleep 30

echo "Отправка SIGINT процессу ORB-SLAM3..."
docker exec orb_slam3_container kill -2 "$PID"
echo "Процесс ORB-SLAM3 завершён."

echo "Завершение работы терминалов..."
kill -2 "$PID_RECEIVER"
kill -2 "$PID_SENDER"

echo "Процесс завершён."








#!/bin/bash

show_help() {
  echo "Использование: $0 [ОПЦИИ]"
  echo
  echo "Опции:"
  echo "  -f FILE        Путь к файлу"
  echo "  -v             Включить подробный вывод (verbose)"
  echo "  -h, --help     Показать эту справку и выйти"
  echo
  exit 0
}

# Значения по умолчанию
file=""
verbose=false

# Парсинг аргументов
while [[ $# -gt 0 ]]; do
  case "$1" in
    -f)
      file="$2"
      shift 2
      ;;
    -v)
      verbose=true
      shift
      ;;
    -h|--help)
      show_help
      ;;
    *)
      echo "Неизвестный аргумент: $1"
      echo "Используй --help для справки."
      exit 1
      ;;
  esac
done

# Вывод, если verbose
if [ "$verbose" = true ]; then
  echo "Выбран файл: $file"
fi

# Пример действия
if [ -n "$file" ]; then
  echo "Обработка файла: $file"
  # Здесь можно добавить логику
else
  echo "Файл не указан. Используй -f <FILE> или --help"
  exit 1
fi
