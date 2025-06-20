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
  echo "Использование: $0 [КОМАНДА] [ОПЦИИ]"
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
  echo "                            По умолчанию - текущее время через команду time.time()."

  echo "  --filterv <СТРОКА>        Модель транспорта. Узнать доступные модели можно в документации CARLA Simulator."
  echo "                            По умолчанию: \"vehicle.dodge.charger\"."

  echo "  --host <СТРОКА>           IP-адрес CARLA сервера."
  echo "                            Тип: строковый."
  echo "                            По умолчанию: \"127.0.0.1\"."

  echo "  --rport <INT>             Порт CARLA сервера."
  echo "                            По умолчанию: 2000."

  echo "  --tm-port <INT>           Порт Traffic Manager. Подробнее про Traffic Manager можно узнать в документации CARLA Simulator."
  echo "                            По умолчанию: 8000."

  echo "  --preview                 Включить окно предпросмотра из камеры."
  echo "                            По умолчанию: True."

  echo "  --rspeed <FLOAT>          Скорость движения в процентах. 100% - нормальная, 50% - половина."
  echo "                            По умолчанию: 100.0."
  echo
  echo "=== Команда: test ==="
  echo "Последовательно передаёт собранный ранее набор данных в ORB-SLAM3 через порт zmq."
  echo "Работает только без команды record."
  echo "Опции:"
  echo "  --dataset <СТРОКА>        Директория.              Обязательный."

  echo "  --tport <INT>             Порт для ZMQ PUB сокета."
  echo "                            По умолчанию: 5555."

  echo "  --tspeed <FLOAT>          Множитель скорости воспроизведения."
  echo "                            По умолчанию: 1.0."

  echo "  --loop                    Зациклить воспроизведение."
  echo "                            По умолчанию: выключено."
  echo
  echo "=== Команда: receive ==="
  echo "Принимает данные от ORB-SLAM3 через порт zmq."
  echo "Опции:"
  echo "  --data-port <INT>         Порт для приема всех данных."
  echo "                            По умолчанию: 5557."

  echo "  --debug                   Включить режим отладки."
  echo "                            По умолчанию: выключено."

  echo "  --debug-output <СТРОКА>   Директория для сохранения отладочных данных."
  echo "                            По умолчанию: \"debug_output\"."

  echo "  --collect-data            Собрать и сохранить данные с ORB-SLAM3."
  echo "                            По умолчанию: выключено."

  echo "  --collection-data-output <СТРОКА>  Директория для сохранения собранных данных."
  echo "                            По умолчанию: \"collected_data\"."

  echo "  --collection-max-frames <INT>  Максимальное количество кадров для сбора."
  echo "                            -1 для сбора всех кадров."
  echo "                            По умолчанию: 100."

  echo "  --collection-interval <FLOAT>  Интервал между собираемыми данными."
  echo "                            0 для сбора всех кадров."
  echo "                            По умолчанию: 1.0."
  exit 0
}

cleanup_pid_files() {
  if [ -f "wt_receiver.pid" ]; then
    rm "wt_receiver.pid"
  fi

  if [ -f "wt_sender.pid" ]; then
    rm "wt_sender.pid"
  fi

  if [ -f "wt_orb_slam3.pid" ]; then
    rm "wt_orb_slam3.pid"
  fi
  
  if [ -f "wt_record.pid" ]; then
    rm "wt_record.pid"
  fi
}

ORBSLAM_PID=""
PID_RECEIVER=""
PID_SENDER=""
PID_RECORD=""

cleanup() {
    echo "Получен сигнал завершения, остановка всех процессов..."
    
    if [ ! -z "$ORBSLAM_PID" ]; then
        echo "Отправка SIGINT процессу ORB-SLAM3..."
        docker exec $CONTAINER_NAME kill -2 "$ORBSLAM_PID" || true
        echo "Процесс ORB-SLAM3 завершён."
    fi
    
    if [ ! -z "$PID_RECEIVER" ]; then
        echo "Завершение работы приемника..."
        kill -2 "$PID_RECEIVER" || true
    fi
    
    if [ ! -z "$PID_SENDER" ]; then
        echo "Завершение работы отправителя..."
        kill -2 "$PID_SENDER" || true
    fi
    
    if [ ! -z "$PID_RECORD" ]; then
        echo "Завершение работы процесса записи..."
        kill -2 "$PID_RECORD" || true
    fi
    
    echo "Все процессы остановлены."
    cleanup_pid_files
    exit 0
}

# Регистрация обработчика сигналов
trap cleanup SIGINT SIGTERM

# Параметры record
DEFAULT_RECORD_FILTERV="vehicle.dodge.charger"
DEFAULT_RECORD_HOST="127.0.0.1"
DEFAULT_RECORD_PORT="2000"
DEFAULT_RECORD_TM_PORT="8000" 
DEFAULT_RECORD_PREVIEW="True"
DEFAULT_RECORD_SPEED="100.0"

# Параметры test
DEFAULT_TEST_PORT="5555"
DEFAULT_TEST_SPEED="1.0"
DEFAULT_TEST_LOOP="False"

# Параметры receive
DEFAULT_RECEIVE_DATA_PORT="5557"
DEFAULT_RECEIVE_DEBUG="False"
DEFAULT_RECEIVE_DEBUG_OUTPUT="debug_output"
DEFAULT_RECEIVE_COLLECT_DATA="False"
DEFAULT_RECEIVE_COLLECTION_DATA_OUTPUT="collected_data"
DEFAULT_RECEIVE_COLLECTION_MAX_FRAMES="100"
DEFAULT_RECEIVE_COLLECTION_INTERVAL="1.0"

declare -A PROVIDED_ARGS

print_record_params() {
  echo "=== Параметры команды record ==="
  echo "  Директория вывода: $RECORD_OUTPUT"
  echo "  Длительность: $RECORD_DURATION секунд"
  
  if [[ "${PROVIDED_ARGS[record_seed]}" == "true" ]]; then
    echo "  Сид: $RECORD_SEED"
  fi
  
  if [[ "${PROVIDED_ARGS[record_filterv]}" == "true" ]]; then
    echo "  Модель транспорта: $RECORD_FILTERV"
  else
    echo "  Модель транспорта: $DEFAULT_RECORD_FILTERV (по умолчанию)"
  fi
  
  if [[ "${PROVIDED_ARGS[record_host]}" == "true" ]]; then
    echo "  IP-адрес сервера: $RECORD_HOST"
  else
    echo "  IP-адрес сервера: $DEFAULT_RECORD_HOST (по умолчанию)"
  fi
  
  if [[ "${PROVIDED_ARGS[record_port]}" == "true" ]]; then
    echo "  Порт сервера: $RECORD_PORT"
  else
    echo "  Порт сервера: $DEFAULT_RECORD_PORT (по умолчанию)"
  fi
  
  if [[ "${PROVIDED_ARGS[record_tm_port]}" == "true" ]]; then
    echo "  Порт Traffic Manager: $RECORD_TM_PORT"
  else
    echo "  Порт Traffic Manager: $DEFAULT_RECORD_TM_PORT (по умолчанию)"
  fi
  
  if [[ "${PROVIDED_ARGS[record_preview]}" == "true" ]]; then
    echo "  Предпросмотр: $RECORD_PREVIEW"
  else
    echo "  Предпросмотр: $DEFAULT_RECORD_PREVIEW (по умолчанию)"
  fi
  
  if [[ "${PROVIDED_ARGS[record_speed]}" == "true" ]]; then
    echo "  Скорость движения: $RECORD_SPEED%"
  else
    echo "  Скорость движения: $DEFAULT_RECORD_SPEED% (по умолчанию)"
  fi
  echo
}

print_test_params() {
  echo "=== Параметры команды test ==="
  echo "  Датасет: $TEST_DATASET"
  
  if [[ "${PROVIDED_ARGS[test_port]}" == "true" ]]; then
    echo "  Порт: $TEST_PORT"
  else
    echo "  Порт: $DEFAULT_TEST_PORT (по умолчанию)"
  fi
  
  if [[ "${PROVIDED_ARGS[test_speed]}" == "true" ]]; then
    echo "  Скорость: $TEST_SPEED"
  else
    echo "  Скорость: $DEFAULT_TEST_SPEED (по умолчанию)"
  fi
  
  if [[ "${PROVIDED_ARGS[test_loop]}" == "true" ]]; then
    echo "  Зацикливание: $TEST_LOOP"
  else
    echo "  Зацикливание: $DEFAULT_TEST_LOOP (по умолчанию)"
  fi
  echo
}

print_receive_params() {
  echo "=== Параметры команды receive ==="
  
  if [[ "${PROVIDED_ARGS[receive_data_port]}" == "true" ]]; then
    echo "  Порт данных: $RECEIVE_DATA_PORT"
  else
    echo "  Порт данных: $DEFAULT_RECEIVE_DATA_PORT (по умолчанию)"
  fi
  
  if [[ "${PROVIDED_ARGS[receive_debug]}" == "true" ]]; then
    echo "  Режим отладки: $RECEIVE_DEBUG"
  else
    echo "  Режим отладки: $DEFAULT_RECEIVE_DEBUG (по умолчанию)"
  fi
  
  if [[ "$RECEIVE_DEBUG" == "true" ]]; then
    if [[ "${PROVIDED_ARGS[receive_debug_output]}" == "true" ]]; then
      echo "  Директория отладки: $RECEIVE_DEBUG_OUTPUT"
    else
      echo "  Директория отладки: $DEFAULT_RECEIVE_DEBUG_OUTPUT (по умолчанию)"
    fi
  fi
  
  if [[ "${PROVIDED_ARGS[receive_collect_data]}" == "true" ]]; then
    echo "  Сбор данных: $RECEIVE_COLLECT_DATA"
  else
    echo "  Сбор данных: $DEFAULT_RECEIVE_COLLECT_DATA (по умолчанию)"
  fi
  
  if [[ "$RECEIVE_COLLECT_DATA" == "true" ]]; then
    if [[ "${PROVIDED_ARGS[receive_collection_data_output]}" == "true" ]]; then
      echo "  Директория сбора: $RECEIVE_COLLECTION_DATA_OUTPUT"
    else
      echo "  Директория сбора: $DEFAULT_RECEIVE_COLLECTION_DATA_OUTPUT (по умолчанию)"
    fi
    
    if [[ "${PROVIDED_ARGS[receive_collection_max_frames]}" == "true" ]]; then
      echo "  Максимум кадров: $RECEIVE_COLLECTION_MAX_FRAMES"
    else
      echo "  Максимум кадров: $DEFAULT_RECEIVE_COLLECTION_MAX_FRAMES (по умолчанию)"
    fi
    
    if [[ "${PROVIDED_ARGS[receive_collection_interval]}" == "true" ]]; then
      echo "  Интервал: $RECEIVE_COLLECTION_INTERVAL"
    else
      echo "  Интервал: $DEFAULT_RECEIVE_COLLECTION_INTERVAL (по умолчанию)"
    fi
  fi
  echo
}

start_orbslam3() {
    wt $BASH_DIR -c "docker exec -i \"$CONTAINER_NAME\" bash -c '$SLAM_CMD' & exit" &
    sleep 1
    ORBSLAM_PID=$(docker exec $CONTAINER_NAME ps aux | grep camera_carla_transfer_extended | awk '{print $2}')
    sleep 1

    echo "Запущен ORB-SLAM3, PID: $ORBSLAM_PID, ожидание 7 секунд..."
    sleep 7
}

start_receiver() {
    local receive_cmd="./receive_data.py"
    
    if [[ "${PROVIDED_ARGS[receive_data_port]}" == "true" ]]; then
      receive_cmd="$receive_cmd --data-port $RECEIVE_DATA_PORT"
    fi
    
    if [[ "${PROVIDED_ARGS[receive_debug]}" == "true" ]]; then
      receive_cmd="$receive_cmd --debug"
    fi
    
    if [[ "${PROVIDED_ARGS[receive_debug_output]}" == "true" ]]; then
      receive_cmd="$receive_cmd --debug-output $RECEIVE_DEBUG_OUTPUT"
    fi
    
    if [[ "${PROVIDED_ARGS[receive_collect_data]}" == "true" ]]; then
      receive_cmd="$receive_cmd --collect-data"
    fi
    
    if [[ "${PROVIDED_ARGS[receive_collection_data_output]}" == "true" ]]; then
      receive_cmd="$receive_cmd --collection-data-output $RECEIVE_COLLECTION_DATA_OUTPUT"
    fi
    
    if [[ "${PROVIDED_ARGS[receive_collection_max_frames]}" == "true" ]]; then
      receive_cmd="$receive_cmd --collection-max-frames $RECEIVE_COLLECTION_MAX_FRAMES"
    fi
    
    if [[ "${PROVIDED_ARGS[receive_collection_interval]}" == "true" ]]; then
      receive_cmd="$receive_cmd --collection-interval $RECEIVE_COLLECTION_INTERVAL"
    fi
    
    wt $BASH_DIR -c "cd \"$CURRENT_DIR\" && echo \$BASHPID > wt_receiver.pid && $receive_cmd & exit" &
    sleep 1
    PPID_RECEIVER=$(cat "wt_receiver.pid")
    PID_RECEIVER=$(ps aux | awk -v ppid=$PPID_RECEIVER '$2 == ppid && $0 ~ /python/ { print $1 }')
    rm "wt_receiver.pid"
    
    echo "Запущен приемник данных, PID: $PID_RECEIVER"
}

start_sender() {
    local dataset=$1
    local test_cmd="./test_monocular_dataset.py --dataset $dataset"
    
    if [[ "${PROVIDED_ARGS[test_port]}" == "true" ]]; then
      test_cmd="$test_cmd --port $TEST_PORT"
    fi
    
    if [[ "${PROVIDED_ARGS[test_speed]}" == "true" ]]; then
      test_cmd="$test_cmd --speed $TEST_SPEED"
    fi
    
    if [[ "${PROVIDED_ARGS[test_loop]}" == "true" ]]; then
      test_cmd="$test_cmd --loop"
    fi
    
    wt $BASH_DIR -c "cd \"$CURRENT_DIR\" && echo \$BASHPID > wt_sender.pid && $test_cmd & exit" &
    sleep 1
    PPID_SENDER=$(cat "wt_sender.pid")
    PID_SENDER=$(ps aux | awk -v ppid=$PPID_SENDER '$2 == ppid && $0 ~ /python/ { print $1 }')
    rm "wt_sender.pid"
    
    echo "Запущен отправитель данных, PID: $PID_SENDER"
}

COMMANDS=()
ALL_ARGS=("$@")
CURRENT_COMMAND=""
i=0

TEST_DATASET=""
TEST_PORT="5555"
TEST_SPEED="1.0"
TEST_LOOP="True"

while [ $i -lt ${#ALL_ARGS[@]} ]; do
  arg="${ALL_ARGS[$i]}"
  
  if [ "$arg" = "record" ] || [ "$arg" = "test" ] || [ "$arg" = "receive" ]; then
    COMMANDS+=("$arg")
    CURRENT_COMMAND="$arg"
    i=$((i+1))
  elif [ "$arg" = "-h" ] || [ "$arg" = "--help" ]; then
    show_help
  else
    case "$arg" in
      -s|--seed)
        if [ "$CURRENT_COMMAND" = "record" ]; then
          RECORD_SEED="${ALL_ARGS[$i+1]}"
          PROVIDED_ARGS[record_seed]=true
          i=$((i+2))
        else
          echo "Ошибка: аргумент $arg можно использовать только с командой record"
          show_help
        fi
        ;;
      --output)
        if [ "$CURRENT_COMMAND" = "record" ]; then
          RECORD_OUTPUT="${ALL_ARGS[$i+1]}"
          PROVIDED_ARGS[record_output]=true
          i=$((i+2))
        else
          echo "Ошибка: аргумент $arg можно использовать только с командой record"
          show_help
        fi
        ;;
      --duration)
        if [ "$CURRENT_COMMAND" = "record" ]; then
          RECORD_DURATION="${ALL_ARGS[$i+1]}"
          i=$((i+2))
        else
          echo "Ошибка: аргумент $arg можно использовать только с командой record"
          show_help
        fi
        ;;
      --filterv)
        if [ "$CURRENT_COMMAND" = "record" ]; then
          RECORD_FILTERV="${ALL_ARGS[$i+1]}"
          PROVIDED_ARGS[record_filterv]=true
          i=$((i+2))
        else
          echo "Ошибка: аргумент $arg можно использовать только с командой record"
          show_help
        fi
        ;;
      --host)
        if [ "$CURRENT_COMMAND" = "record" ]; then
          RECORD_HOST="${ALL_ARGS[$i+1]}"
          PROVIDED_ARGS[record_host]=true
          i=$((i+2))
        else
          echo "Ошибка: аргумент $arg можно использовать только с командой record"
          show_help
        fi
        ;;
      --rport)
        if [ "$CURRENT_COMMAND" = "record" ]; then
          RECORD_PORT="${ALL_ARGS[$i+1]}"
          PROVIDED_ARGS[record_port]=true
          i=$((i+2))
        else
          echo "Ошибка: аргумент $arg можно использовать только с командой record"
          show_help
        fi
        ;;
      --tm-port)
        if [ "$CURRENT_COMMAND" = "record" ]; then
          RECORD_TM_PORT="${ALL_ARGS[$i+1]}"
          PROVIDED_ARGS[record_tm_port]=true
          i=$((i+2))
        else
          echo "Ошибка: аргумент $arg можно использовать только с командой record"
          show_help
        fi
        ;;
      --preview)
        if [ "$CURRENT_COMMAND" = "record" ]; then
          RECORD_PREVIEW=true
          PROVIDED_ARGS[record_preview]=true
          i=$((i+1))
        else
          echo "Ошибка: аргумент $arg можно использовать только с командой record"
          show_help
        fi
        ;;
      --rspeed)
        if [ "$CURRENT_COMMAND" = "record" ]; then
          RECORD_SPEED="${ALL_ARGS[$i+1]}"
          PROVIDED_ARGS[record_speed]=true
          i=$((i+2))
        else
          echo "Ошибка: аргумент $arg можно использовать только с командой record"
          show_help
        fi
        ;;
      --dataset)
        if [ "$CURRENT_COMMAND" = "test" ]; then
          TEST_DATASET="${ALL_ARGS[$i+1]}"
          PROVIDED_ARGS[test_dataset]=true
          i=$((i+2))
        else
          echo "Ошибка: аргумент $arg можно использовать только с командой test"
          show_help
        fi
        ;;
      --tport)
        if [ "$CURRENT_COMMAND" = "test" ]; then
          TEST_PORT="${ALL_ARGS[$i+1]}"
          PROVIDED_ARGS[test_port]=true
          i=$((i+2))
        else
          echo "Ошибка: аргумент $arg можно использовать только с командой test"
          show_help
        fi
        ;;
      --tspeed)
        if [ "$CURRENT_COMMAND" = "test" ]; then
          TEST_SPEED="${ALL_ARGS[$i+1]}"
          PROVIDED_ARGS[test_speed]=true
          i=$((i+2))
        else
          echo "Ошибка: аргумент $arg можно использовать только с командой test"
          show_help
        fi
        ;;
      --loop)
        if [ "$CURRENT_COMMAND" = "test" ]; then
          TEST_LOOP=true
          PROVIDED_ARGS[test_loop]=true
          i=$((i+1))
        else
          echo "Ошибка: аргумент $arg можно использовать только с командой test"
          show_help
        fi
        ;;
      --data-port)
        if [ "$CURRENT_COMMAND" = "receive" ]; then
          RECEIVE_DATA_PORT="${ALL_ARGS[$i+1]}"
          PROVIDED_ARGS[receive_data_port]=true
          i=$((i+2))
        else
          echo "Ошибка: аргумент $arg можно использовать только с командой receive"
          show_help
        fi
        ;;
      --debug)
        if [ "$CURRENT_COMMAND" = "receive" ]; then
          RECEIVE_DEBUG=true
          PROVIDED_ARGS[receive_debug]=true
          i=$((i+1))
        else
          echo "Ошибка: аргумент $arg можно использовать только с командой receive"
          show_help
        fi
        ;;
      --debug-output)
        if [ "$CURRENT_COMMAND" = "receive" ]; then
          RECEIVE_DEBUG_OUTPUT="${ALL_ARGS[$i+1]}"
          PROVIDED_ARGS[receive_debug_output]=true
          i=$((i+2))
        else
          echo "Ошибка: аргумент $arg можно использовать только с командой receive"
          show_help
        fi
        ;;
      --collect-data)
        if [ "$CURRENT_COMMAND" = "receive" ]; then
          RECEIVE_COLLECT_DATA=true
          PROVIDED_ARGS[receive_collect_data]=true
          i=$((i+1))
        else
          echo "Ошибка: аргумент $arg можно использовать только с командой receive"
          show_help
        fi
        ;;
      --collection-data-output)
        if [ "$CURRENT_COMMAND" = "receive" ]; then
          RECEIVE_COLLECTION_DATA_OUTPUT="${ALL_ARGS[$i+1]}"
          PROVIDED_ARGS[receive_collection_data_output]=true
          i=$((i+2))
        else
          echo "Ошибка: аргумент $arg можно использовать только с командой receive"
          show_help
        fi
        ;;
      --collection-max-frames)
        if [ "$CURRENT_COMMAND" = "receive" ]; then
          RECEIVE_COLLECTION_MAX_FRAMES="${ALL_ARGS[$i+1]}"
          PROVIDED_ARGS[receive_collection_max_frames]=true
          i=$((i+2))
        else
          echo "Ошибка: аргумент $arg можно использовать только с командой receive"
          show_help
        fi
        ;;
      --collection-interval)
        if [ "$CURRENT_COMMAND" = "receive" ]; then
          RECEIVE_COLLECTION_INTERVAL="${ALL_ARGS[$i+1]}"
          PROVIDED_ARGS[receive_collection_interval]=true
          i=$((i+2))
        else
          echo "Ошибка: аргумент $arg можно использовать только с командой receive"
          show_help
        fi
        ;;
      *)
        echo "Ошибка: неизвестный аргумент $arg"
        show_help
        ;;
    esac
  fi
done

echo "Найдены команды: ${COMMANDS[@]}"

if [ ${#COMMANDS[@]} -eq 0 ]; then
  echo "Ошибка: не указана команда (record, test или receive)"
  show_help
fi

# Проверка обязательных аргументов
if [[ " ${COMMANDS[@]} " =~ " record " ]]; then
  if [ -z "$RECORD_OUTPUT" ]; then
    echo "Ошибка: не указан обязательный аргумент --output для команды record"
    show_help
  fi
  if [ -z "$RECORD_DURATION" ]; then
    echo "Ошибка: не указан обязательный аргумент --duration для команды record"
    show_help
  fi
fi

if [[ " ${COMMANDS[@]} " =~ " test " ]]; then
  if [ -z "$TEST_DATASET" ]; then
    echo "Ошибка: не указан обязательный аргумент --dataset для команды test"
    show_help
  fi
fi

cleanup_pid_files

echo "============ ПАРАМЕТРЫ ЗАПУСКА ============"
if [[ " ${COMMANDS[@]} " =~ " record " ]]; then
  print_record_params
fi
if [[ " ${COMMANDS[@]} " =~ " test " ]]; then
  print_test_params
fi
if [[ " ${COMMANDS[@]} " =~ " receive " ]]; then
  print_receive_params
fi
echo "=========================================="

if [[ " ${COMMANDS[@]} " =~ " test " ]]; then
  echo "Запуск ORB-SLAM3..."
  start_orbslam3
fi

if [[ " ${COMMANDS[@]} " =~ " receive " ]]; then
  if [[ ! " ${COMMANDS[@]} " =~ " test " ]]; then
    echo "Запуск приемника данных в автономном режиме..."
  else
    echo "Запуск приемника данных вместе с ORB-SLAM3..."
  fi
  start_receiver
fi

if [[ " ${COMMANDS[@]} " =~ " test " ]]; then
  echo "Запуск отправителя данных..."
  start_sender $TEST_DATASET
fi

# Отдельная обработка команды record
if [[ " ${COMMANDS[@]} " =~ " record " ]]; then
  echo "Запуск команды record..."
  
  RECORD_CMD="./record_monocular_dataset.py --output $RECORD_OUTPUT --duration $RECORD_DURATION"
  
  if [[ "${PROVIDED_ARGS[record_seed]}" == "true" ]]; then
    RECORD_CMD="$RECORD_CMD --seed $RECORD_SEED"
  fi
  
  if [[ "${PROVIDED_ARGS[record_filterv]}" == "true" ]]; then
    RECORD_CMD="$RECORD_CMD --filterv \"$RECORD_FILTERV\""
  fi
  
  if [[ "${PROVIDED_ARGS[record_host]}" == "true" ]]; then
    RECORD_CMD="$RECORD_CMD --host $RECORD_HOST"
  fi
  
  if [[ "${PROVIDED_ARGS[record_port]}" == "true" ]]; then
    RECORD_CMD="$RECORD_CMD --port $RECORD_PORT"
  fi
  
  if [[ "${PROVIDED_ARGS[record_tm_port]}" == "true" ]]; then
    RECORD_CMD="$RECORD_CMD --tm-port $RECORD_TM_PORT"
  fi
  
  if [[ "${PROVIDED_ARGS[record_preview]}" == "true" ]]; then
    RECORD_CMD="$RECORD_CMD --preview"
  fi
  
  if [[ "${PROVIDED_ARGS[record_speed]}" == "true" ]]; then
    RECORD_CMD="$RECORD_CMD --speed $RECORD_SPEED"
  fi

  echo "Выполняется команда: $RECORD_CMD"
  echo "Запись набора данных из $RECORD_DURATION секунд симуляции..."
  
  wt $BASH_DIR -c "cd \"$CURRENT_DIR\" && echo \$BASHPID > wt_record.pid && $RECORD_CMD & exit" &
  sleep 1
  PPID_RECORD=$(cat "wt_record.pid")
  PID_RECORD=$(ps aux | awk -v ppid=$PPID_RECORD '$2 == ppid && $0 ~ /python/ { print $1 }')
  rm "wt_record.pid"
  
  echo "Запущен процесс записи датасета, PID: $PID_RECORD"
  
  echo "Ожидание завершения записи набора данных из $RECORD_DURATION секунд симуляции..."
  
  while ps -p $PID_RECORD > /dev/null 2>&1; do
    sleep 1
  done
  
  echo "Процесс записи завершился."
  
  exit 0
fi

if [[ " ${COMMANDS[@]} " =~ " test " ]] || [[ " ${COMMANDS[@]} " =~ " receive " ]]; then
  echo "Все процессы запущены. Нажмите Ctrl+C для завершения."
  
  # Если запущен test без зацикливания
  if [[ " ${COMMANDS[@]} " =~ " test " ]] && [ "$TEST_LOOP" != "true" ]; then
    echo "Режим автозавершения ORB-SLAM3 после окончания подачи датасета активен."
  fi
  
  while true; do
    # Проверка, не завершился ли процесс отправителя в режиме без зацикливания
    if [[ " ${COMMANDS[@]} " =~ " test " ]] && [ "$TEST_LOOP" != "true" ] && [ ! -z "$PID_SENDER" ]; then
      if ! ps -p $PID_SENDER > /dev/null 2>&1; then
        echo "Отправитель данных завершил работу. Датасет полностью обработан."
        
        # Если запущен только test
        if [[ ! " ${COMMANDS[@]} " =~ " receive " ]]; then
          echo "Автоматическое завершение ORB-SLAM3..."
          if [ ! -z "$ORBSLAM_PID" ]; then
            docker exec $CONTAINER_NAME kill -2 "$ORBSLAM_PID" || true
            echo "Процесс ORB-SLAM3 завершён."
            ORBSLAM_PID=""
          fi
          
          echo "Все процессы выполнены, завершение работы."
          break
        else
          echo "Продолжение работы приемника данных..."
          PID_SENDER=""
        fi
      fi
    fi
    
    has_active_processes=false
    
    if [[ " ${COMMANDS[@]} " =~ " test " ]] && [[ ! " ${COMMANDS[@]} " =~ " receive " ]]; then
      if [ ! -z "$PID_SENDER" ] && ! ps -p $PID_SENDER > /dev/null 2>&1; then
        if [ ! -z "$ORBSLAM_PID" ]; then
          has_active_processes=true
        fi
      elif [ ! -z "$PID_SENDER" ]; then
        has_active_processes=true
      fi
    elif [[ " ${COMMANDS[@]} " =~ " receive " ]] && [[ ! " ${COMMANDS[@]} " =~ " test " ]]; then
      
      if [ ! -z "$PID_RECEIVER" ] && ps -p $PID_RECEIVER > /dev/null 2>&1; then 
        has_active_processes=true
      fi
    else
      # Проверка для test+receive
      if [ ! -z "$ORBSLAM_PID" ]; then has_active_processes=true; fi
      if [ ! -z "$PID_RECEIVER" ] && ps -p $PID_RECEIVER > /dev/null 2>&1; then has_active_processes=true; fi
      if [ ! -z "$PID_SENDER" ] && ps -p $PID_SENDER > /dev/null 2>&1; then has_active_processes=true; fi
    fi

    if [ "$has_active_processes" = false ]; then
      echo "Все процессы завершены естественным образом."
      break
    fi
    
    sleep 1
  done
else
  # Бесконечное ожидание до нажатия Ctrl+C
  while true; do
    sleep 1
  done
fi

echo "Процесс завершён."
