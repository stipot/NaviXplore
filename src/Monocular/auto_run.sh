#!/bin/bash

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
