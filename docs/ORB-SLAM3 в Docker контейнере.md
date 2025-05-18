Репозиторий: https://github.com/jonasctrl/ORB-SLAM3-docker
- Установка:
```Shell
docker build -t orb_slam3:latest .
```
- 
```Shell
docker run -it --gpus all --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --name orb_slam3_container \
  orb_slam3:latest
  ```
- Подключение к запущенному контейнеру: 
```Shell
docker exec -it orb_slam3_container bash
```
- Установка EuRoC датасета для проверки ORB-SLAM3:
```Shell
cd /opt/orb_slam3

# Create directory for dataset
mkdir -p Datasets/EuRoc
cd Datasets/EuRoc/

# Download the dataset
wget -c http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip

# Create a directory for MH_01_easy dataset and extract it
mkdir MH01
unzip MH_01_easy.zip -d MH01/
```
Для ускорения разработки и чтобы при пересборке контейнера не пришлось заново скачивать большой файл с медленного сервера http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip, лучше его установить заранее на хост Windows, а после передавать внутрь контейнера командой:
```Shell
docker cp <путь_к_файлу_на_хосте> <имя_или_ID_контейнера>:<путь_внутри_контейнера>
```
- Установка пакетов для отображения окон контейнера на Windows:
```Shell
apt-get update && apt-get install -y x11-apps
```
На Windows необходимо установить VcXsrv.
При каждом запуске/подключении к контейнеру необходимо прописывать команду:
```Shell
export DISPLAY=host.docker.internal:0
```
На Windows необходимо, чтобы при этом был запущен сервер через XLaunch.
Конфигурация запуска:
- ![](images/Pasted%20image%2020250118161628.png)
- ![](images/Pasted%20image%2020250118161710.png)
- ![](images/Pasted%20image%2020250118161723.png)
После можно проверить успешность конфигурации через команду ```xclock``` в контейнере. На Windows должны отобразиться часы.
- На VSCode в Windows установлено дополнение Dev Containers от Microsoft для удобного подключения к запущенному контейнеру. После первого запуска контейнер удобно запускать через приложение Docker Desktop.
- Понадобилось установить дополнительную библиотеку внутрь контейнера для получения внешних данных с камеры:
```Shell
sudo apt-get install libzmq3-dev
```
- Также, для запуска примера CMakeLists внутри контейнера был замещён файлом из репозитория ORB-SLAM3-docker\orb-slam-3-extensions\cmake\CMakeLists.txt, а также в контейнер по пути ORB_SLAM3/Examples/Monocular/ скопирован файл ORB-SLAM3-docker\orb-slam-3-extensions\camera.cc.
## Запуск примера Monocular с помощью Python скрипта
- Внутри репозитория по пути ORB-SLAM3-docker/python-data-stream есть файл camera.py.
- Нужно установить зависимости через команды:
```Shell
  pip install opencv-python
  pip install pyzmq
```
- Проверка установки pyzmq:
```Shell
python -c "import zmq; print(zmq.zmq_version())"
```
- Далее сначала в контейнере нужно убедиться в правильной работе VcXsrv, после запустить пример командой:
```Shell
./Examples/Monocular/camera ./Vocabulary/ORBvoc.txt   ./Examples/Stereo/EuRoC.yaml   tcp://host.docker.internal:5555
```
По порту 5555 теперь ORB-SLAM3 будет ожидать изображения с камеры.
- Теперь можно с хоста Windows запускать скрипт для посылки изображений. Нужно убедиться, что камера включена, далее команда:
```Shell
python camera.py
```
## Установка симулятора городской среды CARLA Simulator
Весь проект требует наличия на системном диске не менее 360 ГБ свободной памяти для Unreal Engine 5 сборки и видеокарты уровня не ниже RTX 3070.
Репозиторий: https://github.com/carla-simulator/carla
Документация: https://carla-ue5.readthedocs.io/en/latest/tuto_first_steps/
- Установка:
```Shell
git clone -b ue5-dev https://github.com/carla-simulator/carla.git CarlaUE5
```
- Строго в терминале Windows PowerShell:
```Shell
cd CarlaUE5
CarlaSetup.bat
```
- На моменте установки Visual Studio 2022 необходимо изменить компоненты:
![](images/Pasted%20image%2020250118165555.png)
- Должны быть проставлены следующие галочки:
![](images/Pasted%20image%2020250118165644.png)
Без этого действия Unreal Engine не сможет запуститься.
В конце сборка в любом случае завершится ошибкой.
- Сначала в консоли "Developer Command Prompt for VS 2022" в директории репозитория нужно собрать проект с Python API:
```Shell
cmake --build Build --target carla-python-api-install
```
- Далее и для каждого последующего запуска так же в консоли VS 2022:
```Shell
cmake --build Build --target launch
```
Для управления окружением и последующей передачи данных куда угодно, нужно сначала запускать Unreal Engine через команду выше, а после через Python использовать нужные команды.

## Develop заметки
- Текущая структура проекта:
	- Docker-контейнер: рабочая директория /opt/orb_slam3/ORB_SLAM3
		- Файлы для моно-инерциального режима: ORB_SLAM3/Examples/Monocular-Inertial
		- Конфигурационный файл с настройками для камеры и IMU: carla.yaml
		- Код на C++ для моно-инерциального режима: mono_inertial_zmq.cc

- Строки, добавленные в CMakeLists.txt после комментария "#Monocular inertial examples":
```CMake
add_executable(mono_inertial
        Examples/Monocular-Inertial/mono_inertial_zmq.cc)
target_link_libraries(mono_inertial ${PROJECT_NAME} ${ZeroMQ_LIBRARIES})
```
- Команда для запуска моно-инерциального режима:
```Shell
./Examples/Monocular-Inertial/mono_inertial ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/carla.yaml tcp://host.docker.internal:5555 tcp://host.docker.internal:5556
```
- Команда для пересборки проекта и обновления кода:
```Shell
./build.sh
```
- Команда для запуска моно-инерциального режима с EuRoC:
```Shell
./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/EuRoC.yaml ../Datasets/EuRoc/MH01 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_monocular-inertial
```

- Команда для запуска моно-режима с EuRoC:
```Shell
./Examples/Monocular/mono_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular/EuRoC.yaml ../Datasets/EuRoc/carla_dataset_mono ../Datasets/EuRoC/carla_dataset_mono/mav0/timestamps.txt carla_dataset-mono
```

- Установка библиотеки JSON в Docker контейнер:
```Shell
wget https://raw.githubusercontent.com/nlohmann/json/v3.11.2/single_include/nlohmann/json.hpp -P ./include/nlohmann/
```

- Описание аргументов receive_data.py:
	- "--data-port":
		- type=int
		- default=5557
		- help="Порт для приема всех данных (по умолчанию 5557)"
	- "--debug":
		- action="store_true"
		- help="Включить режим отладки"
	- "--debug-output":
		- type=str
		- default="debug_output"
		- help="Директория для сохранения отладочных данных"
	- "--collect-data":
		- action="store_true"
		- help="Собрать и сохранить данные с 5 кадров с интервалом в 1 секунду"
	- "--data-output":
		- type=str
		- default="collected_data"
		- help="Директория для сохранения собранных данных"

- Установка векторной базы данных Qdrant:
```Shell
pip install qdrant-client
```
- Запуск Qdrant в Docker контейнере. Перед этим запустить Docker.
```Shell
docker run -p 6333:6333 qdrant/qdrant
```

## Актуальный запуск
- В NaviXplore/src/Monocular запустить
```Shell
./receive_data.py --data-port 5557
```
- Подача датасета: в отдельном терминале в NaviXplore/src/Monocular:
```Shell
./test_monocular_dataset.py --dataset carla_dataset_long_60fps/
```
- В контейнере с ORB-SLAM3:
```Shell
./Examples/Monocular/camera_carla_transfer_extended ./Vocabulary/ORBvoc.txt   ./Examples/Monocular/carla_60.yaml tcp://host.docker.internal:5555 tcp://host.docker.internal:5557
```
