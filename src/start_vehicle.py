#!/usr/bin/env python

import time
import threading
import signal
import argparse
import logging
import carla
import zmq
import cv2
import numpy as np
from collections import deque
import math
import random

TARGET_FPS = 5
FRAME_TIME = 1 / TARGET_FPS
N_VEHICLES = 1

IMU_FREQUENCY = 200
IMU_FRAME_TIME = 1 / IMU_FREQUENCY

context = zmq.Context()

camera_socket = context.socket(zmq.PUB)
camera_socket.bind("tcp://*:5555")

imu_socket = context.socket(zmq.PUB)
imu_socket.bind("tcp://*:5556")

RUNNING = True


def signal_handler(_, __):
    """Вызывается при сигнатуре SIGINT (Ctrl+C) для остановки цикла программы."""
    global RUNNING
    RUNNING = False
    print("Завершение работы...")


signal.signal(signal.SIGINT, signal_handler)


def disable_traffic_lights(world):
    """Принудительно замораживает светофоры в состоянии Green."""
    traffic_lights = world.get_actors().filter("traffic.traffic_light")
    for light in traffic_lights:
        light.set_state(carla.TrafficLightState.Green)
        light.freeze(True)


def get_actor_blueprints(world, filterv):
    """Возвращает доступные чертежи (blueprints) акторов, отфильтрованных по filterv."""
    return world.get_blueprint_library().filter(filterv)


def handle_camera(image, image_queue):
    """Callback для прослушивания данных с камеры, складывает (кадр, метка-времени) в очередь."""
    timestamp = image.timestamp
    image_queue.append((image, timestamp))


def handle_imu(imu_data, imu_data_queue):
    """
    Callback для прослушивания данных IMU.
    Сохраняет ускорения, гироскопические данные и временную метку в очередь.
    """
    timestamp = imu_data.timestamp
    imu_data_queue.append(
        {
            "accel": (
                imu_data.accelerometer.x,
                imu_data.accelerometer.y,
                imu_data.accelerometer.z,
            ),
            "gyro": (
                imu_data.gyroscope.x,
                imu_data.gyroscope.y,
                imu_data.gyroscope.z,
            ),
            "timestamp": timestamp,
        }
    )


def process_data(image_queue, imu_data_queue, display_image):
    """
    Поток для синхронной обработки данных:
    1. Извлекаем последний кадр + метку времени.
    2. Отбираем из очереди IMU все события, чья метка <= метки кадра.
    3. Отправляем кадр и IMU-события через ZMQ.
    4. Опционально выводим кадр в локальном окне.
    """
    global RUNNING
    while RUNNING:
        if not image_queue:
            time.sleep(0.01)
            continue

        image, image_timestamp = image_queue.popleft()

        imu_data_for_frame = []
        while imu_data_queue and imu_data_queue[0]["timestamp"] <= image_timestamp:
            imu_data_for_frame.append(imu_data_queue.popleft())

        if not imu_data_for_frame:
            print(f"Warning: no IMU data for frame at {image_timestamp:.4f}")

        frame_data = np.frombuffer(image.raw_data, dtype=np.uint8)
        frame = frame_data.reshape((image.height, image.width, 4))[:, :, :3]

        _, jpeg_data = cv2.imencode(".jpg", frame)
        camera_socket.send(jpeg_data.tobytes(), zmq.SNDMORE)
        camera_socket.send_string(str(image_timestamp))

        imu_data_str_list = []
        for imu_event in imu_data_for_frame:
            ax, ay, az = imu_event["accel"]
            gx, gy, gz = imu_event["gyro"]
            imu_data_str_list.append(
                f"Accel: ({ax:+8.4f}, {ay:+8.4f}, {az:+8.4f})\n"
                f"Gyro:  ({gx:+8.4f}, {gy:+8.4f}, {gz:+8.4f})"
            )
        imu_data_str = "\n".join(imu_data_str_list)

        imu_socket.send_string(imu_data_str)
        imu_socket.send_string(str(image_timestamp))

        if display_image:
            overlay_frame = frame.copy()
            overlay = overlay_frame.copy()
            cv2.rectangle(overlay, (10, 10), (460, 80), (0, 0, 0), -1)
            alpha = 0.5
            cv2.addWeighted(overlay, alpha, overlay_frame, 1 - alpha, 0, overlay_frame)

            lines = imu_data_str.split("\n")
            if len(lines) >= 2:
                cv2.putText(
                    overlay_frame,
                    lines[0],
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    1,
                )
                cv2.putText(
                    overlay_frame,
                    lines[1],
                    (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    1,
                )

            cv2.imshow("Camera and IMU", overlay_frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                RUNNING = False


def move_vehicle_for_initialization(world, synchronous_master, vehicle, duration=10):
    """
    Инициализация сенсоров автомобиля:
    - Плавные повороты носом (yaw) из стороны в сторону
    - Небольшие вертикальные движения передней частью
    - Лёгкие перемещения вперёд-назад
    - Плавное возвращение в точку спавна в конце
    """
    start_time = time.time()

    spawn_transform = vehicle.get_transform()
    base_loc = spawn_transform.location
    base_yaw = spawn_transform.rotation.yaw
    base_pitch = spawn_transform.rotation.pitch
    base_roll = 0.0

    yaw_amplitude = 10.0
    yaw_frequency = 0.5
    z_amplitude = 0.2
    z_frequency = 1.0
    x_amplitude = 0.3
    x_frequency = 0.25

    main_phase_duration = duration * 0.8
    while (time.time() - start_time) < main_phase_duration and RUNNING:
        current_time = time.time() - start_time

        yaw_offset = yaw_amplitude * math.sin(
            2 * math.pi * yaw_frequency * current_time
        )
        new_yaw = base_yaw + yaw_offset
        new_yaw = new_yaw % 360

        z_offset = z_amplitude * math.sin(2 * math.pi * z_frequency * current_time)
        new_z = base_loc.z + z_offset

        x_offset = x_amplitude * math.cos(2 * math.pi * x_frequency * current_time)
        forward_vector = spawn_transform.get_forward_vector()
        new_x = base_loc.x + forward_vector.x * x_offset
        new_y = base_loc.y + forward_vector.y * x_offset

        new_transform = carla.Transform(
            location=carla.Location(x=new_x, y=new_y, z=new_z),
            rotation=carla.Rotation(pitch=base_pitch, yaw=new_yaw, roll=base_roll),
        )
        vehicle.set_transform(new_transform)

        time.sleep(0.02)
        if synchronous_master:
            world.tick()
        else:
            world.wait_for_tick()

    return_phase_duration = duration * 0.2
    return_start_time = time.time()
    start_transform = vehicle.get_transform()

    while (time.time() - return_start_time) < return_phase_duration and RUNNING:
        alpha = (time.time() - return_start_time) / return_phase_duration
        alpha = alpha * alpha * (3 - 2 * alpha)

        current_loc = carla.Location(
            x=start_transform.location.x
            + (spawn_transform.location.x - start_transform.location.x) * alpha,
            y=start_transform.location.y
            + (spawn_transform.location.y - start_transform.location.y) * alpha,
            z=start_transform.location.z
            + (spawn_transform.location.z - start_transform.location.z) * alpha,
        )

        start_yaw = start_transform.rotation.yaw % 360
        target_yaw = spawn_transform.rotation.yaw % 360

        yaw_diff = (target_yaw - start_yaw + 180) % 360 - 180
        current_yaw = start_yaw + yaw_diff * alpha

        new_transform = carla.Transform(
            location=current_loc,
            rotation=carla.Rotation(
                pitch=spawn_transform.rotation.pitch, yaw=current_yaw, roll=base_roll
            ),
        )
        vehicle.set_transform(new_transform)

        time.sleep(0.02)
        if synchronous_master:
            world.tick()
        else:
            world.wait_for_tick()

    vehicle.set_transform(spawn_transform)

    time.sleep(0.1)
    if synchronous_master:
        world.tick()
    else:
        world.wait_for_tick()


def main():
    global RUNNING
    global N_VEHICLES

    argparser = argparse.ArgumentParser(description="Mono-Inertial SLAM with Carla")
    argparser.add_argument(
        "--seed",
        metavar="S",
        type=int,
        default=int(time.time()),
        help="Установить случайный сид для Traffic Manager",
    )
    argparser.add_argument(
        "--host",
        metavar="H",
        default="127.0.0.1",
        help="IP хост сервера (по-умолчанию: 127.0.0.1)",
    )
    argparser.add_argument(
        "-p",
        "--port",
        metavar="P",
        default=2000,
        type=int,
        help="TCP порт Carla (по-умолчанию: 2000)",
    )
    argparser.add_argument(
        "--filterv",
        metavar="PATTERN",
        default="vehicle.dodge.charger",
        help='Выбрать модель (по-умолчанию: "vehicle.dodge.charger")',
    )
    argparser.add_argument(
        "--tm-port",
        metavar="P",
        default=8000,
        type=int,
        help="Порт для Traffic Manager (по-умолчанию: 8000)",
    )
    argparser.add_argument(
        "--car-lights-on",
        action="store_true",
        default=False,
        help="Активировать свет машин",
    )
    argparser.add_argument(
        "--no-display",
        action="store_true",
        default=False,
        help="Отключить отображение видео",
    )

    args = argparser.parse_args()
    display_image = not args.no_display

    logging.basicConfig(format="%(levelname)s: %(message)s", level=logging.INFO)

    vehicles_list = []
    sensors_list = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False
    random.seed(args.seed)

    try:
        world = client.get_world()

        disable_traffic_lights(world)

        # Traffic Manager
        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        settings = world.get_settings()
        traffic_manager.set_synchronous_mode(True)

        if not settings.synchronous_mode:
            synchronous_master = True
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        blueprints = get_actor_blueprints(world, args.filterv)
        if not blueprints:
            raise ValueError(
                "Не найдено ни одного подходящего автомобиля по заданному фильтру!"
            )

        blueprints = sorted(blueprints, key=lambda bp: bp.id)
        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if N_VEHICLES < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif N_VEHICLES > number_of_spawn_points:
            logging.warning("Запрошено больше машин, чем доступных точек спавна.")
            N_VEHICLES = number_of_spawn_points

        batch = []
        for n, transform in enumerate(spawn_points):
            if n >= N_VEHICLES:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute("color"):
                color_vals = blueprint.get_attribute("color").recommended_values
                blueprint.set_attribute("color", random.choice(color_vals))
            if blueprint.has_attribute("driver_id"):
                driver_vals = blueprint.get_attribute("driver_id").recommended_values
                blueprint.set_attribute("driver_id", random.choice(driver_vals))
            else:
                blueprint.set_attribute("role_name", "autopilot")

            batch.append(
                carla.command.SpawnActor(blueprint, transform).then(
                    carla.command.SetAutopilot(
                        carla.command.FutureActor, True, traffic_manager.get_port()
                    )
                )
            )

        for response in client.apply_batch_sync(batch, synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

        print(f"Добавлено {len(vehicles_list)} машин(ы). Нажмите Ctrl+C для выхода.")
        traffic_manager.global_percentage_speed_difference(30.0)

        vehicle = world.get_actor(vehicles_list[0])
        vehicle.set_autopilot(False)

        blueprint_library = world.get_blueprint_library()
        camera_bp = blueprint_library.find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", "752")
        camera_bp.set_attribute("image_size_y", "480")
        camera_bp.set_attribute("fov", "110")
        camera_spawn_point = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_bp, camera_spawn_point, attach_to=vehicle)
        sensors_list.append(camera)

        imu_bp = blueprint_library.find("sensor.other.imu")
        imu_spawn_point = carla.Transform(carla.Location(x=0, z=0))
        imu_sensor = world.spawn_actor(imu_bp, imu_spawn_point, attach_to=vehicle)
        sensors_list.append(imu_sensor)

        if synchronous_master:
            print("Делаем несколько tick(), чтобы мир обновился...")
            for _ in range(10):
                world.tick()
                time.sleep(0.05)
        else:
            print("Делаем несколько кадров через wait_for_tick()...")
            for _ in range(10):
                world.wait_for_tick()
                time.sleep(0.05)

        image_queue = deque()
        imu_data_queue = deque()
        camera.listen(lambda img: handle_camera(img, image_queue))
        imu_sensor.listen(lambda imu: handle_imu(imu, imu_data_queue))

        data_thread = threading.Thread(
            target=process_data,
            args=(image_queue, imu_data_queue, display_image),
            daemon=True,
        )
        data_thread.start()

        print("Выполняем серию раскачиваний для инициализации IMU...")
        move_vehicle_for_initialization(world, synchronous_master, vehicle)
        print("Инициализация IMU завершена.")

        vehicle.set_autopilot(True)

        while RUNNING:
            if synchronous_master:
                world.tick()
            else:
                world.wait_for_tick()

    finally:
        if synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

        print(
            f"Останавливаем {len(sensors_list)} сенсоров и удаляем {len(vehicles_list)} машин(ы)..."
        )
        for sensor in sensors_list:
            sensor.stop()

        client.apply_batch(
            [carla.command.DestroyActor(x) for x in vehicles_list + sensors_list]
        )

        camera_socket.close()
        imu_socket.close()
        context.term()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print("Готово.")
