#!/usr/bin/env python

"""
Скрипт для сбора монокулярного датасета из симулятора городской среды CARLA Simulator.
Требует запущенного симулятора и установленного Python API для него.
"""

import os
import time
import argparse
import logging
import random
import signal

import cv2
import numpy as np
import carla
from yaml_generator import generate_yaml


SIMULATION_HZ = 60
SIMULATION_DT = 1.0 / SIMULATION_HZ

CAMERA_WIDTH = "640"
CAMERA_HEIGHT = "480"
CAMERA_FOW = "110"

running = True


def signal_handler(sig, frame):
    global running
    running = False
    logging.info("Получен сигнал завершения, остановка записи...")


class SimpleDatasetRecorder:
    def __init__(self, output_path, preview=True):
        self.output_path = output_path
        self.images_path = os.path.join(output_path, "images")
        os.makedirs(self.images_path, exist_ok=True)
        self.timestamps_file = open(
            os.path.join(output_path, "timestamps.txt"), "w", encoding="utf-8"
        )
        self.frame_count = 0
        self.preview = preview

    def handle_image(self, image):
        """Обрабатывает изображения. Записывает их в файлы .png с названием в виде метки времени в наносекундах.
        При активированной опции self.preview показывает окно предпросмотра.
        """
        timestamp_ns = int(time.time() * 1e9)
        filename = f"{timestamp_ns}.png"
        img_array = np.frombuffer(image.raw_data, dtype=np.uint8)
        img = img_array.reshape((image.height, image.width, 4))
        img_bgr = img[:, :, :3]
        output_file = os.path.join(self.images_path, filename)
        cv2.imwrite(output_file, img_bgr)
        self.timestamps_file.write(f"{timestamp_ns}\n")
        self.timestamps_file.flush()
        self.frame_count += 1

        if self.preview:
            cv2.imshow("Recording", img_bgr)
            cv2.waitKey(1)

    def close(self):
        self.timestamps_file.close()
        logging.info("Записано %d кадров", self.frame_count)


def disable_traffic_lights(world):
    """Принудительно замораживает светофоры в состоянии Green."""
    traffic_lights = world.get_actors().filter("traffic.traffic_light")

    for light in traffic_lights:
        light.set_state(carla.TrafficLightState.Green)
        light.freeze(True)


def main():
    signal.signal(signal.SIGINT, signal_handler)
    parser = argparse.ArgumentParser(
        description="Запись монокулярного датасета из CARLA"
    )
    parser.add_argument(
        "--output", type=str, required=True, help="Путь для сохранения датасета"
    )
    parser.add_argument(
        "--duration", type=int, default=60, help="Длительность записи в секундах"
    )
    parser.add_argument("--seed", type=int, default=int(time.time()))
    parser.add_argument("--filterv", metavar="PATTERN", default="vehicle.dodge.charger")
    parser.add_argument("--host", default="127.0.0.1", help="IP-адрес CARLA сервера")
    parser.add_argument("--port", type=int, default=2000, help="Порт CARLA сервера")
    parser.add_argument(
        "--tm-port", type=int, default=8000, help="Порт Traffic Manager"
    )
    parser.add_argument("--preview", dest="preview", action="store_true", default=True)
    parser.add_argument(
        "--speed",
        type=float,
        default=100.0,
        help="Скорость движения в процентах (100%% - нормальная, 50%% - половина)",
    )
    args = parser.parse_args()
    random.seed(args.seed)
    logging.basicConfig(format="%(levelname)s: %(message)s", level=logging.INFO)
    logging.info("Запись датасета в директорию: %s", args.output)
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    disable_traffic_lights(world)
    # original_settings = world.get_settings()
    settings = world.get_settings()
    settings.fixed_delta_seconds = SIMULATION_DT
    settings.synchronous_mode = True
    world.apply_settings(settings)
    vehicle = None
    camera = None
    recorder = None

    try:
        tm = client.get_trafficmanager(args.tm_port)
        tm.set_synchronous_mode(True)
        tm.global_percentage_speed_difference(100 - args.speed)
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter(args.filterv)[0]
        spawn_points = world.get_map().get_spawn_points()

        if not spawn_points:
            raise ValueError("На карте нет точек для спавна")

        vehicle = None

        for spawn_point in spawn_points:
            vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)

            if vehicle:
                break

        if not vehicle:
            raise ValueError("Не удалось создать автомобиль")

        camera_bp = blueprint_library.find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", CAMERA_WIDTH)
        camera_bp.set_attribute("image_size_y", CAMERA_HEIGHT)
        camera_bp.set_attribute("fov", CAMERA_FOW)
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
        recorder = SimpleDatasetRecorder(args.output, preview=args.preview)
        camera.listen(lambda image: recorder.handle_image(image))
        vehicle.set_autopilot(True, args.tm_port)
        logging.info("Начинается запись датасета на %d секунд...", args.duration)
        num_frames = int(args.duration * SIMULATION_HZ)

        for i in range(num_frames):
            if not running:
                break

            world.tick()

            if i % (num_frames // 10) == 0 and i > 0:
                progress = i / num_frames * 100
                logging.info("Прогресс записи: %.1f%%", progress)

        logging.info("Генерация конфигурационного .yaml файла...")
        yaml_name = generate_yaml(
            CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FOW, SIMULATION_HZ, args.output
        )
        logging.info("YAML файл сгенерирован: %s", yaml_name)

    except KeyboardInterrupt:
        logging.info("Запись прервана пользователем")
    except Exception as e:
        logging.error("Ошибка при записи: %s", e)
    finally:
        logging.info("Завершение записи...")
        if camera:
            camera.stop()
        new_settings = world.get_settings()
        new_settings.synchronous_mode = False
        new_settings.fixed_delta_seconds = None
        world.apply_settings(new_settings)

        if camera:
            camera.destroy()
        if vehicle:
            vehicle.set_autopilot(False)
            vehicle.destroy()
        if recorder:
            recorder.close()

        logging.info("Запись завершена")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logging.error("Критическая ошибка: %s", e)
    finally:
        logging.info("Программа завершена")

# Использование: ./record_monocular_dataset.py --output <Директория> --duration <Длительность в секундах>
