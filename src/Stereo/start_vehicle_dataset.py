#!/usr/bin/env python

"""
Неактуальный скрипт для запуска обхода транспорта траектории в стерео режиме в CARLA Simulator.
Требует запущенного симулятора и установленного Python API для него.
"""

import os
import time
import argparse
import logging
import carla
import cv2
import numpy as np
import math
import yaml
import random


TARGET_FPS = 20
SIMULATION_HZ = 20
SIMULATION_DT = 1.0 / SIMULATION_HZ


def create_sensor_configs(output_path, left_transform, right_transform):
    """Создает конфигурационные файлы для стерео датасета."""
    # body.yaml
    body_yaml = {
        'comment': 'CARLA Stereo Vehicle Dataset'
    }
    
    # cam0/sensor.yaml (левая камера)
    left_camera_yaml = {
        'sensor_type': 'camera',
        'comment': 'CARLA Left RGB Camera for Stereo',
        'T_BS': {
            'cols': 4,
            'rows': 4,
            'data': [1.0, 0.0, 0.0, left_transform.location.x,
                     0.0, 1.0, 0.0, left_transform.location.y,
                     0.0, 0.0, 1.0, left_transform.location.z,
                     0.0, 0.0, 0.0, 1.0]
        },
        'rate_hz': TARGET_FPS,
        'resolution': [752, 480],
        'camera_model': 'pinhole',
        'intrinsics': [458.654, 457.296, 367.215, 248.375],
        'distortion_model': 'radial-tangential',
        'distortion_coefficients': [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05]
    }
    
    # cam1/sensor.yaml (правая камера)
    right_camera_yaml = {
        'sensor_type': 'camera',
        'comment': 'CARLA Right RGB Camera for Stereo',
        'T_BS': {
            'cols': 4,
            'rows': 4,
            'data': [1.0, 0.0, 0.0, right_transform.location.x,
                     0.0, 1.0, 0.0, right_transform.location.y,
                     0.0, 0.0, 1.0, right_transform.location.z,
                     0.0, 0.0, 0.0, 1.0]
        },
        'rate_hz': TARGET_FPS,
        'resolution': [752, 480],
        'camera_model': 'pinhole',
        'intrinsics': [458.654, 457.296, 367.215, 248.375],
        'distortion_model': 'radial-tangential',
        'distortion_coefficients': [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05]
    }
    
    os.makedirs(os.path.join(output_path, "mav0", "cam0", "data"), exist_ok=True)
    os.makedirs(os.path.join(output_path, "mav0", "cam1", "data"), exist_ok=True)
    
    with open(os.path.join(output_path, "mav0", "body.yaml"), 'w') as f:
        yaml.dump(body_yaml, f, default_flow_style=None)
    with open(os.path.join(output_path, "mav0", "cam0", "sensor.yaml"), 'w') as f:
        yaml.dump(left_camera_yaml, f, default_flow_style=None)
    with open(os.path.join(output_path, "mav0", "cam1", "sensor.yaml"), 'w') as f:
        yaml.dump(right_camera_yaml, f, default_flow_style=None)


def verify_dataset(dataset_path):
    """Проверяет корректность записанного датасета для стерео камеры."""
    logging.info("\nПроверка стерео датасета:")
    
    time.sleep(0.1)
    
    required_paths = [
        "mav0/timestamps.txt",
        "mav0/body.yaml",
        "mav0/cam0/data",
        "mav0/cam0/data.csv",
        "mav0/cam1/data",
        "mav0/cam1/data.csv"
    ]
    
    for path in required_paths:
        full_path = os.path.join(dataset_path, path)
        if not os.path.exists(full_path):
            logging.error(f"Отсутствует {path}")
            return False
    
    images_path = os.path.join(dataset_path, "mav0", "cam0", "data")
    images = sorted(os.listdir(images_path))
    image_count = len(images)
    
    if image_count == 0:
        logging.error("Датасет пуст!")
        return False
    
    first_img = int(images[0].split('.')[0]) / 1e9
    last_img = int(images[-1].split('.')[0]) / 1e9
    duration = last_img - first_img
    
    cam_freq = image_count / duration if duration > 0 else 0
    
    logging.info(f"Длительность записи: {duration:.1f} секунд")
    logging.info(f"Количество изображений (cam0): {image_count}")
    logging.info(f"Частота камеры: {cam_freq:.1f} Hz (ожидалось {TARGET_FPS})")
    
    return True


class StereoDatasetRecorder:
    def __init__(self, output_path):
        self.base_path = output_path
        self.mav0_path = os.path.join(output_path, "mav0")
        self.cam0_path = os.path.join(self.mav0_path, "cam0")
        self.cam1_path = os.path.join(self.mav0_path, "cam1")

        os.makedirs(self.mav0_path, exist_ok=True)
        os.makedirs(os.path.join(self.cam0_path, "data"), exist_ok=True)
        os.makedirs(os.path.join(self.cam1_path, "data"), exist_ok=True)

        self.common_timestamps = open(os.path.join(self.mav0_path, "timestamps.txt"), "w")
        self.cam0_data = open(os.path.join(self.cam0_path, "data.csv"), "w")
        self.cam0_data.write("#timestamp [ns],filename\n")
        self.cam1_data = open(os.path.join(self.cam1_path, "data.csv"), "w")
        self.cam1_data.write("#timestamp [ns],filename\n")
        
        self.frame_store = {}
        self.frame_count = 0
        self.start_time = None

    def try_save_frame(self, frame_id):
        entry = self.frame_store.get(frame_id, {})
        if 'left' in entry and 'right' in entry:
            if self.start_time is None:
                self.start_time = frame_id * (1e9 / TARGET_FPS)
            
            universal_ts = int(self.start_time + frame_id * (1e9 / TARGET_FPS))
            filename = f"{universal_ts}.png"
            
            left_img = entry['left']
            frame_left = np.frombuffer(left_img.raw_data, dtype=np.uint8)
            frame_left = frame_left.reshape((left_img.height, left_img.width, 4))[:, :, :3]
            frame_left_gray = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY)
            cv2.imwrite(os.path.join(self.mav0_path, "cam0", "data", filename), frame_left_gray)

            right_img = entry['right']
            frame_right = np.frombuffer(right_img.raw_data, dtype=np.uint8)
            frame_right = frame_right.reshape((right_img.height, right_img.width, 4))[:, :, :3]
            frame_right_gray = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)
            cv2.imwrite(os.path.join(self.mav0_path, "cam1", "data", filename), frame_right_gray)

            self.common_timestamps.write(f"{universal_ts}\n")
            self.common_timestamps.flush()
            self.cam0_data.write(f"{universal_ts},{filename}\n")
            self.cam0_data.flush()
            self.cam1_data.write(f"{universal_ts},{filename}\n")
            self.cam1_data.flush()
            self.frame_count += 1
            del self.frame_store[frame_id]

    def handle_left(self, image):
        """Обрабатывает изображение левой камеры."""
        frame_id = image.frame
        if frame_id not in self.frame_store:
            self.frame_store[frame_id] = {}
        self.frame_store[frame_id]['left'] = image
        self.try_save_frame(frame_id)
        return True

    def handle_right(self, image):
        """Обрабатывает изображение правой камеры."""
        frame_id = image.frame
        if frame_id not in self.frame_store:
            self.frame_store[frame_id] = {}
        self.frame_store[frame_id]['right'] = image
        self.try_save_frame(frame_id)
        return True

    def close(self):
        """Закрывает файлы и выводит статистику."""
        self.common_timestamps.close()
        self.cam0_data.close()
        self.cam1_data.close()
        if self.frame_count > 0:
            logging.info(f"Записано {self.frame_count} пар кадров (универсальный timestamp)")
        else:
            logging.warning("Не было записано ни одной пары кадров!")


def move_vehicle_for_initialization(world, vehicle, duration=5):
    """
    Плавное движение для инициализации датасета:
    - Плавные повороты и небольшие смещения для стабилизации датчиков.
    """
    start_time = time.time()
    spawn_transform = vehicle.get_transform()
    base_loc = spawn_transform.location
    base_yaw = spawn_transform.rotation.yaw
    yaw_amplitude = 5.0
    yaw_frequency = 0.5
    while (time.time() - start_time) < duration:
        current_time = time.time() - start_time
        yaw_offset = yaw_amplitude * math.sin(2 * math.pi * yaw_frequency * current_time)
        new_yaw = (base_yaw + yaw_offset) % 360
        new_transform = carla.Transform(
            carla.Location(x=base_loc.x, y=base_loc.y, z=base_loc.z),
            carla.Rotation(pitch=0, yaw=new_yaw, roll=0)
        )
        vehicle.set_transform(new_transform)
        world.tick()
    vehicle.set_transform(spawn_transform)
    world.tick()


def disable_traffic_lights(world):
    """Принудительно замораживает светофоры в состоянии Green."""
    traffic_lights = world.get_actors().filter("traffic.traffic_light")
    for light in traffic_lights:
        light.set_state(carla.TrafficLightState.Green)
        light.freeze(True)


def main():
    argparser = argparse.ArgumentParser(description="Запись стерео датасета из CARLA")
    argparser.add_argument('--output', type=str, required=True, help='Путь для сохранения датасета')
    argparser.add_argument('--duration', type=int, default=60, help='Длительность записи в секундах')
    argparser.add_argument('--vehicle', default='vehicle.dodge.charger', help='Модель автомобиля')
    argparser.add_argument('--map', default='Town01', help='Карта CARLA (без пути)')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(20.0)
    
    available_maps = client.get_available_maps()
    logging.info(f"Доступные карты: {available_maps}")
    map_name = args.map
    full_map_path = None
    for m in available_maps:
        if map_name in m:
            full_map_path = m
            break
    if full_map_path is None:
        raise ValueError(f"Карта '{map_name}' не найдена. Доступные карты: {available_maps}")
    
    logging.info(f"Загрузка карты {full_map_path}...")
    world = client.load_world(full_map_path)
    
    settings = world.get_settings()
    os.makedirs(args.output, exist_ok=True)
    
    recorder = StereoDatasetRecorder(args.output)
    settings.fixed_delta_seconds = SIMULATION_DT
    settings.synchronous_mode = True
    settings.no_rendering_mode = True
    world.apply_settings(settings)

    try:
        world = client.get_world()
        disable_traffic_lights(world)

        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
        traffic_manager.global_percentage_speed_difference(30.0)

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = SIMULATION_DT
        settings.no_rendering_mode = True
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.find(args.vehicle)
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            raise ValueError("Не найдены точки спавна!")
        spawn_point = spawn_points[0]
        
        vehicle = None
        while vehicle is None:
            vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
            if vehicle is None:
                spawn_point = random.choice(spawn_points)
        
        logging.info(f"Создан автомобиль: {vehicle.type_id}")
        
        left_cam_bp = blueprint_library.find('sensor.camera.rgb')
        left_cam_bp.set_attribute('image_size_x', '752')
        left_cam_bp.set_attribute('image_size_y', '480')
        left_cam_bp.set_attribute('fov', '110')
        left_cam_bp.set_attribute('sensor_tick', '0.05')
        left_transform = carla.Transform(
            carla.Location(x=1.5, y=-0.30, z=2.4),
            carla.Rotation(pitch=0, yaw=2, roll=0)
        )
        left_camera = world.spawn_actor(left_cam_bp, left_transform, attach_to=vehicle)
        
        right_cam_bp = blueprint_library.find('sensor.camera.rgb')
        right_cam_bp.set_attribute('image_size_x', '752')
        right_cam_bp.set_attribute('image_size_y', '480')
        right_cam_bp.set_attribute('fov', '110')
        right_cam_bp.set_attribute('sensor_tick', '0.05')
        right_transform = carla.Transform(
            carla.Location(x=1.5, y=0.30, z=2.4),
            carla.Rotation(pitch=0, yaw=-2, roll=0)
        )
        right_camera = world.spawn_actor(right_cam_bp, right_transform, attach_to=vehicle)
        
        create_sensor_configs(args.output, left_transform, right_transform)
        
        recorder = StereoDatasetRecorder(args.output)
        left_camera.listen(lambda image: recorder.handle_left(image))
        right_camera.listen(lambda image: recorder.handle_right(image))
        
        logging.info("Стабилизация мира...")
        for _ in range(10):
            world.tick()
            time.sleep(0.05)
        
        vehicle.set_autopilot(False)
        logging.info("Инициализация: выполняется плавное движение...")
        move_vehicle_for_initialization(world, vehicle, duration=5)
        
        vehicle.set_autopilot(True)
        traffic_manager.vehicle_percentage_speed_difference(vehicle, 0)
        traffic_manager.distance_to_leading_vehicle(vehicle, 10)
        
        logging.info("Начинаем запись...")
        num_ticks = int(args.duration * SIMULATION_HZ)
        for _ in range(num_ticks):
            world.tick()
        
        logging.info("Удаление акторов...")
        left_camera.stop()
        right_camera.stop()
        time.sleep(0.1)
        
        recorder.close()
        verify_dataset(args.output)
        
        left_camera.destroy()
        right_camera.destroy()
        vehicle.destroy()
        
    finally:
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        settings.no_rendering_mode = False
        world.apply_settings(settings)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        logging.info("Готово")
