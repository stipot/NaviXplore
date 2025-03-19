#!/usr/bin/env python

import os
import time
import signal
import argparse
import logging
import carla
import cv2
import numpy as np
import math
import random
import yaml
from collections import deque



SIMULATION_HZ = 200
TARGET_FPS = 20
IMU_FREQUENCY = 200
FRAME_TIME = 1.0 / TARGET_FPS
IMU_FRAME_TIME = 1.0 / IMU_FREQUENCY
SIMULATION_DT = 1.0 / SIMULATION_HZ

def create_sensor_configs(output_path, camera_transform, imu_transform):
    """Создает конфигурационные файлы датасета."""
    # body.yaml
    body_yaml = {
        'comment': 'CARLA Vehicle Dataset'
    }
    
    # cam0/sensor.yaml
    camera_yaml = {
        'sensor_type': 'camera',
        'comment': 'CARLA RGB Camera',
        'T_BS': {
            'cols': 4,
            'rows': 4,
            'data': [1.0, 0.0, 0.0, camera_transform.location.x,
                    0.0, 1.0, 0.0, camera_transform.location.y,
                    0.0, 0.0, 1.0, camera_transform.location.z,
                    0.0, 0.0, 0.0, 1.0]
        },
        'rate_hz': TARGET_FPS,
        'resolution': [752, 480],
        'camera_model': 'pinhole',
        'intrinsics': [458.654, 457.296, 367.215, 248.375],
        'distortion_model': 'radial-tangential',
        'distortion_coefficients': [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05]
    }
    
    # imu0/sensor.yaml
    imu_yaml = {
        'sensor_type': 'imu',
        'comment': 'CARLA IMU Sensor',
        'T_BS': {
            'cols': 4,
            'rows': 4,
            'data': [1.0, 0.0, 0.0, imu_transform.location.x,
                    0.0, 1.0, 0.0, imu_transform.location.y,
                    0.0, 0.0, 1.0, imu_transform.location.z,
                    0.0, 0.0, 0.0, 1.0]
        },
        'rate_hz': IMU_FREQUENCY,
        'gyroscope_noise_density': 1.6968e-04,
        'gyroscope_random_walk': 1.9393e-05,
        'accelerometer_noise_density': 2.0000e-3,
        'accelerometer_random_walk': 3.0000e-3
    }
    
    with open(os.path.join(output_path, "mav0/body.yaml"), 'w') as f:
        yaml.dump(body_yaml, f, default_flow_style=None)
    with open(os.path.join(output_path, "mav0/cam0/sensor.yaml"), 'w') as f:
        yaml.dump(camera_yaml, f, default_flow_style=None)
    with open(os.path.join(output_path, "mav0/imu0/sensor.yaml"), 'w') as f:
        yaml.dump(imu_yaml, f, default_flow_style=None)

def verify_dataset(dataset_path):
    """Проверяет корректность записанного датасета."""
    logging.info("\nПроверка датасета:")
    
    time.sleep(0.1)
    
    required_paths = [
        "mav0/cam0/data",
        "mav0/cam0/data.csv",
        "mav0/cam0/timestamps.txt",
        "mav0/cam0/sensor.yaml",
        "mav0/imu0/data.csv",
        "mav0/imu0/sensor.yaml",
        "mav0/body.yaml"
    ]
    
    for path in required_paths:
        full_path = os.path.join(dataset_path, path)
        if not os.path.exists(full_path):
            logging.error(f"Отсутствует {path}")
            return False
    
    imu_data_path = os.path.join(dataset_path, "mav0/imu0/data.csv")
    with open(imu_data_path, 'r') as f:
        imu_lines = f.readlines()[1:]
        imu_count = len(imu_lines)
    
    images_path = os.path.join(dataset_path, "mav0/cam0/data")
    images = sorted(os.listdir(images_path))
    image_count = len(images)
    
    if image_count == 0 or imu_count == 0:
        logging.error("Датасет пуст!")
        return False
    
    first_img = int(images[0].split('.')[0]) / 1e9
    last_img = int(images[-1].split('.')[0]) / 1e9
    duration = last_img - first_img
    
    cam_freq = image_count / duration
    imu_freq = imu_count / duration
    
    logging.info(f"Длительность записи: {duration:.1f} секунд")
    logging.info(f"Количество изображений: {image_count}")
    logging.info(f"Количество IMU измерений: {imu_count}")
    logging.info(f"Соотношение IMU/Image: {imu_count/image_count:.1f}")
    logging.info(f"Частота камеры: {cam_freq:.1f} Hz (ожидалось {TARGET_FPS})")
    logging.info(f"Частота IMU: {imu_freq:.1f} Hz (ожидалось {IMU_FREQUENCY})")
    
    return True

class DatasetRecorder:
    def __init__(self, output_path):
        self.base_path = output_path
        self.mav0_path = os.path.join(output_path, "mav0")
        self.cam0_path = os.path.join(self.mav0_path, "cam0")
        self.imu0_path = os.path.join(self.mav0_path, "imu0")
        self.is_closed = False
        
        os.makedirs(os.path.join(self.cam0_path, "data"), exist_ok=True)
        os.makedirs(self.imu0_path, exist_ok=True)
        
        self.cam_timestamps = open(os.path.join(self.cam0_path, "timestamps.txt"), "w")
        self.cam_data = open(os.path.join(self.cam0_path, "data.csv"), "w")
        self.cam_data.write("#timestamp [ns],filename\n")
        
        self.imu_data = open(os.path.join(self.imu0_path, "data.csv"), "w")
        self.imu_data.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
        
        self.cam_start_timestamp = None
        self.last_cam_timestamp = None
        self.frame_count = 0
        self.imu_count = 0

    def handle_camera(self, image):
        """Сохраняет кадр и его timestamp в наносекундах."""
        if self.is_closed:
            return False
        
        try:
            timestamp_ns = time.time_ns()
            if self.cam_start_timestamp is None:
                self.cam_start_timestamp = timestamp_ns
            
            self.last_cam_timestamp = timestamp_ns
            filename = f"{timestamp_ns}.png"

            frame = np.frombuffer(image.raw_data, dtype=np.uint8)
            frame = frame.reshape((image.height, image.width, 4))[:, :, :3]
            cv2.imwrite(os.path.join(self.cam0_path, "data", filename), frame)

            self.cam_timestamps.write(f"{timestamp_ns}\n")
            self.cam_timestamps.flush()
            
            self.cam_data.write(f"{timestamp_ns},{filename}\n")
            self.cam_data.flush()
            
            self.frame_count += 1

        except Exception as e:
            logging.error(f"Error in handle_camera: {e}")
        return True

    def handle_imu(self, imu_data):
        """Обрабатывает данные IMU и записывает их с текущим timestamp."""
        if self.is_closed:
            return False
        
        try:
            timestamp_ns = time.time_ns()
            gx = imu_data.gyroscope.x
            gy = imu_data.gyroscope.y
            gz = imu_data.gyroscope.z
            ax = imu_data.accelerometer.x
            ay = imu_data.accelerometer.y
            az = imu_data.accelerometer.z
            self.imu_data.write(f"{timestamp_ns},{gx},{gy},{gz},{ax},{ay},{az}\n")
            self.imu_data.flush()
            self.imu_count += 1
        except Exception as e:
            logging.error(f"Error in handle_imu: {e}")
        return True

    def close(self):
        """Закрывает файлы и выводит статистику с использованием реальных timestamp."""
        if self.is_closed:
            return
        
        self.cam_timestamps.close()
        self.cam_data.close()
        self.imu_data.close()
        
        if self.cam_start_timestamp and self.last_cam_timestamp:
            duration = (self.last_cam_timestamp - self.cam_start_timestamp) / 1e9
            logging.info(f"Записано {self.frame_count} кадров ({self.frame_count/duration:.1f} Hz)")
            logging.info(f"Записано {self.imu_count} IMU измерений ({self.imu_count/duration:.1f} Hz)")
        else:
            logging.warning("Не было записано ни одного кадра!")
        
        self.is_closed = True

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

def main():
    argparser = argparse.ArgumentParser(description="Запись датасета из CARLA")
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
    recorder = DatasetRecorder(args.output)
    settings.fixed_delta_seconds = SIMULATION_DT
    settings.synchronous_mode = True
    settings.no_rendering_mode = True
    world.apply_settings(settings)

    try:
        recorder = DatasetRecorder(args.output)
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.find(args.vehicle)
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            raise ValueError("Не найдены точки спавна!")
        spawn_point = spawn_points[0]
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        logging.info(f"Создан автомобиль: {vehicle.type_id}")
        
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '752')
        camera_bp.set_attribute('image_size_y', '480')
        camera_bp.set_attribute('fov', '110')
        camera_bp.set_attribute('sensor_tick', str(FRAME_TIME))
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4),
                                           carla.Rotation(pitch=0, yaw=0, roll=0))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
        
        imu_bp = blueprint_library.find('sensor.other.imu')
        imu_bp.set_attribute('sensor_tick', str(SIMULATION_DT))
        imu_transform = carla.Transform(carla.Location(x=0, z=0))
        imu = world.spawn_actor(imu_bp, imu_transform, attach_to=vehicle)
        
        create_sensor_configs(args.output, camera_transform, imu_transform)
        camera.listen(lambda image: recorder.handle_camera(image))
        imu.listen(lambda imu_data: recorder.handle_imu(imu_data))
        
        logging.info("Инициализация IMU: выполняется плавное движение...")
        move_vehicle_for_initialization(world, vehicle, duration=5)
        
        vehicle.set_autopilot(True)
        
        num_ticks = int(args.duration * SIMULATION_HZ)
        
        for _ in range(num_ticks):
            world.tick()
        
        logging.info("Удаление объектов...")
        camera.stop()
        imu.stop()
        time.sleep(0.1)
        
        recorder.close()
        verify_dataset(args.output)
        
        camera.destroy()
        imu.destroy()
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
