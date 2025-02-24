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
from collections import deque
from scipy.interpolate import interp1d

# Частоты для симуляции и записи
SIMULATION_HZ = 50     # Частота симуляции CARLA
TARGET_FPS = 20       # Частота камеры
IMU_FREQUENCY = 200   # Желаемая частота IMU (будет эмулироваться)
FRAME_TIME = 1.0 / TARGET_FPS
IMU_FRAME_TIME = 1.0 / IMU_FREQUENCY
SIMULATION_DT = 1.0 / SIMULATION_HZ

class DatasetRecorder:
    def __init__(self, output_path):
        self.base_path = output_path
        self.mav0_path = os.path.join(output_path, "mav0")
        self.cam0_path = os.path.join(self.mav0_path, "cam0")
        self.imu0_path = os.path.join(self.mav0_path, "imu0")
        
        os.makedirs(os.path.join(self.cam0_path, "data"), exist_ok=True)
        os.makedirs(self.imu0_path, exist_ok=True)
        
        self.cam_timestamps = open(os.path.join(self.cam0_path, "timestamps.txt"), "w")
        self.imu_data = open(os.path.join(self.imu0_path, "data.csv"), "w")
        self.imu_data.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
        
        self.last_imu_reading = None
        self.next_imu_reading = None
        self.last_frame_time = 0
        self.start_time = None
        self.frame_count = 0
        self.imu_count = 0

    def interpolate_imu(self, t1, t2, imu1, imu2):
        """Интерполирует данные IMU между двумя измерениями."""
        num_points = max(2, int((t2 - t1) * IMU_FREQUENCY))
        times = np.linspace(t1, t2, num_points)[:-1]  # Исключаем последнюю точку
        
        # Создаем массивы для интерполяции
        t = np.array([t1, t2])
        gyro1, acc1 = imu1.gyroscope, imu1.accelerometer
        gyro2, acc2 = imu2.gyroscope, imu2.accelerometer
        
        # Интерполируем каждую ось
        for time in times:
            alpha = (time - t1) / (t2 - t1)
            # Линейная интерполяция
            gx = gyro1.x + alpha * (gyro2.x - gyro1.x)
            gy = gyro1.y + alpha * (gyro2.y - gyro1.y)
            gz = gyro1.z + alpha * (gyro2.z - gyro1.z)
            ax = acc1.x + alpha * (acc2.x - acc1.x)
            ay = acc1.y + alpha * (acc2.y - acc1.y)
            az = acc1.z + alpha * (acc2.z - acc1.z)
            
            # Добавляем небольшой случайный шум для реалистичности
            noise_scale = 0.01
            gx += np.random.normal(0, noise_scale * abs(gx) if abs(gx) > 0.001 else noise_scale)
            gy += np.random.normal(0, noise_scale * abs(gy) if abs(gy) > 0.001 else noise_scale)
            gz += np.random.normal(0, noise_scale * abs(gz) if abs(gz) > 0.001 else noise_scale)
            ax += np.random.normal(0, noise_scale * abs(ax) if abs(ax) > 0.001 else noise_scale)
            ay += np.random.normal(0, noise_scale * abs(ay) if abs(ay) > 0.001 else noise_scale)
            az += np.random.normal(0, noise_scale * abs(az) if abs(az) > 0.001 else noise_scale)
            
            timestamp_ns = int(time * 1e9)
            self.imu_data.write(f"{timestamp_ns},{gx},{gy},{gz},{ax},{ay},{az}\n")
            self.imu_count += 1

    def handle_camera(self, image):
        """Сохраняет кадр и его timestamp."""
        if self.start_time is None:
            self.start_time = image.timestamp
            
        current_time = image.timestamp
        frame = np.frombuffer(image.raw_data, dtype=np.uint8)
        frame = frame.reshape((image.height, image.width, 4))[:, :, :3]
        
        timestamp_ns = int(current_time * 1e9)
        filename = f"{timestamp_ns}.png"
        
        cv2.imwrite(os.path.join(self.cam0_path, "data", filename), frame)
        self.cam_timestamps.write(f"{timestamp_ns}\n")
        self.cam_timestamps.flush()
        
        self.last_frame_time = current_time
        self.frame_count += 1
        return True

    def handle_imu(self, imu_data):
        """Обрабатывает и интерполирует данные IMU."""
        if self.last_imu_reading is None:
            self.last_imu_reading = (imu_data.timestamp, imu_data)
            return True
            
        self.interpolate_imu(
            self.last_imu_reading[0],
            imu_data.timestamp,
            self.last_imu_reading[1],
            imu_data
        )
        
        self.last_imu_reading = (imu_data.timestamp, imu_data)
        return True

    def close(self):
        """Закрывает файлы и выводит статистику."""
        self.cam_timestamps.close()
        self.imu_data.close()
        
        if self.start_time is not None:
            duration = self.last_frame_time - self.start_time
            logging.info(f"Записано {self.frame_count} кадров ({self.frame_count/duration:.1f} Hz)")
            logging.info(f"Записано {self.imu_count} IMU измерений ({self.imu_count/duration:.1f} Hz)")
        else:
            logging.warning("Не было записано ни одного кадра!")

def main():
    argparser = argparse.ArgumentParser(description="Запись датасета из CARLA")
    argparser.add_argument('--output', type=str, required=True, help='Путь для сохранения датасета')
    argparser.add_argument('--duration', type=int, default=60, help='Длительность записи в секундах')
    argparser.add_argument('--vehicle', default='vehicle.dodge.charger', help='Модель автомобиля')
    argparser.add_argument('--map', default='Town01', help='Карта CARLA (без пути)')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    
    # Получаем список доступных карт
    available_maps = client.get_available_maps()
    logging.info(f"Доступные карты: {available_maps}")
    
    # Ищем нужную карту
    map_name = args.map
    full_map_path = None
    for m in available_maps:
        if map_name in m:
            full_map_path = m
            break
    
    if full_map_path is None:
        raise ValueError(f"Карта '{map_name}' не найдена. Доступные карты: {available_maps}")
    
    # Загружаем карту с полным путем
    logging.info(f"Загрузка карты {full_map_path}...")
    world = client.load_world(full_map_path)
    
    # Настройки для максимальной производительности
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = SIMULATION_DT
    settings.no_rendering_mode = True
    world.apply_settings(settings)

    try:
        recorder = DatasetRecorder(args.output)
        
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.find(args.vehicle)
        
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            raise ValueError("Не найдены точки спавна!")
        spawn_point = spawn_points[0]  # Используем первую точку для стабильности
        
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        logging.info(f"Создан автомобиль: {vehicle.type_id}")
        
        # Настраиваем камеру
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '752')
        camera_bp.set_attribute('image_size_y', '480')
        camera_bp.set_attribute('fov', '90')
        camera_bp.set_attribute('sensor_tick', str(FRAME_TIME))
        
        camera_transform = carla.Transform(
            carla.Location(x=0.0, z=1.0),
            carla.Rotation(pitch=0, yaw=0, roll=0)
        )
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
        
        # Настраиваем IMU
        imu_bp = blueprint_library.find('sensor.other.imu')
        imu_bp.set_attribute('sensor_tick', str(SIMULATION_DT))
        imu_transform = carla.Transform(carla.Location(x=0, z=0))
        imu = world.spawn_actor(imu_bp, imu_transform, attach_to=vehicle)
        
        camera.listen(lambda image: recorder.handle_camera(image))
        imu.listen(lambda imu_data: recorder.handle_imu(imu_data))
        
        # Движение по траектории восьмерки
        start_time = time.time()
        while (time.time() - start_time) < args.duration:
            current_time = time.time() - start_time
            
            # Параметрическая кривая в форме восьмерки
            t = current_time * 0.5
            x = math.cos(t) * 10
            y = math.sin(2*t) * 5
            yaw = math.degrees(math.atan2(math.cos(2*t), -math.sin(t)))
            
            vehicle.set_transform(carla.Transform(
                carla.Location(x=spawn_point.location.x + x, y=spawn_point.location.y + y, z=spawn_point.location.z),
                carla.Rotation(pitch=0, yaw=yaw, roll=0)
            ))
            
            world.tick()
        
        recorder.close()
        
        logging.info("Удаление акторов...")
        camera.stop()
        imu.stop()
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
