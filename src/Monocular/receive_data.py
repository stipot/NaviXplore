#!/usr/bin/env python

"""
Основной скрипт для получения данных от ORB-SLAM3 через порт zmq.
Описание аргументов:
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
"""

import json
import time
import argparse
import logging
import signal
import os
import threading
from typing import List, Optional, Tuple, Callable
from collections import deque
from dataclasses import dataclass
import locale
import cv2
import numpy as np
import zmq
import matplotlib
import matplotlib.pyplot as plt
import pandas as pd


matplotlib.set_loglevel("WARNING")
logging.getLogger("matplotlib").setLevel(logging.WARNING)
logging.getLogger("PIL").setLevel(logging.WARNING)
logging.getLogger("fontTools").setLevel(logging.WARNING)


@dataclass
class KeyPoint:
    """Класс для хранения информации о ключевой точке"""

    image_x: float
    image_y: float
    size: float
    angle: float
    response: float
    octave: int

    x: Optional[float] = None
    y: Optional[float] = None
    z: Optional[float] = None


@dataclass
class CameraPose:
    """Класс для хранения положения и ориентации камеры в мировой системе координат"""

    timestamp: float
    x: float
    y: float
    z: float
    rotation_matrix: Optional[np.ndarray] = None


class ORBDataReceiver:
    """Класс для приема всех данных от ORB-SLAM3: изображений, позиции камеры и точек"""

    def __init__(self, data_port: int, max_buffer_size: int = 10):
        """Инициализация получателя всех данных от ORB-SLAM3"""
        self.data_port = data_port
        self.max_buffer_size = max_buffer_size

        self.frames_buffer = deque(maxlen=max_buffer_size)
        self.timestamps_buffer = deque(maxlen=max_buffer_size)
        self.poses_buffer = deque(maxlen=max_buffer_size)

        self.context = zmq.Context()
        self.data_socket = self.context.socket(zmq.SUB)
        self.data_socket.bind(f"tcp://*:{self.data_port}")
        self.data_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.data_socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1 секунда

        self.running = False
        self.thread = None

        self.callbacks = []

        logging.info("ORBDataReceiver инициализирован на порту %d", data_port)

    def start(self):
        """Запуск приема данных в отдельном потоке"""
        if self.thread is None or not self.thread.is_alive():
            self.running = True
            self.thread = threading.Thread(target=self._receive_data_loop)
            self.thread.daemon = True
            self.thread.start()
            logging.info("Запущен поток приема данных от ORB-SLAM3")

    def stop(self):
        """Остановка приема данных"""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
            logging.info("Поток приема данных остановлен")

        self.data_socket.close()
        self.context.term()
        logging.info("Ресурсы ZMQ освобождены")

    def register_callback(
        self,
        callback: Callable[
            [float, np.ndarray, Optional[CameraPose], List[KeyPoint]], None
        ],
    ):
        """Регистрация функции обратного вызова для обработки новых данных"""
        self.callbacks.append(callback)
        logging.info(
            "Зарегистрирован новый обработчик данных (всего %d)", len(self.callbacks)
        )

    def _decode_image_from_hex(self, hex_string):
        """Декодирование изображения из hex-строки"""
        try:
            binary_data = bytes.fromhex(hex_string)
            img = cv2.imdecode(
                np.frombuffer(binary_data, dtype=np.uint8), cv2.IMREAD_COLOR
            )
            return img
        except Exception as e:
            logging.error("Ошибка декодирования изображения: %s", e)
            return None

    def _receive_data_loop(self):
        """Основной цикл приема всех данных от ORB-SLAM3"""
        while self.running:
            try:
                message = self.data_socket.recv_string()
                try:
                    data = json.loads(message)
                except json.JSONDecodeError as e:
                    logging.error("Ошибка декодирования JSON: %s", e)
                    logging.debug("Полученное сообщение: %s...", message[:100])
                    continue

                timestamp = data.get("timestamp")
                tracking_status = data.get("tracking_status", "UNKNOWN")

                timestamp_str = str(int(timestamp))

                img = None
                if "image_data" in data:
                    img = self._decode_image_from_hex(data["image_data"])
                    if img is None:
                        logging.warning("Не удалось декодировать изображение")

                pose = None
                pose_matrix = None
                if "pose" in data and tracking_status == "OK":
                    try:
                        pose_data = data["pose"]

                        tx = pose_data.get("tx")
                        ty = pose_data.get("ty")
                        tz = pose_data.get("tz")

                        # Матрица преобразования
                        if "pose_matrix" in data:
                            matrix_data = data["pose_matrix"]
                            if len(matrix_data) == 16:  # 4x4
                                pose_matrix = np.array(matrix_data).reshape(4, 4)

                                R = pose_matrix[:3, :3]
                                t = np.array([tx, ty, tz])
                                world_pos = -np.dot(R.T, t)
                                x, y, z = world_pos
                            else:
                                x, y, z = tx, ty, tz
                        else:
                            x, y, z = tx, ty, tz

                        pose = CameraPose(
                            timestamp=timestamp,
                            x=x,
                            y=y,
                            z=z,
                            rotation_matrix=pose_matrix,
                        )
                        self.poses_buffer.append(pose)

                        logging.debug(
                            "Получены данные о позиции: исходные (%.2f, %.2f, %.2f), мировые (%.2f, %.2f, %.2f)",
                            tx,
                            ty,
                            tz,
                            x,
                            y,
                            z,
                        )
                    except Exception as e:
                        logging.error("Ошибка обработки данных о позиции: %s", e)

                points = []
                for point_data in data.get("points", []):
                    try:
                        world_x = point_data.get("world_x")
                        world_y = point_data.get("world_y")
                        world_z = point_data.get("world_z")

                        if (
                            world_x is not None
                            and world_y is not None
                            and world_z is not None
                        ):
                            kp = KeyPoint(
                                image_x=point_data.get("image_x"),
                                image_y=point_data.get("image_y"),
                                size=point_data.get("size"),
                                angle=point_data.get("angle"),
                                response=point_data.get("response"),
                                octave=point_data.get("octave"),
                                x=world_x,
                                y=world_y,
                                z=world_z,
                            )
                            points.append(kp)
                    except Exception as e:
                        logging.error("Ошибка обработки точки: %s", e)

                if img is not None and timestamp is not None:
                    self.frames_buffer.append((timestamp, img, points))
                    self.timestamps_buffer.append(timestamp)

                    for callback in self.callbacks:
                        try:
                            callback(timestamp, img, pose, points)
                        except Exception as e:
                            logging.error("Ошибка в обработчике: %s", e)

                logging.info(
                    "Получены данные: timestamp=%s, статус=%s, точек=%d",
                    timestamp_str,
                    tracking_status,
                    len(points),
                )
                if pose:
                    logging.info(
                        "Позиция камеры (мировая): x=%.2f, y=%.2f (высота), z=%.2f",
                        pose.x,
                        pose.y,
                        pose.z,
                    )

            except zmq.Again:
                continue
            except zmq.ZMQError as e:
                logging.error("Ошибка ZMQ: %s", e)
                break
            except Exception as e:
                logging.error("Непредвиденная ошибка: %s", e)
                logging.exception(e)

    def get_latest_frame(self) -> Optional[Tuple[float, np.ndarray, List[KeyPoint]]]:
        """Получение последнего принятого кадра с точками"""
        if self.frames_buffer:
            return self.frames_buffer[-1]
        return None

    def get_latest_pose(self) -> Optional[CameraPose]:
        """Получение последней позиции камеры"""
        if self.poses_buffer:
            return self.poses_buffer[-1]
        return None

    def get_frame_by_timestamp(
        self, timestamp: float, tolerance: float = 0.05
    ) -> Optional[Tuple[float, np.ndarray, List[KeyPoint]]]:
        """Получение кадра по временной метке с указанной погрешностью"""
        for ts, img, points in self.frames_buffer:
            if abs(ts - timestamp) < tolerance:
                return (ts, img, points)
        return None

    def get_pose_by_timestamp(
        self, timestamp: float, tolerance: float = 0.05
    ) -> Optional[CameraPose]:
        """Получение позиции камеры по временной метке с указанной погрешностью"""
        for pose in self.poses_buffer:
            if abs(pose.timestamp - timestamp) < tolerance:
                return pose
        return None


class DebugVisualizer:
    """Класс для отладочной визуализации данных ORB-SLAM3"""

    def __init__(self, output_dir: str = "debug_output"):
        """
        Инициализация отладочного визуализатора

        Args:
            output_dir: Директория для сохранения отладочных данных
        """
        self.output_dir = output_dir
        self.frame_counter = 0
        self.points_log = []
        self.last_frame = None
        self.last_points = None
        self.last_pose = None
        self.trajectory = []
        self.all_points = set()

        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            logging.info("Создана директория для отладочных данных: %s", output_dir)

        self.points_log_file = os.path.join(output_dir, "points_log.csv")
        with open(self.points_log_file, "w", encoding="utf-8") as f:
            f.write("timestamp,point_id,image_x,image_y,x,y,z\n")

        self.trajectory_log_file = os.path.join(output_dir, "trajectory.csv")
        with open(self.trajectory_log_file, "w", encoding="utf-8") as f:
            f.write("timestamp,x,y,z\n")

        logging.info("Отладочный визуализатор инициализирован, вывод в %s", output_dir)

    def process_frame(
        self,
        timestamp: float,
        image: np.ndarray,
        pose: Optional[CameraPose],
        points: List[KeyPoint],
    ):
        """
        Обработка нового кадра и точек с сохранением отладочной информации

        Args:
            timestamp: Временная метка кадра
            image: Изображение
            pose: Позиция и ориентация камеры
            points: Список ключевых точек
        """
        if image is None:
            logging.warning("Получено пустое изображение для отладки")
            return

        self.last_frame = image.copy()
        self.last_points = points
        self.last_pose = pose
        if pose:
            self.trajectory.append(pose)

            with open(self.trajectory_log_file, "a", encoding="utf-8") as f:
                f.write(f"{str(int(timestamp))},{pose.x},{pose.y},{pose.z}\n")

        for point in points:
            if point.x is not None:
                point_key = (point.x, point.y, point.z)
                self.all_points.add(point_key)

        self.frame_counter += 1
        timestamp_str = str(int(timestamp))

        frame_filename = os.path.join(
            self.output_dir, f"frame_{self.frame_counter:06d}_{timestamp_str}.jpg"
        )

        debug_frame = self.last_frame.copy()

        self._save_points_data(timestamp, points)
        self._visualize_points_on_frame(debug_frame, points)

        pose_text = "No data about position"
        if pose:
            pose_text = f"Pos: {pose.x:.2f}, {pose.y:.2f}, {pose.z:.2f}"

        cv2.putText(
            debug_frame,
            f"Frame: {self.frame_counter}, TS: {timestamp_str}, Points: {len(points)}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
        )
        cv2.putText(
            debug_frame,
            pose_text,
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        cv2.imwrite(frame_filename, debug_frame)

        if self.frame_counter % 5 == 0:
            cv2.imshow("ORB-SLAM3 Debug", debug_frame)
            cv2.waitKey(1)

        logging.info(
            "Сохранен отладочный кадр %d, точек: %d", self.frame_counter, len(points)
        )
        self._print_points_info(timestamp, points, max_points=5)

    def _save_points_data(self, timestamp: float, points: List[KeyPoint]):
        """
        Сохраняет информацию о точках в CSV-файл

        Args:
            timestamp: Временная метка кадра
            points: Список ключевых точек
        """
        with open(self.points_log_file, "a", encoding="utf-8") as f:
            for i, point in enumerate(points):
                if point.x is not None:
                    f.write(
                        f"{str(int(timestamp))},{i},{point.image_x},{point.image_y},"
                        f"{point.x},{point.y},{point.z}\n"
                    )

    def _visualize_points_on_frame(self, frame: np.ndarray, points: List[KeyPoint]):
        """
        Отрисовка точек на изображении с цветовой кодировкой расстояния до камеры

        Args:
            frame: Изображение для отрисовки
            points: Список ключевых точек
        """
        camera_pos = np.array([0, 0, 0])
        if self.last_pose:
            camera_pos = np.array(
                [self.last_pose.x, self.last_pose.y, self.last_pose.z]
            )

        distances = []
        valid_points = []

        for point in points:
            if point.x is not None and point.y is not None and point.z is not None:
                point_pos = np.array([point.x, point.y, point.z])
                distance = np.linalg.norm(point_pos - camera_pos)

                distances.append(distance)
                valid_points.append((point, distance))

        if not distances:
            return

        min_distance = min(distances)
        max_distance = max(distances)
        distance_range = max(0.001, max_distance - min_distance)

        for point, distance in valid_points:
            try:
                x, y = int(point.image_x), int(point.image_y)

                if 0 <= x < frame.shape[1] and 0 <= y < frame.shape[0]:
                    normalized_distance = (distance - min_distance) / distance_range

                    red = int(255 * (1 - normalized_distance))
                    blue = int(255 * normalized_distance)

                    color = (blue, 0, red)

                    cv2.circle(frame, (x, y), 4, color, -1)

                    cv2.putText(
                        frame,
                        f"{distances.index(distance)}",
                        (x + 5, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        (255, 255, 255),
                        1,
                    )

                    cv2.line(frame, (x, y), (x, y - 15), color, 1)
                    cv2.circle(frame, (x, y - 15), 2, color, -1)

            except Exception as e:
                logging.error("Ошибка при отрисовке точки: %s", e)

    def _print_points_info(
        self, timestamp: float, points: List[KeyPoint], max_points: int = 5
    ):
        """
        Вывод подробной информации о точках в консоль

        Args:
            timestamp: Временная метка кадра
            points: Список ключевых точек
            max_points: Максимальное количество точек для вывода
        """
        timestamp_str = str(int(timestamp))

        logging.info(
            "Отладочный вывод точек для кадра %d, TS: %s",
            self.frame_counter,
            timestamp_str,
        )

        valid_points = [p for p in points if p.x is not None]

        if not valid_points:
            logging.info("Нет точек с 3D координатами")
            return

        logging.info(
            "Всего точек с 3D координатами: %d/%d", len(valid_points), len(points)
        )

        logging.info("Ближайшие точки:")
        sorted_points = sorted(
            valid_points,
            key=lambda p: p.z if p.z is not None else float("inf"),
        )

        for i, point in enumerate(sorted_points[:max_points]):
            logging.info(
                "Точка %d: Экран: (%.1f, %.1f), Мир: (%.2f, %.2f, %.2f), Размер: %.1f, Угол: %.1f°",
                i,
                point.image_x,
                point.image_y,
                point.x,
                point.y,
                point.z,
                point.size,
                point.angle,
            )

        depths = [p.z for p in valid_points if p.z is not None]
        if depths:
            logging.info(
                "Статистика глубины - Мин: %.2f, Макс: %.2f, Среднее: %.2f",
                min(depths),
                max(depths),
                sum(depths) / len(depths),
            )

        logging.info("======================================================")

    def generate_report(self):
        """Генерация HTML-отчета с отладочной информацией"""
        report_path = os.path.join(self.output_dir, "debug_report.html")

        with open(report_path, "w", encoding="utf-8") as f:
            f.write(
                """
            <!DOCTYPE html>
            <html>
            <head>
                <meta charset="utf-8">
                <title>ORB-SLAM3 Debug Report</title>
                <style>
                    body { font-family: Arial, sans-serif; margin: 20px; }
                    h1 { color: #333; }
                    .frame-container { margin-bottom: 30px; border-bottom: 1px solid #ccc; padding-bottom: 20px; }
                    img { max-width: 800px; border: 1px solid #ddd; }
                    .stats { font-family: monospace; white-space: pre; }
                </style>
            </head>
            <body>
                <h1>ORB-SLAM3 Debug Report</h1>
            """
            )

            image_files = sorted(
                [f for f in os.listdir(self.output_dir) if f.endswith(".jpg")]
            )

            for img_file in image_files:
                frame_id = img_file.split("_")[1]
                timestamp = img_file.split("_")[2].replace(".jpg", "")

                f.write(
                    f"""
                <div class="frame-container">
                    <h2>Frame {frame_id}, Timestamp: {timestamp}</h2>
                    <img src="{img_file}" alt="Frame {frame_id}">
                </div>
                """
                )

            f.write(
                """
            </body>
            </html>
            """
            )

        if len(self.trajectory) > 0:
            fig_path = os.path.join(self.output_dir, "trajectory.png")
            self._plot_trajectory(fig_path)

            points_3d_path = os.path.join(self.output_dir, "points_3d.png")
            self._plot_points_3d(points_3d_path)

            with open(report_path, "a", encoding="utf-8") as f:
                f.write(
                    """
                <div class="frame-container">
                    <h2>Траектория камеры</h2>
                    <img src="trajectory.png" alt="Camera Trajectory">
                </div>
                <div class="frame-container">
                    <h2>3D карта точек</h2>
                    <img src="points_3d.png" alt="3D Map Points">
                </div>
                """
                )

        return report_path

    def _plot_trajectory(self, output_path):
        """Создает и сохраняет график траектории камеры с учетом системы координат ORB-SLAM3"""
        if len(self.trajectory) < 2:
            return

        try:
            fig = plt.figure(figsize=(12, 10))

            positions = []
            for pose in self.trajectory:
                positions.append([pose.x, pose.y, pose.z])

            if positions:
                positions = np.array(positions)

                ax1 = fig.add_subplot(221)
                ax1.plot(positions[:, 0], positions[:, 2], "r-")
                ax1.set_title("Проекция X-Z (вид сверху)")
                ax1.set_xlabel("X")
                ax1.set_ylabel("Z")

                ax2 = fig.add_subplot(222)
                ax2.plot(positions[:, 0], positions[:, 1], "g-")
                ax2.set_title("Проекция X-Y (вид спереди)")
                ax2.set_xlabel("X")
                ax2.set_ylabel("Y (высота)")

                ax3 = fig.add_subplot(223)
                ax3.plot(positions[:, 2], positions[:, 1], "b-")
                ax3.set_title("Проекция Z-Y (вид сбоку)")
                ax3.set_xlabel("Z")
                ax3.set_ylabel("Y (высота)")

                ax4 = fig.add_subplot(224, projection="3d")
                ax4.plot3D(positions[:, 0], positions[:, 2], positions[:, 1])
                ax4.set_title("3D траектория")
                ax4.set_xlabel("X")
                ax4.set_ylabel("Z")
                ax4.set_zlabel("Y")

            plt.tight_layout()
            plt.savefig(output_path)
            plt.close()
        except Exception as e:
            logging.error("Ошибка при построении траектории: %s", e)

    def _plot_points_3d(self, output_path):
        """Создает и сохраняет 3D график всех точек с путем камеры"""
        if not self.trajectory or not self.all_points:
            return

        try:
            fig = plt.figure(figsize=(12, 10))
            ax = fig.add_subplot(111, projection="3d")

            positions = np.array([[pose.x, pose.y, pose.z] for pose in self.trajectory])
            if len(positions) > 0:
                ax.plot3D(
                    positions[:, 0],
                    positions[:, 2],
                    positions[:, 1],
                    "r-",
                    linewidth=2,
                    label="Camera Path",
                )

                last_camera_pos = positions[-1]
            else:
                last_camera_pos = np.array([0, 0, 0])

            if self.all_points:
                point_positions = np.array(list(self.all_points))

                max_distance = 7.0
                distances = np.sqrt(
                    np.sum((point_positions - last_camera_pos) ** 2, axis=1)
                )
                filtered_indices = distances <= max_distance
                filtered_points = point_positions[filtered_indices]

                if len(filtered_points) > 0:
                    ax.scatter(
                        filtered_points[:, 0],
                        filtered_points[:, 2],
                        filtered_points[:, 1],
                        c="b",
                        marker=".",
                        s=1,
                        alpha=0.5,
                        label="Map Points",
                    )

                logging.info(
                    "Отфильтровано точек: %d/%d (в радиусе %f от камеры)",
                    len(filtered_points),
                    len(point_positions),
                    max_distance,
                )

            if len(positions) > 0:
                ax.scatter(
                    [last_camera_pos[0]],
                    [last_camera_pos[2]],
                    [last_camera_pos[1]],
                    c="g",
                    marker="o",
                    s=50,
                    label="Current Camera",
                )

            ax.set_title("3D Map with Camera Path")
            ax.set_xlabel("X")
            ax.set_ylabel("Z")
            ax.set_zlabel("Y")
            ax.legend()

            plt.tight_layout()
            plt.savefig(output_path)
            plt.close()
        except Exception as e:
            logging.error("Ошибка при построении 3D карты точек: %s", e)


def save_collected_data(output_dir, frames_data):
    """Сохраняет собранные данные в CSV-файлы и изображения внутри указанной директории"""
    if not frames_data:
        logging.warning("Нет данных для сохранения")
        return

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        logging.info("Создана директория для сохранения данных: %s", output_dir)

    images_dir = os.path.join(output_dir, "images")
    if not os.path.exists(images_dir):
        os.makedirs(images_dir)
        logging.info("Создана директория для сохранения изображений: %s", images_dir)

    frames_file = os.path.join(output_dir, "collected_data.csv")
    points_file = os.path.join(output_dir, "collected_data_points.csv")

    frame_data_list = []
    for idx, (ts, img, pose, points) in enumerate(frames_data):
        image_filename = f"frame_{idx:06d}_ts_{int(ts)}.jpg"
        image_path = os.path.join(images_dir, image_filename)
        cv2.imwrite(image_path, img)

        data_row = {
            "frame_idx": idx,
            "timestamp": int(ts),
            "num_points": len(points),
            "image_filename": image_filename,
        }

        if pose:
            data_row["camera_x"] = pose.x
            data_row["camera_y"] = pose.y
            data_row["camera_z"] = pose.z

            if pose.rotation_matrix is not None:
                for i in range(4):
                    for j in range(4):
                        data_row[f"r{i}{j}"] = pose.rotation_matrix[i, j]

        frame_data_list.append(data_row)

    df_frames = pd.DataFrame(frame_data_list)
    df_frames.to_csv(frames_file, index=False)

    points_data_list = []
    for idx, (ts, img, pose, points) in enumerate(frames_data):
        for p_idx, point in enumerate(points):
            point_data = {
                "frame_idx": idx,
                "timestamp": int(ts),
                "point_idx": p_idx,
                "image_x": point.image_x,
                "image_y": point.image_y,
                "size": point.size,
                "angle": point.angle,
                "response": point.response,
                "octave": point.octave,
            }

            if point.x is not None:
                point_data["world_x"] = point.x
                point_data["world_y"] = point.y
                point_data["world_z"] = point.z

            points_data_list.append(point_data)

    df_points = pd.DataFrame(points_data_list)
    df_points.to_csv(points_file, index=False)

    logging.info("Данные о кадрах сохранены в %s", frames_file)
    logging.info("Данные о точках сохранены в %s", points_file)
    logging.info("Изображения сохранены в директории %s", images_dir)

    logging.info("Собрано кадров: %d", len(frames_data))
    total_points = sum(len(points) for _, _, _, points in frames_data)
    total_3d_points = sum(
        len([p for p in points if p.x is not None]) for _, _, _, points in frames_data
    )
    logging.info(
        "Всего точек: %d, из них с 3D координатами: %d", total_points, total_3d_points
    )

    return frames_file, points_file


def ensure_unique_directory(base_dir: str) -> str:
    """
    Создает уникальную директорию с добавлением индекса, если требуется

    Args:
        base_dir: Базовое имя директории

    Returns:
        Уникальное имя директории (с индексом, если требуется)
    """
    if os.path.exists(base_dir) and os.path.isdir(base_dir) and os.listdir(base_dir):
        original_dir = base_dir
        i = 1
        while os.path.exists(f"{original_dir}_{i}"):
            i += 1
        new_dir = f"{original_dir}_{i}"
        logging.warning(
            "Директория '%s' уже существует и содержит файлы. Будет использована новая директория '%s'",
            base_dir,
            new_dir,
        )
        return new_dir
    return base_dir


def main():
    parser = argparse.ArgumentParser(description="Прием данных от ORB-SLAM3")
    parser.add_argument(
        "--data-port",
        type=int,
        default=5557,
        help="""Порт для приема всех данных (по умолчанию 5557)""",
    )
    parser.add_argument("--debug", action="store_true", help="Включить режим отладки")
    parser.add_argument(
        "--debug-output",
        type=str,
        default="debug_output",
        help="""Директория для сохранения отладочных данных""",
    )
    parser.add_argument(
        "--collect-data",
        action="store_true",
        help="""Собрать и сохранить все данные с информацией из ORB-SLAM3""",
    )
    parser.add_argument(
        "--collection-data-output",
        type=str,
        default="collected_data",
        help="""Директория для сохранения собранных данных. Используется только
        при активном аргументе collect-data""",
    )
    parser.add_argument(
        "--collection-max-frames",
        type=int,
        default=100,
        help="""Максимальное количество кадров для сбора.
        -1 для сбора всех кадров. Используется только
        при активном аргументе collect-data""",
    )
    parser.add_argument(
        "--collection-interval",
        type=float,
        default=1.0,
        help="""Интервал между собираемыми данными.
        0 для сбора всех кадров. Используется только
        при активном аргументе collect-data""",
    )
    args = parser.parse_args()

    locale.setlocale(locale.LC_ALL, "")

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(
        format="%(asctime)s - %(levelname)s: %(message)s", level=log_level
    )

    running = True
    debug_mode = args.debug
    collect_data_enabled = args.collect_data
    data_output_dir = args.collection_data_output
    collected_frames = []
    max_frames_to_collect = args.collection_max_frames
    collection_start_time = None
    collection_interval = args.collection_interval

    # Добавляем лог и задержку для режима полного сбора
    if collect_data_enabled and max_frames_to_collect == -1:
        logging.info("Активирован режим полного сбора данных. Сбор будет продолжаться до ручного завершения (Ctrl+C)")
        logging.info("Подождите 2 секунды перед началом...")
        time.sleep(2)  # Задержка в 2 секунды

    def signal_handler(_sig, _frame):
        nonlocal running
        if collect_data_enabled and max_frames_to_collect == -1:
            logging.info("Получен сигнал завершения, сбор данных будет остановлен и результаты сохранены...")
        else:
            logging.info("Получен сигнал завершения...")
        running = False

    def example_processor(timestamp, image, pose, points):
        nonlocal collected_frames, collection_start_time, running

        timestamp_str = str(int(timestamp))
        logging.info("Обработка кадра %s с %d точками", timestamp_str, len(points))

        if (
            collect_data_enabled
            and (
                (len(collected_frames) < max_frames_to_collect)
                or max_frames_to_collect == -1
            )
            and pose is not None
        ):
            current_time = time.time()

            if collection_start_time is None:
                collection_start_time = current_time
                collected_frames.append((timestamp, image.copy(), pose, points))
                if max_frames_to_collect == -1:
                    logging.info("Собран кадр %d (режим полного сбора)", len(collected_frames))
                else:
                    logging.info("Собран кадр 1/%d (статус: OK)", max_frames_to_collect)
            elif collection_interval == 0 or (
                current_time - collection_start_time
                >= len(collected_frames) * collection_interval
            ):
                collected_frames.append((timestamp, image.copy(), pose, points))
                if max_frames_to_collect == -1:
                    logging.info("Собран кадр %d (режим полного сбора)", len(collected_frames))
                else:
                    logging.info(
                        "Собран кадр %d/%d (статус: OK)",
                        len(collected_frames),
                        max_frames_to_collect,
                    )

                if len(collected_frames) >= max_frames_to_collect and max_frames_to_collect != -1:
                    output_file, points_file = save_collected_data(
                        data_output_dir, collected_frames
                    )
                    logging.info(
                        "Сбор данных завершен. Данные сохранены в %s и %s",
                        output_file,
                        points_file,
                    )

                    if collect_data_enabled:
                        logging.info(
                            "Все необходимые данные собраны. Программа будет остановлена."
                        )
                        running = False
        elif collect_data_enabled and pose is None:
            logging.info("Кадр пропущен для сбора данных: статус трекинга не OK")

        if pose:
            logging.info(
                "Позиция камеры (мировая): x=%.2f, y=%.2f (высота), z=%.2f",
                pose.x,
                pose.y,
                pose.z,
            )

            if pose.rotation_matrix is not None and debug_mode:
                logging.debug("Матрица поворота камеры:")
                for i in range(4):
                    row = " ".join(
                        [f"{pose.rotation_matrix[i,j]:.4f}" for j in range(4)]
                    )
                    logging.debug("    [%s]", row)

        valid_points = [p for p in points if p.x is not None]
        if valid_points:
            logging.info(
                "Точек с 3D координатами: %d/%d", len(valid_points), len(points)
            )

            sorted_points = sorted(
                valid_points,
                key=lambda p: p.z if p.z is not None else float("inf"),
            )
            for i, point in enumerate(sorted_points[:3]):
                logging.debug(
                    "Точка %d: Экран (%.1f, %.1f), Мир (%.2f, %.2f, %.2f)",
                    i,
                    point.image_x,
                    point.image_y,
                    point.x,
                    point.y,
                    point.z,
                )

    debug_dir = args.debug_output
    if args.debug:
        debug_dir = ensure_unique_directory(debug_dir)

    if args.collect_data:
        data_output_dir = ensure_unique_directory(data_output_dir)

    signal.signal(signal.SIGINT, signal_handler)

    receiver = ORBDataReceiver(data_port=args.data_port)

    debug_visualizer = None
    if args.debug:
        debug_visualizer = DebugVisualizer(output_dir=debug_dir)
        receiver.register_callback(debug_visualizer.process_frame)
        logging.info("Запущен отладочный режим, вывод в %s", debug_dir)

    receiver.register_callback(example_processor)

    try:
        receiver.start()

        while running:
            time.sleep(0.1)

    except KeyboardInterrupt:
        logging.info("Прервано пользователем")
    finally:
        receiver.stop()

        if debug_visualizer:
            report_path = debug_visualizer.generate_report()
            logging.info("Сгенерирован отчет: %s", report_path)

        if args.collect_data and collected_frames:
            output_file, points_file = save_collected_data(
                data_output_dir, collected_frames
            )
            
            if max_frames_to_collect == -1:
                logging.info(
                    "Программа завершается. Собрано %d кадров в режиме полного сбора. "
                    "Данные сохранены в %s и %s",
                    len(collected_frames),
                    output_file,
                    points_file,
                )
            else:
                logging.info(
                    "Программа завершается. Собрано %d/%d кадров. "
                    "Данные сохранены в %s и %s",
                    len(collected_frames),
                    max_frames_to_collect,
                    output_file,
                    points_file,
                )

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
