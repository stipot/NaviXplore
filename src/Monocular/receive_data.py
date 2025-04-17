#!/usr/bin/env python

import zmq
import json
import time
import argparse
import logging
import signal
import os
import sys
import cv2
import numpy as np
import threading
from typing import List, Dict, Any, Optional, Tuple, Callable
from collections import deque
from dataclasses import dataclass


@dataclass
class KeyPoint:
    """Класс для хранения информации о ключевой точке"""

    image_x: float
    image_y: float
    size: float
    angle: float
    response: float
    octave: int

    world_x: Optional[float] = None
    world_y: Optional[float] = None
    world_z: Optional[float] = None


class ORBFeatureReceiver:
    """Класс для приема и обработки данных ORB-SLAM3"""

    def __init__(self, features_port: int, max_buffer_size: int = 10):
        """Инициализация получателя данных о ключевых точках"""
        self.features_port = features_port
        self.max_buffer_size = max_buffer_size

        self.frames_buffer = deque(maxlen=max_buffer_size)
        self.timestamps_buffer = deque(maxlen=max_buffer_size)

        self.context = zmq.Context()
        self.features_socket = self.context.socket(zmq.SUB)
        self.features_socket.bind(f"tcp://*:{self.features_port}")
        self.features_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.features_socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1 секунда

        self.running = False
        self.thread = None

        self.feature_callbacks = []

        logging.info(f"ORBFeatureReceiver инициализирован на порту {features_port}")

    def start(self):
        """Запуск приема данных в отдельном потоке"""
        if self.thread is None or not self.thread.is_alive():
            self.running = True
            self.thread = threading.Thread(target=self._receive_features_loop)
            self.thread.daemon = True
            self.thread.start()
            logging.info("Запущен поток приема данных о ключевых точках")

    def stop(self):
        """Остановка приема данных"""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
            logging.info("Поток приема данных остановлен")

        self.features_socket.close()
        self.context.term()
        logging.info("Ресурсы ZMQ освобождены")

    def register_callback(
        self, callback: Callable[[float, np.ndarray, List[KeyPoint]], None]
    ):
        """Регистрация функции обратного вызова для обработки новых данных"""
        self.feature_callbacks.append(callback)
        logging.info(
            f"Зарегистрирован новый обработчик данных (всего {len(self.feature_callbacks)})"
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
            logging.error(f"Ошибка декодирования изображения: {e}")
            return None

    def _receive_features_loop(self):
        """Основной цикл приема данных о ключевых точках"""
        while self.running:
            try:
                message = self.features_socket.recv_string()
                try:
                    data = json.loads(message)
                except json.JSONDecodeError as e:
                    logging.error(f"Ошибка декодирования JSON: {e}")
                    logging.debug(f"Полученное сообщение: {message[:100]}...")
                    continue

                timestamp = data.get("timestamp")
                num_points = data.get("num_points", 0)

                img = None
                if "image_data" in data:
                    img = self._decode_image_from_hex(data["image_data"])
                    if img is None:
                        logging.warning("Не удалось декодировать изображение")

                points = []
                for point_data in data.get("points", []):
                    try:
                        kp = KeyPoint(
                            image_x=point_data.get("image_x"),
                            image_y=point_data.get("image_y"),
                            size=point_data.get("size"),
                            angle=point_data.get("angle"),
                            response=point_data.get("response"),
                            octave=point_data.get("octave"),
                            world_x=point_data.get("world_x"),
                            world_y=point_data.get("world_y"),
                            world_z=point_data.get("world_z"),
                        )
                        points.append(kp)
                    except Exception as e:
                        logging.error(f"Ошибка обработки точки: {e}")

                if img is not None and timestamp is not None:
                    self.frames_buffer.append((timestamp, img, points))
                    self.timestamps_buffer.append(timestamp)

                    for callback in self.feature_callbacks:
                        try:
                            callback(timestamp, img, points)
                        except Exception as e:
                            logging.error(f"Ошибка в обработчике: {e}")

                logging.info(
                    f"Получены данные: timestamp={timestamp}, точек={num_points}"
                )

            except zmq.Again:
                continue
            except zmq.ZMQError as e:
                logging.error(f"Ошибка ZMQ: {e}")
                break
            except Exception as e:
                logging.error(f"Непредвиденная ошибка: {e}")
                logging.exception(e)

    def get_latest_frame(self) -> Optional[Tuple[float, np.ndarray, List[KeyPoint]]]:
        """Получение последнего принятого кадра с точками"""
        if self.frames_buffer:
            return self.frames_buffer[-1]
        return None

    def get_frame_by_timestamp(
        self, timestamp: float, tolerance: float = 0.05
    ) -> Optional[Tuple[float, np.ndarray, List[KeyPoint]]]:
        """Получение кадра по временной метке с указанной погрешностью"""
        for ts, img, points in self.frames_buffer:
            if abs(ts - timestamp) < tolerance:
                return (ts, img, points)
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

        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            logging.info(f"Создана директория для отладочных данных: {output_dir}")

        self.points_log_file = os.path.join(output_dir, "points_log.csv")
        with open(self.points_log_file, "w") as f:
            f.write("timestamp,point_id,image_x,image_y,world_x,world_y,world_z\n")

        logging.info(f"Отладочный визуализатор инициализирован, вывод в {output_dir}")

    def process_frame(
        self, timestamp: float, image: np.ndarray, points: List[KeyPoint]
    ):
        """
        Обработка нового кадра и точек с сохранением отладочной информации

        Args:
            timestamp: Временная метка кадра
            image: Изображение
            points: Список ключевых точек
        """
        if image is None:
            logging.warning("Получено пустое изображение для отладки")
            return

        self.last_frame = image.copy()
        self.last_points = points
        self.frame_counter += 1

        frame_filename = os.path.join(
            self.output_dir, f"frame_{self.frame_counter:06d}_{timestamp:.3f}.jpg"
        )

        debug_frame = self.last_frame.copy()

        self._save_points_data(timestamp, points)
        self._visualize_points_on_frame(debug_frame, points)

        cv2.putText(
            debug_frame,
            f"Frame: {self.frame_counter}, TS: {timestamp:.3f}, Points: {len(points)}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
        )

        cv2.imwrite(frame_filename, debug_frame)

        if self.frame_counter % 5 == 0:
            cv2.imshow("ORB-SLAM3 Debug", debug_frame)
            cv2.waitKey(1)

        logging.info(
            f"[DEBUG] Сохранен отладочный кадр {self.frame_counter}, точек: {len(points)}"
        )

        self._print_points_info(timestamp, points, max_points=5)

    def _save_points_data(self, timestamp: float, points: List[KeyPoint]):
        """
        Сохраняет информацию о точках в CSV-файл

        Args:
            timestamp: Временная метка кадра
            points: Список ключевых точек
        """
        with open(self.points_log_file, "a") as f:
            for i, point in enumerate(points):
                if point.world_x is not None:
                    f.write(
                        f"{timestamp},{i},{point.image_x},{point.image_y},"
                        f"{point.world_x},{point.world_y},{point.world_z}\n"
                    )

    def _visualize_points_on_frame(self, frame: np.ndarray, points: List[KeyPoint]):
        """
        Отрисовка точек на изображении с цветовой кодировкой глубины

        Args:
            frame: Изображение для отрисовки
            points: Список ключевых точек
        """
        min_depth, max_depth = float("inf"), -float("inf")

        for point in points:
            if point.world_z is not None:
                min_depth = min(min_depth, point.world_z)
                max_depth = max(max_depth, point.world_z)

        if min_depth == float("inf"):
            min_depth, max_depth = 0, 1

        depth_range = max(0.001, max_depth - min_depth)

        for i, point in enumerate(points):
            try:
                x, y = int(point.image_x), int(point.image_y)

                if 0 <= x < frame.shape[1] and 0 <= y < frame.shape[0]:
                    if point.world_z is not None:
                        normalized_depth = (point.world_z - min_depth) / depth_range
                        color = (
                            int(255 * (1 - normalized_depth)),  # B
                            0,  # G
                            int(255 * normalized_depth),  # R
                        )
                    else:
                        color = (0, 255, 0)  # G

                    cv2.circle(frame, (x, y), 4, color, -1)
                    cv2.putText(
                        frame,
                        f"{i}",
                        (x + 5, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        (255, 255, 255),
                        1,
                    )

                    if (
                        point.world_x is not None
                        and point.world_y is not None
                        and point.world_z is not None
                    ):
                        cv2.line(frame, (x, y), (x, y - 15), color, 1)
                        cv2.circle(frame, (x, y - 15), 2, color, -1)
            except Exception as e:
                logging.error(f"Ошибка при отрисовке точки {i}: {e}")

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
        logging.info(
            f"[DEBUG] === Отладочный вывод точек для кадра {self.frame_counter}, TS: {timestamp:.3f} ==="
        )

        valid_points = [p for p in points if p.world_x is not None]

        if not valid_points:
            logging.info("[DEBUG] Нет точек с 3D координатами")
            return

        logging.info(
            f"[DEBUG] Всего точек с 3D координатами: {len(valid_points)}/{len(points)}"
        )

        sorted_points = sorted(
            valid_points,
            key=lambda p: p.world_z if p.world_z is not None else float("inf"),
        )

        logging.info("[DEBUG] Ближайшие точки:")
        for i, point in enumerate(sorted_points[:max_points]):
            logging.info(
                f"[DEBUG] Точка {i}: "
                f"Экран: ({point.image_x:.1f}, {point.image_y:.1f}), "
                f"Мир: ({point.world_x:.2f}, {point.world_y:.2f}, {point.world_z:.2f}), "
                f"Размер: {point.size:.1f}, Угол: {point.angle:.1f}°"
            )

        depths = [p.world_z for p in valid_points if p.world_z is not None]
        if depths:
            logging.info(
                f"[DEBUG] Статистика глубины - "
                f"Мин: {min(depths):.2f}, "
                f"Макс: {max(depths):.2f}, "
                f"Среднее: {sum(depths)/len(depths):.2f}"
            )

        logging.info("[DEBUG] ======================================================")

    def generate_report(self):
        """Генерация HTML-отчета с отладочной информацией"""
        report_path = os.path.join(self.output_dir, "debug_report.html")

        with open(report_path, "w", encoding="utf-8") as f:
            f.write(
                """
            <!DOCTYPE html>
            <html>
            <head>
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

        logging.info(f"[DEBUG] Сгенерирован HTML-отчет: {report_path}")
        return report_path


def main():
    parser = argparse.ArgumentParser(
        description="Получение и обработка данных ORB-SLAM3"
    )
    parser.add_argument(
        "--features-port",
        type=int,
        default=5557,
        help="Порт для приема данных о точках (по умолчанию 5557)",
    )
    parser.add_argument(
        "--visualize", action="store_true", help="Включить визуализацию"
    )
    parser.add_argument("--debug", action="store_true", help="Включить режим отладки")
    parser.add_argument(
        "--debug-output",
        type=str,
        default="debug_output",
        help="Директория для сохранения отладочных данных",
    )
    args = parser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(
        format="%(asctime)s - %(levelname)s: %(message)s", level=log_level
    )

    running = True

    def signal_handler(sig, frame):
        nonlocal running
        logging.info("Получен сигнал завершения...")
        running = False

    signal.signal(signal.SIGINT, signal_handler)

    receiver = ORBFeatureReceiver(features_port=args.features_port)

    debug_visualizer = None
    if args.debug:
        debug_visualizer = DebugVisualizer(output_dir=args.debug_output)
        receiver.register_callback(debug_visualizer.process_frame)
        logging.info(f"Запущен отладочный режим, вывод в {args.debug_output}")

    def example_processor(timestamp, image, points):
        """Обработчик данных"""
        logging.info(f"Обработка кадра {timestamp} с {len(points)} точками")
        for i, point in enumerate(points[:3], 1):
            if point.world_x is not None:
                logging.debug(
                    f"Точка {i}: изображение ({point.image_x:.1f}, {point.image_y:.1f}), "
                    f"мир ({point.world_x:.2f}, {point.world_y:.2f}, {point.world_z:.2f})"
                )

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
            logging.info(f"Сгенерирован отчет: {report_path}")

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
