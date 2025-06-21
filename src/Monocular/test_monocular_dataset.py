#!/usr/bin/env python

"""
Скрипт для последовательной передачи собранного через скрипт record_monocular_dataset.py
набора данных в ORB-SLAM3 через порт zmq.
"""

import os
import argparse
import logging
import time
import signal
import json
import cv2
import zmq


DEFAULT_PORT = 5555
running = True


def signal_handler(sig, frame):
    """Обработчик сигнала для корректного завершения программы."""
    global running
    running = False
    logging.info("Получен сигнал завершения, остановка воспроизведения...")


class DatasetPlayer:
    """Класс для воспроизведения датасета и отправки через ZMQ."""

    def __init__(self, dataset_path, socket_addr):
        """Инициализирует плеер датасета."""
        self.dataset_path = dataset_path
        self.images_path = os.path.join(dataset_path, "images")
        self.timestamps_file = os.path.join(dataset_path, "timestamps.txt")

        if not os.path.exists(self.images_path):
            raise FileNotFoundError(
                f"Директория с изображениями не найдена: {self.images_path}"
            )

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(socket_addr)
        logging.info("ZMQ сокет открыт на %s", socket_addr)

        self.timestamps = []
        self._load_timestamps()

        self.image_files = sorted(
            [f for f in os.listdir(self.images_path) if f.endswith(".png")]
        )
        if not self.image_files:
            raise ValueError(f"В директории {self.images_path} нет изображений")

        logging.info("Найдено %d изображений", len(self.image_files))

    def _load_timestamps(self):
        """Загружает временные метки из файла."""
        if os.path.exists(self.timestamps_file):
            with open(self.timestamps_file, "r", encoding="utf-8") as f:
                self.timestamps = [int(line.strip()) for line in f if line.strip()]
            logging.info("Загружено %d временных меток", len(self.timestamps))
        else:
            logging.warning(
                "Файл %s не найден, используются имена файлов", self.timestamps_file
            )

    def play(self, speed_factor=1.0, loop=False):
        """Воспроизводит датасет с отправкой кадров через ZMQ."""
        global running

        images = sorted([f for f in os.listdir(self.images_path) if f.endswith(".png")])
        if not images:
            logging.error("Нет изображений для воспроизведения")
            return

        if not self.timestamps:
            self.timestamps = [int(os.path.splitext(f)[0]) for f in images]

        logging.info(
            "Начинается воспроизведение датасета со скоростью %sx", speed_factor
        )

        while running:
            for i in range(len(images) - 1):
                if not running:
                    break

                img_path = os.path.join(self.images_path, images[i])
                frame = cv2.imread(img_path)

                if frame is None:
                    logging.warning("Не удалось прочитать изображение: %s", img_path)
                    continue

                _, jpeg_buffer = cv2.imencode(".jpg", frame)

                message_data = {"timestamp": self.timestamps[i], "frame_id": i}

                message_json = json.dumps(message_data)
                self.socket.send_string(message_json, zmq.SNDMORE)
                self.socket.send(jpeg_buffer.tobytes())

                logging.debug(
                    "Отправлен кадр %s с timestamp %s", i, message_data["timestamp"]
                )

                cv2.imshow("Playback", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    running = False
                    break

                if i < len(self.timestamps) - 1:
                    delay = (
                        (self.timestamps[i + 1] - self.timestamps[i])
                        / 1e9
                        / speed_factor
                    )
                    time.sleep(max(0, delay))

                if i % (len(images) // 10) == 0 and i > 0:
                    progress = i / len(images) * 100
                    logging.info("Прогресс воспроизведения: %.1f%%", progress)

            if not loop or not running:
                break

            logging.info("Перезапуск воспроизведения...")

        logging.info("Воспроизведение завершено")

    def close(self):
        """Закрывает соединение и освобождает ресурсы."""
        self.socket.close()
        self.context.term()
        cv2.destroyAllWindows()


def main():
    """Главная функция. Запускает программу"""
    signal.signal(signal.SIGINT, signal_handler)

    parser = argparse.ArgumentParser(
        description="Воспроизведение датасета с отправкой через ZMQ"
    )
    parser.add_argument(
        "--dataset", metavar="СТРОКА", type=str, required=True, help="Директория"
    )
    parser.add_argument(
        "--port",
        metavar="INT",
        type=int,
        default=DEFAULT_PORT,
        help=f"Порт для ZMQ PUB сокета (по умолчанию {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--speed",
        metavar="FLOAT",
        type=float,
        default=1.0,
        help="Множитель скорости воспроизведения (по умолчанию 1.0)",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="Зациклить воспроизведение (по умолчанию: выключено)",
        default=False,
    )
    args = parser.parse_args()

    logging.basicConfig(format="%(levelname)s: %(message)s", level=logging.INFO)

    if not os.path.exists(args.dataset):
        logging.error("Указанный путь к датасету не существует: %s", args.dataset)
        return

    socket_addr = f"tcp://*:{args.port}"

    try:
        player = DatasetPlayer(args.dataset, socket_addr)
        player.play(speed_factor=args.speed, loop=args.loop)
    except KeyboardInterrupt:
        logging.info("Прервано пользователем")
    except Exception as e:
        logging.error("Ошибка при воспроизведении датасета: %s", e)
    finally:
        if "player" in locals():
            player.close()


if __name__ == "__main__":
    main()
