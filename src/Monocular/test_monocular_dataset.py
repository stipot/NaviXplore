#!/usr/bin/env python

import os
import argparse
import logging
import cv2
import numpy as np
import time
import zmq
import signal
import threading

# Константы
DEFAULT_PORT = 5555

# Глобальная переменная для управления завершением
running = True

def signal_handler(sig, frame):
    """Обработчик сигнала для корректного завершения программы."""
    global running
    running = False
    logging.info("Получен сигнал завершения, останавливаем воспроизведение...")

class DatasetPlayer:
    """Класс для воспроизведения датасета и отправки через ZMQ."""
    
    def __init__(self, dataset_path, socket_addr):
        """Инициализирует плеер датасета."""
        self.dataset_path = dataset_path
        self.images_path = os.path.join(dataset_path, "images")
        self.timestamps_file = os.path.join(dataset_path, "timestamps.txt")
        
        # Проверяем наличие директорий и файлов
        if not os.path.exists(self.images_path):
            raise FileNotFoundError(f"Директория с изображениями не найдена: {self.images_path}")
        
        # Инициализация ZMQ
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(socket_addr)
        logging.info(f"ZMQ сокет открыт на {socket_addr}")
        
        # Загружаем временные метки
        self.timestamps = []
        self._load_timestamps()
        
        # Находим файлы изображений
        self.image_files = sorted([f for f in os.listdir(self.images_path) if f.endswith('.png')])
        if not self.image_files:
            raise ValueError(f"В директории {self.images_path} нет изображений")
        
        logging.info(f"Найдено {len(self.image_files)} изображений")

    def _load_timestamps(self):
        """Загружает временные метки из файла."""
        if os.path.exists(self.timestamps_file):
            with open(self.timestamps_file, 'r') as f:
                self.timestamps = [int(line.strip()) for line in f if line.strip()]
            logging.info(f"Загружено {len(self.timestamps)} временных меток")
        else:
            # Если файл с метками отсутствует, извлекаем метки из имен файлов
            logging.warning(f"Файл {self.timestamps_file} не найден, используем имена файлов")

    def play(self, speed_factor=1.0, loop=False):
        """Воспроизводит датасет с отправкой кадров через ZMQ."""
        global running
        
        images = sorted([f for f in os.listdir(self.images_path) if f.endswith('.png')])
        if not images:
            logging.error("Нет изображений для воспроизведения")
            return
        
        # Извлекаем временные метки из имен файлов, если не загружены
        if not self.timestamps:
            self.timestamps = [int(os.path.splitext(f)[0]) for f in images]
        
        logging.info(f"Начинаем воспроизведение датасета со скоростью {speed_factor}x")
        
        while running:
            for i in range(len(images) - 1):
                if not running:
                    break
                
                # Загружаем текущее изображение
                img_path = os.path.join(self.images_path, images[i])
                frame = cv2.imread(img_path)
                
                if frame is None:
                    logging.warning(f"Не удалось прочитать изображение: {img_path}")
                    continue
                
                # Отправляем изображение через ZMQ (как в start_monocular.py)
                _, jpeg_buffer = cv2.imencode('.jpg', frame)
                self.socket.send(jpeg_buffer.tobytes())
                
                # Показываем изображение
                cv2.imshow("Playback", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    running = False
                    break
                
                # Ждем до следующего кадра с учетом временных меток
                if i < len(self.timestamps) - 1:
                    delay = (self.timestamps[i+1] - self.timestamps[i]) / 1e9 / speed_factor
                    time.sleep(max(0, delay))
                
                # Показываем прогресс
                if i % (len(images) // 10) == 0 and i > 0:
                    progress = i / len(images) * 100
                    logging.info(f"Прогресс воспроизведения: {progress:.1f}%")
            
            if not loop or not running:
                break
            
            logging.info("Перезапуск воспроизведения...")
        
        logging.info("Воспроизведение завершено")

    def close(self):
        """Закрывает соединения и освобождает ресурсы."""
        self.socket.close()
        self.context.term()
        cv2.destroyAllWindows()

def main():
    """Основная функция программы."""
    # Настройка обработчика сигналов
    signal.signal(signal.SIGINT, signal_handler)
    
    # Парсинг аргументов командной строки
    parser = argparse.ArgumentParser(description="Воспроизведение датасета с отправкой через ZMQ")
    parser.add_argument('--dataset', type=str, required=True, help='Путь к директории с датасетом')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT, help=f'Порт для ZMQ PUB сокета (по умолчанию {DEFAULT_PORT})')
    parser.add_argument('--speed', type=float, default=1.0, help='Множитель скорости воспроизведения (по умолчанию 1.0)')
    parser.add_argument('--loop', action='store_true', help='Повторять воспроизведение по кругу')
    args = parser.parse_args()
    
    # Настройка логгирования
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    
    # Проверяем путь к датасету
    if not os.path.exists(args.dataset):
        logging.error(f"Указанный путь к датасету не существует: {args.dataset}")
        return
    
    # Настраиваем ZMQ адрес
    socket_addr = f"tcp://*:{args.port}"
    
    try:
        # Создаем и запускаем плеер
        player = DatasetPlayer(args.dataset, socket_addr)
        player.play(speed_factor=args.speed, loop=args.loop)
    except KeyboardInterrupt:
        logging.info("Прервано пользователем")
    except Exception as e:
        logging.error(f"Ошибка при воспроизведении датасета: {e}")
    finally:
        if 'player' in locals():
            player.close()

if __name__ == '__main__':
    main()
