#!/usr/bin/env python

import zmq
import time
import argparse
import logging
import signal
import sys

def main():
    parser = argparse.ArgumentParser(description='Прием координат от ORB-SLAM3')
    parser.add_argument('--port', type=int, default=5556, help='Порт для приема координат (по умолчанию 5556)')
    args = parser.parse_args()

    logging.basicConfig(format='%(asctime)s - %(levelname)s: %(message)s', level=logging.INFO)
    
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.bind(f"tcp://*:{args.port}")
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    socket.setsockopt(zmq.RCVTIMEO, 1000)
    
    logging.info(f"Ожидание координат на порту {args.port}...")

    running = True
    def signal_handler(sig, frame):
        nonlocal running
        logging.info("Получен сигнал завершения...")
        running = False
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        while running:
            try:
                message = socket.recv_string()
                parts = message.split(',')
                if len(parts) == 4:
                    timestamp, x, y, z = parts
                    logging.info(f"Получены координаты: timestamp={timestamp}, x={x}, y={y}, z={z}")
                else:
                    logging.warning(f"Получено некорректное сообщение: {message}")
            except zmq.Again:
                continue
            except zmq.ZMQError as e:
                logging.error(f"Ошибка ZMQ: {e}")
                break
    finally:
        logging.info('Завершение приема координат...')
        socket.close()
        context.term()
        logging.info('Ресурсы освобождены')

if __name__ == "__main__":
    main()
