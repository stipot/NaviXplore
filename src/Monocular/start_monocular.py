#!/usr/bin/env python

"""
Неактуальный скрипт для запуска обхода транспорта траектории в монокулярном режиме в CARLA Simulator.
Требует запущенного симулятора и установленного Python API для него.
"""

import carla
import zmq
import cv2
import numpy as np
import time
import threading
import signal
import sys
import random
import argparse
import logging

TARGET_FPS = 20
FRAME_TIME = 1 / TARGET_FPS

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")

running = True

USE_CUDA = cv2.cuda.getCudaEnabledDeviceCount() > 0

def signal_handler(sig, frame):
    global running
    running = False
    print("Завершение работы...")

signal.signal(signal.SIGINT, signal_handler)

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        if int_generation in [1, 2, 3]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []

def save_image(image, timestamp):
    thread = threading.Thread(target=process_image, args=(image, timestamp))
    thread.start()

def process_image(image, timestamp):
    if USE_CUDA:
        image_gpu = cv2.cuda_GpuMat()
        image_bgr = image[:, :, ::-1]
        image_gpu.upload(image_bgr)

        processed_image = cv2.cuda.cvtColor(image_gpu, cv2.COLOR_BGR2GRAY)
        result = processed_image.download()
        cv2.imwrite(f"image_{timestamp}.png", result)
    else:
        image_bgr = image[:, :, ::-1]
        cv2.imwrite(f"image_{timestamp}.png", image_bgr)

def process_images(image_queue):
    """Обработка и передача изображений в отдельном потоке"""
    global running
    while running:
        if not image_queue:
            time.sleep(0.01)
            continue

        image = image_queue.pop(0)
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        frame = array.reshape((image.height, image.width, 4))[:, :, :3]
        
        _, jpeg_buffer = cv2.imencode('.jpg', frame)
        socket.send(jpeg_buffer.tobytes())

        cv2.imshow("Camera Feed", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            running = False

def main():
    global running
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--seed', metavar='S', type=int, default=int(time.time()), help='Set random device seed and deterministic mode for Traffic Manager')
    argparser.add_argument('--host', metavar='H', default='127.0.0.1', help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('-p', '--port', metavar='P', default=2000, type=int, help='TCP port to listen to (default: 2000)')
    argparser.add_argument('-n', '--number-of-vehicles', metavar='N', default=1, type=int, help='Number of vehicles (default: 1)')
    argparser.add_argument('--filterv', metavar='PATTERN', default='vehicle.*', help='Filter vehicle model (default: "vehicle.*")')
    argparser.add_argument('--generationv', metavar='G', default='All', help='restrict to certain vehicle generation (values: "2","3","All" - default: "All")')
    argparser.add_argument('--tm-port', metavar='P', default=8000, type=int, help='Port to communicate with TM (default: 8000)')
    argparser.add_argument('--asynch', action='store_true', help='Activate asynchronous mode execution')
    argparser.add_argument('--car-lights-on', action='store_true', default=False, help='Enable automatic car light management')
    argparser.add_argument('--hero', action='store_true', default=False, help='Set one of the vehicles as hero')

    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False
    random.seed(args.seed if args.seed is not None else int(time.time()))

    try:
        world = client.get_world()

        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)

        settings = world.get_settings()
        if not args.asynch:
            traffic_manager.set_synchronous_mode(True)
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            else:
                synchronous_master = False
        else:
            print("You are currently in asynchronous mode, and traffic might experience some issues")

        world.apply_settings(settings)

        blueprints = get_actor_blueprints(world, args.filterv, args.generationv)
        if not blueprints:
            raise ValueError("Couldn't find any vehicles with the specified filters")

        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if args.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif args.number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points

        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        batch = []
        hero = args.hero
        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            if hero:
                blueprint.set_attribute('role_name', 'hero')
                hero = False
            else:
                blueprint.set_attribute('role_name', 'autopilot')

            batch.append(SpawnActor(blueprint, transform)
                .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

        for response in client.apply_batch_sync(batch, synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

        print('spawned %d vehicles, press Ctrl+C to exit.' % len(vehicles_list))

        traffic_manager.global_percentage_speed_difference(30.0)

        vehicle = world.get_actor(vehicles_list[0])
        blueprint_library = world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '640')
        camera_bp.set_attribute('image_size_y', '480')
        camera_bp.set_attribute('fov', '110')

        camera_spawn_point = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_bp, camera_spawn_point, attach_to=vehicle)

        image_queue = []
        camera.listen(lambda image: image_queue.append(image))

        image_thread = threading.Thread(target=process_images, args=(image_queue,))
        image_thread.start()

        vehicle.set_autopilot(True)

        while running:
            if not args.asynch and synchronous_master:
                world.tick()
            else:
                world.wait_for_tick()

    finally:
        if not args.asynch and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        camera.stop()
        image_thread.join()
        socket.close()
        context.term()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
