"""
Функционал генерации конфигурационного .yaml файла для скрипта record_monocular_dataset.py в формате ORB-SLAM3.
"""

import math


def generate_yaml(camera_width, camera_height, camera_fow, simulation_hz, dataset_path):
    """Генерирует конфигурационный файл yaml для ORB-SLAM3"""
    width = int(camera_width)
    height = int(camera_height)
    fow = int(camera_fow)

    fx, fy, cx, cy = compute_intrinsics(width, height, fow)

    yaml_template = """%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters (from CARLA)
#--------------------------------------------------------------------------------------------

File.version: "1.0"

Camera.type: "PinHole"

# Camera calibration and distortion parameters (OpenCV) 
Camera1.fx: {fx}
Camera1.fy: {fy}
Camera1.cx: {cx}
Camera1.cy: {cy}

Camera1.k1: 0.0
Camera1.k2: 0.0
Camera1.p1: 0.0
Camera1.p2: 0.0

Camera.width: {width}
Camera.height: {height}

Camera.newWidth: {width}
Camera.newHeight: {height}

# Camera frames per second 
Camera.fps: {fps}

# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
Camera.RGB: 1

#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------

# ORB Extractor: Number of features per image
ORBextractor.nFeatures: 10000

# ORB Extractor: Scale factor between levels in the scale pyramid
ORBextractor.scaleFactor: 1.2

# ORB Extractor: Number of levels in the scale pyramid
ORBextractor.nLevels: 8

# ORB Extractor: Fast threshold
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500.0
    """

    yaml_filled = yaml_template.format(
        fx=fx, fy=fy, cx=cx, cy=cy, width=width, fps=simulation_hz, height=height
    )
    yaml_name = f"{dataset_path}/carla.yaml"

    try:
        with open(yaml_name, "w") as f:
            f.write(yaml_filled)

        return yaml_name
    except Exception as e:
        raise Exception(
            f"Конфигурационный файл {yaml_name} не удалось сгенерировать:\n{e}"
        )


def compute_intrinsics(width, height, fov):
    fx = width / (2.0 * math.tan(math.radians(fov) / 2.0))
    fy = fx
    cx = width / 2.0
    cy = height / 2.0
    return fx, fy, cx, cy
