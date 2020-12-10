import os
import argparse
import logging
from functools import wraps
import numpy as np
import yaml

from cdp4_data_collection import CDP4DataCollection
import geometry_msgs.msg as geom
from model import Model

from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.srv import GetModelState, GetWorldProperties, SpawnEntity, DeleteModel
from tf.transformations import quaternion_from_euler, euler_from_quaternion


# Set logger configurations.
LOG_FORMAT = "%(levelname)s %(asctime)s - %(message)s"
logging.basicConfig(filename='logs.log',
                    filemode='w',
                    format=LOG_FORMAT,
                    level=logging.INFO)
logger = logging.getLogger('spawn-model')

path_to_models = os.getenv("HBP") + "/Models/"

def wrap_logger(func):
    @wraps(func)
    def inner(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception as e:
            logger.error(e, exc_info=1)
            raise
    return inner

def _spawn_whole_room(room_name, layout_file="layout.yaml"):
    path_to_room = path_to_models + room_name + '/'

    with open(path_to_room + layout_file) as f:
        yaml_file = yaml.load(f)
    
    for model in yaml_file["Layout"]:
        position = model["position"].values()
        obj_name = model["model"]
        obj_dir = path_to_room + obj_name + '/'
        _spawn_single_object(obj_dir, position, object_name=obj_name)
        
def _spawn_single_object(object_dir, position, object_name=None):
    obj_name = object_name if object_name else object_dir
    logger.info(
        "Trying to spawn object {} in x={}, y={}, z={}...".format(
        obj_name, *position)
    )
    obj = Model(object_dir)
    obj.spawn()
    logger.info("Succesfully spawned")

@wrap_logger
def spawn(object_dir, position_str, room_name=""):
    """Tries to spawn new object in Gazebo.

    Args:
        object_name (str):
            name of the object folder. Ball_01, Table1 ...
        room_name (str, optional):
            name of the room folder. kitchen, bed_room etc. Defaults to "".
    """
    if room_name:
        _spawn_whole_room(room_name)
    else:
        position = [float(v) for v in position_str.split(',')]
        object_dir = path_to_models + object_dir + '/'
        _spawn_single_object(object_dir, position)

def set_parser():
    logger.info("Process is started...")
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-r",
        "--room",
        help="name of the room. If not given, the object in $HBP/Models/ will be spawned")
    parser.add_argument(
        "-o",
        "--object",
        help="directory of the object. If not given, all objects in the room will be spawned")
    parser.add_argument(
        "-p",
        "--positions",
        help="directory of the object. It is necessary for spawning a single object.")
    return parser.parse_args()

if __name__ == '__main__':
    args = set_parser()
    room_name = args.room
    object_name = args.object
    positions = args.positions
    spawn(object_name, positions, room_name)
