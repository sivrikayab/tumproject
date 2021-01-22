import os
import argparse
import logging
import random
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

def random_path_yielder(path):
    """It goes into the path and randomly yields a sub-directory.

    Args:
        path (str)
    """
    current_subs = [i[1] for i in os.walk(path)][0]
    current_subs_validated = [name for name in current_subs 
                                if '0' <= name[-1] <='9']
    random_sub_name = random.choice(current_subs_validated)
    while len(current_subs_validated) > 0 :
        current_subs_validated.remove(random_sub_name)
        yield random_sub_name

def _spawn_whole_room(room_name, layout_no, layout_file="layout.yaml"):
    path_to_room = path_to_models + room_name + '/'
    with open(path_to_room + layout_file) as f:
        yaml_file = yaml.load(f)
    
    seen_folder = {}

    layout = "Layout" + str(layout_no)
    for entity in yaml_file[layout]:
        folder = entity["folder"]
        positions = entity["position"]
        absolute_path = path_to_room + folder + '/'
        if folder[-1] == 's': # we go through sub models
            if folder not in seen_folder: # if the sub-dir seen before
                seen_folder[folder] = random_path_yielder(absolute_path)
            folder = next(seen_folder[folder])
            absolute_path += folder + '/'
        position = (positions['x'], positions['y'], positions['z'])
        _spawn_single_object(absolute_path, position, object_name=folder)

def _spawn_single_object(object_dir, position, object_name=None):
    obj_name = object_name if object_name else object_dir
    logger.info(
        "Trying to spawn object {} in x={}, y={}, z={}...".format(
        obj_name, *position)
    )
    obj = Model(object_dir, position)
    obj.spawn()
    logger.info("Succesfully spawned")

@wrap_logger
def spawn(object_dir, position_str, room_name="", layout_no=0):
    """Tries to spawn new object in Gazebo.

    Args:
        object_name (str):
            name of the object folder. Ball_01, Table1 ...
        room_name (str, optional):
            name of the room folder. kitchen, bed_room etc. Defaults to "".
    """
    if room_name:
        _spawn_whole_room(room_name, layout_no)
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
    parser.add_argument(
        "-l",
        "--layout",
        help="which layout you want to spawn. Takes integer value."
    )
    return parser.parse_args()

if __name__ == '__main__':
    args = set_parser()
    room_name = args.room
    object_dir = args.object
    positions = args.positions
    layout_no = args.layout
    spawn(object_dir, positions, room_name, layout_no)