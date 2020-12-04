import os
import argparse
import logging
from functools import wraps
import numpy as np

from cdp4_data_collection import CDP4DataCollection
import geometry_msgs.msg as geom

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


def generate_eqdis_pos():
    """ Generates points that are equally distant from each other.

    Yields:
        [type]: [description]
    """
    logger.info("Trying to generate position...")

    cache = []
    if not cache:
        pos = _get_pose_at_origin()
        cache.append(pos)
        yield pos
    else:
        # Not implemented yet! TODO
        pass


def _get_pose_at_origin():
    orientation = quaternion_from_euler(0, 0, 0)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = 0, 0, 0.25
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    return pose

@wrap_logger
def spawn(object_name, room_name=""):
    """Tries to spawn new object in Gazebo.

    Args:
        object_name (str):
            name of the object folder. Ball_01, Table1 ...
        room_name (str, optional):
            name of the room folder. kitchen, bed_room etc. Defaults to "".
    """
    model_dir = room_name + '/' + object_name if room_name else object_name
    poses = generate_eqdis_pos()

    if object_name is not None:
        logger.info("Trying to spawn object {} in {}...".format(
            object_name, room_name)
        )
        pose = next(poses)
        obj_ = CDP4DataCollection(path_to_models)
        obj_.add_object(model_dir, pose)
        logger.info("Succesfully spawned {} model at {} position".format(
            model_dir, pose)
        )
    else:
        # Not implemented yet! TODO
        logger.info("Trying to spawn all objects in {}...".format(room_name))


if __name__ == '__main__':
    logger.info("Process is started...")
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-r",
        "--room",
        help="name of the room. If not given, the object in $HBP/Models/ will be spawned")
    parser.add_argument(
        "-o",
        "--object",
        help="name of the object. If not given, all objects in the room will be spawned")
    args = parser.parse_args()
    room_name = args.room
    object_name = args.object
    spawn(object_name, room_name)
