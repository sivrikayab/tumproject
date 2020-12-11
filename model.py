import os
import rospy
import yaml
import numpy as np
from collections import defaultdict
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from gazebo_msgs.msg import ModelState, ModelStates

from cdp4_data_collection import CDP4DataCollection


class Model(CDP4DataCollection):

    root_path =  os.getenv("HBP") + "/Models/"

    def __init__(self,
                model_path,
                position=(0,0,0)):
        """
        Args:
            model_path (str): path to model folder. 
                <root_path>/CookingBench/
        """
        super(Model, self).__init__()
        self.model_path = model_path
        self.model_name = model_path.split('/')[-2]
        self.position = position
        self.orientation = self._read_configuration(model_path)
        self.pose = self._convert_to_pose(position, self.orientation)
        self.bottom = ""
        self.top = ""

    @staticmethod
    def _convert_to_pose(position, orientation):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z  = position
        pose.orientation.x, pose.orientation.y, \
            pose.orientation.z, pose.orientation.w = orientation
        return pose

    @staticmethod
    def _read_configuration(model_path, config_file="configs.yaml"):
        """
        reads configs.yaml file and assign parameters.
        """
        with open(model_path + config_file) as f:
            yaml_file = yaml.load(f)
        return tuple(yaml_file["orientation"].values())

    def spawn(self, reference_frame='world'):
        """
        Spawns a new object in the environment

        :param model_name: The model name of the object to be spawned
        :param pose: The pose where the object will be spawned, relative to world coordinates
        :param reference_frame: the reference frame in which the pose will be considered
        """
        with open(self.model_path + "model.sdf", "r") as model:
            sdf = model.read()

        model_name = self.model_name
        while model_name in self.last_model_states.name:
            parts = model_name.split('_')
            try:
                parts[-1] = str(int(parts[-1]) + 1)
            except:
                parts.append('1')
            model_name = "_".join(parts)

        res = self._spawn_model_srv(model_name, sdf, "", self.pose, reference_frame)
        rospy.loginfo(res)

    def get_object_pose(self, object_name, reference_frame='world'):
        """
        Gets the current pose of an object relative to the world's coordinate frame

        :param object_name: the model name of the object
        :param reference_frame: the reference frame from which the pose will be calculated
        """
        return self._get_pose_srv(object_name, reference_frame).pose

    def delete_object(self, model_name):
        """
        Deletes a model from the environment

        :param model_name: The name of the model to be deleted
        """
        try:
            self._delete_model_srv(model_name)
        except:
            rospy.logerr("In delete model: %s" % model_name)

    def __cart_to_ang(self, position):
        """
        Takes object's position as input and returns icub's absolute angles.
        """
        obj_pos = np.array([position.x, position.y, position.z])
        cam_pos = np.array([2.15042024657, 1.23814627784, 1.33805071957])
        rel_pos = np.subtract(obj_pos, cam_pos)
        horizontal_position = np.arctan(rel_pos[1] / rel_pos[0])
        vertical_position = np.arctan(rel_pos[2] / rel_pos[0])
        return horizontal_position, vertical_position

    def set_object_pose(self, object_name, pose):
        """
        :param object_name: the name of the object model
        :param pose: the new pose to model should be set to
        """
        # if store:
        #     self.spawned_objects.append(object_name)
        
        msg = ModelState()

        msg.model_name = object_name
        msg.reference_frame = 'world'
        msg.pose = pose
        msg.scale.x = msg.scale.y = msg.scale.z = 1.0

        # publish message on ros topic
        self._set_model_state_pub.publish(msg)
