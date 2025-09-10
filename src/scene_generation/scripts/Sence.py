#!/usr/bin/env python3
#coding=utf-8

import rospy
import math
from gazebo_msgs.srv import SpawnModel, DeleteModel, ApplyBodyWrench, GetModelState
from geometry_msgs.msg import Pose, Wrench, Point
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
import random
import os

class Sence:
    def __init__(self):
        # Init ROS node and wait for services
        rospy.init_node('spawn_object_node', anonymous=True)
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/delete_model')

        # Create the service proxies
        self.spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.apply_wrench_prox = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.get_model_state_prox = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.delete_model_prox = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        # Init for global variable
        self.spawned_models = []
        self.workspace_limits=[[-0.25, -0.75], [-0.5, 0.5], [0, 0.4]] # x,y,z (min,max) in meters
        self.object_center = [] 
        self.num_obstacles = 5
        self.object_names = []
        self.object_postions = []
        self.object_orientations = []
        self.model_id=[]


    def spawn_object(self, model_name, model_xml, position, orientation_rpy):
        try:
            pose = Pose()
            pose.position.x = position[0]
            pose.position.y = position[1]
            pose.position.z = position[2]
            
            quaternion = quaternion_from_euler(orientation_rpy[0], orientation_rpy[1], orientation_rpy[2])
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            # make the object dynamic
            model_xml = model_xml.replace("<static>true</static>", "<static>false</static>")

            response = self.spawn_model_prox(model_name, model_xml, '', pose, 'world')
            if response.success:
                self.spawned_models.append(model_name)
                rospy.loginfo("Model spawned: %s" % model_name)
            else:
                rospy.logwarn("Failed to spawn model: %s" % response.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def apply_force_towards_target(self, model_name, target_position):
        try:
            # Get the current position of the model
            model_state = self.get_model_state_prox(model_name, '')
            current_position = model_state.pose.position # This is the model's current position
            # Calculate the force direction vector (from current position to target position)
            force_direction = [
                target_position[0] - current_position.x,
                target_position[1] - current_position.y,
                target_position[2] - current_position.z
            ]

            # Calculate the magnitude of the force vector
            force_magnitude = 2  # You can adjust the magnitude as needed
            magnitude = math.sqrt(sum([i ** 2 for i in force_direction]))

            if magnitude == 0:
                rospy.logwarn("Target and current position are the same. No force will be applied.")
                return

            # Normalize the direction vector
            normalized_direction = [i / magnitude for i in force_direction]

            # Create the wrench to apply the force
            force = Wrench()
            force.force.x = normalized_direction[0] * force_magnitude
            force.force.y = normalized_direction[1] * force_magnitude
            force.force.z = normalized_direction[2] * force_magnitude
            body_name = f"{model_name}::link"


            # Apply the force to the model
            resp = self.apply_wrench_prox(
                body_name=body_name,
                reference_frame='world',
                reference_point=Point(0, 0, 0),
                wrench=force,
                start_time=rospy.Time(0),
                duration=rospy.Duration(0.1)
            )
            rospy.loginfo(f"Force applied to {model_name}: {resp.success}")

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
 
    def delete_all_objects(self):
        
        try:
            for model_name in self.spawned_models:
                response = self.delete_model_prox(model_name)
                if response.success:
                    rospy.loginfo("Model deleted: %s" % model_name)
                else:
                    rospy.logwarn("Failed to delete model: %s" % response.status_message)
            self.spawned_models = []  # Clear the list after deletion
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
    def find_safe_positions(self, objects, workspace=[[-0.65, -0.35], [-0.15, 0.15], [0.02, 0.04]], min_dist=0.07,
                            max_dist=0.1,
                            num_positions=1, max_attempts=10000):
        def is_valid(candidate):
            # print(candidate)
            if not (workspace[0][0] <= candidate[0] <= workspace[0][1] and
                    workspace[1][0] <= candidate[1] <= workspace[1][1] and
                    workspace[2][0] <= candidate[2] <= workspace[2][1]):
                return False

            m_d = min(math.dist(candidate, object) for object in objects)
            for obj in objects:
                distance = math.dist(candidate, obj)
                if not (min_dist <= distance and m_d <= max_dist):
                    return False

            return True

        found = []
        for _ in range(max_attempts):
            if len(found) >= num_positions:
                break

            candidate = [
                round(random.uniform(workspace[0][0], workspace[0][1]), 4),
                round(random.uniform(workspace[1][0], workspace[1][1]), 4),
                round(random.uniform(workspace[2][0], workspace[2][1]), 4)
            ]
  
            if is_valid(candidate):
                found = candidate
 
        return found
    def select_random_mesh_models(self, mesh_dir):
        """
        Select random mesh models from the specified directory.
        """
        mesh_files = []
        for folder in os.listdir(mesh_dir):
            folder_path = os.path.join(mesh_dir, folder)
            if os.path.isdir(folder_path):
                for f in os.listdir(folder_path):
                    if f.endswith('.sdf'):
                        mesh_files.append(os.path.join(folder_path, f))
        num_models = self.num_obstacles + 1
        if len(mesh_files) < num_models:
            rospy.logwarn(f"Not enough mesh files in {mesh_dir}, found {len(mesh_files)}, need {num_models}")
            self.model_id = mesh_files
        else:
            self.model_id = random.sample(mesh_files, num_models)

    def random_env_generation(self):
        self.object_names = []
        drop_x = random.uniform(-0.4,-0.6) 
        drop_y = random.uniform(-0.1,0.1) 
        dorp_z = 0.025
        self.object_postions.append([drop_x,drop_y,dorp_z])
        self.object_orientations.append([0,0,random.uniform(-3.14,3.14)])
        self.select_random_mesh_models("workspace/src/scene_generation/meshs")
        for i in range(self.num_obstacles):
            temp_pos = self.find_safe_positions(self.object_postions)
            if temp_pos:
                self.object_postions.append(temp_pos)
                self.object_orientations.append([0,0,random.uniform(-3.14,3.14)])
            else:
                rospy.logwarn("Failed to find a valid position for object %d" % i)
                break
        for object_id in range(len(self.object_postions)):
            if object_id == 0:
                model_name = "target_object"
            else:
                model_name = f"object_{object_id}"
            model_file_path = self.model_id[object_id]
            with open(model_file_path, "r") as f:
                model_xml = f.read()
            self.spawn_object(model_name, model_xml, self.object_postions[object_id], self.object_orientations[object_id])
            self.object_names.append(model_name)
            # Calculate the center of all objects
        self.object_center = [
            sum(pos[0] for pos in self.object_postions) / (self.num_obstacles+1),
            sum(pos[1] for pos in self.object_postions) / (self.num_obstacles+1),
            sum(pos[2] for pos in self.object_postions) / (self.num_obstacles+1)
        ]
        rospy.sleep(1)  # Ensure all objects are spawned before applying forces
        for name in self.object_names: 
            self.apply_force_towards_target(name, self.object_center)
        rospy.loginfo("All objects spawned successfully.")
        

if __name__ == "__main__":
    # 物体 1：静态物体
    scene = Sence()
    scene.random_env_generation()
    rospy.sleep(5)  # Wait to observe the effect of the applied forces
    scene.delete_all_objects()