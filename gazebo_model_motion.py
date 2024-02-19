#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Float64
import math
import yaml
#import random

class BoxMotionController:
    
    def __init__(self):
         
        rospy.init_node('box_motion_controller', anonymous=True)

        setup_file = '/home/riccardo/forest_ws/src/learning_pkg/src/setup.yaml'
        self.box_params = self.load_setup_params(setup_file)

        # Single publisher for setting model state
        self.model_state_publisher = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

        # Subscribers for starting and stopping motion
        self.start_motion_sub = rospy.Subscriber('/start_motion', Float64, self.start_motion_callback)

        # Time step and motion parameters
        self.rate = rospy.Rate(1000)
        self.time = 0
        self.dt = 0.001
        self.start_motion = False
        self.stop_motion = False
        
        # Box parameters
        self.amplitude = {box_name: params['dim'] / 4 for box_name, params in self.box_params.items()}
        self.frequency = {'box_fl': 0.2, 'box_fr': 0.2, 'box_rl': 0.2, 'box_rr': 0.2}
        self.phase_offsets = {'box_fl': 0.0, 'box_fr': math.pi / 4, 'box_rl': 3 * math.pi / 4, 'box_rr': math.pi / 2}

        rospy.loginfo("Box Motion Controller Initialized")

    def run(self):

        while not rospy.is_shutdown():

            if self.start_motion and not self.stop_motion:

                for box_name, params in self.box_params.items():

                    period = 1 / self.frequency[box_name]

                    time_to_first_min = period * self.phase_offsets[box_name] / (2 * math.pi)

                    position_offset = params['dim'] / 2 - self.amplitude[box_name]

                    if self.time < time_to_first_min:
                        position = - params['dim'] / 2
                    else:
                        position = - self.amplitude[box_name] * math.cos(2 * math.pi * self.frequency[box_name] * (self.time) - self.phase_offsets[box_name]) - (position_offset)

                    # Publish velocity to set_model_state topic
                    model_state_msg = ModelState()
                    model_state_msg.model_name = box_name
                    model_state_msg.pose.position.x = params['pos_x']
                    model_state_msg.pose.position.y = params['pos_y']
                    model_state_msg.pose.position.z = position

                    self.model_state_publisher.publish(model_state_msg)

                self.time += self.dt
                self.rate.sleep()

            else:
                self.rate.sleep()

    def start_motion_callback(self, msg):
        # Callback to start the motion
        rospy.loginfo("Motion Started")
        self.start_motion = True

    def load_setup_params(self, file_path):
        try:
            with open(file_path, 'r') as file:
                setup_params = yaml.safe_load(file)
            return setup_params
        except Exception as e:
            rospy.logerr(f"Error loading setup parameters: {e}")
            return {}

if __name__ == '__main__':
    try:
        controller = BoxMotionController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
