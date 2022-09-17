#!/usr/bin/env python
'''
@author : USRG(Unmanned Systems Research Group) @ KAIST
@date   : 2021-03
@brief  : A ROS node script for simple car dynamics simulation.
'''
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Accel
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker

from tf.transformations import quaternion_from_euler
import tf_conversions
import tf2_ros

import numpy as np

import sys
from time import gmtime, strftime

import dynamics

class SimulateStep(object):
    def __init__(self, stateDim, inputDim, dt):
        self.namespace  = ""
        '''
        states : [x, y, yaw, vx]
        '''

        '''
        Initial conditions
        '''
        self.x0          = 0.
        self.y0          = 0.
        self.yaw0        = 0.
        self.vx0         = 0.

        '''
        Parameters (RC platform)
        '''
        self.dt              = dt
        self.max_steer_anlge = 17.75 * np.pi / 180 # rad
        self.max_vx          = 30.0 / 3.6 # m/s
        self.length          = 0.257

        '''
        Variable initialization
        '''
        self.stateDim       = stateDim
        self.inputDim       = inputDim
        self.dynamicsDim    = 4
        self.states_init    = [self.x0, self.y0, self.yaw0, self.vx0]
        self.states         = self.states_init
        self.states_dot     = np.zeros(self.stateDim)
        self.state_hist     = np.zeros([1, (self.stateDim)])
        self.state_dot_hist = np.zeros([1, (self.stateDim)])
        self.data           = np.zeros([1,2*self.dynamicsDim+self.inputDim])
        self.inputs         = np.zeros(self.inputDim)
        self.toggle_switch  = False

        '''
        Vehicle dynamics object
        '''
        self.dynamics    = dynamics.Dynamics(stateDim, inputDim, dt)
        # self.dynamics.modelType = 1
        self.dynamics.max_steer = self.max_steer_anlge

        '''
        ROS publishers
        '''
        self.pose_pub     = rospy.Publisher('simulation/pose', PoseStamped, queue_size=1)
        self.bodyOdom_pub = rospy.Publisher('simulation/bodyOdom', Odometry, queue_size=1) # velocities in the body frame.
        self.poseOdom_pub = rospy.Publisher('simulation/poseOdom', Odometry, queue_size=1) # velocities in the map frame.
        self.input_pub    = rospy.Publisher('simulation/inputs', Joy, queue_size=1)
        self.accel_pub    = rospy.Publisher('simulation/acceleration', Accel, queue_size=1)
        self.br = tf2_ros.TransformBroadcaster()
        self.ego_model_marker_pub = rospy.Publisher('simulation/car_marker', Marker, queue_size=1)

        '''
        Variable definitions for autonomous driving
        '''
        self.auto_mode     = True
        self.control_sub   = rospy.Subscriber('control', AckermannDriveStamped, self.controlSubCallback)
        self.steering      = 0
        self.throttle      = 0
        self.steer_angle_to_norm = 1/self.max_steer_anlge #1/0.25 #30/180*np.pi
        self.throttle_to_norm = 1

    def controlSubCallback(self, msg):
        '''
        Unit conventions
        - msg.drive.steering_angle : angle[rad]
        - msg.drive.acceleration   : acceleration[m/s^2]
        - self.inputs[0]           : normalized steer input(min:-1, max:1)
        - self.inputs[1]           : normalized throttle input(min:-1, max:1)
        '''
        self.steering = msg.drive.steering_angle * self.steer_angle_to_norm
        self.throttle = msg.drive.acceleration * self.throttle_to_norm
        # min/max at +-1
        self.steering = np.clip(self.steering, -1, 1)
        self.throttle = np.clip(self.throttle, -1, 1)
        if self.auto_mode == True:
            self.inputs = np.array([self.steering, self.throttle])

    def simPoseSubCallback(self, msg):
        self.publishTF(self, msg.header.stamp, msg)

    def one_step_forward(self, inputs):
        '''
        Proceed one step
        '''
        if self.auto_mode == False:
            self.inputs     = inputs

        self.states_dot = self.get_states_dot(self.states, self.inputs)

        if self.toggle_switch:
            self.states_dot = np.zeros_like(self.states)
            self.states     = self.states_init
        # Euler method
        self.states = self.states + self.states_dot * self.dt

        # Physical Limitation
        # - velocity limit
        self.states[3] = np.clip(self.states[3], 0, self.max_vx)

        '''
        Stack history(optional)
        '''
        self.state_hist  = np.append(self.state_hist, [self.states], axis=0)
        self.state_dot_hist  = np.append(self.state_dot_hist, [self.states_dot], axis=0)
        data_input = np.hstack([self.states[self.stateDim-self.dynamicsDim:], self.inputs])
        data_label = self.states_dot[self.stateDim-self.dynamicsDim:]
        self.data = np.append(self.data,[np.hstack([data_input,data_label])],axis=0)

        '''
        Publish
        '''
        stamp = rospy.Time.now()

        self.publishPose(stamp, self.states)
        self.publishJoy(stamp, self.inputs, self.toggle_switch)
        self.publishBodyOdom(stamp, self.states)
        self.publishPoseOdom(stamp, self.states)
        self.publishAccel(stamp, self.states_dot)

    def get_states_dot(self, states, inputs):
        return self.dynamics.forward(states, inputs)

    def rotate(self, pos, yaw):
        x_rot = np.cos(yaw)*pos[0] - np.sin(yaw)*pos[1]
        y_rot = np.sin(yaw)*pos[0] + np.cos(yaw)*pos[1]

        return x_rot, y_rot

    def publishTF(self, stamp, poseMsg):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = self.namespace[1:] + "base_link"
        t.transform.translation.x = poseMsg.pose.position.x
        t.transform.translation.y = poseMsg.pose.position.y
        t.transform.translation.z = 0.0
        q = poseMsg.pose.orientation
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w
        self.br.sendTransform(t)

    def publishPose(self, stamp, states):
        poseMsg = PoseStamped()
        poseMsg.header.frame_id  = "odom"
        poseMsg.header.stamp     = stamp
        poseMsg.pose.position.x  = states[0]
        poseMsg.pose.position.y  = states[1]
        poseMsg.pose.position.z  = 0.0
        '''
        State index and definition
            states : [x, y, yaw, vx]
        '''
        roll  = 0.0
        pitch = 0.0
        yaw   = states[2]
        q     = quaternion_from_euler(roll, pitch, yaw)
        poseMsg.pose.orientation.x = q[0]
        poseMsg.pose.orientation.y = q[1]
        poseMsg.pose.orientation.z = q[2]
        poseMsg.pose.orientation.w = q[3]
        self.pose_pub.publish(poseMsg)
        self.publishTF(stamp, poseMsg)

    def publishJoy(self, stamp, inputs, toggle_switch):
        joyMsg = Joy()
        joyMsg.header.stamp   = stamp
        joyMsg.axes           = inputs
        joyMsg.buttons        = [toggle_switch]
        self.input_pub.publish(joyMsg)

    def publishBodyOdom(self, stamp, states):
        odomMsg = Odometry()
        odomMsg.header.frame_id       = "odom"
        odomMsg.header.stamp          = stamp
        odomMsg.pose.pose.position.x  = states[0]
        odomMsg.pose.pose.position.y  = states[1]
        odomMsg.pose.pose.position.z  = 0

        roll  = 0.0
        pitch = 0.0
        yaw   = states[2]
        q     = quaternion_from_euler(roll, pitch, yaw)
        odomMsg.pose.pose.orientation.x = q[0]
        odomMsg.pose.pose.orientation.y = q[1]
        odomMsg.pose.pose.orientation.z = q[2]
        odomMsg.pose.pose.orientation.w = q[3]

        odomMsg.twist.twist.linear.x   = states[3]
        odomMsg.twist.twist.linear.y   = 0.0
        odomMsg.twist.twist.linear.z   = 0.0
        # roll_dot  = 0.0
        # pitch_dot = 0.0
        # yaw_dot   = states[6]
        # odomMsg.twist.twist.angular.x = roll_dot
        # odomMsg.twist.twist.angular.y = pitch_dot
        # odomMsg.twist.twist.angular.z = self.states_dot[2]#yaw_dot
        self.bodyOdom_pub.publish(odomMsg)

    def publishPoseOdom(self, stamp, states):
        odomMsg = Odometry()
        odomMsg.header.frame_id       = "odom"
        odomMsg.header.stamp          = stamp
        odomMsg.pose.pose.position.x  = states[0]
        odomMsg.pose.pose.position.y  = states[1]
        odomMsg.pose.pose.position.z  = 0

        roll  = 0.0
        pitch = 0.0
        yaw   = states[2]
        q     = quaternion_from_euler(roll, pitch, yaw)
        odomMsg.pose.pose.orientation.x = q[0]
        odomMsg.pose.pose.orientation.y = q[1]
        odomMsg.pose.pose.orientation.z = q[2]
        odomMsg.pose.pose.orientation.w = q[3]

        vx, vy = self.rotate([states[3], 0.0], yaw)
        odomMsg.twist.twist.linear.x   = vx
        odomMsg.twist.twist.linear.y   = vy
        odomMsg.twist.twist.linear.z   = 0.0
        # roll_dot  = 0.0
        # pitch_dot = 0.0
        # yaw_dot   = 0.0
        # odomMsg.twist.twist.angular.x = roll_dot
        # odomMsg.twist.twist.angular.y = pitch_dot
        # odomMsg.twist.twist.angular.z = self.states_dot[2]#yaw_dot
        self.poseOdom_pub.publish(odomMsg)
        self.publishRvizVisualization(odomMsg)

    def publishAccel(self, stamp, states_dot):
        """
        State index and definition
            states_dot : [x_dot, y_dot, yaw_dot, vx_dot]
        """
        accelMsg = Accel()
        accelMsg.linear.x  = states_dot[3]
        accelMsg.linear.y  = 0.0
        accelMsg.linear.z  = 0.0
        accelMsg.angular.x = 0.0
        accelMsg.angular.y = 0.0
        accelMsg.angular.z = 0.0
        self.accel_pub.publish(accelMsg)

    def publishRvizVisualization(self,odomMsg):
        ego_model_marker = Marker()
        ego_model_marker.header  = odomMsg.header
        ego_model_marker.header.frame_id  = self.namespace[1:] + "base_link"
        ego_model_marker.type    = Marker.MESH_RESOURCE
        ego_model_marker.action  = Marker.ADD
        ego_model_marker.pose.orientation.w = 1.

        scale = self.length/2.5
        ego_model_marker.pose.position.x = self.length * 0.5
        ego_model_marker.scale.x = scale
        ego_model_marker.scale.y = scale
        ego_model_marker.scale.z = scale


        ego_model_marker.color.a = 0.85
        ego_model_marker.color.r = 0.7
        ego_model_marker.color.g = 0.7
        ego_model_marker.color.b = 0.7

        ego_model_marker.mesh_resource = "package://eurecarr_vehicle_sim/models/car.dae"

        self.ego_model_marker_pub.publish(ego_model_marker)

    def save_state_hist(self):
        data = self.data
        name = "sim_data_"
        timelabel = strftime("%Y-%m-%d-%H-%M", gmtime())
        np.savetxt(name+timelabel+".csv", data, delimiter=",")



def main():
    rospy.init_node('simulate_dynamics')

    dt              = 0.05 # simulation timestep [sec]
    realtime_factor = 1.
    Hz              = int(1/dt)
    stateDim        = 4
    inputDim        = 2
    sim             = SimulateStep(stateDim, inputDim, dt)
    sim.namespace   = rospy.get_namespace()
    rate            = rospy.Rate(Hz*realtime_factor)

    while not rospy.is_shutdown():
        sim.one_step_forward(sim.inputs)

        rate.sleep()

    # Uncomment to save data on exit
    # rospy.on_shutdown(sim.save_state_hist)

if __name__ == "__main__":
    main()