#!/usr/bin/env python
import os
import mujoco as mj
import numpy as np
# import ros2_numpy as rnp

from mujoco_sim_py.mujoco_base import MuJoCoBase
from mujoco.glfw import glfw

import rclpy
from rclpy.node import Node
import ament_index_python
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Pose, Twist


class HectorSim(MuJoCoBase):
    def __init__(self, xml_path):
        super().__init__(xml_path)
        self.simend = 1000.0
        print('Total number of DoFs in the model:', self.model.nv)
        print('Generalized positions:', self.data.qpos)
        print('Generalized velocities:', self.data.qvel)
        print('Actuator forces:', self.data.qfrc_actuator)
        print('Actoator controls:', self.data.ctrl)

        # show the model
        mj.mj_step(self.model, self.data)
        # enable contact force visualization
        self.opt.flags[mj.mjtVisFlag.mjVIS_CONTACTFORCE] = True

        # get framebuffer viewport
        viewport_width, viewport_height = glfw.get_framebuffer_size(self.window)
        viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
        # Update scene and render
        mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam, mj.mjtCatBit.mjCAT_ALL.value, self.scene)
        mj.mjr_render(viewport, self.scene, self.context)

        # Set publisher
        self.pub_joints = rospy.Publisher(Float32MultiArray, 'jointsPosVel', 10)
        self.pub_pose = rospy.Publisher(Pose, 'bodyPose', 10)
        self.pub_twist = rospy.Publisher(Twist, 'bodyTwist', 10)
        # Set subscriber
        self.sub_joints = self.create_subscription(Float32MultiArray, 'jointsTorque', self.controlCallback, 10)
        self.sub_joints

    def controlCallback(self, data):
        self.data.ctrl[:] = data.data[:]

    def reset(self):
        # Set camera configuration
        self.cam.azimuth = 89.608063
        self.cam.elevation = -11.588379
        self.cam.distance = 5.0
        self.cam.lookat = np.array([0.0, 0.0, 1.5])

    def simulate(self):
        while not glfw.window_should_close(self.window):
            simstart = self.data.time

            while (self.data.time - simstart <= 1.0/60.0 and not self.pause_flag):
                # Step simulation environment
                mj.mj_step(self.model, self.data)
                # * Publish joint positions and velocities
                jointsPosVel = Float32MultiArray()
                # get last 10 element of qpos and qvel
                qp = self.data.qpos[-10:].copy()
                qv = self.data.qvel[-10:].copy()
                jointsPosVel.data = np.concatenate((qp, qv)).tolist()

                # * Publish body pose
                bodyPose = Pose()
                pos = self.data.sensor('BodyPos').data.copy()
                ori = self.data.sensor('BodyQuat').data.copy()
                # pos = self.data.qpos[:3].copy()
                # ori = self.data.qpos[3:7].copy()
                bodyPose.position.x = pos[0]
                bodyPose.position.y = pos[1]
                bodyPose.position.z = pos[2]
                bodyPose.orientation.x = ori[1]
                bodyPose.orientation.y = ori[2]
                bodyPose.orientation.z = ori[3]
                bodyPose.orientation.w = ori[0]
                
                # * Publish body twist
                bodyTwist = Twist()
                # get body velocity in world frame
                vel = self.data.sensor('BodyVel').data.copy()
                vel = self.data.qvel[:3].copy()
                angVel = self.data.sensor('BodyGyro').data.copy()
                bodyTwist.linear.x = vel[0]
                bodyTwist.linear.y = vel[1]
                bodyTwist.linear.z = vel[2]
                bodyTwist.angular.x = angVel[0]
                bodyTwist.angular.y = angVel[1]
                bodyTwist.angular.z = angVel[2]

                self.pub_joints.publish(jointsPosVel)
                self.pub_pose.publish(bodyPose)
                self.pub_twist.publish(bodyTwist)

            if self.data.time >= self.simend:
                break

            # get framebuffer viewport
            viewport_width, viewport_height = glfw.get_framebuffer_size(self.window)
            viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

            # Update scene and render
            mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam, mj.mjtCatBit.mjCAT_ALL.value, self.scene)
            mj.mjr_render(viewport, self.scene, self.context)

            # swap OpenGL buffers (blocking call due to v-sync)
            glfw.swap_buffers(self.window)

            # process pending GUI events, call GLFW callbacks
            glfw.poll_events()

        glfw.terminate()


def main(args=None):
    # ros init
    rclpy.init(args=args)
    # rospy.init_node('hector_sim', anonymous=True)

    # get xml path
    hector_desc_path = get_package_share_directory('hector_description')
    xml_path = os.path.join(hector_desc_path, "mjcf/hector.xml")

    mujoco_node = HectorSim(xml_path)
    mujoco_node.reset()
    mujoco_node.simulate()

    # rclpy.spin(mujoco_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # mujoco_node.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
