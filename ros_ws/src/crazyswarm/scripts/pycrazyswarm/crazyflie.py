#!/usr/bin/env python


import sys
import yaml
import rospy
import numpy as np
import time
from std_srvs.srv import Empty
from crazyflie_driver.srv import *
from crazyflie_driver.msg import TrajectoryPolynomialPiece, FullState, Hover
from tf import TransformListener
from std_msgs.msg import Empty as Empty_msg
import geometry_msgs.msg

def arrayToGeometryPoint(a):
    return geometry_msgs.msg.Point(a[0], a[1], a[2])

class TimeHelper:
    def __init__(self):
        rospy.wait_for_service("/next_phase")
        self.nextPhase = rospy.ServiceProxy("/next_phase", Empty)

    def time(self):
        return time.time()

    def sleep(self, duration):
        time.sleep(duration)

    def nextPhase(self):
        self.nextPhase()


class Crazyflie:
    def __init__(self, id, initialPosition, tf):
        self.id = id
        prefix = "/cf" + str(id)
        self.initialPosition = np.array(initialPosition)

        self.tf = tf

        rospy.wait_for_service(prefix + "/set_group_mask")
        self.setGroupMaskService = rospy.ServiceProxy(prefix + "/set_group_mask", SetGroupMask)
        rospy.wait_for_service(prefix + "/takeoff")
        self.takeoffService = rospy.ServiceProxy(prefix + "/takeoff", Takeoff)
        rospy.wait_for_service(prefix + "/land")
        self.landService = rospy.ServiceProxy(prefix + "/land", Land)
        # rospy.wait_for_service(prefix + "/stop")
        # self.stopService = rospy.ServiceProxy(prefix + "/stop", Stop)
        rospy.wait_for_service(prefix + "/go_to")
        self.goToService = rospy.ServiceProxy(prefix + "/go_to", GoTo)
        rospy.wait_for_service(prefix + "/upload_trajectory")
        self.uploadTrajectoryService = rospy.ServiceProxy(prefix + "/upload_trajectory", UploadTrajectory)
        rospy.wait_for_service(prefix + "/start_trajectory")
        self.startTrajectoryService = rospy.ServiceProxy(prefix + "/start_trajectory", StartTrajectory)
        rospy.wait_for_service(prefix + "/update_params")
        self.updateParamsService = rospy.ServiceProxy(prefix + "/update_params", UpdateParams)

        ### added by The-SS begin
        self.pubCmdFullState = rospy.Publisher(prefix + '/cmd_full_state', FullState, queue_size=1)
        self.pubCmdStop = rospy.Publisher(prefix + '/cmd_stop', Empty_msg, queue_size=1)

        self.worldFrame = '/world'
        ### added by The-SS end

    ### added by The-SS begin
    def cmdStop(self):
        '''
        Cuts off power to the drone (instanteous effect)
        '''
        empty = Empty_msg()
        self.pubCmdStop.publish(empty)

    def cmdFullState(self, pose_data, twist_data, acc_data):
        '''
        pose_data: list/array with position and quaternion rotation (x, y, z, qx, qy, qz, qw)
        twist_data: list/array with linear and angular velocities (vx, vy, vz, roll_rate, pitch_rate, yaw_rate)
        acc_data: list/array with acceleration (ax, ay, az)
        '''
        state = FullState()
        pose = geometry_msgs.msg.Pose()
        twist = geometry_msgs.msg.Twist()
        acc = geometry_msgs.msg.Vector3()

        pose.position.x = pose_data[0]
        pose.position.y = pose_data[1]
        pose.position.z = pose_data[2]
        pose.orientation.x = pose_data[3]
        pose.orientation.y = pose_data[4]
        pose.orientation.z = pose_data[5]
        pose.orientation.w = pose_data[6]

        twist.linear.x = twist_data[0]
        twist.linear.y = twist_data[1]
        twist.linear.z = twist_data[2]
        twist.angular.x = twist_data[3]
        twist.angular.y = twist_data[4]
        twist.angular.z = twist_data[5]

        acc.x = acc_data[0]
        acc.y = acc_data[1]
        acc.z = acc_data[2]

        state.pose = pose
        state.twist = twist
        state.acc = acc
        state.header.seq +=1
        state.header.stamp = rospy.Time.now()
        state.header.frame_id = self.worldFrame

        self.pubCmdFullState.publish(state)

    ### added by The-SS end

    def setGroupMask(self, groupMask):
        self.setGroupMaskService(groupMask)

    def takeoff(self, targetHeight, duration, groupMask = 0):
        self.takeoffService(groupMask, targetHeight, rospy.Duration.from_sec(duration))

    def land(self, targetHeight, duration, groupMask = 0):
        self.landService(groupMask, targetHeight, rospy.Duration.from_sec(duration))

    def stop(self, groupMask = 0):
        self.stopService(groupMask)

    def goTo(self, goal, yaw, duration, relative = False, groupMask = 0):
        gp = arrayToGeometryPoint(goal)
        self.goToService(groupMask, relative, gp, yaw, rospy.Duration.from_sec(duration))

    def uploadTrajectory(self, trajectoryId, pieceOffset, trajectory):
        pieces = []
        for poly in trajectory.polynomials:
            piece = TrajectoryPolynomialPiece()
            piece.duration = rospy.Duration.from_sec(poly.duration)
            piece.poly_x   = poly.px.p
            piece.poly_y   = poly.py.p
            piece.poly_z   = poly.pz.p
            piece.poly_yaw = poly.pyaw.p
            pieces.append(piece)
        self.uploadTrajectoryService(trajectoryId, pieceOffset, pieces)

    def startTrajectory(self, trajectoryId, timescale = 1.0, reverse = False, relative = True, groupMask = 0):
        self.startTrajectoryService(groupMask, trajectoryId, timescale, reverse, relative)

    def position(self):
        self.tf.waitForTransform("/world", "/cf" + str(self.id), rospy.Time(0), rospy.Duration(10))
        position, quaternion = self.tf.lookupTransform("/world", "/cf" + str(self.id), rospy.Time(0))
        return np.array(position)

    ### Added by The-SS begin
    def rotation(self):
        self.tf.waitForTransform("/world", "/cf" + str(self.id), rospy.Time(0), rospy.Duration(10))
        position, quaternion = self.tf.lookupTransform("/world", "/cf" + str(self.id), rospy.Time(0))
        return np.array(quaternion)
    ### Added by The-SS end

    def getParam(self, name):
        return rospy.get_param(self.prefix + "/" + name)

    def setParam(self, name, value):
        rospy.set_param(self.prefix + "/" + name, value)
        self.updateParamsService([name])

    def setParams(self, params):
        for name, value in params.iteritems():
            rospy.set_param(self.prefix + "/" + name, value)
        self.updateParamsService(params.keys())


class CrazyflieServer:
    def __init__(self):
        rospy.init_node("CrazyflieAPI", anonymous=False)
        rospy.wait_for_service("/emergency")
        self.emergencyService = rospy.ServiceProxy("/emergency", Empty)
        rospy.wait_for_service("/takeoff")
        self.takeoffService = rospy.ServiceProxy("/takeoff", Takeoff)
        rospy.wait_for_service("/land")
        self.landService = rospy.ServiceProxy("/land", Land)
        # rospy.wait_for_service("/stop")
        # self.stopService = rospy.ServiceProxy("/stop", Stop)
        # rospy.wait_for_service("/go_to")
        # self.goToService = rospy.ServiceProxy("/go_to", GoTo)
        rospy.wait_for_service("/start_trajectory");
        self.startTrajectoryService = rospy.ServiceProxy("/start_trajectory", StartTrajectory)
        # rospy.wait_for_service("/update_params")
        # self.updateParamsService = rospy.ServiceProxy("/update_params", UpdateParams)

        with open("../launch/crazyflies.yaml", 'r') as ymlfile:
            cfg = yaml.load(ymlfile)

        self.tf = TransformListener()

        self.crazyflies = []
        self.crazyfliesById = dict()
        for crazyflie in cfg["crazyflies"]:
            id = int(crazyflie["id"])
            initialPosition = crazyflie["initialPosition"]
            cf = Crazyflie(id, initialPosition, self.tf)
            self.crazyflies.append(cf)
            self.crazyfliesById[id] = cf

    def emergency(self):
        self.emergencyService()

    def takeoff(self, targetHeight, duration, groupMask = 0):
        self.takeoffService(groupMask, targetHeight, rospy.Duration.from_sec(duration))

    def land(self, targetHeight, duration, groupMask = 0):
        self.landService(groupMask, targetHeight, rospy.Duration.from_sec(duration))

    # def stop(self, groupMask = 0):
    #     self.stopService(groupMask)

    # def goTo(self, goal, yaw, duration, groupMask = 0):
    #     gp = arrayToGeometryPoint(goal)
    #     self.goToService(groupMask, True, gp, yaw, rospy.Duration.from_sec(duration))

    def startTrajectory(self, trajectoryId, timescale = 1.0, reverse = False, relative = True, groupMask = 0):
        self.startTrajectoryService(groupMask, trajectoryId, timescale, reverse, relative)

    # def setParam(self, name, value, group = 0):
    #     rospy.set_param("/cfgroup" + str(group) + "/" + name, value)
    #     self.updateParamsService(group, [name])
