# -*- coding: utf-8 -*-
import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from enum import Enum
# from forklift_msg.msg import meteorcar
import statistics
class Action():
    def __init__(self, Subscriber):
        # cmd_vel
        self.cmd_vel = cmd_vel()
        self.Subscriber = Subscriber
        self.NearbySequence = Enum('NearbySequence', 'initial_turn go_straight turn_right parking ')
        self.current_nearby_sequence = self.NearbySequence.initial_turn.value
        # Odometry_param
        self.is_odom_received = False
        self.robot_2d_pose_x = 0.0
        self.robot_2d_pose_y = 0.0
        self.robot_2d_theta = 0.0
        self.initial_robot_pose_x = 0.0
        self.initial_robot_pose_y = 0.0
        # AprilTag_param
        self.is_marker_pose_received = False
        self.marker_2d_pose_x = 0.0
        self.marker_2d_pose_y = 0.0
        self.marker_2d_theta = 0.0
        self.initial_marker_pose_x = 0.0
        self.initial_marker_pose_y = 0.0
        self.initial_marker_pose_theta = 0.0
        # other
        self.check_wait_time = 0
        self.is_triggered = False

    def SpinOnce(self):
        (self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta, \
         self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta)=self.Subscriber.SpinOnce()
                
    def fnSeqChangingDirection(self, desired_angle):
        self.SpinOnce()
        desired_angle_turn = -1. *  math.atan2(self.marker_2d_pose_y, self.marker_2d_pose_x)
        # print(desired_angle_turn)
        
        if desired_angle_turn <0:
            desired_angle_turn = desired_angle_turn + math.pi
        else:
            desired_angle_turn = desired_angle_turn - math.pi

        self.cmd_vel.fnTurn(desired_angle_turn)
        # print(abs(desired_angle_turn))
        if abs(desired_angle_turn) < desired_angle  :
            self.cmd_vel.fnStop()
            if self.check_wait_time > 10 :
                self.check_wait_time = 0
                return True
            else:
                self.check_wait_time =self.check_wait_time  +1
                return False
        else:
            self.check_wait_time =0
            return False
        
    def fnSeqChangingtheta(self, threshod):
        if self.Subscriber.flag or True:
            self.SpinOnce()
            self.marker_2d_theta= self.TrustworthyMarker2DTheta(1)
            desired_angle_turn = -self.marker_2d_theta
            if abs(desired_angle_turn) < threshod  :
                self.cmd_vel.fnStop()
                rospy.sleep(0.1)
                return True
            else:
                self.TurnByTime(desired_angle_turn*2, 1)
                return False
        # else:
        #     self.cmd_vel.fnStop() 
        #     return None
        
    def TurnByTime(self, desired_angle_turn, time):
        initial_time = rospy.Time.now().secs
        while(abs(initial_time - rospy.Time.now().secs) < time):
            self.cmd_vel.fnTurn(desired_angle_turn)
            rospy.sleep(0.1)
        self.cmd_vel.fnStop()
        

    # def fnseqturn(self, threshod):#旋轉到後退所需角度(暫時沒有用到)
    #     self.SpinOnce()
    #     if(self.marker_2d_pose_y > 0):
    #         self.marker_2d_theta = self.marker_2d_theta + 0.15
    #     else:
    #         self.marker_2d_theta = self.marker_2d_theta - 0.15

    #     desired_angle_turn = -self.marker_2d_theta
    #     if abs(desired_angle_turn) < threshod  :
    #         self.cmd_vel.fnStop()
    #         rospy.sleep(0.1)
    #         if self.check_wait_time > 5 :
    #             self.check_wait_time = 0
    #             return True
    #         else:
    #             self.check_wait_time =self.check_wait_time  +1
    #             return False
    #     else:
    #         self.cmd_vel.fnTurn(desired_angle_turn)
    #         rospy.sleep(0.3)
    #         self.check_wait_time =0
    #         return False
        
    def fnSeqMovingNearbyParkingLot(self):
        self.SpinOnce()
        if self.current_nearby_sequence == self.NearbySequence.initial_turn.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_theta = self.robot_2d_theta
                self.initial_robot_pose_x = self.robot_2d_pose_x
                self.initial_robot_pose_y = self.robot_2d_pose_y

                self.initial_marker_pose_theta = self.TrustworthyMarker2DTheta(3)
                self.initial_marker_pose_x = self.marker_2d_pose_x
                # print("initial_marker_pose_theta ", self.initial_marker_pose_theta)
                # decide doing fnSeqMovingNearbyParkingLot or not
                desired_dist = -1* self.initial_marker_pose_x * abs(math.cos((math.pi / 2.) - self.initial_marker_pose_theta))
                if abs(desired_dist) < 0.2: # 0.15
                    return True
            
            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = (math.pi / 2.0) + self.initial_marker_pose_theta - (self.robot_2d_theta - self.initial_robot_pose_theta)
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = -(math.pi / 2.0) + self.initial_marker_pose_theta - (self.robot_2d_theta - self.initial_robot_pose_theta)
            

            
            desired_angle_turn = -1. * desired_angle_turn
            self.cmd_vel.fnTurn(desired_angle_turn)

            if abs(desired_angle_turn) < 0.03:
                self.cmd_vel.fnStop()
                if self.check_wait_time >20:
                    self.check_wait_time = 0
                    self.current_nearby_sequence = self.NearbySequence.go_straight.value
                    self.is_triggered = False
                else:
                    self.check_wait_time =self.check_wait_time +1
            elif abs(desired_angle_turn) < 0.045 and self.check_wait_time :
                self.cmd_vel.fnStop()
                if self.check_wait_time > 20:
                    self.check_wait_time = 0
                    self.current_nearby_sequence = self.NearbySequence.go_straight.value
                    self.is_triggered = False
                else:
                    self.check_wait_time =self.check_wait_time +1
            else:
                self.check_wait_time =0    

        elif self.current_nearby_sequence == self.NearbySequence.go_straight.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_x = self.robot_2d_pose_x
                self.initial_robot_pose_y = self.robot_2d_pose_y

            dist_from_start = self.fnCalcDistPoints(self.initial_robot_pose_x, self.robot_2d_pose_x, self.initial_robot_pose_y, self.robot_2d_pose_y)
            desired_dist = -1* self.initial_marker_pose_x * abs(math.cos((math.pi / 2.) - self.initial_marker_pose_theta))
            # HACK: self spin error correct
            
            desired_dist = desired_dist

            remained_dist = desired_dist - dist_from_start 
            if remained_dist < 0  :remained_dist =0

            
            self.cmd_vel.fnGoStraight(desired_dist)

            if abs(remained_dist) < 0.07:
                self.cmd_vel.fnStop()
                self.current_nearby_sequence = self.NearbySequence.turn_right.value
                self.is_triggered = False


        elif self.current_nearby_sequence == self.NearbySequence.turn_right.value:
            if self.is_triggered == False:
                self.is_triggered = True
                self.initial_robot_pose_theta = self.robot_2d_theta

            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = (math.pi / 2.0) + (self.robot_2d_theta - self.initial_robot_pose_theta)
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = -(math.pi / 2.0) + (self.robot_2d_theta - self.initial_robot_pose_theta)
            
            self.cmd_vel.fnTurn(desired_angle_turn)
            if abs(desired_angle_turn) < 0.03:
                self.cmd_vel.fnStop()
                if self.check_wait_time > 10:
                    self.check_wait_time = 0
                    self.current_nearby_sequence = self.NearbySequence.parking.value
                    self.is_triggered = False
                    return True                
                else:
                    self.check_wait_time =self.check_wait_time  +1
            elif abs(desired_angle_turn) < 0.045 and self.check_wait_time:
                self.cmd_vel.fnStop()
                if self.check_wait_time > 10:
                    self.check_wait_time = 0
                    self.current_nearby_sequence = self.NearbySequence.parking.value
                    self.is_triggered = False
                    return True                
                else:
                    self.check_wait_time =self.check_wait_time  +1
            else:
                self.check_wait_time =0    
        return False

    def fnSeqParking(self, parking_dist):
        if self.Subscriber.flag:
            self.SpinOnce()
            desired_angle_turn = math.atan2(self.marker_2d_pose_y - 0, self.marker_2d_pose_x - 0)
            if desired_angle_turn <0:
                desired_angle_turn = desired_angle_turn + math.pi
            else:
                desired_angle_turn = desired_angle_turn - math.pi


            self.cmd_vel.fnTrackMarker(-desired_angle_turn)

            if (abs(self.marker_2d_pose_x) < parking_dist)  :
                self.cmd_vel.fnStop()
                if self.check_wait_time > 10:
                    self.check_wait_time = 0
                    return True
                else:
                    self.check_wait_time =self.check_wait_time  +1
            # elif (abs(self.marker_2d_pose_x) < parking_dist) and self.check_wait_time:
            #     self.cmd_vel.fnStop()
            #     if self.check_wait_time > 10:
            #         self.check_wait_time = 0
            #         return True
            #     else:
            #         self.check_wait_time =self.check_wait_time  +1
            else:
                self.check_wait_time =0
                return False
            
        else:
            self.cmd_vel.fnStop() 
            return None
        
    def fnSeqdecide(self, decide_dist):#decide_dist偏離多少公分要後退
        if self.Subscriber.flag:
            self.SpinOnce()
            dist = self.marker_2d_pose_y
            if  abs(dist) < abs(decide_dist):
                return True
            else:
                return False
        else: return None
        

    def fnseqmove_to_marker_dist(self, marker_dist): #(使用marker)前後移動到距離marker_dist公尺的位置
        if self.Subscriber.flag:
            self.SpinOnce()
            if(marker_dist < 2.0):
                threshold = 0.015
            else:
                threshold = 0.03

            dist = math.sqrt(self.marker_2d_pose_x**2 + self.marker_2d_pose_y**2)
            
            if dist < (marker_dist-threshold):
                self.cmd_vel.fnGoStraight(-(marker_dist - dist))
                return False
            elif dist > (marker_dist+threshold):
                self.cmd_vel.fnGoStraight(-(marker_dist - dist))
                return False
            else:
                self.cmd_vel.fnStop()
                return True
        else: 
            self.cmd_vel.fnStop()
            return None
            
    def fnCalcDistPoints(self, x1, x2, y1, y2):
        return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

    def TrustworthyMarker2DTheta(self, time):
        marker_2d_theta_list = [0.0]
        initial_time = rospy.Time.now().secs
        
        while(abs(initial_time - rospy.Time.now().secs) <= time):
            self.SpinOnce()
            marker_2d_theta_list.append(self.marker_2d_theta)
            # print("self.marker_2d_theta", self.marker_2d_theta)
            rospy.sleep(0.05)
        # print("marker_2d_theta_list", marker_2d_theta_list)
        threshold = 0.5
        mean = statistics.mean(marker_2d_theta_list)
        stdev = statistics.stdev(marker_2d_theta_list)
        upcutoff = mean + threshold * stdev
        downcutoff = mean - threshold * stdev
        clean_list = []
        for i in marker_2d_theta_list:
            if(i > downcutoff and i < upcutoff):
               clean_list.append(i)
               
        return statistics.median(clean_list)
     
class cmd_vel():
    def __init__(self):
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.front = False

    def cmd_pub(self, twist):
        if not self.front:
            twist.linear.x = -twist.linear.x

        if twist.linear.x == 0:
            if twist.angular.z > 0.2:
                twist.angular.z = 0.2
            elif twist.angular.z < -0.2:
                twist.angular.z = -0.2
            elif twist.angular.z > 0 and twist.angular.z < 0.1:
                twist.angular.z = 0.1
            elif twist.angular.z < 0 and twist.angular.z > -0.1:
                twist.angular.z = -0.1
        else :
            if twist.angular.z > 0.08:
                twist.angular.z = 0.08
            elif twist.angular.z < -0.08:
                twist.angular.z = -0.08
            elif abs(twist.angular.z) < 0.05:
                twist.angular.z = 0.0

        if twist.linear.x > 0.2:
            twist.linear.x = 0.2
        elif twist.linear.x < -0.2:
            twist.linear.x = -0.2
        elif twist.linear.x > 0 and twist.linear.x < 0.05:
            twist.linear.x = 0.05
        elif twist.linear.x < 0 and twist.linear.x > -0.05:
            twist.linear.x = -0.05  

        self.pub_cmd_vel.publish(twist)

    def fnStop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        self.cmd_pub(twist)

    def fnTurn(self, theta):
        Kp = 0.2 #1.0
        angular_z = Kp * theta
        # if abs(angular_z)  < 0.05: angular_z=0.05

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        
        twist.angular.z = -angular_z

        self.cmd_pub(twist)

    def fnGoStraight(self,v):
        twist = Twist()
        twist.linear.x = v*0.8
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        self.cmd_pub(twist)

    def fnGoBack(self):
        twist = Twist()
        twist.linear.x = -0.1
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        self.cmd_pub(twist)

    def fnTrackMarker(self, theta):
        Kp = 4.0 #6.5

        twist = Twist()
        twist.linear.x = 0.1
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0

        twist.angular.z = -Kp * theta
        self.cmd_pub(twist)

  
