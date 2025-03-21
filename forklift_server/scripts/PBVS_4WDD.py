# -*- coding: utf-8 -*-
import rospy
import forklift_server.msg
from enum import Enum
from PBVS_Action_4WDD import Action
# from forklift_msg.msg import meteorcar
class PBVS():
    ParkingSequence = Enum( 'ParkingSequence', \
                            'changing_direction_1 \
                            Changingtheta \
                            decide \
                            back_turn \
                            back \
                            moving_nearby_parking_lot \
                            parking \
                            stop')
    

    def __init__(self, _as, subscriber, mode):
        self._as = _as
        self._feedback = forklift_server.msg.PBVSFeedback()
        self._result = forklift_server.msg.PBVSResult()
        self.subscriber = subscriber
        self.mode = mode.command
        self.layer = mode.layer
        self.ActionCode=mode.ActionCode
        self.ShelfParameter=mode.ShelfParameter
        self. UpDownPosition=mode.UpDownPosition
        self. ForrwardBackwardPosition=mode.ForrwardBackwardPosition
        self. TilePositionv=mode.TilePosition
        self. MovePosition=mode.MovePosition
        self.Action = Action(self.subscriber)
        self.init_PBVS_parame()
        

    def init_PBVS_parame(self):
        self.is_sequence_finished = False
        if self.mode == "parking_bodycamera":
            self.subscriber.updown = True
            self.subscriber.offset_x = rospy.get_param(rospy.get_name() + "/bodycamera_tag_offset_x", 0.325)
            self.ChangingDirection_threshold = rospy.get_param(rospy.get_name() + "/bodycamera_ChangingDirection_threshold", 0.01)
            self.Parking_distance = rospy.get_param(rospy.get_name() + "/bodycamera_parking_stop", 1.8)
            self.Changingtheta_threshod = rospy.get_param(rospy.get_name() + "/bodycamera_Changingtheta_threshold", 0.1)
            self.decide_distance = rospy.get_param(rospy.get_name() + "/bodycamera_decide_distance", 0.04)
            self.back_distance = rospy.get_param(rospy.get_name() + "/bodycamera_back_distance", 3.0)
            # self.current_parking_sequence = self.ParkingSequence.init_fork.value #for 大車
            self.current_parking_sequence = self.ParkingSequence.changing_direction_1.value # for 小車
            self.main_loop()

        else:
            rospy.logwarn("mode is not correct")
            self._result.result = 'fail'
            self._as.set_succeeded(self._result)
            return

   
    def main_loop(self):
        r = rospy.Rate(10)
        while(not rospy.is_shutdown()):
            if(self.PBVS()):
                break
            r.sleep()

    def __del__(self):
        rospy.logwarn('delet PBVS')
        
    def PBVS(self):
        if self._as.is_preempt_requested():
            rospy.logwarn('PBVS Preempted')
            self.current_parking_sequence = self.ParkingSequence.stop.value
            
            
        self._feedback.feedback = str(self.ParkingSequence(self.current_parking_sequence))
        self._as.publish_feedback(self._feedback)
            # ============parking============
        if self.current_parking_sequence == self.ParkingSequence.changing_direction_1.value:
            self.is_sequence_finished = self.Action.fnSeqChangingDirection(self.ChangingDirection_threshold)
            print("here-1")
            if self.is_sequence_finished == True:
                self.current_parking_sequence = self.ParkingSequence.moving_nearby_parking_lot.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.moving_nearby_parking_lot.value:
            self.is_sequence_finished = self.Action.fnSeqMovingNearbyParkingLot()
            print("here-2")
            if self.is_sequence_finished == True:
                self.current_parking_sequence = self.ParkingSequence.parking.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.parking.value:
            self.is_sequence_finished = self.Action.fnSeqParking(self.Parking_distance)
            print("here-3")
            if self.is_sequence_finished == True:
                self.current_parking_sequence = self.ParkingSequence.Changingtheta.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.Changingtheta.value:
            self.is_sequence_finished = self.Action.fnSeqChangingtheta(self.Changingtheta_threshod)
            print("here-4")
            if self.is_sequence_finished == True:
                self.current_parking_sequence = self.ParkingSequence.decide.value
                self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.decide.value:
            self.is_sequence_finished = self.Action.fnSeqdecide(self.decide_distance)
            print("here-5")
            if self.is_sequence_finished == True:
                self.current_parking_sequence = self.ParkingSequence.stop.value
                self.is_sequence_finished = False

            elif self.is_sequence_finished == False:
                self.current_parking_sequence = self.ParkingSequence.back.value
                self.is_sequence_finished = False

        # elif self.current_parking_sequence == self.ParkingSequence.back_turn.value:
        #     turn_threshod = 0.04
        #     self.is_sequence_finished = self.Action.fnseqturn(self.back_distance, turn_threshod)
            
        #     if self.is_sequence_finished == True:
        #         self.current_parking_sequence = self.ParkingSequence.back.value
        #         self.is_sequence_finished = False

        elif self.current_parking_sequence == self.ParkingSequence.back.value:
            self.is_sequence_finished = self.Action.fnseqmove_to_marker_dist(self.back_distance)
            print("here-6")
            if self.is_sequence_finished == True:
                self.current_parking_sequence = self.ParkingSequence.parking.value
                self.is_sequence_finished = False
        # ============stop============
        elif self.current_parking_sequence == self.ParkingSequence.stop.value:
            # self.window.destroy()
            rospy.sleep(1)
            return True

        return False
            

