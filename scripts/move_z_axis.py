#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from xarm_msgs.srv import Move, MoveRequest

class move_z:
    def __init__(self):
        ### Node init ###
        rospy.init_node("move_z_gesture", anonymous=True)
        rospy.loginfo("Starting move_z.")

        ### Variables ###
        self.myMove = MoveRequest()
        self.z_pose = 200           #(mm)
        self.gesture = 'None'
        self.step = 0
        
        ### Constants ###
        self.myMove.mvvelo = 200    #(mm/s)
        self.myMove.mvacc = 2000    #(mm/s²)
        self.z_0 = 200              #(mm)
        self.z_1 = 100              #(mm)
        
        ### Subscribers ###
        rospy.Subscriber("/hand/gesture", String, self.gesture_callback)

        ### Service proxy ###
        # Wait for move line service to be available 
        rospy.loginfo('Waiting for move_line service to be available ...')
        rospy.wait_for_service('/ufactory/move_line')
        self.move_proxy = rospy.ServiceProxy('/ufactory/move_line', Move)
        rospy.loginfo('Service available')

        rospy.loginfo('Going to initial position')
        self.move_line_service_call(self.z_pose)

    def move_line_service_call(self, z_pose):
        # Set the target pose
        self.myMove.pose = [200,0,z_pose,3.14,0,0]
        # Call the service
        self.move_proxy(self.myMove)
        rospy.loginfo('Service called successfuly to {} z position'.format(z_pose))

    def calculate_z_pose(self,step):
        displacement = (self.z_1 - self.z_0) * step/10
        z_pose = displacement + z_pose

    def gesture_callback(self,data):
        self.gesture = data.data

    def main_loop(self):
        rospy.on_shutdown(self.cleanup)
        r = rospy.Rate(.5)
        rospy.loginfo('Starting main loop')
        while not rospy.is_shutdown():
            if self.gesture == 'Thumb_Up':
                if self.step == 10: continue
                self.step += 1
            elif self.gesture == 'Thumb_Down':
                if self.step == 0: continue
                self.step -= 1
            else : 
                continue
            self.z_pose = self.calculate_z_pose(self.step)
            self.move_line_service_call(self.z_pose)
            r.sleep()

    def cleanup(self):
        # Before killing node
        rospy.loginfo("Shutting down move_z.")
        self.move_line_service_call(200)

if __name__ == "__main__":
    move_z_gesture = move_z()
    move_z_gesture.main_loop()