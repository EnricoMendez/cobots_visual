#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from xarm_msgs.srv import Move, MoveRequest, SetAxis, SetInt16

class move_z:
    def __init__(self):
        ### Node init ###
        rospy.init_node("move_z_gesture", anonymous=True)
        rospy.loginfo("Starting move_z.")

        ### Variables ###
        self.myMove = MoveRequest()
        self.z_pose = 200           #(mm)
        self.gesture = 'None'
        self.step = 5
        
        ### Constants ###
        self.myMove.mvvelo = 200        #(mm/s)
        self.myMove.mvacc =  200        #(mm/s²)
        self.z_0 = 100                  #(mm) lowest posible value for z_pose
        self.z_1 = 300                  #(mm) highest posible value for z_pose
        self.z_dif = self.z_1 - self.z_0
        self.step_size = 10
        
        ### Subscribers ###
        rospy.Subscriber("/hand/gesture", String, self.gesture_callback)

        ### Robot initial setup ###
        self.robot_enable()

        ### Service proxy ###
        # Wait for move line service to be available 
        rospy.loginfo('Waiting for move_line service to be available ...')
        rospy.wait_for_service('/ufactory/move_line')
        self.move_proxy = rospy.ServiceProxy('/ufactory/move_line', Move)
        rospy.loginfo('Service available')

        rospy.loginfo('Going to initial position')
        self.step = self.step_size/2
        self.z_pose = self.calculate_z_pose(self.step)
        self.move_line_service_call(self.z_pose)        # Move to initial pose

    def robot_enable(self):
        # 1 Enable motors
        rospy.wait_for_service('/ufactory/motion_ctrl') 
        enable_mot_service_proxy = rospy.ServiceProxy('/ufactory/motion_ctrl', SetAxis)
        enable_mot_service_proxy(8, 1)
        rospy.loginfo('Motors enabled')

        # 2 Set robot mode
        rospy.wait_for_service('/ufactory/set_mode') 
        mode_service_proxy = rospy.ServiceProxy('/ufactory/set_mode', SetInt16)
        print("calling MODE service")
        mode_service_proxy(0) 
        rospy.loginfo('Mode was set to 0')

        # 3 Set robot state
        rospy.wait_for_service('/ufactory/set_state') #This is a convenience method that blocks until the service is available.
        mode_service_proxy = rospy.ServiceProxy('/ufactory/set_state', SetInt16)
        mode_service_proxy(0)
        rospy.loginfo('Robot state set to 0')

    def move_line_service_call(self, z_pose):
        # Set the target pose
        self.myMove.pose = [250.0,0.0,z_pose,3.14,0.0,0.0]
        # Call the service
        self.move_proxy(self.myMove)
        rospy.loginfo('Service called successfuly to {} z position'.format(z_pose))

    def calculate_z_pose(self,step):
        displacement = (self.z_dif) * step/self.step_size
        z_pose = displacement + self.z_0
        return z_pose

    def gesture_callback(self,data):
        self.gesture = data.data

    def main_loop(self):
        rospy.on_shutdown(self.cleanup)
        r = rospy.Rate(.5)
        rospy.loginfo('Starting main loop')
        while not rospy.is_shutdown():
            if self.gesture == 'Thumb_Up':
                self.gesture = ''   # Reset the gesture variable
                if self.step == self.step_size: continue
                self.step += 1
            elif self.gesture == 'Thumb_Down':
                self.gesture = ''   # Reset the gesture variable
                if self.step == 0: continue
                self.step -= 1
            else : 
                continue
            rospy.loginfo('Step set to : {}'.format(self.step))
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