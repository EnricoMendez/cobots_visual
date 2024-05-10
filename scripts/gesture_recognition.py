#!/usr/bin/env python3
import rospy
import mediapipe as mp 
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import rospkg

class gesture_recognition:
    def __init__(self):
		### Node init ###
        rospy.init_node("Gesture_recognition", anonymous=True)
        rospy.loginfo("Starting gesture_recognition.")
	    ### Variables ###
        self.image_received = 0 #Flag to indicate that we have already received an image
        self.cv_image = 0       #This is just to create the global variable cv_image 
        self.position_msg = String
        self.gesture_msg = String


		### Constants ###
          
        # Get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        # Get the file path for this ROS package
        pkg_path = str(rospack.get_path('cobots_visual'))
        model_path = pkg_path + '/scripts/gesture_recognizer.task'

        self.bridge = CvBridge()
        
        # Media pipe recognizer setup
        BaseOptions = mp.tasks.BaseOptions
        self.GestureRecognizer = mp.tasks.vision.GestureRecognizer
        GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
        VisionRunningMode = mp.tasks.vision.RunningMode
        self.options = GestureRecognizerOptions(
        base_options=BaseOptions(model_asset_path=model_path),
        running_mode=VisionRunningMode.IMAGE)

        ### Subscribers ###
        rospy.Subscriber("usb_cam/image_raw", Image, self.camera_callback)


        ### Publishers ###
        self.gesture_pub = rospy.Publisher("/hand/gesture", String, queue_size=10)
        self.position_pub = rospy.Publisher("/hand/position", String, queue_size=10)

    def camera_callback(self,data):
        print('Image recieved')
        self.cv_image = self.bridge.imgmsg_to_cv2(data)
        self.image_received = True

    def main_loop(self):
        rospy.on_shutdown(self.cleanup)
        r = rospy.Rate(0.1)
        print('waiting for image {}'.format(self.image_received))
        while not rospy.is_shutdown():
            if not self.image_received:
                 continue
			
            with self.GestureRecognizer.create_from_options(self.options) as recognizer:
				
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=self.cv_image)
                gesture_recognition_result = recognizer.recognize(mp_image) 
                if gesture_recognition_result.hand_landmarks == []: 
                    self.gesture_msg = ''
                    self.position_msg = ''	
                else:
                    self.gesture_msg = str((gesture_recognition_result.gestures[0][0].category_name))
                    self.position_msg = str((gesture_recognition_result.hand_landmarks[0][9]))	
                self.gesture_pub.publish(self.gesture_msg)
                self.position_pub.publish(self.position_msg)			
        r.sleep()


    def cleanup(self):
		# Before killing node
        rospy.loginfo("Shutting down gesture_recognition.")

if __name__ == "__main__":
	Gesture_recognition = gesture_recognition()
	Gesture_recognition.main_loop()