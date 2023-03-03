#! /usr/bin/env python

import os
import rospy
from rospkg import RosPack
from threading import Thread
from std_msgs.msg import Bool
# from demo_3_march.srv import detectService

pkg_path = RosPack().get_path("demo_3_march")
rosbag_command_args = ""
hi_audio_file = "audios/hi.mpeg"
hi_bag_file = "bags/hi.bag"

products_video_file = "videos/products.mp4"
products_bag_file = "bags/products.bag"

faq_audio_file = "audios/faq.mp3"

class subscriberSaver():

    def __init__(self, topic_name, msg_type):
        self.current_message = msg_type()
        rospy.Subscriber(topic_name, msg_type, self.callback, queue_size=1)

    def callback(self, msg):
        self.current_message = msg
    
if __name__ == '__main__':

    rospy.init_node("main")
    short_check = subscriberSaver("short_detected", Bool)
    long_check = subscriberSaver("long_detected", Bool)

    chatbot_starter = rospy.Publisher("chatbot_control", Bool, queue_size=1)
    chatbot_end_check = subscriberSaver("chatbot_control", Bool)

    while not rospy.is_shutdown():

        print("short detect")
        if not short_check.current_message.data:
            continue

        # hi 
        gesture_thread = Thread(target=os.system, 
                                args=(("rosbag play " + os.path.join(pkg_path, hi_bag_file) + rosbag_command_args)))
        gesture_thread.start()
        os.system("mpg321 " + os.path.join(pkg_path, hi_audio_file))
        gesture_thread.join()

        print('long detect')
        while long_check.current_message.data:

            # play products video and gestures
            gesture_thread = Thread(target=os.system,
                                    args=(("rosbag play " + os.path.join(pkg_path, products_bag_file) + rosbag_command_args)))
            gesture_thread.start()
            os.system("mplayer -fs " + os.path.join(pkg_path, products_video_file))
            gesture_thread.join()

            # play would you like to know more
            os.system("mpg321 " + os.path.join(pkg_path, faq_audio_file))

            # chatbot
            while chatbot_starter.get_num_connections() <= 1:
                pass
            chatbot_starter.publish(True)
            while chatbot_end_check.current_message == None:
                pass

            while chatbot_end_check.current_message.data:
                pass