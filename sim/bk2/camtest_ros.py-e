#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def callback(data):
    """
    :param data: sensor_msg array containing the image in the Gazsbo format
    :return: nothing but sets [cv_image] to the usefull image that can be use
             in opencv (numpy array)
    """
    global cv_image
    global bridge
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")


bridge = CvBridge()
cv_image = np.zeros((640, 480))
rospy.init_node('CAMnod', anonymous=True)
image_sub = rospy.Subscriber("/automobile/image_raw", Image, callback)

while not rospy.is_shutdown():
    cv2.imshow("frame preview", cv_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# When everything done, release the capture
cv2.destroyAllWindows()
