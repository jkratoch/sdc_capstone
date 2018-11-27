import numpy as np
import cv2
from styx_msgs.msg import TrafficLight


RED_THRESHOLD = 700

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        # FOR SIMULATOR ONLY
        # DH 27/11/2018

        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower1 = np.array([0, 50, 50])
        upper1 = np.array([10, 256, 256])
        mask1 = cv2.inRange(hsv, lower1, upper1)

        lower2 = np.array([170, 50, 50])
        upper2 = np.array([180, 256, 256])
        mask2 = cv2.inRange(hsv, lower2, upper2)

        mask = mask1 + mask2
        red = np.copy(image)
        red[mask == 0] = [0, 0, 0]

        red_count = np.count_nonzero(red)

        if red_count > RED_THRESHOLD:
            print('RED')
            return TrafficLight.RED
        else:
            print('NOT RED')
            return TrafficLight.UNKNOWN
