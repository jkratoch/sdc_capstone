import numpy as np
import cv2
import tensorflow as tf
from styx_msgs.msg import TrafficLight


RED_THRESHOLD = 1000
DETECTION_THRESHOLD = 0.5

LABEL_DICT = {1: [TrafficLight.GREEN, 'Green'],
              2: [TrafficLight.RED, 'Red'], 
              3: [TrafficLight.YELLOW, 'Yellow'],
              4: [TrafficLight.UNKNOWN, 'Unknown']}

num_classes = 4

class TLClassifier(object):
    def __init__(self, launch_type, model_path):
        self.launch_type = launch_type

        if self.launch_type == "SITE":
            self.graph = tf.Graph()
            with self.graph.as_default():
                od_graph_def = tf.GraphDef()
                with tf.gfile.GFile(model_path, 'rb') as fid:
                    serialized_graph = fid.read()
                    od_graph_def.ParseFromString(serialized_graph)
                    tf.import_graph_def(od_graph_def, name='')

            self.sess = tf.Session(graph=self.graph)

            tensor_names = ['num_detections', 'detection_boxes', 
                            'detection_scores', 'detection_classes']

            self.tensor_dict = {}
            for tensor_name in tensor_names:
                self.tensor_dict[tensor_name] = self.graph.get_tensor_by_name(
                                                    tensor_name + ':0')
            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')

    def get_classification(self, image):
        if self.launch_type == "SIM":
            return self.sim_classifier(image)
        else:
            return self.site_classifier(image)

    def sim_classifier(self, image):
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

    def site_classifier(self, image):
        image = np.expand_dims(image, axis=0)

        with self.graph.as_default():
            output_dict = self.sess.run(self.tensor_dict, 
                                        feed_dict={self.image_tensor: image})

        scores = output_dict['detection_scores'][0]
        classes = output_dict['detection_classes'][0]
        votes = []

        for i in range(num_classes):
            if scores[i] > DETECTION_THRESHOLD:
                votes.append(int(classes[i]))

        if votes:
            majority_vote = max(votes, key=votes.count)
        else:
            majority_vote = 4

        print(LABEL_DICT[majority_vote][1])
        return LABEL_DICT[majority_vote][0]


        


