#!/usr/bin/env python
import numpy as np 
import cv2
import os
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge, CvBridgeError
import threading

class NN_Detector():
    def __init__(self):
        self.labelsPath = os.path.join('../yolov3/yolo.names')
        self.LABELS = open(self.labelsPath).read().strip().split('\n')
        self.weightsPath = os.path.join('../yolov3/yolov3_custom_colab_tiny_final.weights')
        self.configPath = os.path.join('../yolov3/yolov3_custom_colab_tiny.cfg')
        self.net = cv2.dnn.readNetFromDarknet(self.configPath,self.weightsPath)
        self._image_pub = rospy.Publisher("/camera/nn_detections/image_raw", Image, queue_size=1)
        self._point_pub = rospy.Publisher("detector/bb_center", PointStamped, queue_size=1)
        self._image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_callback, queue_size=1)
        self._detect_sub = rospy.Subscriber("/robot/get_new_point", String, self.predict, queue_size=1)
        self._bridge = CvBridge()
        self._lock = threading.RLock()
        self._image = None

    def camera_callback(self, Img_msg):
        with self._lock:
            try:
                self._image = self._bridge.imgmsg_to_cv2(Img_msg, "bgr8")
            except (CvBridgeError, TypeError) as e:
                rospy.logerr('Could not convert img msg to cv2. Error: {}'.format(e))
                return
        #self.predict('foo')
    def predict(self, msg):
        if self._image is None:
            return
        with self._lock:
            np.random.seed(42)
            COLORS = np.random.randint(0,255,size=(len(self.LABELS), 3), dtype="uint8")
            (H,W) = self._image.shape[:2]

            ln = self.net.getLayerNames()
            ln = [ln[i[0]-1] for i in self.net.getUnconnectedOutLayers()]

            blob = cv2.dnn.blobFromImage(self._image, 1/255.0, (416,416), swapRB=True,crop=False)
            self.net.setInput(blob)
            layerOutputs = self.net.forward(ln)

            boxes = []
            confidences = []
            classIDs = []
            threshold = 0.2

            for output in layerOutputs:
                for detection in output:
                    scores = detection[5:]
                    classID = np.argmax(scores)
                    confidence = scores[classID]

                    if confidence > threshold:
                        box = detection[0:4] * np.array([W,H,W,H])
                        (centerX, centerY, width, height) = box.astype("int")

                        x = int(centerX - (width/2))
                        y = int(centerY - (height/2))

                        boxes.append([x, y, int(width), int(height)])
                        confidences.append(float(confidence))
                        classIDs.append(classID)
            idxs = cv2.dnn.NMSBoxes(boxes, confidences, threshold, 0.1)
            if len(idxs) > 0:
            	publish = True
                for i in idxs.flatten():
                    (x, y) = (boxes[i][0],boxes[i][1])
                    (w,h) = (boxes[i][2], boxes[i][3])
                    centerx = x + (w/2)
                    centery = y + (h/2)
                    if(publish):
                    	color = (0,255,0)
                    else:
                    	color = (0,0,0)
                    cv2.rectangle(self._image, (x,y), (x+w, y+h), color, 2)
                    text = "{}".format(self.LABELS[classIDs[i]], confidences[i])
                    cv2.putText(self._image, text, (x+15, y-10), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                    if(publish):
	                    pt = PointStamped()
	                    pt.header.stamp = rospy.Time.now()
	                    pt.header.frame_id = "camera_color_optical_frame"
	                    pt.point.x = centerx 
	                    pt.point.y = centery 
	                    pt.point.z = 0
	                    self._point_pub.publish(pt)
	                    publish = False
                    
            try:
                ros_msg = self._bridge.cv2_to_imgmsg(self._image, encoding="bgr8")
            except (CvBridgeError, TypeError) as e:
                rospy.logwarn('Could not convert image from cv2 to imgmsg. Error: {}'.format(e))
                return
            self._image_pub.publish(ros_msg)
            return

if __name__ == '__main__':
    rospy.init_node('NN_Node')
    foo = NN_Detector()
    while not rospy.is_shutdown():
        rospy.spin()
