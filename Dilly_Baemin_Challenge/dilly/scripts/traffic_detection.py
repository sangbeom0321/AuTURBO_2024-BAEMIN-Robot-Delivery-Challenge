#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
import cv2
import numpy as np
from ultralytics import YOLO

class TrafficDetectionNode():
    def __init__(self):
        rospy.init_node('traffic_detection_node', anonymous=True)
        
        # YOLOv8 model
        try:
            ckpt_path = "/home/yjkim/ultralytics/pt_file/best.pt"
            self.model = YOLO(ckpt_path)
            rospy.loginfo("YOLO model loaded successfully.")
        except Exception as e:
            rospy.logerr(f"Error loading YOLO model: {e}")
            self.model = None

        # Subscriber to image topic
        self.image_sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.image_callback)

        # Publisher to detection topic
        self.Traffic_pub = rospy.Publisher('/trafficSign', Bool, queue_size=5)
        self.TrafficSignPub = True
        self.Red = False

        self.rate = rospy.Rate(5)
        self.image_np = np.ndarray((480, 640, 3))


    def image_callback(self, msg):
        if self.model is None:
            rospy.logerr("Model not loaded. Skipping image processing.")
            return

        try:
            # Convert CompressedImage to numpy array
            np_arr = np.frombuffer(msg.data, np.uint8)
            # Decode image
            self.image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # print(self.image_np.shape)

            # # Perform object detection using YOLOv8
            # results = self.model.predict(image_np, conf=0.70, save=False)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def process_detections(self):
        results = self.model.predict(self.image_np, conf=0.68, save=False, verbose=True)
        detection_data = []
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls)  # Class ID of the detected object
                conf = float(box.conf)  # Confidence of the detection
                bbox = box.xyxy.tolist()  # Bounding box in [x_min, y_min, x_max, y_max]

                if class_id == 82:
                    self.Red = True
                # # Format detection as a string (you could modify this to publish in other formats)
                # detection_info = f"Class: {class_id}, Confidence: {conf}, BBox: {bbox}"
                # detection_data.append(detection_info)

        # Convert the list of detections to a single string
        # detection_str = "\n".join(detection_data)
        # rospy.loginfo(f"Detections: \n{detection_str}")
        # return detection_str
    
    def TrafficSign(self):
        if self.Red == False:
            rospy.loginfo("Traffic is Green / Run")
        else:
            for i in range(1000):
                if self.Red == False:
                    rospy.loginfo("Run")
                    break
                else:
                    rospy.loginfo("Stop")


    def run(self):
        while not rospy.is_shutdown():
            # Process and publish detection results
            self.process_detections()
            # self.TrafficSign()
            # self.Traffic_pub.publish(self.TrafficSign)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        trafficNode = TrafficDetectionNode()
        trafficNode.run()
    except rospy.ROSInterruptException:
        pass
