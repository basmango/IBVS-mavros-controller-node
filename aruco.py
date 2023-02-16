import rospy
import cv2
import cv_bridge
import numpy as np
from cv2 import aruco
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import String

class ArucoDetector:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/video_frames', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/detected_aruco_frames', Image, queue_size=10)
        self.coord_pub = rospy.Publisher('aruco_coordinates', String, queue_size=10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        parameters =cv2.aruco.DetectorParameters() 
        corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)
        
        cv2.circle(cv_image, (623,380), 10, (0, 255, 0), -1)
      
        if len(corners) > 0:
            for r in  corners:
                print(r)
                (ptA, ptB, ptC, ptD) = r[0]
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))
                ptA = (int(ptA[0]), int(ptA[1]))
                #cv2.line(cv_image, ptA, (pt_star[0],pt_star[1]), (0, 255, 0), 2)
                #cv2.line(cv_image, ptB, (pt_star[2],pt_star[3]), (0, 255, 0), 2)
                #cv2.line(cv_image, ptC, (pt_star[4],pt_star[5]), (0, 255, 0), 2)
                #cv2.line(cv_image, ptD, (pt_star[6],pt_star[7]), (0, 255, 0), 2)
                # draw the center (x, y)-coordinates of the AprilTag
                (cX, cY) = (int((ptA[0]+ptC[0])/2), int((ptA[1]+ptC[1])/2))
                cv2.circle(cv_image, (cX, cY), 10, (0, 0, 255), -1)
                cv2.circle(cv_image, ptA, 5, (255, 0, 0), -1)
                cv2.circle(cv_image, ptB, 5, (255, 0, 0), -1)
                cv2.circle(cv_image, ptC, 5, (255, 0, 0), -1)
                cv2.circle(cv_image, ptD, 5, (255, 0, 0), -1)



            # convert the corner coordinates to text
            corners_text = f"({ptA[0]:.2f}, {ptA[1]:.2f}), ({ptB[0]:.2f}, {ptB[1]:.2f}), ({ptC[0]:.2f}, {ptC[1]:.2f}), ({ptD[0]:.2f}, {ptD[1]:.2f})"

            # convert the center coordinates to text
            center_text = f"({cX:.2f}, {cY:.2f})"

            # concatenate the center and corner coordinates into a single string
            message = f"Center {center_text}, Corners {corners_text}"

            # publish message
            self.coord_pub.publish(message)



            #cv2.circle(cv_image, ((pt_star[0]+pt_star[2]+pt_star[4]+pt_star[6])/4,(pt_star[1]+pt_star[3]+pt_star[5]+pt_star[7])/4), 5, (0, 255, 0), -1)
            #cv2.circle(cv_image, (pt_star[0],pt_star[1]), 3, (0, 255, 0), -1)
            #cv2.circle(cv_image, (pt_star[2],pt_star[3]), 3, (0, 255, 0), -1)
            #cv2.circle(cv_image, (pt_star[4],pt_star[5]), 3, (0, 255, 0), -1)
            #cv2.circle(cv_image, (pt_star[6],pt_star[7]), 3, (0, 255, 0), -1)
            #cv_image = aruco.drawDetectedMarkers(cv_image, corners, ids)
        
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))

if __name__ == '__main__':
    rospy.init_node('aruco_detector', anonymous=True)
    aruco_detector = ArucoDetector()
    rospy.spin()
