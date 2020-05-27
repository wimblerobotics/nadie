import rosbag
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage

cv_bridge = CvBridge()
bag = rosbag.Bag("/home/wimble/2020-03-12-22-54-03.bag", mode='r')
for topic, msg, t in bag.read_messages(topics="/t265/fisheye2/image_raw/compressed"):
    #print(topic, t, msg)
    img = cv_bridge.compressed_imgmsg_to_cv2(msg)
    cv2.imshow('frame', img)
    cv2.waitKey(0)
    #cv2.destroyAllWindows()
    #exit()