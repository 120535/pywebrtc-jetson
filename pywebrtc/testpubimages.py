'''
Publish test images to the ROS2 image topic

Build (in the docker)

cd /workspaces/isaac_ros-dev
colcon build --symlink-install --packages-select pywebrtc
source install/setup.bash
ros2 run pywebrtc testpubimages


 - and for the receiver...
ros2 run pywebrtc websrvr

'''


# ros2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String     # for carrying json to/from the browser - ROS2 
from cv_bridge import CvBridge
from sensor_msgs.msg import Image   # image, internal to ROS2 (and eventually published to browser via WebRTC)

import cv2
import numpy
import math


# First aiortc, then ROS2

####################################
# ROS2 - publish and subscribe
####################################

jsonTopic = 'webrtcpub'
imageTopic = 'rtcimage'

class TestWebRTCPubImage(Node):

    def __init__(self):
        super().__init__('testpubimages')

        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, imageTopic, 10)

        # and for the json
        self.jsonpublisher_ = self.create_publisher(String, jsonTopic, 10)


        # testing - create sample animation
        # generate flag
        height, width = 480, 640
        data_bgr = numpy.hstack(
            [
                self._create_rectangle(
                    width=213, height=480, color=(255, 0, 0)
                ),  # blue
                self._create_rectangle(
                    width=214, height=480, color=(255, 255, 255)
                ),  # white
                self._create_rectangle(width=213, height=480, color=(0, 0, 255)),  # red
            ]
        )

        # shrink and center it
        M = numpy.float32([[0.5, 0, width / 4], [0, 0.5, height / 4]])
        data_bgr = cv2.warpAffine(data_bgr, M, (width, height))

        # compute animation
        omega = 2 * math.pi / height
        id_x = numpy.tile(numpy.array(range(width), dtype=numpy.float32), (height, 1))
        id_y = numpy.tile(
            numpy.array(range(height), dtype=numpy.float32), (width, 1)
        ).transpose()

        self.cels = []
        for k in range(30):
            phase = 2 * k * math.pi / 30
            map_x = id_x + 10 * numpy.cos(omega * id_x + phase)
            map_y = id_y + 10 * numpy.sin(omega * id_x + phase)
            self.cels.append( cv2.remap(data_bgr, map_x, map_y, cv2.INTER_LINEAR) )       

            # self.frames.append(
            #     VideoFrame.from_ndarray(
            #         cv2.remap(data_bgr, map_x, map_y, cv2.INTER_LINEAR), format="bgr24"
            #     )
            # )        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        jtimer_period = 5.0  # seconds
        self.jtimer = self.create_timer(jtimer_period, self.timer_jsoncallback)

    # for the flag animation
    def _create_rectangle(self, width, height, color):
        data_bgr = numpy.zeros((height, width, 3), numpy.uint8)
        data_bgr[:, :] = color
        return data_bgr

    def timer_callback(self):
        self.i += 1

        img = self.cels[self.i % 30]
        self.publisher_.publish(self.bridge.cv2_to_imgmsg(img, "passthrough"))
        # self.get_logger().info('Publishing image:')

    def timer_jsoncallback(self):
        msg = String()
        msg.data = 'Image count: %d' % self.i
        self.jsonpublisher_.publish(msg)
        self.get_logger().info('Published: "%s"' % msg.data)



## Full main 
        
def main(args=None):
    rclpy.init(args=args)

    webrtc_pubsub = TestWebRTCPubImage()

    rclpy.spin(webrtc_pubsub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    webrtc_pubsub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
