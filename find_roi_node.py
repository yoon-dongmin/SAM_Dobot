import rclpy as rp
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Findroi(Node):
	def __init__(self):
		super().__init__('find_roi')
		self.img_subscriber = self.create_subscription(
			Image, 
			'/camera/color/image_raw', # 이미지 토픽
			self.image_callback,
			10
		)

		self.depth_subscriber = self.create_subscription(
				Image,
				'/camera/aligned_depth_to_color/image_raw', # 이미지 토픽
				self.depth_callback,
				0
			)

		self.depth_image = None
		self.x1, self.y1 = 178, 67
		self.x2, self.y2 = 567, 341


		self.cv_bridge = CvBridge()

	def mouse_callback(self, event, x, y, flags, param):
		if event == cv2.EVENT_LBUTTONDOWN:
			self.get_logger().info(f"마우스 클릭 좌표: ({x}, {y})")
			if self.depth_image is not None:
				depth = self.depth_image[y][x]
				self.get_logger().info(f"마우스 클릭 depth: ({depth})")

	def image_callback(self, msg):
		self.color_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		#self.color_image = self.color_image[self.y1:self.y2, self.x1:self.x2]
		cv2.namedWindow("color")
		cv2.setMouseCallback("color", self.mouse_callback)
		cv2.imshow("color", self.color_image)
		cv2.waitKey(1)

	def depth_callback(self, msg):
		self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
		#self.depth_image = self.depth_image[self.y1:self.y2, self.x1:self.x2]
		
		
def main(args=None):
	rp.init(args=args)
	dsbd = Findroi()
	rp.spin(dsbd)
	rp.shutdown()

if __name__ == '__main__':
	main()