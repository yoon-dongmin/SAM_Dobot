from dobot_ros2_interface.srv import Boxpoint
import rclpy as rp
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from segment_anything import sam_model_registry, SamAutomaticMaskGenerator,SamPredictor
import open3d as o3d
from ament_index_python.packages import get_package_share_directory

class FindServer(Node):
	def __init__(self):
		super().__init__('find_server')
		self.get_logger().info("start")
		pt_path = get_package_share_directory('dobot_depallet') + '/model/sam_vit_b_01ec64.pth'
		sam_checkpoint = pt_path
		model_type = "vit_b"
		device = "cpu"
		self.sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
		self.sam.to(device=device)
		self.mask_generator = SamAutomaticMaskGenerator(self.sam)
		self.get_logger().info("load sam model")
		self.server = self.create_service(Boxpoint,'find_server', self.callback_service)
		self.img_subscriber = self.create_subscription(Image,'/camera/color/image_raw',self.image_callback,10)
		self.depth_subscriber = self.create_subscription(Image,'/camera/aligned_depth_to_color/image_raw',0)
		self.cv_bridge = CvBridge()
		self.color_image = None
		self.depth_image = None
		self.x1, self.y1 = 178, 67
		self.x2, self.y2 = 567, 341

	def image_callback(self, msg):
		self.color_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		self.color_image = self.color_image[self.y1:self.y2, self.x1:self.x2]

	def depth_callback(self, msg):
		self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')
		self.depth_image = self.depth_image[self.y1:self.y2, self.x1:self.x2]
	
	def callback_service(self, request, response):
		if request.req_str == 'find block':
			if self.color_image is not None and self.depth_image is not None:
				x, y, z = self.segment_box(self.color_image, self.depth_image)
				self.get_logger().info(f"found box! x:{x}, y:{y}, z:{z}")
				response.x = float(x)
				response.y = float(y)
				response.z = float(z)
				response.res_str = 'found'
				return response
		else:
			self.get_logger().info('Wrong Command!')
			response.res_str = 'wrong Command'
			return response

		response.res_str = 'no image'
		return response
	
	def segment_box(self, color_image, depth_image):
		height = depth_image.shape[0]
		width = depth_image.shape[1]
		self.get_logger().info("searching box...")
		# self.mask_generator = SamAutomaticMaskGenerator(
		# model=self.sam,
		# points_per_side=32,
		# pred_iou_thresh=0.86,
		# stability_score_thresh=0.92,
		# crop_n_layers=1,
		# crop_n_points_downscale_factor=3,
		# min_mask_region_area=100, # Requires open-cv to run post-processing
		# )
		masks = self.mask_generator.generate(color_image)
		mask_list = []
		depht_avg_min = float('inf')
		box_mask_index = None
		
		for i in range(len(masks)):
			if np.count_nonzero(masks[i]["segmentation"]) < 5000 and np.count_nonzero(masks[i]["segmentation"]) > 3000:
				mask_list.append(i)
		depth = depth_image.copy()
		
		for i in mask_list:
			non_zero_depth = depth[masks[i]["segmentation"]]
			non_zero_depth.sort()
			size = len(non_zero_depth)
			depht_avg = sum(non_zero_depth[int(size/4):int(size/4*3)]) / size * 2
			if depht_avg_min > depht_avg:
				depht_avg_min = depht_avg
				box_mask_index = i
		if box_mask_index is None:
			self.get_logger().info("NOT detected box")
			return

		box_depth = np.where(masks[box_mask_index]["segmentation"], depth, 0)
		
		depth_raw = o3d.geometry.Image(box_depth)
		fx = fy = 30 # X, Y 축에 대한 초점 거리 (임의로 설정)
		cx = width / 2 # X 축 중심 좌표 (이미지의 중앙)
		cy = height / 2
		
		camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy,cx, cy)
		
		pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_raw, camera_intrinsic)
		try:
			plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,ransac_n=3,num_iterations=1000)
			[a, b, c, d] = plane_model
			#print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
			inlier_cloud = pcd.select_by_index(inliers)
			inlier_cloud.paint_uniform_color([1.0, 0, 0])
			#outlier_cloud = pcd.select_by_index(inliers, invert=True)
			obb = inlier_cloud.get_minimal_oriented_bounding_box()
			obb.color = (0, 1, 0)
			x, y, z = obb.get_center()
			z = self.depth_point(a, b, c, d, x, y)
			x, y, z = self.trans(x, y, z, width, height)
			cv2.circle(depth, (int(x), int(y)), 10, 255, -1)
			cv2.imshow('point_box', box_depth*255)
		except:
			self.get_logger().info("ransac fail")
			pass
		image = color_image
		image[masks[box_mask_index]["segmentation"]] = [0, 0, 255]
		cv2.imshow('segment_color_image', image)
		cv2.waitKey(1)
		return x, y, z

	def trans(self, x_3d, y_3d, z_3d, width, height):
		fx = 30.0
		fy = 30.0
		cx = width // 2
		cy = height // 2
		z = z_3d * 1000.0
		x = (( x_3d * fx / z_3d) + cx)
		y = (( y_3d * fy / z_3d) + cy)
		return x, y ,z

	def depth_point(self, a, b, c, d, x, y):
		z = (a*x + b*y + d) / -c
		return z
		
def main(args=None):
	rp.init(args=args)
	server = FindServer()
	rp.spin(server)
	rp.shutdown()
if __name__ == '__main__':
	main()