import rclpy as rp
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
import time
from dobot_ros2_interface.srv import Dashboard
from dobot_ros2_interface.srv import Boxpoint
from dobot_ros2_interface.action import Move

class DepalletNode(Node):
	def __init__(self):
		super().__init__('depallet_node')
		self.dashboard_client = self.create_client(Dashboard, '/dashboard_server')
		self.find_client = self.create_client(Boxpoint, '/find_server')
		self.move_client = ActionClient(self, Move, 'move_goal_server')
		self.home_pose = [245, 5, 50]
		self.find_pose = [193, -270, 50] #처음 잡기 위치
		self.pallet_pose = [193, -270, -50]
		# x1, y1, x2, y2
		self.camera_coords = [184, 91, 554, 397]
		self.robot_coords = [193.9073473, -142.76081633, 385.76001919, 163.64729464]
		# cam z1, robot z1, cam z2, robot z2
		self.z_coords = [444, -54, 427, -37]
		self.camera_z1 = self.z_coords[0]
		self.robot_z1 = self.z_coords[1]
		self.plus_x = 2
		self.plus_y = 0
		self.plus_z = -0.3
		self.scale_pose()
		self.depallet()
	def req_dash(self, req_msg):
		while not self.dashboard_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Service not available, waiting again...')
		request = Dashboard.Request()
		request.req_str = str(req_msg)
		future = self.dashboard_client.call_async(request)
		rp.spin_until_future_complete(self, future)
		if future.result() is not None:
			response = future.result()
			self.get_logger().info(f"Dashboard Operated{response}")
		else:
			self.get_logger().error('Service call failed')
	def req_find(self, req_msg):
		while not self.find_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Service not available, waiting again...')
		request = Boxpoint.Request()
		request.req_str = req_msg
		future = self.find_client.call_async(request)
		self.get_logger().info(f"{request}")
		rp.spin_until_future_complete(self, future)
		if future.result() is not None:
			response = future.result()
			self.get_logger().info(f"find box {response}")
		else:
			self.get_logger().error('Service call failed')
		return response
	def send_goal(self, goal_pose):
		goal_msg = Move.Goal()
		goal_msg.goal_pose.position.x = float(goal_pose[0])
		goal_msg.goal_pose.position.y = float(goal_pose[1])
		goal_msg.goal_pose.position.z = float(goal_pose[2])
		goal_msg.goal_pose.orientation.w = 0.0
		self.move_client.wait_for_server()
		self.send_goal_future = self.move_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
		self.send_goal_future.add_done_callback(self.goal_response_callback)
	def goal_response_callback(self,future):
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.get_logger().error('Goal rejected')
			return
		self.get_logger().info('Goal accepted')
		self._get_result_future = goal_handle.get_result_async()
		self._get_result_future.add_done_callback(self.get_result_callback)
	def get_result_callback(self, future):
		result = future.result().result
		self.get_logger().info('Result: {0}'.format(result.result_str))
	def feedback_callback(self, feedback_msg):
		feedback = feedback_msg.feedback
		self.get_logger().info('Received feedback: {0}'.format(feedback.current_pose))
	def scale_pose(self):
		x1_cam, y1_cam, x2_cam, y2_cam = self.camera_coords
		x1_robot, y1_robot, x2_robot, y2_robot = self.robot_coords
		camera_z1, robot_z1, camera_z2, robot_z2 = self.z_coords
		self.scale_x = (y2_robot - y1_robot) / (x2_cam - x1_cam)
		self.scale_y = (x2_robot - x1_robot) / (y2_cam - y1_cam)
		self.scale_z = (camera_z2 - camera_z1) / (robot_z2 - robot_z1)
	def transform_img2base(self, goal_pose):
		x, y, z= goal_pose[0], goal_pose[1], goal_pose[2]
		y_ = self.scale_x * x + self.robot_coords[1] + self.plus_x
		x_ = self.scale_y * y + self.robot_coords[0] + self.plus_y
		z_ = self.robot_z1 + self.scale_z * (z - self.camera_z1) + self.plus_z
		return [x_, y_, z_]
	def depallet(self):
		self.get_logger().info('start')
		self.req_dash("power on")
		time.sleep(1)
		pallet_pose = self.pallet_pose
		while True:
			self.send_goal(self.find_pose)
			time.sleep(3)
			req_msg = self.req_find("find block")
			msg = req_msg.res_str
			if msg == 'found':
				goal_pose = [req_msg.x, req_msg.y, req_msg.z]
			else:
				self.get_logger().info(msg)
				continue
			if goal_pose[2] >= 455: #상자가 없고 바닥만 인식이되면 종료
				break
			elif goal_pose[2] < 250: #블럭 이외의 것이 찍혓을때
				continue
			goal_pose = self.transform_img2base(goal_pose)
			self.send_goal(goal_pose)
			time.sleep(3)
			self.req_dash("gripper on")
			time.sleep(3)
			lift_pose = goal_pose.copy()
			lift_pose[2] = 50
			self.send_goal(lift_pose)
			time.sleep(3)
			self.send_goal(pallet_pose)
			time.sleep(3)
			self.req_dash("gripper off")
			time.sleep(3)
			pallet_pose[2]+=17.5
		self.req_dash("power off")
		time.sleep(1)
		self.get_logger().info("depallet fin.")

def main(args=None):
	rp.init(args=args)
	node = DepalletNode()
	rp.spin_once(node)
	rp.shutdown()
if __name__ == '__main__':
	main()