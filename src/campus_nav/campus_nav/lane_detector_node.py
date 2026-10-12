#!/usr/bin/env python3

import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from campus_nav.coord_bridge import nav_to_gz, yaw_from_quaternion

class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector_node')

        self.declare_parameter('left_lane_offset', 1.2)
        self.declare_parameter('max_shift_clamp', 1.5)
        self.declare_parameter('control_rate_hz', 10.0)

        self.left_lane_offset = float(self.get_parameter('left_lane_offset').value)
        self.max_shift_clamp = float(self.get_parameter('max_shift_clamp').value)
        hz = float(self.get_parameter('control_rate_hz').value)

        self.bridge = CvBridge()
        
        # State
        self.raw_path = None
        self.speed_profile = None
        self.latest_image = None
        self.camera_info = None
        self.vehicle_pose_nav = None
        self.vehicle_yaw = 0.0
        
        self.ipm_matrix = None
        self.bev_width = 400
        self.bev_height = 400
        
        # Define bev_scale (m/px). Example: 400px = 8m lateral (so 400px=8m, 1px=0.02m)
        self.lateral_range = 8.0 # meters (±4m)
        self.forward_range = 8.0 # meters (0 to 8m)
        self.bev_scale_x = self.lateral_range / self.bev_width # meters per pixel (lateral)
        self.bev_scale_y = self.forward_range / self.bev_height # meters per pixel (forward)

        # Publishers
        self.target_path_pub = self.create_publisher(Path, '/campus/target_path', 10)
        self.debug_img_pub = self.create_publisher(Image, '/campus/vision_debug_image', 10)
        self.status_pub = self.create_publisher(String, '/campus/lane_detector_status', 10)

        # Subscribers
        self.path_sub = self.create_subscription(Path, '/campus/raw_target_path', self.path_callback, 10)
        self.speed_sub = self.create_subscription(Float32MultiArray, '/campus/speed_profile', self.speed_callback, 10)
        self.img_sub = self.create_subscription(Image, '/real_sense/image_raw', self.img_callback, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, '/real_sense/camera_info', self.cam_info_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/campus/vehicle_pose', self.pose_callback, 10)

        self.timer = self.create_timer(1.0 / hz, self.process_pipeline)

        self.get_logger().info(f"LaneDetectorNode ready | left_offset={self.left_lane_offset}m | max_clamp={self.max_shift_clamp}m")

    def path_callback(self, msg: Path):
        self.raw_path = msg

    def speed_callback(self, msg: Float32MultiArray):
        self.speed_profile = msg

    def img_callback(self, msg: Image):
        self.latest_image = msg

    def cam_info_callback(self, msg: CameraInfo):
        if self.camera_info is None:
            self.camera_info = msg
            self.init_ipm()

    def pose_callback(self, msg: PoseStamped):
        self.vehicle_pose_nav = msg
        self.vehicle_yaw = yaw_from_quaternion(msg.pose.orientation)

    def init_ipm(self):
        # K matrix (fx, fy, cx, cy)
        K = np.array(self.camera_info.k).reshape(3, 3)
        img_w = self.camera_info.width
        img_h = self.camera_info.height
        
        # Source points in image space (tune these for your camera/pitch)
        # Assuming camera is pitched down slightly.
        # Bottom of image is near the car, middle is far.
        src = np.float32([
            [img_w * 0.1, img_h * 0.9],   # bottom-left
            [img_w * 0.9, img_h * 0.9],   # bottom-right
            [img_w * 0.35, img_h * 0.55], # top-left
            [img_w * 0.65, img_h * 0.55]  # top-right
        ])
        
        # Destination points in BEV space
        dst = np.float32([
            [0, self.bev_height],
            [self.bev_width, self.bev_height],
            [0, 0],
            [self.bev_width, 0]
        ])
        
        self.ipm_matrix = cv2.getPerspectiveTransform(src, dst)

    def classify_road_pixels(self, bgr_image):
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        # Grass: high saturation greens (H: 35-85, S: 40-255, V: 40-255)
        grass_mask = cv2.inRange(hsv, (35, 40, 40), (85, 255, 255))
        # Road = NOT grass AND NOT sky (high brightness, low saturation)
        sky_mask = cv2.inRange(hsv, (90, 0, 200), (130, 30, 255))
        road_mask = cv2.bitwise_not(cv2.bitwise_or(grass_mask, sky_mask))
        # Apply Region of Interest: only look at bottom 60% of image (road area)
        h, w = road_mask.shape
        road_mask[:int(h * 0.4), :] = 0
        return road_mask

    def process_pipeline(self):
        if self.raw_path is None or self.latest_image is None or self.ipm_matrix is None or self.vehicle_pose_nav is None:
            self.get_logger().info(f"Waiting for topics... path:{self.raw_path is not None} img:{self.latest_image is not None} ipm:{self.ipm_matrix is not None} pose:{self.vehicle_pose_nav is not None}", throttle_duration_sec=2.0)
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV bridge error: {e}")
            return
        
        road_mask = self.classify_road_pixels(cv_img)
        bev_mask = cv2.warpPerspective(road_mask, self.ipm_matrix, (self.bev_width, self.bev_height))
        
        # Check fallback
        total_pixels = self.bev_width * self.bev_height
        road_pixels = cv2.countNonZero(bev_mask)
        if road_pixels < 0.10 * total_pixels:
            self.get_logger().warn("Vision failed (<10% road pixels). Fallback to raw path.")
            self.status_pub.publish(String(data="FALLBACK"))
            self.target_path_pub.publish(self.raw_path)
            
            # Publish empty debug image
            debug_img = cv2.cvtColor(bev_mask, cv2.COLOR_GRAY2BGR)
            self.debug_img_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, 'bgr8'))
            return
            
        self.status_pub.publish(String(data="ACTIVE"))
        
        # Calculate center line from BEV
        # Look at slices every 10%
        center_cols = []
        for y in range(0, self.bev_height, self.bev_height // 10):
            row = bev_mask[y, :]
            coords = np.where(row > 0)[0]
            if len(coords) > 0:
                left_col = coords[0]
                right_col = coords[-1]
                center_cols.append((left_col + right_col) / 2.0)
        
        if not center_cols:
            avg_center_col = self.bev_width / 2.0
        else:
            avg_center_col = np.mean(center_cols)
            
        # Left lane target column
        # Offset left in BEV is -X (assuming x=0 is left, x=width is right)
        shift_pixels = self.left_lane_offset / self.bev_scale_x
        target_col = avg_center_col - shift_pixels
        
        # Determine Buggy GZ Coords
        nav_x = self.vehicle_pose_nav.pose.position.x
        nav_y = self.vehicle_pose_nav.pose.position.y
        vx, vy = nav_to_gz(nav_x, nav_y)
        yaw = self.vehicle_yaw
        
        # Modify waypoints
        shifted_path = Path()
        shifted_path.header = self.raw_path.header
        
        debug_img = cv2.cvtColor(bev_mask, cv2.COLOR_GRAY2BGR)
        
        # Detect sharp turn ahead (within 10m)
        is_sharp_turn = False
        yaw_min = 0.0
        yaw_max = 0.0
        for i in range(len(self.raw_path.poses) - 1):
            p1 = self.raw_path.poses[i].pose.position
            p2 = self.raw_path.poses[i+1].pose.position
            dist = math.hypot(p1.x - vx, p1.y - vy)
            if dist < 12.0:
                seg_yaw = math.atan2(p2.y - p1.y, p2.x - p1.x)
                angle_diff = math.atan2(math.sin(seg_yaw - yaw), math.cos(seg_yaw - yaw))
                yaw_min = min(yaw_min, angle_diff)
                yaw_max = max(yaw_max, angle_diff)
                
        if (yaw_max - yaw_min) > math.radians(25):
            is_sharp_turn = True
            self.get_logger().info("Sharp turn detected ahead! Bypassing lane shift.", throttle_duration_sec=1.0)
        
        for i, pose_stamped in enumerate(self.raw_path.poses):
            wx = pose_stamped.pose.position.x
            wy = pose_stamped.pose.position.y
            
            # Distance from buggy
            dx = wx - vx
            dy = wy - vy
            dist = math.hypot(dx, dy)
            
            if dist < 10.0 and not is_sharp_turn:
                # Calculate lateral shift based on BEV observation.
                # In BEV, center column is width/2. Vehicle is roughly at width/2.
                # Lateral shift in meters
                lateral_shift_px = (self.bev_width / 2.0) - target_col
                lateral_shift_m = lateral_shift_px * self.bev_scale_x
                
                # Clamp
                lateral_shift_m = max(-self.max_shift_clamp, min(self.max_shift_clamp, lateral_shift_m))
                
                # Calculate path heading at this waypoint
                if i < len(self.raw_path.poses) - 1:
                    nx = self.raw_path.poses[i+1].pose.position.x
                    ny = self.raw_path.poses[i+1].pose.position.y
                    path_yaw = math.atan2(ny - wy, nx - wx)
                else:
                    path_yaw = yaw # fallback for the last waypoint
                
                # Shift perpendicular to path heading
                shift_angle = path_yaw + (math.pi / 2.0)
                new_wx = wx + lateral_shift_m * math.cos(shift_angle)
                new_wy = wy + lateral_shift_m * math.sin(shift_angle)
                
                # Draw on debug image (approximate projection back to BEV)
                # Local coords
                lx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
                ly = dx * math.sin(-yaw) + dy * math.cos(-yaw)
                # Map to BEV pixels
                # x in BEV = center - ly / scale_x (left is positive y in local frame)
                # y in BEV = bottom - lx / scale_y (forward is positive x in local frame)
                px = int(self.bev_width / 2.0 - ly / self.bev_scale_x)
                py = int(self.bev_height - lx / self.bev_scale_y)
                
                new_lx = lx
                new_ly = ly + lateral_shift_m
                new_px = int(self.bev_width / 2.0 - new_ly / self.bev_scale_x)
                new_py = int(self.bev_height - new_lx / self.bev_scale_y)
                
                if 0 <= px < self.bev_width and 0 <= py < self.bev_height:
                    cv2.circle(debug_img, (px, py), 3, (0, 0, 255), -1) # Original: Red
                if 0 <= new_px < self.bev_width and 0 <= new_py < self.bev_height:
                    cv2.circle(debug_img, (new_px, new_py), 3, (0, 255, 0), -1) # Shifted: Green
            else:
                new_wx = wx
                new_wy = wy
                
            new_pose = PoseStamped()
            new_pose.header = pose_stamped.header
            new_pose.pose.position.x = new_wx
            new_pose.pose.position.y = new_wy
            new_pose.pose.position.z = pose_stamped.pose.position.z # speed is kept
            new_pose.pose.orientation = pose_stamped.pose.orientation
            shifted_path.poses.append(new_pose)
            
        self.target_path_pub.publish(shifted_path)
        self.debug_img_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, 'bgr8'))
        
def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
