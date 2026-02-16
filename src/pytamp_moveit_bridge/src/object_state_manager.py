#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from cv_bridge import CvBridge
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from yolov8_msgs.msg import Yolov8Inference
from pytamp_moveit_bridge.msg import ObjectState, WorldState
from image_geometry import PinholeCameraModel
import math

class ObjectStateManager(Node):
    def __init__(self):
        super().__init__('object_state_manager')
        
        # Parameters
        self.target_frame = 'panda_link0' # Base frame of the robot
        self.camera_frame = None # Will be observed from messages
        
        # Tools
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.camera_info_received = False

        # Publishers
        self.world_state_pub = self.create_publisher(WorldState, '/world_state', 10)
        
        # Subscribers
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, 10)
        
        # Synchronization for Inference + Depth
        # Logic: We need depth to project 2D OBB to 3D.
        # We assume /Yolov8_Inference and /camera/depth/image_raw are roughly synced 
        # or we just take the latest depth.
        # Since Yolov8 might be slower, using ApproximateTimeSynchronizer is good.
        
        self.inference_sub = message_filters.Subscriber(self, Yolov8Inference, '/Yolov8_Inference')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.inference_sub, self.depth_sub], 
            queue_size=10, 
            slop=0.5 # Generous slop as inference might be delayed
        )
        self.ts.registerCallback(self.sync_callback)

        self.get_logger().info('ObjectStateManager Initialized. Waiting for data...')
        self.create_timer(5.0, self.check_status)

    def check_status(self):
        if not self.camera_info_received:
            self.get_logger().warn("WAITING FOR CAMERA INFO... (Topic: /camera/camera_info)")
        else:
             self.get_logger().info(f"Status: CamInfo=OK, Frame={self.camera_frame}. Waiting for synced Inference+Depth...", throttle_duration_sec=10.0)

    def info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_model.fromCameraInfo(msg)
            self.camera_frame = msg.header.frame_id
            self.camera_info_received = True
            self.get_logger().info(f'Camera Info received. Frame: {self.camera_frame}')

    def sync_callback(self, inference_msg, depth_msg):
        if not self.camera_info_received:
            return

        try:
            # Convert depth image
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        world_state = WorldState()
        
        # Process each detection
        for result in inference_msg.yolov8_inference:
            # result has .class_name and .coordinates (8 floats = 4 points x,y)
            coords = result.coordinates
            if len(coords) < 8:
                continue
                
            points = np.array(coords).reshape(4, 2)
            
            # 1. Compute 2D Center
            center_u = np.mean(points[:, 0])
            center_v = np.mean(points[:, 1])
            
            # 2. Get Depth at Center
            # Bounds check
            h, w = depth_image.shape
            u_int = int(np.clip(center_u, 0, w-1))
            v_int = int(np.clip(center_v, 0, h-1))
            
            depth_val = depth_image[v_int, u_int]
            
            # Handle float (meters) or uint16 (mm)
            if depth_image.dtype == np.uint16:
                depth_m = depth_val / 1000.0
            else:
                depth_m = float(depth_val)
            
            # Heuristic: If depth is huge (> 10m) for a tabletop task, 
            # it's likely millimeters masquerading as float.
            if depth_m > 10.0:
                self.get_logger().warn(f"Detected huge depth {depth_m:.2f}. Assuming mm and scaling by 1/1000.", throttle_duration_sec=2.0)
                depth_m /= 1000.0

            if depth_m <= 0.0 or np.isnan(depth_m):
                continue # Invalid depth

            # 3. Project to 3D (Camera Frame)
            ray = self.camera_model.projectPixelTo3dRay((center_u, center_v))
            # ray is normalized (z=1), so multiply by depth
            point_camera = np.array(ray) * depth_m
            
            # DEBUG
            # self.get_logger().info(f"Depth Raw: {depth_val}, Meters: {depth_m}")
            # self.get_logger().info(f"Cam Point: {point_camera}")
            
            # 4. Compute Orientation (Angle of major axis)
            # Find longest edge
            p0 = points[0]
            p1 = points[1]
            p2 = points[2]
            dist01 = np.linalg.norm(p0 - p1)
            dist12 = np.linalg.norm(p1 - p2)
            
            if dist01 > dist12:
                # 0-1 is long edge
                angle = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
            else:
                # 1-2 is long edge
                angle = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
            
            # This angle is in Image Plane (Z-axis rotation in camera frame usually?)
            # Wait, OBB angle in image -> Roll/Pitch/Yaw in 3D?
            # In top-down view, it corresponds to Yaw around World Z.
            # But the camera might be tilted.
            # Construct a Quaternion in Camera Frame? 
            # Simplified: Represent as rotation around Camera Z (optical axis)
            # q_cam = rpy(0, 0, angle)
            # We will transform this pose to World Frame.
            
            q_cam = self.euler_to_quaternion(0, 0, angle)

            # 5. Transform to Target Frame (Robot Base)
            try:
                # Create PoseStamped in Camera Frame
                p_cam = rclpy.time.Time() # Use latest transform or sync?
                # Using latest available transform is often safer for reliable lookups
                
                # Transform Point
                trans = self.tf_buffer.lookup_transform(
                    self.target_frame, 
                    self.camera_frame, 
                    rclpy.time.Time()) # Get latest
                
                # Apply transform manually or use do_transform_pose
                # Let's use do_transform_pose logic (simplified for point)
                # But we also need to transform the orientation which depends on the plane.
                # If we assume objects are flat on table, we might just want to project the points to world 
                # and compute angle there.
                
                # Better approach: Transform the 2 points (Head and Tail of long edge) to 3D World, 
                # then compute angle in World XY plane.
                
                # Helper to transform a local 3D point to world
                def transform_pt(pt_cam):
                    p_stamped = tf2_geometry_msgs.PointStamped()
                    p_stamped.header.frame_id = self.camera_frame
                    p_stamped.point.x = float(pt_cam[0])
                    p_stamped.point.y = float(pt_cam[1])
                    p_stamped.point.z = float(pt_cam[2])
                    p_out = self.tf_buffer.transform(p_stamped, self.target_frame)
                    return np.array([p_out.point.x, p_out.point.y, p_out.point.z])

                center_world = transform_pt(point_camera)
                # DEBUG
                self.get_logger().info(f"Detection: Class={result.class_name} | Depth Raw: {depth_val:.1f} | World Point: {center_world}", throttle_duration_sec=2.0)
                     
                # Angle in World Frame
                
                # Angle in World Frame
                # We need one more point along the angle direction in 3D
                vec_len_px = 10.0 # 10 pixels along angle
                u_vec = center_u + vec_len_px * math.cos(angle)
                v_vec = center_v + vec_len_px * math.sin(angle)
                
                ray_vec = self.camera_model.projectPixelTo3dRay((u_vec, v_vec))
                point_vec_camera = np.array(ray_vec) * depth_m # Assume same depth (flat object)
                
                vec_world_pt = transform_pt(point_vec_camera)
                
                # Vector in World
                world_edge = vec_world_pt - center_world
                yaw_world = math.atan2(world_edge[1], world_edge[0])
                
                q_world = self.euler_to_quaternion(0, 0, yaw_world)
                
                # Create ObjectState
                obj_state = ObjectState()
                obj_state.object_id = f"{result.class_name}_{int(self.get_clock().now().nanoseconds)}"
                obj_state.class_name = result.class_name
                obj_state.pose.position.x = center_world[0]
                obj_state.pose.position.y = center_world[1]
                obj_state.pose.position.z = center_world[2]
                obj_state.pose.orientation = q_world
                obj_state.confidence = 1.0 # TODO: Pass confidence if available
                
                # Dimensions (approx from OBB pixels converted to meters at depth)
                # This is rough, assume box centered
                # dist01_m = (dist01 / fx) * depth
                # dist12_m = (dist12 / fy) * depth (approx)
                # avg_f = (fx + fy) / 2
                # w_m = (dist01 / avg_f) * depth
                # h_m = (dist12 / avg_f) * depth
                
                # Actually Pinhole model has fx, fy
                # Simple approximation:
                # Project corner points to 3D and measure distance
                # But we skipped corner projection.
                # Let's leave dimensions as 0 for now or approx
                
                world_state.objects.append(obj_state)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f"TF Error: {e}")
                continue
        
        if world_state.objects:
            self.world_state_pub.publish(world_state)
            self.get_logger().info(f"Published state with {len(world_state.objects)} objects")

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectStateManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
