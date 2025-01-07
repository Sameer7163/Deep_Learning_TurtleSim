from ultralytics import YOLO

import cv2
import numpy as np
import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist



class TurtlePathPlanner(Node):

    def __init__(self):
        super().__init__('turtle_path_planner')
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.cmd_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.turtle_pose = Pose()
        self.get_logger().info("Turtle Path Planner Node Initialized.")

    def pose_callback(self, msg):
        self.turtle_pose = msg
    
    def spawn_turtle(self, x, y, theta, name="turtle1"):
        """Spawn a new turtle at a given position and orientation with a unique name."""
        self.get_logger().info(f"Spawning a new turtle '{name}'...")
        spawn_client = self.create_client(Spawn, '/spawn')

        while not spawn_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn("Waiting for Spawn service...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name  # Use a dynamic name for each turtle

        future = spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            self.get_logger().info(f"Turtle '{name}' spawned successfully at ({x}, {y})!")
        else:
            self.get_logger().error(f"Failed to spawn turtle '{name}'.")

    def move_turtle(self, target_x, target_y, name="turtle1"):
        tolerance = 0.5
        rate = self.create_rate(10)
        
        rclpy.spin_once(self)

        distance = math.sqrt((target_x - self.turtle_pose.x) ** 2 + (target_y - self.turtle_pose.y) ** 2)
        angle_to_goal = math.atan2(target_y - self.turtle_pose.y, target_x - self.turtle_pose.x)
        angle_difference = angle_to_goal - self.turtle_pose.theta

        vel_msg = Twist()
        vel_msg.linear.x = 1.5 * distance if distance > tolerance else 0.0
        vel_msg.angular.z = 4.0 * angle_difference if abs(angle_difference) > 0.1 else 0.0
        
        self.cmd_publisher.publish(vel_msg)
    
    def extract_bounding_boxes(self, image_path):
        image = cv2.imread(image_path)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centers = []

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            cx, cy = x + w // 2, y + h // 2
            centers.append((cx, cy))

        return centers

    def plan_path(self, image_path,i):
        # Extract bounding box centers
        centers = self.extract_bounding_boxes(image_path)
        self.get_logger().info(f"Extracted Centers: {centers}")

        # Map image coordinates to turtlesim world
        image_width, image_height = 1280, 720  # Replace with the actual dimensions of result.jpg
        world_positions = [
            (cx * 11.0 / image_width, cy * 11.0 / image_height) for cx, cy in centers
        ]
        

        # Spawn and move to each detected position with a unique turtle
        for i, (x, y) in enumerate(world_positions):
            # Spawn the turtle
            self.get_logger().info(f"Moving turtle  to target ({x:.2f}, {y:.2f})")
            self.move_turtle(x, y)  # Move the turtle to the target position
            self.get_logger().info(f"Turtle reached target ({x:.2f}, {y:.2f})")

        self.get_logger().info("Path tracing complete!")


def main(args=None):
    rclpy.init(args=args)
    planner = TurtlePathPlanner()

    #Use the model we trained to predict and detect objects from the video
    model = YOLO('/home/vboxuser/proj/runs/detect/train4/weights/best.pt')

    #Use this video to detect the objects in the video and store the predictions in results
    results = model.predict('/home/vboxuser/proj/sr1.mp4')

    #For each frame in the video trace the objects using turtlesim
    for i,result in enumerate(results):
        boxes = result.boxes  # Boxes object for bounding box outputs
        masks = result.masks  # Masks object for segmentation masks outputs
        keypoints = result.keypoints  # Keypoints object for pose outputs
        probs = result.probs  # Probs object for classification outputs
        obb = result.obb  # Oriented boxes object for OBB outputs
        result.show()  # display to screen
        result.save(filename="result.jpg")  # save to disk  
        
        image_path = '/home/vboxuser/proj/result.jpg'  # Replace with your result image path
        planner.plan_path(image_path,i)

        
    planner.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
