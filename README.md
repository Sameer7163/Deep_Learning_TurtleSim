# Deep_Learning_TurtleSim
Training Results:
Ultralytics Version: YOLO11
Dataset: Mars Rock Detection dataset

Train.py uses YOLO11 to train on the mars rock detection dataset and store the results in runs/train4.
TurtlePathPlanner.py uses the model that we trained and uses that to detect the rocks in the video sr1.mp4. After detecting the rocks, turtlesim is used to trace the path of the rocks starting from smallest to the largest rock.
Overview
1.	Load Pre-Trained YOLO Model:
The script uses a YOLOv8 model to detect objects frame by frame in a video.
2.	Object Detection:
The bounding boxes of detected objects in each frame are saved as images (result.jpg).
3.	Coordinate Extraction:
Extracts object coordinates from each saved image.
4.	Coordinate Mapping:
Converts image coordinates to the Turtlesim world coordinate system.
5.	Turtle Path Planning:
Moves the Turtlesim turtle to each detected object location sequentially.
________________________________________
Modules and Dependencies
•	YOLOv8: For object detection.
•	OpenCV: For image processing and coordinate extraction.
•	ROS2 (rclpy): For interacting with the Turtlesim node.
•	Turtlesim: For simulating a turtle navigating the world.
________________________________________
Steps Performed by the Script
1.	Initialization:
The TurtlePathPlanner node initializes, subscribes to the turtle's pose, and creates a publisher to send movement commands.
2.	Object Detection:
The YOLO model detects objects in the video, saves bounding box predictions, and extracts object coordinates.
3.	Coordinate Conversion:
Converts the pixel coordinates of bounding boxes to the Turtlesim world coordinates for navigation.
4.	Turtle Navigation:
Moves the turtle in the Turtlesim world to the detected object positions using proportional control for linear and angular velocities.
5.	Path Completion:
After navigating to all detected object positions, the script logs a message indicating the completion of the path tracing.
________________________________________
Workflow
1.	Load YOLO Model:
The YOLO model is loaded with a specified path to the pre-trained weights.
2.	Object Detection on Video:
The video is processed frame by frame, and the results are saved as images.
3.	Extract Bounding Box Centers:
Each result image is processed to find the center of bounding boxes.
4.	Coordinate Mapping and Turtle Movement:
The centers are mapped to the Turtlesim world, and the turtle is commanded to move to each detected position.
________________________________________
Execution Steps
1.	Install YOLOv8 and ROS2: Ensure that YOLOv8 is installed with the following command:
pip install ultralytics
2.	Launch Turtlesim:
Start the Turtlesim node using the command:
ros2 run turtlesim turtlesim_node
3.	Run the Script:
Execute the Python script that performs object detection and moves the turtle.
________________________________________
Expected Output
•	The YOLO model detects objects in the video and saves bounding box predictions as images.
•	The Turtlesim turtle navigates to each detected object position in the simulation sequentially.
•	Upon reaching the last object position, the script logs a message indicating that the path tracing is complete.
________________________________________
 
     
