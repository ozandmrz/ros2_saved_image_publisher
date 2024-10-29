### GitHub Repository Description (English)

# Saved Image Publisher

This project is designed to publish images from a specified directory in a ROS2 environment based on timestamps read from a `timestamps.txt` file. It aims to facilitate image streaming and management in robotic systems utilizing ROS2.

## Features

- **Image Publishing**: Publishes images from a specified directory through ROS2.
- **Timestamp Support**: Utilizes timestamps from `timestamps.txt` for sequential image publishing.
- **Feedback Mechanism**: Provides detailed feedback during file checks and reading operations.

## Requirements

- **ROS2**: This project is developed to work within a ROS2 environment.
- **Python**: Written in Python, with dependencies including:
  - `cv_bridge`: Bridges OpenCV with ROS2.
  - `sensor_msgs`: Defines image messages for ROS2.
  - `opencv-python`: Used for image processing.

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/username/saved_image_publisher.git
   cd saved_image_publisher
   ```

2. Install required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Create a workspace:
   ```bash
   mkdir -p ~/ws_publish/src
   cp -r saved_image_publisher ~/ws_publish/src/
   cd ~/ws_publish
   ```

4. Build with Colcon:
   ```bash
   colcon build
   ```

5. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

To publish images, run the following command:

```bash
ros2 run saved_image_publisher image_sender --ros-args -p image_folder:=/path/to/image/folder -p timestamp_file:=/path/to/timestamps.txt
```

Make sure to adjust the `image_folder` and `timestamp_file` parameters to your directory paths.
