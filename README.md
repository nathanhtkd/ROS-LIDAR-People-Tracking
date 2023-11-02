# ROS-LIDAR-People-Tracking

This ROS2-based system utilizes LIDAR data to accurately identify, track, and count unique individuals moving within a robot's vicinity. The system processes laser range finder data to detect people in real-time, estimating the center location of each person and publishing the count of distinct individuals observed.

## Features

-   **Person Detection**: Converts LIDAR scans to point clouds, applying clustering algorithms to detect individual people.
-   **Tracking**: Implements tracking mechanisms to follow people across multiple frames of LIDAR data.
-   **Counting**: Maintains a cumulative count of unique individuals detected since activation.
-   **Modular Design**: Comprises multiple nodes with single responsibilities, promoting the ROS philosophy of simplicity and reusability.
-   **Efficient Communication**: Nodes communicate via custom-designed topics, ensuring optimal data flow and system performance.
-   **Automatic Execution**: Includes a launch file for easy initialization and coordination of playback and recording of bag files.
