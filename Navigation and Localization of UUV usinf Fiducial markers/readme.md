# AprilTag Vision-Based Navigation System

This project provides a complete solution for real-time vision-based navigation using AprilTags and MAVLink. It uses two cameras to detect AprilTags, calculates the 3D position and orientation of the camera relative to the tags, and sends this data to a flight controller via MAVLink.

A web-based interface is included to stream the live camera feeds and visualize the camera's trajectory in a 3D plot.

## Features
* **Dual-Camera Support:** Simultaneously processes video from two cameras.
* **AprilTag Detection:** Detects AprilTags and estimates their pose (position and orientation).
* **MAVLink Integration:** Sends `VISION_POSITION_ESTIMATE` messages to a flight controller.
* **Web Interface:** Streams live video feeds and a real-time 3D plot of the camera's position.
* **Modular Design:** Easy to modify for different camera setups or tag configurations.

## Prerequisites
* A Linux-based system (like a Raspberry Pi 4 or Jetson Nano)
* Two USB or CSI cameras
* A MAVLink-compatible flight controller (e.g., Pixhawk)
* A serial connection between your computer and the flight controller (e.g., `/dev/serial0`)

## Installation

1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/your-username/your-repo-name.git](https://github.com/your-username/your-repo-name.git)
    cd your-repo-name
    ```

2.  **Install Python dependencies:**
    It is highly recommended to use a virtual environment.
    ```bash
    python3 -m venv venv
    source venv/bin/activate
    pip install -r requirements.txt
    ```

## Usage

1.  **Hardware Setup:**
    * Connect your two cameras to the system.
    * Ensure the flight controller is connected via a serial port (e.g., `/dev/serial0`).

2.  **Configuration:**
    * Edit `vision_tracking.py` to update the `predefined_positions` dictionary with the known real-world coordinates of your AprilTags. These values should be in millimeters.
    * **Crucially**, you must perform **camera calibration** and replace the placeholder `camera_matrix1`, `camera_matrix2`, `dist_coeffs1`, and `dist_coeffs2` values with your own calibration results. Incorrect calibration will lead to inaccurate pose estimation.

3.  **Run the script:**
    ```bash
    python vision_tracking.py
    ```

4.  **Access the Web Interface:**
    Open a web browser and navigate to `http://<your-pi-ip-address>:5000`. You will see the two camera feeds and the 3D plot.

## How it Works

The script works by:
1.  Initializing a MAVLink connection to the flight controller.
2.  Starting two camera streams in separate threads.
3.  In the `process_frame` function, it detects AprilTags in each frame.
4.  It uses the `cv2.solvePnP` function to calculate the camera's pose (position and orientation) relative to the detected tag.
5.  Using the known global position of the AprilTag (`predefined_positions`), it transforms the camera's pose from the tag's local coordinate system to the global coordinate system.
6.  The calculated global position and orientation are sent to the flight controller as a `VISION_POSITION_ESTIMATE` MAVLink message.
7.  The script also runs a Flask web server to provide a visual interface for monitoring.

## Authors

Govind Pandey
Aryan Singh

## Contributing

Feel free to open issues or submit pull requests.