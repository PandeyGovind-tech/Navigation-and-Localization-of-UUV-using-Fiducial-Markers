"""
This script uses two cameras to detect AprilTags, estimate their 3D position and orientation,
and send this data via MAVLink. It also provides a web-based interface for
real-time video streams and a 3D plot of detected tag positions using Flask.

The script is designed for a Raspberry Pi or similar single-board computer,
interfacing with a flight controller (like a Pixhawk) via a serial connection.
"""
import cv2
import numpy as np
import pupil_apriltags as apriltag
import threading
from flask import Flask, Response
import time
from pymavlink import mavutil
import io
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

# --- Global Variables ---
master = None
positions = []  # Store tag positions for plotting
last_plot_time = 0
PLOT_INTERVAL = 2  # seconds

# --- MAVLink Connection Setup ---
try:
    master = mavutil.mavlink_connection('/dev/serial0', baud=57600)
    master.wait_heartbeat()
    print("MAVLink connection established.")
except Exception as e:
    print(f"MAVLink connection failed: {e}")
    print("Continuing without MAVLink. Position data will not be sent.")

# --- Flask App Setup ---
app = Flask(__name__)

# --- Helper Functions ---
def rotation_matrix_to_euler_angles(R):
    """
    Converts a 3x3 rotation matrix to Euler angles (roll, pitch, yaw).
    """
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6

    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0
    return [roll, pitch, yaw]

# --- Camera Initialization ---
def init_camera(index):
    """
    Initializes a camera with specific resolution settings.
    """
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        print(f"Camera at index {index} could not be opened.")
        return None
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    return cap

cap1 = init_camera(0)
cap2 = init_camera(1)

if not cap1 and not cap2:
    print("No cameras available. Exiting.")
    exit()

# --- AprilTag Detector and Camera Parameters ---
detector = apriltag.Detector()

predefined_positions = {
    0: np.array([282, 0, 158]),
    5: np.array([190, 789, 69]),
    2: np.array([571, 521, 137]),
    4: np.array([378, 974, 105]),
    1: np.array([0, 382, 137]),
    6: np.array([290, 0, 173]),
    11: np.array([182, 789, 84]),
    8: np.array([571, 529, 152]),
    10: np.array([370, 974, 120]),
    7: np.array([0, 374, 152]),
}

# Calibration matrices (example values, should be replaced with real calibration data)
camera_matrix1 = np.array([[280, 0, 350], [0, 280, 211], [0, 0, 1]], dtype=float)
camera_matrix2 = np.array([[510, 0, 205], [0, 303, 150], [0, 0, 1]], dtype=float)
dist_coeffs1 = np.zeros((4, 1))
dist_coeffs2 = np.zeros((4, 1))

# --- Core Processing Logic ---
def process_frame(cap, camera_matrix, dist_coeffs, cam_id):
    """
    Captures a frame, detects AprilTags, and calculates position and orientation.
    Sends data via MAVLink and returns the annotated frame.
    """
    global last_plot_time
    success, frame = cap.read()
    if not success:
        return None

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    results = detector.detect(gray)

    # Prioritize detection of a specific tag set
    tags_0_3 = [r for r in results if r.tag_id in [0, 5, 2, 4]]
    tags_7_10 = [r for r in results if r.tag_id in [6, 11, 8, 10]]
    target_tags = tags_0_3 if tags_0_3 else tags_7_10

    for res in target_tags:
        tag_id = res.tag_id
        corners = res.corners.astype(float)
        tag_size = 0.175 if tag_id in [0, 1, 2, 3] else 0.085 # Tag sizes in meters

        obj_points = np.array([
            [-tag_size / 2, -tag_size / 2, 0],
            [tag_size / 2, -tag_size / 2, 0],
            [tag_size / 2, tag_size / 2, 0],
            [-tag_size / 2, tag_size / 2, 0]
        ])

        success, rvec, tvec = cv2.solvePnP(obj_points, corners, camera_matrix, dist_coeffs)
        if not success:
            continue

        tvec = tvec.flatten()
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        roll, pitch, yaw = rotation_matrix_to_euler_angles(rotation_matrix)

        tag_global = predefined_positions.get(tag_id)
        if tag_global is None:
            continue

        # Calculate camera's global position
        R_cam_to_tag = rotation_matrix
        R_tag_to_cam = R_cam_to_tag.T
        t_tag_to_cam = -R_tag_to_cam @ tvec.reshape(3, 1)
        cam_global = tag_global.reshape(3, 1) / 100 + t_tag_to_cam  # Convert predefined to meters

        # Prepare and send MAVLink message
        timestamp_us = int(time.time() * 1e6)
        x, y, z = cam_global.flatten()
        
        if master:
            try:
                master.mav.vision_position_estimate_send(
                    timestamp_us, x, y, z, roll, pitch, yaw
                )
            except Exception as e:
                print(f"Error sending MAVLink message: {e}")

        # Update position for the 3D plot
        current_time = time.time()
        if current_time - last_plot_time >= PLOT_INTERVAL:
            positions.append((x, y, z))
            last_plot_time = current_time

        if len(positions) > 100:
            positions.pop(0)

        # Draw annotations on the frame
        (ptA, _, _, _) = corners
        cX, cY = int(res.center[0]), int(res.center[1])
        cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
        cv2.putText(frame, f"Cam{cam_id} ID: {tag_id}", (int(ptA[0]), int(ptA[1] - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        cv2.putText(frame, f"Pos: ({x:.2f}, {y:.2f}, {z:.2f})",
                    (int(ptA[0]), int(ptA[1] + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        cv2.putText(frame, f"Orientation: ({np.degrees(roll):.1f}, {np.degrees(pitch):.1f}, {np.degrees(yaw):.1f})",
                    (int(ptA[0]), int(ptA[1] + 40)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    return frame

# --- Flask Routes ---
def generate_frames(camera_id):
    """
    Generator function to stream video frames.
    """
    while True:
        if camera_id == 1 and cap1:
            frame = process_frame(cap1, camera_matrix1, dist_coeffs1, 1)
        elif camera_id == 2 and cap2:
            frame = process_frame(cap2, camera_matrix2, dist_coeffs2, 2)
        else:
            break

        if frame is None:
            time.sleep(0.1)
            continue

        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/video_feed1')
def video_feed1():
    """
    Video streaming route for camera 1.
    """
    return Response(generate_frames(1), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed2')
def video_feed2():
    """
    Video streaming route for camera 2.
    """
    return Response(generate_frames(2), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/plot_feed')
def plot_feed():
    """
    Generates and serves a 3D plot of the detected tag positions.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    if positions:
        xs, ys, zs = zip(*positions)
        ax.plot(xs, ys, zs, marker='o')
        ax.scatter(xs[0], ys[0], zs[0], color='red', label='Start')
        ax.legend()

    ax.set_xlim([0, 10])
    ax.set_ylim([0, 10])
    ax.set_zlim([0, 10])

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title("Tag Position Plot")

    canvas = FigureCanvas(fig)
    buf = io.BytesIO()
    canvas.print_png(buf)
    plt.close(fig)

    return Response(buf.getvalue(), mimetype='image/png')

@app.route('/')
def index():
    """
    Main web page to display camera streams and plot.
    """
    return '''
    <html>
    <head>
        <title>Dual AprilTag Camera Stream & Plot</title>
        <style>
            body { font-family: sans-serif; }
            .container {
                display: flex;
                flex-direction: row;
                justify-content: space-around;
                align-items: flex-start;
                flex-wrap: wrap; /* Added for responsiveness */
            }
            .section {
                margin: 20px;
                text-align: center;
            }
            img {
                border: 1px solid #ccc;
                max-width: 100%; /* Make images responsive */
                height: auto;
            }
        </style>
    </head>
    <body>
        <h1>AprilTag Camera Streams and 3D Plot</h1>
        <div class="container">
            <div class="section">
                <h2>Camera 1</h2>
                <img src="/video_feed1" width="320">
            </div>
            <div class="section">
                <h2>Camera 2</h2>
                <img src="/video_feed2" width="320">
            </div>
            <div class="section">
                <h2>3D Tag Position Plot</h2>
                <img src="/plot_feed" width="500" id="plot">
            </div>
        </div>
        <script>
            setInterval(() => {
                const plot = document.getElementById('plot');
                // Use a unique query parameter to prevent browser caching
                plot.src = '/plot_feed?' + new Date().getTime(); 
            }, 1000);
        </script>
    </body>
    </html>
    '''

# --- Main Execution Block ---
if __name__ == '__main__':
    # Start the Flask web server
    app.run(host='0.0.0.0', port=5000)