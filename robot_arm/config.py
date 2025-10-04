"""Hardware and camera configuration.

Edit this file to match your arm measurements and camera intrinsics.
"""

# Arm link lengths in cm. For a yaw + 2-pitch arm use P1 (shoulder->elbow), P2 (elbow->wrist), P3 (wrist->gripper offset)
P1 = 12.0
P2 = 12.3
P3 = 8.0

# Degrees of freedom description
# Example: yaw + two pitches + claw
DOF = {"yaw": 1, "pitches": 2, "claw": 1}

# Serial port for Arduino (change to your device)
SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 9600

# Camera indices (OpenCV VideoCapture). Camera A = wide angle (global), Camera B = close-up (on arm)
CAMERA_A_INDEX = 0
CAMERA_B_INDEX = 1

# Simple camera intrinsics for size-based depth estimation (if you don't have calibrated stereo)
# Focal length in pixels (approx). Update using a calibration or measured focal length.
FOCAL_LENGTH_PIX = 800.0

# Physical known width of the debris object (cm) for size-based depth estimation. Set to None if unknown.
KNOWN_DEBRIS_WIDTH_CM = 2.0

# Servo mappings and limits (these values should mirror the Arduino sketch pins)
SERVO_PINS = {
    "yaw": 6,
    "p1": 7,
    "p2": 8,
    "p3": 9,  # optional, keep for compatibility
    "roll": 10,
    "claw": 11,
}

# Safety limits: reachable workspace radius in cm (approx)
WORKSPACE_RADIUS_CM = 30
