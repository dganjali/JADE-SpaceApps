# Robotic Arm Project (modular scaffold)

This scaffold implements a modular robotic arm stack (vision → planning → control → Arduino firmware).

Overview
- Vision: detect debris using simple color/contour detector or swap in an ML model.
- Planning: inverse kinematics and simple trajectory smoothing.
- Control: serial communication with Arduino for servos and high-level arm commands.
- Arduino firmware: simple parser that accepts semicolon-delimited servo targets.

Where to add measurements
- `robot_arm/config.py`: put your arm link lengths (P1, P2, P3 considered as link1, link2, gripper offset), servo pin mapping, camera intrinsics (focal length) and any offsets. Update these to match your hardware.
- If your arm has 1 yaw + 2 pitches + 1 claw, set DOF = {"yaw":1, "pitches":2, "claw":1} in `config.py`. The kinematics module will use the 2-link planar IK + yaw.

Quick start
1. Edit `robot_arm/config.py` to match your link lengths and serial port.
2. Install Python deps: `pip install -r robot_arm/requirements.txt`.
3. Upload `robot_arm/arduino/arm_controller.ino` to your Arduino (adjust pins in that file).
4. Run the high-level demo (after configuring camera indices and serial port):

   python3 robot_arm/main.py

Notes
- The vision code contains simple color/contour-based detection as a starting point.
- The IK solver supports either a 2-link planar arm (recommended for 2 pitches) or will fallback to the 3-link solver from the legacy code if you enable it.
- Wherever the code needs hardware-specific measurements (link lengths, focal length for depth by size, servo pin numbers) it's clearly marked in `config.py` and README.

Files
- `vision/` - detection, depth estimation, handoff tracker for camera A→B
- `planning/` - kinematics and trajectory smoothing
- `control/` - serial comm and arm command wrappers
- `arduino/arm_controller.ino` - firmware
- `main.py` - small demo that wires the pieces together

If you want, I can: wire up a simple unit test for the IK, or adapt the Arduino sketch to send feedback. Tell me which you'd prefer next.
