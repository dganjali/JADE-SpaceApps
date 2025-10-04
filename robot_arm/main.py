"""Minimal demo that runs the vision -> planning -> control loop.

Edit `robot_arm/config.py` for your serial port, link lengths, and camera indices.
"""
import time
import cv2
from robot_arm.vision.detect import detect_debris
from robot_arm.vision.depth import size_based_depth
from robot_arm.vision.tracker import image_to_arm_coords, should_switch_to_closeup
from robot_arm.planning.kinematics import IKSolver2R
from robot_arm.control.serial_comm import SerialComm
from robot_arm.control.arm_control import ArmController
import robot_arm.config as cfg
import math

def main():
    # Create IK solver
    ik = IKSolver2R(cfg.P1, cfg.P2, cfg.P3)

    # Serial
    serial = SerialComm(cfg.SERIAL_PORT, cfg.BAUDRATE)
    try:
        serial.open()
    except Exception as e:
        print(f"Warning: couldn't open serial port {cfg.SERIAL_PORT}: {e}")

    arm = ArmController(ik, serial)

    # Open camera A
    capA = cv2.VideoCapture(cfg.CAMERA_A_INDEX)
    capB = cv2.VideoCapture(cfg.CAMERA_B_INDEX)

    cam_params = {"fx": cfg.FOCAL_LENGTH_PIX, "fy": cfg.FOCAL_LENGTH_PIX, "cx": capA.get(cv2.CAP_PROP_FRAME_WIDTH)/2, "cy": capA.get(cv2.CAP_PROP_FRAME_HEIGHT)/2}

    try:
        while True:
            ret, frame = capA.read()
            if not ret:
                print("No frame from Camera A")
                time.sleep(0.1)
                continue

            det = detect_debris(frame)
            if det is None:
                # nothing found
                cv2.imshow("A", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            # estimate depth via size-based method
            z = None
            if cfg.KNOWN_DEBRIS_WIDTH_CM and det['w'] > 0:
                z = size_based_depth(det['w'], cfg.KNOWN_DEBRIS_WIDTH_CM, cfg.FOCAL_LENGTH_PIX)

            # map to arm coordinates (approx)
            x_arm, y_arm, z_arm = image_to_arm_coords(det['cx'], det['cy'], z if z is not None else 10.0, cam_params)
            print(f"Detected at pixels {det['cx']},{det['cy']} area={det['area']} -> approx arm coords {x_arm:.1f},{y_arm:.1f},{z_arm:.1f}")

            # decide to switch to closeup
            if should_switch_to_closeup(z_arm, threshold=10.0):
                print("Switching to close-up camera B for fine alignment")
                # capture from camera B and run a quick local alignment
                retb, frameb = capB.read()
                if retb:
                    detb = detect_debris(frameb)
                    if detb:
                        # recompute coordinates in arm frame (simple)
                        xb, yb, zb = image_to_arm_coords(detb['cx'], detb['cy'], z_arm, {"fx": cfg.FOCAL_LENGTH_PIX, "fy": cfg.FOCAL_LENGTH_PIX, "cx": capB.get(cv2.CAP_PROP_FRAME_WIDTH)/2, "cy": capB.get(cv2.CAP_PROP_FRAME_HEIGHT)/2})
                        x_arm, y_arm, z_arm = xb, yb, zb

            # Send move command (phi keep horizontal)
            phi = 0.0
            try:
                arm.move_to(x_arm, y_arm, z_arm, phi_rad=phi, steps=10, claw_open=1)
                # close claw
                arm.close_claw()
                time.sleep(1)
                # lift a bit
                arm.move_to(x_arm, y_arm, z_arm+5, phi_rad=phi, steps=8, claw_open=0)
            except Exception as e:
                print(f"Move failed: {e}")

            cv2.imshow("A", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        capA.release()
        capB.release()
        cv2.destroyAllWindows()
        try:
            serial.close()
        except:
            pass

if __name__ == "__main__":
    main()
