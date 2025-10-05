"""Serial communication helper to send servo targets to Arduino.
"""
import serial
import time
from typing import Iterable

class SerialComm:
    def __init__(self, port: str, baud: int = 9600, timeout: float = 1.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        # give Arduino time to reset
        time.sleep(2)
        # Zero micro-servos on startup (safe neutral positions)
        try:
            self.zero_servos()
        except Exception as e:
            # don't crash if zeroing fails; just warn
            print(f"Warning: failed to zero servos on open(): {e}")

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send_targets(self, angles: Iterable[float]):
        """angles: (yaw, p1, p2, p3, roll, claw) - numeric

        Sends a semicolon-delimited line ending with newline.
        """
        yaw, p1, p2, p3, roll, claw = angles
        line = f"yaw:{yaw:.2f};p1:{p1:.2f};p2:{p2:.2f};p3:{p3:.2f};roll:{roll:.2f};cw:{claw:.2f}\n"
        if self.ser and self.ser.is_open:
            self.ser.write(line.encode())
        else:
            raise RuntimeError("Serial port not open")

    def zero_servos(self):
        """Send safe neutral/mid angles to all servos (yaw,p1,p2,p3,roll,claw).

        Adjust neutral values here to match your hardware; defaults are conservative.
        """
        # neutral positions (degrees) and claw open
        neutral = (90.0, 90.0, 90.0, 90.0, 0.0, 1.0)
        # if the port isn't open yet, open temporarily to send
        opened_here = False
        if not (self.ser and self.ser.is_open):
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(2)
            opened_here = True

        try:
            yaw, p1, p2, p3, roll, claw = neutral
            line = f"yaw:{yaw:.2f};p1:{p1:.2f};p2:{p2:.2f};p3:{p3:.2f};roll:{roll:.2f};cw:{claw:.2f}\n"
            self.ser.write(line.encode())
            time.sleep(0.5)
        finally:
            if opened_here:
                try:
                    self.ser.close()
                except:
                    pass
