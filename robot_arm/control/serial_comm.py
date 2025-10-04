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
