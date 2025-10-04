#!/usr/bin/env python3
import argparse
import math
import os
import random
import signal
import sys
import time

try:
    from gz.transport import Node
    from gz.msgs.double_pb2 import Double
    from gz.msgs.boolean_pb2 import Boolean
except Exception as e:
    print("Failed to import Gazebo Python bindings (gz.transport, gz.msgs). Make sure they are installed and on PYTHONPATH.")
    print(e)
    sys.exit(1)

# Random walk generator for thruster commands

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default='training_satellite', help='Model name in Gazebo')
    parser.add_argument('--thrusters', nargs='*', default=['thruster_px','thruster_nx','thruster_py','thruster_ny','thruster_pz','thruster_nz'])
    parser.add_argument('--rate', type=float, default=10.0, help='Publish rate Hz')
    parser.add_argument('--maxN', type=float, default=2.0, help='Max thrust Newtons per thruster')
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args()

    random.seed(args.seed)

    node = Node()

    pubs = {}
    for thr in args.thrusters:
        topic = f"/model/{args.model}/thrusters/{thr}"
        pubs[thr] = node.advertise(topic, Double)

    print("Random-walk thruster publisher running. Ctrl-C to stop.")

    running = True
    def stop(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    dt = 1.0 / max(1e-3, args.rate)
    state = {thr: 0.0 for thr in args.thrusters}

    while running:
        for thr in args.thrusters:
            # Random walk step
            step = random.uniform(-0.2, 0.2)
            val = max(-args.maxN, min(args.maxN, state[thr] + step))
            state[thr] = val
            msg = Double()
            msg.data = val
            pubs[thr].publish(msg)
        time.sleep(dt)

    # zero on exit
    zero_pub = node.advertise(f"/model/{args.model}/thrusters/zero", Boolean)
    zb = Boolean(); zb.data = True
    zero_pub.publish(zb)
    print("Stopped. Zero thrust sent.")

if __name__ == '__main__':
    main()
