#!/usr/bin/env python3
"""
Generate randomized orbital docking scenarios
Creates SDF world files with random initial positions and orientations
"""

import random
import math
import sys

def generate_random_pose():
    """Generate random position and orientation"""
    # Random position in space (50-100m distance)
    distance = random.uniform(50, 100)
    theta = random.uniform(0, 2 * math.pi)
    phi = random.uniform(-math.pi/4, math.pi/4)
    
    x = distance * math.cos(theta) * math.cos(phi)
    y = distance * math.sin(theta) * math.cos(phi)
    z = distance * math.sin(phi)
    
    # Random orientation (roll, pitch, yaw in radians)
    roll = random.uniform(-math.pi, math.pi)
    pitch = random.uniform(-math.pi/2, math.pi/2)
    yaw = random.uniform(-math.pi, math.pi)
    
    return (x, y, z, roll, pitch, yaw)

def generate_scenario(filename="leo_random.sdf", seed=None):
    """Generate a random scenario world file"""
    if seed is not None:
        random.seed(seed)
    
    # Chaser always starts at origin with random orientation
    chaser_x, chaser_y, chaser_z = 0, 0, 0
    chaser_roll = random.uniform(-math.pi, math.pi)
    chaser_pitch = random.uniform(-math.pi/2, math.pi/2)
    chaser_yaw = random.uniform(-math.pi, math.pi)
    
    # Dock at random position with random orientation
    dock_x, dock_y, dock_z, dock_roll, dock_pitch, dock_yaw = generate_random_pose()
    
    print(f"🎲 Generating Random Scenario (seed={seed})")
    print(f"=" * 60)
    print(f"Chaser Pose: pos=({chaser_x:.1f}, {chaser_y:.1f}, {chaser_z:.1f}) m")
    print(f"             rot=({math.degrees(chaser_roll):.1f}\u00b0, {math.degrees(chaser_pitch):.1f}\u00b0, {math.degrees(chaser_yaw):.1f}\u00b0)")
    print(f"Dock Pose:   pos=({dock_x:.1f}, {dock_y:.1f}, {dock_z:.1f}) m")
    print(f"             rot=({math.degrees(dock_roll):.1f}\u00b0, {math.degrees(dock_pitch):.1f}\u00b0, {math.degrees(dock_yaw):.1f}\u00b0)")
    
    distance = math.sqrt(dock_x**2 + dock_y**2 + dock_z**2)
    print(f"Initial Distance: {distance:.1f} m")
    print(f"=" * 60)
    
    # Read template
    with open('worlds/leo_advanced.sdf', 'r') as f:
        content = f.read()
    
    # Replace chaser pose
    chaser_pose_old = content.split('<model name="chaser">')[1].split('</pose>')[0].split('<pose>')[1]
    chaser_pose_new = f"{chaser_x} {chaser_y} {chaser_z} {chaser_roll:.3f} {chaser_pitch:.3f} {chaser_yaw:.3f}"
    content = content.replace(
        f'<model name="chaser">\n      <!-- Random initial orientation: roll=45°, pitch=-30°, yaw=60° -->\n      <pose>{chaser_pose_old}</pose>',
        f'<model name="chaser">\n      <!-- Random orientation: r={math.degrees(chaser_roll):.1f}°, p={math.degrees(chaser_pitch):.1f}°, y={math.degrees(chaser_yaw):.1f}° -->\n      <pose>{chaser_pose_new}</pose>'
    )
    
    # Replace dock pose
    dock_pose_old = content.split('<model name="dock">')[1].split('</pose>')[0].split('<pose>')[1]
    dock_pose_new = f"{dock_x:.1f} {dock_y:.1f} {dock_z:.1f} {dock_roll:.3f} {dock_pitch:.3f} {dock_yaw:.3f}"
    content = content.replace(
        f'<model name="dock">\n      <!-- Random initial orientation: roll=-50°, pitch=40°, yaw=-70° -->\n      <pose>{dock_pose_old}</pose>',
        f'<model name="dock">\n      <!-- Random orientation: r={math.degrees(dock_roll):.1f}°, p={math.degrees(dock_pitch):.1f}°, y={math.degrees(dock_yaw):.1f}° -->\n      <pose>{dock_pose_new}</pose>'
    )
    
    # Write output
    output_path = f'worlds/{filename}'
    with open(output_path, 'w') as f:
        f.write(content)
    
    print(f"✅ Generated: {output_path}")
    return output_path

if __name__ == "__main__":
    seed = None
    filename = "leo_random.sdf"
    
    if len(sys.argv) > 1:
        try:
            seed = int(sys.argv[1])
        except ValueError:
            filename = sys.argv[1]
    
    if len(sys.argv) > 2:
        filename = sys.argv[2]
    
    generate_scenario(filename, seed)
