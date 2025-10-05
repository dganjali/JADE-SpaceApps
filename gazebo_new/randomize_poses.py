#!/usr/bin/env python3
"""
Randomize spacecraft poses with ~30m separation
Modifies leo_advanced.sdf in-place
"""
import random
import re
import sys

def generate_random_pose(target_distance=30.0):
    """Generate random positions ~30m apart with random orientations"""
    # Chaser at origin with random rotation
    chaser_x, chaser_y, chaser_z = 0.0, 0.0, 0.0
    chaser_roll = random.uniform(-0.5, 0.5)
    chaser_pitch = random.uniform(-0.5, 0.5)
    chaser_yaw = random.uniform(-0.5, 0.5)
    
    # Dock at random position ~30m away with random rotation
    # Generate random direction
    theta = random.uniform(0, 2 * 3.14159)
    phi = random.uniform(-0.5, 0.5)  # Limit vertical angle
    
    # Add some variance to distance (25-35m)
    distance = random.uniform(25.0, 35.0)
    
    dock_x = chaser_x + distance * (1 - phi**2)**0.5 * (theta % 3.14159 - 1.5708)
    dock_y = chaser_y + distance * (1 - phi**2)**0.5 * (1 - abs(theta % 3.14159 - 1.5708))
    dock_z = chaser_z + distance * phi
    
    dock_roll = random.uniform(-0.7, 0.7)
    dock_pitch = random.uniform(-0.7, 0.7)
    dock_yaw = random.uniform(-0.7, 0.7)
    
    return {
        'chaser': f"{chaser_x:.2f} {chaser_y:.2f} {chaser_z:.2f} {chaser_roll:.3f} {chaser_pitch:.3f} {chaser_yaw:.3f}",
        'dock': f"{dock_x:.2f} {dock_y:.2f} {dock_z:.2f} {dock_roll:.3f} {dock_pitch:.3f} {dock_yaw:.3f}"
    }

def main():
    world_file = 'worlds/leo_advanced.sdf'
    
    # Read the file
    with open(world_file, 'r') as f:
        content = f.read()
    
    # Generate new poses
    poses = generate_random_pose()
    
    # Replace chaser pose
    content = re.sub(
        r'(<model name="chaser">.*?<pose>)[^<]+(</pose>)',
        r'\g<1>' + poses['chaser'] + r'\g<2>',
        content,
        flags=re.DOTALL
    )
    
    # Replace dock pose
    content = re.sub(
        r'(<model name="dock">.*?<pose>)[^<]+(</pose>)',
        r'\g<1>' + poses['dock'] + r'\g<2>',
        content,
        flags=re.DOTALL
    )
    
    # Write back
    with open(world_file, 'w') as f:
        f.write(content)
    
    print(f"✅ Randomized poses:")
    print(f"   Chaser: {poses['chaser']}")
    print(f"   Dock:   {poses['dock']}")
    distance = ((float(poses['dock'].split()[0])**2 + 
                 float(poses['dock'].split()[1])**2 + 
                 float(poses['dock'].split()[2])**2)**0.5)
    print(f"   Separation: {distance:.1f}m")

if __name__ == '__main__':
    main()
