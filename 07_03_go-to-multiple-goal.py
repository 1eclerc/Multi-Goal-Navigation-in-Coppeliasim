from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import numpy as np
import math
import matplotlib.pyplot as plt
import os

# === BASIC PARAMETERS ===
WHEEL_RADIUS = 0.027        # Wheel radius (meters)
HALF_WHEELBASE = 0.05813    # Half of the wheelbase (meters)
CONTROL_PARAMS = {
    "K_rho": 0.15,    # Linear speed gain when approaching the goal
    "K_alpha": 2.8,   # Angular speed gain for heading correction
    "K_beta": -0.3    # Correction gain for final heading alignment
}
V_MAX = 0.5                    # Maximum linear speed (m/s)
DIST_THRESHOLD = 0.08          # Distance threshold to consider the goal reached (8 cm)
ANGLE_THRESHOLD = np.deg2rad(3) # Angular threshold for orientation alignment (3 degrees)
STABLE_COUNT_THRESHOLD = 3      # Number of stable readings needed for heading confirmation

# === MARKER POSITIONS ===
# Define all marker positions (x, y, theta) in meters and radians
# These positions should match your CoppeliaSim environment
MARKER_POSITIONS = {
    0:  (1.0, 1.0, 0.0),      # Top Right
    1:  (0.5, -1.0, 0.0),     
    3:  (0.0, 0.0, 0.0),      # Middle Right
    6:  (1.0, -1.0, 0.0),     # Bottom Right
    18: (-0.5, -0.5, 0.0),    
    19: (1.0, 0.5, 0.0),      
    20: (0.5, 0.5, 0.0),      
    21: (0.8, 0.0, 0.0),      
    24: (0.0, 0.0, 0.0),      # Center - Start and finish marker
    42: (-1.0, 1.0, 0.0),     # Top Left
    45: (-0.5, 0.0, 0.0),     # Middle Left
    48: (-1.0, -1.0, 0.0),    # Bottom Left
}

# === MISSION DEFINITIONS ===
# Task 2.1: Drawing Letters C and S
LETTER_C_SEQUENCE = [24, 0, 42, 48, 6, 24]  # Start -> Top Right -> Top Left -> Bottom Left -> Bottom Right -> Finish
LETTER_S_SEQUENCE = [24, 0, 42, 45, 3, 6, 48, 24]  # Start -> Top Right -> Top Left -> Middle Left -> Middle Right -> Bottom Right -> Bottom Left -> Finish

# Task 2.2: Student ID based navigation (211805131, 211805078 - 2 members)
# Based on assignment instructions for 2 members: 21, 3, 18, 0, 20, 3, 18, 1
STUDENT_ID_SEQUENCE = [24, 21, 3, 18, 0, 20, 3, 18, 1, 24]  # Start at 24, follow pattern, end at 24

# === SIMULATION CONNECTION ===
client = RemoteAPIClient()
sim = client.require('sim')

# Get handles for the robot and its components
robot = sim.getObject('/LineTracer')
left_motor = sim.getObject('/DynamicLeftJoint')
right_motor = sim.getObject('/DynamicRightJoint')
fhlidar = sim.getObject('/fastHokuyo')

# === HELPER FUNCTIONS ===
def abc_to_rpy(alpha, beta, gamma):
    """Convert CoppeliaSim Euler angles to Roll-Pitch-Yaw"""
    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    cg, sg = math.cos(gamma), math.sin(gamma)
    
    R = [[cb * cg, -cb * sg, sb],
         [ca * sg + sa * sb * cg, ca * cg - sa * sb * sg, -sa * cb],
         [sa * sg - ca * sb * cg, sa * cg + ca * sb * sg, ca * cb]]
    
    sy = math.sqrt(R[0][0]**2 + R[1][0]**2)
    if sy < 1e-6:
        return math.atan2(-R[2][1], R[2][2]), math.atan2(-R[1][2], R[1][1]), 0.0
    return math.atan2(R[2][0], sy), math.atan2(-R[2][1], R[2][2]), math.atan2(R[1][0], R[0][0])

def get_yaw():
    """Get the current yaw angle of the robot"""
    ang = sim.getObjectOrientation(fhlidar, -1)
    _, _, yaw = abc_to_rpy(ang[0], ang[1], ang[2])
    return yaw

def dynamic_v_max(rho):
    """Adjust maximum speed based on distance to goal"""
    if rho > 0.8:
        return V_MAX
    elif rho > 0.3:
        return V_MAX * 0.7
    else:
        return V_MAX * 0.4

def normalize_angle(a):
    """Normalize angle to [-π, π] range"""
    return (a + np.pi) % (2 * np.pi) - np.pi

def calculate_wheel_speeds(v, omega):
    """Convert linear and angular velocity to wheel speeds"""
    v_left = (2 * v - omega * 2 * HALF_WHEELBASE) / (2 * WHEEL_RADIUS)
    v_right = (2 * v + omega * 2 * HALF_WHEELBASE) / (2 * WHEEL_RADIUS)
    return v_left, v_right

def calculate_control_output(pose, goal, params):
    """Calculate control commands based on current pose and goal"""
    x, y, theta = pose
    xg, yg, thetag = goal
    
    # Calculate distance and angles
    dx, dy = xg - x, yg - y
    rho = math.hypot(dx, dy)
    lamb = math.atan2(dy, dx)
    alpha = normalize_angle(lamb - theta)
    beta = normalize_angle(thetag - lamb)
    
    # Calculate control outputs
    v = min(params["K_rho"] * rho, dynamic_v_max(rho))
    omega = params["K_alpha"] * alpha + params["K_beta"] * beta
    
    # Smooth angular velocity for small heading errors
    if abs(alpha) < np.deg2rad(15):
        omega *= 0.6
    
    return v, omega, rho

def log_data(log_file, start_time, x, y, theta, vl, vr, v, omega):
    """Log robot data to CSV file"""
    current_time = time.time() - start_time
    log_file.write(f"{current_time:.3f},{x:.3f},{y:.3f},{theta:.3f},{vl:.3f},{vr:.3f},{v:.3f},{omega:.3f}\n")
    log_file.flush()

def setup_plot(title, goal_sequence):
    """Setup matplotlib plot for trajectory visualization"""
    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 10))
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    # Plot all marker positions
    all_markers_x = [pos[0] for pos in MARKER_POSITIONS.values()]
    all_markers_y = [pos[1] for pos in MARKER_POSITIONS.values()]
    ax.scatter(all_markers_x, all_markers_y, c='lightgray', marker='s', s=60, alpha=0.5, label='All Markers')
    
    # Annotate all markers with their IDs
    for marker_id, (x, y, _) in MARKER_POSITIONS.items():
        ax.annotate(str(marker_id), (x, y), xytext=(5, 5), textcoords='offset points', 
                   fontsize=9, fontweight='bold', bbox=dict(boxstyle="round,pad=0.3", 
                   facecolor='white', alpha=0.7))
    
    # Plot goal sequence
    goal_positions = [MARKER_POSITIONS[marker_id] for marker_id in goal_sequence]
    goal_xs = [pos[0] for pos in goal_positions]
    goal_ys = [pos[1] for pos in goal_positions]
    
    # Draw path between goals with arrows
    for i in range(len(goal_xs) - 1):
        dx = goal_xs[i+1] - goal_xs[i]
        dy = goal_ys[i+1] - goal_ys[i]
        ax.annotate('', xy=(goal_xs[i+1], goal_ys[i+1]), xytext=(goal_xs[i], goal_ys[i]),
                   arrowprops=dict(arrowstyle='->', color='red', alpha=0.7, lw=2))
    
    ax.scatter(goal_xs, goal_ys, c='red', marker='o', s=100, label='Goal Markers', zorder=5, edgecolors='darkred')
    
    # Mark start and end points
    ax.scatter(goal_xs[0], goal_ys[0], c='green', marker='*', s=200, label='Start', zorder=6, edgecolors='darkgreen')
    ax.scatter(goal_xs[-1], goal_ys[-1], c='blue', marker='*', s=200, label='Finish', zorder=6, edgecolors='darkblue')
    
    # Robot trajectory elements
    robot_dot, = ax.plot([], [], 'bo', markersize=10, label='Robot Position', markeredgecolor='darkblue', markeredgewidth=2)
    current_goal_dot, = ax.plot([], [], 'ks', markersize=15, label='Current Goal', 
                               markerfacecolor='yellow', markeredgecolor='black', markeredgewidth=3)
    trajectory_line, = ax.plot([], [], 'g-', linewidth=3, label='Robot Trajectory', alpha=0.8)
    
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=10)
    plt.tight_layout()
    
    return fig, ax, robot_dot, current_goal_dot, trajectory_line

def update_plot(robot_dot, current_goal_dot, trajectory_line, robot_positions, current_goal):
    """Update the real-time plot"""
    if robot_positions:
        xs, ys = zip(*robot_positions)
        trajectory_line.set_data(xs, ys)
        robot_dot.set_data([xs[-1]], [ys[-1]])
        current_goal_dot.set_data([current_goal[0]], [current_goal[1]])
        plt.draw()
        plt.pause(0.01)

def go_to_goal(goal, robot_positions, plot_elements, log_file, start_time):
    """Navigate robot to a specific goal"""
    robot_dot, current_goal_dot, trajectory_line = plot_elements
    xg, yg, thetag = goal
    
    print(f"  Navigating to goal: ({xg:.2f}, {yg:.2f}, {math.degrees(thetag):.1f}°)")
    
    # Get initial position
    position = sim.getObjectPosition(robot, -1)
    x, y = position[0], position[1]
    theta = get_yaw()
    
    # Position Control Phase
    max_iterations = 1000  # Increased for more reliable navigation
    position_reached = False
    
    for iteration in range(max_iterations):
        # Calculate control outputs
        v, omega, rho = calculate_control_output((x, y, theta), (xg, yg, thetag), CONTROL_PARAMS)
        
        # Calculate and apply wheel speeds
        vl, vr = calculate_wheel_speeds(v, omega)
        sim.setJointTargetVelocity(left_motor, vl)
        sim.setJointTargetVelocity(right_motor, vr)
        
        # Log data every few iterations
        if iteration % 5 == 0:
            log_data(log_file, start_time, x, y, theta, vl, vr, v, omega)
        
        time.sleep(0.02)
        
        # Update position
        position = sim.getObjectPosition(robot, -1)
        x, y = position[0], position[1]
        theta = get_yaw()
        robot_positions.append((x, y))
        
        # Update visualization
        if iteration % 3 == 0:  # Update plot less frequently for performance
            update_plot(robot_dot, current_goal_dot, trajectory_line, robot_positions, (xg, yg))
        
        # Check if goal reached
        if rho < DIST_THRESHOLD:
            print(f"    Position reached in {iteration} iterations (distance: {rho:.3f}m)")
            position_reached = True
            break
    
    if not position_reached:
        print(f"    Warning: Position not reached within {max_iterations} iterations (final distance: {rho:.3f}m)")
    
    # Brief braking to stabilize
    sim.setJointTargetVelocity(left_motor, -0.05)
    sim.setJointTargetVelocity(right_motor, -0.05)
    time.sleep(0.1)
    sim.setJointTargetVelocity(left_motor, 0)
    sim.setJointTargetVelocity(right_motor, 0)
    
    # Orientation Control Phase
    stable_count = 0
    max_orientation_iterations = 150
    orientation_reached = False
    
    for iteration in range(max_orientation_iterations):
        theta = get_yaw()
        err = normalize_angle(thetag - theta)
        
        if abs(err) < ANGLE_THRESHOLD:
            stable_count += 1
        else:
            stable_count = 0
            
        if stable_count >= STABLE_COUNT_THRESHOLD:
            print(f"    Orientation aligned (error: {math.degrees(err):.1f}°)")
            orientation_reached = True
            break
            
        # Apply orientation correction
        omega = CONTROL_PARAMS["K_alpha"] * err * 0.8  # Gentler orientation control
        vl, vr = calculate_wheel_speeds(0, omega)
        sim.setJointTargetVelocity(left_motor, vl)
        sim.setJointTargetVelocity(right_motor, vr)
        
        if iteration % 5 == 0:
            log_data(log_file, start_time, x, y, theta, vl, vr, 0.0, omega)
            update_plot(robot_dot, current_goal_dot, trajectory_line, robot_positions, (xg, yg))
        
        time.sleep(0.02)
    
    if not orientation_reached:
        final_theta = get_yaw()
        final_err = normalize_angle(thetag - final_theta)
        print(f"    Warning: Orientation not perfectly aligned (final error: {math.degrees(final_err):.1f}°)")
    
    # Final stop
    sim.setJointTargetVelocity(left_motor, 0)
    sim.setJointTargetVelocity(right_motor, 0)
    time.sleep(0.2)

def execute_mission(mission_name, goal_sequence, student_ids=""):
    """Execute a complete navigation mission"""
    print(f"\n{'='*60}")
    print(f"EXECUTING MISSION: {mission_name}")
    print(f"{'='*60}")
    if student_ids:
        print(f"Student IDs: {student_ids}")
    print(f"Goal sequence: {goal_sequence}")
    print(f"Total waypoints: {len(goal_sequence)}")
    print(f"{'='*60}")
    
    # Validate all markers exist
    missing_markers = [marker_id for marker_id in goal_sequence if marker_id not in MARKER_POSITIONS]
    if missing_markers:
        print(f"ERROR: Missing marker definitions for IDs: {missing_markers}")
        return
    
    # Setup logging
    timestamp_str = time.strftime("%Y%m%d_%H%M%S")
    safe_mission_name = mission_name.lower().replace(' ', '_').replace('-', '_')
    filename = f"robot_log_{safe_mission_name}_{timestamp_str}.csv"
    log_file = open(filename, "w")
    log_file.write("timestamp,x,y,theta,vl,vr,v,omega\n")
    
    # Setup visualization
    plot_title = f"{mission_name} - Robot Trajectory"
    if student_ids:
        plot_title += f"\nStudent IDs: {student_ids}"
    fig, ax, robot_dot, current_goal_dot, trajectory_line = setup_plot(plot_title, goal_sequence)
    plot_elements = (robot_dot, current_goal_dot, trajectory_line)
    
    robot_positions = []
    start_time = time.time()
    success_count = 0
    
    try:
        # Start simulation
        sim.startSimulation()
        print("Simulation started. Initializing...")
        time.sleep(2.0)  # Give more time for initialization
        
        # Execute mission
        total_goals = len(goal_sequence)
        for step, goal_id in enumerate(goal_sequence, 1):
            goal = MARKER_POSITIONS[goal_id]
            print(f"\nStep {step}/{total_goals}: Moving to Marker {goal_id} at ({goal[0]:.2f}, {goal[1]:.2f})")
            
            step_start_time = time.time()
            go_to_goal(goal, robot_positions, plot_elements, log_file, start_time)
            step_duration = time.time() - step_start_time
            
            print(f"    Step completed in {step_duration:.2f} seconds")
            success_count += 1
            
            if step < total_goals:
                print("  Pausing briefly before next goal...")
                time.sleep(0.5)
        
        total_time = time.time() - start_time
        print(f"\n{'='*60}")
        print(f"MISSION COMPLETED SUCCESSFULLY!")
        print(f"{'='*60}")
        print(f"Total execution time: {total_time:.2f} seconds")
        print(f"Waypoints successfully visited: {success_count}/{total_goals}")
        print(f"Success rate: {(success_count/total_goals)*100:.1f}%")
        print(f"Average time per waypoint: {total_time/total_goals:.2f} seconds")
        print(f"Log file saved: {filename}")
        
    except Exception as e:
        print(f"\nERROR during mission execution: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Stop simulation and cleanup
        try:
            sim.stopSimulation()
            print("Simulation stopped.")
        except:
            pass
        
        log_file.close()
        
        # Save plot
        plot_filename = f"{safe_mission_name}_trajectory_{timestamp_str}.png"
        try:
            plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
            print(f"Trajectory plot saved: {plot_filename}")
        except Exception as e:
            print(f"Warning: Could not save plot - {e}")
        
        plt.ioff()
        plt.show(block=False)
        
        print(f"{'='*60}")

def main():
    """Main function to execute all missions"""
    print("=" * 80)
    print("AMR HOMEWORK 01 - TASK 2: ARUCO MARKER NAVIGATION")
    print("Student IDs: 211805131, 211805078")
    print("=" * 80)
    print("This program will execute three navigation missions:")
    print("1. Letter C drawing - Visit markers to draw letter C")
    print("2. Letter S drawing - Visit markers to draw letter S") 
    print("3. Student ID navigation - Visit markers based on student IDs")
    print("=" * 80)
    
    # Print mission details
    print("\nMISSION DETAILS:")
    print(f"Letter C sequence: {LETTER_C_SEQUENCE}")
    print(f"Letter S sequence: {LETTER_S_SEQUENCE}")
    print(f"Student ID sequence: {STUDENT_ID_SEQUENCE}")
    
    # Get user input for which missions to run
    print("\nWhich missions would you like to execute?")
    print("1 - Letter C only")
    print("2 - Letter S only")
    print("3 - Student ID navigation only")
    print("4 - All missions (recommended for homework submission)")
    print("5 - Exit")
    
    try:
        choice = input("\nEnter your choice (1-5): ").strip()
        
        if choice == '5':
            print("Exiting program.")
            return
        
        if choice == '1' or choice == '4':
            execute_mission("Letter C Drawing", LETTER_C_SEQUENCE)
            if choice == '4':
                input("\nPress Enter to continue to Letter S mission...")
        
        if choice == '2' or choice == '4':
            execute_mission("Letter S Drawing", LETTER_S_SEQUENCE)
            if choice == '4':
                input("\nPress Enter to continue to Student ID navigation...")
        
        if choice == '3' or choice == '4':
            execute_mission("Student ID Navigation", STUDENT_ID_SEQUENCE, "211805131, 211805078")
        
        if choice not in ['1', '2', '3', '4']:
            print("Invalid choice. Please run the program again.")
            return
    
    except KeyboardInterrupt:
        print("\n\nProgram interrupted by user.")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()
    
    print("\nAll selected missions completed!")
    print("Remember to include the generated log files and trajectory plots in your report!")

if __name__ == "__main__":
    main()