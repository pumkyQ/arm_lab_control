import os
import time
import mujoco as mj
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt

def main():
    # 1. Path to the model
    # Get the directory of this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Resolve absolute path to dclaw3xh.xml
    xml_path = os.path.abspath(os.path.join(
        script_dir, 
        "../kiis_2024_impedance_control/dclaw/resource/robel_sim/dclaw/dclaw3xh.xml"
    ))
    
    print(f"Loading MuJoCo model from: {xml_path}")
    if not os.path.exists(xml_path):
        print("Error: Model XML file not found!")
        return

    # 2. Load model and data
    model = mj.MjModel.from_xml_path(xml_path)
    data = mj.MjData(model)
    
    # 3. Simulation Parameters
    dt = 0.001
    model.opt.timestep = dt
    
    # Control Mode: True = Cartesian Impedance Control, False = Cartesian PD Control
    use_impedance_control = True
    
    # Number of actuators (DClaw has 9 joints)
    n_actuators = 9
    
    # Find starting joint indices for DClaw in the system coordinate vectors (qpos, qvel)
    # This dynamically adapts if contact bodies like target_ball are defined first in XML
    start_qpos = model.joint("FFJ10").qposadr[0]
    start_qvel = model.joint("FFJ10").dofadr[0]
    
    # Fingertips site names and their corresponding parent body IDs
    tips = ["FFtip", "MFtip", "THtip"]
    end_effector_body_names = ["FFL12", "MFL22", "THL32"]
    end_effector_ids = [model.body(name).id for name in end_effector_body_names]
    
    # 4. Desired Goals for the 3 fingertips (x, y, z in world frame)
    # The mount is at z = 0.30, and fingers point downward.
    # The target ball is spawned at [0, 0, 0.07] with radius 0.04 (top of ball is at z = 0.11).
    # We set goals around/inside the ball to pinch or touch it.
    # Set goals slightly inside the box boundary to squeeze it
    # Squeezing the static pillar (base center at [0, 0, 0.04], box size x/y is 5cm, height is 8cm)
    # Squeezing the dynamic elevated pillar (box size x/y is 3cm, height is 6cm, center at z=0.11)
    goals = np.array([
        [0.0, -0.010, 0.11],  # FFtip goal (squeezing in from front)
        [0.010, 0.007, 0.11],  # MFtip goal (squeezing in from back-right)
        [-0.010, 0.007, 0.11]  # THtip goal (squeezing in from back-left)
    ])
    
    # 5. Controller Gains and Impedance Parameters
    # Cartesian Space PD Parameters (for simple PD control)
    kp_cartesian = 180.0
    kd_cartesian = 4.0
    
    # Cartesian Space Impedance Parameters
    # Inertia shaping (Md), Stiffness (Kd), Damping (Dd)
    desired_inertia = np.eye(3) * 0.015
    desired_stiffness = np.eye(3) * 80.0   # Squeeze stiffness (gentler)
    desired_damping = np.eye(3) * 20.0    # High damping to suppress contact chattering
    Md_inv = np.linalg.inv(desired_inertia)
    
    # Low-pass filter for force sensor feedback to prevent chattering
    lpf_alpha = 0.15
    f_ext_filtered = [np.zeros(3) for _ in range(3)]
    
    # Singularity damping term (for pseudo-inverse)
    singularity_damping = 0.03
    
    # 6. Initialize tracking variables for Jacobian derivative computation
    jacp_prev = [np.zeros((3, model.nv)) for _ in range(3)]
    jacr_prev = [np.zeros((3, model.nv)) for _ in range(3)]
    
    # Logging arrays for plotting
    forces_log = []
    errors_log = []
    time_log = []
    
    print("\n--- Starting Standalone MuJoCo Simulation ---")
    print("Default Mode: Cartesian Space Impedance Control")
    print("Close the viewer window to view performance plots.")
    print("Press Ctrl+C in the terminal to exit.\n")
    
    # Control settings for manual interaction (Default to TRUE for live interactive demonstration)
    control_settings = {
        "use_impedance_control": True,
        "interactive_mode": True,
        "z_target": 0.11
    }
    
    def key_callback(keycode):
        try:
            key_char = chr(keycode).lower()
            if key_char == 'm':
                control_settings["use_impedance_control"] = not control_settings["use_impedance_control"]
                mode_str = "Impedance Control" if control_settings["use_impedance_control"] else "Cartesian PD Control"
                print(f"\n>>> [KEYPRESS] 'M' pressed. Active Control Mode: {mode_str} <<<")
            elif key_char == 'u':
                control_settings["z_target"] = 0.17
                print(f"\n>>> [KEYPRESS] 'U' pressed. Command: Lift Up Target (z -> 0.17m) <<<")
            elif key_char == 'd':
                control_settings["z_target"] = 0.11
                print(f"\n>>> [KEYPRESS] 'D' pressed. Command: Lower Down Target (z -> 0.11m) <<<")
        except ValueError:
            pass

    # Launch the passive viewer with key callback
    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        # Initial visualization settings
        with viewer.lock():
            viewer.opt.flags[mj.mjtVisFlag.mjVIS_CONTACTFORCE] = 0
            # Enable geom group 3 (where the target ball/box is defined)
            viewer.opt.geomgroup[3] = 1
            
            # Draw visual markers for fingertip goals
            for goal in goals:
                viewer.user_scn.ngeom += 1
                mj.mjv_initGeom(
                    viewer.user_scn.geoms[viewer.user_scn.ngeom - 1],
                    mj.mjtGeom.mjGEOM_SPHERE,
                    np.zeros(3), np.zeros(3), np.zeros(9),
                    np.array([0.1, 0.9, 0.1, 0.5]) # Translucent green sphere
                )
                # Position the marker
                viewer.user_scn.geoms[viewer.user_scn.ngeom - 1].pos = goal
                viewer.user_scn.geoms[viewer.user_scn.ngeom - 1].size = [0.008, 0, 0]
        
        step_count = 0
        
        # Log array for control modes
        mode_log = []
        
        print("\n=== INTERACTIVE GRASPING SIMULATION STARTED ===")
        print("Press 'M' key to manually toggle control mode (Impedance <-> Cartesian PD)")
        print("==============================================\n")

        while viewer.is_running():
            step_start = time.perf_counter()
            t = data.time
            
            # Interactive manual mode handles variables dynamically modified by keyboard callbacks
            use_impedance_control = control_settings["use_impedance_control"]
            z_target = control_settings["z_target"]
                
            # Apply dynamic Z coordinate to target goals
            current_goals = goals.copy()
            current_goals[:, 2] = z_target
            
            # Update visual markers for fingertip goals in the viewer
            with viewer.lock():
                for idx, goal in enumerate(current_goals):
                    viewer.user_scn.geoms[idx].pos = goal
            
            # Console output every 1.0 second
            if step_count % 1000 == 0:
                mode_str = "Impedance" if use_impedance_control else "Cartesian PD"
                lift_str = "Lifting" if z_target > 0.12 else "Grasping"
                print(f"Time: {t:5.1f}s | Mode: {mode_str:<12} | Z-Target: {z_target:.2f}m ({lift_str})")
                
            # Initialize joint torque vector for the robot (9 actuators)
            tau_ctrl = np.zeros(n_actuators)
            
            # Get full system Mass matrix (size nv x nv)
            M_full = np.zeros((model.nv, model.nv))
            mj.mj_fullM(model, data, M_full)
            
            # Get full bias force vector (Coriolis, Centrifugal, Gravity)
            c_full = data.qfrc_bias
            
            # Slice mass matrix and bias forces to the 9 robot joints
            M_robot = M_full[start_qvel : start_qvel + n_actuators, start_qvel : start_qvel + n_actuators]
            c_robot = c_full[start_qvel : start_qvel + n_actuators]
            
            # Track errors and forces for logging
            ee_errors_step = []
            ee_forces_step = []
            
            # Compute control torques for each fingertip
            for idx, tip in enumerate(tips):
                # 1. Position error and velocity in world frame
                tip_pos = data.site(tip).xpos
                x_error = current_goals[idx] - tip_pos
                ee_errors_step.extend(x_error)
                
                # Get local-to-world rotation matrix of the site
                rotation_matrix = data.site(tip).xmat.reshape(3, 3)
                
                # 2. Compute Jacobian (size 3 x nv) for the fingertip
                jacp = np.zeros((3, model.nv))
                jacr = np.zeros((3, model.nv))
                mj.mj_jac(model, data, jacp, jacr, tip_pos, end_effector_ids[idx])
                
                # Slice Jacobian to the 9 robot joints
                J = jacp[:, start_qvel : start_qvel + n_actuators]
                
                # Compute end-effector linear velocity: xvel = J * qvel
                qvel_robot = data.qvel[start_qvel : start_qvel + n_actuators]
                xvel = J @ qvel_robot
                
                # 3. Compute Jacobian derivative numerically: J_dot = (J_curr - J_prev) / dt
                J_dot = (J - jacp_prev[idx][:, start_qvel : start_qvel + n_actuators]) / dt
                jacp_prev[idx] = jacp.copy()
                
                # 4. Compute virtual contact force feedback from fingertip force sensors
                # In MuJoCo, the F/T force sensor outputs Fx, Fy, Fz in the site's local frame.
                # Sensor index can be found by name.
                sensor_name = f"{tip[:2]}_force"
                raw_sensor_data = data.sensor(sensor_name).data.copy()
                # Rotate local force vector to global frame
                f_ext_raw = rotation_matrix @ raw_sensor_data
                
                # Apply 1st-order Low-pass Filter to suppress contact chattering in simulation
                f_ext = lpf_alpha * f_ext_raw + (1.0 - lpf_alpha) * f_ext_filtered[idx]
                f_ext_filtered[idx] = f_ext.copy()
                
                ee_forces_step.extend(f_ext)
                
                # 5. Compute Damped Pseudo-Inverse of J for redundancy resolution
                # J# = (J^T * J + lambda^2 * I)^(-1) * J^T
                product = J.T @ J + singularity_damping * np.identity(n_actuators)
                J_pinv = np.linalg.inv(product) @ J.T
                
                # 6. Apply Control Law
                if use_impedance_control:
                    # Cartesian Impedance Control formulation:
                    # x_ddot_cmd = Md_inv * (Kd * e_x - Dd * v_x - f_ext)
                    # tau = M * J# * (x_ddot_cmd - J_dot * q_dot) + c
                    x_ddot_cmd = Md_inv @ (desired_stiffness @ x_error - desired_damping @ xvel - f_ext)
                    tau_finger = M_robot @ J_pinv @ (x_ddot_cmd - J_dot @ qvel_robot)
                else:
                    # Simple Cartesian PD Control formulation:
                    # tau = J^T * (Kp * e_x - Kd * v_x)
                    f_cmd = kp_cartesian * x_error - kd_cartesian * xvel
                    tau_finger = J.T @ f_cmd
                
                # Sum the joint torque contributions (since fingers move independently)
                tau_ctrl += tau_finger
            
            # Add gravity & dynamic bias compensation
            tau_ctrl += c_robot
            
            # Apply control torques to DClaw actuators
            data.ctrl[:n_actuators] = tau_ctrl
            
            # Step physics simulation
            mj.mj_step(model, data)
            
            # Log data every 10 steps (10ms)
            if step_count % 10 == 0:
                forces_log.append(ee_forces_step)
                errors_log.append(ee_errors_step)
                time_log.append(data.time)
                mode_log.append(use_impedance_control)
            
            step_count += 1
            
            # Sync viewer visualization
            viewer.sync()
            
            # Simple rate limiter to match real-time
            time_until_next_step = dt - (time.perf_counter() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
                
    # 7. Post-Simulation Plotting
    print("\nSimulation ended. Plotting results...")
    forces_log = np.array(forces_log)
    errors_log = np.array(errors_log)
    time_log = np.array(time_log)
    mode_log = np.array(mode_log)
    
    # Calculate force and error magnitudes for each fingertip
    forces_ff = np.linalg.norm(forces_log[:, 0:3], axis=1)
    forces_mf = np.linalg.norm(forces_log[:, 3:6], axis=1)
    forces_th = np.linalg.norm(forces_log[:, 6:9], axis=1)
    
    errors_ff = np.linalg.norm(errors_log[:, 0:3], axis=1)
    errors_mf = np.linalg.norm(errors_log[:, 3:6], axis=1)
    errors_th = np.linalg.norm(errors_log[:, 6:9], axis=1)
    
    # Apply clean style parameters
    plt.rcParams['font.family'] = 'sans-serif'
    plt.rcParams['text.color'] = '#2c3e50'
    
    fig, axes = plt.subplots(2, 1, figsize=(13, 9.5), sharex=True)
    
    # Modern HSL-tailored colors
    color_ff = '#E74C3C'  # Sunset Red
    color_mf = '#2ECC71'  # Emerald Green
    color_th = '#3498DB'  # Sky Blue
    
    # Plot 1: Fingertip Force Magnitudes
    axes[0].plot(time_log, forces_ff, color=color_ff, linewidth=2.0, label='Index Finger (FF)')
    axes[0].plot(time_log, forces_mf, color=color_mf, linewidth=2.0, label='Middle Finger (MF)')
    axes[0].plot(time_log, forces_th, color=color_th, linewidth=2.0, label='Thumb (TH)')
    axes[0].set_ylabel("Grasping Force Magnitude (N)", fontsize=11, fontweight='bold', labelpad=10)
    axes[0].set_title("Fingertip Grasping Force Feedback (Magnitude)", fontsize=14, fontweight='bold', pad=15)
    axes[0].legend(loc='upper right', frameon=True, framealpha=0.9, facecolor='#ffffff', edgecolor='#dcdde1')
    
    # Plot 2: Position Error Magnitudes
    axes[1].plot(time_log, errors_ff, color=color_ff, linewidth=2.0, label='Index Finger (FF)')
    axes[1].plot(time_log, errors_mf, color=color_mf, linewidth=2.0, label='Middle Finger (MF)')
    axes[1].plot(time_log, errors_th, color=color_th, linewidth=2.0, label='Thumb (TH)')
    axes[1].set_xlabel("Time (seconds)", fontsize=11, fontweight='bold', labelpad=10)
    axes[1].set_ylabel("Position Squeeze Error (m)", fontsize=11, fontweight='bold', labelpad=10)
    axes[1].set_title("Fingertip Position Tracking Errors during Grasping", fontsize=14, fontweight='bold', pad=15)
    axes[1].legend(loc='upper right', frameon=True, framealpha=0.9, facecolor='#ffffff', edgecolor='#dcdde1')
    
    # Clean up Spines and Customize Grid for both
    for ax in axes:
        # Hide the right and top spines for minimalist layout
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['left'].set_color('#7f8c8d')
        ax.spines['bottom'].set_color('#7f8c8d')
        
        # Subtle horizontal grids
        ax.grid(True, which='both', linestyle='-', linewidth=0.5, color='#f1f2f6')
        
    # Plot backgrounds dynamically based on the logged controller mode
    n_points = len(time_log)
    if n_points > 0:
        start_idx = 0
        current_mode = mode_log[0]
        impedance_midpoints = []
        pd_midpoints = []
        
        for i in range(1, n_points):
            if mode_log[i] != current_mode or i == n_points - 1:
                t_start = time_log[start_idx]
                t_end = time_log[i]
                color = '#EDF4FC' if current_mode else '#FDF5EC'
                
                # Apply background span to both plots
                for ax in axes:
                    ax.axvspan(t_start, t_end, color=color, alpha=0.9)
                
                # Keep track of segment midpoints to place descriptive text cards
                midpoint = (t_start + t_end) / 2.0
                if current_mode:
                    impedance_midpoints.append(midpoint)
                else:
                    pd_midpoints.append(midpoint)
                
                # Draw mode transition boundary line
                if mode_log[i] != current_mode:
                    for ax in axes:
                        ax.axvline(x=t_end, color='#7f8c8d', linestyle='--', linewidth=1.2, alpha=0.6)
                        
                start_idx = i
                current_mode = mode_log[i]
        
        # Place the floating info cards at the center of the largest segments
        ylim_force = axes[0].get_ylim()[1]
        bbox_impedance = dict(boxstyle='round,pad=0.5', facecolor='#ffffff', edgecolor='#3498DB', linewidth=1.5, alpha=0.95)
        bbox_pd = dict(boxstyle='round,pad=0.5', facecolor='#ffffff', edgecolor='#E74C3C', linewidth=1.5, alpha=0.95)
        
        if len(impedance_midpoints) > 0:
            axes[0].text(impedance_midpoints[0], ylim_force * 0.70, 
                         "Impedance Grasping & Lifting\n(Compliant & Dynamic Force Adjustment)", 
                         ha='center', va='center', fontsize=10.5, fontweight='bold', color='#2980B9', bbox=bbox_impedance)
        if len(pd_midpoints) > 0:
            axes[0].text(pd_midpoints[0], ylim_force * 0.70, 
                         "Cartesian PD Grasping & Lifting\n(Stiff & Dangerous Grasping Force)", 
                         ha='center', va='center', fontsize=10.5, fontweight='bold', color='#C0392B', bbox=bbox_pd)
                 
    # Adjust spacing and save
    plot_save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "grasping_force_comparison.png")
    plt.tight_layout()
    plt.savefig(plot_save_path, dpi=300)
    print(f"Comparison plot saved successfully to: {plot_save_path}")
    
    # plt.show()

if __name__ == "__main__":
    main()
