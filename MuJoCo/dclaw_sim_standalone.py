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
    goals = np.array([
        [0.0, -0.015, 0.05],  # FFtip goal (squeezing in from front)
        [0.015, 0.010, 0.05],  # MFtip goal (squeezing in from back-right)
        [-0.015, 0.010, 0.05]  # THtip goal (squeezing in from back-left)
    ])
    
    # 5. Controller Gains and Impedance Parameters
    # Cartesian Space PD Parameters (for simple PD control)
    kp_cartesian = 180.0
    kd_cartesian = 4.0
    
    # Cartesian Space Impedance Parameters
    # Inertia shaping (Md), Stiffness (Kd), Damping (Dd)
    desired_inertia = np.eye(3) * 0.015
    desired_stiffness = np.eye(3) * 150.0
    desired_damping = np.eye(3) * 6.0
    Md_inv = np.linalg.inv(desired_inertia)
    
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
    
    # Control settings for manual interaction
    control_settings = {
        "use_impedance_control": True,
        "interactive_mode": False
    }
    
    def key_callback(keycode):
        try:
            key_char = chr(keycode).lower()
            if key_char == 'm':
                control_settings["use_impedance_control"] = not control_settings["use_impedance_control"]
                control_settings["interactive_mode"] = True
                mode_str = "Impedance Control" if control_settings["use_impedance_control"] else "Cartesian PD Control"
                print(f"\n>>> [MANUAL MODE] Control mode toggled to: {mode_str} <<<\n")
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
            
            # Scenario management (automated unless user interacts manually)
            if not control_settings["interactive_mode"]:
                if t >= 16.0:
                    print("\nAutomated presentation scenario completed. Exiting simulation...")
                    break
                    
                # 0~8s is Impedance Grasping, 8~16s is Cartesian PD Grasping
                if t < 8.0:
                    use_impedance_control = True
                else:
                    use_impedance_control = False
            else:
                # Manual mode (read settings modified by keyboard callback)
                use_impedance_control = control_settings["use_impedance_control"]
            
            # Console output every 1.0 second
            if step_count % 1000 == 0:
                mode_str = "Impedance Control" if use_impedance_control else "Cartesian PD Control"
                print(f"Time: {t:4.1f}s | Squeezing Mode: {mode_str}")
                
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
                x_error = goals[idx] - tip_pos
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
                f_ext = rotation_matrix @ raw_sensor_data
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
    
    fig, axes = plt.subplots(2, 1, figsize=(12, 9), sharex=True)
    
    # Plot 1: Fingertip Force Magnitudes
    axes[0].plot(time_log, forces_ff, 'r-', linewidth=1.5, label='Index Finger (FF)')
    axes[0].plot(time_log, forces_mf, 'g-', linewidth=1.5, label='Middle Finger (MF)')
    axes[0].plot(time_log, forces_th, 'b-', linewidth=1.5, label='Thumb (TH)')
    axes[0].set_ylabel("Grasping Force Magnitude (N)", fontsize=11)
    axes[0].set_title("Fingertip Grasping Force Feedback (Magnitude)", fontsize=13, fontweight='bold', pad=15)
    axes[0].legend(loc='upper right', framealpha=0.9)
    axes[0].grid(True, linestyle=':', alpha=0.6)
    
    # Plot 2: Position Error Magnitudes
    axes[1].plot(time_log, errors_ff, 'r-', linewidth=1.5, label='Index Finger (FF)')
    axes[1].plot(time_log, errors_mf, 'g-', linewidth=1.5, label='Middle Finger (MF)')
    axes[1].plot(time_log, errors_th, 'b-', linewidth=1.5, label='Thumb (TH)')
    axes[1].set_xlabel("Time (seconds)", fontsize=11)
    axes[1].set_ylabel("Position Squeeze Error (m)", fontsize=11)
    axes[1].set_title("Fingertip Position Tracking Errors during Grasping", fontsize=13, fontweight='bold', pad=15)
    axes[1].legend(loc='upper right', framealpha=0.9)
    axes[1].grid(True, linestyle=':', alpha=0.6)
    
    # Highlight Control Phases on both subplots
    for ax in axes:
        # Phase backgrounds
        # Impedance Control Phase (0s ~ 8s)
        ax.axvspan(0, 8, color='#e6f2ff', alpha=0.6, label='Impedance Control')
        # Cartesian PD Control Phase (8s ~ end)
        ax.axvspan(8, max(16.0, time_log[-1]), color='#fff0e6', alpha=0.6, label='Cartesian PD Control')
        
        # Vertical boundary line between controllers
        ax.axvline(x=8.0, color='gray', linestyle='--', alpha=0.8)
            
    # Add text labels on the top subplot to guide the presentation
    ylim_force = axes[0].get_ylim()[1]
    axes[0].text(4.0, ylim_force * 0.75, "Impedance Grasping\n(Compliant & Low Grasping Force)", ha='center', va='center', fontsize=11, fontweight='bold', color='#004080')
    axes[0].text(12.0, ylim_force * 0.75, "Cartesian PD Grasping\n(Stiff & Excessive Grasping Force)", ha='center', va='center', fontsize=11, fontweight='bold', color='#804000')
    
    # Save the plot image so the user can easily copy it into their PPT
    plot_save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "grasping_force_comparison.png")
    plt.tight_layout()
    plt.savefig(plot_save_path, dpi=300)
    print(f"Comparison plot saved successfully to: {plot_save_path}")
    
    plt.show()

if __name__ == "__main__":
    main()
