#!/usr/bin/env python3
import os
import time
import argparse
import numpy as np
import mujoco
import mujoco.viewer

def main():
    parser = argparse.ArgumentParser(description="MuJoCo Programmatic Trajectory Control for Robot Finger")
    parser.add_argument(
        "--model",
        type=str,
        default="finger",
        help="Model name to simulate (e.g. 'finger', 'finger2', 'manipulator_test')"
    )
    args = parser.parse_args()

    # Get absolute path to the XML model file
    current_dir = os.path.dirname(os.path.abspath(__file__))
    model_name = args.model
    if model_name.endswith(".xml"):
        xml_filename = model_name
    else:
        xml_filename = f"{model_name}.xml"
    xml_path = os.path.join(current_dir, xml_filename)

    if not os.path.exists(xml_path):
        print(f"Error: Model file not found at '{xml_path}'")
        return

    print(f"Loading MuJoCo model from: {xml_path}")
    
    try:
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        
        print("Launching passive viewer for programmatic control...")
        
        # Retrieve the number of actuators
        nu = model.nu
        print(f"Number of actuators/joints to control: {nu}")
        
        # Start passive viewer
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Set camera configuration
            viewer.cam.distance = 0.8
            viewer.cam.elevation = -20
            viewer.cam.lookat = [0.0, 0.0, 0.1]
            
            print("Simulating... Close the window to exit.")
            
            while viewer.is_running():
                step_start = time.time()
                
                # Current simulation time
                sim_time = data.time
                
                # Apply control commands (sine-wave trajectories for each joint)
                for i in range(nu):
                    # Different joints get different phases and amplitudes
                    amplitude = 0.5 if args.model == "finger" else 0.8
                    frequency = 1.0 + 0.5 * i
                    phase_offset = i * (np.pi / 3)
                    
                    target_pos = amplitude * np.sin(2 * np.pi * frequency * sim_time + phase_offset)
                    
                    # Clip to joint range limits if defined
                    ctrl_limit = model.actuator_ctrlrange[i]
                    target_pos = np.clip(target_pos, ctrl_limit[0], ctrl_limit[1])
                    
                    data.ctrl[i] = target_pos
                
                # Step the simulation physics
                mujoco.mj_step(model, data)
                
                # Sync viewer graphics
                viewer.sync()
                
                # Real-time synchronization
                elapsed = time.time() - step_start
                if elapsed < model.opt.timestep:
                    time.sleep(model.opt.timestep - elapsed)
                    
    except Exception as e:
        print(f"Failed to run trajectory simulation: {e}")

if __name__ == "__main__":
    main()
