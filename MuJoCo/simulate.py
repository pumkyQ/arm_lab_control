#!/usr/bin/env python3
import os
import argparse
import mujoco
import mujoco.viewer

def main():
    parser = argparse.ArgumentParser(description="MuJoCo Interactive Simulation for Robot Finger")
    parser.add_argument(
        "--model",
        type=str,
        default="finger",
        help="Model name to simulate (e.g. 'finger', 'finger2', 'hello')"
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
        # Load model and create simulation data
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        
        print("Model loaded successfully. Launching interactive viewer...")
        print("Controls in Viewer:")
        print("  - Mouse Left Drag: Rotate camera")
        print("  - Mouse Right Drag: Translate camera")
        print("  - Double Click Body: Select body for force application")
        print("  - Ctrl + Mouse Left Drag: Apply external forces")
        print("  - Use the 'Actuators' panel on the right to manually control joint positions!")
        
        def load_callback():
            # Reload and recompile the XML model from disk
            new_model = mujoco.MjModel.from_xml_path(xml_path)
            new_data = mujoco.MjData(new_model)
            return new_model, new_data

        # Launch the interactive viewer with loader callback
        mujoco.viewer.launch(loader=load_callback)
        
    except Exception as e:
        print(f"Failed to run simulation: {e}")

if __name__ == "__main__":
    main()
