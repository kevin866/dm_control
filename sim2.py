import mujoco
import glfw
import numpy as np

# Path to your XML file
xml_path = "dm_control/suite/acrobot_wind.xml"

# Load model and data
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# Initialize xfrc_applied with zeros, shape should be (model.nbody, 6)
data.xfrc_applied = np.zeros((model.nbody, 6), dtype=np.float64)

# Define force and torque as 3D vectors (reshape to (3, 1))
force = np.array([0.0, 0.0, 5.0]).reshape(3, 1)
torque = np.array([0.0, 0.0, 0.0]).reshape(3, 1)
point = np.array([0.0, 0.0, 0.0]).reshape(3, 1)  # application point in local coordinates

# Get body ID
body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "upper_arm")
# Apply external force/torque
print(type(force[0]))
mujoco.mj_applyFT(model, data, force, torque, point, body_id, data.xfrc_applied)

# Viewer setup and rendering loop
with mujoco.viewer.launch(model, data) as viewer:
    print("Press ESC to exit.")
    while viewer.is_running():
        step_start = data.time
        mujoco.mj_step(model, data)
        viewer.sync()
