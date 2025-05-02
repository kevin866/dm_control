import mujoco
import glfw
import numpy as np

# Initialize GLFW
glfw.init()
window = glfw.create_window(1200, 900, "MuJoCo Viewer", None, None)
glfw.make_context_current(window)

# Load model and data
model = mujoco.MjModel.from_xml_path("dm_control/suite/acrobot_wind.xml")
data = mujoco.MjData(model)

# Create context and scene
scene = mujoco.MjvScene(model, maxgeom=10000)
cam = mujoco.MjvCamera()
opt = mujoco.MjvOption()
context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)

# Set default camera
mujoco.mjv_defaultCamera(cam)
cam.lookat[:] = [0,0,0]  # Focus on known object
cam.distance = 10.0
cam.azimuth = 90
cam.elevation = -30

# Main render loop
while not glfw.window_should_close(window):
    mujoco.mj_step(model, data)

    mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
    viewport = mujoco.MjrRect(0, 0, *glfw.get_framebuffer_size(window))
    mujoco.mjr_render(viewport, scene, context)

    glfw.swap_buffers(window)
    glfw.poll_events()

# Cleanup
glfw.terminate()
