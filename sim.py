from dm_control import mjcf
from dm_control import suite
from dm_control.rl import control
import numpy as np

# Load modified XML
model = mjcf.from_path('acrobot_with_wind.xml')

# Build environment
physics = mjcf.Physics.from_mjcf_model(model)
env = control.Environment(
    physics=physics,
    task=suite.acrobot.swingup.SwingUp()
)

# Control index (check ordering)
print("Actuators:", physics.model.actuator_names)

# Run simulation
for _ in range(1000):
    # Apply wind disturbance to the second actuator (index 1)
    ctrl = np.array([0.0, 5.0 * np.sin(0.5 * env.physics.time)])
    timestep = env.step(ctrl)
    print("Reward:", timestep.reward)
    env.physics.render()
