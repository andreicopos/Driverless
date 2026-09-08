import math
import matplotlib.pyplot as plt

from vehicle_model.state import VehicleState
from vehicle_model.command import VehicleCommand
from vehicle_model.parameters import VehicleParameters
from vehicle_model.kinematic_bicycle import KinematicBicycle
from vehicle_model.simulator import Simulator


# TEMPORARY VALUES — NOT ARTTU MEASUREMENTS
params = VehicleParameters(
    wheelbase=1.5,
    max_steering_angle=math.radians(30.0),
)

model = KinematicBicycle(params)

sim = Simulator(
    model=model,
    state=VehicleState(
        x=0.0,
        y=0.0,
        yaw=0.0,
        velocity=10.0,
    ),
    dt=0.001,
)

xs = []
ys = []

duration = 10.0
steps = int(duration / sim.dt)

for _ in range(steps):

    command = VehicleCommand(
        steering_angle=math.radians(10.0),
        longitudinal_command=0.0,
    )

    state = sim.step(command)

    xs.append(state.x)
    ys.append(state.y)


plt.figure()
plt.plot(xs, ys)
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.axis("equal")
plt.grid(True)
plt.title("Kinematic Bicycle Model")
plt.show()