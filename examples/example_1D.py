from trackgen import Track
from simplecarcontroller import Controller1D_PID, SimulatorWithController
from simplecarsim import Car

import numpy as np

import matplotlib.pyplot as plt

# straight track
track = Track(
    crns = np.array([False]),
    lpar = np.array([500.0]),
    delTh = np.array([0.0]),
)
track.compTrackXY()

# define controller
controller = Controller1D_PID(Kp = 0.01, Ki = 0.0, Kd = 0.1)

# define simulation (with controller)
V_req_m_per_s = 25.0
sim = SimulatorWithController(track = track, V_m_per_s = V_req_m_per_s)

car = Car()

time_output_s, state_output = sim.simulate(car = car, controller = controller)

plt.figure()
plt.plot(time_output_s, [V_req_m_per_s for state in state_output], 'r--', linewidth=1)
plt.plot(time_output_s, [state.vx_m_per_s for state in state_output], 'k', linewidth=1)
plt.savefig("vx_m_per_s.png")

plt.figure()
plt.plot(sim.t_tracking_s, track.xm, 'r--', linewidth=1)
plt.plot(time_output_s, [state.x_m for state in state_output], 'k', linewidth=1)
plt.savefig("x_m.png")