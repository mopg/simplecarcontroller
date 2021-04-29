from trackgen import Track
from simplecarcontroller import Controller2D_PID, SimulatorWithController
from simplecarsim import Car, State
from math import pi

import numpy as np

import matplotlib.pyplot as plt

# circle track
track = Track(
    crns = np.array([True]),
    lpar = np.array([100.0]),
    delTh = np.array([2 * pi]),
)
track.compTrackXY()
# track.plot()

# # Random track
# crns = np.array( [False,True,False,True,True,True,False,True,True,False], dtype=bool )
# delTh = np.array( [0,pi/2,0,pi/2,pi/2,pi/2,0,pi/4,pi/4,0], dtype=float )
# lpar = np.array( [20,10,20,10,-10,10,200,-10,10,200], dtype=float )
# track = Track( length = 250., left = True, crns = crns )
# track.solve( lpar, delTh, case = 2 )
# track.compTrackXY()
# track.plot()

# define controller
# controller = Controller2D_PID(
#     Kp_ATE = 0.01, Ki_ATE = 0.0, Kd_ATE = 0.05,
#     Kp_CTE = 0.01, Ki_CTE = 0.0, Kd_CTE = 0.05,
# )
controller = Controller2D_PID(
    Kp_ATE = 0.1, Ki_ATE = 0.0, Kd_ATE = 0.5,
    Kp_CTE = 0.1, Ki_CTE = 0.0, Kd_CTE = 0.5,
)

# define simulation (with controller)
V_req_m_per_s = 5.0
sim = SimulatorWithController(track = track, V_m_per_s = V_req_m_per_s)#, t_final_time_s = 10.0)

car = Car(
    initial_state = State(y_m = 1.0)
)

time_output_s, state_output = sim.simulate(car = car, controller = controller)

plt.figure()
plt.plot(time_output_s, [V_req_m_per_s for state in state_output], 'r--', linewidth=1)
plt.plot(time_output_s, [state.get_V_m_per_s() for state in state_output], 'k', linewidth=1)
plt.savefig("vel_m_per_s_2D.png")

plt.figure()
plt.plot(track.xm, track.ym, 'r--', linewidth=1)
plt.plot([state.x_m for state in state_output], [state.y_m for state in state_output], 'k', linewidth=1)
plt.savefig("position_2D.png")

plt.figure()
plt.plot(time_output_s, [state.s_x for state in state_output], 'k', linewidth=1)
plt.savefig("s_x_command_2D.png")

plt.figure()
plt.plot(time_output_s, [state.delta_rad * 180/pi for state in state_output], 'k', linewidth=1)
plt.savefig("delta_deg_command_2D.png")