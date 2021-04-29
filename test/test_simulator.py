import unittest

from simplecarsim import Car, State, Simulator
from math import pi, cos, sin
import numpy as np

class TestSimulator(unittest.TestCase):

    def test_simulator_constant_vx_straight_line(self):

        t_fin_s = 12.0

        vx_m_per_s = 3.0
        initial_state = State(vx_m_per_s = vx_m_per_s)
        car = Car(initial_state = initial_state)
        sim = Simulator(t_final_time_s = t_fin_s)
        
        t_output_s, state_output = sim.simulate(car = car)

        self.assertAlmostEqual(t_output_s[-1], t_fin_s)
        self.assertAlmostEqual(state_output[-1].x_m, vx_m_per_s * t_fin_s, places=2)
        self.assertAlmostEqual(state_output[-1].y_m, 0.0)
        self.assertAlmostEqual(state_output[-1].psi_rad, 0.5 * pi)
        self.assertAlmostEqual(state_output[-1].vx_m_per_s, vx_m_per_s)
        self.assertAlmostEqual(state_output[-1].vy_m_per_s, 0.0)
        self.assertAlmostEqual(state_output[-1].psi_dot_rad_per_s, 0.0)

    def test_simulator_only_linear_acceleration(self):

        t_fin_s = 12.0

        vx_m_per_s = 3.0
        initial_state = State(vx_m_per_s = vx_m_per_s)
        car = Car(initial_state = initial_state)

        s_x = [0., 0.2]
        delta_rad = [0., 0.]
        input_time_s = [0., t_fin_s]
        sim = Simulator(t_final_time_s = t_fin_s, s_x = s_x, delta_rad = delta_rad, input_time_s = input_time_s, Delta_t_sim_s = 0.005)
        
        t_output_s, state_output = sim.simulate(car = car)

        self.assertGreater(state_output[-1].x_m, 0.0)
        self.assertAlmostEqual(state_output[-1].y_m, 0.0)
        self.assertAlmostEqual(state_output[-1].psi_rad, 0.5 * pi)
        self.assertGreater(state_output[-1].vx_m_per_s, vx_m_per_s)
        self.assertAlmostEqual(state_output[-1].vy_m_per_s, 0.0)
        self.assertAlmostEqual(state_output[-1].psi_dot_rad_per_s, 0.0)

    def test_simulator_circles_symmetry(self):

        t_fin_s = 50.0

        vx_m_per_s = 25.0
        initial_state = State(vx_m_per_s = vx_m_per_s)
        car = Car(initial_state = initial_state)

        s_x = np.zeros(2)
        delta_rad = 3.0 * np.ones(2) * pi / 180.0
        input_time_s = [0., t_fin_s]
        sim_CCW = Simulator(t_final_time_s = t_fin_s, s_x = s_x, delta_rad = delta_rad, input_time_s = input_time_s, Delta_t_sim_s = 0.005)
        
        t_output_s_CCW, state_output_CCW = sim_CCW.simulate(car = car)

        self.assertAlmostEqual(t_output_s_CCW[-1], t_fin_s, places=1)

        delta_rad = -3.0 * np.ones(2) * pi / 180.0
        sim_CW = Simulator(t_final_time_s = t_fin_s, s_x = s_x, delta_rad = delta_rad, input_time_s = input_time_s, Delta_t_sim_s = 0.005)
        car = Car(initial_state = initial_state) # reset state

        t_output_s_CW, state_output_CW = sim_CW.simulate(car = car)

        self.assertAlmostEqual(t_output_s_CW[-1], t_fin_s, places=1)

        self.assertAlmostEqual(state_output_CW[-1].x_m, state_output_CCW[-1].x_m)
        self.assertAlmostEqual(state_output_CW[-1].y_m, -state_output_CCW[-1].y_m)
        self.assertAlmostEqual(state_output_CW[-1].psi_rad, -state_output_CCW[-1].psi_rad + pi)
        self.assertAlmostEqual(state_output_CW[-1].vx_m_per_s, state_output_CCW[-1].vx_m_per_s)
        self.assertAlmostEqual(state_output_CW[-1].vy_m_per_s, -state_output_CCW[-1].vy_m_per_s)
        self.assertAlmostEqual(state_output_CW[-1].psi_dot_rad_per_s, -state_output_CCW[-1].psi_dot_rad_per_s)

    def test_simulator_convergence(self):

        t_fin_s = 25.0

        vx_m_per_s = 25.0
        initial_state = State(vx_m_per_s = vx_m_per_s)
        car = Car(initial_state = initial_state)

        s_x = [0., 0.2]
        delta_rad = 3.0 * np.ones(2) * pi / 180.0
        input_time_s = [0., t_fin_s]
        
        Delta_t_sim_1_s = 0.005
        sim_1 = Simulator(t_final_time_s = t_fin_s, s_x = s_x, delta_rad = delta_rad, input_time_s = input_time_s, Delta_t_sim_s = Delta_t_sim_1_s)
        t_output_s_1, state_output_1 = sim_1.simulate(car = car)

        Delta_t_sim_2_s = 0.001
        car = Car(initial_state = initial_state) # make sure state is reset
        sim_2 = Simulator(t_final_time_s = t_fin_s, s_x = s_x, delta_rad = delta_rad, input_time_s = input_time_s, Delta_t_sim_s = Delta_t_sim_2_s)
        t_output_s_2, state_output_2 = sim_2.simulate(car = car)

        self.assertLess(np.abs(state_output_1[-1].x_m - state_output_2[-1].x_m), 0.4)
