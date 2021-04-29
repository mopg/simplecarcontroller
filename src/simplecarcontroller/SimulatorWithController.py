from simplecarsim import Simulator
from scipy.interpolate import interp1d
from copy import deepcopy
from math import cos, sin

class SimulatorWithController(Simulator):

    def __init__(
        self,
        track,
        V_m_per_s = 16.0,
        Delta_t_sim_s = 1e-3,
        Delta_t_control_s = 0.05,
        t_final_time_s = None
    ):

        self.track = track

        self.t_tracking_s = track.sm / V_m_per_s
        self.t_final_time_s = t_final_time_s if t_final_time_s is not None else self.t_tracking_s[-1]

        self.required_V_m_per_s = V_m_per_s

        # interpolation objects needed for tracking
        self.x_m_interp = interp1d(self.t_tracking_s, track.xm)
        self.y_m_interp = interp1d(self.t_tracking_s, track.ym)
        self.theta_rad_interp = interp1d(self.t_tracking_s, track.th)

        self.Delta_t_sim_s = Delta_t_sim_s
        self.Delta_t_control_s = Delta_t_control_s

    def simulate(
        self,
        car,
        controller,
    ):

        curr_time_s = 0.0

        state_output = [deepcopy(car.current_state)]
        time_output_s = [curr_time_s]

        s_x = 0.0
        delta_rad = 0.0
        time_last_control_update_s = -1e9 # forcing controls update at first step
        
        while curr_time_s <= self.t_final_time_s:

            if (curr_time_s - time_last_control_update_s) > self.Delta_t_control_s:

                time_last_control_update_s = curr_time_s

                # interpolate required tracking numbers inputs
                required_x_m = self.x_m_interp(curr_time_s)
                required_y_m = self.y_m_interp(curr_time_s)
                required_theta_rad = self.theta_rad_interp(curr_time_s)

                # compute along-track error (ATE)
                ATE = (required_x_m - car.current_state.x_m) * cos(required_theta_rad) + (required_y_m - car.current_state.y_m) * sin(required_theta_rad)

                # compute cross-track error
                CTE = (required_x_m - car.current_state.x_m) * sin(required_theta_rad) - (required_y_m - car.current_state.y_m) * cos(required_theta_rad)

                # compute required control commands
                s_x, delta_rad = controller.compute_control_commands(ATE = ATE, CTE = CTE, Delta_t_s = self.Delta_t_control_s)

            # advance state
            car.advance_state(delta_rad = delta_rad, s_x = s_x, Delta_t_s = self.Delta_t_sim_s)

            # save state for output
            state_output.append(deepcopy(car.current_state))
            time_output_s.append(curr_time_s)

            curr_time_s += self.Delta_t_sim_s

        return time_output_s, state_output