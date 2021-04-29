from .Controller import Controller

class Controller1D(Controller):

    pass

class Controller1D_PID(Controller1D):

    def __init__(self, Kp = 0.0, Ki = 0.0, Kd = 0.0):

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.integral_error = 0.0
        self.previous_error = 0.0

    def compute_control_commands(
        self,
        ATE,
        CTE,
        Delta_t_s,
    ):

        # This controller is 1D so ignores CTE

        delta_rad = 0.0

        derivative_error = (ATE - self.previous_error) / Delta_t_s
        s_x = self.Kp * ATE + self.Ki * self.integral_error + self.Kd * derivative_error

        self.integral_error += ATE * Delta_t_s
        self.previous_error = ATE
        
        return s_x, delta_rad