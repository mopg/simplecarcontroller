from .Controller import Controller

class Controller2D(Controller):

    pass

class Controller2D_PID(Controller2D):

    def __init__(
        self,
        Kp_ATE = 0.0,
        Ki_ATE = 0.0,
        Kd_ATE = 0.0,
        Kp_CTE = 0.0,
        Ki_CTE = 0.0,
        Kd_CTE = 0.0,
    ):

        self.Kp_ATE = Kp_ATE
        self.Ki_ATE = Ki_ATE
        self.Kd_ATE = Kd_ATE

        self.Kp_CTE = Kp_CTE
        self.Ki_CTE = Ki_CTE
        self.Kd_CTE = Kd_CTE

        self.integral_error_ATE = 0.0
        self.previous_error_ATE = 0.0

        self.integral_error_CTE = 0.0
        self.previous_error_CTE = 0.0

    def compute_control_commands(
        self,
        ATE,
        CTE,
        Delta_t_s,
    ):

        # along track (to figure out longitudinal command)
        derivative_error_ATE = (ATE - self.previous_error_ATE) / Delta_t_s
        s_x = self.Kp_ATE * ATE + self.Ki_ATE * self.integral_error_ATE + self.Kd_ATE * derivative_error_ATE

        self.integral_error_ATE += ATE * Delta_t_s
        self.previous_error_ATE = ATE

        # along track (to figure out steering angle command)
        derivative_error_CTE = (CTE - self.previous_error_CTE) / Delta_t_s
        delta_rad = self.Kp_CTE * CTE + self.Ki_CTE * self.integral_error_CTE + self.Kd_CTE * derivative_error_CTE

        self.integral_error_CTE += CTE * Delta_t_s
        self.previous_error_CTE = CTE

        print(ATE, CTE, s_x, delta_rad)
        
        return s_x, delta_rad