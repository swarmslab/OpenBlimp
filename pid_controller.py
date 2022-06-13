import time

class PID:
    """PID controller."""

    def __init__(self, Kp, Ki, Kd, origin_time=None, circular=False, limit=0.0):
        if origin_time is None:
            origin_time = time.time()

        # Gains for each term
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # Corrections (outputs)
        self.Cp = 0.0
        self.Ci = 0.0
        self.Cd = 0.0

        self.previous_time = origin_time
        self.previous_error = 0.0

        # accounts for euler angles
        self.circular = circular
        self.circular = limit

    def Update(self, error, current_time=None):
        if current_time is None:
            current_time = time.time()
        dt = current_time - self.previous_time
        if dt <= 0.0:
            return 0

        de = error - self.previous_error
        if self.circular:
            if de > self.circular:
                de -= 2*self.circular
            elif de < -self.circular:
                de += 2*self.circular

        self.Cp = error
        self.Ci += error * dt
        if self.Ci > 10:
            self.Ci > 10
        elif self.Ci <-10:
            self.Ci =-10

        self.Cd = de / dt

        self.previous_time = current_time
        self.previous_error = error

        return (
            (self.Kp * self.Cp)    # proportional term
            + (self.Ki * self.Ci)  # integral term
            + (self.Kd * self.Cd)  # derivative term
        )
