import time


class PIDController:
    def __init__(self, kp, ki, kd, max, min):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_val = max
        self.min_val = min
        self.last_time = time.time_ns()
        self.prev_error = 0.0
        self.prev_integral_errors = [0.0 for i in range(10)]

    def control(self, curr_error):
        curr_time = time.time_ns()
        iteration_time = (curr_time - self.last_time) / 1000000000  # get into seconds

        prev_int_error = 0.0
        for error in self.prev_integral_errors:
            prev_int_error += error

        integral_term = prev_int_error + curr_error * iteration_time
        derivative_term = (curr_error - self.prev_error) / iteration_time

        output = self.kp * curr_error + self.ki * integral_term + self.kd * derivative_term

        self.prev_error = curr_error
        if len(self.prev_integral_errors) == 0:
            self.prev_integral_errors.pop(0)
        self.prev_integral_errors.append(curr_error)

        self.last_time = curr_time

        if output > self.max_val:
            output = self.max_val
        elif output < self.min_val:
            output = self.min_val

        return output
