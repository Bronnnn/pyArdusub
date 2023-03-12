from src.Controller import Constants

class pid_controller:
    __class_id = 0

    def __init__(self, coefficients:dict):
        self.__ID = pid_controller.__class_id

        # set coefficients
        self._coefficients = {}
        self.set_coefficients(coefficients=coefficients)

        # init inputs
        self._target = 0
        self._position = 0
        self._error = 0
        self._error_last = 0

        # init error
        self._integral_error = 0
        self._derivative_error = 0

        # utils
        self._warning_zero_dt = False

    # utils
    def relax_integrator(self):
        self._integral_error = 0

    # change coefficients
    def get_coefficients(self) -> dict:
        return self._coefficients

    def check_coefficients_key(self, key):
        valid_keys = ["k_p", "k_i", "k_d"]
        if key not in valid_keys: raise KeyError(f"{key} invalid. available keys: {valid_keys}")

    def set_coefficients(self, coefficients:dict):
        for key in coefficients.keys():
            self.check_coefficients_key(key)
            self._coefficients[key] = coefficients[key]

    # compute output
    def update_position(self, position):
        self._position = position

    def update_target(self, target):
        self._target = target

    def update_error(self, delta_t_s):
        if delta_t_s <= 0:
            self._warning_zero_dt = True
            self._error = self._target - self._position
        else:
            self._warning_zero_dt = False
            self._error = self._target - self._position
            self._integral_error += self._error * delta_t_s
            self._derivative_error = (self._error - self._error_last) / delta_t_s
            self._error_last = self._error


    def compute(self, position, target, delta_t_s):
        self.update_position(position)
        self.update_target(target)
        self.update_error(delta_t_s)
        output = self._coefficients["k_p"] * self._error \
                 + self._coefficients["k_i"] * self._integral_error \
                 + self._coefficients["k_d"] * self._derivative_error

        return output

class depth_controller(pid_controller):
    """
        Warning: Because of some legacy workaround, z will work between [0-1000]
        where 0 is full reverse, 500 is no output and 1000 is full throttle.
        x,y and r will be between [-1000 and 1000].
        """
    def __init__(self, coefficients:dict):
        super().__init__(coefficients)
        self._min_saturation = Constants.z_min
        self._max_saturation = Constants.z_max
        self._offset = Constants.z_offset

    def saturate(self, output):
        if output >= self._max_saturation: output = self._max_saturation
        if output <= self._min_saturation: output = self._min_saturation
        output += self._offset
        return output

    def compute(self, position, target, delta_t_s):
        self.update_position(position)
        self.update_target(target)
        self.update_error(delta_t_s)

        output = self._coefficients["k_p"] * self._error \
                 + self._coefficients["k_i"] * self._integral_error \
                 + self._coefficients["k_d"] * self._derivative_error

        return self.saturate(output)







