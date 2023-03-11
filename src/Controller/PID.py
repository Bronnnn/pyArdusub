class PID:
    __class_id = 0
    def __init__(self, k_p, k_i, k_d, error):
        self.__ID = PID.__class_id
        self.__k_p = k_p
        self.__k_i = k_i
        self.__k_d = k_d


