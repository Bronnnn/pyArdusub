"""
        Warning: Because of some legacy workaround, z will work between [0-1000]
        where 0 is full reverse, 500 is no output and 1000 is full throttle.
        x,y and r will be between [-1000 and 1000].
        """
z_max = 500
z_min = -500
z_offset = 500

x_max = y_max = r_max = 1000
x_min = y_min = r_min = -1000
x_offset = y_offset = r_offset = 0