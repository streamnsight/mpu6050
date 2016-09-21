import math


class Complementary:
    def __init__(self, ratio=0.98):
        self.ratio = ratio
        self.inv_ratio = 1.0 - ratio
        self._pitch = 0.0 # angle around X axis
        self._roll = 0.0 # angle around Y axis
        return

    def run(self, dt, accel_data, gyro_data):
        self._pitch += gyro_data[0] * dt
        self._roll += gyro_data[1] * dt

        # compensate drift with accelerometer data if it is no junk
        # mag = abs(accel_data[0]) + abs(accel_data[1]) + abs(accel_data[2])
        mag = math.sqrt(accel_data[0] * accel_data[0] + accel_data[1] * accel_data[1]  + accel_data[2] * accel_data[2])
        if 0.5 < mag < 2.0:
            # Turning around the X axis results in a vector on the Y-axis
            pitch_accel = math.atan2(accel_data[1], accel_data[2]) * 180 / math.pi
            # Turning around the Y axis results in a vector on the X-axis
            roll_accel = math.atan2(accel_data[0], accel_data[2]) * 180 / math.pi

            self._pitch = self._pitch * self.ratio + pitch_accel * self.inv_ratio
            self._roll = self._roll * self.ratio + pitch_accel * self.inv_ratio

        return self._pitch, self._roll

