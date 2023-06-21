import time
import odrive
from odrive.utils import start_liveplotter
from odrive.enums import *
from fibre.libfibre import ObjectLostError
def wait_until(cond, sec_limit: float = 10.0):
    tick = 0
    while not (cond()) and tick < sec_limit:
        time.sleep(0.1)
        tick += 0.1
    print("wait timed out") if tick >= sec_limit else None


class ConfigureIP:
    """
    Configure the inverted pendulum for the hardware:
        Antigravity KV300 BLDC motors and AMT102-V encoders
    ODrive v3.6
    """

    MOTOR_KV = 300
    ENCODER_CPR = 8192
    NUM_POLES = 24
    VBUS_VOLTAGE = 24

    def __init__(self, calibrate=True):
        self.reconnect()
        self.configure()
        self.calibrate()

    @classmethod
    def get_odrive(cls, output=True):
        # connect to Odrive
        odrv = None
        print("Looking for ODrive...") if output else None
        odrv = odrive.find_any()
        print("Found ODrive.") if output else None
        return odrv

    def reconnect(self):
        self.odrive = self.get_odrive()
        self.flywheel = self.odrive.axis1
        self.axle = self.odrive.axis0

    def configure(self):
        print("Erasing old configuration")
        try:
            self.odrive.erase_configuration()
        except ObjectLostError:
            pass
        self.reconnect(output=False)

        print("Applying configuration settings...")
        self._config_flywheel()
        # self._config_encoder(self.flywheel)
        self._config_encoder(self.axle)
        try:
            self.odrive.save_configuration()
        except ObjectLostError:
            pass

        self.reconnect()


    def _config_encoder(self, axis):
        axis.encoder.config.cpr = ConfigureIP.ENCODER_CPR
        axis.encoder.config.mode = ENCODER_MODE_INCREMENTAL
        axis.encoder.config.use_index = True

    def _config_flywheel(self):
        self.odrive.config.enable_brake_resistor = True
        self.odrive.config.brake_resistance = 1
        # to avoid motors from heating up when finding index (ENCODER_INDEX_SEARCH)
        self.flywheel.config.calibration_lockin.current = 2
        j
        self.flywheel.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.flywheel.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        self.flywheel.controller.config.vel_limit = 10

        # The motors have a peak current of 9 amps
        self.flywheel.motor.config.current_lim = 9
        self.flywheel.motor.config.requested_current_range = 10

        # The speed of the motor will be limited to this speed in [turns/sec]
        self.flywheel.motor.config.pole_pairs = (
            ConfigureIP.NUM_POLES / 2
        )  # The MN4004 has 24 magnet poles, so 12 pole pairs
        # The MN4004 has an idle current of 0.2 A but we set it at 2 for it to go faster during calibration
        self.flywheel.motor.config.calibration_current = 2

        # this is specified in the odrive documentation
        self.flywheel.motor.config.torque_constant = 8.27 / ConfigureIP.MOTOR_KV
        self.flywheel.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT

        # self.flywheel.controller.config.vel_gain = 0.01
        # self.flywheel.controller.config.pos_gain = 212.0

        # vel_integrator_gain = 0.5 * vel_gain * bandwidth (in hertz)
        # self.flywheel.controller.config.vel_integrator_gain = 0.5 * self.flywheel.controller.config.vel_gain * 1/(self.flywheel.encoder.config.bandwidth/1000)

        self._config_sensorless()

    def _config_sensorless(self):
        self.flywheel.config.enable_sensorless_mode = True

        self.flywheel.controller.config.vel_gain = 0.01
        self.flywheel.controller.config.vel_integrator_gain = 0.05

        # 5 turns_per_sec / (2 * 3.14159265 * NUM_POLES / 2)
        self.flywheel.sensorless_estimator.config.pm_flux_linkage = 5.51328895422 / (
            2 * ConfigureIP.NUM_POLES * ConfigureIP.MOTOR_KV
        )

    def calibrate(self):
        input("Make sure the motor is free to move, then press enter...")
        print("Calibrating flywheel motor and encoders")
        self.flywheel.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        print("\nCalibrating Flywheel (You should hear a beep)...")

        # Wait for calibration to take place
        wait_until(lambda: self.flywheel.motor.config.phase_inductance != 0)
        print("Finished!")



