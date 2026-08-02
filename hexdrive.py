"""HexDrive Hexpansion App for BadgeBot."""

# This is the app to be installed from the HexDrive Hexpansion EEPROM.
# it is copied onto the EEPROM and renamed as app.py/mpy
# It is then run from the EEPROM by the BadgeOS.

import ota
from machine import PWM, Pin
from system.eventbus import eventbus
from system.hexpansion.config import HexpansionConfig
from system.scheduler.events import RequestStopAppEvent
import app
import micropython

try:
    from micropython import const
except ImportError:
    # CPython / simulator fallback – const() is an identity function on MicroPython
    const = lambda x: x  # noqa: E731

# Define the minimum BadgeOS version required to run this app (e.g. if we need features that are only available in a certain version of BadgeOS)
_MIN_BADGEOS_VERSION = [1, 9, 0]     # v1.9.0 is required to be able to read the EEPROM with 16-bit addressing

# HexDrive Hexpansion constants
# Hardware defintions:
_ENABLE_PIN  = 0  # First LS pin used to enable the SMPSU

# Default values and limits:
_DEFAULT_PWM_FREQ = const(20000)           # 20kHz is a good default for motors as it is above the audible range for most people and works with most motors and ESCs
_DEFAULT_SERVO_FREQ = const(50)            # 50Hz = 20mS period
_DEFAULT_KEEP_ALIVE_PERIOD = const(1000)   # 1 second
_MAX_NUM_CHANNELS = const(4)               # Max number of PWM channels supported by any type of HexDrive (Hexpansion limitation, not BadgeBot limit)
_MAX_NUM_MOTORS = const(2)                 # Max number of motor channels supported by any type of HexDrive

# Servo Constants
_MIN_SERVO_FREQ = const(10)                # 10Hz = 100mS period (minimum frequency for servos)
_MAX_SERVO_FREQ = const(200)               # 200Hz = 5mS period (can work with some Servos but not all)
_SERVO_CENTRE    = const(1500)             # 1500us pulse width is the centre position for most RC servos (but some may be different, so we allow this to be trimmed)
_MAX_SERVO_RANGE = const(1400)             # 1400us either side of centre (VERY WIDE)
_SERVO_MAX_TRIM  = const(1000)             # 1000us either side of centre for trimming the centre position

# EEPROM Constants
_EEPROM_ADDR  = const(0x50)                # I2C address of the EEPROM on the HexDrive and HexSense Hexpansion
_EEPROM_NUM_ADDRESS_BYTES = const(2)       # Number of bytes used for the memory address when reading from the EEPROM (e.g. 2 for 16-bit addressing)
_PID_ADDR     = const(0x12)                # Address in the EEPROM where the Product ID (PID) byte is stored - used to identify the type of Hexpansion


class HexDriveType:
    """Represents a sub-type of HexDrive Hexpansion module."""
    __slots__ = ("pid", "name", "motors", "servos", "hw_ver")

    def __init__(self, pid_byte: int, motors: int = 0, servos: int = 0, name: str = "Unknown"):
        self.pid: int = pid_byte         # Product ID byte read from the EEPROM to identify the type of HexDrive
        self.name: str = name            # A friendly name for the type of HexDrive
        self.motors: int = motors        # Number of motor channels supported by this type of HexDrive (0, 1 or 2)
        self.servos: int = servos        # Number of servo channels supported by this type of HexDrive (0, 2 or 4)

_HEXDRIVE_TYPES = (
    HexDriveType(0xCA, motors=2, name="2 Motor"),
    HexDriveType(0xCB, motors=2, servos=4),
    HexDriveType(0xCC, servos=4, name="4 Servo"),
    HexDriveType(0xCD, motors=1, servos=2, name="1 Mot 2 Srvo"),
    HexDriveType(0xCE, motors=1, name="1 Motor"),
)

class HexDriveApp(app.App):         # pylint: disable=no-member
    """ HexDrive Hexpansion App for BadgeBot."""
    VERSION = 6         # Increment this when making changes to the app that require the hexpansion app to be re-flashed with the new code.

    def __init__(self, config: HexpansionConfig):
        super().__init__()

        self.config: HexpansionConfig = config
        self._logging: bool = True
        self._keep_alive_period: int = _DEFAULT_KEEP_ALIVE_PERIOD
        self._power_state: bool = False
        self._pwm_setup: bool = False
        self._time_since_last_update: int = 0
        self._outputs_energised: bool = False
        self.PWMOutput: list[PWM | None] = [None] * _MAX_NUM_CHANNELS
        self._freq: list[int] = [0] * _MAX_NUM_CHANNELS
        self._motor_output: list[int] = [0] * _MAX_NUM_MOTORS

        # LS Pins
        self._power_control = self.config.ls_pin[_ENABLE_PIN]

        self._servo_centre = [_SERVO_CENTRE] * _MAX_NUM_CHANNELS
        eventbus.on_async(RequestStopAppEvent, self._handle_stop_app, self)
        # What version of BadgeOS are we running on?
        try:
            ver = self._parse_version(ota.get_version())
            #print(f"D:S/W {ver}")
            # e.g. v1.9.0-beta.1
            if ver >= _MIN_BADGEOS_VERSION:
                # we need v1.9.0+ to be able to read the EEPROM with 16-bit addressing, so if we are running on an older version then we cannot continue
                pass
            else:
                print("D:BadgeOS Upgrade required")
                return
        except Exception as e:      # pylint: disable=broad-except
            print(f"D:Ver check failed {e}")

        # read hexpansion header from EEPROM to find out which sub-type we are
        try:
            self._hexdrive_type: HexDriveType = self._check_port_for_hexdrive(self.config.port)
        except Exception as e:      # pylint: disable=broad-except
            print(f"D:{self.config.port}:HexDrive type check failed {e}")
            return

        self.initialise()


    def initialise(self) -> bool:
        """Initialise the app - return True if successful, False if failed."""
        self._pwm_setup = False

        # report app starting and which port it is running on
        print(f"D:HexDrive Type:'{self._hexdrive_type.name}' V{self.VERSION} by RobotMad on port {self.config.port}")

        # Initialise HS Pins
        for _, hs_pin in enumerate(self.config.pin):
            # Set HexDrive Hexpansion HS pins to low level outputs
            hs_pin.init(mode=Pin.OUT)
            hs_pin.value(0)

        # Initialise LS Pins
        try:
            self._power_control.init(mode=Pin.OUT)
        except Exception as e:      # pylint: disable=broad-except
            print(f"D:{self.config.port}:ls_pin setup failed {e}")
            return False

        # ensure SMPSU is turned off to start with
        self.set_power(False)

        # allocate PWM outputs according to the type of HexDrive
        return self._pwm_init()


    def deinit(self) -> bool:
        """ De-initialise the app - return True if successful, False if failed."""
        # Turn off all PWM outputs & release resources
        self.set_power(False)
        self._pwm_deinit()
        for hs_pin in self.config.pin:
            hs_pin.init(mode=Pin.IN)
        return True


    async def _handle_stop_app(self, event):
        """ Handle the RequestStopAppEvent so that we can release resources """
        try:
            if event.app == self:
                if self._logging:
                    print(f"D:{self.config.port}:Stop")
                self.deinit()
        except (AttributeError, TypeError):
            pass


    @micropython.native
    def background_update(self, delta: int):
        """ This is called from the main loop of the BadgeOS to allow the app to do any background processing it needs to do. """
        if (self.config is None) or not self._pwm_setup:
            # if we are not properly initialised then do not attempt to do anything
            return
        # Check keep alive period and turn off PWM outputs if exceeded
        self._time_since_last_update += delta
        if self._time_since_last_update > self._keep_alive_period:
            self._time_since_last_update = 0
            if self._outputs_energised:
                self._outputs_energised = False
                # First time the keep alive period has expired so report it
                if self._logging:
                    print(f"D:{self.config.port}:Timeout")
            if self._pwm_setup:
                for channel,pwm in enumerate(self.PWMOutput):
                    if pwm is not None:
                        try:
                            pwm.duty_u16(0)
                        except Exception as e:          # pylint: disable=broad-except
                            print(self._pwm_log_string(channel) + f"Off failed {e}")
                            self.PWMOutput[channel] = None  # Tidy Up
            # we keep retriggering in case anything else has corrupted the PWM outputs


    def get_status(self) -> bool:
        """ Get the current status of the app - True if the app is running and able to respond to commands, False if not. """
        return self._pwm_setup


    @property
    def capabilities(self) -> int:
        """ Return the capabilities of this HexDrive Hexpansion app as a bitmask of flags. """
        return 0    # for compatbility with HexDrive2 - this version has no extra sensor capabilities


    def set_logging(self, state: bool):
        """ Set the logging state - True to enable logging, False to disable logging. """
        self._logging = state


    def set_power(self, state: bool) -> bool:
        """ Turn the SMPSU on or off. Returns the new power state.
            Note that just because the SMPSU is turned off does not mean that the outputs are NOT energised as there could be external battery power. """
        if state == self._power_state:
            return True  # No change needed
        if self._logging:
            print(f"D:{self.config.port}:Power={'On' if state else 'Off'}")
        try:
            self._power_control.init(mode=Pin.OUT)
            self._power_control.value(state)
        except Exception as e:      # pylint: disable=broad-except
            print(f"D:{self.config.port}:{e}")
            return False
        self._power_state = state
        return True


    # Deprecated
    # def get_power(self) -> bool:
    #     """ Get the current state of the SMPSU enable pin. Returns True if enabled, False if disabled. """
    #     return self._power_state


    def set_keep_alive(self, period: int):
        """ Set the keep alive period in milliseconds:
            This is the period of time that can elapse without any commands being received before the app automatically
            turns off all outputs to prevent damage to motors or servos if something goes wrong. """
        self._keep_alive_period = period


    def set_freq(self, freq: int, channel: int | None = None) -> bool:
        """ Set the PWM frequency for a specific output, or all outputs if channel is None. Returns True if successful, False if failed.
            Use 50 to 200 for Servos and 5000 to 20000 for motors. """
        if not self._pwm_setup:
            return False
        for this_channel, pwm in enumerate(self.PWMOutput):
            if (channel is None or this_channel == channel):
                if pwm is not None:
                    try:
                        pwm.freq(freq)
                        if self._logging:
                            print(self._pwm_log_string(this_channel) + f"{freq}Hz")
                            print(self.PWMOutput[this_channel])
                    except Exception as e:  # pylint: disable=broad-except
                        print(self._pwm_log_string(this_channel) + f"1:{e}")
                        print(f"pwm: {pwm}")
                        return False
                self._freq[this_channel] = freq
        return True


    def _pwm_log_string(self, channel: int | None) -> str:
        """ Helper method to generate a log string for a PWM output change. """
        return f"D:{self.config.port}:PWM[{'All' if channel is None else channel}]:"


    def set_servoposition(self, channel: int | None = None, position: int | None = None) -> bool:
        """ Set the position for a specific servo output, or all servo outputs if channel is None. Returns True if successful, False if failed.
            The pulse width for a specific servo output is position + the centre offset (in us)
            Based on standard RC servos with centre at 1500us and range of 1000-2000us.
            The position is a signed value from -1000 to 1000 which is scaled to 500-2500us.
            This is a very wide range and may not be suitable for all servos, some will
            only be happy with 1000-2000us (i.e. position in the range -500 to 500). """
        if not self._pwm_setup:
            return False
        if position is None:
            # position == None -> Turn off PWM (some servos will then turn off, others will stay in last position)
            if channel is None:
                # channel == None -> Turn off all PWM outputs
                for ch, pwm in enumerate(self.PWMOutput):
                    if pwm is not None:
                        try:
                            pwm.duty_ns(0)
                        except Exception as e:  # pylint: disable=broad-except
                            print(self._pwm_log_string(ch) + f"2:{e}")
                if self._logging:
                    print(self._pwm_log_string(None) + "Off")
                self._outputs_energised = False
                return True
            elif channel < 0 or channel >= self._hexdrive_type.servos:
                return False
            else:
                pwm = self.PWMOutput[channel]
                if pwm:
                    try:
                        pwm.duty_ns(0)
                        if self._logging:
                            print(self._pwm_log_string(channel) + "Off")
                    except Exception as e:  # pylint: disable=broad-except
                        print(self._pwm_log_string(channel) + f"3:{e}")
                        return False
            # check if all channels are now off and set outputs_energised accordingly
            self._check_outputs_energised()
        elif channel is not None:
            if channel < 0 or channel >= self._hexdrive_type.servos:
                return False
            if abs(position) > _MAX_SERVO_RANGE:
                return False
            pulse_width_in_ns = (self._servo_centre[channel] + position) * 1000 # convert from us to ns
            pwm = self.PWMOutput[channel]
            if pwm is None:
                # Channel hasn't been setup yet so we need to initialise it from scratch
                if self._freq[channel] > _MAX_SERVO_FREQ or self._freq[channel] < _MIN_SERVO_FREQ:
                    # force the frequency to be suitable for use with Servos otherwise the pulse width will not be accepted
                    self._freq[channel] = _DEFAULT_SERVO_FREQ
                try:
                    pwm = PWM(self.config.pin[channel], freq = self._freq[channel]) # Exception "PWM is inactive" if you try to set duty here
                    pwm.duty_ns(pulse_width_in_ns)
                    self.PWMOutput[channel] = pwm
                    if self._logging:
                        print(self._pwm_log_string(channel) + f"{pwm} init")
                except Exception as e:      # pylint: disable=broad-except
                    # There are a finite number of PWM resources so it is possible that we run out
                    print(self._pwm_log_string(channel) + f"4:{e}")
                    return False
            else:
                # Channel is already setup so we just need to change the duty cycle and possibly the frequency if it is too high for the servo
                try:
                    if _MAX_SERVO_FREQ < pwm.freq():
                        # Ensure the frequency is suitable for use with Servos
                        # otherwise the pulse width will not be accepted
                        self._freq[channel] = _DEFAULT_SERVO_FREQ
                        pwm.freq(_DEFAULT_SERVO_FREQ)
                        if self._logging:
                            print(self._pwm_log_string(channel) + f"{_DEFAULT_SERVO_FREQ}Hz for Servo")
                except Exception as e:          # pylint: disable=broad-except
                    print(self._pwm_log_string(channel) + f"5:{e}")
                    return False
                # Scale servo position to PWM duty cycle (500-2500us)
                try:
                    if 2000 < abs(pulse_width_in_ns - pwm.duty_ns()):    # allow tolerance of 2us to avoid unnecessary updates
                        pwm.duty_ns(pulse_width_in_ns)
                except Exception as e:          # pylint: disable=broad-except
                    print(self._pwm_log_string(channel) + f"6:{e}")
                    return False

            self._outputs_energised = True
        self._time_since_last_update = 0
        return True


    def set_servocentre(self, centre: int, channel: int | None = None) -> bool:
        """ Set the centre position for a specific servo output, or all servo outputs if channel is None. Returns True if successful, False if failed.
            Note this does not change the current position of the servo.
            It will only affect the position next time it is set.
            You can use this to trim the centre position of the servo. """
        if not self._pwm_setup:
            return False
        if channel is not None and (channel < 0 or channel >= self._hexdrive_type.servos):
            return False
        if centre < (_SERVO_CENTRE - _SERVO_MAX_TRIM ) or centre > (_SERVO_CENTRE + _SERVO_MAX_TRIM):
            return False
        if channel is None:
            self._servo_centre = [centre] * 4
        else:
            self._servo_centre[channel] = centre
        return True


    # Set pairs of PWM duty cycles in one go using a signed value per motor channel (0-65535)
    def set_motors(self, outputs: tuple[int, ...]) -> bool:
        """ Set the motor outputs using a signed value for each motor channel. Returns True if successful, False if failed.
            The outputs are signed values in a tuple from -65535 to 65535 which are scaled to the PWM duty cycle range of 0-65535.
            A positive value will drive the motor in one direction, a negative value will drive it in the opposite direction,
            and a value of 0 will stop the motor. """
        if not self._pwm_setup or len(outputs) > self._hexdrive_type.motors:
            return False
        for motor, output in enumerate(outputs):
            if abs(output) > 65535:
                return False
            if output == self._motor_output[motor]:
                # no change in output for this motor so skip to the next one
                continue
            try:
                # if the output is changing direction then we need to switch which signal is being driven as the PWM output
                # rather than test for change of direction and also test that PWMOutput to be disabled exists we just do the latter check.
                output_to_enable = (motor<<1) if output > 0 else ((motor<<1)+1)
                output_to_disable = (motor<<1)+1 if output > 0 else (motor<<1)
                pwm_to_disable = self.PWMOutput[output_to_disable]
                pwm_to_enable = self.PWMOutput[output_to_enable]
                if pwm_to_disable:
                    pwm_to_disable.duty_u16(0)
                    if self._logging:
                        print(self._pwm_log_string(output_to_disable) + f"{0}")
                if pwm_to_enable:
                    pwm_to_enable.duty_u16(abs(output))
                    if self._logging:
                        print(self._pwm_log_string(output_to_enable) + f"{abs(output)}")
                elif not self._set_pwmoutput(output_to_enable, abs(output)):
                    return False
            except Exception as e:          # pylint: disable=broad-except
                print(f"D:{self.config.port}:{e}")
                return False
            self._motor_output[motor] = output
        self._check_outputs_energised()
        self._time_since_last_update = 0
        return True


    # Set all 4 PWM duty cycles in one go using a tuple (0-65535)
    def set_pwm(self, duty_cycles: tuple[int, ...]) -> bool:
        """ Set the PWM duty cycle for all outputs at once using a tuple of values. Returns True if successful, False if failed.
            The duty_cycles are values from 0 to 65535. """
        if not self._pwm_setup:
            return False
        self._outputs_energised = any(duty_cycles)
        for channel, duty_cycle in enumerate(duty_cycles):
            if not self._set_pwmoutput(channel, duty_cycle):
                return False
        self._time_since_last_update = 0
        return True


# --------------------------------------------------
# Private methods for internal use only.
# --------------------------------------------------

    def _pwm_init(self) -> bool:
        self._pwm_setup = False
        # HS Pins
        if self.config.pin is not None and len(self.config.pin) == 4:
            # Allocate PWM generation to pins
            for channel, _ in enumerate(self.config.pin):
                self._freq[channel] = 0
                if channel < (2 * self._hexdrive_type.motors):
                    # First channels are for motors (can be 0, 1 or 2 motors)
                    self._freq[channel] = _DEFAULT_PWM_FREQ
                    if 0 == channel % 2:
                        # initialise motor PWM output on even channel
                        self._motor_output[(channel>>1)] = 0
                        #if not self._set_pwmoutput(channel, 0):
                        #    return False
                        #print(f"D:{self.config.port}:Motor PWM[{channel}]")
                    else:
                        # ignore the motor PWM output on odd channel - we will switch it on when needed
                        pass
                elif channel < ((2 * self._hexdrive_type.motors) + self._hexdrive_type.servos):
                    # Remaining channels are for servos (can be 4, 2 or 0 servos
                    self._freq[channel] = _DEFAULT_SERVO_FREQ
                    #if not self._set_pwmoutput(channel, 0):
                    #    return False
                    #print(f"D:{self.config.port}:Servo PWM[{channel}]")
                else:
                    # ignore the remaining channels - we will switch them on when needed
                    pass
        self._pwm_setup = True
        return self._pwm_setup


    # De-initialise all PWM outputs
    def _pwm_deinit(self):
        for channel, pwm in enumerate(self.PWMOutput):
            if pwm is not None:
                try:
                    pwm.deinit()
                except Exception:       # pylint: disable=broad-except
                    pass
                self.PWMOutput[channel] = None
            self._freq[channel] = 0
            self._motor_output[(channel>>1)] = 0
        self._pwm_setup = False


    # are any of the PWM outputs energised?
    def _check_outputs_energised(self):
        energised_output = False
        for channel, pwm in enumerate(self.PWMOutput):
            if pwm is not None:
                try:
                    if 0 < pwm.duty_ns():
                        energised_output = True
                        break
                except Exception as e:        # pylint: disable=broad-except
                    print(self._pwm_log_string(channel) + f"Check failed {e}")
        if self._outputs_energised != energised_output:
            #if self._logging:
            #    print(f"D:{self.config.port}:Outputs {'Energised' if energised_output else 'De-energised'}")
            self._outputs_energised = energised_output


    # Set a single PWM duty cycle (0-65535) for a specific output
    # if the channel has not been setup yet then we initialise it from scratch, otherwise we just change the duty cycle
    def _set_pwmoutput(self, channel: int, duty_cycle: int) -> bool:
        if duty_cycle < 0 or duty_cycle > 65535:
            return False
        try:
            pwm = self.PWMOutput[channel]
            if pwm is None:
                # Channel hasn't been setup yet so we need to initialise it from scratch
                pwm = PWM(self.config.pin[channel], freq = self._freq[channel])
                self.PWMOutput[channel] = pwm
                pwm.duty_u16(duty_cycle)
                if self._logging:
                    print(self._pwm_log_string(channel) + f"{pwm} init")
            elif duty_cycle != pwm.duty_u16():
                pwm.duty_u16(duty_cycle)
                if self._logging:
                    print(self._pwm_log_string(channel) + f"{duty_cycle}")
        except Exception as e:              # pylint: disable=broad-except
            print(self._pwm_log_string(channel) + f"7:{e}")
            return False
        return True


    def _check_port_for_hexdrive(self, port: int) -> HexDriveType:
        #just read the part of the header which contains the PID
        pid_bytes = self.config.i2c.readfrom_mem(_EEPROM_ADDR, _PID_ADDR, 2, addrsize = 8*_EEPROM_NUM_ADDRESS_BYTES)
        # check which type of HexDrive this is by scanning the HEXDRIVE_TYPES list
        for _, hexpansion_type in enumerate(_HEXDRIVE_TYPES):
            if pid_bytes[0] == hexpansion_type.pid:
                return hexpansion_type
        # we are not interested in this type of hexpansion
        raise ValueError(f"Unknown HexDrive PID 0x{pid_bytes[0]:02X} on port {port}")


    def _parse_version(self, version):
        """ Parse a version string, e.g. that of BadgeOS, into a list of components for comparison. Handles versions in the format v1.9.0-beta.1+build.123
            The version is split into components based on the delimiters '.' '-' and '+'."""
        #pre_components = ["final"]
        #build_components = ["0", "000000z"]
        #build = ""
        components = []
        if "+" in version:
            version, build = version.split("+", 1)          # pylint: disable=unused-variable
        #    build_components = build.split(".")
        if "-" in version:
            version, pre_release = version.split("-", 1)    # pylint: disable=unused-variable
        #    if pre_release.startswith("rc"):
        #        # Re-write rc as c, to support a1, b1, rc1, final ordering
        #        pre_release = pre_release[1:]
        #    pre_components = pre_release.split(".")
        version = version.strip("v").split(".")
        components = [int(item) if item.isdigit() else item for item in version]
        #components.append([int(item) if item.isdigit() else item for item in pre_components])
        #components.append([int(item) if item.isdigit() else item for item in build_components])
        return components


__app_export__ = HexDriveApp
