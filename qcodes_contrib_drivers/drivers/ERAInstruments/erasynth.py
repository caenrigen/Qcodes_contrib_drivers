"""Driver for the ERASynth/ERASynth+/ERASynth++ signal generators.

Author: Victor Negîrneac, vnegirneac@qblox.com

For official instrument support visit:

- https://erainstruments.com
- https://github.com/erainstruments/

NB this driver adds parameters using the new `@add_paramter` decorator-style.
See QCoDeS docs on writing drivers for more details.
"""
# pylint: disable=dangerous-default-value
from __future__ import annotations

from typing import Union
import time
import json
from qcodes import Instrument
from qcodes import validators

try:
    from qcodes.instrument.base import InstanceAttr, add_parameter
except ImportError as e:
    raise ImportError(
        "This (decorator-style) driver requires qcodes version > 0.26.0."
    ) from e

try:
    import serial
    from serial.tools.list_ports import comports
except ModuleNotFoundError as e:
    raise ModuleNotFoundError(
        "ERAInstrument drivers require the `pyserial` package to be installed.\n"
        "Install it with `pip install pyserial` and try again."
    ) from e

import logging

logger = logging.getLogger(__name__)

class ERASynthBase(Instrument):
    """
    A Base class for the ERASynth/ERASynth+/ERASynth++ instruments.
    """

    @staticmethod
    def print_serial_ports():
        """Utility to list all serial communication ports."""
        for port, description, hwid in sorted(comports()):
            print(f"{port!r}\n    description: {description!r}\n    hwid: {hwid!r}")

    def __init__(
        self, name: str, address: str, baudrate: int = 115200, serial_timeout=3.0
    ):
        """
        Create an instance of the instrument.

        Args:
            name: Instrument name.
            address: Used to connect to the instrument e.g., "COM3".
            baudrate: The speed of the serial communication.
            serial_timeout: timeout for serial read operations.

        Example:

        .. code-block::

            # import driver
            from qcodes_contrib_drivers.drivers.ERAInstruments import erasynth

            # list communication ports
            erasynth.ERASynth.print_serial_ports()

            # Instantiate the instrument
            lo = erasynth.ERASynth("erasynth", "/dev/cu.usbmodem141101")
            # lo.factory_reset() # if desired, also resets wifi-related parameters
            lo.preset() # reset settings

            # print updated snapshot once to make sure the snapshot will be up-to-date
            # takes a few seconds
            print()
            lo.print_readable_snapshot(update=True)

            # Configure the local oscillator
            lo.ref_osc_source("int")  # Use internal reference
            lo.frequency(4.7e9)
            lo.power(10)  # Set the amplitude to 10 dBm
            lo.on()  # Turn on the output
            print()
            lo.print_readable_snapshot()


        .. seealso::

            The driver uses the serial communication directly. The raw serial commands
            can be found here:
            https://github.com/erainstruments/erasynth-docs/blob/master/erasynth-command-list.pdf
        """
        super().__init__(name)

        self.ser = serial.Serial()
        self.ser.baudrate = baudrate
        self.ser.port = address
        self.ser.serial_timeout = serial_timeout
        self.open_port()

    # ##############################################################################
    # Standard LO parameters
    # ##############################################################################

    # NB `initial_value` is not used because that would make the initialization slow

    @add_parameter
    def _parameter_status(
        self,
        val_mapping={False: "0", True: "1"},
        get_cmd=InstanceAttr("_get_rfoutput"),
        set_cmd=InstanceAttr("_set_rfoutput"),
    ):
        """Turns the output state (`True`/`False`)."""

    @add_parameter
    def _parameter_power(
        self,
        label="Power",
        unit="dBm",
        vals=validators.Numbers(min_value=-60.0, max_value=20.0),
        get_cmd=InstanceAttr("_get_pwer"),
        get_parser=float,
        set_parser=lambda power: f"{power:.2f}",  # only to decimal points supported
        set_cmd=InstanceAttr("_set_power"),
    ):
        """Signal power in dBm of the ERASynth signal, 'amplitude' in EraSynth docs."""

    @add_parameter
    def _parameter_ref_osc_source(
        self,
        val_mapping={"int": "0", "ext": "1"},
        get_cmd=InstanceAttr("_get_reference_int_ext"),
        set_cmd=InstanceAttr("_set_reference_int_ext"),
    ):
        """
        Set to external if a 10 MHz reference is connected to the REF input connector.
        """

    # ##############################################################################
    # ERASynth specific parameters
    # ##############################################################################

    @add_parameter
    def _parameter_temperature(
        self,
        label="Temperature",
        unit="\u00B0C",
        get_cmd=InstanceAttr("_get_temperature"),
    ):
        """Temperature of the device."""

    @add_parameter
    def _parameter_voltage(
        self,
        label="Voltage",
        unit="V",
        get_cmd=InstanceAttr("_get_voltage"),
    ):
        """The input voltage value from power input of the ERASynth."""

    @add_parameter
    def _parameter_current(
        self,
        label="Current",
        unit="V",
        get_cmd=InstanceAttr("_get_current"),
    ):
        """The current value drawn by the ERASynth."""

    @add_parameter
    def _parameter_em(
        self,
        label="Embedded version",
        get_cmd=InstanceAttr("_get_em"),
    ):
        """The firmware version of the ERASynth."""

    @add_parameter
    def _parameter_wifi_rssi(
        self,
        label="WiFi RSSI",
        get_cmd=InstanceAttr("_get_rssi"),
    ):
        """The Wifi received signal power."""

    @add_parameter
    def _parameter_pll_lmx1_status(
        self,
        label="PLL LMX1 status",
        val_mapping={"locked": "1", "not_locked": "0"},
        get_cmd=InstanceAttr("_get_lock_lmx1"),
    ):
        """PLL lock status of LMX1."""

    @add_parameter
    def _parameter_pll_lmx2_status(
        self,
        label="PLL LMX2 status",
        val_mapping={"locked": "1", "not_locked": "0"},
        get_cmd=InstanceAttr("_get_lock_lmx2"),
    ):
        """PLL lock status of LMX2."""

    @add_parameter
    def _parameter_pll_xtal_status(
        self,
        label="PLL XTAL status",
        val_mapping={"locked": "1", "not_locked": "0"},
        get_cmd=InstanceAttr("_get_lock_xtal"),
    ):
        """PLL lock status of XTAL."""

    @add_parameter
    def _parameter_modulation_on_off(
        self,
        val_mapping={"off": "0", "on": "1"},
        get_cmd=InstanceAttr("_get_modulation_on_off"),
        set_cmd=InstanceAttr("_set_modulation_on_off"),
    ):
        """Modulation on/off."""

    @add_parameter
    def _parameter_modulation_signal_waveform(
        self,
        val_mapping={"sine": "0", "triangle": "1", "ramp": "2", "square": "3"},
        get_cmd=InstanceAttr("_get_modulation_signal_waveform"),
        set_cmd=InstanceAttr("_set_modulation_signal_waveform"),
    ):
        """Internal modulation waveform."""

    @add_parameter
    def _parameter_modulation_source(
        self,
        val_mapping={"internal": "0", "external": "1", "microphone": "2"},
        get_cmd=InstanceAttr("_get_modulation_source"),
        set_cmd=InstanceAttr("_set_modulation_source"),
    ):
        """Modulation source."""

    @add_parameter
    def _parameter_modulation_type(
        self,
        val_mapping={
            "narrowband_fm": "0",
            "wideband_fm": "1",
            "am": "2",
            "pulse": "3",
        },
        get_cmd=InstanceAttr("_get_modulation_type"),
        set_cmd=InstanceAttr("_set_modulation_type"),
    ):
        """Modulation type."""

    @add_parameter
    def _parameter_modulation_freq(
        self,
        label="Modulation frequency",
        unit="Hz",
        vals=validators.Numbers(min_value=0, max_value=20e9),
        get_cmd=InstanceAttr("_get_modulation_freq"),
        get_parser=int,
        set_cmd=InstanceAttr("_set_modulation_freq"),
        set_parser=lambda freq: str(int(freq)),
    ):
        """Internal modulation frequency in Hz."""

    @add_parameter
    def _parameter_modulation_am_depth(
        self,
        label="AM depth",
        unit="%",
        vals=validators.Numbers(min_value=0, max_value=100),
        get_cmd=InstanceAttr("_get_modulation_am_depth"),
        get_parser=int,
        set_cmd=InstanceAttr("_set_modulation_am_depth"),
        set_parser=lambda depth: str(int(depth)),
    ):
        """AM modulation depth."""

    @add_parameter
    def _parameter_modulation_fm_deviation(
        self,
        label="FM deviation",
        unit="Hz",
        vals=validators.Numbers(min_value=0, max_value=20e9),
        get_cmd=InstanceAttr("_get_modulation_fm_deviation"),
        get_parser=int,
        set_cmd=InstanceAttr("_set_modulation_fm_deviation"),
        set_parser=lambda freq: str(int(freq)),
    ):
        """FM modulation deviation."""

    @add_parameter
    def _parameter_modulation_pulse_period(
        self,
        label="Pulse period",
        unit="s",
        vals=validators.Numbers(min_value=1e-6, max_value=10),
        get_cmd=InstanceAttr("_get_modulation_pulse_period"),
        get_parser=lambda val: int(val) * 1e-6,
        set_cmd=InstanceAttr("_set_modulation_pulse_period"),
        set_parser=lambda period: str(int(period * 1e6)),
    ):
        """Pulse period in seconds."""

    @add_parameter
    def _parameter_modulation_pulse_width(
        self,
        label="Pulse width",
        unit="s",
        vals=validators.Numbers(min_value=1e-6, max_value=10),
        get_cmd=InstanceAttr("_get_modulation_pulse_width"),
        get_parser=lambda val: int(val) * 1e-6,
        set_cmd=InstanceAttr("_set_modulation_pulse_width"),
        set_parser=lambda period: str(int(period * 1e6)),
    ):
        """Pulse width in s."""

    @add_parameter
    def _parameter_sweep(
        self,
        val_mapping={"off": "0", "on": "1"},
        get_cmd=InstanceAttr("_get_sweep_start_stop"),
        set_cmd=InstanceAttr("_set_sweep_start_stop"),
    ):
        """Sweep on/off."""

    @add_parameter
    def _parameter_sweep_trigger(
        self,
        val_mapping={"freerun": "0", "external": "1"},
        get_cmd=InstanceAttr("_get_sweep_trigger"),
        set_cmd=InstanceAttr("_set_sweep_trigger"),
    ):
        """Sweep trigger freerun/external."""

    @add_parameter
    def _parameter_sweep_start(
        self,
        label="Sweep start",
        unit="Hz",
        vals=validators.Numbers(min_value=250e3, max_value=20e9),
        get_cmd=InstanceAttr("_get_sweep_start"),
        get_parser=int,
        set_cmd=InstanceAttr("_set_sweep_start"),
        set_parser=lambda freq: str(int(freq)),
    ):
        """Sweep start frequency in Hz."""

    @add_parameter
    def _parameter_sweep_stop(
        self,
        label="Sweep stop",
        unit="Hz",
        vals=validators.Numbers(min_value=250e3, max_value=20e9),
        get_cmd=InstanceAttr("_get_sweep_stop"),
        get_parser=int,
        set_cmd=InstanceAttr("_set_sweep_stop"),
        set_parser=lambda freq: str(int(freq)),
    ):
        """Sweep stop frequency in Hz."""

    @add_parameter
    def _parameter_sweep_step(
        self,
        label="Sweep step",
        unit="Hz",
        vals=validators.Numbers(min_value=0, max_value=20e9),
        get_cmd=InstanceAttr("_get_sweep_step"),
        get_parser=int,
        set_cmd=InstanceAttr("_set_sweep_step"),
        set_parser=lambda freq: str(int(freq)),
    ):
        """Sweep step frequency in Hz."""

    @add_parameter
    def _parameter_sweep_dwell(
        self,
        label="Sweep dwell",
        unit="s",
        vals=validators.Numbers(min_value=1e-3, max_value=10),
        get_cmd=InstanceAttr("_get_sweep_dwell"),
        get_parser=lambda val: int(val) * 1e-3,
        set_cmd=InstanceAttr("_set_sweep_dwell"),
        set_parser=lambda period: str(int(period * 1e3)),
    ):
        """Sweep dwell time in s. Requires sweep_trigger('freerun')."""

    @add_parameter
    def _parameter_reference_tcxo_ocxo(
        self,
        val_mapping={"tcxo": "0", "ocxo": "1"},
        get_cmd=InstanceAttr("_get_reference_tcxo_ocxo"),
        set_cmd=InstanceAttr("_set_reference_tcxo_ocxo"),
    ):
        """
        Set to external if a 10 MHz reference is connected to the REF input connector.
        """

    @add_parameter
    def _parameter_synthesizer_mode(
        self,
        val_mapping={"low_spurious": "0", "low_phase_noise": "1"},
        get_cmd=InstanceAttr("_get_phase_noise_mode"),
        set_cmd=InstanceAttr("_set_phase_noise_mode"),
    ):
        """Synthesizer mode: low spurious/low phase noise."""

    # WiFi control, NB initial_cache_value is used to avoid overriding these values

    @add_parameter
    def _parameter_wifi_mode(
        self,
        val_mapping={"station": "0", "hotspot": "1", "": ""},
        get_cmd=InstanceAttr("_get_wifi_mode"),
        set_cmd=InstanceAttr("_set_wifi_mode"),
    ):
        """WiFi Mode: station/hotspot."""

    @add_parameter
    def _parameter_wifi_station_ssid(
        self,
        vals=validators.Strings(),
        get_cmd=InstanceAttr("_get_wifi_sta_ssid"),
        set_cmd=InstanceAttr("_set_wifi_sta_ssid"),
    ):
        """Sets network SSID for WiFi module."""

    @add_parameter
    def _parameter_wifi_station_password(
        self,
        vals=validators.Strings(),
        get_cmd=InstanceAttr("_get_wifi_sta_password"),
        set_cmd=InstanceAttr("_set_wifi_sta_password"),
    ):
        """Sets network password for WiFi module."""

    @add_parameter
    def _parameter_wifi_hotspot_ssid(
        self,
        vals=validators.Strings(),
        get_cmd=InstanceAttr("_get_wifi_ap_ssid"),
        set_cmd=InstanceAttr("_set_wifi_ap_ssid"),
    ):
        """Sets hotspot SSID for WiFi module."""

    @add_parameter
    def _parameter_wifi_hotspot_password(
        self,
        vals=validators.Strings(),
        get_cmd=InstanceAttr("_get_wifi_ap_password"),
        set_cmd=InstanceAttr("_set_wifi_ap_password"),
    ):
        """Sets hotspot password for WiFi module."""

    @add_parameter
    def _parameter_wifi_ip_address(
        self,
        vals=validators.Strings(),
        get_cmd=InstanceAttr("_get_wifi_ip_address"),
        set_cmd=InstanceAttr("_set_wifi_ip_address"),
    ):
        """Sets IP address for WiFi module."""

    @add_parameter
    def _parameter_wifi_subnet_address(
        self,
        vals=validators.Strings(),
        get_cmd=InstanceAttr("_get_wifi_subnet_address"),
        set_cmd=InstanceAttr("_set_wifi_subnet_address"),
    ):
        """Sets Subnet mask for WiFi module."""

    @add_parameter
    def _parameter_wifi_gateway_address(
        self,
        vals=validators.Strings(),
        get_cmd=InstanceAttr("_get_wifi_gateway_address"),
        set_cmd=InstanceAttr("_set_wifi_gateway_address"),
    ):
        """Sets default gateway for WiFi module."""

    # ##################################################################################
    # Public methods
    # ##################################################################################

    # Standard LO methods

    def on(self):
        """
        Turns ON the RF output.
        """
        self.status(True)

    def off(self):
        """
        Turns OFF the RF output.
        """
        self.status(False)

    # ERASynth specific methods

    def get_configuration(self, par_name: str = None) -> Union[dict, str]:
        """
        Returns the configuration JSON that contains all parameters.
        """
        config_json = json.loads(self._get_json_str("RA", "rfoutput"))
        return config_json if par_name is None else config_json[par_name]

    def get_diagnostic_status(self, par_name: str = None):
        """
        Returns the diagnostic JSON.
        """
        config_json = json.loads(self._get_json_str("RD", "temperature"))
        return config_json if par_name is None else config_json[par_name]

    def open_port(self):
        """Opens the serial communication port."""
        if self.ser.is_open:
            self.log.info("Serial port is already open.")
        else:
            self.ser.open()
            while not self.ser.is_open:
                time.sleep(50e-3)
            self.log.info("Serial port opened.")

    def preset(self):
        """
        Presets the device to known values.

        .. warning::

            After the reset the output is set to power 0.0 dBm @ 1GHz!
        """
        self._clear_read_buffer()
        self._write_cmd("PP")

    def factory_reset(self):
        """Does factory reset on the device."""
        self._clear_read_buffer()
        self._write_cmd("PR")

    def esp8266_upload_mode(self):
        """Sets the ESP8266 module in upload mode."""
        self._clear_read_buffer()
        self._write_cmd("U")

    def wifi_on(self):
        """Turn ESP8266 WiFi module on."""
        self._clear_read_buffer()
        self._write_cmd("PE01")

    def wifi_off(self):
        """Turn ESP8266 WiFi module off."""
        self._clear_read_buffer()
        self._write_cmd("PE00")

    def run_self_test(self):
        """
        Sets all settable parameters to different values.

        NB serves as self test of the instrument because setting readable parameters
        is done by setting and confirming the value.
        """
        par_values = list(_SELF_TEST_LIST)

        if isinstance(self, (ERASynthPlus, ERASynthPlusPlus)):
            # Only ERASynth+ and ERASynth++ have this functionality
            par_values += [
                ("reference_tcxo_ocxo", "tcxo"),
                ("reference_tcxo_ocxo", "ocxo"),
            ]

        num_tests = len(par_values)
        for i, (name, val) in enumerate(par_values):
            print(f"\r[{i+1:2d}/{num_tests}] Running...", end="")
            self.set(name, val)

        print("\nDone!")

    # ##################################################################################
    # Private methods
    # ##################################################################################

    def _write_cmd(self, cmd: str):
        self.ser.write(f">{cmd}\r".encode("ASCII"))

    def _set_param_and_confirm(self, cmd_id: str, par_name: str, cmd_arg: str = ""):
        """Set a parameter and reads it back until both values match."""
        sleep_before_read = 0.0
        timeout = 5
        t_start = time.time()
        cmd = f"{cmd_id}{cmd_arg}"
        while True:
            self._clear_read_buffer()
            self._write_cmd(cmd)
            time.sleep(sleep_before_read)
            value = self.get_configuration(par_name)
            if value == cmd_arg:
                break
            if time.time() > t_start + timeout:
                raise TimeoutError(f"Command {cmd!r} failed.")
            # keep increasing the wait time
            sleep_before_read += 5e-3

    def _readline(self):
        return self.ser.readline().decode("ASCII").strip()

    def _get_json_str(self, cmd: str, first_key: str):
        """
        Sends command and reads result until the result looks like a JSON.
        """
        while True:
            self._clear_read_buffer()
            self._write_cmd(cmd)
            read_str = self._readline()
            if (
                read_str.startswith('{"' + first_key) and read_str[-1] == "}"
            ):  # This way we ignore everything until we see the configuration JSON
                break
        return read_str

    def _clear_read_buffer(self):
        """
        Clears the read buffer.

        Flushing the buffer does not always seem to work (see pySerial documentation).
        Instead we just read until empty.
        """
        self.ser.read(self.ser.in_waiting)

    # ##################################################################################
    # get commands
    # ##################################################################################

    def _get_rfoutput(self):
        return self.get_configuration("rfoutput")

    def _get_pwer(self):
        return self.get_configuration("amplitude")

    def _get_frequency(self):
        return self.get_configuration("frequency")

    def _get_reference_int_ext(self):
        return self.get_configuration("reference_int_ext")

    def _get_modulation_on_off(self):
        return self.get_configuration("modulation_on_off")

    def _get_modulation_signal_waveform(self):
        return self.get_configuration("modulation_signal_waveform")

    def _get_modulation_source(self):
        return self.get_configuration("modulation_source")

    def _get_modulation_type(self):
        return self.get_configuration("modulation_type")

    def _get_modulation_freq(self):
        return self.get_configuration("modulation_freq")

    def _get_modulation_am_depth(self):
        return self.get_configuration("modulation_am_depth")

    def _get_modulation_fm_deviation(self):
        return self.get_configuration("modulation_fm_deviation")

    def _get_modulation_pulse_period(self):
        return self.get_configuration("modulation_pulse_period")

    def _get_modulation_pulse_width(self):
        return self.get_configuration("modulation_pulse_width")

    def _get_sweep_start_stop(self):
        return self.get_configuration("sweep_start_stop")

    def _get_sweep_trigger(self):
        return self.get_configuration("sweep_trigger")

    def _get_sweep_start(self):
        return self.get_configuration("sweep_start")

    def _get_sweep_stop(self):
        return self.get_configuration("sweep_stop")

    def _get_sweep_step(self):
        return self.get_configuration("sweep_step")

    def _get_sweep_dwell(self):
        return self.get_configuration("sweep_dwell")

    def _get_phase_noise_mode(self):
        return self.get_configuration("phase_noise_mode")

    def _get_wifi_mode(self):
        return self.get_configuration("wifi_mode")

    def _get_wifi_sta_ssid(self):
        return self.get_configuration("wifi_sta_ssid")

    def _get_wifi_sta_password(self):
        return self.get_configuration("wifi_sta_password")

    def _get_wifi_ap_ssid(self):
        return self.get_configuration("wifi_ap_ssid")

    def _get_wifi_ap_password(self):
        return self.get_configuration("wifi_ap_password")

    def _get_wifi_ip_address(self):
        return self.get_configuration("wifi_ip_address")

    def _get_wifi_subnet_address(self):
        return self.get_configuration("wifi_subnet_address")

    def _get_wifi_gateway_address(self):
        return self.get_configuration("wifi_gateway_address")

    def _get_temperature(self):
        return self.get_diagnostic_status("temperature")

    def _get_voltage(self):
        return self.get_diagnostic_status("voltage")

    def _get_current(self):
        return self.get_diagnostic_status("current")

    def _get_em(self):
        return self.get_diagnostic_status("em")

    def _get_rssi(self):
        return self.get_diagnostic_status("rssi")

    def _get_lock_lmx1(self):
        return self.get_diagnostic_status("lock_lmx1")

    def _get_lock_lmx2(self):
        return self.get_diagnostic_status("lock_lmx2")

    def _get_lock_xtal(self):
        return self.get_diagnostic_status("lock_xtal")

    # ##################################################################################
    # set commands
    # ##################################################################################

    def _set_rfoutput(self, value: str):
        self._set_param_and_confirm(cmd_id="P0", par_name="rfoutput", cmd_arg=value)

    def _set_power(self, value: str):
        self._set_param_and_confirm(cmd_id="A", par_name="amplitude", cmd_arg=value)

    def _set_frequency(self, value: str):
        self._set_param_and_confirm(cmd_id="F", par_name="frequency", cmd_arg=value)

    def _set_reference_int_ext(self, value: str):
        self._set_param_and_confirm(
            cmd_id="P1", par_name="reference_int_ext", cmd_arg=value
        )

    def _set_modulation_on_off(self, value: str):
        self._set_param_and_confirm(
            cmd_id="MS", par_name="modulation_on_off", cmd_arg=value
        )

    def _set_modulation_signal_waveform(self, value: str):
        self._set_param_and_confirm(
            cmd_id="M2", par_name="modulation_signal_waveform", cmd_arg=value
        )

    def _set_modulation_source(self, value: str):
        self._set_param_and_confirm(
            cmd_id="M1", par_name="modulation_source", cmd_arg=value
        )

    def _set_modulation_type(self, value: str):
        self._set_param_and_confirm(
            cmd_id="M0", par_name="modulation_type", cmd_arg=value
        )

    def _set_modulation_freq(self, value: str):
        self._set_param_and_confirm(
            cmd_id="M3", par_name="modulation_freq", cmd_arg=value
        )

    def _set_modulation_am_depth(self, value: str):
        self._set_param_and_confirm(
            cmd_id="M5", par_name="modulation_am_depth", cmd_arg=value
        )

    def _set_modulation_fm_deviation(self, value: str):
        self._set_param_and_confirm(
            cmd_id="M4", par_name="modulation_fm_deviation", cmd_arg=value
        )

    def _set_modulation_pulse_period(self, value: str):
        self._set_param_and_confirm(
            cmd_id="M6", par_name="modulation_pulse_period", cmd_arg=value
        )

    def _set_modulation_pulse_width(self, value: str):
        self._set_param_and_confirm(
            cmd_id="M7", par_name="modulation_pulse_width", cmd_arg=value
        )

    def _set_sweep_start_stop(self, value: str):
        self._set_param_and_confirm(
            cmd_id="SS", par_name="sweep_start_stop", cmd_arg=value
        )

    def _set_sweep_trigger(self, value: str):
        self._set_param_and_confirm(
            cmd_id="S0", par_name="sweep_trigger", cmd_arg=value
        )

    def _set_sweep_start(self, value: str):
        self._set_param_and_confirm(cmd_id="S1", par_name="sweep_start", cmd_arg=value)

    def _set_sweep_stop(self, value: str):
        self._set_param_and_confirm(cmd_id="S2", par_name="sweep_stop", cmd_arg=value)

    def _set_sweep_step(self, value: str):
        self._set_param_and_confirm(cmd_id="S3", par_name="sweep_step", cmd_arg=value)

    def _set_sweep_dwell(self, value: str):
        self._set_param_and_confirm(cmd_id="S4", par_name="sweep_dwell", cmd_arg=value)

    def _set_phase_noise_mode(self, value: str):
        self._set_param_and_confirm(
            cmd_id="P9", par_name="phase_noise_mode", cmd_arg=value
        )

    def _set_wifi_mode(self, value: str):
        self._set_param_and_confirm(cmd_id="PEW", par_name="wifi_mode", cmd_arg=value)

    def _set_wifi_sta_ssid(self, value: str):
        self._set_param_and_confirm(
            cmd_id="PES0", par_name="wifi_sta_ssid", cmd_arg=value
        )

    def _set_wifi_sta_password(self, value: str):
        self._set_param_and_confirm(
            cmd_id="PEP0", par_name="wifi_sta_password", cmd_arg=value
        )

    def _set_wifi_ap_ssid(self, value: str):
        self._set_param_and_confirm(
            cmd_id="PES1", par_name="wifi_ap_ssid", cmd_arg=value
        )

    def _set_wifi_ap_password(self, value: str):
        self._set_param_and_confirm(
            cmd_id="PEP1", par_name="wifi_ap_password", cmd_arg=value
        )

    def _set_wifi_ip_address(self, value: str):
        self._set_param_and_confirm(
            cmd_id="PEI", par_name="wifi_ip_address", cmd_arg=value
        )

    def _set_wifi_subnet_address(self, value: str):
        self._set_param_and_confirm(
            cmd_id="PEN", par_name="wifi_subnet_address", cmd_arg=value
        )

    def _set_wifi_gateway_address(self, value: str):
        self._set_param_and_confirm(
            cmd_id="PEG", par_name="wifi_gateway_address", cmd_arg=value
        )

    def _get_reference_tcxo_ocxo(self):
        """NB not tested if this parameter is available for <6GHz ERASynth!"""
        return self.get_configuration("reference_tcxo_ocxo")

class ERASynth(ERASynthBase):
    """
    Driver for the ERASynth model instrument.

    For ERASynth+/ERASynth++ see `EraSynthPlus`/`EraSynthPlusPlus` classes.
    """

    @add_parameter
    def _parameter_frequency(
        self,
        label="Frequency",
        unit="Hz",
        vals=validators.Numbers(min_value=250e3, max_value=6e9),
        get_cmd=InstanceAttr("_get_frequency"),
        get_parser=int,
        set_cmd=InstanceAttr("_set_frequency"),
        set_parser=lambda freq: str(int(freq)),
    ):
        """The RF Frequency in Hz."""

    @add_parameter
    def _parameter_reference_tcxo_ocxo(
        self,
        val_mapping={"tcxo": "0", "ocxo": "1"},
        get_cmd=InstanceAttr("_get_reference_tcxo_ocxo"),
    ):
        """
        NB not tested if this parameter is available for <6GHz ERASynth!
        """

class ERASynthPlus(ERASynthBase):
    """
    Driver for the ERASynth+ model instrument.

    For ERASynth/ERASynth++ see `EraSynth`/`EraSynthPlusPlus` classes.
    """


    @add_parameter
    def _parameter_frequency(
        self,
        label="Frequency",
        unit="Hz",
        vals=validators.Numbers(min_value=250e3, max_value=15e9),
        get_cmd=InstanceAttr("_get_frequency"),
        get_parser=int,
        set_cmd=InstanceAttr("_set_frequency"),
        set_parser=lambda freq: str(int(freq)),
    ):
        """The RF Frequency in Hz."""

    @add_parameter
    def _parameter_reference_tcxo_ocxo(
        self,
        val_mapping={"tcxo": "0", "ocxo": "1"},
        get_cmd=InstanceAttr("_get_reference_tcxo_ocxo"),
        set_cmd=InstanceAttr("_set_reference_tcxo_ocxo"),
    ):
        """
        """

    def _set_reference_tcxo_ocxo(self, value: str):
        self._set_param_and_confirm(
            cmd_id="P5", par_name="reference_tcxo_ocxo", cmd_arg=value
        )


class ERASynthPlusPlus(ERASynthPlus):
    """
    Driver for the ERASynth++ model instrument.

    For ERASynth/ERASynth+ see `EraSynth`/`EraSynthPlus` classes.
    """

    @add_parameter
    def _parameter_frequency(
        self,
        label="Frequency",
        unit="Hz",
        vals=validators.Numbers(min_value=250e3, max_value=20e9),
        get_cmd=InstanceAttr("_get_frequency"),
        get_parser=int,
        set_cmd=InstanceAttr("_set_frequency"),
        set_parser=lambda freq: str(int(freq)),
    ):
        """The RF Frequency in Hz."""


_SELF_TEST_LIST = [
    ("frequency", 3.3e9),
    ("modulation_am_depth", 30),
    ("modulation_fm_deviation", 1e3),
    ("modulation_freq", 2e3),
    ("modulation_pulse_period", 0.003),
    ("modulation_pulse_width", 0.002),
    ("power", 0.01),
    ("power", -0.01),
    ("sweep_dwell", 0.001),
    ("sweep_start", 2e9),
    ("sweep_step", 0.5e9),
    ("sweep_stop", 6e9),
    ("status", False),
    ("status", True),
    ("modulation_on_off", "on"),
    ("modulation_on_off", "off"),
    ("modulation_signal_waveform", "triangle"),
    ("modulation_signal_waveform", "ramp"),
    ("modulation_signal_waveform", "square"),
    ("modulation_signal_waveform", "sine"),
    ("modulation_source", "internal"),
    ("modulation_source", "external"),
    ("modulation_source", "microphone"),
    ("modulation_type", "narrowband_fm"),
    ("modulation_type", "am"),
    ("modulation_type", "pulse"),
    ("modulation_type", "wideband_fm"),
    ("ref_osc_source", "ext"),
    ("ref_osc_source", "int"),
    ("synthesizer_mode", "low_phase_noise"),
    ("synthesizer_mode", "low_spurious"),
    ("sweep", "on"),
    ("sweep", "off"),
    ("sweep_trigger", "freerun"),
    ("sweep_trigger", "external"),
    ("wifi_mode", "hotspot"),
    ("wifi_mode", "station"),
    ("wifi_station_ssid", "ERA_123"),
    ("wifi_station_ssid", "ERA"),
    ("wifi_station_password", "era1234"),
    ("wifi_station_password", "era19050"),
    ("wifi_hotspot_ssid", "ERA"),
    ("wifi_hotspot_ssid", "ERASynth"),
    ("wifi_hotspot_password", "erainstruments+"),
    ("wifi_hotspot_password", "erainstruments"),
    ("wifi_ip_address", "192.168.001.151"),
    ("wifi_ip_address", "192.168.001.150"),
    ("wifi_gateway_address", "192.168.001.002"),
    ("wifi_gateway_address", "192.168.001.001"),
    ("wifi_subnet_address", "255.255.255.001"),
    ("wifi_subnet_address", "255.255.255.000"),
]
"""
A list of `Tuple[<parameter_name, value>]` used for a self-test of the instrument.
It is intended to check that read/write parameters are set correctly.
"""
