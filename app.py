import app
import asyncio
import neopixel
import requests
import time
from math import pi
from micropython import const

from app_components import clear_background, Menu, Notification
from app_components.tokens import colors
from events.input import Buttons, BUTTON_TYPES
from system.hexpansion.config import HexpansionConfig
from system.scheduler.events import RequestForegroundPushEvent
from system.eventbus import eventbus

APP_VERSION = "1.0.0"
# Keep in sync with the folder path in emfcamp/hexpansion-firmwares
VERSION_URL = (
    "https://raw.githubusercontent.com/emfcamp/hexpansion-firmwares/"
    "main/0xcafe/0x0c02/VERSION"
)

# LED pin (HS_F)
HS_F = 0

# Rough indoor air-quality bands (ppm)
GOOD_PPM = 800
OK_PPM = 1200
POOR_PPM = 2000

menu_items = ["1", "2", "3", "4", "5", "6"]

SCD4X_DEFAULT_ADDR = 0x62
_REINIT = const(0x3646)
_DATAREADY = const(0xE4B8)
_STOPPERIODICMEASUREMENT = const(0x3F86)
_STARTPERIODICMEASUREMENT = const(0x21B1)
_STARTLOWPOWERPERIODICMEASUREMENT = const(0x21AC)
_READMEASUREMENT = const(0xEC05)
_SERIALNUMBER = const(0x3682)


class SCD4X:
    def __init__(self, i2c, address=SCD4X_DEFAULT_ADDR):
        self.address = address
        self.i2c = i2c
        self._buffer = bytearray(18)
        self._cmd = bytearray(2)
        self._crc_buffer = bytearray(2)
        self._temperature = None
        self._relative_humidity = None
        self._co2 = None
        self.stop_periodic_measurement()

    @property
    def CO2(self):
        if self.data_ready:
            self._read_data()
        return self._co2

    @property
    def temperature(self):
        if self.data_ready:
            self._read_data()
        return self._temperature

    @property
    def relative_humidity(self):
        if self.data_ready:
            self._read_data()
        return self._relative_humidity

    def _read_data(self):
        self._send_command(_READMEASUREMENT, cmd_delay=0.001)
        self._read_reply(9)
        self._co2 = (self._buffer[0] << 8) | self._buffer[1]
        temp = (self._buffer[3] << 8) | self._buffer[4]
        self._temperature = -45 + 175 * (temp / 2**16)
        humi = (self._buffer[6] << 8) | self._buffer[7]
        self._relative_humidity = 100 * (humi / 2**16)

    @property
    def data_ready(self):
        self._send_command(_DATAREADY, cmd_delay=0.001)
        self._read_reply(3)
        return not ((self._buffer[0] & 0x07 == 0) and (self._buffer[1] == 0))

    @property
    def serial_number(self):
        self._send_command(_SERIALNUMBER, cmd_delay=0.001)
        self._read_reply(9)
        return (
            self._buffer[0],
            self._buffer[1],
            self._buffer[3],
            self._buffer[4],
            self._buffer[6],
            self._buffer[7],
        )

    def stop_periodic_measurement(self):
        self._send_command(_STOPPERIODICMEASUREMENT, cmd_delay=0.5)

    def start_periodic_measurement(self):
        self._send_command(_STARTPERIODICMEASUREMENT)

    def start_low_periodic_measurement(self):
        self._send_command(_STARTLOWPOWERPERIODICMEASUREMENT)

    def reinit(self):
        self.stop_periodic_measurement()
        self._send_command(_REINIT, cmd_delay=0.02)

    def _check_buffer_crc(self, buf):
        for i in range(0, len(buf), 3):
            self._crc_buffer[0] = buf[i]
            self._crc_buffer[1] = buf[i + 1]
            if self._crc8(self._crc_buffer) != buf[i + 2]:
                raise RuntimeError("CRC check failed while reading data")
        return True

    def _send_command(self, cmd, cmd_delay=0):
        self._cmd[0] = (cmd >> 8) & 0xFF
        self._cmd[1] = cmd & 0xFF
        self.i2c.writeto(self.address, self._cmd)
        if cmd_delay:
            time.sleep(cmd_delay)

    def _read_reply(self, num):
        self.i2c.readfrom_into(self.address, self._buffer)
        self._check_buffer_crc(self._buffer[0:num])

    @staticmethod
    def _crc8(buffer):
        crc = 0xFF
        for byte in buffer:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc = crc << 1
        return crc & 0xFF


def co2_color(ppm):
    if ppm is None:
        return (0.5, 0.5, 0.5), (40, 40, 40)
    if ppm < GOOD_PPM:
        return (0.2, 0.9, 0.3), (0, 180, 40)
    if ppm < OK_PPM:
        return (0.95, 0.85, 0.15), (180, 140, 0)
    if ppm < POOR_PPM:
        return (1.0, 0.45, 0.1), (220, 80, 0)
    return (1.0, 0.15, 0.1), (255, 20, 0)


def co2_label(ppm):
    if ppm is None:
        return "waiting…"
    if ppm < GOOD_PPM:
        return "good"
    if ppm < OK_PPM:
        return "ok"
    return "poor"


def _version_tuple(version):
    parts = []
    for part in version.strip().lstrip("v").split("."):
        try:
            parts.append(int(part))
        except ValueError:
            parts.append(0)
    return tuple(parts)


def _is_newer(remote, local):
    return _version_tuple(remote) > _version_tuple(local)


class CanaryApp(app.App):
    def __init__(self, config=None):
        super().__init__()
        self.button_states = Buttons(self)
        self.hexpansion_config = None
        self.menu = Menu(
            self,
            menu_items,
            select_handler=self.select_handler,
            back_handler=self.back_handler,
        )
        self.sensor = None
        self.leds = None
        self.brightness = 100  # LED brightness, percent (10..100)
        self.co2 = None
        self.temperature = None
        self.humidity = None
        self.status = "Select port"
        self.error = None
        self.foregrounded = False
        self._poll_ms = 0
        self.update_available = False
        self.latest_version = None
        self.notification = None

    def _check_for_update(self):
        try:
            resp = requests.get(VERSION_URL)
            if getattr(resp, "status_code", 200) != 200:
                return
            remote = resp.text.strip().split()[0]
            if remote and _is_newer(remote, APP_VERSION):
                self.latest_version = remote
                self.update_available = True
                self.notification = Notification(
                    "Please update via Hexpansions"
                )
        except Exception:
            pass

    async def background_task(self):
        await asyncio.sleep(3)
        self._check_for_update()
        while True:
            await asyncio.sleep(3600)
            self._check_for_update()

    def select_handler(self, item, idx):
        self.hexpansion_config = HexpansionConfig(idx + 1)
        self._setup_hardware(self.hexpansion_config)

    def back_handler(self):
        if self.sensor is not None or self.leds is not None:
            self._cleanup_hardware()
            self.hexpansion_config = None
            self.status = "Select port"
            self.error = None
            self.co2 = None
            self.temperature = None
            self.humidity = None
        else:
            self.minimise()

    def _setup_hardware(self, config):
        self.error = None
        self.status = "Starting…"
        self.co2 = None
        self.temperature = None
        self.humidity = None

        try:
            # SK6805 is WS2812-compatible; one LED on HS_F
            self.leds = neopixel.NeoPixel(config.pin[HS_F], 1)
            scale = self.brightness / 100
            self.leds[0] = tuple(int(20 * scale) for _ in range(3))
            self.leds.write()
        except Exception as e:
            self.leds = None
            self.error = "LED: {}".format(e)
            self.status = "LED failed"
            return

        try:
            i2c = config.i2c
            addrs = i2c.scan()
            if SCD4X_DEFAULT_ADDR not in addrs:
                self.error = "No SCD41 at 0x62"
                self.status = "scan: {}".format(
                    " ".join("0x{:02x}".format(a) for a in addrs) or "empty"
                )
                return

            self.sensor = SCD4X(i2c)
            self.sensor.start_periodic_measurement()
            self.status = "warming up…"
        except Exception as e:
            self.sensor = None
            self.error = str(e)
            self.status = "Sensor failed"

    def _cleanup_hardware(self):
        if self.sensor is not None:
            try:
                self.sensor.stop_periodic_measurement()
            except Exception:
                pass
            self.sensor = None

        if self.leds is not None:
            try:
                self.leds[0] = (0, 0, 0)
                self.leds.write()
            except Exception:
                pass
            self.leds = None

    def deinit(self):
        self._cleanup_hardware()

    def update(self, delta):
        if not self.foregrounded:
            eventbus.emit(RequestForegroundPushEvent(self))
            self.foregrounded = True

        if self.notification:
            self.notification.update(delta)

        if self.hexpansion_config is None:
            self.menu.update(delta)
            return

        if self.button_states.get(BUTTON_TYPES["CANCEL"]):
            self.button_states.clear()
            self.back_handler()
            return

        # B button steps LED brightness in 10% increments
        if self.button_states.get(BUTTON_TYPES["RIGHT"]):
            self.button_states.clear()
            self._step_brightness()

        if self.sensor is None:
            return

        self._poll_ms += delta
        if self._poll_ms < 500:
            return
        self._poll_ms = 0

        try:
            if self.sensor.data_ready:
                self.co2 = self.sensor.CO2
                self.temperature = self.sensor.temperature
                self.humidity = self.sensor.relative_humidity
                self.status = co2_label(self.co2)
                self.error = None
                self._update_led()
        except Exception as e:
            self.error = str(e)
            self.status = "Read error"

    def _step_brightness(self):
        self.brightness += 10
        if self.brightness > 100:
            self.brightness = 0
        self._update_led()

    def _update_led(self):
        if self.leds is None:
            return
        _, led_rgb = co2_color(self.co2)
        scale = self.brightness / 100
        self.leds[0] = tuple(int(c * scale) for c in led_rgb)
        self.leds.write()

    def draw(self, ctx):
        clear_background(ctx)

        if self.hexpansion_config is None:
            self.menu.draw(ctx)
            ctx.save()
            ctx.font_size = 22
            ctx.text_align = ctx.CENTER
            ctx.rgb(*colors["dark_green"]).rectangle(-120, -120, 240, 100).fill()
            ctx.rgb(*colors["dark_green"]).rectangle(-120, 20, 240, 100).fill()
            rotation_angle = (self.menu.position - 1) * pi / 3
            ctx.rgb(*colors["mid_green"]).rotate(rotation_angle).rectangle(
                80, -120, 40, 240
            ).fill()
            ctx.rgb(1, 1, 1).rotate(-rotation_angle).move_to(0, -45).text(
                "Select hexpansion port:"
            )
            ctx.restore()
            if self.notification:
                self.notification.draw(ctx)
            return

        display_rgb, _ = co2_color(self.co2)
        ctx.save()
        ctx.rgb(0.05, 0.08, 0.06).rectangle(-120, -120, 240, 240).fill()

        ctx.text_align = ctx.CENTER
        ctx.font_size = 18
        ctx.rgb(0.6, 0.7, 0.6).move_to(0, -90).text("Canary")

        ctx.font_size = 48
        ctx.rgb(*display_rgb)
        if self.co2 is not None:
            ctx.move_to(0, -10).text(str(self.co2))
            ctx.font_size = 20
            ctx.move_to(0, 28).text("ppm CO2")
        else:
            ctx.font_size = 22
            ctx.move_to(0, 0).text(self.status)

        ctx.font_size = 16
        ctx.rgb(0.55, 0.6, 0.55)
        if self.co2 is not None and self.temperature is not None:
            ctx.move_to(0, 58).text(
                "{:.1f}C  {:.0f}%RH".format(self.temperature, self.humidity or 0)
            )
        elif self.error:
            ctx.rgb(1, 0.4, 0.3).move_to(0, 58).text(self.error)
        else:
            ctx.move_to(0, 58).text(self.status)

        ctx.font_size = 14
        if self.update_available:
            ctx.rgb(1.0, 0.75, 0.2).move_to(0, 85).text("Please update")
            ctx.rgb(0.4, 0.45, 0.4).move_to(0, 102).text(
                "Hexpansions app ({})".format(self.latest_version or "?")
            )
        else:
            ctx.rgb(0.4, 0.45, 0.4).move_to(0, 90).text(
                "port {}  v{}  LED {}%".format(
                    self.hexpansion_config.port, APP_VERSION, self.brightness
                )
            )
        ctx.restore()

        if self.notification:
            self.notification.draw(ctx)


__app_export__ = CanaryApp
