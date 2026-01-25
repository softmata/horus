"""
HORUS Hardware Nodes - Python implementations of common robotics nodes.

This module provides Python-native implementations of hardware interface nodes
using popular Python libraries. These nodes integrate seamlessly with the
HORUS pub/sub system.

Available Nodes:
    - SerialNode: Serial/UART communication (pyserial)
    - JoystickNode: Gamepad/joystick input (pygame)
    - KeyboardNode: Keyboard input capture (pynput)
    - ImuNode: IMU sensor reading (smbus2 for I2C IMUs)
    - GpsNode: GPS/GNSS position data (pynmea2)
    - CameraNode: Camera capture (opencv-python)

Example:
    from horus import Scheduler
    from horus.nodes import SerialNode, CameraNode

    serial = SerialNode(port="/dev/ttyUSB0", baudrate=115200)
    camera = CameraNode(device_id=0, width=640, height=480)

    scheduler = Scheduler()
    scheduler.add(serial, order=0, logging=True)
    scheduler.add(camera, order=1, logging=True)
    scheduler.run()
"""

from typing import Optional, Dict, Any, List, Callable, Tuple
import time
import threading
from dataclasses import dataclass, field

# Import base Node class
from horus import Node

# Optional imports - gracefully handle missing dependencies
_HAS_SERIAL = False
_HAS_PYGAME = False
_HAS_PYNPUT = False
_HAS_SMBUS = False
_HAS_PYNMEA = False
_HAS_CV2 = False

try:
    import serial
    _HAS_SERIAL = True
except ImportError:
    pass

try:
    import pygame
    _HAS_PYGAME = True
except ImportError:
    pass

try:
    from pynput import keyboard
    _HAS_PYNPUT = True
except ImportError:
    pass

try:
    import smbus2
    _HAS_SMBUS = True
except ImportError:
    pass

try:
    import pynmea2
    _HAS_PYNMEA = True
except ImportError:
    pass

try:
    import cv2
    import numpy as np
    _HAS_CV2 = True
except ImportError:
    pass

_HAS_RPLIDAR = False
try:
    from rplidar import RPLidar
    _HAS_RPLIDAR = True
except ImportError:
    pass


# =============================================================================
# Data Classes for Node Messages
# =============================================================================

@dataclass
class SerialData:
    """Serial data message."""
    port: str
    data: bytes
    timestamp: float = field(default_factory=time.time)

    # Parity constants
    PARITY_NONE = 0
    PARITY_ODD = 1
    PARITY_EVEN = 2


@dataclass
class JoystickState:
    """Joystick/gamepad state."""
    axes: List[float] = field(default_factory=list)
    buttons: List[bool] = field(default_factory=list)
    hats: List[Tuple[int, int]] = field(default_factory=list)
    device_id: int = 0
    device_name: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class KeyboardState:
    """Keyboard input state."""
    key: str = ""
    keycode: int = 0
    pressed: bool = False
    modifiers: Dict[str, bool] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)


@dataclass
class ImuData:
    """IMU sensor data."""
    # Accelerometer (m/s^2)
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    # Gyroscope (rad/s)
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    # Magnetometer (uT) - optional
    mag_x: float = 0.0
    mag_y: float = 0.0
    mag_z: float = 0.0
    # Temperature (Celsius) - optional
    temperature: float = 0.0
    # Metadata
    frame_id: str = "imu_link"
    timestamp: float = field(default_factory=time.time)


@dataclass
class GpsData:
    """GPS/GNSS position data."""
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    # Fix quality
    fix_type: int = 0  # 0=no fix, 1=GPS, 2=DGPS, etc.
    satellites: int = 0
    hdop: float = 99.0
    # Velocity (m/s) - optional
    speed: float = 0.0
    course: float = 0.0
    # Metadata
    frame_id: str = "gps"
    timestamp: float = field(default_factory=time.time)

    def has_fix(self) -> bool:
        return self.fix_type > 0 and self.satellites >= 4


@dataclass
class ImageData:
    """Camera image data."""
    data: bytes = b""
    width: int = 0
    height: int = 0
    encoding: str = "bgr8"
    step: int = 0  # Row stride in bytes
    frame_id: str = "camera"
    timestamp: float = field(default_factory=time.time)


@dataclass
class LaserScan:
    """2D LIDAR scan data (ROS-compatible format)."""
    ranges: List[float] = field(default_factory=list)
    intensities: List[float] = field(default_factory=list)
    angle_min: float = 0.0
    angle_max: float = 6.283185307  # 2*pi
    angle_increment: float = 0.017453293  # ~1 degree
    time_increment: float = 0.0
    scan_time: float = 0.0
    range_min: float = 0.1
    range_max: float = 12.0
    frame_id: str = "laser"
    timestamp: float = field(default_factory=time.time)


# =============================================================================
# SerialNode - Serial/UART Communication
# =============================================================================

class SerialNode(Node):
    """
    Serial/UART Communication Node.

    Handles serial communication with devices like Arduino, GPS modules,
    motor controllers, and other serial peripherals.

    Requires: pyserial (`pip install pyserial`)

    Topics:
        - Publishes to: {topic_prefix}.rx (received data)
        - Subscribes to: {topic_prefix}.tx (data to transmit)

    Example:
        serial = SerialNode(
            port="/dev/ttyUSB0",
            baudrate=115200,
            topic_prefix="serial"
        )

        # In another node, send data:
        node.send("serial.tx", b"Hello Arduino!")

        # Receive data:
        if node.has_msg("serial.rx"):
            data = node.get("serial.rx")
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 9600,
        topic_prefix: str = "serial",
        bytesize: int = 8,
        parity: str = 'N',
        stopbits: float = 1,
        timeout: float = 0.1,
        simulation: bool = False,
        name: Optional[str] = None,
        rate: float = 100,
    ):
        """
        Create a serial communication node.

        Args:
            port: Serial port path (e.g., "/dev/ttyUSB0", "COM3")
            baudrate: Baud rate (e.g., 9600, 115200)
            topic_prefix: Prefix for pub/sub topics
            bytesize: Data bits (5, 6, 7, or 8)
            parity: Parity ('N'=none, 'E'=even, 'O'=odd)
            stopbits: Stop bits (1, 1.5, or 2)
            timeout: Read timeout in seconds
            simulation: Run in simulation mode (for prototyping without hardware)
            name: Node name
            rate: Tick rate in Hz
        """
        self.port = port
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self.timeout = timeout
        self.simulation = simulation or not _HAS_SERIAL

        self._serial: Optional['serial.Serial'] = None
        self._rx_buffer = bytearray()
        self._bytes_received = 0
        self._bytes_transmitted = 0

        rx_topic = f"{topic_prefix}.rx"
        tx_topic = f"{topic_prefix}.tx"

        super().__init__(
            name=name or f"serial_{port.replace('/', '_')}",
            pubs=[rx_topic],
            subs=[tx_topic],
            tick=self._tick,
            init=self._init,
            shutdown=self._shutdown,
            rate=rate,
        )

        self._rx_topic = rx_topic
        self._tx_topic = tx_topic

    def _init(self, node: 'SerialNode') -> None:
        """Initialize serial port."""
        if self.simulation:
            self.log_info(f"SerialNode running in simulation mode (prototyping)")
            return

        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=self.bytesize,
                parity=self.parity,
                stopbits=self.stopbits,
                timeout=self.timeout,
            )
            self.log_info(f"Opened serial port {self.port} at {self.baudrate} baud")
        except Exception as e:
            self.log_error(f"Failed to open serial port: {e}")
            self.simulation = True

    def _shutdown(self, node: 'SerialNode') -> None:
        """Close serial port."""
        if self._serial and self._serial.is_open:
            self._serial.close()
            self.log_info(f"Closed serial port {self.port}")

    def _tick(self, node: 'SerialNode') -> None:
        """Process serial I/O."""
        # Handle TX (data to send)
        while self.has_msg(self._tx_topic):
            data = self.get(self._tx_topic)
            if data is not None:
                self._transmit(data)

        # Handle RX (incoming data)
        self._receive()

    def _transmit(self, data: Any) -> bool:
        """Transmit data over serial."""
        if isinstance(data, str):
            data = data.encode('utf-8')
        elif isinstance(data, SerialData):
            data = data.data
        elif not isinstance(data, (bytes, bytearray)):
            data = str(data).encode('utf-8')

        if self.simulation:
            self._bytes_transmitted += len(data)
            return True

        if self._serial and self._serial.is_open:
            try:
                written = self._serial.write(data)
                self._bytes_transmitted += written
                return True
            except Exception as e:
                self.log_error(f"Serial write error: {e}")
        return False

    def _receive(self) -> None:
        """Receive data from serial port."""
        if self.simulation:
            return

        if self._serial and self._serial.is_open:
            try:
                if self._serial.in_waiting > 0:
                    data = self._serial.read(self._serial.in_waiting)
                    if data:
                        self._bytes_received += len(data)
                        msg = SerialData(
                            port=self.port,
                            data=data,
                            timestamp=time.time(),
                        )
                        self.send(self._rx_topic, msg)
            except Exception as e:
                self.log_error(f"Serial read error: {e}")

    def write(self, data: bytes) -> bool:
        """Direct write to serial port (bypasses pub/sub)."""
        return self._transmit(data)

    def readline(self) -> Optional[bytes]:
        """Read a line from serial port (bypasses pub/sub)."""
        if self.simulation or not self._serial:
            return None
        try:
            return self._serial.readline()
        except Exception:
            return None

    @property
    def bytes_received(self) -> int:
        return self._bytes_received

    @property
    def bytes_transmitted(self) -> int:
        return self._bytes_transmitted


# =============================================================================
# JoystickNode - Gamepad/Joystick Input
# =============================================================================

class JoystickNode(Node):
    """
    Joystick/Gamepad Input Node.

    Captures input from gamepads, joysticks, and game controllers.
    Supports Xbox, PlayStation, and generic controllers.

    Requires: pygame (`pip install pygame`)

    Topics:
        - Publishes to: {topic_prefix}.state (JoystickState)
        - Publishes to: {topic_prefix}.axes (list of axis values)
        - Publishes to: {topic_prefix}.buttons (list of button states)

    Example:
        joystick = JoystickNode(device_id=0, topic_prefix="joy")

        # Read joystick state
        if node.has_msg("joy.state"):
            state = node.get("joy.state")
            print(f"Left stick: {state.axes[0]}, {state.axes[1]}")
    """

    def __init__(
        self,
        device_id: int = 0,
        topic_prefix: str = "joystick",
        deadzone: float = 0.1,
        simulation: bool = False,
        name: Optional[str] = None,
        rate: float = 60,
    ):
        """
        Create a joystick input node.

        Args:
            device_id: Joystick device index (0 for first controller)
            topic_prefix: Prefix for pub/sub topics
            deadzone: Axis deadzone (0.0 to 1.0)
            simulation: Run in simulation mode
            name: Node name
            rate: Tick rate in Hz
        """
        self.device_id = device_id
        self.deadzone = deadzone
        self.simulation = simulation or not _HAS_PYGAME

        self._joystick = None
        self._initialized = False

        state_topic = f"{topic_prefix}.state"
        axes_topic = f"{topic_prefix}.axes"
        buttons_topic = f"{topic_prefix}.buttons"

        super().__init__(
            name=name or f"joystick_{device_id}",
            pubs=[state_topic, axes_topic, buttons_topic],
            tick=self._tick,
            init=self._init,
            shutdown=self._shutdown,
            rate=rate,
        )

        self._state_topic = state_topic
        self._axes_topic = axes_topic
        self._buttons_topic = buttons_topic

    def _init(self, node: 'JoystickNode') -> None:
        """Initialize pygame and joystick."""
        if self.simulation:
            self.log_info("JoystickNode running in simulation mode")
            return

        try:
            pygame.init()
            pygame.joystick.init()

            if pygame.joystick.get_count() > self.device_id:
                self._joystick = pygame.joystick.Joystick(self.device_id)
                self._joystick.init()
                self._initialized = True
                self.log_info(f"Initialized joystick: {self._joystick.get_name()}")
            else:
                self.log_warning(f"No joystick found at index {self.device_id}")
                self.simulation = True
        except Exception as e:
            self.log_error(f"Failed to initialize joystick: {e}")
            self.simulation = True

    def _shutdown(self, node: 'JoystickNode') -> None:
        """Cleanup pygame."""
        if self._initialized:
            pygame.joystick.quit()
            pygame.quit()

    def _tick(self, node: 'JoystickNode') -> None:
        """Poll joystick state."""
        if self.simulation:
            # Publish zero state in simulation
            state = JoystickState(
                axes=[0.0] * 6,
                buttons=[False] * 12,
                hats=[(0, 0)],
                device_id=self.device_id,
                device_name="Simulated Joystick",
            )
            self.send(self._state_topic, state)
            return

        if not self._initialized or not self._joystick:
            return

        # Process pygame events
        pygame.event.pump()

        # Read axes with deadzone
        num_axes = self._joystick.get_numaxes()
        axes = []
        for i in range(num_axes):
            value = self._joystick.get_axis(i)
            if abs(value) < self.deadzone:
                value = 0.0
            axes.append(value)

        # Read buttons
        num_buttons = self._joystick.get_numbuttons()
        buttons = [self._joystick.get_button(i) for i in range(num_buttons)]

        # Read hats (D-pad)
        num_hats = self._joystick.get_numhats()
        hats = [self._joystick.get_hat(i) for i in range(num_hats)]

        # Create and publish state
        state = JoystickState(
            axes=axes,
            buttons=buttons,
            hats=hats,
            device_id=self.device_id,
            device_name=self._joystick.get_name(),
        )

        self.send(self._state_topic, state)
        self.send(self._axes_topic, axes)
        self.send(self._buttons_topic, buttons)


# =============================================================================
# KeyboardNode - Keyboard Input Capture
# =============================================================================

class KeyboardNode(Node):
    """
    Keyboard Input Capture Node.

    Captures keyboard input events (key press/release).
    Runs a background listener thread.

    Requires: pynput (`pip install pynput`)

    Topics:
        - Publishes to: {topic_prefix}.events (KeyboardState for each event)
        - Publishes to: {topic_prefix}.pressed (currently pressed keys)

    Example:
        keyboard = KeyboardNode(topic_prefix="keyboard")

        # Check for key events
        if node.has_msg("keyboard.events"):
            event = node.get("keyboard.events")
            if event.key == 'w' and event.pressed:
                print("W key pressed!")
    """

    def __init__(
        self,
        topic_prefix: str = "keyboard",
        simulation: bool = False,
        name: Optional[str] = None,
        rate: float = 60,
    ):
        """
        Create a keyboard input node.

        Args:
            topic_prefix: Prefix for pub/sub topics
            simulation: Run in simulation mode
            name: Node name
            rate: Tick rate in Hz
        """
        self.simulation = simulation or not _HAS_PYNPUT

        self._listener = None
        self._pressed_keys: Dict[str, bool] = {}
        self._events: List[KeyboardState] = []
        self._lock = threading.Lock()

        events_topic = f"{topic_prefix}.events"
        pressed_topic = f"{topic_prefix}.pressed"

        super().__init__(
            name=name or "keyboard_input",
            pubs=[events_topic, pressed_topic],
            tick=self._tick,
            init=self._init,
            shutdown=self._shutdown,
            rate=rate,
        )

        self._events_topic = events_topic
        self._pressed_topic = pressed_topic

    def _init(self, node: 'KeyboardNode') -> None:
        """Start keyboard listener."""
        if self.simulation:
            self.log_info("KeyboardNode running in simulation mode")
            return

        try:
            self._listener = keyboard.Listener(
                on_press=self._on_press,
                on_release=self._on_release,
            )
            self._listener.start()
            self.log_info("Keyboard listener started")
        except Exception as e:
            self.log_error(f"Failed to start keyboard listener: {e}")
            self.simulation = True

    def _shutdown(self, node: 'KeyboardNode') -> None:
        """Stop keyboard listener."""
        if self._listener:
            self._listener.stop()

    def _on_press(self, key) -> None:
        """Handle key press event."""
        try:
            key_str = key.char if hasattr(key, 'char') and key.char else str(key)
        except AttributeError:
            key_str = str(key)

        # Get keycode
        keycode = getattr(key, 'vk', 0) if hasattr(key, 'vk') else ord(key_str[0]) if len(key_str) == 1 else 0

        event = KeyboardState(
            key=key_str,
            keycode=keycode,
            pressed=True,
        )

        with self._lock:
            self._pressed_keys[key_str] = True
            self._events.append(event)

    def _on_release(self, key) -> None:
        """Handle key release event."""
        try:
            key_str = key.char if hasattr(key, 'char') and key.char else str(key)
        except AttributeError:
            key_str = str(key)

        keycode = getattr(key, 'vk', 0) if hasattr(key, 'vk') else ord(key_str[0]) if len(key_str) == 1 else 0

        event = KeyboardState(
            key=key_str,
            keycode=keycode,
            pressed=False,
        )

        with self._lock:
            self._pressed_keys.pop(key_str, None)
            self._events.append(event)

    def _tick(self, node: 'KeyboardNode') -> None:
        """Publish accumulated keyboard events."""
        if self.simulation:
            return

        with self._lock:
            # Publish all pending events
            for event in self._events:
                self.send(self._events_topic, event)
            self._events.clear()

            # Publish currently pressed keys
            self.send(self._pressed_topic, list(self._pressed_keys.keys()))


# =============================================================================
# ImuNode - Inertial Measurement Unit
# =============================================================================

class ImuNode(Node):
    """
    IMU (Inertial Measurement Unit) Sensor Node.

    Reads accelerometer, gyroscope, and optionally magnetometer data
    from I2C-connected IMU sensors.

    Requires: smbus2 (`pip install smbus2`)

    Supported sensors:
        - MPU6050 (6-axis: accel + gyro)
        - MPU9250 (9-axis: accel + gyro + mag)
        - BMI160, ICM20948, etc. (with custom register maps)

    Topics:
        - Publishes to: {topic_prefix} (ImuData)

    Example:
        imu = ImuNode(
            i2c_bus=1,
            i2c_address=0x68,  # MPU6050 default
            topic_prefix="imu"
        )
    """

    # MPU6050 register addresses
    MPU6050_PWR_MGMT_1 = 0x6B
    MPU6050_ACCEL_XOUT_H = 0x3B
    MPU6050_GYRO_XOUT_H = 0x43
    MPU6050_TEMP_OUT_H = 0x41

    def __init__(
        self,
        i2c_bus: int = 1,
        i2c_address: int = 0x68,
        topic_prefix: str = "imu",
        frame_id: str = "imu_link",
        accel_scale: float = 16384.0,  # LSB/g for ±2g range
        gyro_scale: float = 131.0,      # LSB/(°/s) for ±250°/s range
        simulation: bool = False,
        name: Optional[str] = None,
        rate: float = 100,
    ):
        """
        Create an IMU sensor node.

        Args:
            i2c_bus: I2C bus number (usually 1 on Raspberry Pi)
            i2c_address: I2C device address (0x68 for MPU6050)
            topic_prefix: Prefix for pub/sub topics
            frame_id: Coordinate frame ID
            accel_scale: Accelerometer scale factor
            gyro_scale: Gyroscope scale factor
            simulation: Run in simulation mode
            name: Node name
            rate: Sample rate in Hz
        """
        self.i2c_bus = i2c_bus
        self.i2c_address = i2c_address
        self.frame_id = frame_id
        self.accel_scale = accel_scale
        self.gyro_scale = gyro_scale
        self.simulation = simulation or not _HAS_SMBUS

        self._bus = None
        self._sample_count = 0
        self._sim_angle = 0.0

        super().__init__(
            name=name or f"imu_{i2c_address:02x}",
            pubs=[topic_prefix],
            tick=self._tick,
            init=self._init,
            shutdown=self._shutdown,
            rate=rate,
        )

        self._topic = topic_prefix

    def _init(self, node: 'ImuNode') -> None:
        """Initialize I2C and wake up IMU."""
        if self.simulation:
            self.log_info("ImuNode running in simulation mode")
            return

        try:
            self._bus = smbus2.SMBus(self.i2c_bus)
            # Wake up MPU6050 (clear sleep bit)
            self._bus.write_byte_data(self.i2c_address, self.MPU6050_PWR_MGMT_1, 0)
            time.sleep(0.1)  # Wait for sensor to stabilize
            self.log_info(f"IMU initialized on I2C bus {self.i2c_bus}, address 0x{self.i2c_address:02x}")
        except Exception as e:
            self.log_error(f"Failed to initialize IMU: {e}")
            self.simulation = True

    def _shutdown(self, node: 'ImuNode') -> None:
        """Close I2C bus."""
        if self._bus:
            self._bus.close()

    def _tick(self, node: 'ImuNode') -> None:
        """Read and publish IMU data."""
        if self.simulation:
            self._publish_simulated_data()
            return

        try:
            data = self._read_sensor_data()
            self.send(self._topic, data)
            self._sample_count += 1
        except Exception as e:
            self.log_error(f"IMU read error: {e}")

    def _read_sensor_data(self) -> ImuData:
        """Read raw sensor data from MPU6050."""
        # Read accelerometer (6 bytes)
        accel_data = self._bus.read_i2c_block_data(
            self.i2c_address, self.MPU6050_ACCEL_XOUT_H, 6
        )
        ax = self._to_signed_16bit(accel_data[0], accel_data[1]) / self.accel_scale * 9.81
        ay = self._to_signed_16bit(accel_data[2], accel_data[3]) / self.accel_scale * 9.81
        az = self._to_signed_16bit(accel_data[4], accel_data[5]) / self.accel_scale * 9.81

        # Read gyroscope (6 bytes)
        gyro_data = self._bus.read_i2c_block_data(
            self.i2c_address, self.MPU6050_GYRO_XOUT_H, 6
        )
        gx = self._to_signed_16bit(gyro_data[0], gyro_data[1]) / self.gyro_scale * 0.0174533  # deg/s to rad/s
        gy = self._to_signed_16bit(gyro_data[2], gyro_data[3]) / self.gyro_scale * 0.0174533
        gz = self._to_signed_16bit(gyro_data[4], gyro_data[5]) / self.gyro_scale * 0.0174533

        # Read temperature
        temp_data = self._bus.read_i2c_block_data(
            self.i2c_address, self.MPU6050_TEMP_OUT_H, 2
        )
        temp = self._to_signed_16bit(temp_data[0], temp_data[1]) / 340.0 + 36.53

        return ImuData(
            accel_x=ax, accel_y=ay, accel_z=az,
            gyro_x=gx, gyro_y=gy, gyro_z=gz,
            temperature=temp,
            frame_id=self.frame_id,
        )

    def _publish_simulated_data(self) -> None:
        """Generate simulated IMU data."""
        import math
        self._sim_angle += 0.01

        data = ImuData(
            accel_x=0.0,
            accel_y=0.0,
            accel_z=9.81,  # Gravity
            gyro_x=0.01 * math.sin(self._sim_angle),
            gyro_y=0.01 * math.cos(self._sim_angle),
            gyro_z=0.0,
            temperature=25.0,
            frame_id=self.frame_id,
        )
        self.send(self._topic, data)

    @staticmethod
    def _to_signed_16bit(high: int, low: int) -> int:
        """Convert two bytes to signed 16-bit integer."""
        value = (high << 8) | low
        if value >= 0x8000:
            value -= 0x10000
        return value


# =============================================================================
# GpsNode - GPS/GNSS Position
# =============================================================================

class GpsNode(Node):
    """
    GPS/GNSS Position Node.

    Reads GPS data from NMEA-compatible GPS receivers via serial port.
    Parses standard NMEA sentences (GGA, RMC, VTG).

    Requires: pyserial, pynmea2 (`pip install pyserial pynmea2`)

    Topics:
        - Publishes to: {topic_prefix}.fix (GpsData with position)

    Example:
        gps = GpsNode(
            port="/dev/ttyUSB0",
            baudrate=9600,
            topic_prefix="gps"
        )
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 9600,
        topic_prefix: str = "gps",
        frame_id: str = "gps",
        simulation: bool = False,
        name: Optional[str] = None,
        rate: float = 10,
    ):
        """
        Create a GPS sensor node.

        Args:
            port: Serial port path
            baudrate: Serial baud rate (usually 9600 for GPS)
            topic_prefix: Prefix for pub/sub topics
            frame_id: Coordinate frame ID
            simulation: Run in simulation mode
            name: Node name
            rate: Tick rate in Hz
        """
        self.port = port
        self.baudrate = baudrate
        self.frame_id = frame_id
        self.simulation = simulation or not (_HAS_SERIAL and _HAS_PYNMEA)

        self._serial = None
        self._last_fix = GpsData(frame_id=frame_id)
        self._fix_count = 0

        # Simulation state
        self._sim_lat = 37.7749
        self._sim_lon = -122.4194

        fix_topic = f"{topic_prefix}.fix"

        super().__init__(
            name=name or f"gps_{port.replace('/', '_')}",
            pubs=[fix_topic],
            tick=self._tick,
            init=self._init,
            shutdown=self._shutdown,
            rate=rate,
        )

        self._fix_topic = fix_topic

    def _init(self, node: 'GpsNode') -> None:
        """Open serial port."""
        if self.simulation:
            self.log_info("GpsNode running in simulation mode")
            return

        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0,
            )
            self.log_info(f"GPS serial port opened: {self.port}")
        except Exception as e:
            self.log_error(f"Failed to open GPS serial port: {e}")
            self.simulation = True

    def _shutdown(self, node: 'GpsNode') -> None:
        """Close serial port."""
        if self._serial and self._serial.is_open:
            self._serial.close()

    def _tick(self, node: 'GpsNode') -> None:
        """Read and parse GPS data."""
        if self.simulation:
            self._publish_simulated_data()
            return

        if not self._serial or not self._serial.is_open:
            return

        try:
            # Read available lines
            while self._serial.in_waiting:
                line = self._serial.readline().decode('ascii', errors='ignore').strip()
                if line.startswith('$'):
                    self._parse_nmea(line)
        except Exception as e:
            self.log_error(f"GPS read error: {e}")

    def _parse_nmea(self, sentence: str) -> None:
        """Parse NMEA sentence and update position."""
        try:
            msg = pynmea2.parse(sentence)

            if isinstance(msg, pynmea2.GGA):
                # Global Positioning System Fix Data
                if msg.latitude and msg.longitude:
                    self._last_fix.latitude = msg.latitude
                    self._last_fix.longitude = msg.longitude
                    self._last_fix.altitude = float(msg.altitude) if msg.altitude else 0.0
                    self._last_fix.fix_type = int(msg.gps_qual) if msg.gps_qual else 0
                    self._last_fix.satellites = int(msg.num_sats) if msg.num_sats else 0
                    self._last_fix.hdop = float(msg.horizontal_dil) if msg.horizontal_dil else 99.0
                    self._last_fix.timestamp = time.time()

                    self._fix_count += 1
                    self.send(self._fix_topic, self._last_fix)

            elif isinstance(msg, pynmea2.RMC):
                # Recommended Minimum Navigation Information
                if msg.spd_over_grnd:
                    self._last_fix.speed = float(msg.spd_over_grnd) * 0.514444  # knots to m/s
                if msg.true_course:
                    self._last_fix.course = float(msg.true_course)

        except pynmea2.ParseError:
            pass  # Invalid NMEA sentence

    def _publish_simulated_data(self) -> None:
        """Generate simulated GPS data."""
        import math
        import random

        # Add small random walk
        self._sim_lat += random.gauss(0, 0.00001)
        self._sim_lon += random.gauss(0, 0.00001)

        data = GpsData(
            latitude=self._sim_lat,
            longitude=self._sim_lon,
            altitude=10.0 + random.gauss(0, 0.5),
            fix_type=1,
            satellites=8,
            hdop=1.2,
            speed=0.0,
            course=0.0,
            frame_id=self.frame_id,
        )
        self.send(self._fix_topic, data)

    @property
    def last_fix(self) -> GpsData:
        return self._last_fix

    @property
    def fix_count(self) -> int:
        return self._fix_count


# =============================================================================
# CameraNode - Video Capture
# =============================================================================

class CameraNode(Node):
    """
    Camera Capture Node.

    Captures images from USB cameras, webcams, or video files.
    Uses OpenCV for cross-platform video capture.

    Requires: opencv-python (`pip install opencv-python`)

    Topics:
        - Publishes to: {topic_prefix}.image (ImageData)
        - Publishes to: {topic_prefix}.image_raw (raw numpy array)

    Example:
        camera = CameraNode(
            device_id=0,
            width=640,
            height=480,
            fps=30,
            topic_prefix="camera"
        )
    """

    def __init__(
        self,
        device_id: int = 0,
        width: int = 640,
        height: int = 480,
        fps: float = 30.0,
        topic_prefix: str = "camera",
        frame_id: str = "camera",
        simulation: bool = False,
        name: Optional[str] = None,
        rate: Optional[float] = None,
    ):
        """
        Create a camera capture node.

        Args:
            device_id: Camera device index (0 for default camera) or video file path
            width: Capture width in pixels
            height: Capture height in pixels
            fps: Target framerate
            topic_prefix: Prefix for pub/sub topics
            frame_id: Coordinate frame ID
            simulation: Run in simulation mode
            name: Node name
            rate: Tick rate in Hz (defaults to fps)
        """
        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps
        self.frame_id = frame_id
        self.simulation = simulation or not _HAS_CV2

        self._capture = None
        self._frame_count = 0
        self._last_frame_time = 0.0

        image_topic = f"{topic_prefix}.image"
        raw_topic = f"{topic_prefix}.image_raw"

        super().__init__(
            name=name or f"camera_{device_id}",
            pubs=[image_topic, raw_topic],
            tick=self._tick,
            init=self._init,
            shutdown=self._shutdown,
            rate=rate or fps,
        )

        self._image_topic = image_topic
        self._raw_topic = raw_topic

    def _init(self, node: 'CameraNode') -> None:
        """Open camera capture."""
        if self.simulation:
            self.log_info("CameraNode running in simulation mode")
            return

        try:
            self._capture = cv2.VideoCapture(self.device_id)

            if not self._capture.isOpened():
                raise RuntimeError("Failed to open camera")

            # Set capture properties
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self._capture.set(cv2.CAP_PROP_FPS, self.fps)

            # Get actual properties
            actual_width = int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self._capture.get(cv2.CAP_PROP_FPS)

            self.log_info(f"Camera opened: {actual_width}x{actual_height} @ {actual_fps:.1f}fps")

        except Exception as e:
            self.log_error(f"Failed to open camera: {e}")
            self.simulation = True

    def _shutdown(self, node: 'CameraNode') -> None:
        """Release camera capture."""
        if self._capture:
            self._capture.release()

    def _tick(self, node: 'CameraNode') -> None:
        """Capture and publish frame."""
        if self.simulation:
            self._publish_simulated_frame()
            return

        if not self._capture or not self._capture.isOpened():
            return

        ret, frame = self._capture.read()
        if not ret:
            return

        self._frame_count += 1
        self._last_frame_time = time.time()

        # Publish raw numpy array (for CV processing)
        self.send(self._raw_topic, frame)

        # Publish ImageData message
        height, width = frame.shape[:2]
        channels = frame.shape[2] if len(frame.shape) > 2 else 1

        image_data = ImageData(
            data=frame.tobytes(),
            width=width,
            height=height,
            encoding="bgr8" if channels == 3 else "mono8",
            step=width * channels,
            frame_id=self.frame_id,
        )
        self.send(self._image_topic, image_data)

    def _publish_simulated_frame(self) -> None:
        """Generate simulated camera frame."""
        self._frame_count += 1

        if _HAS_CV2:
            # Create a visual test pattern with numpy/cv2
            frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)

            # Add some visual elements
            cv2.putText(
                frame, "SIMULATION",
                (self.width // 4, self.height // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2
            )
            cv2.putText(
                frame, f"Frame: {self._frame_count}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1
            )

            self.send(self._raw_topic, frame)
            frame_bytes = frame.tobytes()
        else:
            # Generate simple byte array without numpy
            frame_bytes = bytes(self.height * self.width * 3)
            self.send(self._raw_topic, frame_bytes)

        image_data = ImageData(
            data=frame_bytes,
            width=self.width,
            height=self.height,
            encoding="bgr8",
            step=self.width * 3,
            frame_id=self.frame_id,
        )
        self.send(self._image_topic, image_data)

    @property
    def frame_count(self) -> int:
        return self._frame_count

    def get_actual_fps(self) -> float:
        """Calculate actual capture framerate."""
        if self._frame_count < 2:
            return 0.0
        return self.fps  # Approximate


# =============================================================================
# LidarNode - 2D LIDAR Scanner (RPLidar support)
# =============================================================================

class LidarNode(Node):
    """
    2D LIDAR Scanner Node.

    Reads scan data from RPLidar devices (A1, A2, A3, S1, S2).
    Publishes LaserScan messages compatible with ROS conventions.

    Requires: rplidar-roboticia (`pip install rplidar-roboticia`)

    Topics:
        - Publishes to: {topic_prefix} (LaserScan)

    Example:
        lidar = LidarNode(
            port="/dev/ttyUSB0",
            topic_prefix="scan"
        )

        # Read scan data
        if node.has_msg("scan"):
            scan = node.get("scan")
            print(f"Got {len(scan.ranges)} range measurements")
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        topic_prefix: str = "scan",
        frame_id: str = "laser",
        num_samples: int = 360,
        range_min: float = 0.15,
        range_max: float = 12.0,
        simulation: bool = False,
        name: Optional[str] = None,
        rate: float = 10,
    ):
        """
        Create a LIDAR scanner node.

        Args:
            port: Serial port path for RPLidar
            topic_prefix: Prefix for pub/sub topics
            frame_id: TF frame ID for the laser
            num_samples: Number of samples per scan (for binning)
            range_min: Minimum valid range in meters
            range_max: Maximum valid range in meters
            simulation: Run in simulation mode
            name: Node name
            rate: Tick rate in Hz
        """
        self.port = port
        self.frame_id = frame_id
        self.num_samples = num_samples
        self.range_min = range_min
        self.range_max = range_max
        self.simulation = simulation or not _HAS_RPLIDAR

        self._lidar = None
        self._scan_iterator = None
        self._scan_count = 0
        self._sim_angle = 0.0

        super().__init__(
            name=name or "lidar",
            pubs=[topic_prefix],
            tick=self._tick,
            init=self._init,
            shutdown=self._shutdown,
            rate=rate,
        )

        self._topic = topic_prefix

    def _init(self, node: 'LidarNode') -> None:
        """Initialize RPLidar connection."""
        if self.simulation:
            self.log_info("LidarNode running in simulation mode")
            return

        try:
            self._lidar = RPLidar(self.port)
            info = self._lidar.get_info()
            health = self._lidar.get_health()
            self.log_info(f"RPLidar connected: {info.get('model', 'unknown')}, "
                         f"firmware: {info.get('firmware', 'unknown')}, "
                         f"health: {health[0]}")
            self._scan_iterator = self._lidar.iter_scans()
        except Exception as e:
            self.log_error(f"Failed to initialize RPLidar: {e}")
            self.simulation = True

    def _shutdown(self, node: 'LidarNode') -> None:
        """Stop and disconnect RPLidar."""
        if self._lidar:
            try:
                self._lidar.stop()
                self._lidar.stop_motor()
                self._lidar.disconnect()
                self.log_info("RPLidar disconnected")
            except Exception as e:
                self.log_error(f"Error shutting down RPLidar: {e}")

    def _tick(self, node: 'LidarNode') -> None:
        """Read and publish LIDAR scan."""
        if self.simulation:
            self._publish_simulated_scan()
            return

        if not self._scan_iterator:
            return

        try:
            scan = next(self._scan_iterator)
            self._process_scan(scan)
        except StopIteration:
            self._scan_iterator = self._lidar.iter_scans()
        except Exception as e:
            self.log_error(f"LIDAR read error: {e}")

    def _process_scan(self, scan: List[Tuple[float, float, float]]) -> None:
        """Process raw RPLidar scan into LaserScan message.

        Args:
            scan: List of (quality, angle_deg, distance_mm) tuples
        """
        import math

        # Initialize bins for averaging
        ranges = [float('inf')] * self.num_samples
        intensities = [0.0] * self.num_samples
        counts = [0] * self.num_samples

        angle_increment = 2 * math.pi / self.num_samples

        for quality, angle_deg, distance_mm in scan:
            if distance_mm == 0:
                continue

            distance_m = distance_mm / 1000.0
            if distance_m < self.range_min or distance_m > self.range_max:
                continue

            angle_rad = math.radians(angle_deg)
            # Bin the measurement
            bin_idx = int(angle_rad / angle_increment) % self.num_samples

            # Keep closest measurement per bin
            if distance_m < ranges[bin_idx]:
                ranges[bin_idx] = distance_m
                intensities[bin_idx] = float(quality)
            counts[bin_idx] += 1

        # Replace inf with range_max for bins with no data
        ranges = [r if r != float('inf') else self.range_max for r in ranges]

        scan_msg = LaserScan(
            ranges=ranges,
            intensities=intensities,
            angle_min=0.0,
            angle_max=2 * math.pi,
            angle_increment=angle_increment,
            range_min=self.range_min,
            range_max=self.range_max,
            frame_id=self.frame_id,
            timestamp=time.time(),
        )

        self.send(self._topic, scan_msg)
        self._scan_count += 1

    def _publish_simulated_scan(self) -> None:
        """Generate simulated LIDAR scan."""
        import math
        import random

        self._sim_angle += 0.05
        ranges = []
        intensities = []
        angle_increment = 2 * math.pi / self.num_samples

        for i in range(self.num_samples):
            angle = i * angle_increment
            # Simulate a room with walls
            base_range = 5.0 + 2.0 * math.sin(angle * 4 + self._sim_angle)
            noise = random.gauss(0, 0.02)
            r = max(self.range_min, min(self.range_max, base_range + noise))
            ranges.append(r)
            intensities.append(random.uniform(10, 50))

        scan_msg = LaserScan(
            ranges=ranges,
            intensities=intensities,
            angle_min=0.0,
            angle_max=2 * math.pi,
            angle_increment=angle_increment,
            range_min=self.range_min,
            range_max=self.range_max,
            frame_id=self.frame_id,
            timestamp=time.time(),
        )

        self.send(self._topic, scan_msg)
        self._scan_count += 1

    @property
    def scan_count(self) -> int:
        return self._scan_count

    def get_device_info(self) -> Optional[Dict[str, Any]]:
        """Get RPLidar device information."""
        if self._lidar and not self.simulation:
            try:
                return self._lidar.get_info()
            except Exception:
                pass
        return None

    def get_health(self) -> Optional[Tuple[str, int]]:
        """Get RPLidar health status."""
        if self._lidar and not self.simulation:
            try:
                return self._lidar.get_health()
            except Exception:
                pass
        return None


# =============================================================================
# Module Exports
# =============================================================================

__all__ = [
    # Data classes
    "SerialData",
    "JoystickState",
    "KeyboardState",
    "ImuData",
    "GpsData",
    "ImageData",
    "LaserScan",
    # Nodes
    "SerialNode",
    "JoystickNode",
    "KeyboardNode",
    "ImuNode",
    "GpsNode",
    "CameraNode",
    "LidarNode",
]
