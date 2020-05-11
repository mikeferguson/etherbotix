# Copyright (c) 2014-2020 Michael E. Ferguson
# Copyright (c) 2008-2013 Vanadium Labs LLC.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Michael E. Ferguson

import queue
import socket
import struct
import sys
import time
from etherbotix_python.ax12 import (
    AX_READ_DATA,
    AX_WRITE_DATA,
    AX_SYNC_READ,
    AX_SYNC_WRITE,
    P_BAUD_RATE,
    P_GOAL_POSITION_L,
    P_LED,
    P_PRESENT_LOAD_L,
    P_PRESENT_POSITION_L,
    P_PRESENT_TEMPERATURE,
    P_PRESENT_VOLTAGE,
    P_TORQUE_ENABLE,
    P_VERSION
)


class Packet:
    """A return packet, parsed."""

    id = 0           # Servo ID
    error = 0        # Error level
    params = []      # Just the payload
    params_int = []  # Payload as integers
    raw = []         # The whole raw packet
    valid = False

    def __init__(self, raw):
        """Create a packet from byte array."""
        self.raw = raw
        self.params = raw[5:-1]
        self.id = int(raw[2])
        self.error = int(raw[4])
        self.params_int = self.params
        self.valid = True

        if self.raw[0] != self.raw[1] != 0xff:
            self.valid = False
            print("failed packet: header is invalid")
        elif sum(self.raw[2:]) % 256 != 255:
            self.valid = False
            print("failed packet: checksum is invalid", self.raw)


class Etherbotix:
    """This class controls Etherbotix."""

    def __init__(self, ip="192.168.0.42", port=6707):
        """
        Construct an Etherbotix instance and opens the network connection.

        ip - the IP address of the etherbotix.
        port - The ethernet port to communicate with.
        """
        self._ip = ip
        self._port = port
        self._conn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._conn.bind(("", 0))
        self._conn.setblocking(0)
        self.packets = queue.Queue()

        self.unique_id = None
        self.version = -1
        self.baud_rate = -1
        self.digital_in = 0
        self.digital_dir = 0
        self.digital_out = 0
        self.user_io_use = 0
        self.a0 = 0
        self.a1 = 0
        self.a2 = 0
        self.system_time = -1
        self.servo_current = 0
        self.aux_current = 0
        self.system_voltage = 0
        self.led = 0
        self.imu_flags = -1
        self.motor_period = 10
        self.motor_max_step = 10
        self.motor1_vel = 0
        self.motor2_vel = 0
        self.motor1_pos = 0
        self.motor2_pos = 0
        self.motor1_current = 0
        self.motor2_current = 0
        self.motor1_kp = 0
        self.motor1_kd = 0
        self.motor1_ki = 0
        self.motor1_windup = 0
        self.motor2_kp = 0
        self.motor2_kd = 0
        self.motor2_ki = 0
        self.motor2_windup = 0
        self.accel_x = 0
        self.accel_y = 0
        self.accel_z = 0
        self.gyro_x = 0
        self.gyro_y = 0
        self.gyro_z = 0
        self.mag_x = 0
        self.mag_y = 0
        self.mag_z = 0
        self.tim12_mode = 0
        self.tim12_count = 0
        self.packets_recv = 0
        self.packets_bad = 0

        if sys.version < "3":
            self.MAGIC = "\xffBOT"
        else:
            self.MAGIC = b"\xffBOT"

    def getPacket(self):
        """Get a packet returned."""
        if self.packets.empty():
            self.recv()
        try:
            return self.packets.get_nowait()
        except queue.Empty:
            return None

    def recv(self, timeout=0.1):
        """Actually read some packet data from the device."""
        t = time.time()
        packets = 0
        while (True):
            try:
                values = self._conn.recv(1024)
                if len(values) < 8:
                    print("failed packet: packet is too short")
                    continue
                if values[0:4] != self.MAGIC:
                    print("failed packet: magic number is invalid")
                    continue
                packet = Packet(values[4:])
                if packet.valid:
                    self.packets.put(packet)
                    packets += 1
            except socket.error:
                if packets > 0:
                    return packets
                if time.time() - t > timeout:
                    return -1
                time.sleep(0.001)

    def send(self, packet):
        """
        Actually send a packet to the Etherbotix.

        packet -- byte array to send.
        """
        msg = self.MAGIC
        try:
            msg += packet
            print(msg)
        except TypeError:
            msg += bytes(packet)
        self._conn.sendto(bytes(msg), 0, (self._ip, self._port))

    def execute(self, index, ins, params, ret=True):
        """
        Send an instruction to a servo.device.

        index -- The ID of the servo/device to write.
        ins -- The instruction to send.
        params -- Byte array of parameters to send.
        ret -- Whether to read a return packet.

        Returns the packet if read.
        """
        length = 2 + len(params)
        checksum = 255 - ((index + length + ins + sum(params)) % 256)
        packet = [0xFF, 0xFF, index, length, ins] + params
        packet.append(checksum)
        self.send(packet)
        if ret:
            return self.getPacket()
        return None

    def read(self, index, start, length):
        """
        Read values of registers.

        index -- The ID of the servo/device to read.
        start -- The starting register address to read from.
        length -- The number of bytes to read.

        Returns a byte array of read, or None if failed.
        """
        # Empty the queue
        while True:
            try:
                self.packets.get_nowait()
            except queue.Empty:
                break
        # Send our packet and check response
        packet = self.execute(index, AX_READ_DATA, [start, length])
        try:
            if packet.id != index:
                return None
            return packet.params_int
        except Exception:
            return None

    def write(self, index, start, values, ret=True):
        """
        Write values to registers.

        index -- The ID of the servo.
        start -- The starting register address to begin writing.
        vaues -- The data to write.
        ret -- Whether to read a return packet.

        Returns the error level.
        """
        return self.execute(index, AX_WRITE_DATA, [start] + values, ret=ret)

    def syncWrite(self, start, values):
        """
        Write values to registers on many servos.

        start -- The starting register address to begin writing to.
        values -- The data to write, in a list of lists. Format should be
                  [(id1, val1, val2), (id2, val1, val2)]
        """
        data = [start, len(values[0]) - 1]
        for v in values:
            data += v
        return self.execute(0xFE, AX_SYNC_WRITE, data, ret=False)

    def syncRead(self, servos, start, length, ret=True):
        """
        Read values of registers on many servos.

        servos -- A list of the servo IDs to read from.
        start -- The starting register address to begin reading at.
        length -- The number of bytes to read from each servo.
        ret -- Whether to read a return packet.

        Returns a list of bytes read.
        """
        return self.execute(0xFE, AX_SYNC_READ, [start, length] + servos, ret=ret)

    def setBaud(self, index, baud):
        """
        Set baud rate of a device.

        index -- The ID of the device to write (Note: Etherbotix is 253).
        baud -- The baud rate.

        Returns the error level.
        """
        return self.write(index, P_BAUD_RATE, [baud, ])

    def enableTorque(self, index, ret=True):
        """
        Turn on the torque of a servo.

        index -- The ID of the device to enable.

        Returns The error level.
        """
        return self.write(index, P_TORQUE_ENABLE, [1], ret=ret)

    def disableTorque(self, index, ret=True):
        """
        Turn off the torque of a servo.

        index -- The ID of the device to disable.

        Returns The error level.
        """
        return self.write(index, P_TORQUE_ENABLE, [0], ret=ret)

    def setLed(self, index, value):
        """
        Set the status of the LED on a servo.

        index -- The ID of the device to write.
        value -- 0 to turn the LED off, >0 to turn it on.

        Returns The error level.
        """
        return self.write(index, P_LED, [value])

    def setPosition(self, index, value):
        """
        Set the position of a servo.

        index -- The ID of the device to write.
        value -- The position to go to, in servo ticks.

        Returns The error level.
        """
        return self.write(index, P_GOAL_POSITION_L, [value % 256, value >> 8])

    def getPosition(self, index):
        """
        Get the position of a servo.

        index -- The ID of the device to read.

        Returns The servo position, in ticks.
        """
        values = self.read(index, P_PRESENT_POSITION_L, 2)
        try:
            return int(values[0]) + (int(values[1]) << 8)
        except Exception:
            return -1

    def getLoad(self, index):
        """
        Get the torque of a servo.

        index -- The ID of the device to read.

        Returns The torque, unitless.
        """
        values = self.read(index, P_PRESENT_LOAD_L, 2)
        try:
            return int(values[0]) + ((int(values[1]) << 8) & 0x3)
        except Exception:
            return -1

    def getVoltage(self, index):
        """
        Get the voltage of a device.

        index -- The ID of the device to read.

        Returns The voltage, in Volts.
        """
        try:
            return int(self.read(index, P_PRESENT_VOLTAGE, 1)[0]) / 10.0
        except Exception:
            return -1

    def getTemperature(self, index):
        """
        Get the temperature of a device.

        index -- The ID of the device to read.

        Returns The temperature, in degrees C.
        """
        try:
            return int(self.read(index, P_PRESENT_TEMPERATURE, 1)[0])
        except Exception:
            return -1

    ###########################################################################
    # Extended Etherbotix Driver

    # Helper definition for analog and digital access.
    LOW = 0
    # Helper definition for analog and digital access.
    HIGH = 0xff
    # Helper definition for analog and digital access.
    INPUT = 0
    # Helper definition for analog and digital access.
    OUTPUT = 0xff

    # Digital/Analog Access
    P_DIGITAL_IN = 6
    P_DIGITAL_DIR = 7
    P_DIGITAL_OUT = 8
    P_USER_IO_USE = 9
    P_A0 = 10
    P_A1 = 12
    P_A2 = 14

    # Other Data
    P_SYSTEM_TIME = 16
    P_SERVO_CURRENT = 20
    P_AUX_CURRENT = 22
    P_SYSTEM_VOLTAGE = 24
    P_LED = 25
    P_IMU_FLAGS = 28

    # Base Motors
    P_MOTOR_PERIOD = 29
    P_MOTOR_MAX_STEP = 30
    P_MOTOR1_VEL = 32
    P_MOTOR2_VEL = 34
    P_MOTOR1_POS = 36
    P_MOTOR2_POS = 40
    P_MOTOR1_CURRENT = 44
    P_MOTOR2_CURRENT = 46
    P_MOTOR1_KP = 48
    P_MOTOR1_KD = 52
    P_MOTOR1_KI = 56
    P_MOTOR1_WINDUP = 60
    P_MOTOR2_KP = 64
    P_MOTOR2_KD = 68
    P_MOTOR2_KI = 72
    P_MOTOR2_WINDUP = 76

    # IMU
    P_ACC_X = 80
    P_ACC_Y = 82
    P_ACC_Z = 84
    P_GYRO_X = 86
    P_GYRO_Y = 88
    P_GYRO_Z = 90
    P_MAG_X = 92
    P_MAG_Y = 94
    P_MAG_Z = 96

    # Device Setup
    P_USART_BAUD = 98
    P_USART_CHAR = 99
    P_TIM9_MODE = 100
    P_TIM9_COUNT = 102
    P_TIM12_MODE = 104
    P_TIM12_COUNT = 106
    P_SPI_BAUD = 108

    # Stats
    P_PACKETS_RECV = 120
    P_PACKETS_BAD = 124

    # Devices
    P_DEVICE_BOOTLOADER = 192
    P_DEVICE_UNIQUE_ID = 193
    P_DEVICE_M1_TRACE = 194
    P_DEVICE_M2_TRACE = 195

    def getAnalog(self, index, leng=1):
        """
        Get the value of an analog input pin.

        index -- The ID of the pin to read (0 to 2).

        Returns 16-bit analog value of the pin, None if error.
        """
        try:
            values = self.read(253, self.P_A0 + int(index * 2), 2)
            return int(values[0]) + (int(values[1]) << 8)
        except Exception:
            return None

    def getDigital(self, index):
        """
        Get the value of an digital input pin.

        index -- The ID of the pin to read (0 to 7).

        Returns 0 for low, 255 for high, None if error.
        """
        try:
            if index < 8:
                x = self.read(253, self.P_DIGITAL_IN, 1)[0]
                self.digital_in = x
            else:
                return None
        except Exception:
            return None
        if x & (2 ** (index % 8)):
            return 255
        else:
            return 0

    def setDigital(self, index, value, direction=1):
        """
        Set the value & direction of a digital pin.

        index -- The ID of the pin to write (0 to 7).
        value -- 0 or 1.
        direction -- Input (0) or output (1)
        """
        if index > 7:
            return None
        mask = 2**index
        if value > 0:
            value = mask
        if direction > 0:
            direction = mask
        value = (self.digital_out & ~mask) | value
        direction = (self.digital_dir & ~mask) | direction
        if self.version > 0:
            # Use new API, no mask
            self.write(253, self.P_DIGITAL_DIR, [direction, value], ret=False)
        else:
            self.write(253, self.P_DIGITAL_IN, [mask, direction, value], ret=False)
        # Store in case we don't read soon
        self.digital_out = value
        self.digital_dir = direction

    def setTimer12Mode(self, mode):
        """
        Set the mode for Timer 12.

        mode -- The mode, one of:
            0 - not active
            1 - count on rising edge of D5.
        """
        self.write(253, self.P_TIM12_MODE, [mode], ret=False)

    def getTimer12Count(self):
        """Return 16-bit count, or None if read fails."""
        try:
            values = self.read(253, self.P_TIM12_COUNT, 2)
            self.tim12_count = int(values[0]) + (int(values[1]) << 8)
            return self.tim12_count
        except Exception:
            return None

    def getUniqueId(self):
        """Get the unique ID of the board."""
        if self.unique_id is None:
            packet = self.read(253, self.P_DEVICE_UNIQUE_ID, 12)
            self.unique_id = "".join([hex(p)[2:4] for p in packet])
        return "0x" + self.unique_id

    def getImuVersion(self):
        return self.imu_flags & 0x0f

    ###########################################################################
    # Extended State

    def updateFromPacket(self, packet):
        """
        Parse a packet into member variables.

        packet -- A packet, as returned from getPacket().

        Returns True if state was updated.
        """
        if packet is None or not packet.valid:
            return False
        if packet.id != 253 or len(packet.params) != 128:
            return False

        self.version = packet.params_int[P_VERSION]
        self.baud_rate = packet.params_int[P_BAUD_RATE]
        self.digital_in = packet.params_int[self.P_DIGITAL_IN]
        self.digital_dir = packet.params_int[self.P_DIGITAL_DIR]
        self.digital_out = packet.params_int[self.P_DIGITAL_OUT]
        self.user_io_use = packet.params_int[self.P_USER_IO_USE]
        self.a0 = struct.unpack_from("<H", packet.params, self.P_A0)[0]
        self.a1 = struct.unpack_from("<H", packet.params, self.P_A1)[0]
        self.a2 = struct.unpack_from("<H", packet.params, self.P_A2)[0]
        self.system_time = struct.unpack_from("<L", packet.params, self.P_SYSTEM_TIME)[0]
        servo_current = struct.unpack_from("<h", packet.params, self.P_SERVO_CURRENT)[0]
        self.servo_current = servo_current / 1000.0  # mA -> A
        aux_current = struct.unpack_from("<h", packet.params, self.P_AUX_CURRENT)[0]
        self.aux_current = aux_current / 1000.0  # mA -> A
        system_voltage = struct.unpack_from("<B", packet.params, self.P_SYSTEM_VOLTAGE)[0]
        self.system_voltage = system_voltage / 10.0  # 100mV -> V
        self.led = struct.unpack_from("<H", packet.params, self.P_LED)[0]
        self.imu_flags = packet.params_int[self.P_IMU_FLAGS]

        self.motor_period = struct.unpack_from("<B", packet.params, self.P_MOTOR_PERIOD)[0]
        self.motor_max_step = struct.unpack_from("<H", packet.params, self.P_MOTOR_MAX_STEP)[0]

        self.motor1_vel = struct.unpack_from("<h", packet.params, self.P_MOTOR1_VEL)[0]
        self.motor2_vel = struct.unpack_from("<h", packet.params, self.P_MOTOR2_VEL)[0]
        self.motor1_pos = struct.unpack_from("<i", packet.params, self.P_MOTOR1_POS)[0]
        self.motor2_pos = struct.unpack_from("<i", packet.params, self.P_MOTOR2_POS)[0]
        self.motor1_current = struct.unpack_from("<h", packet.params, self.P_MOTOR1_CURRENT)[0]
        self.motor2_current = struct.unpack_from("<h", packet.params, self.P_MOTOR2_CURRENT)[0]

        self.motor1_kp = struct.unpack_from("<f", packet.params, self.P_MOTOR1_KP)[0]
        self.motor1_kd = struct.unpack_from("<f", packet.params, self.P_MOTOR1_KD)[0]
        self.motor1_ki = struct.unpack_from("<f", packet.params, self.P_MOTOR1_KI)[0]
        self.motor1_windup = struct.unpack_from("<f", packet.params, self.P_MOTOR1_WINDUP)[0]
        self.motor2_kp = struct.unpack_from("<f", packet.params, self.P_MOTOR2_KP)[0]
        self.motor2_kd = struct.unpack_from("<f", packet.params, self.P_MOTOR2_KD)[0]
        self.motor2_ki = struct.unpack_from("<f", packet.params, self.P_MOTOR2_KI)[0]
        self.motor2_windup = struct.unpack_from("<f", packet.params, self.P_MOTOR2_WINDUP)[0]

        self.accel_x = struct.unpack_from("<h", packet.params, self.P_ACC_X)[0]
        self.accel_y = struct.unpack_from("<h", packet.params, self.P_ACC_Y)[0]
        self.accel_z = struct.unpack_from("<h", packet.params, self.P_ACC_Z)[0]
        self.gyro_x = struct.unpack_from("<h", packet.params, self.P_GYRO_X)[0]
        self.gyro_y = struct.unpack_from("<h", packet.params, self.P_GYRO_Y)[0]
        self.gyro_z = struct.unpack_from("<h", packet.params, self.P_GYRO_Z)[0]
        self.mag_x = struct.unpack_from("<h", packet.params, self.P_MAG_X)[0]
        self.mag_y = struct.unpack_from("<h", packet.params, self.P_MAG_Y)[0]
        self.mag_z = struct.unpack_from("<h", packet.params, self.P_MAG_Z)[0]

        self.tim12_mode = struct.unpack_from("<h", packet.params, self.P_TIM12_MODE)[0]
        self.tim12_count = struct.unpack_from("<h", packet.params, self.P_TIM12_COUNT)[0]

        self.packets_recv = struct.unpack_from("<L", packet.params, self.P_PACKETS_RECV)[0]
        self.packets_bad = struct.unpack_from("<L", packet.params, self.P_PACKETS_BAD)[0]
        return True
