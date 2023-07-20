#!/usr/bin/env python2

# @file  imu_control.py
#
# @license  LGPL-3+
#
# This file is part of the OpenAXES project, a wireless IMU.
# Copyright 2023 Nils Stanislawski and Fritz Webering
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


from __future__ import print_function
#udpstream.py MAC Port
#Connect to IMU with adress MAC, start a measurement (w/o MAG) and
#Stream the resulting data over UDP to the given port.

#apt install python-pip && pip install pygatt[GATTTOOL] pexpect
#pip install bluepy
#wget https://s3.amazonaws.com/plugable/bin/fw-0a5c_21e8.hcd -O /lib/firmware/fw-0a5c_21e8.hcd
#wget https://s3.amazonaws.com/plugable/bin/fw-0a5c_21e8.hcd -O /lib/firmware/brcm/BCM20702A0-0a5c-21e8.hcd
#modprobe -r btusb && modprobe btusb
#wget https://github.com/winterheart/broadcom-bt-firmware/blob/master/brcm/BCM20702A1-0b05-17cb.hcd?raw=true -O /lib/firmware/brcm/BCM20702A1-0b05-17cb.hcd

#gatttool -b "18:04:ED:C4:75:45" --char-write-req -a 0x002c -n 0100
#gatttool -b 18:04:ED:C4:75:45" --char-write-req -a 0x0032 -n 02 --listen

from __future__ import division
from curses import meta
import os
import subprocess
import socket
import re
import struct
import sys
import time
import argparse
import datetime

STILL_BEEP_SOUND_FILE = '/usr/share/sounds/freedesktop/stereo/message.oga'
STILL_GYRO_THRESHOLD = 0.01

IMU_OUT_INDEX = 'INDEX'
IMU_OUT_QUATERNION = 'QUATERNION'
IMU_OUT_ACCEL_ADXL355 = 'ACCEL_ADXL355'
IMU_OUT_ACCEL_BMI160 = 'ACCEL_BMI160'
IMU_OUT_GYRO = 'GYRO'
IMU_OUT_MAGNET = 'MAGNET'
IMU_OUT_EXT_ANALOG = 'EXT_ANALOG'
IMU_OUT_TEMP = 'TEMP'
IMU_OUT_PRES = 'PRES'
IMU_OUT_GYRO_ECC = 'GYRO_ECC'
IMU_OUT_ACCEL = 'ACCEL' # Pseudo-value to prevent typos in payload variable

import json
try:
    with open('my_imu_addresses.json', 'r') as f:
        IMUS = json.load(f)
except IOError:
	IMUS = {}

IMUS_MAC = {mac: name for name, mac in IMUS.items()}

IMU_output_info = [
    { 'name': IMU_OUT_INDEX,          'bit':0x8000, 'fmt':'H', 'count':1, 'scale':1, },
    { 'name': IMU_OUT_QUATERNION,     'bit':0x4000, 'fmt':'4h', 'count':4, 'scale':1/32767, },
    { 'name': IMU_OUT_ACCEL_ADXL355,  'bit':0x2000, 'fmt':'3h', 'count':3, 'scale':1/1600, },
    { 'name': IMU_OUT_ACCEL_BMI160,   'bit':0x1000, 'fmt':'3h', 'count':3, 'scale':1/1600, },
    { 'name': IMU_OUT_GYRO,           'bit':0x0800, 'fmt':'3h', 'count':3, 'scale':0.001065264, },
    { 'name': IMU_OUT_MAGNET,         'bit':0x0400, 'fmt':'3h', 'count':3, 'scale':1, },
    { 'name': IMU_OUT_EXT_ANALOG,     'bit':0x0200, 'fmt':'B', 'count':1, 'scale':1, },
    { 'name': IMU_OUT_TEMP,           'bit':0x0100, 'fmt':'h', 'count':1, 'scale':1, },
    { 'name': IMU_OUT_PRES,           'bit':0x0080, 'fmt':'h', 'count':1, 'scale':1, },
    { 'name': IMU_OUT_GYRO_ECC,       'bit':0x0040, 'fmt':'3h', 'count':3, 'scale':1, },
]
IMU_mode_info = IMU_output_info + [
    { 'name': 'SELECT_FILTER_ACCEL',  'bit': 0x0002},
    { 'name': 'ENABLE_SC',            'bit': 0x0001},
]
IMU_mode_dict = { info['name'] : info for info in IMU_mode_info }
IMU_mode_default = '0xE000'
IMU_mode_stop = '0000'

IMU_COMMANDS_HEADER_LENGTH = 2
IMU_COMMANDS = {
    'CALIBRATION_COEFFICIENTS'      : (0x10, 4 * 3 * (9 + 3)), # float matrix and vector for 3 sensors
    'SAVE_CALIBRATION'              : (0x11, 0),
    'LOAD_CALIBRATION'              : (0x12, 0),
    'USE_CALIBRATION'               : (0x13, 1), # boolean true == enable, false == disable
    'ENABLE_UART_TRANSMISSION'      : (0x14, 1), # boolean true == enable, false == disable
}

IMU_CONFIG_CALIBRATION_COEFFICIENTS = 0x10
IMU_CONFIG_PAYLOAD_LENGTHS = {
    IMU_CONFIG_CALIBRATION_COEFFICIENTS: 4 * 3 * (9 + 3),
}

def single_byte(string):
    value = int(string, base=0) # Automatically determine base
    if value < 0 or value > 0xFF:
        raise ValueError('Byte must be between 0 and 255 (inclusive)')
    return value

def verbose_print(*args, **kwargs):
    level = 1
    if 'level' in kwargs:
        level = kwargs['level']
        del kwargs['level']
    if ARGS.verbose >= level:
        print(*args, **kwargs)

def run_command(*args, **kwargs):
    if ARGS.verbose >= 2: print('%s: Executing: %s' % (IMU, ' '.join(args[0])))
    output = subprocess.check_output(*args, **kwargs)
    if ARGS.verbose >= 4: print('%s: |--> Result: "%s"' % (IMU, output))
    return output

class GattClient:
    descriptor_uuids = {
        '2900': 'CHAR_EXT_PROPS',     # Characteristic Extended Properties
        '2901': 'CHAR_USER_DESC',     # Characteristic User Description
        '2902': 'CLIENT_CHAR_CFG',    # Client Characteristic Configuration
        '2903': 'SERV_CHAR_CFG',      # Server Characteristic Configuration
        '2904': 'CHAR_FORMAT',        # Characteristic Presentation Format
        '2905': 'CHAR_AGG_FORMAT',    # Characteristic Aggregate Format
        '2906': 'VALID_RANGE',        # Valid Range
        '2907': 'EXT_REPORT_REF',     # External Report Reference Descriptor
        '2908': 'REPORT_REF',         # Report Reference Descriptor
    }
    handles = {}
    def __init__(self, mac, interface):
        self.mac = mac
        self.interface = interface
        output = run_command(['gatttool', '-i', self.interface, '-b', self.mac, '--char-desc'])
        regex = r'handle = (0x[0-9a-fA-F]{4}), uuid = 0000([0-9a-fA-F]{4})-0000-1000-8000-00805f9b34fb'
        for match in re.finditer(regex, output.lower()):
            handle, uuid = match.group(1, 2)
            if uuid not in self.handles and uuid not in self.descriptor_uuids:
                self.handles[uuid] = { 'VALUE': handle }
                last_uuid = uuid
            if uuid in self.descriptor_uuids:
                descriptor_name = self.descriptor_uuids[uuid]
                self.handles[last_uuid][descriptor_name] = handle
        verbose_print('%s: GattClient.handles = %s' % (IMU, repr(self.handles)), level=4)

    def handle_write(self, handle, value):
        command = ['gatttool', '-i', self.interface, '-b', self.mac, '--mtu=100', '--char-write-req', '-a', handle, '-n', value]
        output = run_command(command, stderr=subprocess.STDOUT)
        if 'Characteristic value was written successfully' in output:
            return True
        else:
            print(output, file=sys.stderr)
            return False

    def char_write(self, uuid, value, descriptor_name='VALUE'):
        return self.handle_write(self.handles[uuid][descriptor_name], value)

    def char_write_ccc(self, uuid, value):
        return self.char_write(uuid, value, descriptor_name='CLIENT_CHAR_CFG')

    def char_write_listen(self, uuid, value, descriptor_name='VALUE'):
        handle = self.handles[uuid][descriptor_name]
        command = ["gatttool", "-i", INTERFACE, "-b", MAC, "--char-write-req", "-a", handle, "-n", value, '--mtu=100', "--listen"]
        verbose_print(' '.join(command), level=2)
        return subprocess.Popen(command, stdout=subprocess.PIPE, universal_newlines=True)

    def handle_read(self, handle):
        command = ['gatttool', '-i', self.interface, '-b', self.mac, '--mtu=100', '--char-read', '-a', handle]
        output = run_command(command)
        assert output[0:33] == 'Characteristic value/descriptor: '
        return output[33:].strip()

    def char_read(self, uuid, descriptor_name='VALUE'):
        return self.handle_read(self.handles[uuid][descriptor_name])

    def char_read_byte(self, uuid):
        return int(self.char_read(uuid), base=16)

    def char_read_string(self, uuid):
        return bytearray.fromhex(self.char_read(uuid)).decode('utf-8')


arg_parser = argparse.ArgumentParser(description='Receive IMU packets via BLE, log them, and send on via UDP to localhost.')
arg_parser.add_argument('IMU', help='Name or BLE MAC address of the IMU. Known names are: ' + ', '.join(sorted(IMUS.keys())))
arg_parser.add_argument('PORT', nargs='?', default=1024, type=int, help='UDP port and name of the log file (default 1024)')
arg_parser.add_argument('--text', '--processing', '-t', help='Send UDP packets as text instead of binary. For compatibility with the dsense_vis processing app.', action='store_true')
arg_parser.add_argument('--hci', help='Name or MAC-prefix the hci adapter to use (default: "5C:F3:70:" to auto-detect USB-BT400)', default="5C:F3:70:")
arg_parser.add_argument('--command', nargs='+', metavar='HEX_OR_CMD',
    help='Send a command or raw hex value to the command and configuration characteristic of the IMU. Commands are: ' + ', '.join(IMU_COMMANDS.keys()))
arg_parser.add_argument('--reset', nargs='?', help='Reset bit mask for the IMU. Possible reset bits are: SENSORS==0x01, QUATERNION==0x02, INDEX==0x04. (default value 0x07)', default='0x00', const='0x07', type=single_byte, metavar='BITS')
arg_parser.add_argument('--reset-only', nargs='?', help='Reset the IMU (as with --reset=BITS), then exit. (default value 0x07)', default='0x00', const='0x07', type=single_byte, metavar='BITS')
arg_parser.add_argument('--reset-sensors', help='Reset the IMU sensor ICs, then exit (same as --reset-only=0x01)', dest='reset_only', action='store_const', const=0x01)
arg_parser.add_argument('--no-reset', help='Don\'t reset the IMU before starting a measurement. Same as --reset=0', dest='reset', action='store_const', const=0)
arg_parser.add_argument('--status', help='Show information about the current IMU status, then exit', action='store_true')
arg_parser.add_argument('-b', '--battery', help='Show battery level at startup', action='store_true')
arg_parser.add_argument('--mode',
    help=('Select the outputs and mode bits for the IMU (default %s). ' % IMU_mode_default) +
    'Can be a hex number or a comma-separated list of any of the following: ' +
    ', '.join([info['name'] for info in IMU_mode_info]),
    default=IMU_mode_default)
arg_parser.add_argument('--write-calibration-full', metavar='COEFFS',
    help='Write calibration coefficients to IMU. COEFFS must be a space separated string of 36 floating point coefficients (9 + 3 + 9 + 3) in row-major order: 3x3 ADXL355 matrix, 3x1 ADXL355 bias, 3x3 BMI160 accel matrix, 3x1 BMI160 accel bias, 3x3 BMI160 gyro matrix, 3x1 BMI160 gyro bias.')
arg_parser.add_argument('-v', '--verbose', help='Print more information (can be given multiple times to print even more information)', action='count')
arg_parser.add_argument('--version', help='Print IMU firmware version', action='store_true')
arg_parser.add_argument('-l', '--logfile', help='Filename of the logfile (default: log_$imu_$date.csv if IMU name is given or port_$port.log otherwise)', default=None)
arg_parser.add_argument('--logdir', help='In which folder to place the log file (default "logs")', default='logs')
arg_parser.add_argument('--skip', '-s', type=int, help='Skip the first SKIP packets, for example it there is packet loss at the beginning (default: 0).', default=0)
arg_parser.add_argument('--still', type=int, help='Count seconds during which the IMU is still. If still for at least STILL seconds, play a sound', default=0)
arg_parser.add_argument('--beep', help='Sound file to play when IMU is still, see --still (default: %s)' % STILL_BEEP_SOUND_FILE, default=STILL_BEEP_SOUND_FILE)

ARGS = arg_parser.parse_args()

logfilename = ARGS.logfile # Might be None, that is checked below
SAMPLE_RATE = 100
INTERFACE = None
if ARGS.IMU in IMUS:
    IMU = ARGS.IMU
    MAC = IMUS[ARGS.IMU]
    now = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    if logfilename is None:
        logfilename = 'log_%s_%s.csv' % (ARGS.IMU, now)
else: # Given IMU is not a name, so it must be a MAC address
    MAC = ARGS.IMU
    if not re.match('[a-z0-9]{2}(:[a-z0-9]{2}){5}', MAC, re.IGNORECASE):
        print('ERROR: "%s" is not an IMU name and not a MAC address' % MAC, file=sys.stderr)
        sys.exit(1)
    IMU = IMUS_MAC.get(MAC, MAC)
PORT = ARGS.PORT

if ':' in ARGS.hci: # MAC prefix given. Do auto detection.
    output = run_command("hcitool dev | grep -E '\\s%s' | cut -f2" % ARGS.hci, shell=True)
    if len(output) > 0:
        verbose_print("Auto-detected interface %s" % output)
        INTERFACE = output.strip()
    else:
        print("Interface auto-detection failed. Please connect a bluetooth adapter with a mac starting with %s." % ARGS.hci, file=sys.stderr)
        sys.exit(1)
else:
    INTERFACE = ARGS.hci

gatt = GattClient(MAC, INTERFACE)

IMUBattery_UUID = '2a19' # Standard Bluetooth UUID for battery service
IMUConfig_UUID = 'cf01'
IMUStatus_UUID = 'cf02'
IMUCommand_UUID = 'cf03'
SensorData_UUID = 'cf04'
IMUVersion_UUID = 'cf05'

CCC_EnableNotification = "0100"

NotifyPeriod = "{:02X}".format(int(1000 / SAMPLE_RATE))

SensorData_Handle = gatt.handles[SensorData_UUID]['VALUE']
notificationRegex = ur"^Notification handle = %s value: (?P<data>([0-9a-f]{2} )*)\n$" % SensorData_Handle
batteryNotifictionRegex = ur"^Notification handle = 0x0025 value: (?P<data>[0-9a-f]{2}) \n$"


def bytes_to_hex(b):
    return ''.join('{:02x}'.format(ord(c)) for c in b)


verbose_print("%s: Creating UDP socket for port %s" % (IMU,PORT))
#if PORT2 == None:
#	print("%s: Creating UDP socket for port %s" % (MAC,PORT))
#else:
#	print("%s: Creating UDP socket for port %s and port %s" % (MAC,PORT,PORT2))
udpSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


if ARGS.reset_only != 0:
    ARGS.reset = ARGS.reset_only
if ARGS.reset != 0:
    verbose_print("%s: Resetting IMU through characteristic 3" % IMU)
    reset_hex = '%02X' % ARGS.reset
    gatt.char_write(IMUCommand_UUID, reset_hex)
    if ARGS.reset_only != 0:
        sys.exit(0)

def hex_to_uint(hex):
    if type(hex) in (list, tuple):
        hex = ''.join(hex)
    return int(hex, base=16)

def hex_to_int16(hex):
    value = hex_to_uint(hex)
    if value > 0x7FFF:
        return value - 0x10000
    return value

CHAR2_OFFSET_BUILD_TIME = 0
CHAR2_OFFSET_BATTERY_CURRENT = 4
CHAR2_OFFSET_BATTERY_CURRENT_AVERAGE = 6
CHAR2_OFFSET_BATTERY_LEVEL = 8
CHAR2_OFFSET_VERSION_STRING = 10

imu_status_typedef_v0_3 = [ # IMU status data format for firmware version 0.3 and up
    ('int16_t', 'battery_current'),            # Battery current in steps of 156.25 uA per LSB (with 0.01 Ohm sense resistor)
    ('int16_t', 'battery_current_average'),    # Battery current (averaged over time) in steps of 156.25 uA per LSB (with 0.01 Ohm sense resistor)
    ('uint16_t', 'battery_voltage'),           # Battery voltage in steps of 0.078125 mV per LSB
    ('uint16_t', 'battery_capacity'),          # Battery remaining capacity in units of 0.5 mAh per LSB (with 0.01 Ohm sense resistor)
    ('uint16_t', 'battery_capacity_full'),     # Battery full capacity in units of 0.5 mAh per LSB (with 0.01 Ohm sense resistor)
    ('uint8_t', 'battery_percent'),            # Battery state of charge in percent
    ('uint8_t', 'calibration_status'),         # Bitfield: Status of IMU calibration. See CHAR2_CALIBRATION_STATUS_* macros for meaning
]
struct_types = {
    'int16_t': 'h',
    'uint16_t': 'H',
    'uint8_t': 'B',
    'int8_t': 'b',
}
def unpack_c_struct(typedef, bytes, endian='>'):
    format = endian + ''.join([ struct_types[t] for t, _name in typedef ])
    values = struct.unpack(format, bytes)
    return { name: value for value, (_t, name) in zip(values, typedef)}


battery_data_hex = gatt.char_read(IMUBattery_UUID)
battery_level = int(battery_data_hex, base=16)
if ARGS.verbose >= 1 or ARGS.battery:
    print('%s: Battery level: %s%%' % (IMU, battery_level))
if battery_level < 15:
    print('WARNING: %s Battery level is below 15%' % IMU)


if ARGS.version:
    firmware_version_hex = gatt.char_read(IMUVersion_UUID)
    firmware_version = bytearray.fromhex(firmware_version_hex).decode('utf-8')
    print('%s: Firmware version %s' % (IMU, firmware_version))
    sys.exit(0)

if ARGS.status:
    status_data_hex = gatt.char_read(IMUStatus_UUID)
    verbose_print('%s: status_data_hex = %s' % (IMU, status_data_hex.strip()), level=4)
    imu_status = unpack_c_struct(imu_status_typedef_v0_3, bytearray.fromhex(status_data_hex))
    verbose_print(imu_status, level=3)
    charging = imu_status['battery_current'] > 0
    charging_str = ('' if charging else 'dis') + 'charging' 
    print('%s: battery_voltage = %.2f V' % (IMU, imu_status['battery_voltage'] * 0.078125 / 1000))
    print('%s: battery_current = %.2f mA (%s)' % (IMU, imu_status['battery_current'] * 156.25 / 1000, charging_str))
    print('%s: battery_current_average =  %.2f mA (%s)' % (IMU, imu_status['battery_current_average'] * 156.25 / 1000, charging_str))
    print('%s: battery_capacity =  %.2f mAh' % (IMU, imu_status['battery_capacity'] * 0.5))
    print('%s: battery_capacity_full =  %.2f mAh' % (IMU, imu_status['battery_capacity_full'] * 0.5))
    print('%s: calibration_status =  0x%02X' % (IMU, imu_status['calibration_status']))
    print('%s: battery_level = %s' % (IMU, imu_status['battery_percent']))
    sys.exit(0)

def IMU_write_config(command, data_bytes):
    code, payload_length = IMU_COMMANDS[command]
    total_length = payload_length + IMU_COMMANDS_HEADER_LENGTH
    header = [code, total_length]
    data_bytes = struct.pack('>BB', *header) + data_bytes
    assert total_length == len(data_bytes)
    data_hex = bytes_to_hex(data_bytes)
    if gatt.char_write(IMUCommand_UUID, data_hex):
        print('Success')
    else:
        sys.exit(1)

if ARGS.command: # TODO: rewrite to use IMU_write_config(command, data_bytes)
    command = ARGS.command[0]
    if command in IMU_COMMANDS:
        code, payload_length = IMU_COMMANDS[command]
        payload = b''
        if payload_length > 0:
            payload_hex = ARGS.command[1]
            while len(payload_hex) == 1: 
                payload_hex = '0' + payload_hex
            payload = str(bytearray.fromhex(payload_hex))
        assert len(payload) == payload_length
        data = chr(code) + chr(payload_length + IMU_COMMANDS_HEADER_LENGTH) + payload
    else:
        try:
            data = str(bytearray.fromhex(command))
        except ValueError:
            print('Command is not a hex string')
            sys.exit(1)
        if len(data) > 1:
            assert ord(data[1]) == len(data[2:]) + IMU_COMMANDS_HEADER_LENGTH
    gatt.char_write(IMUCommand_UUID, bytes_to_hex(data))
    sys.exit(0)


if ARGS.write_calibration_full:
    coeffs = [float(s) for s in ARGS.write_calibration_full.strip().split()]
    if len(coeffs) != 36:
        print('Error: need 36 arguments, got %d' % len(coeffs))
        sys.exit(1)
    print('%s: Writing calibration data.' % IMU)
    print('Accelerometer ADXL355 misalignment and scaling:')
    print('%+ 8.5f  %+ 8.5f  %+ 8.5f  ' % tuple(coeffs[0:3]))
    print('%+ 8.5f  %+ 8.5f  %+ 8.5f  ' % tuple(coeffs[3:6]))
    print('%+ 8.5f  %+ 8.5f  %+ 8.5f  ' % tuple(coeffs[6:9]))
    print()
    print('Accelerometer ADXL355 bias:')
    print('%+ 8.5f  %+ 8.5f  %+ 8.5f  ' % tuple(coeffs[9:12]))
    print()
    print('Accelerometer BMI160 misalignment and scaling:')
    print('%+ 8.5f  %+ 8.5f  %+ 8.5f  ' % tuple(coeffs[12:15]))
    print('%+ 8.5f  %+ 8.5f  %+ 8.5f  ' % tuple(coeffs[15:18]))
    print('%+ 8.5f  %+ 8.5f  %+ 8.5f  ' % tuple(coeffs[18:21]))
    print()
    print('Accelerometer BMI160 bias:')
    print('%+ 8.5f  %+ 8.5f  %+ 8.5f  ' % tuple(coeffs[21:24]))
    print()
    print('Gyroscope misalignment and scaling:')
    print('%+ 8.5f  %+ 8.5f  %+ 8.5f  ' % tuple(coeffs[24:27]))
    print('%+ 8.5f  %+ 8.5f  %+ 8.5f  ' % tuple(coeffs[27:30]))
    print('%+ 8.5f  %+ 8.5f  %+ 8.5f  ' % tuple(coeffs[30:33]))
    print()
    print('Gyroscope bias:')
    print('%+ 8.5f  %+ 8.5f  %+ 8.5f  ' % tuple(coeffs[33:36]))
    print()
    packed_floats = struct.pack('<'+36*'f', *coeffs)
    IMU_write_config('CALIBRATION_COEFFICIENTS', packed_floats)
    sys.exit(0)


## Calculate IMU mode bits from output options
if '0x' in ARGS.mode:
    IMU_mode_int = int(ARGS.mode, base=16) | IMU_mode_dict['ENABLE_SC']['bit']
else:
    IMU_mode_int = 0
    for name in ARGS.mode.split(','):
        IMU_mode_int = IMU_mode_int | IMU_mode_dict[name.strip()]['bit']

IMU_mode_int = IMU_mode_int | IMU_mode_dict['ENABLE_SC']['bit'] # Force set "enable measurement" bit
IMU_mode_hex = '%04X' % IMU_mode_int
IMU_mode_names = [ info['name'] for info in IMU_mode_info if IMU_mode_int & info['bit'] != 0 ]

verbose_print('IMU mode is 0x%s = %s' % (IMU_mode_hex, str(IMU_mode_names)))

## Calculate IMU output format descriptor for struct.unpack()
IMU_packet_length = 20
IMU_output_struct_format = '>'
IMU_output_names = []
for info in IMU_output_info:
    if IMU_mode_int & info['bit'] == 0: continue
    if struct.calcsize(IMU_output_struct_format + info['fmt']) <= IMU_packet_length:
        IMU_output_struct_format += info['fmt']
        IMU_output_names.append(info['name'])
    else:
        print('IMU mode contains too many outputs to fit into a packet of %d bytes' % IMU_packet_length)
        print('The following output will be left out: %s' % info['name'])
padding_bytes = IMU_packet_length - struct.calcsize(IMU_output_struct_format)
if padding_bytes > 0:
    IMU_output_struct_format += str(padding_bytes) + 'B'

verbose_print('IMU_output_names is %s' % str(IMU_output_names), level=2)
verbose_print('IMU_output_struct_format is %s' % IMU_output_struct_format, level=2)


verbose_print("%s: Enabling notification for SensorData" % IMU)
gatt.char_write_ccc(SensorData_UUID, CCC_EnableNotification)
#print("%s: Setting notify period" % IMU)
#run_command(["gatttool", "-i", INTERFACE, "-b", MAC, "--char-write-req", "-a", SensorNotifyPeriod_Handle, "-n", NotifyPeriod])

print('%s: Starting BLE receiver' % IMU)
gatttool = gatt.char_write_listen(IMUConfig_UUID, IMU_mode_hex)


## Set connection interval to a lower value to avoid packet loss.
## The default connection interval is too slow to transmit all packets at 100 Hz.
time.sleep(0.1) # Wait for gatttool to open connection
connection_handles = run_command(['hcitool', 'con'])
if MAC.upper() in connection_handles:
    connection_handles = connection_handles.split()
    hci_handle = connection_handles[2+connection_handles.index(MAC)]
    verbose_print('%s: HCI connection handle: %s' %(IMU, hci_handle), level=3)
    run_command(["hcitool", "lecup", "--handle", hci_handle, "--min", "6", "--max", "7", "--latency", "0", "--timeout", "500"])
else:
    print("%s: ERROR, unable to set connection parameters" % IMU, file=sys.stderr)
    gatttool.kill()
    gatt.char_write(IMUConfig_UUID, IMU_mode_stop)
    sys.exit(1)

lastTimestamp = 0
lossCount = 0
systemtime = int(time.time() * 1000000)
lastIndex = 100000000000000

## Initialize log file
if os.path.exists(ARGS.logdir):
    if not os.path.isdir(ARGS.logdir):
        print('--logdir=%s is not a directory' % ARGS.logdir, file=sys.stderr)
        sys.exit(1)
else:
    os.makedirs(ARGS.logdir)

if logfilename is None:
    logfilename = 'port_%d.log' % PORT
logfilename = os.path.join(ARGS.logdir, logfilename)
logfile = open(logfilename, mode='a')
if not logfile:
    print("Could not open logfile %s for appending" % logfilename)
    sys.exit(1)

log_format = '%s;%s;%d;%d' + 13*';%f'
log_header = 'MAC;IMU;TIME;INDEX;QUATERNION_0;QUATERNION_1;QUATERNION_2;QUATERNION_3;ACCEL_ADXL355_0;ACCEL_ADXL355_1;ACCEL_ADXL355_2;ACCEL_BMI160_0;ACCEL_BMI160_1;ACCEL_BMI160_2;GYRO_0;GYRO_1;GYRO_2'
assert len(log_format.split(';')) == len(log_header.split(';'))
print(log_header, file=logfile)

payload = {
    'TIME': systemtime,
    IMU_OUT_INDEX: 0,
    IMU_OUT_QUATERNION: (0,0,0,0),
    IMU_OUT_QUATERNION+'_RAW': (0,0,0,0),
    IMU_OUT_ACCEL: (0,0,0),
    IMU_OUT_ACCEL_ADXL355: (0,0,0),
    IMU_OUT_ACCEL_BMI160: (0,0,0),
    IMU_OUT_ACCEL+'_RAW': (0,0,0),
    IMU_OUT_GYRO: (0,0,0),
    IMU_OUT_GYRO+'_RAW': (0,0,0),
    IMU_OUT_GYRO_ECC: (0,0,0),
}

packets_hex = []
last_data_bytes = bytearray(20)
lost_total = 0

skip_count = 0
still_count = 0
still_seconds = 0
still_phase = 1

try:
    while gatttool.poll() == None:
        outstr = gatttool.stdout.readline()
        systemtime = int(time.time() * 1000000)
        verbose_print(outstr.strip(), level=4)
        matches = re.search(notificationRegex, outstr)
        #batMatches = re.search(batteryNotifictionRegex, outstr)
        if matches:
            if skip_count < ARGS.skip:
                skip_count += 1
                verbose_print('Skipping packet %d of %d...' % (skip_count, ARGS.skip), level=2)
                continue
            if len(packets_hex) == 0:
                first_systemtime = systemtime
                print('%s: First packet received (skipped %d packets)' % (IMU, ARGS.skip))
            groups = matches.groups()
            # print(len(groups[0].strip().split()), 'bytes:', groups[0])
            data_bytes = bytearray.fromhex(groups[0])
            if (data_bytes[0] == 0 or data_bytes[0] == 3) and data_bytes[1:] == last_data_bytes[1:]:
                verbose_print('Strange packet duplication detected...', level=2)
                continue

            data = struct.unpack(IMU_output_struct_format, data_bytes) #format string: big endian unsigned short, unsigned char, 7h = hhhhhhh = 7 * short integer
            data = list(data)
            packets_hex.append(groups[0].strip())
            payload['TIME'] = systemtime
            for name in IMU_output_names:
                count = IMU_mode_dict[name]['count']
                scale = IMU_mode_dict[name]['scale']
                if count > 1:
                    raw = tuple(data[0:count])
                    payload[name+'_RAW'] = raw
                    payload[name] = tuple(x * scale for x in data[0:count])
                    if (32767 in raw or -32767 in raw or -32768 in raw) and (name != 'QUATERNION'):
                        t = (systemtime - first_systemtime) / 1000000
                        print('%s: WARNING t=%.2f: value saturated: %s' % (IMU, t, name))
                else:
                    payload[name+'_RAW'] = data[0]
                    payload[name] = data[0] * scale
                if name == IMU_OUT_ACCEL_ADXL355:
                    payload[IMU_OUT_ACCEL] = payload[IMU_OUT_ACCEL_ADXL355]
                    payload[IMU_OUT_ACCEL+'_RAW'] = payload[IMU_OUT_ACCEL_ADXL355+'_RAW']
                if name == IMU_OUT_ACCEL_BMI160:
                    payload[IMU_OUT_ACCEL] = payload[IMU_OUT_ACCEL_BMI160]
                    payload[IMU_OUT_ACCEL+'_RAW'] = payload[IMU_OUT_ACCEL_BMI160+'_RAW']
                del data[0:count]

            index_diff = payload[IMU_OUT_INDEX] - lastIndex
            if index_diff > 1:
                lost_total += index_diff
                print('%s: %d packets lost' % (IMU, index_diff))
                #for i in range(1,4): print(packets_hex[-i])

            if ARGS.verbose >= 3:
                #pretty = "% 6d:  (%+0.2f,%+0.2f,%+0.2f,%+0.2f)  (%+0.2f,%+0.2f,%+0.2f)" % (
                pretty = "i=% 6d:  (%+0.2f,%+0.2f,%+0.2f,%+0.2f)  (%+06.2f,%+06.2f,%+06.2f)  (%+0.2f,%+0.2f,%+0.2f)" % (
                    payload[IMU_OUT_INDEX],
                    payload[IMU_OUT_QUATERNION][0],
                    payload[IMU_OUT_QUATERNION][1],
                    payload[IMU_OUT_QUATERNION][2],
                    payload[IMU_OUT_QUATERNION][3],
                    payload[IMU_OUT_ACCEL][0],
                    payload[IMU_OUT_ACCEL][1],
                    payload[IMU_OUT_ACCEL][2],
                    payload[IMU_OUT_GYRO][0],
                    payload[IMU_OUT_GYRO][1],
                    payload[IMU_OUT_GYRO][2],
                )
                print(pretty)

            ## output IMU bytearray via UDP
            if ARGS.text:
                datagram = "%s;%d;%d;%f;%f;%f;%f;%f;%f;%f\n" % (
                    MAC,
                    0,
                    0,
                    payload[IMU_OUT_QUATERNION][1],
                    payload[IMU_OUT_QUATERNION][2],
                    payload[IMU_OUT_QUATERNION][3],
                    payload[IMU_OUT_QUATERNION][0],
                    payload[IMU_OUT_ACCEL][0],
                    payload[IMU_OUT_ACCEL][1],
                    payload[IMU_OUT_ACCEL][2],
                )
            else:
                datagram = struct.pack('>H7h',
                    payload[IMU_OUT_INDEX],
                    payload[IMU_OUT_QUATERNION+'_RAW'][0],
                    payload[IMU_OUT_QUATERNION+'_RAW'][1],
                    payload[IMU_OUT_QUATERNION+'_RAW'][2],
                    payload[IMU_OUT_QUATERNION+'_RAW'][3],
                    payload[IMU_OUT_ACCEL+'_RAW'][0],
                    payload[IMU_OUT_ACCEL+'_RAW'][1],
                    payload[IMU_OUT_ACCEL+'_RAW'][2]
                )
            #print(repr(datagram))
            udpSock.sendto(datagram, ("127.0.0.1", PORT)) 
            #udpSock.sendto(msg, ("127.0.0.1", PORT))
            #if PORT2 != None:
            #	udpSock.sendto(msg, ("127.0.0.1", PORT2))

            ## Log orientation to file
            log_msg = log_format % (
                MAC,
                IMU,
                payload['TIME'],
                payload[IMU_OUT_INDEX],
                payload[IMU_OUT_QUATERNION][0],
                payload[IMU_OUT_QUATERNION][1],
                payload[IMU_OUT_QUATERNION][2],
                payload[IMU_OUT_QUATERNION][3],
                payload[IMU_OUT_ACCEL_ADXL355][0],
                payload[IMU_OUT_ACCEL_ADXL355][1],
                payload[IMU_OUT_ACCEL_ADXL355][2],
                payload[IMU_OUT_ACCEL_BMI160][0],
                payload[IMU_OUT_ACCEL_BMI160][1],
                payload[IMU_OUT_ACCEL_BMI160][2],
                payload[IMU_OUT_GYRO][0],
                payload[IMU_OUT_GYRO][1],
                payload[IMU_OUT_GYRO][2],
            )
            # print(log_msg)
            print(log_msg, file=logfile)

            if ARGS.still > 0:
                is_still = sum(payload[IMU_OUT_GYRO]) < STILL_GYRO_THRESHOLD
                if is_still:
                    still_count += 1
                    if still_count % SAMPLE_RATE == 0:
                        still_seconds = still_count // SAMPLE_RATE
                        print('%s: Still phase %d, still for %d seconds' % (IMU, still_phase, still_seconds), end='\r')
                        if ARGS.beep and (still_seconds == ARGS.still or still_seconds == 40):
                            subprocess.Popen(['paplay', ARGS.beep])
                else:
                    if still_seconds >= ARGS.still:
                        still_phase += 1
                        print('')
                    still_count, still_seconds = 0, 0
        #elif batMatches:
            #groups = batMatches.groups()
            #batLevel = int(groups[0], 16)
            #print("%s: Battery at %d%%" % (IMU,batLevel))
        #else:
            #print("%s: %s" % (IMU,outstr))
            lastIndex = payload[IMU_OUT_INDEX] #um doppelte falsche Transmissions rauszuwerfen
            last_data_bytes = data_bytes
except KeyboardInterrupt as i:
    print('\nKeyboard Interrupt: Shutting down', file=sys.stderr)
except IOError as ioe:
    print('\nIO Error: Shutting down', file=sys.stderr)
    ARGS.verbose = False # prevent further IOErrors from verbose_print
except Exception as e:
    import traceback
    traceback.print_exc(file=sys.stderr)

if still_seconds > 0:
    print('')

if gatttool.poll() == None:
    print("%s: Killing gatttool" % IMU, file=sys.stderr)
    gatttool.kill()

verbose_print('\n%s: Stopping measurement' % IMU, file=sys.stderr)
gatt.char_write(IMUConfig_UUID, IMU_mode_stop) 

verbose_print("%s: Exiting" % IMU, file=sys.stderr)

if lost_total > 0:
    print('WARNING: %s Total packets lost: %d' % (IMU, lost_total), file=sys.stderr)
