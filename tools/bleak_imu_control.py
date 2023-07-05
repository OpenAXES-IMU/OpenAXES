#!/usr/bin/env python3

# @file  bleak_imu_control.py
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


import asyncio
from bleak import BleakClient
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

IMUS = {
    'imu1': '18:04:ED:C4:75:35',
    'imu2': '18:04:ED:C4:75:34',
    'imu3': '18:04:ED:C4:75:45',
    'imu6': '18:04:ED:C4:75:05',
    'imu9': '18:04:ED:C4:75:1A',
}
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

UUID_base = "0000%s-0000-1000-8000-00805f9b34fb"
SensorData_UUID = UUID_base % '2a30'
IMUConfig_UUID = UUID_base % '2aad'
IMUBattery_UUID = UUID_base % '2a19'
IMUReset_UUID = UUID_base % '2a31'
IMUVersion_UUID = UUID_base % 'fff5'



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


arg_parser = argparse.ArgumentParser(description='Receive IMU packets via BLE, log them, and send on via UDP to localhost.')
arg_parser.add_argument('IMU', help='Name or BLE MAC address of the IMU. Known names are: ' + ', '.join(sorted(IMUS.keys())))
arg_parser.add_argument('PORT', nargs='?', default=1024, type=int, help='UDP port and name of the log file (default 1024)')
arg_parser.add_argument('--text', '--processing', '-t', help='Send UDP packets as text instead of binary. For compatibility with the dsense_vis processing app.', action='store_true')
arg_parser.add_argument('--hci', help='Name or MAC-prefix the hci adapter to use (default: "5C:F3:70:" to auto-detect USB-BT400)', default="5C:F3:70:")
arg_parser.add_argument('--reset', nargs='?', help='Reset bit mask for the IMU. Possible reset bits are: SENSORS==0x01, QUATERNION==0x02, INDEX==0x04. (default value 0x07)', default='0x00', const='0x07', type=single_byte, metavar='BITS')
arg_parser.add_argument('--reset-only', help='Reset the IMU (as with --reset=BITS), then exit', default='0x00', type=single_byte, metavar='BITS')
arg_parser.add_argument('--reset-sensors', help='Reset the IMU sensor ICs, then exit (same as --reset-only=0x01)', dest='reset_only', action='store_const', const=0x01)
arg_parser.add_argument('--no-reset', help='Don\'t reset the IMU before starting a measurement. Same as --reset=0', dest='reset', action='store_const', const=0)
arg_parser.add_argument('-b', '--battery', help='Show the battery status, then exit', action='store_true')
arg_parser.add_argument('--mode',
    help=('Select the outputs and mode bits for the IMU (default %s). ' % IMU_mode_default) +
    'Can be a hex number or a comma-separated list of any of the following: ' +
    ', '.join([info['name'] for info in IMU_mode_info]),
    default=IMU_mode_default)
arg_parser.add_argument('-v', '--verbose', help='Print more information (can be given multiple times to print even more information)', action='count', default=0)
arg_parser.add_argument('--version', help='Print IMU firmware version', action='store_true')
arg_parser.add_argument('-l', '--logfile', help='Filename of the logfile (default: log_$imu_$date.csv if IMU name is given or port_$port.log otherwise)', default=None)
arg_parser.add_argument('--logdir', help='In which folder to place the log file (default "logs")', default='logs')
arg_parser.add_argument('--skip', '-s', type=int, help='Skip the first SKIP packets, for example it there is packet loss at the beginning (default: 0).', default=0)
arg_parser.add_argument('--still', type=int, help='Count seconds during which the IMU is still. If still for at least STILL seconds, play a sound', default=0)
arg_parser.add_argument('--beep', help='Sound file to play when IMU is still, see --still (default: %s)' % STILL_BEEP_SOUND_FILE, default=STILL_BEEP_SOUND_FILE)

ARGS = arg_parser.parse_args()

SAMPLE_RATE = 100
INTERFACE = None
if ARGS.IMU in IMUS:
    IMU = ARGS.IMU
    MAC = IMUS[ARGS.IMU]
    now = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    logfilename = 'log_%s_%s.csv' % (ARGS.IMU, now)
else:
    MAC = ARGS.IMU
    IMU = IMUS_MAC.get(MAC, MAC)
PORT = ARGS.PORT

if ':' in ARGS.hci: # MAC prefix given. Do auto detection.
    output = subprocess.check_output("hcitool dev | grep -E '\\s%s' | cut -f2" % ARGS.hci, shell=True)
    if len(output) > 0:
        verbose_print("Auto-detected interface %s" % output)
        INTERFACE = output.strip()
    else:
        print("Interface auto-detection failed. Please connect a bluetooth adapter with a mac starting with %s." % ARGS.hci, file=sys.stderr)
        sys.exit(1)
else:
    INTERFACE = ARGS.hci

gatt = BleakClient(MAC)

# Global variables for stats output at the end
lost_total = 0
still_seconds = 0


async def main():
    connection_task = gatt.connect()

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
        await gatt.write_gatt_char(IMUReset_UUID, bytes([ARGS.reset]))
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

    char2_typedef = [
        ('int16_t', 'battery_current'),            # Battery current in steps of 156.25 uA per LSB (with 0.01 Ohm sense resistor)
        ('int16_t', 'battery_current_average'),    # Battery current (averaged over time) in steps of 156.25 uA per LSB (with 0.01 Ohm sense resistor)
        ('uint16_t', 'battery_voltage'),           # Battery voltage in steps of 0.078125 mV per LSB
        ('uint16_t', 'battery_capacity'),          # Battery remaining capacity in units of 0.5 mAh per LSB (with 0.01 Ohm sense resistor)
        ('uint16_t', 'battery_capacity_full'),     # Battery full capacity in units of 0.5 mAh per LSB (with 0.01 Ohm sense resistor)
        ('uint8_t', 'battery_percent'),            # Battery state of charge in percent
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


    if ARGS.version:
        firmware_version_hex = await gatt.read_gatt_char(IMUVersion_UUID)
        firmware_version = firmware_version_hex.decode('utf-8')
        print('%s: Firmware version %s' % (IMU, firmware_version))
        sys.exit(0)


    if ARGS.battery:
        bat_info_raw = await gatt.read_gatt_char(IMUBattery_UUID)
        verbose_print('%s: bat_info_raw = %s' % (IMU, bat_info_raw.hex(' ')), level=4)
        if len(bat_info_raw) > 1: # new firmware version
            battery_info = unpack_c_struct(char2_typedef, bytearray.fromhex(bat_info_raw))
            battery_level = battery_info['battery_percent']
            verbose_print(battery_info, level=3)
            charging = battery_info['battery_current'] > 0
            charging_str = ('' if charging else 'dis') + 'charging' 
            print('%s: battery_voltage = %.2f V' % (IMU, battery_info['battery_voltage'] * 0.078125 / 1000))
            print('%s: battery_current = %.2f mA (%s)' % (IMU, battery_info['battery_current'] * 156.25 / 1000, charging_str))
            print('%s: battery_current_average =  %.2f mA (%s)' % (IMU, battery_info['battery_current_average'] * 156.25 / 1000, charging_str))
            print('%s: battery_capacity =  %.2f mAh' % (IMU, battery_info['battery_capacity'] * 0.5))
            print('%s: battery_capacity_full =  %.2f mAh' % (IMU, battery_info['battery_capacity_full'] * 0.5))
        else: # old firmware version: single byte battery value
            battery_level = bat_info_raw[0]
        print('%s: battery_level = %s' % (IMU, battery_level))
        #sys.exit(0)



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

    packet_queue = asyncio.Queue()

    def imu_data_callback(uuid, data):
        systemtime = int(time.time() * 1000000)
        packet_queue.put_nowait((systemtime, data))

    verbose_print("%s: Enabling notification for SensorData" % IMU)
    await connection_task
    await gatt.start_notify(SensorData_UUID, imu_data_callback)
    #print("%s: Setting notify period" % IMU)
    #subprocess.check_call(["gatttool", "-i", INTERFACE, "-b", MAC, "--char-write-req", "-a", SensorNotifyPeriod_Handle, "-n", NotifyPeriod])

    print('%s: Starting Measurement (setting IMU mode)' % IMU)
    await gatt.write_gatt_char(IMUConfig_UUID, bytes.fromhex(IMU_mode_hex))

    #time.sleep(0.1)
    connection_handles = subprocess.check_output(['hcitool', 'con']).decode('utf-8')
    verbose_print(connection_handles, level=4)
    if MAC in connection_handles:
        connection_handles = connection_handles.split()
        hci_handle = connection_handles[2+connection_handles.index(MAC)]
        subprocess.check_call(["hcitool", "lecup", "--handle", hci_handle, "--min", "6", "--max", "7", "--latency", "0", "--timeout", "500"])
        verbose_print('%s: HCI connection handle: %s' %(IMU, hci_handle), level=3)
    else:
        print("%s: Warning, unable to set connection parameters" % IMU, file=sys.stderr)
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

    logfilename = ARGS.logfile # Might be None, that is checked below
    if logfilename is None:
        logfilename = 'port_%d.log' % PORT
    logfilename = os.path.join(ARGS.logdir, logfilename)
    logfile = open(logfilename, mode='a')
    if not logfile:
        print("Could not open logfile %s for appending" % logfilename)
        sys.exit(1)

    log_format = '%s;%s;%d;%d' + 13*';%f' + 3*';%d'
    log_header = 'MAC;IMU;TIME;INDEX;QUATERNION_0;QUATERNION_1;QUATERNION_2;QUATERNION_3;ACCEL_ADXL355_0;ACCEL_ADXL355_1;ACCEL_ADXL355_2;ACCEL_BMI160_0;ACCEL_BMI160_1;ACCEL_BMI160_2;GYRO_0;GYRO_1;GYRO_2;GYRO_ECC_0;GYRO_ECC_1;GYRO_ECC_2'
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

    exit_loop = False
    while not exit_loop:
        try:
            data_bytes: bytes
            systemtime, data_bytes = await packet_queue.get()
            verbose_print(data_bytes.hex(' '), level=4)
            if skip_count < ARGS.skip:
                skip_count += 1
                verbose_print('Skipping packet %d of %d...' % (skip_count, ARGS.skip), level=2)
                continue
            if len(packets_hex) == 0:
                first_systemtime = systemtime
                print('%s: First packet received (skipped %d packets)' % (IMU, ARGS.skip))
            # print(len(groups[0].strip().split()), 'bytes:', groups[0])
            if (data_bytes[0] == 0 or data_bytes[0] == 3) and data_bytes[1:] == last_data_bytes[1:]:
                verbose_print('Strange packet duplication detected...', level=2)
                continue

            data = struct.unpack(IMU_output_struct_format, data_bytes) #format string: big endian unsigned short, unsigned char, 7h = hhhhhhh = 7 * short integer
            data = list(data)
            packets_hex.append(data_bytes.hex(' '))
            payload['TIME'] = systemtime
            for name in IMU_output_names:
                count = IMU_mode_dict[name]['count']
                scale = IMU_mode_dict[name]['scale']
                if count > 1:
                    raw = tuple(data[0:count])
                    payload[name+'_RAW'] = raw
                    payload[name] = tuple(x * scale for x in data[0:count])
                    if 32767 in raw or -32767 in raw or -32768 in raw:
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
                print(str(index_diff) + ' packets lost')
                for i in range(1,4): print(packets_hex[-i])

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
                payload[IMU_OUT_GYRO_ECC][0],
                payload[IMU_OUT_GYRO_ECC][1],
                payload[IMU_OUT_GYRO_ECC][2],
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

            lastIndex = payload[IMU_OUT_INDEX] #um doppelte falsche Transmissions rauszuwerfen
            last_data_bytes = data_bytes
        except KeyboardInterrupt as i:
            exit_loop = True
            print('\nKeyboard Interrupt: Shutting down', file=sys.stderr)
        except IOError as ioe:
            exit_loop = True
            print('\nIO Error: Shutting down', file=sys.stderr)
            ARGS.verbose = False # prevent further IOErrors from verbose_print
        except Exception as e:
            exit_loop = True
            import traceback
            traceback.print_exc(file=sys.stderr)
    verbose_print('\n%s: Stopping measurement' % IMU, file=sys.stderr)
    await gatt.write_gatt_char(IMUConfig_UUID, bytes.fromhex(IMU_mode_stop))
    if still_seconds > 0:
        print('') # newline

loop = asyncio.get_event_loop()

# Side note: Apparently, async() will be deprecated in 3.4.4.
# See: https://docs.python.org/3.4/library/asyncio-task.html#asyncio.async
tasks = asyncio.gather(
    asyncio.ensure_future(main())
)



try:
    loop.run_until_complete(tasks)
    asyncio.run(main())
except KeyboardInterrupt as e:
    print("Caught keyboard interrupt. Canceling tasks...")
    tasks.cancel()
    gatt.write_gatt_char(IMUConfig_UUID, bytes.fromhex(IMU_mode_stop))
    loop.run_forever()
    tasks.exception()
finally:
    loop.close()

verbose_print("%s: Exiting" % IMU, file=sys.stderr)

if lost_total > 0:
    print('WARNING: %s Total packets lost: %d' % (IMU, lost_total), file=sys.stderr)
