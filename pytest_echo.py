# SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: CC0-1.0
from time import sleep

import pytest
import serial.tools.list_ports
from pytest_embedded import Dut


def test_usb_device_serial_example(dut: Dut) -> None:

    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if (p.device == '/dev/ttyACM0'):      # Get the usb_serial_jtag port
            with serial.Serial(p.device) as s:
                s.write(b'hi, espressif\n')
                sleep(1)
                res = s.readline()
                assert b'hi, espressif' in res
                s.write(b'See you again!\n')
                sleep(1)
                res = s.readline()
                assert b'See you again!' in res
                s.write(b'Echo a very long buffer. Assume this buffer is very large and you can see whole buffer\n')
                sleep(1)
                res = s.readline()
                assert b'Echo a very long buffer. Assume this buffer is very large and you can see whole buffer' in res
                s.write(b'64 bytes buffer:-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-\n')
                sleep(1)
                res = s.readline()
                assert b'64 bytes buffer:-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-' in res

            return

    raise Exception('usb_serial_jtag port not found')

ports = list(serial.tools.list_ports.comports())
for p in ports:
    if (p.device == '/dev/ttyACM0'):      # Get the usb_serial_jtag port
        with serial.Serial(p.device) as s:
            s.write(b'hi, espressif\n')
            sleep(1)
            res = s.readline()
            print(res)
            s.write(b'See you again!\n')
            sleep(1)
            res = s.readline()
            print(res)
            
            s.write(b'Echo a very long buffer. Assume this buffer is very large and you can see whole buffer\n')
            sleep(1)
            res = s.readline()
            print(res)
            
            s.write(b'64 bytes buffer:-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-\n')
            sleep(1)
            res = s.readline()
            print(res)
            