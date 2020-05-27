#!/usr/bin/python

import sys
import mraa
import time

right_motor_enable = 15
left_motor_enable = 13

# Export the GPIO pin for use
right_motor_enable_pin = mraa.Gpio(right_motor_enable)
left_motor_enable_pin = mraa.Gpio(left_motor_enable)

# Small delay to allow udev rules to execute (necessary only on up)
time.sleep(0.1)

# Configure the pin direction
right_motor_enable_pin.dir(mraa.DIR_OUT)
left_motor_enable_pin.dir(mraa.DIR_OUT)
dev = mraa.Spi(0)
#dev.frequency(100000)
dev.mode(0)

# Loop
while True:
    tb = bytearray(1)
    tb[0] = 0
    left_motor_enable_pin.write(0)
    rxbuf = dev.write(tb)
    v = rxbuf[0] << 8
    rxbuf = dev.write(tb)
    v = v | rxbuf[0]
    left_motor_enable_pin.write(1)
    print ("v", v)

    txbuf = bytearray(2)
    txbuf[0] = 0x00
    txbuf[1] = 0x00

    left_motor_enable_pin.write(0)
    time.sleep(0.1)
    rxbuf = dev.write(txbuf)
    left_value = (rxbuf[0] << 8) | rxbuf[1]
    left_milliamps = (1000 * (left_value - 2048)) / 89.95
    left_motor_enable_pin.write(1)
    time.sleep(0.1)

    txbuf = bytearray(2)
    txbuf[0] = 0x00
    txbuf[1] = 0x00
    right_motor_enable_pin.write(0)
    time.sleep(0.1)
    rxbuf = dev.write(txbuf)
    right_value = (rxbuf[0] << 8) | rxbuf[1]
    right_milliamps = (1000 * (right_value - 2048)) / 89.95
    right_motor_enable_pin.write(1)
    time.sleep(0.1)

    print ("left:", hex(left_value), left_milliamps, "right:", hex(right_value), right_milliamps)

    refV = 3.0
    lsb =  refV / 4096
    lmv = (left_value - 2048) * lsb * 1000
    lamps = lmv / 66
    print ("left_value", left_value, "left mv", lmv, "lamps", lamps)
    time.sleep(0.5)

    # # Turn the LED on and wait for 0.5 seconds
    # pin.write(1)
    # time.sleep(0.5)
    # # Turn the LED off and wait for 0.5 seconds
    # pin.write(0)
    # time.sleep(0.5)
