#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 13 03:03:39 2021

@author: jfoo
"""

'''
import busio
import adafruit_amg88xx

#The way to create an I2C object depends on the board you are using. For boards with labeled SCL and SDA pins, you can:

import board

#You can also use pins defined by the onboard microcontroller through the microcontroller.pin module.
#Now, to initialize the I2C bus:
#Once you have created the I2C interface object, you can use it to instantiate the AMG88xx object
#You can also optionally use the alternate i2c address (make sure to solder the jumper on the back of the board if you want to do this)
#Pixels can be then be read by doing:
    
i2c_bus = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c_bus)
amg = adafruit_amg88xx.AMG88XX(i2c_bus, addr=0x69)

try: 
    while 1:
        print(amg.pixels)
except Exception as e:
    print(e)
    
    
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
'''


import time
from time import sleep
import busio
import board
import numpy as np
import adafruit_amg88xx

i2c = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c)

'''
while True:
    print('Rows:')
    rows = amg.pixels
    for row in rows:
        print(row)
    print()
    print('Cols:')
    cols = np.transpose(rows)
    for col in cols:
        print(col)
    
    sleep(1)
    print("\n")
    time.sleep(1)
'''

while True:
    raw = amg.pixels
    corrected = np.rot90(raw, 3)
    for row in corrected:
        print(['{0:.1f}'.format(temp) for temp in row])
        print("")
    print("\n")
    time.sleep(1)