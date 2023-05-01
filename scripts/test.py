#!/usr/bin/env python3

import RPi.GPIO as gpio
import time

def main():
	gpio.setmode(gpio.BCM)
	gpio.setup(23, gpio.OUT)
	gpio.setup(24, gpio.OUT)
	gpio.setup(25, gpio.OUT)

	gpio.output(23, gpio.LOW)
	gpio.output(24, gpio.LOW)
	gpio.output(25, gpio.LOW)

	# while True:
	# 	print('1')
	# 	gpio.output(23, gpio.HIGH)
	# 	gpio.output(24, gpio.LOW)
	# 	gpio.output(25, gpio.LOW)
	# 	time.sleep(1)
	# 	print('2')
	# 	gpio.output(23, gpio.LOW)
	# 	gpio.output(24, gpio.HIGH)
	# 	gpio.output(25, gpio.LOW)
	# 	time.sleep(1)
	# 	print('3')
	# 	gpio.output(23, gpio.LOW)
	# 	gpio.output(24, gpio.LOW)
	# 	gpio.output(25, gpio.HIGH)
	# 	time.sleep(1)
	
if __name__ == '__main__':
    main()