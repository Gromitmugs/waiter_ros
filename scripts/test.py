#!/usr/bin/env python3

import RPi.GPIO as gpio
import time

def main():
	gpio.setmode(gpio.BCM)
	gpio.setup(24, gpio.IN, pull_up_down=gpio.PUD_DOWN)
	# while True:
	# 	print('1')
	# 	gpio.output(24, gpio.HIGH)
	# 	time.sleep(1)
	# 	print('0')
	# 	gpio.output(24, gpio.LOW)
	# 	time.sleep(1)
	i = gpio.input(24) 
	while True:
		if gpio.input(24):
			print('1')
		else:
			print('0')

if __name__ == '__main__':
    main()