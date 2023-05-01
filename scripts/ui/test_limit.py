#!/usr/bin/env python3

import RPi.GPIO as gpio
import time

def main():
	gpio.setmode(gpio.BCM)
	gpio.setup(12, gpio.IN)

	while True:
		if gpio.input(12) == 1:
			print('1')
		else:
			print('0')
			
if __name__ == '__main__':
    main()