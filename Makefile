# base makefile for all projects

ARDUINO_DIR = /home/ildus/Arduino/arduino-1.6.12

BOARD_TAG    = mega
BOARD_SUB    = atmega2560
USER_LIB_PATH := $(realpath ./libs)

include /usr/share/arduino/Arduino.mk
