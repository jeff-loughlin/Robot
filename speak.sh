#!/bin/bash
/usr/bin/espeak --stdout -s 110 "$1" | /usr/bin/aplay
#/usr/bin/espeak -s 110 "$1"
