#!/bin/bash

JOYFILE=$1

if [ "$#" -ne 1 ]
then
  echo "Usage: ./usejoy.sh file"
  exit 1
fi

echo $JOYFILE copied to /var/lib/joystick/joystick.state and applied
cp $JOYFILE /var/lib/joystick/joystick.state
jscal-restore /dev/input/js0
