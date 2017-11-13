# location of joystick storage
/var/lib/joystick/joystick.state

# to configure a new joystick
```
# use ui to configure joystick
jstest-gtk
# save joystick settings to /var/lib/joystick/joystick.state
sudo jscal-store /dev/input/js0  
```
probably best to copy the above state file here 

# to load xbox one controller on bluetooth
```
sudo copy xbox_one_joystick.state /var/lib/joystick/joystick.state
sudo jscal-restore
```
or just
```
sudo ./usejoy.sh xbox_one_joystick.state
```
