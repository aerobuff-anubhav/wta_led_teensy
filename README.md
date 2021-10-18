# LED with 32 Bit Teensy&reg; Boards

Following isntructions have been logged for [teensy41]; however, other boards would have similar installation steps.

## Installation
- Create udev rules for Teensy on the powering computer/local machine if not already present
```
$ cd /etc/udev/rules.d/
$ sudo touch 00-teensy.rules
$ sudo gedit 00-teensy.rules
```
- Copy the [UDEV Rules for Teensy boards]. More details for Teensy loaders can be be found at - [Using The Teensy Loader on Ubuntu Linux]
- Install rosserial
```
$ sudo apt install ros-melodic-rosserial-arduino
$ sudo apt install ros-melodic-rosserial-client
```
- Clone the repo to your local machine
```
git clone http://192.168.1.101/distributed-wta/wta_led_teensy.git
```
- Build the catkin package
- Open the led.ino file in Arduino IDE
- Select the correct board. <br>
Tools &#8594; Board &#8594; Teensyduino &#8594; Teensy 4.1
- Upload the code to Teensy.

## Execution
- Run following command in the terminal
```
roslaunch wta_led_teensy led.launch
```
- In split terminal, run following command and change data values to see different behavior
```
rostopic pub /assignment std_msgs/Int64 "data: 2"
```

The data value assignment is as follows if pins are connected the correct way:
```
Pin 13: White LED | Weapon 1
Pin 39: Red LED | Weapon 2
Pin 36: Blue LED | Weapon 3
Pin 33: Yellow LED | Weapon 4
```
**Note:** Pins are chosen arbitrarily. 

## Troubleshoot
\# | Problem | Solution | Notes
-- | -- | -- | --
1 | error: cannot convert 'usb_serial_class*' to 'HardwareSerial*' in assignment <br> iostream = &Serial;| append `\|\| defined (__IMXRT1062__)` to <br> AdruinoHardware.h | [github: Teensy 4.0 Support #455]



## References
1. [teensy41]
1. [UDEV Rules for Teensy boards]
1. [Using The Teensy Loader on Ubuntu Linux]
1. [github: Teensy 4.0 Support #455]

[//]: # (Hyperlinks to the References)
[teensy41]: https://www.pjrc.com/store/teensy41.html
[UDEV Rules for Teensy boards]: https://www.pjrc.com/teensy/00-teensy.rule
[Using The Teensy Loader on Ubuntu Linux]: https://www.pjrc.com/teensy/loader_linux.html
[github: Teensy 4.0 Support #455]: https://github.com/ros-drivers/rosserial/issues/455
