# RedBoard Python Code

**Version 0.1.7, 21st September 2020, @Approx_Eng**

This is a clean re-write of the Python code to drive the Red Robotics RedBoard 
(get one from https://www.tindie.com/products/redrobotics/redboard/) and its 
associated I2C connected display and motor expansion boards. Proper docs will 
appear in time, but for now:

## Installing the library

I strongly recommend using a virtual environment, and never installing with `sudo` - this board and software
doesn't need it and it'll just trip you up later!

### Install pre-requisite system packages

From a clean headless raspbian installation you'll need the following additional packages installed,
use this command to get them all in one go:

```bash
> sudo apt install libtiff5 libopenjp2-7-dev fonts-dejavu libpython3-dev libjpeg-dev pigpiod
```

### Get with pip

Simplest option:

```bash
> pip3 install redboard
```

Note - this code requires Python 3, at least version 3.6.

### Building from source

If you want to fiddle with the library you can clone it from here and use it directly:

```bash
> git clone https://github.com/ApproxEng/RedBoard.git
> virtualenv --python=python3.7 venv
> source venv/bin/activate
> cd RedBoard/src/python
> python setup.py develop
```

## Initialisation

```python
import redboard
r = redboard.RedBoard()
```

## Driving motors

```python
# Set first built-in motor driver to full speed forwards
r.m0 = 1
# Set second built-in motor driver to half speed reverse
r.m1 = 0.5
# Print out current motor speeds
print(r.m0, r.m1)
# If you've got an MX2 expansion connected you can address those motors too.
# i.e. to drive the first motor on the first connected MX2 board
r.m2 = 1
# You can use 'motor' rather than 'm' if you want to be more verbose
r.motor0 = 1
# You can configure motors to invert their direction, handy if your wiring
# wasn't perfect the first time:
r.m1_invert = True
```

## Driving servos

```python
# Set the servo on pin 20 to neutral position
r.s20 = 0
# Set the servo on pin 22 to extreme negative position
r.s22 = -1
# Get the pulse width range for the servo on pin 21
print(r.s21_config)
# prints (500, 2500)
# Set the pulse width range for that servo to use a bit less of the potential
# maximum range, servos vary in what range they'll actually accept
r.s21_config = 600, 2400
# read the current servo position
print(r.s21)
```

## Reading from the ADC channels

```python
# Read the battery level
print(r.adc0)
# Read the other channels
print(r.adc1, r.adc2, r.adc3)
# Read the divisor used to convert readings into voltage
print(r.adc0_divisor)
# (prints 1100 for the battery channel, 7891 for the others by default)
# Set the divisor, use this to calibrate the ADCs if required
r.adc0_divisor=1200
```

## Set the onboard LED

```python
# Set by hue, saturation, value. Hue is a circular quantity, so you can use any value (it'll
# be converted into the 0.0-1.0 range by taking only the positive fractional part). Saturation
# and value should be in 0.0-1.0 range.
from time import sleep
for h in range(0,100):
    r.led0 = h/100, 1, 0.1
    sleep(0.1)
# If you want white, that's just any colour with saturation set to 0:
r.led0 = 0, 0, 1
# Change the brightness, the default value is 0.6 which is reasonable for most uses, a value of
# 1.0 can be quite dazzling.
r.led0_brightness = 0.4
# Change the saturation compensation, higher values make paler colours more colourful,
# the default value is 2, setting to 0 will only allow you to show shades of white
r.led0_saturation = 3
# Change the gamma compensation, this changes how brightness is mapped onto a curve, and
# is set to 2 for the board by default, this should give you a smooth brightness change but you
# can override it if you really want to for some reason
r.led0_gamma = 1.9
# You can also set colour from any of the named CSS4 colours
r.led0 = 'pink'
# If for some reason you want to get or set as RGB rather than HSV you can, i.e. for red:
# Note that gamma, saturation, and brightness are still applied if set, so using RGB values
# is exactly equivalent to using the corresponding HSV ones, i.e. in this case RGB of 1,0,0
# is exactly the same as HSV of 0,1,1
r.led0_rgb = 1.0, 0.0, 0.0
```

You can find the available named colours at https://www.w3.org/TR/css-color-4/#named-colors

## Showing the configuration and test GUI

The redboard library comes with a text-based GUI which allows you to drive any attached motors
and servos, show the values of any analogue inputs, and calibrate all of these. Motors can be
set to reverse or not, servos can have their maximum and minimum pulse widths configured, and
ADC channels can be set against a known voltage. This configuration is printed to the console
when you exit the GUI as a YAML file, you can then load this into a Python dict and set the
config property on the board in your own code from that dict.

```python
> redboard-gui
```

Note that this command might not appear if you have an early version of `pip` installed, if
you've installed the redboard package but don't have this option try updating pip, uninstalling
and reinstalling the `redboard` package.

## Using MX2 motor expansion boards without the RedBoard

```python
# Initialise an MX2 board, default address is 0x30, no jumpers bridged
mx = redboard.MX2()
# Provides motor 0 and 1, use the same way as the RedBoard
mx.m0 = 1.0
mx.m1_invert = True
print(mx.m1)
# etc etc...
```

## Use an attached OLED expansion to show text

First install some fonts!

```bash
> sudo apt install fonts-dejavu
```

Now create a display object and use it:

```python
d = redboard.Display()
d.text(line1='Hello', line2='Python', line3='World')
```

## Using an attached servo expander

Because the really useful header on the RedBoard+ has exactly the right pins to
add one of those 16 channel servo expanders based on the PCA9685, I've added
a driver for that board to this library:

```python
expander = redboard.PCA9685()
expander.servo3 = 0.4
```

This works exactly the same way as the built-in servo properties on the
regular RedBoard driver class, it just gives you another 16 of them to
play with (0 to 15). Remember to power your expansion board properly,
16 servos can pull quite a lot of current!

You can also launch a GUI similar to that for the main RedBoard for the
servo expander with `pca9685-gui`, this will allow you to test all the
servos on the expander and configure minimum and maximum pulse ranges
for each one. Because the driver uses hwsupport, it handles the same
configuration mechanism as the RedBoard for save and load of per-servo
settings.

Use with boards like this one from AdaFruit (or similar, much cheaper, ones from ebay!) 
https://learn.adafruit.com/16-channel-pwm-servo-driver 

## Save and restore configuration

The `RedBoard` object exposes its configuration as a `dict` containing the following:

* Motor invert true / false per motor
* ADC divisors per ADC
* Servo pulse ranges per servo pin

To get the current config as a python `dict`:

```python
r.config
```

To set the config from a `dict` you just do:

```python
r.config = new_config
```

Typically you'd save this out to a YAML file or similar and use it to
load your configuration rather than having to put it into your code
every time, but you could just set it directly.