# RedBoard Python Code

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
# Set by hue, saturation, value. Values of 0.1 are good for non-blinding!
# Hue, Saturation and Value are all from 0.0 to 1.0, Hue of 0 is red, so
# to create a colour wheel over ten seconds we can do this:
from time import sleep
for h in range(0,100):
    r.set_led(h/100, 1, 0.1)
    sleep(0.1)
# If you want white, that's just any colour with saturation set to 0:
r.set_led(0, 0, 0.1)
# If you don't like your eyes, just set value to 1 and look closely at the LED
# (please don't do this!)
```

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