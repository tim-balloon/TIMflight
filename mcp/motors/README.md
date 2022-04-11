# Motors

Evan C. Mayer, 10 April, 2022

`actuators.c`

High-level: calls functions defined in other files in this dir. [EZStepper motors](https://www.allmotion.com/New%20PDF%27s/EZ17_23/Command_Set.pdf) with attached encoders communicating over an RS485 bus:
* Secondary mirror actuator steppers driving linear actuators for focusing
* Payload elevation balance
* Locking mechanism to fix inner frame to outer frame orientation
* Half wave plate rotator (HWPR) (polarization optics) 
* Shutter for shielding cold optics from sun
* Valves for cryogenics (He4 pot, pump valves)

`balance.c`

[EZStepper motors](https://www.allmotion.com/New%20PDF%27s/EZ17_23/Command_Set.pdf) with attached encoders communicating over an RS485 bus:
* Implementation of payload elevation balance

`cryovalves.c`

[EZStepper motors](https://www.allmotion.com/New%20PDF%27s/EZ17_23/Command_Set.pdf) with attached encoders communicating over an RS485 bus:
* Implementation of valves for cryogenics

`ec_motors.c`

Non-stepper motors commanding via EtherCat.
* Reaction wheel (rw) for azimuth pointing
* Pivot motor (piv) for transferring angular momentum to flight train (balloon) to avoid saturating reaction wheel angular rate capability
* Elevation drive

`ezstep.c`

EZStepper communication utility functions.

`hwpr.c`

Half-wave plate rotator control. An optical component that rotates.

`motors.c`

Mix of calculating control quantities for non-stepper motor drives and logging. Current settings and PID stuff.

`xystage.c`

Another EZStepper implementation file. Not at all clear where the XY stage is, what it's for, whether it's used in flight or is only for ground tests.