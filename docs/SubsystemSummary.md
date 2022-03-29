# Gondola Subsystems

The [gondola design](https://arxiv.org/ftp/arxiv/papers/2009/2009.14340.pdf) (see section IIH onward) is heritage [BLAST-TNG](https://arxiv.org/pdf/1808.08597.pdf). 

## Hardware

### Detectors

When an array of [microwave kinetic inductance detectors (MKIDs)](https://en.wikipedia.org/wiki/Kinetic_inductance_detector) is stimulated by a comb of tones from an onboard synthesizer, light from the science target modulates the resonant properties of the detectors, which are recorded by the radio frequency system-on-chip. Synthesizer commands go in, command acknowledgements and science data come out. These are routed to telemetry and onboard disk storage.

### Sensors

Many sensors provide state estimation for subsystem control and to a Kalman filter for attitude estimation:
* two "star cameras" aligned to telescope boresight image star patterns to bootstrap attitude estimates ("pointing")
* two high-sample-rate, low-drift-rate 3-axis fiber gyroscopes for angular rate estimates
* quad-constellation GPS for position/velocity information
* digital sun sensor for redundant sun position estimation for avoidance
* encoders for motor position feedback
* tilt sensors for redundant low-fidelity attitude estimation
* magnetometer for redundant low-fidelity attitude estimation

### Actuators

Azimuth attitude is controlled by a reaction wheel underneath the gondola and a pivot motor near the junction of the gondola support with the balloon. Elevation attitude is controlled by a brushless servo motor.

### Power

Solar panels charge onboard batteries.

### Flight Computers

Two low-power computers running a real-time variant of a Linux operating system operate simultaneously as a failsafe. One computer is in charge, and relinquishes control in the event of a software or hardware malfunction triggering a reboot. They run the flight software main control program, `mcp`, which coordinates the various software and hardware tasks.

## Software

# Ground Subsystems

## Hardware

## Software