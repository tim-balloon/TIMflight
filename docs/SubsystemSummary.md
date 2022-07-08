# Gondola Subsystems

The [gondola design](https://arxiv.org/ftp/arxiv/papers/2009/2009.14340.pdf) (see section IIH onward) is heritage [BLAST-TNG](https://arxiv.org/pdf/1808.08597.pdf). 

## Hardware

### Detectors

The TIM camera comprises two arrays of [microwave kinetic inductance detectors (MKIDs)](https://en.wikipedia.org/wiki/Kinetic_inductance_detector), which are used to observe the sky between 240 and 420 µm and are coupled to the flight computers through the (planned) [radio frequency system-on-chip, RFSoC](https://agenda.infn.it/event/15448/contributions/95719/attachments/65674/80041/LTD_poster_Sinclair1.pdf) which communicates via a REDIS server hosted on the RFSoC for commanding and a UDP data stream to the flight computers for science data storage and telemetry downlink. In detail, these detectors are wired such that they create a superconducting notch filter to ground with a resonant frequency set by the particular geometry of the KID (capacitance) and the loading conditions (kinetic inductance). The strength of these detectors comes from the ability to string hundreds or thousands of these in parallel along a single microwave feedline with the resonant frequencies set during [fabrication](https://arxiv.org/abs/1803.04275) and spaced appropriately so as to avoid resonator "collisions" in frequency space. The detectors are "read out" by sending in combs of microwave probe tones aligned with the measured resonator frequencies and recording the output phase shift of the tone frequencies. These data are combined with "maps" of the resonator shapes to determine the fractional frequency shift ∆f/f which is then converted to optical power based on the material type. These probe tone and readout operations are then performed by the aforementioned RFSoCs.


### Attitude control
The attitude control system (ACS) comprises a slew of independent sensors, the outputs of which are fed into a Kalman filter for boresight pointing estimation. These sensors cover the configuration spaces of delicate or rugged, fast or slow, precise or coarse, and absolute or relative, and the importance of flying a wide range of types and specialties is to provide the best estimate of boresight position while hedging bets for device failures.
* two "star cameras" aligned parallel to the telescope boresight which image star patterns. These are combined with GPS information and [pattern correlation](astrometry.net) to provide absolute pointing information accurate to a few arcseconds at a cadence of roughly 20 seconds. These data are packetized and sent to the FCs from the control computers.
* two high-sample-rate, low-drift-rate 3-axis fiber gyroscopes provide fast (100hz) and relative angular velocity and acceleration information to the ACS which are integrated between star camera samples to estimate boresight position. Data comes in over a serial link as well as from the CSBF SIP interface.
* quad-constellation GPS provides position and velocity information for the payload to an accuracy of roughly 10 meters. These data are largely used to calculate the payload latitude, longitude, and local sidereal time (LST) which are fed into other sensors such as the star cameras.
* An array of pinhole sun sensors which image the sun onto a photoreceptor and the resulting current data is used to back out the position of the sun within the viewing angle of the sensor. These sensors provide a coarse absolute pointing measurement relative to the sun and are largely used as redundant sun avoidance systems.
* Encoders provide positioning feedback for the many motor systems, both pointing and otherwise, on the balloon.
* Inclinometers provide redundant and rugged low-speed (order 1Hz) and coarse (degrees) 2D measurements of both the inner and outer frame tilts relative to the local gravitational vector.
* A magnetometer provides a coarse and redundant measurement of the azimuthal position relative to the magnetic poles. Of note is that this sensor becomes increasingly unreliable the closer to the magnetic pole the payload drifts due to the high inclination angle of the magnetic field lines.

See [Section 3.1](https://arxiv.org/pdf/2012.01039.pdf) for more information.


### Thermometry

TIM is a cryogenic experiment, and as such careful measurement and control of temperatures is critical to operations. Additionally, electronics are generally bounded to operate between -40 and 85C, a requirement which necessitates tracking temperatures external to the receiver as well. BLAST featured three types of thermometry for the varying regimes.
* ruthenium oxide (ROX) thermometry are resistive devices which act as ultracold thermometers from ~5K down to <250mK.
* diode thermometry provides measurements of temperatures from ~2K to >400K.
* thermistors cover the external thermometry from <-50 to >100C.


### Motors
Azimuth control for the gondola is controlled by a reaction wheel aligned with the yaw axis of the gondola as well as a pivot motor attached to the balloon flight train. The reaction wheel is used for fine pointing, scanning, and stabilization while the pivot is used to "dump" angular momentum to the flight train and balloon itself to prevent the reaction wheel from saturating. Elevation is controlled via a brushless servo motor and can be locked using a pinning mechanism when needed.

See [Section 3.2](https://arxiv.org/pdf/2012.01039.pdf).


### Power
Solar panels couple to charge controller systems which dynamically modify the power draw to charge the payload battery system and maintain operations.

See [Section 4](https://arxiv.org/pdf/2012.01039.pdf).

### Flight Computers

Two low-power computers running a real-time variant of a Linux operating system operate simultaneously as a failsafe. One computer is in charge, and relinquishes control in the event of a software or hardware malfunction triggering a reboot. They run the flight software main control program, `mcp`, which coordinates the various software and hardware tasks. Additionally, the in-charge computer shares private sensor information with the secondary computer to maintain identical copies of all flight data written to parallel HDD sets.

## Software

[This slide deck](https://uploads-ssl.webflow.com/5f36afb5b4462e6fae2a317c/60f0456377cf175cf1e38561_StarSpec_Scientific_Ballooning_Brochure_July_2021_printed.pdf) from Javier Romualdez' company has the best high-level summary of the ground software that exists until we document what will be used on TIM.

# Ground Subsystems

## Hardware

## Software