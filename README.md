[![Build-ground-software](https://github.com/tim-balloon/TIMflight/actions/workflows/build-ground-software.yml/badge.svg)](https://github.com/tim-balloon/TIMflight/actions/workflows/build-ground-software.yml)

# Terahertz Intensity Mapper (TIM) flight and ground operations code

## What is TIM?

![TIM](./docs/img/TIM.png)

The [Terahertz Intensity Mapper (TIM)](https://arxiv.org/ftp/arxiv/papers/2009/2009.14340.pdf) experiment is a stratospheric balloon for studying the star formation history of the universe via [line intensity mapping](https://arxiv.org/pdf/1903.04496.pdf).

It is a 2m telescope sensitive to the far-IR wavelength range of the electromagnetic spectrum, and observes using a cryogenically-cooled MKID detector array to scan a patch of sky and measure the 3D spatial distribution of molecular gas via emission line tracers of star formation such as [CII], [NII], and [OIII]. To do this, it flies above most of the atmosphere in a gondola suspended under a balloon.

See the [subsystem summary](./docs/SubsystemSummary.md) for a simplified overview of the balloon's software subsystems.
