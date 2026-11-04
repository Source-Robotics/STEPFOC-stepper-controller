# STEPFOC Stepper controller


[![License: GPLv3](https://img.shields.io/badge/license-GPLv3-blue)](https://opensource.org/license/gpl-3-0/) ![Issues](https://img.shields.io/github/issues/Source-Robotics/STEPFOC-stepper-controller) ![release](https://img.shields.io/github/v/release/Source-Robotics/STEPFOC-stepper-controller)

<img src="Photos/STEPFOC PCB.png" alt="drawing" width="5000"/>

## 📖 Project Overview

**STEPFOC** is a FOC based stepper motor controller designed to deliver affordable, high-performance hardware paired with powerful open-source software. Its mission is to transform low-cost stepper motors into high-performance robotic servos, enabling advanced robotics applications without the high price tag.

The controller features a 14-bit magnetic encoder, inline current sensing, CAN and UART interfaces, and supports position, velocity, torque, and impedance control modes.


> [!NOTE]
> Optimized for NEMA17 format but it can also be attached to nema 23 and 14 steppers!


> [!IMPORTANT]
>     **STEPFOC stepper driver is built on the same hardware and software foundation as the Spectral Micro BLDC driver. As a result, some references or images in this documentation may still show the Spectral Micro, since both drivers share about 99% of their features. They also use the same CAN and UART command sets, with only minor differences in UART-based calibration and setup procedures.**

## Where to buy? 

You can buy the STEPFOC stepper controller here: <br />
 https://source-robotics.com/products/stepfoc-stepper-controller

## 💾Firmware 
STEPFOC firmware is located [here](https://github.com/Source-Robotics/STEPFOC-stepper-controller/tree/main/STEPFOC%20firmware). To install it follow [this guide!](https://docs.source-robotics.com/STEPFOC-docs/latest/07-firmware/update/)


## 📚Documentation:

- [Motor hardware setup guide](https://youtu.be/60UMc95H8iE)
- [Official website](https://source-robotics.com/products/spectral-micro-bldc-controller)
- [DOCS](https://docs.source-robotics.com/STEPFOC-docs/) Offers great starting guides with project examples + code 


### API and control
- [GUI software: Motor GUI](https://motorgui.com/), in the browser ([guide](https://docs.source-robotics.com/STEPFOC-docs/latest/05-setup/motor-gui/))
- [Python API & Examples](https://github.com/Source-Robotics/Spectral-BLDC-Python/tree/main)
- [Source Robotics toolbox](https://github.com/Source-Robotics/Source-Robotics-Toolbox/tree/main) 

## 🌐 More about STEPFOC drivers

| YouTube | Instagram | Twitter | LinkedIn |
|--------|-----------|---------|----------|
| [![YouTube](https://img.shields.io/badge/YouTube-FF0000?logo=youtube&logoColor=white)](https://www.youtube.com/@source-robotics) | [![Instagram](https://img.shields.io/badge/Instagram-E4405F?logo=instagram&logoColor=white)](https://www.instagram.com/source_robotics/) | [![Twitter](https://img.shields.io/badge/Twitter-1DA1F2?logo=twitter&logoColor=white)](https://twitter.com/SourceRobotics) | [![LinkedIn](https://img.shields.io/badge/LinkedIn-0A66C2?logo=linkedin&logoColor=white)](https://www.linkedin.com/company/source-robotics) |


| Discord | Forum | Hackaday | Blog |
|--------|-------|------| ------|
| [![Discord](https://img.shields.io/badge/Discord-5865F2?logo=discord&logoColor=white)](https://discord.com/invite/prjUvjmGpZ) | [![Discussions](https://img.shields.io/badge/Discussions-0A0A0A?logo=github)](https://github.com/orgs/Source-Robotics/discussions) | [![Hackaday](https://img.shields.io/badge/Hackaday-000000?logo=hackaday&logoColor=white)](https://hackaday.io/project/191860-parol6-desktop-robotic-arm) | [![Blog](https://img.shields.io/badge/Blog-Source%20Robotics-blue)](https://source-robotics.com/blogs/blog) 



## ⚠️ Safety, Liability & Disclaimer
1. The software and hardware are still in development and may contain bugs, errors, or incomplete features.
2. Users are encouraged to use this software and hardware responsibly and at their own risk.

## 🛡️ Licensing

Project is under GPLv3 Licence
