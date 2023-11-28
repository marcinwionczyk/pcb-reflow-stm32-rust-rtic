# pcb-reflow-stm32-rust-rtic
This is how the reflow process look like:
https://youtu.be/pWV79pceqmc?si=kS1UuS4xw2wpDsws

## Software

Rust code for STM32F411RCT6 microcontroller to drive PCB reflow solder plate.
Code is based on https://github.com/gabrielvegamotta/PCB-Reflow/blob/main/V1-Arduino-PTH/Firmware/PCB_Reflow_V1.6.ino

Base of the code was generated with command: 
`cargo generate --git https://github.com/burrbull/stm32-template/`

In order to build it, you need to prepare your environment according to:
https://docs.rust-embedded.org/book/intro/install.html

and issue a command within this folder:
- `cargo build --release` for release version
- `cargo build` for unoptimised, debug version

Then you may run software on microcontroller with `cargo run` and flash it via STLink device with 
`cargo flash --release --chip STM32F411RCT6x`  
## Hardware
This is how the PCB top layer look like:
<img src="./docs/hardware/mainboard_v3_top.png">

Here you have [schematics in PDF](./docs/hardware/mainboard_v3.pdf)

Some major parts:
1. MAX6675 as a temperature sensor. You can get it for example from here:
https://botland.store/high-precision-temperature-sensors/12504-max6675-thermocouple-spi-temperature-sensor-5904422307868.html
2. TC2004A LCD Character screen: https://botland.store/alphanumeric-and-graphic-displays/19736-lcd-display-4x20-characters-green-with-connectors-justpi-5903351243087.html
3. PCF8574 as I2C port expander
4. SWT-11136 buttons: https://botland.store/tact-switch/11138-tact-switch-12x12mm-cap-black-5-pieces-5904422307530.html
5. 
6. micromatch connectors (I have plenty of them)

PCB was designed with Autodesk EAGLE. 
- [schematics](./docs/hardware/mainboard_v3.sch)
- [PCB project](./docs/hardware/mainboard_v3.brd)

I power it with +12V DC.
If you want to order a PCB, here you have [gerber files](./docs/hardware/mainboard_v3_2023-11-27.zip)