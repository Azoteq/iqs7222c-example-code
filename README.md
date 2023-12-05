# IQS7222C Arduino Example Code

## Introduction

This Arduino example code demonstrates how to set up and use the IQS7222C Integrated Circuit (IC). The IQS7222C is a 10 Channel Mutual / 8 Channel Self-capacitive and inductive Touch and Proximity Controller with I2C communications interface, configurable GPIOs, and low-power options.

This example code is specifically aimed at the IQS7222C Evaluation Kit (PCB number _AZP1192A2_).

This example code is intended for an Arduino Compatible board that uses 3.3 V logic, such as [Sparkfun's Pro Micro (3.3V, 8 MHz)](https://learn.sparkfun.com/tutorials/pro-micro--fio-v3-hookup-guide/hardware-overview-pro-micro). If a 5V logic Arduino board is used, a logic-level translator will be required between the Arduino-based board and the IQS7222C.

## Arduino Code Configuration

The behaviour and pin assignments of the Arduino code can be configured with the `#define` settings at the start of `iqs7222c-example-code.ino`.

Change the following pin assignments and parameters to suit your hardware:
```c
/*** Defines ***/
#define DEMO_IQS7222C_ADDR                      0x44
#define DEMO_IQS7222C_POWER_PIN                 4
#define DEMO_IQS7222C_RDY_PIN                   7
```


* `DEMO_IQS7222C_ADDR` is the IQS7222C I2C Slave address. For more information, refer to the datasheet and application notes found on the [IQS7222C Product Page](https://www.azoteq.com/product/iqs7222c/).

* `DEMO_IQS7222C_POWER_PIN` can be used to power the IQS7222C directly from an Arduino GPIO. This parameter sets which pin to use. This is an optional setting and can be removed if the IQS7222C is powered from the VCC pin or an external power supply.

* `DEMO_IQS7222C_RDY_PIN` sets the pin assignment for the IQS7222C ready pin. This must support external interrupts. On the Sparkfun Pro Micro, pins 0, 1, 2, 3, and 7 support interrupts.

> :memo: **Note:** Please note that powering an IQS device directly from a GPIO is _generally_ not recommended. However, the `DEMO_IQS7222C_POWER_PIN` in this example could be used as an enable input to a voltage regulator.

## Example Code Flow Diagram

<p align="center">
  <img src="docs/images/flow-diagram.svg" />
</p>

## Sparkfun Board Library Installation

To use the Sparkfun Pro Micro, the Sparkfun Board Library must be installed
in the Arduino IDE.

Add the Sparkfun Board Library by opening Preferences (*File* > *Preferences*),
and paste the following URL into the "Additional Board Manager URLs" text box.

```
https://raw.githubusercontent.com/sparkfun/Arduino_Boards/master/IDE_Board_Manager/package_sparkfun_index.json
```

![board_manager_link](docs/images/arduino-board-manager-link.png)

Click "OK".
Then open the Board Manager under **Tools** > **Board** > **Boards Manager...**.

![board_manager](docs/images/board-manager.png)

Search for "sparkfun", and install "SparkFun AVR Boards by SparkFun".

![board-manager-sparkfun](docs/images/board-manager-sparkfun.png)

You can now select the "SparkFun Pro Micro" in the Board selection menu.

![select-pro-micro](docs/images/select-pro-micro.png)

Also be sure to select the "3.3 V, 8 MHz" version under *Tools* > *Processor*.

![select-3v3](docs/images/select-3v3.png)

Source: [Pro Micro Hookup Guide](https://learn.sparkfun.com/tutorials/pro-micro--fio-v3-hookup-guide)

## Serial Communication and Interface
The example code provides verbose serial feedback to aid in the
demonstration of start-up and operational functions. It also has two built-in
commands to demonstrate the IQS7222C's functionality. To use these built-in
commands, the Arduino code simply sends an 'f' or 'r' over the serial
interface.

`1 - "f\n" - Force open a communication(RDY) window`


`2 - "r\n" - Request a Software Reset during runtime`

It is important to take note of the newline ('\n') character that is needed to
complete any serial request. It can be activated in the built-in Arduino IDE
Serial monitor and is shown inside the blue rectangle in the figure below.

![iqs323_successful_serial](docs/images/iqs7222c_successful_serial.png)
