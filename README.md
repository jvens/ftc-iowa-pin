# FTC Iowa Pin

## Description
The FTC Iowa Pin is a small lapel pin in the shape of Iowa containing 24 low power LEDs making the FIRST logo.  The controler is a 8-bit PIC microcontroller made by Microchip.  The power comes from a single CR2032 battery giving the pin XX hours of continous use before the battery needs to be changed.  

## Hardware
The hardware can be split into three main sections: power, LED drivers, and MCU.  There are many ways that the circuit can be populated for various battery life to cost tradeoffs.  These will be better documented after the boards have been fully tested.  

### Power
The main power comes from a single CR2032 Lithium-Ion battery providing 225mA-hr of energy.  The battery supplies a voltage of 2-3 volts nominally which directly powers the MCU circuity (ToDo: add ability to use an LDO if I need a 1.8 V regulated supply).  The battery also drives the step up circuity which is made up of a boost regulator and optional charge pump.  The circuit contains pads for two different boost options and one charge pump options, and jumpers to choose between options.  The output of the step up circuit is 6 volts which is used to power the LED drivers.

### LED Drivers
The LEDs are grouped into 12 sets.  Each set has two LEDs in series.  The sets are grouped into two columns of six rows.  The columns connect Anodes and rows connect Cathodes of the LEDs.  Driving each Anode is a PMOS transistor.  The transistor can be used as a binary switch or an current limiting switch.  When used as a binary switch the pullup resistor turns the switch off when the MCU puts the line to high-Z, and the switch turns on when the MCU drives the line low.  When used as a current limiting switch the MCU outputs an analog voltage using its internal DAC which is amplified by the op-amps to drive the gate.  Each LED is turned on by driving its column high and its row low, and turned off by setting the row to high-Z.

### MCU
The MCU is a PIC16 made by Microchip.  It uses GPIOs to drive the LED rows, and either GPIO or DAC to drive the LED columns.  It also contains a UART Rx line for reprogramming, ICSP for initial programming, and power on reset circuit.  User interaction is done by a capacitive touch pad connecting to the MCU comparator.  The other side of the comparator is driven by the internal DAC to measure the charge time of the touch pad and parallel capacitance.

## Software
ToDo
