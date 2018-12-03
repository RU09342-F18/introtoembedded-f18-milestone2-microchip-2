# Description

In this milestone, the simple problem of controlling the temperature in a closed loop system was observed and solved. The purpose of this milestone was to use a thermistor, an MSP430F5529, and a fan to regulate the temperature of the thermistor. This was achieved using the on board analog to digital converter on the MSP430 and using PWM to power the fan in correspondence with the closed loop feedback provided by the thermistor.

The code resides in the folder *MSP430F5529* and the report is *Milestone_2__Microchip_.pdf*.

# Setup

One 12 Volt power supply is used to both drive a cooling fan and power the voltage regulator. The cooling fan speed is controlled by a low-side driver. This is then pointed at the voltage regulator. The voltage regulator is setup to generate heat driving a high wattage load (2 Watts). A NTC thermistor was used to measure the temperature of the voltage regulator. It is important to maintain good thermal contact between the regulator and thermistor to accurately measure the temperature, so the use of thermal paste is suggested.

# Using the software

To interface with the board, the USB connection was utilized. Serial data running at 9600 bps was transmitted over USB to and from the computer. The board transmitted the temperature read from the thermistor in degrees Celcius with two decimal places of precision. The format is ASCII, and each line ended with a newline character.

When transmitting data from the computer to the board, the raw data value is used. A single byte is used to set the target temperature of the voltage regulator in degrees Celcius. No newline characters or other delimiting bytes were used.
