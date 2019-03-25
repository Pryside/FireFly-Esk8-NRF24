# FireFly-Esk8-NRF24

This is a modded Version of Solidgeek's FireFly Firmware with extra Features

Features:
- Action Based Information
    Example: Power and Speed while Riding, Voltage/Stats while standing
- A somewhat accurate Battery Level (where the throttle usually is)
- total Kilometer counter (Saves all your total stats without stressing the EEPROM)
    (saves everytime vesc gets turned off before the remote or every 2Km of ridden distance)
- total energy used meter (does the same as total KM counter)

STILL TO DO
- Better functions for calibrating your throttle

OPTIONAL
- 3-quickpress-hold on trigger for Cruise Control // not really needed

Basically the same as Solidgeeks awesome Firmware but with some nice upgrades :)


Additional Information:
Please check out the library file, I've added all the required libs there.

Also change the pipline hex code in the transmitter as well as reciever program, so you and your friends dont use the same channel in the end. basically replace any character of the pipeline code with 1-9 or A-F.

For those of you wondering how many times the statistics like used energy and ridden Km get saved. Basically everytime the vesc shuts down the voltage drops below a reasonable range (2.5V per cell) and the remote automatically saves the "progress". So always shutdown board, then remote to save your km. Otherwise every 2nd km gets saved. This of course can be changed but keep in mind that the atmega328 only allows for around 100.000 EEPROM write/read cycles. So with my program you're on the save side and your EEPROM should be ok.
