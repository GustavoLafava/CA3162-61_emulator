##  Emulator to replace both Intersil CA3161(E)/CA3162(E) using Arduino on bare ATmega

This Arduino sketch emulates both Intersil CA3161 and CA3162 (also NTE2054/NTE2032, 
ECG2054/ECG2032 ) used in many old device as a simple three digit voltmeter.
It's written to run on bare ATmega88 or ATmega328 MCU, but can be easily ported to 
other Arduinos/MCUs.

Differences from the original CA2162(E):
* Emulator don't have differential input at this time and accepts only positive 
  voltage as input, i.e. works only in configurations where CA3162E pin 10 is 
  connected to ground.
* Zero and gain ajustment can be done only calibrated using the serial interface 
  (9600bps 8n1)
* No hold mode, no fast mode.
* ADC resolution depends on the reference voltage used. In case of ATmega8
  internal AREF (very inaccurate) it will be 3mV (or ~3 degrees of Celsius for
  SL-30).

Emulator requires minimum external components:
* Decoupling ATmega VCC and AREF pins to GND via 0.1uF ceramic caps is required.
* To be interfaced with the three BJT PNP transistors based digit drivers, it needs ~ 1K 
  resistors in transistor base lines.
* It's recommended to connect RESET pin to VCC using a 10K resistor.
* (OPTIONAL) You could use an external voltage reference to increment the ADC precision,
  like the LM4040 voltage reference diode.

## Calibration

To calibrate the emulator by voltage:

* Connect Arduino serial interface and use the command STATUS to get the uncalibrate
  voltage values.
* Connect your voltmeter to Arduino A0 pin (PC0 pin of ATmega, pin 23 on
  DIP28);
* Write down both values: the uncalibrate value and the one displayed on the voltmeter
  display;
* Repeat previous step 2-3 times, at different input values. See the table below for 
  example:

  | Display | Voltmeter, mV |
  |---------| --------------|
  |      83 |           100 |
  |     184 |           203 |
  |     307 |           329 |
  |     589 |           620 |
* Find linear approximation for this data. You could use Wolframalpha to calculate zero
  and gain by entering the data like this:
  [linear fit {83, 100}, {184, 203}, {307, 329}, {589, 620}](http://www.wolframalpha.com/input/?i=linear+fit+{83%2C+100}%2C+{184%2C+203}%2C+{307%2C+329}%2C+{589%2C+620}).
  You will get an expression like this: `1.02812 x + 14.0734`
* Use the multiplier from that expression and use the command CALIB to set both the 
  values:

  CALIB 1.02812 14.0734
* If the value displayed on the three segment is correct, save the calibration values in 
  the eeprom, using the command STORE

## External useful informations
* [CA3162 datasheet](http://www.intersil.com/content/dam/Intersil/documents/ca31/ca3162.pdf)
* [CA3161 datasheet](http://www.intersil.com/content/dam/Intersil/documents/ca31/ca3161.pdf)
* Using ATmega8 with internal 8MHz oscillator as minimal Arduino [tutorial](http://www.neonile.net/articles/atmega8-arduino-bootloader-optiboot) and [another one](http://todbot.com/blog/2009/05/26/minimal-arduino-with-atmega8/).
