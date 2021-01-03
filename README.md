[![Build Status](https://travis-ci.org/ECUality/ECUality.svg?branch=master)](https://travis-ci.org/ECUality/ECUality)

# ECUality

This project is about taking full control of a dated fuel-injection system, and making that process easier to repeat. Currently, the system is built to run the engine in a 1984 VW vanagon. I aim to create a platform that bears adaptation requiring a minimum of external tools. 

ECUality differs from other ECU projects in that I avoid relying on complex tuning software, favoring simple map-tuning algorithms that run whenever the engine and O2 sensor are warm. 

More info [here](https://hackaday.io/project/4622-ecuality1).

## Install

1. Install the [Arduino IDE](https://www.arduino.cc/en/Main/Donate).
2. Clone the repository.
3. Connect an Arduino Mega 2560 to your computer's USB port.
4. Start the Arduino IDE.
5. Open `ECUality.ino` in the IDE.
6. Click "Upload". This will build the project, and upload the compiled firmware to your Arduino.

## License
[GNU GPLv3](LICENSE.txt)

## Dependencies
Platform: 8-bit Arduino Mega 2560 (The chip is AtMega 2560), atop the ECU shield specified in the shcematic and layout files.  
Build: Arduino IDE (uses gcc toolchain)
