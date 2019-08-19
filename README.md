# ESP32-NECIR
This tiny library useses the RMT module to receive telegrams according to the NEC protocol.

## Features
* Receiving telegrams according to the NEC protocol
* Detects repeated codes ( If button is kept pressed on the remote control)
* A ready to use default receive function for quick tests

## Usage
Create a new subfolder called 'components' in your project root directory (not in 'main').
Clone or add this repo as an submodule to 'components/'. As the repo name 'ESP32-NECIR' is just used to clarify that this library is for the ESP32 only, you may change the target folder name to something handy, e.g NECIR.

Git clone example (from project root):

git clone https://github.com/flo90/ESP32-NECIR.git components/NECIR

git submodule add https://github.com/flo90/ESP32-NECIR.git components/NECIR

A menuconfig entry called "NECIR" should appear in the "Component config" submenu.

After including necir.h (#include <necir.h>) just initialize the library by calling necir_init().

To use this library productively, reimplement the function necir_callback(...) somewhere. The included function is defined as 'weak' so that the reimplementation is prefered over it during linking.

## Configuration
The library is configurable through 'make menuconfig', it povides the following configuration options:

* Rx RMT channel - The RMT module channel used for receiving
* Rx RMT of memory blocks - Number of RMT module memory blocks used for receiving
* Rx GPIO - Input GPIO of received signal (connected to the receiver)


## Test results
This library was successfully testet with a 44 key remote which was shipped with a led controller.

## Known issues
Sometimes the repeated counter jumps in a very short time.
