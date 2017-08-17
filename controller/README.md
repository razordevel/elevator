# Controller

The hardware of the controller is an Arduino Due. 

To protect the input of the Arduino all +5V inputs have to be restricted to +3,3V to protect the Arduino. The Razorcat demonstrator uses a set of 220 kOhm and 100 kOhm resistors. The wiring of the resistors is documented here:
#TBD

The elevat directory contains the Arduino code for the elevator controller.

The tessy directory contains a TESSY project that is able to do unit tests for the controller code. The source directory contains the Arduino controller code in a raw C form to enable the unit tests.