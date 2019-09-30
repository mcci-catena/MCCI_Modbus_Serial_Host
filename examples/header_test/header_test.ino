/*

Module:  header_test.ino

Function:
    Just include the header file, to make sure things work.

Copyright notice and License:
    See LICENSE file accompanying this project.

Author:
    Terry Moore, MCCI Corporation   September 2019

*/

#include <MCCI_Modbus_Serial_Host.h>

static_assert(MCCI_MODBUS_SERIAL_HOST_VERSION >= MCCI_MODBUS_SERIAL_HOST_VERSION_CALC(0, 0, 1, 0));

void setup() {
    // do nothing.
}

void loop() {
    // do nothing.
}
