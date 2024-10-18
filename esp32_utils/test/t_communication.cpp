#include <Arduino.h>
#include "serial_utils/serial_utils.h"

void setup() {
    setupSerial();
}

void loop() {
    String message = readMessage();

    if (message != "") {
        if (verifyChecksum(message)) sendMessage("R,OK");
        else sendMessage("R,Error");
    }
}
