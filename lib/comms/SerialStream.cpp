#include "SerialStream.h"

SerialStream::SerialStream() {}

void SerialStream::write(const uint8_t* buffer, std::size_t length) {
    Serial.write(buffer, length);
}

void SerialStream::read(uint8_t* buffer, std::size_t length) {
    Serial.readBytes(buffer, length);
}

uint8_t SerialStream::available() {
    return Serial.available();
}

bool SerialStream::isOpen() const {
    return Serial;
}

void SerialStream::close() {
    if (Serial) {
        Serial.end();
    }
}
