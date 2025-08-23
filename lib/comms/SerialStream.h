#ifndef SERIAL_STREAM_H
#define SERIAL_STREAM_H

#include <Arduino.h>
#include <memory>

#include "IStream.h"

class SerialStream : public IStream {
public:
    explicit SerialStream();

    void write(const uint8_t* buffer, std::size_t length) override;
    void read(uint8_t* buffer, std::size_t length) override;
    uint8_t available() override;
    bool isOpen() const override;
    void close() override;
};

#endif