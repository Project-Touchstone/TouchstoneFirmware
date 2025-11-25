#ifndef MINBIT_SERIAL_NODE_H
#define MINBIT_SERIAL_NODE_H

#include <memory>
#include <string>
#include <Arduino.h>
#include "SerialStream.h"
#include "MinBiTCore.h"

class MinBiTSerialNode {
public:
    using ReadHandler = std::function<void(std::shared_ptr<MinBiTCore>, std::shared_ptr<MinBiTCore::Request>)>;

    MinBiTSerialNode(std::string name);
    ~MinBiTSerialNode();

    // Initialize and open the serial port
    bool begin(unsigned int baudRate);

    // Sets read handler
    void setReadHandler(ReadHandler readHandler);

    // Close the serial port
    void end();

    // Get the MinBiTCore protocol object
    std::shared_ptr<MinBiTCore> getProtocol();

    // Check if the serial port is open
    bool isOpen() const;

private:
    std::shared_ptr<SerialStream> serialStream;
    std::shared_ptr<MinBiTCore> protocol;

    ReadHandler readHandler;

    // Attaches protocol
    void attachProtocol();
};

#endif // MINBIT_SERIAL_NODE_H