#include "RUDPSerialNode.h"

RUDPSerialNode::RUDPSerialNode(std::string name)
    : protocol(std::make_shared<RUDPCore>(name)) {}

RUDPSerialNode::~RUDPSerialNode() {
    end();
}

bool RUDPSerialNode::begin(unsigned int baudRate) {
    Serial.begin(baudRate);
    protocol->attachStream(serialStream);
    return true;
}

void RUDPSerialNode::updateDate() {
    protocol->updateData();
}

void RUDPSerialNode::end() {
    serialStream->close();
}

std::shared_ptr<RUDPCore> RUDPSerialNode::getProtocol() {
    return protocol;
}

bool RUDPSerialNode::isOpen() const {
    return serialStream && serialStream->isOpen();
}