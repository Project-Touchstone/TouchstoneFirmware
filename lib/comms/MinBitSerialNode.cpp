#include "MinBiTSerialNode.h"

MinBiTSerialNode::MinBiTSerialNode(std::string name)
    : protocol(std::make_shared<MinBiTCore>(name, serialStream))
{
    protocol->setWriteMode(MinBiTCore::WriteMode::BULK);
    protocol->setRequestTimeout(1000);
}

MinBiTSerialNode::~MinBiTSerialNode() {
    end();
}

bool MinBiTSerialNode::begin(unsigned int baudRate) {
    Serial.begin(baudRate);
    attachProtocol();
    return true;
}

void MinBiTSerialNode::setReadHandler(ReadHandler readHander) {
    this->readHandler = readHandler;
}

void MinBiTSerialNode::attachProtocol() {
    protocol->setReadHandler([this](std::shared_ptr<MinBiTCore::Request> request) {
        if (readHandler) {
            readHandler(protocol, request);
        }
        protocol->fetchData();
    });
    protocol->fetchData();
}

void MinBiTSerialNode::end() {
    serialStream->close();
}

std::shared_ptr<MinBiTCore> MinBiTSerialNode::getProtocol() {
    return protocol;
}

bool MinBiTSerialNode::isOpen() const {
    return serialStream && serialStream->isOpen();
}