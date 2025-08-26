#include "MinBiTSerialServer.h"

MinBiTSerialServer::MinBiTSerialServer(std::string name)
    : protocol(std::make_shared<MinBiTCore>(name, serialStream))
{
    protocol->setNodeType(MinBiTCore::NodeType::SERVER);
    protocol->setWriteMode(MinBiTCore::WriteMode::PACKET);
    protocol->setRequestTimeout(1000);
}

MinBiTSerialServer::~MinBiTSerialServer() {
    end();
}

bool MinBiTSerialServer::begin(unsigned int baudRate) {
    Serial.begin(baudRate);
    attachProtocol();
    return true;
}

void MinBiTSerialServer::setReadHandler(ReadHandler readHander) {
    this->readHandler = readHandler;
}

void MinBiTSerialServer::attachProtocol() {
    protocol->setReadHandler([this](std::shared_ptr<MinBiTCore::Request> request) {
        if (readHandler) {
            readHandler(protocol, request);
        }
        protocol->fetchData();
    });
    protocol->fetchData();
}

void MinBiTSerialServer::end() {
    serialStream->close();
}

std::shared_ptr<MinBiTCore> MinBiTSerialServer::getProtocol() {
    return protocol;
}

bool MinBiTSerialServer::isOpen() const {
    return serialStream && serialStream->isOpen();
}