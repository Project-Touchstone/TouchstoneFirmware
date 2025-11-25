// ...existing code...
#include "RUDPCore.h"

#include <algorithm>
#include <cassert>

using namespace std::chrono_literals;

/// Packet implementation

RUDPCore::Packet::Packet(uint8_t sequenceNum, uint8_t header)
    : sequenceNum(sequenceNum), header(header), payloadLength(0), payload(), readPos(0) {}

RUDPCore::Packet::Packet(uint8_t sequenceNum, uint8_t header, const std::vector<uint8_t>& payload)
    : sequenceNum(sequenceNum), header(header), payloadLength(payload.size()), payload(payload), readPos(0) {}

std::vector<uint8_t> RUDPCore::Packet::toBytes() const {
    std::vector<uint8_t> out;
    out.reserve(3 + payloadLength);
    out.push_back(static_cast<uint8_t>(sequenceNum));
    out.push_back(header);
    out.push_back(static_cast<uint8_t>(payloadLength));
    if (!payload.empty()) {
        out.insert(out.end(), payload.begin(), payload.end());
    }
    return out;
}

int64_t RUDPCore::Packet::getSequenceNum() const {
    return static_cast<int64_t>(sequenceNum);
}

uint8_t RUDPCore::Packet::getHeader() const {
    return header;
}

std::size_t RUDPCore::Packet::GetPayloadLength() const {
    return payloadLength;
}

void RUDPCore::Packet::writeBytes(const uint8_t* buffer, std::size_t length) {
    if (length == 0) return;
    payload.insert(payload.end(), buffer, buffer + length);
    payloadLength = payload.size();
}

void RUDPCore::Packet::writeByte(uint8_t value) {
    payload.push_back(value);
    payloadLength = payload.size();
}

void RUDPCore::Packet::writeFloat(float value) {
    uint8_t buf[sizeof(float)];
    std::memcpy(buf, &value, sizeof(float));
    writeBytes(buf, sizeof(float));
}

void RUDPCore::Packet::writeInt16(int16_t data) {
    uint8_t buf[2];
    buf[0] = static_cast<uint8_t>(data & 0xFF);
    buf[1] = static_cast<uint8_t>((data >> 8) & 0xFF);
    writeBytes(buf, 2);
}

uint8_t RUDPCore::Packet::readByte() {
    if (readPos >= payload.size()) return 0;
    return payload[readPos++];
}

uint8_t RUDPCore::Packet::peekByte() const {
    if (readPos >= payload.size()) return 0;
    return payload[readPos];
}

void RUDPCore::Packet::readBytes(uint8_t* buffer, std::size_t len) {
    if (len == 0) return;
    std::size_t avail = payload.size() - readPos;
    std::size_t toRead = std::min(len, avail);
    if (toRead) {
        std::memcpy(buffer, payload.data() + readPos, toRead);
        readPos += toRead;
    }
    // if requested more than available, zero the rest
    if (toRead < len) {
        std::memset(buffer + toRead, 0, len - toRead);
    }
}

float RUDPCore::Packet::readFloat() {
    float v = 0.0f;
    uint8_t buf[sizeof(float)];
    readBytes(buf, sizeof(float));
    std::memcpy(&v, buf, sizeof(float));
    return v;
}

int16_t RUDPCore::Packet::readInt16() {
    uint8_t buf[2] = {0,0};
    readBytes(buf, 2);
    int16_t val = static_cast<int16_t>(static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8));
    return val;
}

/// RUDPCore implementation

RUDPCore::RUDPCore(std::string name) : name(std::move(name)), stream(nullptr) {}

RUDPCore::~RUDPCore() {
    flush();
}

void RUDPCore::attachStream(std::shared_ptr<IStream> stream) {
    std::lock_guard<std::mutex> lock(dataMutex);
    this->stream = stream;
}

void RUDPCore::setReadHandler(ReadHandler handler) {
    std::lock_guard<std::mutex> lock(dataMutex);
    readHandler = std::move(handler);
}

void RUDPCore::setResponseHandler(uint8_t header, ReadHandler handler) {
    std::lock_guard<std::mutex> lock(responseMutex);
    responseHandlers[header] = std::move(handler);
}

void RUDPCore::clearResponseHandler(uint8_t header) {
    std::lock_guard<std::mutex> lock(responseMutex);
    responseHandlers.erase(header);
}

std::shared_ptr<RUDPCore::Packet> RUDPCore::waitForResponse(uint8_t header, uint32_t timeoutMs) {
    std::unique_lock<std::mutex> lock(responseMutex);
    // check if we already have a response
    auto it = lastResponseMap.find(header);
    if (it != lastResponseMap.end()) {
        auto pkt = it->second;
        lastResponseMap.erase(it);
        return pkt;
    }
    // wait for condition variable to be signalled with that header
    bool got = responseCv.wait_for(lock, std::chrono::milliseconds(timeoutMs), [&]() {
        return lastResponseMap.find(header) != lastResponseMap.end();
    });
    if (!got) return nullptr;
    auto pkt = lastResponseMap[header];
    lastResponseMap.erase(header);
    return pkt;
}

std::shared_ptr<IStream> RUDPCore::getStream() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return stream;
}

RUDPCore::Packet RUDPCore::createPacket(uint8_t header) {
    uint8_t seq = nextSeqNum.fetch_add(1);
    return Packet(seq, header);
}

void RUDPCore::holdPacket(const Packet& packet) {
    std::vector<uint8_t> bytes = packet.toBytes();
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        appendToWriteBuffer(bytes.data(), bytes.size());

        // Clears last response with same header
        lastResponseMap.erase(packet.getHeader());
    }
}

void RUDPCore::sendPacket(const Packet& packet) {
    holdPacket(packet);
    sendAll();
}

void RUDPCore::sendAll() {
    // attempt immediate send if stream available
    std::shared_ptr<IStream> s = getStream();
    if (s) {
        std::lock_guard<std::mutex> lock(dataMutex);
        if (!writeBuffer.empty()) {
            s->write(writeBuffer.data(), writeBuffer.size());
            writeBuffer.clear();
        }
    }
}

void RUDPCore::updateData() {
    // Pull bytes from stream into readBuffer
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        if (stream) {
            // read all available bytes
            std::size_t avail = stream->available();
            if (avail > 0) {
                std::vector<uint8_t> tmp(avail);
                std::size_t got = stream->read(tmp.data(), avail);
                if (got > 0) appendToReadBuffer(tmp.data(), got);
            }
        }
    }

    // Process complete packets in readBuffer
    while (true) {
        std::shared_ptr<Packet> pkt;
        {
            std::lock_guard<std::mutex> lock(dataMutex);
            if (!characterizePacket()) break;
            // we have a full packet in readBuffer; construct it
        
            std::vector<uint8_t> payload;
            payload.insert(payload.end(), readBuffer.begin() + 3, readBuffer.begin() + 3 + currPayloadLen);
            pkt = std::make_shared<Packet>(currSeqNum, currHeader, payload);

            // erase consumed bytes
            readBuffer.erase(readBuffer.begin(), readBuffer.begin() + 3 + currPayloadLen);
        }

        // Check if the packet is in order
        if (currSeqNum == expectedSeqNum) {
            //Send packet to handlers
            callHandlers(pkt);

            expectedSeqNum++;

            //Check for waiting out of order packets
            while (incomingPackets.find(expectedSeqNum) != incomingPackets.end()) {
                // Send packet to handlers
                callHandlers(incomingPackets[expectedSeqNum]);
                // Remove out of order packet
                incomingPackets.erase(expectedSeqNum);
                
                expectedSeqNum++;
            }
        } else {
            // Packet is out of order, add to dictionary for later processing
            incomingPackets[currSeqNum] = pkt;
        }
    }

    // Sends any data in write buffer if stream available
    sendAll();
}

void RUDPCore::callHandlers(std::shared_ptr<Packet> pkt) {
    uint8_t header = pkt->getHeader();
    // Calls a response handler if one exists
    auto it = responseHandlers.find(header);
    if (it != responseHandlers.end()) {
        auto handler = it->second;
        handler(pkt);
    }

    // Calls general read handler
    readHandler(pkt);

    // Updates last response map
    lastResponseMap[header] = pkt;
}

void RUDPCore::flush() {
    std::lock_guard<std::mutex> lock(dataMutex);
    readBuffer.clear();
    writeBuffer.clear();
    incomingPackets.clear();
    {
        std::lock_guard<std::mutex> rlock(responseMutex);
        lastResponseMap.clear();
    }
}

std::size_t RUDPCore::getReadBufferSize() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return readBuffer.size();
}

std::size_t RUDPCore::getWriteBufferSize() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return writeBuffer.size();
}

bool RUDPCore::characterizePacket() {
    if (newPacket) {
        // Must be called with dataMutex locked if used internally; it's safe to call without external lock
        if (readBuffer.size() < 3) return false; // need at least seq, header, len
        // Peek bytes
        currSeqNum = readBuffer[0];
        currHeader = readBuffer[1];
        currPayloadLen = readBuffer[2];

        newPacket = false;
    }
    // Do we have the full payload?
    if (readBuffer.size() < 3 + static_cast<std::size_t>(currPayloadLen)) return false;
    newPacket = true;
    return true;
}

void RUDPCore::appendToReadBuffer(const uint8_t* data, std::size_t length) {
    if (length == 0) return;
    readBuffer.insert(readBuffer.end(), data, data + length);
}

void RUDPCore::appendToWriteBuffer(const uint8_t* data, std::size_t length) {
    if (length == 0) return;
    writeBuffer.insert(writeBuffer.end(), data, data + length);
}