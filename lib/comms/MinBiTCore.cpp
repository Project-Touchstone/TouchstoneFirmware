#include "MinBiTCore.h"

std::atomic<int64_t> MinBiTCore::Request::nextId{ 1 };

MinBiTCore::Request::Request(uint8_t header)
    : header(header),
    responseHeader(0),
    responseLength(-1),
    status(Status::UNSENT),
    id(nextId.fetch_add(1))
{
}

void MinBiTCore::Request::Start() {
    std::lock_guard<std::mutex> lock(requestMutex);
    sentTime = std::chrono::steady_clock::now();
    status = Status::WAITING;
}

void MinBiTCore::Request::SetStatus(Status newStatus) {
    std::lock_guard<std::mutex> lock(requestMutex);
    status = newStatus;
}

void MinBiTCore::Request::SetResponseHeader(uint8_t responseHeader) {
    std::lock_guard<std::mutex> lock(requestMutex);
    this->responseHeader = responseHeader;
}

void MinBiTCore::Request::SetPayloadLength(int responseLength) {
    std::lock_guard<std::mutex> lock(requestMutex);
    this->responseLength = responseLength;
}

MinBiTCore::Request::Status MinBiTCore::Request::GetStatus() {
    std::lock_guard<std::mutex> lock(requestMutex);
    return status;
}

int64_t MinBiTCore::Request::GetId() const {
    return id;
}

uint8_t MinBiTCore::Request::GetHeader() const {
    return header;
}

uint8_t MinBiTCore::Request::GetResponseHeader() {
    std::lock_guard<std::mutex> lock(requestMutex);
    return responseHeader;
}

int MinBiTCore::Request::GetResponseLength() {
    std::lock_guard<std::mutex> lock(requestMutex);
    return responseLength;
}

bool MinBiTCore::Request::IsWaiting() {
    std::lock_guard<std::mutex> lock(requestMutex);
    return status == Status::WAITING;
}

bool MinBiTCore::Request::IsTimedOut() {
    std::lock_guard<std::mutex> lock(requestMutex);
    return status == Status::TIMEDOUT;
}

std::chrono::steady_clock::time_point MinBiTCore::Request::GetSentTime() {
    std::lock_guard<std::mutex> lock(requestMutex);
    return sentTime;
}

MinBiTCore::Request::Status MinBiTCore::Request::WaitSync(int pollIntervalMs) {
    while (true) {
        {
            std::lock_guard<std::mutex> lock(requestMutex);
            if (status != Status::WAITING)
                return status;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
    }
}

// DataProtocol handles serialization and communication of data packets over an IStream.
MinBiTCore::MinBiTCore(std::string name, std::shared_ptr<IStream> stream)
    : name(name), stream(std::move(stream)) {}

MinBiTCore::~MinBiTCore() {
    // Ensure the stream is closed on destruction.
    if (stream && stream->isOpen()) {
        stream->close();
    }
    // No dynamic allocations to clean up, but destructor ensures stream is closed.
}

void MinBiTCore::setReadHandler(ReadHandler handler) {
    // Set the callback to be invoked when data is read.
    readHandler = handler;
}

std::shared_ptr<IStream> MinBiTCore::getStream() {
    // Return the underlying stream.
    return stream;
}

void MinBiTCore::setNodeType(NodeType type) {
    nodeType = type;
}

bool MinBiTCore::isClient() const {
    return nodeType == NodeType::CLIENT;
}

bool MinBiTCore::isServer() const {
    return nodeType == NodeType::SERVER;
}

void MinBiTCore::setWriteMode(WriteMode mode) {
    // Set the mode for writing packets (immediate or buffered).
    this->writeMode = mode;
}

void MinBiTCore::setRequestTimeout(uint16_t timeoutMs) {
    this->requestTimeoutMs = timeoutMs;
}

bool MinBiTCore::loadPacketLengthsByRequest(std::unordered_map<uint8_t, int16_t>* lengthsByRequest) {
    this->lengthsByRequest = lengthsByRequest;
}

bool MinBiTCore::loadPacketLengthsByResponse(std::unordered_map<uint8_t, int16_t>* lengthsByResponse) {
    this->lengthsByResponse = lengthsByResponse;
}

bool MinBiTCore::getExpectedPacketLength(std::shared_ptr<Request> request, int16_t& length) const {
    // If client, lengths by response header are the priority
    if (isClient() && lengthsByResponse) {
        auto it = lengthsByResponse->find(request->GetResponseHeader());
        if (it != lengthsByResponse->end()) {
            length = it->second;
            return true;
        }
    }

    // Otherwise searches by request header
    if (lengthsByRequest) {
        auto it = lengthsByRequest->find(request->GetHeader());
        if (it != lengthsByRequest->end()) {
            length = it->second;
            return true;
        }
    }
    
    return false;
}

bool MinBiTCore::getPacketParameters(int16_t expectedLength, std::size_t& payloadLength, std::size_t& totalPacketLength) {
    payloadLength = 0;
    totalPacketLength = 1; // 1 byte for response header

    if (expectedLength == -1) {
        // Need at least 2 bytes: header + length byte
        if (getReadBufferSize() < 2) {
            return false;
        }
        uint8_t lengthByte;
        {
            std::lock_guard<std::mutex> lock(dataMutex);
            lengthByte = readBuffer[1];
        }
        // Sets expected payload length
        payloadLength = lengthByte;
        // Adds to total packet length
        totalPacketLength += 1 + payloadLength; // header + length byte + payload
    }
    else {
        // Updates expected payload length
        payloadLength = expectedLength;
        totalPacketLength += expectedLength;
    }
    return true;
}

std::shared_ptr<MinBiTCore::Request> MinBiTCore::writeHeader(uint8_t header) {
    writeByte(header);
    if (isClient()) {
        auto request = std::make_shared<Request>(header);
        {
            std::lock_guard<std::mutex> lock(dataMutex);
            requestQueue.push(request);
        }
        return request;
    }
    // If server, do not create or start a request
    return nullptr;
}

void MinBiTCore::writeBytes(const uint8_t* buffer, std::size_t length) {
    // Appends data to write buffer
    appendToWriteBuffer(buffer, length);

    // Writes packet immediately if in immediate mode
    if (writeMode == WriteMode::IMMEDIATE) {
        writePacket();
    }
}

void MinBiTCore::writeByte(uint8_t value) {
    // Write a single byte.
    writeBytes(&value, sizeof(value));
}

void MinBiTCore::writeFloat(float value) {
    // Serialize a float with correct endianness and write.
    uint8_t* networkValue = reinterpret_cast<uint8_t*>(&value);
    writeBytes(networkValue, sizeof(networkValue));
}

void MinBiTCore::writeInt16(int16_t data) {
    // Serialize and write a 16-bit integer.
    uint8_t buffer[sizeof(data)];
    memcpy(buffer, &data, sizeof(data));
    writeBytes(buffer, sizeof(data));
}

void MinBiTCore::writePacket() {
    // Write the contents of the write buffer as a packet.
    std::lock_guard<std::mutex> lock(dataMutex);
    if (!stream || !stream->isOpen()) return;

    if (isClient()) {
        // Starts current request if client
        std::shared_ptr<Request> request;
        if (!getCurrentRequest(request) || !request) {
            std::cerr << "( " + name + ") Failed to write packet: no header present" << std::endl;
            return;
        }
        request->Start();
    }

    size_t trueBufferSize = writeBuffer.size();
    stream->write(writeBuffer.data(), trueBufferSize);

    // Clears write buffer
    writeBuffer.clear();
}

void MinBiTCore::checkForTimeouts() {
    std::lock_guard<std::mutex> lock(dataMutex);
    if (!requestQueue.empty()) {
        std::shared_ptr<Request> req = requestQueue.front();
        auto now = std::chrono::steady_clock::now();

        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - req->GetSentTime()).count() > requestTimeoutMs) {
            std::cerr << "( " + name + ") Request with header " << int(req->GetHeader()) << " timed out after " << requestTimeoutMs << " ms." << std::endl;
            req->SetStatus(Request::Status::TIMEDOUT);
            clearRequest();
            flush();
            // Calls read handler
            if (readHandler) {
                readHandler(req);
            }
        }
    }
}

bool MinBiTCore::characterizePacket(std::shared_ptr<MinBiTCore::Request>& request, bool& variableLength, std::size_t payloadLength) {
    // Sets default values
    variableLength = false;
    payloadLength = 0;
    
    // Request pointer
    if (isClient() || packetFlag) {
        // Gets current request if already created
        if (!getCurrentRequest(request)) {
            std::cerr << "( " + name + ") Request queue cleared unexpectedly" << std::endl;
            flush();
            return false;
        }
    }
    else {
        // Creates new request if server and packet was recently received
        uint8_t receivedHeader = peekByte();
        // Creates and starts request
        request = std::make_shared<Request>(receivedHeader);
        request->Start();
        {
            // Adds to request queue
            std::lock_guard<std::mutex> lock(dataMutex);
            requestQueue.push(request);
        }
    }

    // Only process requests that have not yet been fufilled
    if (!request->IsWaiting()) {
        return false;
    }

    // Packet is being processed
    if (!packetFlag) {
        // Sets response header (if client)
        if (isClient()) {
            request->SetResponseHeader(peekByte());
        }
        packetFlag = true;
    }

    // Determine expected response length for request
    int16_t expectedLength = 0;
    if (!getExpectedPacketLength(request, expectedLength)) {
        std::cerr << "( " + name + ") No response length found for request header " << int(request->GetHeader()) << std::endl;
        clearRequest();
        flush();
        return false;
    }

    // Gets packet length parameters
    std::size_t totalPacketLength;
    if (!getPacketParameters(expectedLength, payloadLength, totalPacketLength)) {
        // Waits until able to access all packet parameters
        return false;
    }

    // Wait until the full packet is available
    if (getReadBufferSize() < totalPacketLength) {
        return false;
    }

    variableLength = (expectedLength == -1);
    return true;
}

void MinBiTCore::fetchData() {
    if (!stream || !stream->isOpen()) return;

    uint8_t* readBuffer;
    uint8_t bytesTransferred = stream->available();
    stream->read(readBuffer, bytesTransferred);
    appendToReadBuffer(readBuffer, bytesTransferred);

    // Process packets only when enough data is available
    while (getReadBufferSize() > 0 && (!requestQueue.empty() || isServer())) {
        std::shared_ptr<MinBiTCore::Request> request;
        bool variableLength;
        std::size_t payloadLength;
        if (!characterizePacket(request, variableLength, payloadLength)) {
            break;
        }

        // Now we have the full packet, so process it
        readByte(); // Removes header

        // If variable length, remove the length byte as well
        if (variableLength) {
            readByte();
        }

        // Set payload length
        request->SetPayloadLength(payloadLength);

        // Request has been fulfilled
        request->SetStatus(Request::Status::FUFILLED);

        // Calls read handler if exists
        if (readHandler) {
            readHandler(request);
            // Clears request from queue
            clearRequest();
        }
    }
    // Flushes data not associated with request
    if (getReadBufferSize() > 0 && getRequestQueueSize() == 0) {
        flush();
    }

    // Timeout check: remove requests that have timed out
    checkForTimeouts();
}

uint8_t MinBiTCore::readByte() {
    // Read a single byte from the read buffer.
    uint8_t value;
    readBytes(&value, sizeof(value));
    return value;
}

uint8_t MinBiTCore::peekByte() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return readBuffer[0];
}

void MinBiTCore::readBytes(uint8_t* buffer, std::size_t len) {
    // Read a sequence of bytes from the read buffer.
    std::lock_guard<std::mutex> lock(dataMutex);
    if (readBuffer.size() < len) throw std::runtime_error("( " + name + ") Buffer underflow");
    std::memcpy(buffer, readBuffer.data(), len);
    readBuffer.erase(readBuffer.begin(), readBuffer.begin() + len);
}

int16_t MinBiTCore::readInt16() {
    uint8_t buffer[sizeof(int16_t)];
    readBytes(buffer, sizeof(int16_t));
    int16_t value;
    std::memcpy(&value, buffer, sizeof(int16_t));
    return value;
}

float MinBiTCore::readFloat() {
    // Read a float from the read buffer, handling endianness.
    uint8_t buffer[sizeof(float)];
    readBytes(buffer, sizeof(float));
    float networkValue;
    std::memcpy(&networkValue, buffer, sizeof(float));
    return networkValue;
}

bool MinBiTCore::clearRequest() {
    // Removes current request from queue
    std::lock_guard<std::mutex> lock(dataMutex);
    packetFlag = false;
    if (!requestQueue.empty() > 0)
    {
        requestQueue.pop();
        return true;
    }
    return false;
}

bool MinBiTCore::getCurrentRequest(std::shared_ptr<Request>& request) {
    std::lock_guard<std::mutex> lock(dataMutex);
    if (!requestQueue.empty()) {
        request = requestQueue.front();
        return true;
    }
    request = nullptr;
    return false;
}

bool MinBiTCore::isPacketPending() {
    // Check if there is a pending packet to be read.
    return packetFlag;
}

void MinBiTCore::flush() {
    // Clear the read buffer
    std::lock_guard<std::mutex> lock(dataMutex);
    readBuffer.clear();
}

std::size_t MinBiTCore::getReadBufferSize() {
    // Get the size of the read buffer (thread-safe).
    std::lock_guard<std::mutex> lock(dataMutex); // Ensure thread-safe access
    return readBuffer.size();
}

std::size_t MinBiTCore::getWriteBufferSize() {
    // Get the size of the write buffer (thread-safe).
    std::lock_guard<std::mutex> lock(dataMutex); // Ensure thread-safe access
    return writeBuffer.size();
}

std::size_t MinBiTCore::getRequestQueueSize() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return requestQueue.size();
}

void MinBiTCore::appendToReadBuffer(const uint8_t* data, std::size_t length) {
    // Append data to the read buffer (thread-safe).
    std::lock_guard<std::mutex> lock(dataMutex);
    readBuffer.insert(readBuffer.end(), data, data + length);
}

void MinBiTCore::appendToWriteBuffer(const uint8_t* data, std::size_t length) {
    // Append data to the write buffer (thread-safe).
    std::lock_guard<std::mutex> lock(dataMutex);
    writeBuffer.insert(writeBuffer.end(), data, data + length);
}