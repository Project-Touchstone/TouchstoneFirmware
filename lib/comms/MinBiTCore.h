#ifndef MINBIT_CORE_H
#define MINBIT_CORE_H

#include <vector>
#include <mutex>
#include <chrono>
#include <cstring>
#include <iostream>
#include <unordered_map>
#include <thread>
#include <queue>

#include "IStream.h"

class MinBiTCore {
    public:
        enum class WriteMode {
            IMMEDIATE,
            PACKET
        };

        struct PacketLengthEntry {
            uint8_t header;
            int length;
        };

        class Request {
        public:
            enum class Status {
                INCOMING,
                OUTGOING,
                COMPLETE,
                TIMEDOUT
            };

            Request(uint8_t header, Status status);

            void Start();
            void SetStatus(Status newStatus);
            void SetResponseHeader(uint8_t responseHeader);
            void SetPayloadLength(int responseLength);

            Status GetStatus();
            int64_t GetId() const;
            uint8_t GetHeader() const;
            uint8_t GetResponseHeader();
            int GetResponseLength();
            bool IsIncoming();
            bool IsOutgoing();
            bool IsWaiting();
            bool IsComplete();
            bool IsTimedOut();

            std::chrono::steady_clock::time_point GetSentTime();

            Status WaitSync(int pollIntervalMs = 5);

        private:
            static std::atomic<int64_t> nextId;
            int64_t id;
            uint8_t header;
            uint8_t responseHeader;
            int responseLength;
            Status status;
            std::chrono::steady_clock::time_point sentTime;
            mutable std::mutex requestMutex;
        };

        using ReadHandler = std::function<void(std::shared_ptr<MinBiTCore::Request>)>;

        MinBiTCore(std::string name, std::shared_ptr<IStream> stream);
        ~MinBiTCore();

        //Sets read handler
        void setReadHandler(ReadHandler handler);

        // Gets stream object
        std::shared_ptr<IStream> getStream();

        // Set writing mode
        void setWriteMode(WriteMode mode);

        // Set request timeout
        void setRequestTimeout(uint16_t timeoutMs);

        // Loads packet length information
        bool loadOutgoingByRequest(std::unordered_map<uint8_t, int16_t>* map);
        bool loadOutgoingByResponse(std::unordered_map<uint8_t, int16_t>* map);
        bool loadIncomingByRequest(std::unordered_map<uint8_t, int16_t>* map);

        // Writing functions
        std::shared_ptr<MinBiTCore::Request> writeRequest(uint8_t header);
        void writeBytes(const uint8_t* buffer, std::size_t length);
        void writeByte(uint8_t value);
        void writeFloat(float value);
        // Writes a 16 bit integer
        void writeInt16(int16_t data);
        // Writes packet
        void sendAll();

        // Reading functions
        void fetchData();
        uint8_t readByte();
        uint8_t peekByte();
        void readBytes(uint8_t* buffer, std::size_t len);
        float readFloat();
        int16_t readInt16();

        template <typename T>
        T readData();

        // Packet management

        // Gets the expected length for a header, returns false if not found
        bool getExpectedPacketLength(std::shared_ptr<Request> request, int16_t& length) const;
        bool getPacketParameters(int16_t expectedLength, std::size_t& payloadLength, std::size_t& totalPacketLength);
        bool getOutgoingRequest(std::shared_ptr<Request>& request);
        bool isPacketPending();
        // Flushes the read buffer
        void flush();
        bool clearRequest();
        std::size_t getReadBufferSize();
        std::size_t getWriteBufferSize();
        std::size_t getNumOutgoingRequests();

    private:
        std::string name;
        std::shared_ptr<IStream> stream;
        std::vector<uint8_t> readBuffer;
        std::vector<uint8_t> writeBuffer;
        std::queue<std::shared_ptr<MinBiTCore::Request>> unsentRequests;
        std::queue<std::shared_ptr<MinBiTCore::Request>> outgoingRequests;
        // Current request being processed
        std::shared_ptr<Request> currRequest;

        uint16_t requestTimeoutMs = 1000; // or make this configurable
        std::mutex dataMutex;

        // Outgoing packet lengths by request header
        std::unordered_map<uint8_t, int16_t>* outgoingByRequest;
        // Outgoing packet legnths by response header
        std::unordered_map<uint8_t, int16_t>* outgoingByResponse;
        // Incoming packet lengths by request header
        std::unordered_map<uint8_t, int16_t>* incomingByRequest;

        WriteMode writeMode = WriteMode::IMMEDIATE;

        //Read handler
		ReadHandler readHandler;

        // Buffer management
        void appendToReadBuffer(const uint8_t* data, std::size_t length);
        void appendToWriteBuffer(const uint8_t* data, std::size_t length);

        // Processing loop
        void checkForTimeouts();
        bool characterizePacket(bool& variableLength, std::size_t payloadLength);
};

template <typename T>
T MinBiTCore::readData() {
    T data;
    uint8_t buffer[sizeof(data)];
    MinBiTCore::readBytes(buffer, sizeof(data));

    std::memcpy(&data, buffer, sizeof(data));

    return data;
}

#endif // MINBIT_CORE_H