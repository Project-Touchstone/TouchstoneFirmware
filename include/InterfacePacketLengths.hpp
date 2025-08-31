#ifndef INTERFACE_PACKET_LENGTHS
#define INTERFACE_PACKET_LENGTHS

#include <Arduino.h>
#include <unordered_map>


std::unordered_map<uint8_t, int16_t> incomingByRequest = {
    { 1,  0 },
    { 16, 0 },
    { 17, -1 },
    { 18, 1 },
    { 19, 2 },
    { 20, 1 },
    { 21, 2 },
    { 22, 4 },
    { 23, 5 },
    { 24, 1 },
    { 25, 2 },
    { 26, 1 },
    { 27, 2 },
    { 32, 0 },
    { 48, 3 },
    { 59, 5 },
    { 50, 5 }
};

#endif