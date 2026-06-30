#pragma once

#include <atomic>
#include <string>
#include <socketcan.h>
#include <mutex>
#include <unordered_map>
#include <thread>
#include <motor_controller.h>

class CanBusManager{
    private:
        std::string channelName;
        SocketCan bus;
        std::atomic<bool> running{true};
        std::mutex transmitMutex;
        std::unordered_map <uint8_t,MotorController*>motors;
        std::thread receiverThread;

    public:
        explicit CanBusManager (const std::string& channel);

        void addMotor(const MotorController&);

        void send(can_frame& frame);

        bool start();

        void receiveLoop();

        bool stop();


        CanBusManager(const CanBusManager&) = delete;
        CanBusManager& operator= (const CanBusManager&) = delete;
        ~CanBusManager();

};