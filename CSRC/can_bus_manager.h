#pragma once

#include <atomic>
#include <string>
#include <socketcan.h>
#include <mutex>
#include <thread>
#include <motor_controller.h>
#include <condition_variable>
#include <chrono>
#include <array>


class CanBusManager{
    private:
        std::string channelName;
        SocketCan bus;
        std::atomic<bool> running{false};
        std::mutex transmitMutex;
        std::thread receiverThread;
        std::mutex requestMutex;
        std::mutex responseMutex;
        std::condition_variable responseCV;

        bool responseWaiting{false};
        bool responseReady{false};

        uint8_t expectedDevice{0};
        uint8_t expectedFunction{0};

        std::array <MotorController*,128> motors{};

        can_frame receivedResponse{};

    public:
        explicit CanBusManager (std::string channel);

        bool addMotor(MotorController& motor);

        bool send(can_frame& frame);

        bool start();

        void receiveLoop();

        bool stop();

        bool request(can_frame& request_frame, uint8_t expected_device, uint8_t expected_function, can_frame& response_frame, std::chrono::milliseconds timeout);


        CanBusManager(const CanBusManager&) = delete;
        CanBusManager& operator= (const CanBusManager&) = delete;
        ~CanBusManager();

};