#include <iostream>
#include "can_bus_manager.h"

CanBusManager::CanBusManager(std::string channel)
    :channelName(channel)
    {}

bool CanBusManager::addMotor(MotorController& motor){
    uint8_t id = get_device_id(motor.get_device_id_from_motor());

    if(id == 0 || id >= motors.size() || motors[id] != nullptr){
        return false;
    }

    motors[id] = &motor;
    return true;

}

bool CanBusManager::send (can_frame& frame){
    std::lock_guard<std::mutex> lock(transmitMutex);

    if(!bus.isOpen() || !running.load()){
        std::cerr << "Cannot send CAN frame: bus not open\n";
        return false;
    }

    bus.write(&frame);
    return true;
}

void CanBusManager::receiveLoop(){
    while(running.load()){
        can_frame frame = bus.read();

        if(frame.can_id == 0 && frame.can_dlc == 0){
            std::cerr << "Empty can frame from read timeout" << '\n';
            continue;
        }

        if(!running.load()){
            std::cerr << channelName << " bus is closing" << '\n';
            break;
        }

        uint8_t device_id = get_device_id(frame.can_id);
        uint8_t functionID = get_func_id(frame.can_id);

        bool matchedResponse = false;

        {
            std::lock_guard<std::mutex> lock (responseMutex);
            if(responseWaiting == true && device_id == expectedDevice && functionID == expectedFunction){
                receivedResponse = frame;
                responseReady = true;
                matchedResponse = true;
            }
        }

        if(matchedResponse){
            responseCV.notify_one();
        }

        if(motors[device_id] != nullptr ){
            motors[device_id]->handleFrame(frame);
        }
        

    }
}

bool CanBusManager::start(){
    if(running.load()){
        return true;
    }

    if(!bus.open(channelName)){
        std::cerr << "Failed to open CAN channel" << channelName << '\n';
        return false;
    }

    running.store(true);

    try{
        receiverThread = std::thread(&CanBusManager::receiveLoop, this);
    }
    catch(const std::system_error& error){
        std::cerr << "Failed to start receiver thread: " << error.what() << '\n';
        running.store(false);
        bus.close();
        return false;
    }
    return true;
}

bool CanBusManager::stop(){
    running.store(false);

    if(receiverThread.joinable()) {
        receiverThread.join();
    }

    std::lock_guard<std::mutex> lock(transmitMutex);

    if(bus.isOpen()){
        bus.close();
    }

    if(!running.load() && !bus.isOpen()){
        return true;
    } else {
        return false;
    }
}

bool CanBusManager::request(can_frame& request_frame, uint8_t expected_device, uint8_t expected_function, can_frame& response_frame, std::chrono::milliseconds timeout){
    std::lock_guard<std::mutex> request_lock(requestMutex);
    std::unique_lock<std::mutex> response_lock(responseMutex);

    responseWaiting = true;
    responseReady = false;
    expectedDevice = expected_device;
    expectedFunction = expected_function;

    bool success = send(request_frame);
    if(!success){
        std::cerr << "socket could not send message" << '\n';
        return false;
    }

    bool received = responseCV.wait_for(response_lock,timeout, [this] {return responseReady;});

    responseWaiting = false;

    if(!received){
        std::cerr << "socket did not receive answer" << '\n';
        return false;
    }

    response_frame = receivedResponse;
    responseReady = false;
    return true;
}

CanBusManager::~CanBusManager(){
    stop();
}
    