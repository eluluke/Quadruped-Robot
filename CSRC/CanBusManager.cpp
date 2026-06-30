#include "CanBusManager.h"
#include <iostream>

void CanBusManager::send (can_frame& frame){
    std::lock_guard<std::mutex> lock(transmitMutex);

    if(!bus.isOpen() || !running.load()){
        std::cerr << "Cannot send CAN frame: bus not open\n";
        return;
    }

    bus.write(&frame);
}

void CanBusManager::receiveLoop(){
    while(running.load()){
        can_frame frame = bus.read();

        if(!running.load()){
            std::cerr << channelName << " bus is closing" << '\n';
            break;
        }

        uint8_t device_id = get_device_id(frame.can_id);

        auto motor = motors.find(device_id);

        if(motor == motors.end()){
            continue;
        }

        if(motor->second == nullptr) {
            continue;
        }

        motor->second->handleFrame(frame);
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
}

CanBusManager::~CanBusManager(){
    stop();
}