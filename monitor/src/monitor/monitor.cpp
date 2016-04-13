#include "monitor.h"

Monitor* Monitor::instance = nullptr;

void Monitor::run() {
    getInstance()->subscriber_init();
    ros::spin();

}

trilean Monitor::evaluate(StateRegisterType event) {
    StateRegister::stateRegister = event;
    if (property1 == nullptr) {
        property1 = new Property();
        property1->construct_children_node_func = construct_block0;
        construct_block0(property1);
    }

    return FALSE;
}

Monitor* Monitor::getInstance(){
    if (instance == nullptr){
        instance = new Monitor();
    }
    return instance;
}

Monitor::Monitor(){
    subscriber_deinit();
    property1 = nullptr;
}

Monitor::~Monitor(){
    delete instance;
}