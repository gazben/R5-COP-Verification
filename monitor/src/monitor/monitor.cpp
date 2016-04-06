#include "monitor.h"

const std::string EventInterface::getTopicName() {
    return topic_name;
}

//Monitor
Property* Monitor::property1 = nullptr;

void Monitor::run(int argc, char **argv) {
    ros::init(argc, argv, "subscribe_to_vel");
    ros::spin();  //the point of no return
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