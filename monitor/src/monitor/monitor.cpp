#include "monitor.h"

void Monitor::run() {
    getInstance().subscriber_init();
    getInstance().subscriber_subsribe();
}

trilean Monitor::evaluate(StateRegisterType event) {
    StateRegister::stateRegister = event;
    if (property1 == nullptr) {
        property1 = new Property();
        property1->construct_children_node_func = construct_block0;
        construct_block0(property1);
    }

    trilean result = property1->evaluate();
    if(result == TRUE){
        Monitor::getInstance().true_action();
    }else if (result == FALSE){
        Monitor::getInstance().false_action();
    }

    return result;
}

Monitor& Monitor::getInstance(){
    static Monitor instance;
    return instance;
}

Monitor::Monitor(){
    property1 = nullptr;
}

Monitor::~Monitor(){
    subscriber_deinit();
}