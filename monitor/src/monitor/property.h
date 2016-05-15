#ifndef Property_h__
#define Property_h__

#ifdef DEBUG_NO_ROS
#include <iostream>
#define ROS_INFO_STREAM(args) std::cout<<(args)<<std::endl
#else
#include <ros/ros.h>
#endif

/* GLOBAL INCLUDES */
#include <functional>
#include <vector>
#include <iostream>

/* LOCAL INCLUDES */
#include "output_state.h"
#include "state_register.h"

//GENERATED FILES
#include "gen_commands.h" //true and false commands
/* INCLUDES END */


class Property {
protected:
    static Property* current_block;
    static unsigned int current_max_id;
    static unsigned int level;
    static bool evaluated;

    unsigned int id;
    StateRegister* state_register_ptr;
public:
    Property();
    ~Property();

    std::vector<trilean> input_states;
    std::vector<trilean> output_states;
    Property* root_node;
    Property* children_node;

    std::function <class Property*(class Property*)> construct_children_func;
    std::vector <std::function<trilean(class Property*)>> eval_functions;
    Property* constructChildrenBlock();
    trilean isEventFired(StateRegisterType);
    trilean evaluate();
    void freeChildrenNode();
    void printBlock(Property*);
};

#endif // Property_h__
