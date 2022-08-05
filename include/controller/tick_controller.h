#pragma once
#include <json_cpp.h>
#include <cell_world.h>

namespace controller {

    enum Behavior{
        Explore,
        Pursue
    };

    enum Mode{
        Initialize,
        Moving,
        Tuning
    };


    struct Tick_outputs : json_cpp::Json_object{
        Json_object_members(
                Add_member(left);
                Add_member(right);
                Add_member(speed);
        )
        int left;
        int right;
        float speed;
    };




    struct Robot_move : json_cpp::Json_object{
        Json_object_members(
                Add_member(dtheta);
                Add_member(alpha);
                Add_member(distance);
                Add_member(left_ticks);
                Add_member(right_ticks);
                Add_member(speed);
        );
        int dtheta;
        int alpha;
        int distance;
        int left_ticks;
        int right_ticks;
        int speed;
    };



    // values located in robot_library/config/pid.json

}