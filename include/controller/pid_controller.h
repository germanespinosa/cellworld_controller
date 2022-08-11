#pragma once
#include <json_cpp.h>
#include <cell_world.h>
#include <vector>

#define ROBOT_SPEED 1800

namespace controller {
    enum Behavior{
        Explore,
        Pursue
    };


    struct Controller_outputs : json_cpp::Json_object{
        Json_object_members(
                Add_member(left);
                Add_member(right);
                Add_member(speed);
        )
        double left;
        double right;
        double speed;
    };


//    struct Pid_inputs : json_cpp::Json_object{
//        Json_object_members(
//                Add_member(location);
//                Add_member(rotation);
//                Add_member(destination);
//        );
//        cell_world::Location location;
//        double rotation;
//        cell_world::Location destination;
//    };

    struct Controller_inputs : json_cpp::Json_object{
        Json_object_members(
                Add_member(location);
                Add_member(next_coordinate);
                Add_member(current_coordinate); // from tracker when initializing - then store when dont want involved anymore
                Add_member(previous_coordinate);
                );
        cell_world::Location location;
        cell_world::Coordinates next_coordinate;
        cell_world::Coordinates current_coordinate;
        cell_world::Coordinates previous_coordinate;

    };


//
    // values located in robot_library/config/pid.json
    struct Pid_parameters : json_cpp::Json_object{
        Json_object_members(
                Add_member(P_pursue);
                Add_member(I_pursue);
                Add_member(D_pursue);
                Add_member(P_explore);
                Add_member(I_explore);
                Add_member(D_explore);
                Add_member(pursue_speed);
                Add_member(explore_speed);
                );
        double P_pursue;
        double I_pursue;
        double D_pursue;
        double P_explore;
        double I_explore;
        double D_explore;
        double pursue_speed;
        double explore_speed;
    };

    struct Pid_controller : json_cpp::Json_object {
        Pid_controller (const Pid_parameters &);
        Json_object_members(
                Add_member(error);
                Add_member(normalized_error);
                Add_member(error_integral);
                Add_member(error_derivative);
                Add_member(last_error);
                Add_member(out);
                Add_member(in);
                );
        Controller_outputs process(const Controller_inputs &, Behavior);
        static double normalize_error(double);
        cell_world::Coordinates delta_coordinates(const cell_world::Coordinates &c1, const  cell_world::Coordinates &c2);
        int get_side(const cell_world::Coordinates &delta_side);
        std::pair<int,int> get_ticks(const cell_world::Coordinates &delta_move, int side);
        Pid_parameters parameters;
        double error{};
        double error_integral{};
        double error_derivative{};
        double normalized_error{};
        double last_error{};
        Controller_outputs out;
        Controller_inputs in;
        cell_world::Timer timer;
    };
}