#pragma once
#include <cell_world.h>

namespace controller {
    struct Agent_operational_limits :json_cpp::Json_object {
        Json_object_members(
        Add_member(max_forward);
        Add_member(min_forward);
        Add_member(max_backward);
        Add_member(min_backward);
        Add_member(stop);
        )
        double max_forward;
        double min_forward;
        double max_backward;
        double min_backward;
        double stop;
        double convert(double);
    };

    struct Agent {
        virtual void set_left(double) = 0;
        virtual void set_right(double) = 0;
        virtual void capture() = 0;
        virtual bool update() = 0;
        virtual bool stop();
    };

    struct Tick_agent_move : json_cpp::Json_object {
        Json_object_members(
                Add_member(move);
                Add_member(orientation);
                Add_member(left_ticks);
                Add_member(right_ticks);
                Add_member(speed);
        )
        cell_world::Move move;
        int orientation;
        int left_ticks;
        int right_ticks;
        int speed;
        int update_orientation(int current_orientation);
    };

    struct Tick_agent_moves : json_cpp::Json_object {
        Json_object_members(
                Add_member(turning_moves);
                Add_member(forward_moves);
                )
        Tick_agent_move find_tick_move(cell_world::Move move, int current_orientation);
        Tick_agent_move get_forward_move(int current_orientation);

        json_cpp::Json_vector<Tick_agent_move> turning_moves;
        json_cpp::Json_vector<Tick_agent_move> forward_moves;
    };

    struct Tick_agent {
        virtual bool is_ready() = 0;
        virtual void execute_move(cell_world::Move_list) = 0;
        virtual void move_count_reset() = 0;
        virtual void set_rotation(float){};
        virtual void set_coordinate(cell_world::Coordinates){};
        virtual void correct_robot(){};
        virtual bool needs_correction(){return false;};
        virtual bool use_joystick(){return false;};
        virtual void joystick_control(){};
        virtual void capture(){};
        cell_world::Coordinates current_coordinates;
        cell_world::Timer timer;

    };
}