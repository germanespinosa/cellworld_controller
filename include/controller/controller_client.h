#pragma once
#include <tcp_messages.h>
#include <cell_world.h>
#include <controller/pid_controller.h>

namespace controller{
    struct Controller_server;
    struct Prey_controller_server;

    struct Agent_values : json_cpp::Json_object {
        Json_object_members(
                Add_member(left);
                Add_member(right);
                Add_member(speed);
        )
        int left, right, speed;
    };

    struct Controller_client : tcp_messages::Message_client {
        Routes(
                Add_route("(.*)(step)", on_step, cell_world::Step);
                )
        virtual void on_step(const cell_world::Step &) {}
        bool pause();
        bool resume();
        bool stop();
        bool set_destination(const cell_world::Location &);
        bool set_behavior(Behavior);
        Controller_server *local_server = nullptr;
    };


    struct Prey_controller_client : tcp_messages::Message_client {
        Routes(
                Add_route("(.*)(step)", on_step, cell_world::Step);
        )
        virtual void on_step(const cell_world::Step &) {}
        bool pause();
        bool resume();
        bool stop();
        bool set_destination(const cell_world::Coordinates &);
        bool tune();
        bool is_move_done();
        Prey_controller_server *local_server = nullptr;
    };
}