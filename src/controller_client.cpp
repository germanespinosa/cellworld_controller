#include <controller/controller_client.h>
#include <controller/controller_service.h>

using namespace tcp_messages;
using namespace cell_world;

namespace controller {

    bool Controller_client::pause() {
        if (local_server)
        {
            return local_server->pause();
        }else {
            return send_request(Message("pause_predator")).get_body<bool>();
        }
    }

    bool Controller_client::resume() {
        if (local_server)
        {
            return local_server->resume();
        }else {
            return send_request(Message("resume_predator")).get_body<bool>();
        }
    }

    bool Controller_client::stop() {
        if (local_server)
        {
            return true;
        }else {
            return send_request(Message("stop_predator")).get_body<bool>();
        }
    }


    bool Controller_client::set_destination(const cell_world::Location &location) {
        if (local_server)
        {
            return local_server->set_destination(location);
        }else {
            return send_request(Message("set_destination", location)).get_body<bool>();
        }
    }

    bool Controller_client::set_behavior(Behavior behavior) {
        if (local_server)
        {
            return local_server->set_behavior(behavior);
        }else {
            return send_request(Message("set_behavior", behavior)).get_body<bool>();
        }
    }

    int Controller_client::set_agent_values(const Agent_values &values) {
        if (local_server)
        {
            return local_server->set_agent_values(values);
        } else{
            return send_request(Message("set_agent_values", values)).get_body<int>();
        }
    }

    bool Controller_client::set_left_ticks(int left) {
        if (local_server)
        {
            return local_server->set_left_ticks(left);
        } else{
            return send_request(Message("set_left_ticks", left)).get_body<bool>();
        }
    }

    bool Controller_client::set_right_ticks(int right) {
        if (local_server)
        {
            return local_server->set_right_ticks(right);
        } else{
            return send_request(Message("set_right_ticks", right)).get_body<bool>();
        }
    }

    bool Controller_client::set_speed(int speed) {
        if (local_server)
        {
            return local_server->set_speed(speed);
        } else{
            return send_request(Message("set_speed", speed)).get_body<bool>();
        }
    }

    bool Controller_client::agent_move_number(int move_number) {
        if (local_server)
        {
            return local_server->agent_move_number(move_number);
        } else{
            return send_request(Message("move_number")).get_body<bool>();  // TODO: double check dont know if this is an int
        }
    }

    bool Controller_client::tune() {
        if (local_server)
        {
            return local_server->tune();
        }else {
            return send_request(Message("tune")).get_body<bool>();
        }
    }

    bool Controller_client::is_move_done(){
        if (local_server){
            return local_server->is_move_done();
        } else {
            return send_request(Message("is_move_done")).get_body<bool>();
        }
    }


}