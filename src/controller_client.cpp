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

    bool Controller_client::set_rotation(float rotation) {
        if (local_server)
        {
            return local_server->set_rotation(rotation);
        }else {
            return send_request(Message("set_rotation", rotation)).get_body<bool>();
        }
    }

    bool Controller_client::set_coordinate(cell_world::Coordinates coordinate) {
        if (local_server)
        {
            return local_server->set_coordinate(coordinate);
        }else {
            return send_request(Message("set_coordinate", coordinate)).get_body<bool>();
        }
    }
}