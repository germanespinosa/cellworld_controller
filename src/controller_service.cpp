#include <controller/controller_service.h>
#include <filesystem>

using namespace cell_world;
using namespace tcp_messages;
using namespace std;
using namespace json_cpp;

namespace controller {

   string logs_path = "";

    bool Controller_service::set_destination(const cell_world::Location &location) {
        return ((Controller_server *) _server)->set_destination(location);
    }

    bool Controller_service::stop_controller() {
        return true;
    }

    bool Controller_service::pause_controller() {
        return ((Controller_server *) _server)->pause();
    }

    bool Controller_service::resume_controller() {
        return ((Controller_server *) _server)->resume();
    }

    int Controller_service::get_port() {
        string port_str(std::getenv("CONTROLLER_PORT") ? std::getenv("CONTROLLER_PORT") : "4590");
        return atoi(port_str.c_str());
    }

    bool Controller_service::set_behavior(int behavior) {
        return ((Controller_server *) _server)->set_behavior(behavior);
}

    void Controller_service::set_logs_folder(const string &path) {
        logs_path = path;
        filesystem::create_directory(filesystem::path(logs_path));
    }

    void Controller_server::send_step(const Step &step) {
        if (!clients.empty()) {
            broadcast_subscribed(Message (step.agent_name + "_step", step));
        }
        for (auto local_client: subscribed_local_clients){
            local_client->on_step(step);
        }
    }

    Controller_server::Controller_server(const string &pid_config_file_path,
                                         Agent &agent,
                                         Controller_tracking_client &tracking_client,
                                         Controller_experiment_client &experiment_client):
            agent(agent),
            world(Resources::from("world_configuration").key("hexagonal").get_resource<World_configuration>(),Resources::from("world_implementation").key("hexagonal").key("canonical").get_resource<World_implementation>()),
            cells(world.create_cell_group()),
            paths(world.create_paths(Resources::from("paths").key("hexagonal").key("00_00").key("astar").get_resource<Path_builder>())),
            map(cells),
            navigability(cells, world.cell_shape,Transformation(world.cell_transformation.size, world.cell_transformation.rotation)),
            pid_controller (Json_from_file<Pid_parameters>(pid_config_file_path)),
            tracking_client(tracking_client),
            destination_timer(5),
            experiment_client(experiment_client)
    {
        tracking_client.controller_server = this;
        experiment_client.controller_server = this;
        tracking_client.subscribe();
        state = Controller_state::Stopped;
        process = thread(&Controller_server::controller_process, this);
    }

    void Controller_server::controller_process() {
        state = Controller_state::Playing;
        Pid_inputs pi;
        while(state != Controller_state::Stopped){
            // if there is no information from the tracker
            if (!tracking_client.agent.is_valid() ||
                state == Controller_state::Paused ||
                destination_timer.time_out()){
                agent.set_left(0);
                agent.set_right(0);
                agent.update();
                continue;
            }

            //PID controller
            pi.location = tracking_client.agent.step.location;
            pi.rotation = tracking_client.agent.step.rotation;
            pi.destination = get_next_stop();
            auto robot_command = pid_controller.process(pi, behavior);
            agent.set_left(robot_command.left);
            agent.set_right(robot_command.right);
            agent.update();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    bool Controller_server::set_destination(const cell_world::Location &new_destination) {
        destination = new_destination;
        destination_timer.reset();
        new_destination_data = true;
        return true;
    }

    cell_world::Location Controller_server::get_next_stop() {
        auto agent_location = tracking_client.agent.step.location;
        if (navigability.is_visible(agent_location, destination)) {
            return destination;
        }
        auto destination_cell_index = cells.find(destination);
        auto next_stop_test = cells.find(agent_location);
        auto next_stop = next_stop_test;
        while (navigability.is_visible(agent_location, cells[next_stop_test].location)){
            next_stop = next_stop_test;
            auto move = paths.get_move(cells[next_stop], cells[destination_cell_index]);
            if (move == Move{0,0}) break;
            next_stop_test = cells.find(map[cells[next_stop].coordinates + move]);
        }
        return cells[next_stop].location;
    }

    bool Controller_server::pause() {
        if (state == Controller_state::Playing) {
            state = Controller_state::Paused;
            return true;
        }
        return false;
    }

    bool Controller_server::resume() {
        if (state == Controller_state::Paused) {
            state = Controller_state::Playing;
            return true;
        }
        return false;
    }

    void Controller_server::set_occlusions(const std::string &occlusions) {
        auto occlusions_cgb = Resources::from("cell_group").key("hexagonal").key(occlusions).key("occlusions").get_resource<Cell_group_builder>();
        world.set_occlusions(occlusions_cgb);
        cells = world.create_cell_group();
        paths = Paths(world.create_paths(Resources::from("paths").key("hexagonal").key(occlusions).key("astar").get_resource<Path_builder>()));
        navigability = Location_visibility(cells, world.cell_shape,Transformation(world.cell_transformation.size * 1.25, world.cell_transformation.rotation));
    }

    void Controller_server::join() {
        process.join();
        Server::join();
    }

    bool Controller_server::set_behavior(int behavior) {
        this->behavior = static_cast<Behavior> (behavior);
        experiment_client.set_behavior(behavior);
        return true;
    }

    void Controller_server::send_capture(int frame) {
        experiment_client.capture(frame);
        if (!clients.empty()){
            broadcast_subscribed(Message("prey_captured", frame));
        }
        for (auto local_client: subscribed_local_clients){
            local_client->on_prey_captured(frame);
        }
    }

    void Controller_server::Controller_tracking_client::on_step(const Step &step) {
        if (step.agent_name == agent.agent_name) {
            controller_server->send_step(step);
            agent.step = step;
            agent.timer = Timer(.5);
        } else if (step.agent_name == adversary.agent_name) {
            if (contains_agent_state(agent.agent_name)) {
                auto predator = get_current_state(agent.agent_name);
                auto is_captured = capture.is_captured( predator.location, to_radians(predator.rotation), step.location);
                if (is_captured)
                    controller_server->send_capture(step.frame);
                if (visibility.is_visible(predator.location, step.location) &&
                    angle_difference(predator.location.atan(step.location), predator.rotation) < view_angle) {
                    if (peeking.is_seen(predator.location, step.location)) {
                        controller_server->send_step(step);
                    }
                } else {
                    peeking.not_visible();
                }
            }
        }
        Tracking_client::on_step(step);
    }

    string get_experiment_file(const string &experiment_name){
        return logs_path + experiment_name + ".json";
    }

    void Controller_server::Controller_experiment_client::on_experiment_started(
            const experiment::Start_experiment_response &experiment) {
        experiment.save(get_experiment_file(experiment.experiment_name));
        Experiment_client::on_experiment_started(experiment);
    }

    void Controller_server::Controller_experiment_client::on_episode_started(const string &experiment_name) {
        experiment::Start_experiment_response experiment;
        experiment.load(get_experiment_file(experiment_name));
        controller_server->set_occlusions(experiment.world.occlusions);
        Experiment_client::on_episode_started(experiment_name);
    }

    Controller_server::Controller_experiment_client::Controller_experiment_client() {

    }
}

