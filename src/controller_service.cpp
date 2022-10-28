#include <controller/controller_service.h>
#include <controller/controller_service.h>
#include <filesystem>
#include <iostream>
#include <mutex>

using namespace cell_world;
using namespace tcp_messages;
using namespace std;
using namespace json_cpp;

namespace controller {
    mutex robot_mtx;
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
                                         Tick_agent &agent,
                                         Controller_tracking_client &tracking_client,
                                         Controller_experiment_client &experiment_client):
            agent(agent),
            world(World::get_from_parameters_name("robot", "canonical", "21_05")),
            cells(world.create_cell_group()),
            paths(world.create_paths(Resources::from("paths").key("robot").key("21_05").key("astar").get_resource<Path_builder>())),
            map(cells),
            pid_controller (Json_from_file<Pid_parameters>(pid_config_file_path)),
            tracking_client(tracking_client),
            destination_timer(0),
            experiment_client(experiment_client)
            // TODO: might want to change default paths and world to robot ... we will see
    {
        tracking_client.controller_server = this;
        experiment_client.controller_server = this;
        experiment_client.subscribe();
        tracking_client.subscribe();
        state = Controller_state::Stopped;
        process = thread(&Controller_server::controller_process, this);
    }

    void Controller_server::controller_process() {                      // setting robot velocity
        // reset tick count on  robot
//        agent.move_count_reset();
        state = Controller_state::Playing;
        int i;
        while(state != Controller_state::Stopped){
            robot_mtx.lock();
            if (this->tracking_client.capture.cool_down.time_out()){
            // if there is no information from the tracker
                if (!tracking_client.agent.is_valid() ||
                    state == Controller_state::Paused ||
                    destination_timer.time_out()){
    //                    cout << "PAUSE ROBOT" << endl;
                    i = 0;
                } else {
                    if (agent.is_ready()){
                        auto next_moves = get_next_moves();
                        if (!next_moves.empty()) agent.execute_move(next_moves);
                    }
                }
                // check if needs correction due to high error or joystick use
                if (agent.needs_correction()) {
                    agent.correct_robot();
                }
            }
            // check if joystick is on each loop
            if (agent.use_joystick()) {
                experiment_client.human_intervention(true);
                agent.joystick_control();
                experiment_client.human_intervention(false);
            }
//            // check if needs correction due to high error or joystick use
//            if (agent.needs_correction()) {
//                agent.correct_robot();
//            }

            robot_mtx.unlock();
            //prevents overflowing the robot ( max 10 commands per second)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    bool Controller_server::set_destination(const cell_world::Location &new_destination) {
        destination = new_destination;
//        cout << "new destination: " << destination << endl;
        destination_timer = Timer(5);
        new_destination_data = true;
        return true;
    }

    cell_world::Cell_group Controller_server::get_possible_next_cells(const cell_world::Cell &cell){

        cell_world::Cell_group possible_moves;

        auto candidates = world.connection_pattern.get_candidates(cell.coordinates);

        auto prev_move = world.connection_pattern.back();
        auto prev_candidate = candidates.back();
        bool prev_valid = map.find(prev_candidate) != Not_found && !map[prev_candidate].occluded;

        for (unsigned int i=0; i < candidates.size();i++){
            auto curr_move = world.connection_pattern[i];
            auto curr_candidate = candidates[i];
            bool curr_valid = map.find(curr_candidate) != Not_found && !map[curr_candidate].occluded;
            if (curr_valid) {
                possible_moves.add(map[curr_candidate]);
                if (prev_valid) {
                    //test double moves
                    auto double_move = curr_move + prev_move;
                    auto double_candidate = cell.coordinates + double_move;
                    bool double_valid = map.find(double_candidate) != Not_found && !map[double_candidate].occluded;
                    if (double_valid) possible_moves.add(map[double_candidate]);
                }
            }
            prev_move = curr_move;
            prev_candidate = curr_candidate;
            prev_valid = curr_valid;
        }
        return possible_moves;
    }

    // This function returns next move based on current coordinate of the robot wrt the current destination
    cell_world::Move_list Controller_server::get_next_moves() {
        auto agent_cell_index = cells.find(agent.current_coordinates); // current robot coordinates
        auto &agent_cell = cells[agent_cell_index];

        auto destination_cell_index = cells.find(destination); // destination
        auto &destination_cell = cells[destination_cell_index];

        Move_list moves;
        if (agent_cell != destination_cell) {
            auto possible_moves = get_possible_next_cells(cells[agent_cell_index]);
            int min_distance = -1;
            for (auto &possible_move: possible_moves) {
                auto step_count = paths.get_steps(possible_move, destination_cell);
                if (min_distance == -1 || min_distance > step_count) {
                    min_distance = step_count;
                }
            }
            for (auto &possible_move: possible_moves) {
                auto step_count = paths.get_steps(possible_move, destination_cell);
                if (step_count == min_distance) {
                    auto move = possible_move.get().coordinates - agent.current_coordinates;
                    moves.push_back(move);
                }
            }
        }
        return moves;
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

    void Controller_server::set_occlusions(const std::string &occlusions, float margin) {
        // TODO: change world that is loaded hexagonal -> robot
        //auto occlusions_cgb = Resources::from("cell_group").key("robot").key(occlusions).key("occlusions").get_resource<Cell_group_builder>();
        //world.set_occlusions(occlusions_cgb);
        //cells = world.create_cell_group();
        //paths = Paths(world.create_paths(Resources::from("paths").key("robot").key(occlusions).key("astar").get_resource<Path_builder>()));
        //tracking_client.visibility.update_occlusions(cells);
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
    }

    void Controller_server::set_occlusions(Cell_group &cells) {
    }

    void Controller_server::Controller_tracking_client::on_step(const Step &step) {
        if (!capture.cool_down.time_out()) return;
        if (step.agent_name == agent.agent_name) {
            if (agent.last_update.to_seconds()>.1) {
                controller_server->send_step(step);
                agent.last_update.reset();
            }
            agent.step = step;
            agent.timer = Timer(.5);
        } else if (step.agent_name == adversary.agent_name) {
            adversary.step = step;
            adversary.timer = Timer(.5);
            if (contains_agent_state(agent.agent_name)) {
                auto predator = get_current_state(agent.agent_name);
                robot_mtx.lock();
                    auto is_captured = capture.is_captured( predator.location, to_radians(predator.rotation), step.location);
                    if (is_captured) {
                        cout << "capture" << endl;
                        controller_server->agent.capture();
                    } // triggering it here creates all sorts of events... cmalybe this should be in the embedded?
                robot_mtx.unlock();
                if (visibility.is_visible(predator.location, step.location) &&
                        to_degrees(angle_difference(predator.location.atan(step.location), to_radians(predator.rotation))) < view_angle / 2) {
                    if (peeking.is_seen(predator.location, step.location)) {
                        if (adversary.last_update.to_seconds()>.1) {
                            controller_server->send_step(step);
                            adversary.last_update.reset();
                        }
                    }
                } else {
                    peeking.not_visible();
                }
            }
        }
        Tracking_client::on_step(step);
    }

    Controller_server::Controller_tracking_client::Controller_tracking_client(Location_visibility &visibility,
                                                                              float view_angle,
                                                                              Capture &capture,
                                                                              Peeking &peeking,
                                                                              const string &agent_name,
                                                                              const string &adversary_name)  :
            agent(agent_name),
            adversary(adversary_name),
            visibility(visibility),
            view_angle(view_angle),
            capture(capture),
            peeking(peeking){
    }

    Controller_server::Controller_tracking_client::Controller_tracking_client(cell_world::World world,
                                                                              float view_angle,
                                                                              const string &agent_name,
                                                                              const string &adversary_name):
            agent(agent_name),
            adversary(adversary_name),
            visibility(world.create_cell_group(), world.cell_shape, world.cell_transformation),
            view_angle(view_angle),
            capture(Resources::from("capture_parameters").key("default").get_resource<Capture_parameters>(), world),
            peeking(Resources::from("peeking_parameters").key("default").get_resource<Peeking_parameters>(), world){

    }

    void Controller_server::Controller_tracking_client::set_occlusions(Cell_group &cells) {
//        visibility.update_occlusions(cells);
//        capture.visibility.update_occlusions(cells);
//        peeking.peeking_visibility.update_occlusions(cells);
    }

    string get_experiment_file(const string &experiment_name){
        return logs_path + experiment_name + "_controller.json";
    }

    void Controller_server::Controller_experiment_client::on_experiment_started(
            const experiment::Start_experiment_response &experiment) {
        experiment.save(get_experiment_file(experiment.experiment_name));
        Experiment_client::on_experiment_started(experiment);
    }

    void Controller_server::Controller_experiment_client::on_episode_started(const string &experiment_name) {
        experiment::Start_experiment_response experiment;
        experiment.load(get_experiment_file(experiment_name));
        //controller_server->set_occlusions(experiment.world.occlusions);
        //controller_server->tracking_client.capture.visibility = Location_visibility(controller_server->cells, controller_server->world.cell_shape, controller_server->world.cell_transformation);
        Experiment_client::on_episode_started(experiment_name);
    }

    Controller_server::Controller_experiment_client::Controller_experiment_client() {
    }

    bool Controller_service::set_rotation(float rotation) {
        return ((Controller_server *) _server)->set_rotation(rotation);
    }

    bool Controller_server::set_rotation(float rotation) {
        agent.set_rotation(rotation);
        return true;
    }

    bool Controller_service::set_coordinate(cell_world::Coordinates coordinate) {
        return ((Controller_server *) _server)->set_coordinate(coordinate);
    }

    bool Controller_server::set_coordinate(cell_world::Coordinates coordinate) {
        agent.set_coordinate(coordinate);
        return true;
    }

}

