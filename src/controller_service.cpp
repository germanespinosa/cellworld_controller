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
                                         Agent &agent,
                                         Controller_tracking_client &tracking_client,
                                         Controller_experiment_client &experiment_client):
            agent(agent),
            world(World::get_from_parameters_name("hexagonal", "canonical")),
            cells(world.create_cell_group()),
            paths(world.create_paths(Resources::from("paths").key("hexagonal").key("00_00").key("astar").get_resource<Path_builder>())),
            map(cells),
            navigability(cells, world.cell_shape,Transformation(world.cell_transformation.size, world.cell_transformation.rotation)),
            pid_controller (Json_from_file<Pid_parameters>(pid_config_file_path)),
            tracking_client(tracking_client),
            destination_timer(0),
            experiment_client(experiment_client)
    {
        tracking_client.controller_server = this;
        experiment_client.controller_server = this;
        experiment_client.subscribe();
        tracking_client.subscribe();
        state = Controller_state::Stopped;
        process = thread(&Controller_server::controller_process, this);
    }

    void Controller_server::controller_process() {                      // setting robot velocity
        state = Controller_state::Playing;
        Pid_inputs pi;
        while(state != Controller_state::Stopped){
            robot_mtx.lock();
            if (this->tracking_client.capture.cool_down.time_out()){
            // if there is no information from the tracker
                if (!tracking_client.agent.is_valid() ||
                    state == Controller_state::Paused ||
                    destination_timer.time_out()){
                    agent.set_left(0);
                    agent.set_right(0);
                    agent.update();
                } else {
                    //PID controller
                    pi.location = tracking_client.agent.step.location;
                    pi.rotation = tracking_client.agent.step.rotation;
                    pi.destination = get_next_stop();
                    auto dist = destination.dist(pi.location);
                    if (dist < world.cell_transformation.size / 2) {
                        agent.set_left(0);
                        agent.set_right(0);
                        agent.update();
                    } else {
                        auto robot_command = pid_controller.process(pi, behavior);
                        agent.set_left(robot_command.left);
                        agent.set_right(robot_command.right);
                        agent.update();
                    }
                }
            }
            robot_mtx.unlock();
            //prevents overflowing the robot ( max 10 commands per second)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    bool Controller_server::set_destination(const cell_world::Location &new_destination) {
        cout << "New destination: " << new_destination << endl;
        destination = new_destination;
        destination_timer = Timer(5);
        new_destination_data = true;
        return true;
    }

#define goal_weight 0
#define occlusion_weight 0.0001 //was 0.0001
#define decay 2

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

        auto total_gravity_change = Location(0,0);
        for (auto &cell_r : cells.occluded_cells()) {
            Cell cell = cell_r.get();
            auto distance = cell.location.dist(agent_location);
            auto theta = cell.location.atan(agent_location);
            auto gravity = occlusion_weight / pow(distance,decay);
            total_gravity_change = total_gravity_change.move(theta, gravity);
        }
        auto distance = cells[next_stop].location.dist(agent_location);
        auto theta = agent_location.atan(cells[next_stop].location);
        auto gravity = goal_weight / pow(distance,decay);
        total_gravity_change = total_gravity_change.move(theta, gravity);

        return cells[next_stop].location + total_gravity_change;
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
        auto occlusions_cgb = Resources::from("cell_group").key("hexagonal").key(occlusions).key("occlusions").get_resource<Cell_group_builder>();
        world.set_occlusions(occlusions_cgb);
        cells = world.create_cell_group();
        paths = Paths(world.create_paths(Resources::from("paths").key("hexagonal").key(occlusions).key("astar").key("robot").get_resource<Path_builder>()));
        navigability = Location_visibility(cells, world.cell_shape,Transformation(world.cell_transformation.size * (1 + margin), world.cell_transformation.rotation)); // robot size
        tracking_client.visibility.update_occlusions(cells);
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
        navigability.update_occlusions(cells);
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
                        controller_server->agent.set_left(0);
                        controller_server->agent.set_right(0);
                        controller_server->agent.capture();
                        controller_server->agent.update();
                        controller_server->agent.capture();
                        controller_server->agent.update();
                        controller_server->agent.update();
                        controller_server->send_capture(step.frame);
                    }
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
        visibility.update_occlusions(cells);
        capture.visibility.update_occlusions(cells);
        peeking.peeking_visibility.update_occlusions(cells);
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
        controller_server->set_occlusions(experiment.world.occlusions);
        Experiment_client::on_episode_started(experiment_name);
    }

    Controller_server::Controller_experiment_client::Controller_experiment_client() {
    }



////////////////////////////////////////////////

    mutex prey_robot_mtx;
    string prey_logs_path = "";

    bool Prey_controller_service::set_destination(const cell_world::Location &location) {
        return ((Prey_controller_server *) _server)->set_destination(location);
    }

    bool Prey_controller_service::stop_controller() {
        return true;
    }

    bool Prey_controller_service::pause_controller() {
        return ((Prey_controller_server *) _server)->pause();
    }

    bool Prey_controller_service::resume_controller() {
        return ((Prey_controller_server *) _server)->resume();
    }

    int Prey_controller_service::get_port() {
        string port_str(std::getenv("CONTROLLER_PORT") ? std::getenv("CONTROLLER_PORT") : "4590");
        return atoi(port_str.c_str());
    }

    bool Prey_controller_service::set_behavior(int behavior) {
        return ((Prey_controller_server *) _server)->set_behavior(behavior);
    }

    void Prey_controller_service::set_logs_folder(const string &path) {
        prey_logs_path = path;
        filesystem::create_directory(filesystem::path(prey_logs_path));
    }

    void Prey_controller_server::send_step(const Step &step) {
        if (!clients.empty()) {
            broadcast_subscribed(Message (step.agent_name + "_step", step));
        }
        for (auto local_client: subscribed_local_clients){
            local_client->on_step(step);
        }
    }
    bool Prey_controller_service::tune_controller() {
        return ((Prey_controller_server *) _server)->tune();
    }


    // this should be an int right
    int Prey_controller_service::set_agent_values(const Agent_values &values) {
        ((Prey_controller_server *) _server)->agent.set_left(values.left);
        ((Prey_controller_server *) _server)->agent.set_right(values.right);
        ((Prey_controller_server *) _server)->agent.set_speed(values.speed);
        auto move_number = ((Prey_controller_server *) _server)->agent.update();
        return move_number;
    }

    bool Prey_controller_service::is_move_done() {
        return ((Prey_controller_server *) _server)->agent.is_move_done();
    }



    Prey_controller_server::Prey_controller_server(Tick_agent &agent,
                                         Controller_tracking_client &tracking_client,
                                         Controller_experiment_client &experiment_client):
            agent(agent),
            world(World::get_from_parameters_name("hexagonal", "canonical")),
            cells(world.create_cell_group()),
            paths(world.create_paths(Resources::from("paths").key("hexagonal").key("00_00").key("astar").get_resource<Path_builder>())),
            map(cells),
            navigability(cells, world.cell_shape,Transformation(world.cell_transformation.size, world.cell_transformation.rotation)),
            tick_controller(),
            tracking_client(tracking_client),
            destination_timer(0),
            experiment_client(experiment_client)
    {
        tracking_client.controller_server = this;
        experiment_client.controller_server = this;
        experiment_client.subscribe();
        tracking_client.subscribe();
        state = Controller_state::Stopped;
        process = thread(&Prey_controller_server::controller_process, this);
    }

    void Prey_controller_server::controller_process() {                      // setting robot velocity
        state = Controller_state::Playing;
        //state = Controller_state::Tune;
        Tick_controller_inputs ci;
        // TODO: if move done current = next, prev = current, next = move
        // TODO: figure out way to reinitialize if gamepad intervention occurs -- if gamepad notification mode == initialize
        while(state != Controller_state::Stopped){
            prey_robot_mtx.lock();
            if (this->tracking_client.capture.cool_down.time_out()){
                // if there is no information from the tracker or controller is paused or destination timeout
                // fixed tune logic - if tune while loop is basically skipped however process will continue to run to detect state change
                if (!tracking_client.agent.is_valid() ||
                    state == Controller_state::Paused ||
                    state == Controller_state::Tune ||
                    destination_timer.time_out()){
                    if (state != Controller_state::Tune) {
                        agent.set_left(0); // need to send 0 or will stay at last pwm sent
                        agent.set_right(0);
                        agent.set_speed(-1);
                        move_number = agent.update();
                    }
                } else {
                    // TODO: fix this to work for any spawn location and orientation
                    // Probably PID to get initializing position
                    if (mode == Initialize){
                        cout << "INITIALIZE ROBOT" << endl;
                        // ERROR: the current and next coordinate need to be modified
                        ci.location = tracking_client.agent.step.location;
                        ci.current_coordinate = Coordinates(0,0);//cells[cells.find(ci.location)].coordinates;
                        ci.next_coordinate = ci.current_coordinate.operator+(Coordinates(2,0));  // based on location
                        agent.set_speed(500);
                        agent.set_left(0);
                        agent.set_right(0);
                        move_number = agent.update();
                        cout << "INITIAL MOVE NUM" << move_number << endl;
                        mode = Moving; //TODO: make sure this changes
                    }
                    if (agent.is_move_done() || mode == Waiting){        // TODO: this waits till move is done to get next coordinate need to sort out how to modify this for real robot
                        ci.location = tracking_client.agent.step.location;
                        if (mode!= Waiting){
                            cout << "MOVE NUMBER " << move_number << endl;
                            ci.previous_coordinate = ci.current_coordinate;
                        }
                        ci.current_coordinate = ci.next_coordinate;
                        ci.next_coordinate = get_next_coordinate(ci.current_coordinate);    // this is where the next coordinate is found
                        mode = Ready;
                    }

                    // catch when destination reached ... send 0
                    if (ci.current_coordinate == ci.next_coordinate) {
                        agent.set_left(0);
                        agent.set_right(0);
                        agent.set_speed(0);
                        move_number = agent.update();
                        mode = Waiting;
                        cout << "PREVIOUS COORDINATE: " << ci.previous_coordinate << endl;
                        cout << "CURRENT COORDINATE: " << ci.current_coordinate << endl;
                        cout << "NEXT COORDINATE: " << ci.next_coordinate << endl;
                    } else if (mode == Ready) {
                        auto robot_command = tick_controller.process(ci);
                        agent.set_speed(robot_command.speed);
                        agent.set_left(robot_command.left);
                        agent.set_right(robot_command.right);
                        move_number = agent.update();
                        mode = Moving;
                        // TODO: set previous coordinate to current coordinate
                        // TODO: change mode to move instead of initialize
                    }
                }
            }
            prey_robot_mtx.unlock();
            //prevents overflowing the robot ( max 10 commands per second)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    bool Prey_controller_server::set_destination(const cell_world::Location &new_destination) {
//        cout << "New destination: " << new_destination << endl;
        destination = new_destination;
        destination_timer = Timer(5);
        new_destination_data = true;
        return true;
    }


    // finds next coordinate
    cell_world::Coordinates Prey_controller_server::get_next_coordinate(const cell_world::Coordinates &current_coordinate) {

        auto agent_location = tracking_client.agent.step.location;  // this uses the tracker to find the location TODO: we do not want to use the tracker
        auto destination_cell_index = cells.find(destination);
        //auto agent_cell_index = cells.find(agent_location);
        auto agent_cell_index = cells.find(current_coordinate);
        auto move = paths.get_move(cells[agent_cell_index], cells[destination_cell_index]);  // returns next move
        auto next_stop = cells.find(map[cells[agent_cell_index].coordinates + move]);

        // send next coordinate instead of next location
        return cells[next_stop].coordinates;
    }



    bool Prey_controller_server::tune() {
        state = Controller_state::Tune;
        return true;
    }


    bool Prey_controller_server::pause() {
        state = Controller_state::Paused;
        return true;
    }

    bool Prey_controller_server::resume() {
        if (state == Controller_state::Paused) {
            state = Controller_state::Playing;
            return true;
        }
        return false;
    }

    void Prey_controller_server::set_occlusions(const std::string &occlusions, float margin) {
        auto occlusions_cgb = Resources::from("cell_group").key("hexagonal").key("00_00").key("occlusions").get_resource<Cell_group_builder>();
        world.set_occlusions(occlusions_cgb);
        cells = world.create_cell_group();
        paths = Paths(world.create_paths(Resources::from("paths").key("hexagonal").key(occlusions).key("astar").key("robot").get_resource<Path_builder>()));
        navigability = Location_visibility(cells, world.cell_shape,Transformation(world.cell_transformation.size * (1 + margin), world.cell_transformation.rotation)); // robot size
        tracking_client.visibility.update_occlusions(cells);
    }

    void Prey_controller_server::join() {
        process.join();
        Server::join();
    }

    bool Prey_controller_server::set_behavior(int behavior) {
        this->behavior = static_cast<Behavior> (behavior);
        experiment_client.set_behavior(behavior);
        return true;
    }

    void Prey_controller_server::send_capture(int frame) {
        experiment_client.capture(frame);
    }

    void Prey_controller_server::set_occlusions(Cell_group &cells) {
        navigability.update_occlusions(cells);
    }

    void Prey_controller_server::Controller_tracking_client::on_step(const Step &step) {
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
                prey_robot_mtx.lock();
                auto is_captured = capture.is_captured( predator.location, to_radians(predator.rotation), step.location);
                if (is_captured) {
                    controller_server->agent.set_left(0);
                    controller_server->agent.set_right(0);
                    controller_server->agent.capture();
                    controller_server->agent.update();
                    controller_server->agent.capture();
                    controller_server->agent.update();
                    controller_server->agent.update();
                    controller_server->send_capture(step.frame);
                }
                prey_robot_mtx.unlock();
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

    Prey_controller_server::Controller_tracking_client::Controller_tracking_client(Location_visibility &visibility,
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

    Prey_controller_server::Controller_tracking_client::Controller_tracking_client(cell_world::World world,
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

    void Prey_controller_server::Controller_tracking_client::set_occlusions(Cell_group &cells) {
        visibility.update_occlusions(cells);
        capture.visibility.update_occlusions(cells);
        peeking.peeking_visibility.update_occlusions(cells);
    }

    void Prey_controller_server::Controller_experiment_client::on_experiment_started(
            const experiment::Start_experiment_response &experiment) {
        experiment.save(get_experiment_file(experiment.experiment_name));
        Experiment_client::on_experiment_started(experiment);
    }

    void Prey_controller_server::Controller_experiment_client::on_episode_started(const string &experiment_name) {
        experiment::Start_experiment_response experiment;
        experiment.load(get_experiment_file(experiment_name));
        controller_server->set_occlusions(experiment.world.occlusions);
        Experiment_client::on_episode_started(experiment_name);
    }

    Prey_controller_server::Controller_experiment_client::Controller_experiment_client() {
    }








}

