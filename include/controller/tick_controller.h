#include <cell_world.h>

namespace controller {

#define ROBOT_SPEED 800

    struct Tick_commands : json_cpp::Json_object {
        Json_object_members(
                Add_member(left);
                Add_member(right);
                Add_member(speed);
                Add_member(move_number);
        )
        int left, right, speed, move_number;
    };

    enum Tick_controller_mode{
        Initialize,
        Moving,
        Ready,
        Waiting
    };

    struct Tick_controller_inputs : json_cpp::Json_object{
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

    struct Tick_controller_outputs : json_cpp::Json_object{
        Json_object_members(
                Add_member(left);
                Add_member(right);
                Add_member(speed);
        )
        double left;
        double right;
        double speed;
    };

    struct Tick_controller : json_cpp::Json_object {
        Json_object_members(
                Add_member(error);
                Add_member(normalized_error);
                Add_member(error_integral);
                Add_member(error_derivative);
                Add_member(last_error);
                Add_member(out);
                Add_member(in);
        );
        Tick_controller_outputs process(const Tick_controller_inputs &);
        static double normalize_error(double);
        cell_world::Coordinates delta_coordinates(const cell_world::Coordinates &c1, const  cell_world::Coordinates &c2);
        int get_side(const cell_world::Coordinates &delta_side);
        std::pair<int,int> get_ticks(const cell_world::Coordinates &delta_move, int side);
        double error{};
        double error_integral{};
        double error_derivative{};
        double normalized_error{};
        double last_error{};
        Tick_controller_outputs out;
        Tick_controller_inputs in;
        cell_world::Timer timer;
    };
}