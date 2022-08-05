#include <controller/pid_controller.h>

using namespace cell_world;
using namespace std;

#define RATIO 1.0

namespace controller{
    Controller_outputs Pid_controller::process(const Controller_inputs &inputs, Behavior behavior) {
        // TODO: come up with a better way to compute this difference
        in = inputs;
        auto delta_step = delta_coordinates(in.previous_coordinate, in.current_coordinate);
        auto delta_move = delta_coordinates(in.current_coordinate, in.next_coordinate);
        cout << "delta move: " << delta_move << endl;
        out.left = 100;
        out.right = 100;
        out.speed = 100; // TODO: put this in header
        return out;
    }

    // TODO: get rid of this
    Pid_controller::Pid_controller(const Pid_parameters &parameters): parameters(parameters) {
        timer.reset();
    }

    cell_world::Coordinates Pid_controller::delta_coordinates(const Coordinates &c1, const  Coordinates &c2){
        // final - initial
        return Coordinates( c2.x - c1.x, c2.y - c1.y);
    }
}