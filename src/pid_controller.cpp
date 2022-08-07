// TODO:
// 1. ask German better way to store tick values ... maybe json file
// 2. clean code once working**

#include <controller/pid_controller.h>

using namespace cell_world;
using namespace std;

#define RATIO 1.0

namespace controller{
    Controller_outputs Pid_controller::process(const Controller_inputs &inputs, Behavior behavior) {
        // TODO: come up with a better way to compute this difference
        in = inputs;
        auto delta_side = delta_coordinates(in.previous_coordinate, in.current_coordinate);
        auto delta_move = delta_coordinates(in.current_coordinate, in.next_coordinate);
        cout << "delta step: " << delta_side << endl;
        cout << "delta move: " << delta_move << endl;
        // STEP 1: find what side of the hexagon the robot entered (basically its rotation)
        int side = get_side(delta_side);
        // STEP 2: decide move based on side and next coordinate

        cout << "SIDE: " <<side << endl;

        out.left = 100;
        out.right = 100;
        out.speed = 100; // TODO: put this in header
        return out;
    }

    // TODO: understand then get rid of this
    Pid_controller::Pid_controller(const Pid_parameters &parameters): parameters(parameters) {
        timer.reset();
    }

    cell_world::Coordinates Pid_controller::delta_coordinates(const Coordinates &c1, const  Coordinates &c2){
        // final - initial
        return Coordinates( c2.x - c1.x, c2.y - c1.y);
    }
    int Pid_controller::get_side(const Coordinates &delta_side){
        // returns int representing what side of hexagon robot entered (provides rot info without tracker)
        Coordinates side0 = Coordinates(2, 0);
        Coordinates side1 = Coordinates(1, -1);
        Coordinates side2 = Coordinates(-1, -1);
        Coordinates side3 = Coordinates(-2, 0);
        Coordinates side4 = Coordinates(-1, 1);
        Coordinates side5 = Coordinates(1, 1);
        Coordinates sides[] = {side0, side1, side2, side3, side4, side5};
        int side_count = 0;
        for (auto &c: sides){
            if (delta_side == c) return side_count;
            side_count ++;
        }
        cout << " PROBLEM NO SIDE MATCH" <<endl;
        return 1000; // this should never occur so will indicate an error
    }
}