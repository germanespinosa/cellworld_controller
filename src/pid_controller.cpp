// TODO:
// 1. ask German better way to store tick values ... maybe json file
// 2. CLEAN!!! code once working**

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

        // STEP 1: find what side of the hexagon the robot entered (basically its rotation)
        int side = get_side(delta_side);
        // STEP 2: decide move based on side and next coordinate
        std::pair<int,int> tick = get_ticks(delta_move, side);


        out.left = tick.first;
        out.right = tick.second;
//        if
        out.speed = ROBOT_SPEED;
        // TODO: come up with way to assign speed based on move for time being could do logic based on out.l/r
        // see if vector pair - triple
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
        Coordinates side1 = Coordinates(1, 1);
        Coordinates side2 = Coordinates(-1, 1);
        Coordinates side3 = Coordinates(-2, 0);
        Coordinates side4 = Coordinates(-1, -1);
        Coordinates side5 = Coordinates(1, -1);
        Coordinates sides[] = {side0, side1, side2, side3, side4, side5};
        int side_count = 0;
        for (auto &c: sides){
            if (delta_side == c) return side_count;
            side_count ++;
        }
        cout << "PROBLEM NO SIDE MATCH" <<endl;
        cout << "DELTA SIDE " << delta_side <<endl;
        return 1000; // this should never occur so will indicate an error
    }
    std::pair<int,int> Pid_controller::get_ticks(const cell_world::Coordinates &delta_move, int side){
        // TODO:get these values from json file ints m0_L, m0_R ... etc
//        vector<pair<int,int>> move_ticks = {{1,600},{231,535},{432,432},{535,231},{600,1},{450,-450}}; // m0,m1,m2,m3,m4,m5    212,815
        vector<pair<int,int>> move_ticks = {{5,600},{73,284},{256,256},{284,73},{600,5},{338,-338}};
        // side 0 condition
        Coordinates m0 = Coordinates(-1, 1);
        Coordinates m1 = Coordinates(1, 1);
        Coordinates m2 = Coordinates(2, 0);
        Coordinates m3 = Coordinates(1, -1);
        Coordinates m4 = Coordinates(-1, -1);
        Coordinates m5 = Coordinates(-2, 0);
        Coordinates deltas[] = {m0, m1, m2, m3, m4, m5};
        int count = 0;
        int delta_index;
        // find match
        for (auto &d: deltas){
            if (delta_move == d) delta_index = count;
            count ++;
        }
        // deal with side
        delta_index = delta_index + side;
        if (delta_index > 5){
            delta_index = delta_index - 6;
        }

        return move_ticks[delta_index];
    }
}

//{"m1": {'L': 212, 'R': 815},
//"m2": {'L': 231, 'R': 535},
//"m3": {'L': 432, 'R': 432},
//"m4": {'L': 535, 'R': 231},
//"m5": {'L': 815, 'R': 212},
//"m6": {'L': 450, 'R': -450},
//"m7": {'L': 420, 'R': -420},
//"m8": {'L': 216, 'R': 216}}