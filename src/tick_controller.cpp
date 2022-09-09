// TODO:
// 1. ask German better way to store tick values ... maybe json file
// 2. CLEAN!!! code once working**

#include <controller/tick_controller.h>

using namespace cell_world;
using namespace std;

#define RATIO 1.0

namespace controller{
    Tick_controller_outputs Tick_controller::process(const Tick_controller_inputs &inputs) {
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

    cell_world::Coordinates Tick_controller::delta_coordinates(const Coordinates &c1, const  Coordinates &c2){
        // final - initial
        return Coordinates( c2.x - c1.x, c2.y - c1.y);
    }
    int Tick_controller::get_side(const Coordinates &delta_side){
        // returns int representing what side of hexagon robot entered (provides rot info without tracker)
        Coordinates side0 = Coordinates(2, 0);
        Coordinates side1 = Coordinates(1, 1);
        Coordinates side2 = Coordinates(-1, 1);
        Coordinates side3 = Coordinates(-2, 0);
        Coordinates side4 = Coordinates(-1, -1);
        Coordinates side5 = Coordinates(1, -1);
        Coordinates sides[] = {side0, side1, side2, side3, side4, side5};
        int side_count = 0;
        // loops through all sides to find match
        for (auto &c: sides){
            if (delta_side == c) return side_count; // return side match
            side_count ++;
        }
        cout << "PROBLEM NO SIDE MATCH" <<endl;
        cout << "DELTA SIDE " << delta_side <<endl;
        return 1000; // this should never occur so will indicate an error
    }
    std::pair<int,int> Tick_controller::get_ticks(const cell_world::Coordinates &delta_move, int side){
        // TODO:get these values from json file ints m0_L, m0_R ... etc
        vector<pair<int,int>> move_ticks = {{1,600},{231,535},{432,432},{535,231},{600,1},{450,-450}}; // m0,m1,m2,m3,m4,m5    212,815
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

