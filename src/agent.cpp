#include <controller/agent.h>
#include <thread>

using namespace std;

namespace controller {
    double Agent_operational_limits::convert(double v) {
        if (v==0) return stop;
        if (v > 0) { // forward values
            return abs(max_forward - min_forward) * v + min_forward;
        }
        return abs(max_backward - min_backward) * v + min_backward;
    }

    bool Agent::stop() {
        set_right(0);
        set_left(0);
        update();
        return true;
    }

    Tick_agent_move Tick_agent_moves::find_tick_move(cell_world::Move move, int current_orientation) {
        Tick_agent_move tam;
        int target_orientation = -1;
        for (auto tm:turning_moves){
            if (tm.move == move) target_orientation = tm.orientation;
        }
        int orientation_change = (target_orientation - current_orientation + 12) % 12;

        for (auto tm:turning_moves){
            if (orientation_change == tm.orientation) return tm;
        }
        return Tick_agent_move();
    }

    int Tick_agent_move::update_orientation(int current_orientation) {
        return (current_orientation + orientation ) % 12;
    }

    Tick_agent_move Tick_agent_moves::get_forward_move(int current_orientation) {
        return forward_moves[current_orientation % 2];
    }
}