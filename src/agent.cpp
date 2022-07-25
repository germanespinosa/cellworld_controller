#include <controller/agent.h>

using namespace easy_tcp;
using namespace std;

namespace controller {
    bool Agent::stop() {
        set_right(0);
        set_left(0);
        set_speed(0);
        update();
        return true;
    }
}