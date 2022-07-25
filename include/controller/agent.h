#pragma once
#include <easy_tcp.h>
#include <json_cpp.h>

namespace controller {
    struct Agent {
        virtual void set_left(int) = 0;
        virtual void set_right(int) = 0;
        virtual void set_speed(int) = 0;
        virtual void capture() = 0;
        virtual bool update() = 0;
        virtual bool stop();
    };
}