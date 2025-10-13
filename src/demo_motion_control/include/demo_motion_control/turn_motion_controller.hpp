#ifndef TURN_MOTION_CONTROLLER_HPP
#define TURN_MOTION_CONTROLLER_HPP

#include "demo_motion_control/motion_control_interface.hpp"

namespace motioncontroller
{
    class TurnMotionController : public MotionController
    {
    private:
    public:
        void start() override;
        void stop() override;
    };
}

#endif