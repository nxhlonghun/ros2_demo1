#include <iostream>
#include <demo_motion_control/turn_motion_controller.hpp>

namespace motioncontroller
{
    void TurnMotionController::start()
    {
        std::cout << "TurnMotionController started turning." << std::endl;
    }

    void TurnMotionController::stop()
    {
        std::cout << "TurnMotionController stopped turning." << std::endl;
    }
} // namespace motioncontroller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(motioncontroller::TurnMotionController, motioncontroller::MotionController)
