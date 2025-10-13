#include <iostream>
#include "demo_motion_control/straight_motion_controller.hpp"

namespace motioncontroller
{
    void StraightMotionController::start()
    {
        std::cout << "SpinMotionController started straighting." << std::endl;
    }

    void StraightMotionController::stop()
    {
        std::cout << "SpinMotionController stopped straighting." << std::endl;
    }
} // namespace motioncontroller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(motioncontroller::StraightMotionController, motioncontroller::MotionController)
