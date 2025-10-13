#include <iostream>
#include <demo_motion_control/spin_motion_controller.hpp>

namespace motioncontroller
{
    void SpinMotionController::start()
    {
        std::cout << "SpinMotionController started spinning." << std::endl;
    }

    void SpinMotionController::stop()
    {
        std::cout << "SpinMotionController stopped spinning." << std::endl;
    }
} // namespace motioncontroller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(motioncontroller::SpinMotionController, motioncontroller::MotionController)
