#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "joystick_to_kobuki_controller/joystick_to_kobuki_controller.hpp"

namespace kobuki
{
    class JoystickControllerNodelet : public nodelet::Nodelet
    {
        public:
        JoystickControllerNodelet(){};
        ~JoystickControllerNodelet(){};

        virtual void onInit()
        {
            ros::NodeHandle nh = this->getPrivateNodeHandle();

            std::string name = nh.getUnresolvedNamespace(); //getting a name for the controller nodelet
            int pos = name.find_last_of('/');
            name = name.substr(pos + 1);

            NODELET_INFO_STREAM("Initializing nodelet... ["<< name <<"]");
            controller_.reset(new MotionController(nh, name));

            if (controller_->init())
            {
                NODELET_INFO_STREAM("Nodelet initialised. [" << name << "]");
            }
            else
            {
                NODELET_ERROR_STREAM("Couldn't initialise nodelet! Please restart. [" << name << "]");
            }
        }

        private:
        boot::shared_ptr<MotionController> controller_;

    };
}

PLUGINLIB_EXPORT_CLASS(kobuki::MotionControllerNodelet, nodelet::Nodelet);