#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <yocs_controllers/default_controller.hpp>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/Led.h>
#include <kobuki>


namespace kobuki
{

    class MotionController : yocs::MotionController
    {
        public:
        //signatures for your constructors
        MotionController(ros::NodeHandle& nh, std::string& name) : Controller(), nh_(nh), name_(name){};

        //the destructor
        ~MotionController()
        {
            setBaseControl(0,0); //(linear velocity = 0, rotational velocity = 0);
        };

        bool init()
        {
            //CB may mean "callback" in response to receiving a message.
            //These have publishers that technically aren't nodes, rostopic pub command publishes to these topics 
            enable_controller_subscriber_ = nh_.subscribe("enable", 10, &MotionController::enableCB, this);
            disable_controller_subscriber_ = nh_.subscribe("disable", 10, &MotionController::disableCB, this);

            //Subscribes and reads message from the joystick
            joystick_subscriber_ = nh.subscribe("joystick", 10, &MotionController::joystickEventCB, this);
        }

        
        private:
        ros::NodeHandle nh_;
        std::string name_;
        ros::Subscriber enable_controller_subscriber_, disable_controller_subscriber_;
        ros::Subscriber joystick_event_subscriber_;
        double linearSpeed;
        double rotationalSpeed;

        //logging output of enabling and disabling the controller
        //"msg" represents the incoming topic message
        void enableCB(const std_msgs::EmptyConstPtr msg);
        void disableCB(const std_msgs::EmptyConstPtr msg);

        //responding to message published by joystick
        void joystickEventCB(double lVel, double rVel);

    };

    void MotionController::enableCB(const std_msgs::EmptyConstPtr msg)
    {
        //"this" is a pointer to the memory addres of the current object
        //enable() is from yos_controller::enable(), a member function 
        //checks if the controller is enabled; line 77 (http://docs.ros.org/en/kinetic/api/yocs_controllers/html/default__controller_8hpp_source.html)
        //"->" means accessing a member function of the pointer, which makes sense since 
        //MotionController inherits from yos_controller

        if (this->enable())
        {
            ROS_INFO_STREAM("Controller has been enabled. [" << name_ <<"]");
        }
        else
        {
            ROS_INFO_STREAM("Controller was already enabled. [" << name_ <<"]");
        }
    };

    void MotionController::disableCB(const std_msgs::EmptyConstPtr msg)
    {
        //MotionController class inherits from yocs_controller class
        //disable() belongs to the yocs_controller class
        //checks if the controller has been disabled
        if (this->disable())
        {
            ROS_INFO_STREAM("Controller has been disabled. [" << name_ <<"]");
        }
        else
        {
            ROS_INFO_STREAM("Controller was already disabled. [" << name_ <<"]");
        }
    };

    //When a message from the joystick node is received, this function is called.
    //This function should control the motors based on the message received from joystick
    //one parameter: an array with 2 elements, the first element should be lVel, the second rVel
    void joystickEventCB(const std_msgs::Int16MultiArray::ConstPtr& msg)
    {
        //value ranges: -100 to 100
        int lVal = msg[0];
        int rVal = msg[1];

        //Translating values to velocity 
        float lVel = float(lVal) * 1.5 / 100.0;
        float rVel = float(rVal) * 1.5 / 100.0;

        cout << "linear velocity: " << lVel << endl;
        cout << "rotational velocity: " << rVel << endl;
        // setBaseControl(lVel, rVel);

    };


}