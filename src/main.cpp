#include "ros/ros.h"
#include "PrmController.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "a4_prm");
    // Create nodehandle
    ros::NodeHandle nh;
    // Declare a Prm Controller object 
    std::shared_ptr<PrmController> gc(new PrmController(nh));
    // Start spinning threads until program shuts down
    ros::spin();
    ros::shutdown();
    return 0;
}
