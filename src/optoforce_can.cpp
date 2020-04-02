#include "optoforce_can/optoforce_can.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "optoforce_can_node");
    ros::NodeHandle nh;
    optoforcecan_ROS ft(nh);

    static int running = 1;
    unsigned int  i;
    
    ft.InitDriver();
    
    while(1)
    {
        ft.calcForceMoment();
        ft.FTpublish();
    }

    return 0;
}