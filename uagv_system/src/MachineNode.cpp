#include <ros/ros.h>
#include <uagv_system/Machine.h>

/**
 * @class MachineBase
 * @author I-chun Han
 * @date 12/22/17
 * @file MachineBase.h
 * @brief Machine class is used to receive signals from VCS server, App or controlboard. This is a public class. 
 * There are no concrete implement in this class. After receiving signal, check State and Mode, 
 * then execute requests in concrete implement class such as MachineBase class or CargoMachine class.
 */





int main(int argc, char **argv)
{
	ros::init(argc, argv, "MachineNode");
	Machine machine;
	
	ros::spin();
        return 0;
}
