#include <uagv_system/Mode.h>
#include <ros/ros.h>

Mode::Mode(Machine * machine){}
Mode::~Mode(){}	
void Mode::setMode(){}
void Mode::navigation(){}
void Mode::registration(){}
void Mode::remote(){}
void Mode::liftUp(){}   
void Mode::liftDown(){}
void Mode::transition(){}
void Mode::autoCharging(){}


AutoMode::AutoMode(Machine * machine){}
AutoMode::~AutoMode(){}    
void AutoMode::navigation()
{
    ROS_INFO("recieve set Navigation in Mode.cpp");
    //ROS_INFO(state());
    this->getState()->navigation();
}
void AutoMode::registration(){}
void AutoMode::remote(){}
void AutoMode::liftUp(){}  
void AutoMode::liftDown(){}
void AutoMode::transition(){this->state()->transition();}
void AutoMode::autoCharging(){this->state()->autoCharging();}


ManualMode::ManualMode(Machine * machine){}
ManualMode::~ManualMode(){}    
void ManualMode::navigation(){this->state()->navigation();}
void ManualMode::registration(){this->state()->registration();}
void ManualMode::remote(){this->state()->remote();}
void ManualMode::liftUp(){this->state()->liftUp();}  
void ManualMode::liftDown(){this->state()->liftDown();} 
void ManualMode::transition(){}
void ManualMode::autoCharging(){}


DebugMode::DebugMode(Machine * machine){}
DebugMode::~DebugMode(){}
void DebugMode::navigation(){}
void DebugMode::registration(){}
void DebugMode::remote(){}
void DebugMode::liftUp(){}  
void DebugMode::liftDown(){}
void DebugMode::transition(){}
void DebugMode::autoCharging(){}


ErrorMode::ErrorMode(Machine * machine){}
ErrorMode::~ErrorMode(){}    
void ErrorMode::navigation(){}
void ErrorMode::registration(){}
void ErrorMode::remote(){}
void ErrorMode::liftUp(){}  
void ErrorMode::liftDown(){}
void ErrorMode::transition(){}
void ErrorMode::autoCharging(){}


WarningMode::WarningMode(Machine * machine){}
WarningMode::~WarningMode(){}   
void WarningMode::navigation(){}
void WarningMode::registration(){}
void WarningMode::remote(){}
void WarningMode::liftUp(){}  
void WarningMode::liftDown(){}
void WarningMode::transition(){}
void WarningMode::autoCharging(){}



