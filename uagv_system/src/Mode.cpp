#include <uagv_system/Mode.h>
#include <ros/ros.h>
#include <uagv_system/Machine.h>
//#include <uagv_system/State.h>

void Mode::setMode(){}
void Mode::navigation(){}
void Mode::registration(){}
void Mode::remote(){}
void Mode::liftUp(){}   
void Mode::liftDown(){}
void Mode::transition(){}
void Mode::autoCharging(){}
Mode * Mode::InitialMode(Machine * machine){
    return new AutoMode(machine);
}




void AutoMode::navigation()
{
    ROS_INFO("recieve set Navigation in Mode.cpp");
    //ROS_INFO(state());
    this->context()->getState()->navigation();
}
void AutoMode::registration(){}
void AutoMode::remote(){}
void AutoMode::liftUp(){}  
void AutoMode::liftDown(){}
void AutoMode::transition(){this->context()->getState()->transition();}
void AutoMode::autoCharging(){this->context()->getState()->autoCharging();}


  
void ManualMode::navigation(){this->context()->getState()->navigation();}
void ManualMode::registration(){this->context()->getState()->registration();}
void ManualMode::remote(){this->context()->getState()->remote();}
void ManualMode::liftUp(){this->context()->getState()->liftUp();}  
void ManualMode::liftDown(){this->context()->getState()->liftDown();} 
void ManualMode::transition(){}
void ManualMode::autoCharging(){}



void DebugMode::navigation(){}
void DebugMode::registration(){}
void DebugMode::remote(){}
void DebugMode::liftUp(){}  
void DebugMode::liftDown(){}
void DebugMode::transition(){}
void DebugMode::autoCharging(){}


  
void ErrorMode::navigation(){}
void ErrorMode::registration(){}
void ErrorMode::remote(){}
void ErrorMode::liftUp(){}  
void ErrorMode::liftDown(){}
void ErrorMode::transition(){}
void ErrorMode::autoCharging(){}


void WarningMode::navigation(){}
void WarningMode::registration(){}
void WarningMode::remote(){}
void WarningMode::liftUp(){}  
void WarningMode::liftDown(){}
void WarningMode::transition(){}
void WarningMode::autoCharging(){}



