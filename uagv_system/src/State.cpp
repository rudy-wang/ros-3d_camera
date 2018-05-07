#include <uagv_system/State.h>
#include <ros/ros.h>
#include <uagv_system/Machine.h>

State* State::transitionState(){}
void State::recovery(){}
void State::setMode(){}
void State::emergencyStopUltrasonic(){}
void State::emergencyStopBumpers(){}
void State::emergencyStopButton(){}
void State::emergencyStopIR(){}
void State::clean(){}
void State::navigation(){}
void State::registration(){}
void State::remote(){}
void State::liftUp(){}    
void State::liftDown(){}
void State::transition(){}
void State::autoCharging(){}
void State::stop(){}
void State::pause(){}
State * State::InitialState(Machine * machine){
    return new WaitingState(machine);
}



State* WaitingState::transitionState(){}
void WaitingState::recovery(){}
void WaitingState::setMode(){}
void WaitingState::emergencyStopUltrasonic(){}
void WaitingState::emergencyStopBumpers(){}
void WaitingState::emergencyStopButton(){} 
void WaitingState::emergencyStopIR(){}
void WaitingState::clean(){}
void WaitingState::navigation(){ROS_INFO("recieve set Navigation in State.cpp");} //{cargoMachine.executeNavigation()}
void WaitingState::registration(){} //{cargoMachine.executeRegistration()}
void WaitingState::remote(){} //{cargoMachine.executeRemote()}
void WaitingState::liftUp(){} //{cargoMachine.executeLiftUp()}    
void WaitingState::liftDown(){} //{cargoMachine.executeLiftDown()}
void WaitingState::transition(){} //{cargoMachine.executeTransition()}
void WaitingState::autoCharging(){} //{cargoMachine.autoCharging()}
void WaitingState::stop(){}
void WaitingState::pause(){}




void NavigationState::recovery(){} //it's empty here, do recovery behavior automatically in ROS
void NavigationState::setMode(){}
void NavigationState::emergencyStopUltrasonic(){} //{cargoMachine.stopNavigation()}
void NavigationState::emergencyStopBumpers(){} //{cargoMachine.stopNavigation()}
void NavigationState::emergencyStopButton(){} //{cargoMachine.stopNavigation()}
void NavigationState::emergencyStopIR(){} //{cargoMachine.stopNavigation()}
void NavigationState::clean(){} //{cargoMachine.executeNavigation()}
void NavigationState::navigation(){}
void NavigationState::registration(){}
void NavigationState::remote(){}
void NavigationState::liftUp(){}    
void NavigationState::liftDown(){}
void NavigationState::transition(){}
void NavigationState::autoCharging(){}
void NavigationState::stop(){} //{cargoMachine.stopNavigation()}
void NavigationState::pause(){}


 
void RemoteState::recovery(){}
void RemoteState::setMode(){}
void RemoteState::emergencyStopUltrasonic(){} //{cargoMachine.stopRemeote()}
void RemoteState::emergencyStopBumpers(){} //{cargoMachine.stopRemeote()}
void RemoteState::emergencyStopButton(){} //{cargoMachine.stopRemeote()}
void RemoteState::emergencyStopIR(){} //{cargoMachine.stopRemeote()}
void RemoteState::clean(){} //{cargoMachine.executeRemote()}
void RemoteState::navigation(){}
void RemoteState::registration(){}
void RemoteState::remote(){}
void RemoteState::liftUp(){}    
void RemoteState::liftDown(){}
void RemoteState::transition(){}
void RemoteState::autoCharging(){}
void RemoteState::stop(){} //{cargoMachine.stopRemeote()}
void RemoteState::pause(){}


   
void RegistrationState::recovery(){}
void RegistrationState::setMode(){}
void RegistrationState::emergencyStopUltrasonic(){} // do nothing!!!
void RegistrationState::emergencyStopBumpers(){} //{cargoMachine.stopRegistration()}
void RegistrationState::emergencyStopButton(){} //{cargoMachine.stopRegistration()}
void RegistrationState::emergencyStopIR(){} //{cargoMachine.stopRegistration()}
void RegistrationState::clean(){} //{cargoMachine.executeRegistration()}
void RegistrationState::navigation(){}
void RegistrationState::registration(){}
void RegistrationState::remote(){}
void RegistrationState::liftUp(){}   
void RegistrationState::liftDown(){}
void RegistrationState::transition(){}
void RegistrationState::autoCharging(){}
void RegistrationState::stop(){} //{cargoMachine.stopRegistration()}
void RegistrationState::pause(){}



   
void LiftUpState::recovery(){}
void LiftUpState::setMode(){}
void LiftUpState::emergencyStopUltrasonic(){} //{cargoMachine.stopLiftUp()}
void LiftUpState::emergencyStopBumpers(){} //{cargoMachine.stopLiftUp()}
void LiftUpState::emergencyStopButton(){} //{cargoMachine.stopLiftUp()}
void LiftUpState::emergencyStopIR(){} //{cargoMachine.stopLiftUp()}
void LiftUpState::clean(){} //{cargoMachine.executeLiftUp}
void LiftUpState::navigation(){}
void LiftUpState::registration(){}
void LiftUpState::remote(){}
void LiftUpState::liftUp(){}   
void LiftUpState::liftDown(){}
void LiftUpState::transition(){}
void LiftUpState::autoCharging(){}
void LiftUpState::stop(){} //{cargoMachine.stopLiftUp()}
void LiftUpState::pause(){}



void LiftDownState::recovery(){}
void LiftDownState::setMode(){}
void LiftDownState::emergencyStopUltrasonic(){} //{cargoMachine.stopLiftDown()}
void LiftDownState::emergencyStopBumpers(){} //{cargoMachine.stopLiftDown()}
void LiftDownState::emergencyStopButton(){} //{cargoMachine.stopLiftDown()}
void LiftDownState::emergencyStopIR(){} //{cargoMachine.stopLiftDown()}
void LiftDownState::clean(){} //{cargoMachine.executeLiftDown()}
void LiftDownState::navigation(){}
void LiftDownState::registration(){}
void LiftDownState::remote(){}
void LiftDownState::liftUp(){}   
void LiftDownState::liftDown(){}
void LiftDownState::transition(){}
void LiftDownState::autoCharging(){}
void LiftDownState::stop(){} //{cargoMachine.stopLiftDown()}
void LiftDownState::pause(){}



void ChargingState::recovery(){}//it's empty here, do recovery behavior automatically in ROS
void ChargingState::setMode(){}
void ChargingState::emergencyStopUltrasonic(){}
void ChargingState::emergencyStopBumpers(){}
void ChargingState::emergencyStopButton(){}
void ChargingState::emergencyStopIR(){}
void ChargingState::clean(){}
void ChargingState::navigation(){}
void ChargingState::registration(){}
void ChargingState::remote(){}
void ChargingState::liftUp(){}    
void ChargingState::liftDown(){}
void ChargingState::transition(){}
void ChargingState::autoCharging(){}
void ChargingState::stop(){}
void ChargingState::pause(){}

