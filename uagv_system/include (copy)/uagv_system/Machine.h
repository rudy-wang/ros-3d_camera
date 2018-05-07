#define SETMODE_COMMAND_TOPIC "setmode_bt"
#define TASK_COMMAND_TOPIC "task_bt"
#define ERROR_MSG_TOPIC "error_msg"
#define STOP 10
#define PAUSE 11
#define NAVIGATION 12
#define REMOTE 13
#define REGISTRATION 14
#define LIFTUP 15
#define LIFTDOWN 16
#define TRANSITION 17
#define AUTOCHARGING 18

#define ULTRASONIC 20
#define BUMPER 21
#define IR 22
#define BUTTON 23
#define CLEAN 24

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <uagv_system/Mode.h>


/**
 * @class Machine
 * @author I-chun Han
 * @date 12/22/17
 * @file Machine.h
 * @brief Machine class is used to receive signals from VCS server, App or controlboard. This is a public class. 
 * There are no concrete implement in this class. After receiving signal, check State and Mode, 
 * then execute requests in concrete implement class such as MachineBase class or CargoMachine class.
 */

class Machine
{
friend class Mode;
friend class AutoMode;
friend class ManualMode;
friend class DebugMode;
friend class ErrorMode;
friend class WarningMode;

friend class State;
friend class WaitingState;
friend class NavigationState;
friend class RemoteState;
friend class RegistrationState;
friend class LiftUpState;
friend class LiftDownState;
friend class ChargingState;


private:
    State* _state;
    Mode* _mode;

    ros::Subscriber mSetModeCommandSubscriber;
    ros::Subscriber mTaskCommandSubscriber;
    ros::Subscriber mErrorMsgSubscriber;
    int tasktype;
    int errorType;


protected:
State* getState() const {return _state;};
void setState(State* newState) {_state = newState;};
void changeState(State* newState)
    { 
      if (newState != this->getState())
      {
        delete this->getState();
        this->setState(newState);
      }
    };
Mode* getMode() const {return _mode;};
void setMode(Mode* newMode) {_mode = newMode;};
void changeMode(Mode* newMode)
    { ROS_INFO("set autoMode 1");
      if (newMode != this->getMode())
      {
        delete this->getMode();
        this->setMode(newMode);
      }
    };

public:
	// Default Constructor & Destructor
	Machine();
	~Machine();
	
	// Public Methods
    	/**
	 * @brief receive set mode command from APP or server. 
     * Call funcctions through both State-control and Mode-control.
	 * @param md Mode-class: automode, manualMode, debugMode, errorMode, warningMode
	 */
    void receiveSetModeCommand(const std_msgs::Int16::ConstPtr& setModemsg);
    void receiveTaskCommand(const std_msgs::Int16::ConstPtr& taskMsg);
    void receiveErrorMsg(const std_msgs::Int16::ConstPtr& errorMsg);
	

};


