#include <uagv_system/Machine.h>
#include <uagv_system/Mode.h>
#include <uagv_system/State.h>

/*
Machine::Machine():_state(State::InitialState(this)),_mode(Mode::InitialMode(this))
{
    ros::NodeHandle machineNode;
    mSetModeCommandSubscriber = machineNode.subscribe(SETMODE_COMMAND_TOPIC, 1, &Machine::receiveSetModeCommand, this);
    mTaskCommandSubscriber = machineNode.subscribe(TASK_COMMAND_TOPIC, 1, &Machine::receiveTaskCommand, this);
    mErrorMsgSubscriber = machineNode.subscribe(ERROR_MSG_TOPIC, 1, &Machine::receiveErrorMsg, this);
    
}
Machine::~Machine()
{
   delete this->getState();
   delete this->getMode();
}


*/	


void Machine::receiveSetModeCommand(const std_msgs::Int16::ConstPtr& setModemsg)
{
  if(setModemsg->data==0)
  {   
 
      Mode* autoMode = new AutoMode(this);
      changeMode(autoMode);
  }
  else if(setModemsg->data==1)
  {
      changeMode(new ManualMode(this));    
  }
      else if(setModemsg->data==2)
  {
      changeMode(new DebugMode(this));
  } 
  else 
  {
      ROS_ERROR("Invalid mode setting");
  }

  changeState(new WaitingState(this));
}


void Machine::receiveTaskCommand(const std_msgs::Int16::ConstPtr& taskMsg)
{
  tasktype = taskMsg->data;
      switch(tasktype)
      {
          case STOP:
              {
                  getState()->stop();
              } break;
          case PAUSE:
              {
                  getState()->pause();
              } break;
          case NAVIGATION:
              {
                  getMode()->navigation();
              } break;
          case REMOTE:
              {
                  getMode()->remote();
              } break;
          case REGISTRATION:
              {
                  getMode()->registration();
              } break;
          case LIFTUP:
              {
                  getMode()->liftUp();
              } break;
          case LIFTDOWN:
              {
                  getMode()->liftDown();
              } break;
          case TRANSITION:
              {
                  getMode()->transition();
              } break;
          case AUTOCHARGING:
              {
                  getMode()->autoCharging();
              } break;
          default:
              {
                  ROS_ERROR("Invalid task command");
              } break;
      }
}



void Machine::receiveErrorMsg(const std_msgs::Int16::ConstPtr& errorMsg)
{
    errorType = errorMsg->data;
    switch(errorType)
    {
        case ULTRASONIC:
            {
                getState()->emergencyStopUltrasonic();
            } break;
        case BUMPER:
            {
                getState()->pause();
            } break;
        case IR:
            {
               getState()->stop(); 
            } break;
        case BUTTON:
            {
                getState()->stop();
            } break;
        case CLEAN:
            {
                getState()->pause();
            } break;
        default:
            {
                ROS_ERROR("Invalid error message");
            } break;
    }
}
