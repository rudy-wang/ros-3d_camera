#include <uagv_system/Machine.h>



Machine::Machine()
{
    ros::NodeHandle machineNode;
    mSetModeCommandSubscriber = machineNode.subscribe(SETMODE_COMMAND_TOPIC, 1, &Machine::receiveSetModeCommand, this);
    mTaskCommandSubscriber = machineNode.subscribe(TASK_COMMAND_TOPIC, 1, &Machine::receiveTaskCommand, this);
    mErrorMsgSubscriber = machineNode.subscribe(ERROR_MSG_TOPIC, 1, &Machine::receiveErrorMsg, this);
    _mode = new AutoMode(this);
    _state = new WaitingState(this);
    
}
Machine::~Machine()
{
   delete this->getState();
   delete this->getMode();
}


	


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
                  this->getState()->stop();
              } break;
          case PAUSE:
              {
                  this->getState()->pause();
              } break;
          case NAVIGATION:
              {
                  this->getMode()->navigation();
              } break;
          case REMOTE:
              {
                  this->getMode()->remote();
              } break;
          case REGISTRATION:
              {
                  this->getMode()->registration();
              } break;
          case LIFTUP:
              {
                  this->getMode()->liftUp();
              } break;
          case LIFTDOWN:
              {
                  this->getMode()->liftDown();
              } break;
          case TRANSITION:
              {
                  this->getMode()->transition();
              } break;
          case AUTOCHARGING:
              {
                  this->getMode()->autoCharging();
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
                this->getState()->emergencyStopUltrasonic();
            } break;
        case BUMPER:
            {
                this->getState()->pause();
            } break;
        case IR:
            {
               this->getState()->stop(); 
            } break;
        case BUTTON:
            {
                this->getState()->stop();
            } break;
        case CLEAN:
            {
                this->getState()->pause();
            } break;
        default:
            {
                ROS_ERROR("Invalid error message");
            } break;
    }
}
