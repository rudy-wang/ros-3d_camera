#ifndef O_H
#define O_H

#include "system_base.h"
#include "ros_msg.h"

class Registration;

class LaserMsgLite;

class Object
{
	public:
		Object( Registration *assignedBy ){ task = assignedBy; }
		Object(){ task = NULL; }
		~Object(){ task = NULL; }
		virtual bool scan( sensor_msgs::LaserScan msg_new, float meanDepth );
		virtual void localization();
		
		Registration *task;
};

class Charging : public Object
{
	public:
		Charging( Registration *assignedBy ) : Object( assignedBy ){}
		Charging(){ task = NULL; }
		~Charging(){ task = NULL; }
		virtual bool scan( sensor_msgs::LaserScan msg_new, float meanDepth );
		virtual void localization();
};

class LifttingOutside : public Object
{
	public:
		LifttingOutside( Registration *assignedBy ) : Object( assignedBy ){}
		LifttingOutside(){ task = NULL; }
		~LifttingOutside(){ task = NULL; }
		virtual bool scan( sensor_msgs::LaserScan msg_new, float meanDepth );
		virtual void localization();
};

class LifttingUnder : public Object
{
	public:
		LifttingUnder( Registration *assignedBy ) : Object( assignedBy ){}
		LifttingUnder(){ task = NULL; }
		~LifttingUnder(){ task = NULL; }
		virtual bool scan( sensor_msgs::LaserScan msg_new, float meanDepth );
		virtual void localization();
};
#endif
