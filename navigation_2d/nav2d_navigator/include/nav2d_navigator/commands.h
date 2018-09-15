// Definition of commands and possible results for Navigator command service
#ifndef NAVIGATOR_COMMANDS_H
#define NAVIGATOR_COMMANDS_H

#define NAV_STOP_SERVICE		"Stop"
#define NAV_PAUSE_SERVICE		"Pause"
#define NAV_EXPLORE_SERVICE		"StartExploration"
#define NAV_REGISTRATION_SERVICE	"StartRegistration"
#define NAV_LIFTUP_SERVICE		"LiftUp"
#define NAV_LIFTDOWN_SERVICE		"LiftDown"
#define NAV_LIFTHALT_SERVICE		"LiftHalt"
#define NAV_GETMAP_SERVICE		"StartMapping"
#define NAV_LOCALIZE_SERVICE		"StartLocalization"
#define NAV_GOAL_TOPIC			"goal"
#define NAV_STATUS_TOPIC		"nav_status"
#define NAV_MOVE_ACTION			"MoveTo"
#define NAV_EXPLORE_ACTION		"Explore"
#define NAV_REGISTRATION_ACTION		"Registration"
#define NAV_LIFT_ACTION			"Lift"
#define NAV_GETMAP_ACTION		"GetFirstMap"
#define NAV_LOCALIZE_ACTION		"Localize"
#define LS_NODE				"/sick_tim571_2050101"

#define NAV_ST_IDLE		0
#define NAV_ST_NAVIGATING	1
#define NAV_ST_REGISTRATION	2
#define NAV_ST_LIFTING		3
#define NAV_ST_EXPLORING	4
#define NAV_ST_WAITING		5
#define NAV_ST_RECOVERING	6
#define LIFT_ERROR		0
#define LIFT_UP			1
#define LIFT_DOWN		2
#define LIFT_HALT		3
#define LIFT_ONLOAD		4

#endif
