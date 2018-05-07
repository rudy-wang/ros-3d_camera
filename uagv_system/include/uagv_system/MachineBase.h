#include <string>
#include <iostream>
//#include <uagv_system/Mode.h>
//#include <uagv_system/State.h>
/**
 * @class MachineBase
 * @author I-chun Han
 * @date 12/22/17
 * @file MachineBase.h
 * @brief The provide concrete functions in UAGV moving base
 */

class MachineBase
{

public:
    // Default Constructor & Destructor
    MachineBase();
    ~MachineBase();

    // Public Methods

	/**
	 * @brief call ROS services on Mainbaord or send command signal to Controlboard.
     * Execute functions according to Mode-control.
     * govern recovery behavior through State-control. 
	 */
    void executeOnMainBoard();
    void executeOnControlBoard();
    
	/**
	 * @brief basic functions in machine base
	 */
    void executeNavigation();
    void executeRemote();
    void executeRegistration();
    void executeAutoCharging();
       
    void stopNavigation();
    void stopRemeote();
    void stopRegistration();


    void pauseNavigation();
    void pauseRemeote();
    void pauseRegistration();


};

