class Mode;
class State;
#include <uagv_system/MachineBase.h>
/**
 * @class CatgoMachine
 * @author I-chun Han
 * @date 12/29/17
 * @file MachineBase.h
 * @brief A concrete implement Class of all cargo functions, not including functions of machinebase.
 * send message to controlboard triggering lift up and down functions.
 */

class CargoMachine:public MachineBase
{
public:

	// Default Constructor & Destructor
	CargoMachine();
	~CargoMachine();
	
	// Public Methods
	/**
	 * @brief send message to controlboard triggering lift up and down functions. 
	 */
    void executeLiftUp();
    void executeLiftDown();  
    
    void executeTransition();
    void stopLiftUp();
    void stopLiftDown();
    
    void pauseLiftUp();
    void pauseLiftDown();
//////////////////////////////////////////////////////////
    void executeOnMainBoard();
    void executeOnControlBoard();

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


