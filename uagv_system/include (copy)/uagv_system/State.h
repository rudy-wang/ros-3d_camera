class Mode;
#include <uagv_system/CargoMachine.h>
/**
 * @class State and six subclasses
 * @author I-chun Han
 * @date 12/22/17
 * @file MachineBase.h
 * @brief The main states and modes control and provide functions that UAGV moving base
 */
class Machine;
class CargoMachine;
class State
{
friend class Machine;         
friend class CargoMachine; 
public:
    // use Mode and MachineBase class 

    
	// Default Constructor & Destructor
	State(Machine * machine);
	~State();
	
	// Public Methods
	/**
	 * @brief Callback function to receive set mode command from APP or server. 
     * Call funcctions through both State-control and Mode-control.
	 * @param md Mode-class: automode, manualMode, debugMode, errorMode, warningMode
	 */
    
    virtual void recovery();
    virtual void setMode();
    virtual void emergencyStopUltrasonic();
    virtual void emergencyStopBumpers();
    virtual void emergencyStopButton();
    virtual void emergencyStopIR();
    virtual void clean();
    virtual void navigation();
    virtual void registration();
    virtual void remote();
    virtual void liftUp() ;  
    virtual void liftDown();
    virtual void transition();
    virtual void autoCharging();
    virtual void stop();
    virtual void pause();

protected:
    CargoMachine *cargoMachine(){return _cargoMachine;}
	// Internal Methods
private:
    CargoMachine *_cargoMachine;
};


class WaitingState : public State
{
public:
	// Default Constructor & Destructor
	WaitingState(Machine * machine);
	~WaitingState();
    
    void recovery(); 
    void setMode();
    void emergencyStopUltrasonic();
    void emergencyStopBumpers();
    void emergencyStopButton();
    void clean();
    void navigation();
    void registration();
    void remote();
    void liftUp()  ; 
    void liftDown();
    void transition();
    void autoCharging();
    void stop();
    void pause();

protected:
    CargoMachine *cargoMachine(){return _cargoMachine;}
	// Internal Methods
private:
    CargoMachine *_cargoMachine;
};

class NavigationState : public State
{
public:
	// Default Constructor & Destructor
	NavigationState(Machine * machine);
	~NavigationState();
    
    void recovery(); //it's empty here, do recovery behavior automatically in ROS
    void setMode();
    void emergencyStopUltrasonic();
    void emergencyStopBumpers();
    void emergencyStopButton();
    void emergencyStopIR();
    void clean();
    void navigation();
    void registration();
    void remote();
    void liftUp();   
    void liftDown();
    void transition();
    void autoCharging();
    void stop();
    void pause();

protected:
    CargoMachine *cargoMachine(){return _cargoMachine;}
	// Internal Methods
private:
    CargoMachine *_cargoMachine;
};


class RemoteState : public State
{
public:
	// Default Constructor & Destructor
	RemoteState(Machine * machine);
	~RemoteState();
    
    void recovery();
    void setMode();
    void emergencyStopUltrasonic(); //{cargoMachine.stopRemeote();};
    void emergencyStopBumpers(); //{cargoMachine.stopRemeote();};
    void emergencyStopButton(); //{cargoMachine.stopRemeote();};
    void emergencyStopIR(); //{cargoMachine.stopRemeote();};
    void clean(); //{cargoMachine.executeRemote();};
    void navigation();
    void registration();
    void remote();
    void liftUp();    
    void liftDown();
    void transition();
    void autoCharging();
    void stop(); //{cargoMachine.stopRemeote();};
    void pause();

protected:
    CargoMachine *cargoMachine(){return _cargoMachine;}
	// Internal Methods
private:
    CargoMachine *_cargoMachine;

};
class RegistrationState : public State
{
public:
	// Default Constructor & Destructor
	RegistrationState(Machine * machine);
	~RegistrationState();
    
    void recovery();
    void setMode();
    void emergencyStopUltrasonic(); // do nothing!!!
    void emergencyStopBumpers(); //{cargoMachine.stopRegistration();};
    void emergencyStopButton(); //{cargoMachine.stopRegistration();};
    void emergencyStopIR(); //{cargoMachine.stopRegistration();};
    void clean(); //{cargoMachine.executeRegistration();};
    void navigation();
    void registration();
    void remote();
    void liftUp();   
    void liftDown();
    void transition();
    void autoCharging();
    void stop(); //{cargoMachine.stopRegistration();};
    void pause();

protected:
    CargoMachine *cargoMachine(){return _cargoMachine;}
	// Internal Methods
private:
    CargoMachine *_cargoMachine;
};

class LiftUpState : public State
{
public:
	// Default Constructor & Destructor
	LiftUpState(Machine * machine);
	~LiftUpState();
    
    void recovery();
    void setMode();
    void emergencyStopUltrasonic(); //{cargoMachine.stopLiftUp();};
    void emergencyStopBumpers(); //{cargoMachine.stopLiftUp();};
    void emergencyStopButton(); //{cargoMachine.stopLiftUp();};
    void emergencyStopIR(); //{cargoMachine.stopLiftUp()};
    void clean(); //{cargoMachine.executeLiftUp};
    void navigation();
    void registration();
    void remote();
    void liftUp();  
    void liftDown();
    void transition();
    void autoCharging();
    void stop(); //{cargoMachine.stopLiftUp();};
    void pause();

protected:
    CargoMachine *cargoMachine(){return _cargoMachine;}
	// Internal Methods
private:
    CargoMachine *_cargoMachine;

};

class LiftDownState : public State
{
public:
	// Default Constructor & Destructor
	LiftDownState(Machine * machine);
	~LiftDownState();
    
    void recovery();
    void setMode();
    void emergencyStopUltrasonic(); //{cargoMachine.stopLiftDown();};
    void emergencyStopBumpers(); //{cargoMachine.stopLiftDown();};
    void emergencyStopButton(); //{cargoMachine.stopLiftDown();};
    void emergencyStopIR(); //{cargoMachine.stopLiftDown();};
    void clean(); //{cargoMachine.executeLiftDown();};
    void navigation();
    void registration();
    void remote();
    void liftUp() ;
    void liftDown();
    void transition();
    ;void autoCharging();
    void stop(); //{cargoMachine.stopLiftDown();};
    void pause();

protected:
    CargoMachine *cargoMachine(){return _cargoMachine;}
	// Internal Methods
private:
    CargoMachine *_cargoMachine;

};

class ChargingState : public State
{
public:
	// Default Constructor & Destructor
	ChargingState(Machine * machine);
	~ChargingState();
    
    void recovery(); //it's empty here, do recovery behavior automatically in ROS
    void setMode();
    void emergencyStopUltrasonic();
    void emergencyStopBumpers();
    void emergencyStopButton();
    void emergencyStopIR();
    void clean();
    void navigation();
    void registration();
    void remote();
    void liftUp();
    void liftDown();
    void transition();
    void autoCharging();
    void stop();
    void pause();

protected:
    CargoMachine *cargoMachine(){return _cargoMachine;}
	// Internal Methods
private:
    CargoMachine *_cargoMachine;
};
