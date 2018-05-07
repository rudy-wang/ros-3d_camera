#ifndef STATE_H 
#define STATE_H

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
    State * InitialState(Machine * machine);
    
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

private:
    Machine * _context;
    CargoMachine *_cargoMachine;

protected:
    Machine * context() const {return _context;};
    CargoMachine *cargoMachine(){return _cargoMachine;}
    void      context(Machine * newMachine) {_context = newMachine;};
    virtual State * transitionState() = 0;
public:
    State(Machine * machine):_context(machine){};
    State(const State * source):_context(source->context()){};
    ~State(){}
};


class WaitingState : public State
{
public:  
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
private:
    CargoMachine *_cargoMachine;
protected:
    CargoMachine *cargoMachine(){return _cargoMachine;}
    virtual State * transitionState();

public:
    WaitingState(Machine * machine):State(machine){};
    WaitingState(const State * source):State(source){};
    ~WaitingState();
};

class NavigationState : public State
{
public:
    
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
public:
    NavigationState(Machine * machine):State(machine){};
    NavigationState(const State * source):State(source){};
    ~NavigationState();
};


class RemoteState : public State
{
public:    
    
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
public:
    RemoteState(Machine * machine):State(machine){};
    RemoteState(const State * source):State(source){};
    ~RemoteState();

};
class RegistrationState : public State
{
public:
    
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
public:
    RegistrationState(Machine * machine):State(machine){};
    RegistrationState(const State * source):State(source){};
    ~RegistrationState();
};

class LiftUpState : public State
{
public:
    
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
public:
    LiftUpState(Machine * machine):State(machine){};
    LiftUpState(const State * source):State(source){};
    ~LiftUpState();
};

class LiftDownState : public State
{
public:
    
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
public:
    LiftDownState(Machine * machine):State(machine){};
    LiftDownState(const State * source):State(source){};
    ~LiftDownState();
};

class ChargingState : public State
{
public:
    
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
public:
    ChargingState(Machine * machine):State(machine){};
    ChargingState(const State * source):State(source){};
    ~ChargingState();
};
#endif
