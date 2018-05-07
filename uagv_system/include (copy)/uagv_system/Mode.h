#include <uagv_system/State.h>

/**
 * @class Mode and five subclasses
 * @author I-chun Han
 * @date 12/22/17
 * @file MachineBase.h
 * @brief The main states and modes control and provide functions that UAGV moving base
 */
class State;
class Machine;
class Mode
{
  friend class Machine;
  friend class State;
  public:
	// Default Constructor & Destructor
	Mode(Machine * machine);
	~Mode();
	
	// Public Methods
	/**
	 * @brief Callback function to receive set mode command from APP or server. 
     * Call funcctions through both State-control and Mode-control.
	 * @param md Mode-class: automode, manualMode, debugMode, errorMode, warningMode
	 */
    void setMode();
    virtual void navigation();
    virtual void registration();
    virtual void remote();
    virtual void liftUp();   
    virtual void liftDown();
    virtual void transition();
    virtual void autoCharging();
  protected:
    State * state() const {return _state;};
	
  private:
    Machine * _machine;
    State * _state;
};


class AutoMode : public Mode
{
  friend class Machine;
  friend class State;
public:
	// Default Constructor & Destructor
	AutoMode(Machine * machine);
	~AutoMode();
    
    void navigation();
    void registration();
    void remote();
    void liftUp() ;
    void liftDown();
    void transition();
    void autoCharging();

  protected:
    State * state() const {return _state;};
	
  private:
    Machine * _machine;
    State * _state;

};

class ManualMode : public Mode
{
public:
	// Default Constructor & Destructor
	ManualMode(Machine * machine);
	~ManualMode();
    
    void navigation();
    void registration();
    void remote();
    void liftUp();
    void liftDown();
    void transition();
    void autoCharging();
  protected:
    State * state() const {return _state;};
	
  private:
    Machine * _machine;
    State * _state;
};


class DebugMode : public Mode
{
public:
	// Default Constructor & Destructor
	DebugMode(Machine * machine);
	~DebugMode();
    
    void navigation();
    void registration();
    void remote();
    void liftUp() ; 
    void liftDown();
    void transition();
    void autoCharging();

  protected:
    State * state() const {return _state;};
	
  private:
    Machine * _machine;
    State * _state;

};
class ErrorMode : public Mode
{
public:
	// Default Constructor & Destructor
	ErrorMode(Machine * machine);
	~ErrorMode();
    
    void navigation();
    void registration();
    void remote();
    void liftUp();
    void liftDown();
    void transition();
    void autoCharging();

  protected:
    State * state() const {return _state;};
	
  private:
    Machine * _machine;
    State * _state;

};

class WarningMode : public Mode
{
public:
	// Default Constructor & Destructor
	WarningMode(Machine * machine);
	~WarningMode();
   
    void navigation();
    void registration();
    void remote();
    void liftUp(); 
    void liftDown();
    void transition();
    void autoCharging();

  protected:
    State * state() const {return _state;};
	
  private:
    Machine * _machine;
    State * _state;

};



