#ifndef MODE_H 
#define MODE_H

//#include <uagv_system/State.h>

/**
 * @class Mode and five subclasses
 * @author I-chun Han
 * @date 12/22/17
 * @file MachineBase.h
 * @brief The main states and modes control and provide functions that UAGV moving base
 */

class Machine;
class State;
class Mode
{
friend class Machine;

private:
    Machine * _context;
protected:
    Machine * context() const {return _context;};
    void context(Machine * newMachine) {_context = newMachine;};
public:
    Mode(Machine * machine):_context(machine){};
    Mode(const Mode * source):_context(source->context()){};
    ~Mode(){}
public:
    static Mode * InitialMode(Machine * machine);
    void setMode();
    virtual void navigation();
    virtual void registration();
    virtual void remote();
    virtual void liftUp();   
    virtual void liftDown();
    virtual void transition();
    virtual void autoCharging();

};



class AutoMode : public Mode
{
friend class Machine;

public:
    AutoMode(Machine * machine):Mode(machine){};
    AutoMode(const Mode * source):Mode(source){};
    ~AutoMode();
public:
    virtual void navigation();
    virtual void registration();
    virtual void remote();
    virtual void liftUp();   
    virtual void liftDown();
    virtual void transition();
    virtual void autoCharging();
};



class ManualMode : public Mode
{
friend class Machine;
public:
    ManualMode(Machine * machine):Mode(machine){};
    ManualMode(const Mode * source):Mode(source){};
    ~ManualMode();
public:
    void navigation();
    void registration();
    void remote();
    void liftUp();
    void liftDown();
    void transition();
    void autoCharging();
};



class DebugMode : public Mode
{
friend class Machine;
public:
    DebugMode(Machine * machine):Mode(machine){};
    DebugMode(const Mode * source):Mode(source){};
    ~DebugMode();
public:    
    void navigation();
    void registration();
    void remote();
    void liftUp() ; 
    void liftDown();
    void transition();
    void autoCharging();

};




class ErrorMode : public Mode
{
friend class Machine;
public:
    ErrorMode(Machine * machine):Mode(machine){};
    ErrorMode(const Mode * source):Mode(source){};
    ~ErrorMode();
public:
    void navigation();
    void registration();
    void remote();
    void liftUp();
    void liftDown();
    void transition();
    void autoCharging();

};





class WarningMode : public Mode
{
friend class Machine;
public:
    WarningMode(Machine * machine):Mode(machine){};
    WarningMode(const Mode * source):Mode(source){};
    ~WarningMode();
public:  
    void navigation();
    void registration();
    void remote();
    void liftUp(); 
    void liftDown();
    void transition();
    void autoCharging();

};


#endif
