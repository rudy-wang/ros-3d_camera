#include <uagv_system/CargoMachine.h>
#include <iostream>

using namespace std;

CargoMachine::CargoMachine(){}
CargoMachine::~CargoMachine(){}
	
void CargoMachine::executeLiftUp(){
        //setState(liftupState);
        cout<< "execute liftup\n";
        //cout<< getState()<<"\n";
        executeOnControlBoard();
    }
void CargoMachine::executeLiftDown(){
        //setState(liftDownState);
        cout<< "execute liftdown\n";
        //cout<< getState()<<"\n";
        executeOnControlBoard();
    }   
    
void CargoMachine::executeTransition(){
        cout<< "execute liftdown\n";
        //mode.navigation();
        //mode.registration();
        //mode.liftup();
        //mode.navigation();
        //mode.liftDown();
        cout<< "....................\n"	;
    }
void CargoMachine::stopLiftUp(){
        cout<< "stop liftup\n";
        executeOnControlBoard();
    }
void CargoMachine::stopLiftDown(){
        cout<< "stop liftdown\n";
        executeOnControlBoard();
    }
    
void CargoMachine::pauseLiftUp(){
        cout<< "pause liftup\n";
        executeOnControlBoard();
    }
void CargoMachine::pauseLiftDown(){
        cout<< "pause liftdown\n";
        executeOnControlBoard();
    }



