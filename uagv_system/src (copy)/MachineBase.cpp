#include <uagv_system/MachineBase.h>
#include <iostream>

using namespace std;

MachineBase::MachineBase(){};
MachineBase::~MachineBase(){};
	


// Call ROS services
void MachineBase::executeOnMainBoard(){
        cout<< "on mainBouard\n";
}
void MachineBase::executeOnControlBoard(){
        cout<< "on controlBouard\n";
}
    
	/**
	 * @brief basic functions in machine base
	 */
void MachineBase::executeNavigation(){
        //setState(navigationState);
        cout<< "execute Navigation\n";
        //cout<< getState()<<"\n";
        executeOnMainBoard();
    }
void MachineBase::executeRemote(){
        //setState(remoteState);
        cout<< "execute Remote\n";
        //cout<< getState()<<"\n";
        executeOnMainBoard();
    }
void MachineBase::executeRegistration(){
        //setState(registrationState);
        cout<< "execute Registration\n"	;
        //cout<< getState()<<"\n";
        executeOnMainBoard();
    }
void MachineBase::executeAutoCharging(){
        cout<< "execute AutoCharging\n"	;
        //mode.navigation();
        //mode.registration();
        cout<< "....................\n"	;
    }
       
void MachineBase::stopNavigation(){cout<< "stop Navigation\n";}
void MachineBase::stopRemeote(){cout<< "stop Remeote\n";} // removed in use case??
void MachineBase::stopRegistration(){cout<< "stop Registration\n";}


void MachineBase::pauseNavigation(){cout<< "pause Navigation\n";}
void MachineBase::pauseRemeote(){cout<< "pause Remeote\n";}
void MachineBase::pauseRegistration(){cout<< "pause Registration\n";}
	




