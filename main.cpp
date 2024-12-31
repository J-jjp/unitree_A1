#include "leg_control.h"


int main(int argc, char** argv){
    std::shared_ptr<leg_control>  FL_leg = std::make_shared<leg_control>("FL", "/dev/ttyUSB0");



    while (1)
    {
        /* code */
    }
    
}