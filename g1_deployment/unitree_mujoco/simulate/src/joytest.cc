#include <iostream>
#include "joystick/joystick.h"
#include <unistd.h>

int main() {
    Joystick js("/dev/input/js0");  // Adjust device path if needed
    
    if(!js.isFound()) {
        std::cout << "Joystick not found!" << std::endl;
        return 1;
    }
    
    std::cout << "Press buttons and move sticks to see their indices..." << std::endl;
    std::cout << "Press Ctrl+C to exit" << std::endl << std::endl;
    
    while(true) {
        js.getState();
        
        // Print all button states
        int button_size = sizeof(js.button_) / sizeof(js.button_[0]);
        int axis_size = sizeof(js.axis_) / sizeof(js.axis_[0]);
        std::cout << "\rButtons: ";
        for(size_t i = 0; i < button_size; i++) {
            if(js.button_[i]) {
                std::cout << "btn[" << i << "]=1 ";
            }
        }
        
        // Print all axis values
        std::cout << "| Axes: ";
        for(size_t i = 0; i < axis_size; i++) {
            if(abs(js.axis_[i]) > 1000) {  // Threshold to filter noise
                std::cout << "axis[" << i << "]=" << js.axis_[i] << " ";
            }
        }
        
        std::cout << "          " << std::flush;
        usleep(50000);  // 50ms delay
    }
    
    return 0;
}