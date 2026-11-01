#include "robot.h"

bool Robot::init() {
  MECOtron::init(); // Initialize the MECOtron

  // Initializing the robot's specific variables

  return true;
}

void Robot::control() {
    // Static counter to keep track of time (persists across 100Hz function calls)
    static int tick = 0; 
    float u_k = 0.0;

    if(controlEnabled()) {
        LED1(ON);
        LED2(OFF);

        // 0.5 Hz period = 2 seconds = 200 ticks
        // 5 repetitions = 10 seconds = 1000 ticks
        if (tick < 1000) {
            // For the first half of the period (100 ticks), apply 5V. Else 0V.
            if ((tick % 200) < 100) {
                u_k = 5.0;
            } else {
                u_k = 0;
            }
            tick++; // Advance timer
        } else {
            // Stop the cart after 5 cycles are complete
            u_k = 0.0;
        }

        // Apply exactly the same voltage to both motors to prevent turning
        setVoltageMotorA(u_k);
        setVoltageMotorB(u_k);

    } else {
        LED1(OFF);
        LED2(ON);
        setVoltageMotorA(0.0); 
        setVoltageMotorB(0.0); 
        tick = 0; // Reset timer so you can rerun the experiment by toggling Button 0
    }

    // Read wheel speeds for identification
    float speed_A = getSpeedMotorA();
    float speed_B = getSpeedMotorB();

    // Send data to QRoboticsCenter channels so the GPIO recorder can capture them
    writeValue(0, u_k);      // Channel 0: Input Voltage
    writeValue(1, speed_A);  // Channel 1: Motor A Speed
    writeValue(2, speed_B);  // Channel 2: Motor B Speed
}

bool Robot::controlEnabled() {
  return _button_states[0];       // The control is enabled if the state of button 0 is true
}

void Robot::button0callback() {
  if(toggleButton(0)) {           // Switches the state of button 0 and checks if the new state is true
    message("Robot enabled.");    // Display a message in the status bar of QRoboticsCenter
  }
  else {
    message("Robot disabled.");
  }
}

void Robot::button1callback() {
  toggleButton(1);
  init();                         // Reset the MECOtron and reinitialize the Robot object
  message("Reset.");
}
