#include "robot.h"

bool Robot::init() {
  MECOtron::init(); 
  return true;
}

void Robot::control() {
    static int tick = 0; 
    
    // Aparte geheugenwaarden (integratoren) voor motor A en B
    static float integral_error_A = 0.0;
    static float integral_error_B = 0.0;
    
    // PI-parameters Motor A (uit MATLAB)
    const float Kp_A = 0.8581;  // Kp voor Motor A
    const float Ti_A = 0.0659;  // Ti voor Motor A

    // PI-parameters Motor B (uit MATLAB)
    const float Kp_B = 0.9090;  
    const float Ti_B = 0.0682;  

    const float Ts = 0.01;         // 100Hz = 10ms sampletijd
    const float max_voltage = 12.0;

    float u_k_A = 0.0;
    float u_k_B = 0.0;
    float ref_velocity = 0.0;

    if(controlEnabled()) {
        LED1(ON);
        LED2(OFF);

        // 1. Referentieprofiel: Eerst 0.5s stilstand, daarna stap naar 5 rad/s
        if (tick < 50) {
            ref_velocity = 0.0; // Eerste 0.5s (50 ticks)
        } else if (tick < 750) {
            ref_velocity = 7.0; // Daarna 7s lang naar 5 rad/s
        } else {
            ref_velocity = 0.0; // Stop na 5 seconden
        }

        // 2. Metingen inlezen per motor
        float speed_A = getSpeedMotorA();
        float speed_B = getSpeedMotorB();

        // --- 3. PI-REGELAAR MOTOR A ---
        float error_A = ref_velocity - speed_A;
        integral_error_A += error_A * Ts;
        u_k_A = Kp_A * error_A + (Kp_A / Ti_A) * integral_error_A;

        // Anti-windup Motor A
        if (u_k_A > max_voltage) {
            u_k_A = max_voltage;
            integral_error_A -= error_A * Ts;
        } else if (u_k_A < -max_voltage) {
            u_k_A = -max_voltage;
            integral_error_A -= error_A * Ts;
        }

        // --- 4. PI-REGELAAR MOTOR B ---
        float error_B = ref_velocity - speed_B;
        integral_error_B += error_B * Ts;
        u_k_B = Kp_B * error_B + (Kp_B / Ti_B) * integral_error_B;

        // Anti-windup Motor B
        if (u_k_B > max_voltage) {
            u_k_B = max_voltage;
            integral_error_B -= error_B * Ts;
        } else if (u_k_B < -max_voltage) {
            u_k_B = -max_voltage;
            integral_error_B -= error_B * Ts;
        }

        // 5. Motoren onafhankelijk aansturen
        setVoltageMotorA(u_k_A);
        setVoltageMotorB(u_k_B);

        tick++;

        // 6. Data sturen naar QRoboticsCenter
        writeValue(0, ref_velocity); // Kanaal 0: Referentie
        writeValue(1, speed_A);      // Kanaal 1: Snelheid Motor A
        writeValue(2, speed_B);      // Kanaal 2: Snelheid Motor B
        writeValue(3, u_k_A);        // Kanaal 3: Motorspanning Motor A
        writeValue(4, u_k_B);        // Kanaal 4: Motorspanning Motor B

    } else {
        LED1(OFF);
        LED2(ON);
        setVoltageMotorA(0.0); 
        setVoltageMotorB(0.0); 
        
        // Reset tijdteller en integratoren
        tick = 0; 
        integral_error_A = 0.0;
        integral_error_B = 0.0;
        
        writeValue(0, 0.0);
        writeValue(1, 0.0);
        writeValue(2, 0.0);
        writeValue(3, 0.0);
        writeValue(4, 0.0);
    }
}

bool Robot::controlEnabled() {
  return _button_states[0];       
}

void Robot::button0callback() {
  if(toggleButton(0)) {           
    message("PI Experiment (2a/2b) started.");    
  } else {
    message("Experiment disabled.");
  }
}

void Robot::button1callback() {
  toggleButton(1);
  init();                         
  message("Reset.");
}