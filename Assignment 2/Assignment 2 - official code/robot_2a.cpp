#include "robot_2a.h"

bool Robot::init() {
  MECOtron::init(); 
  return true;
}

void Robot::control() {
    static int tick = 0; 
    static float integral_error = 0.0;
    
    // Jouw PI-parameters uit MATLAB
    const float Kp = 0.8133;
    const float Ti = 0.1121;
    const float Ts = 0.01;         // 100Hz = 10ms sampletijd
    const float max_voltage = 12.0;

    float u_k = 0.0;
    float ref_velocity = 0.0;

    if(controlEnabled()) {
        LED1(ON);
        LED2(OFF);

        // 1. Referentieprofiel: Eerst 0.5s stil, dan constante stap
        if (tick < 50) {
            ref_velocity = 0.0; // Eerste 0.5s (50 ticks)
        } else if (tick < 500) {
            ref_velocity = 5.0; // Daarna 4.5s lang naar 5 rad/s
        } else {
            ref_velocity = 0.0; // Stop na 5 seconden
        }

        // 2. Meting inlezen
        float speed_A = getSpeedMotorA();
        float speed_B = getSpeedMotorB();
        float measured_velocity = (speed_A + speed_B) / 2.0;

        // 3. PI-Algoritme
        float error = ref_velocity - measured_velocity;
        integral_error += error * Ts; // Integreren

        u_k = Kp * error + (Kp / Ti) * integral_error;

        // 4. Anti-Windup en Voltage limiet
        if (u_k > max_voltage) {
            u_k = max_voltage;
            integral_error -= error * Ts; // Stop integratie
        } else if (u_k < -max_voltage) {
            u_k = -max_voltage;
            integral_error -= error * Ts; // Stop integratie
        }

        // 5. Motoren aansturen
        setVoltageMotorA(u_k);
        setVoltageMotorB(u_k);

        tick++;

        // 6. Data sturen naar QRoboticsCenter voor de plots in je verslag
        writeValue(0, ref_velocity);      // Kanaal 0: De stap referentie
        writeValue(1, measured_velocity); // Kanaal 1: De werkelijke snelheid
        writeValue(2, u_k);               // Kanaal 2: De motorspanning (control signal)

    } else {
        LED1(OFF);
        LED2(ON);
        setVoltageMotorA(0.0); 
        setVoltageMotorB(0.0); 
        
        tick = 0; 
        integral_error = 0.0;
        
        writeValue(0, 0.0);
        writeValue(1, 0.0);
        writeValue(2, 0.0);
    }
}

bool Robot::controlEnabled() {
  return _button_states[0];       
}

void Robot::button0callback() {
  if(toggleButton(0)) {           
    message("PI Experiment (2a) started.");    
  } else {
    message("Experiment disabled.");
  }
}

void Robot::button1callback() {
  toggleButton(1);
  init();                         
  message("Reset.");
}