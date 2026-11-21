/*
 * STATE ESTIMATION AND STATE FEEDBACK CONTROL TEMPLATE
 *
 * This is a template to get you started with the implementation of the state estimation and state feedback control
 * on your own cart.
 *
 */

#include "robot.h"

bool Robot::init() {
  MECOtron::init(); // Initialize the MECOtron

  desired_velocity(0) = 0;
  return true;
}

void Robot::control() {

  float volt_A = 0.0;
  float volt_B = 0.0;
  Matrix<1> desired_velocity; //control signal
  desired_velocity.Fill(0); //Initialize matrix with zeros

  // State estimation
  if(StateEstimationEnabled()) {   // only do this if controller is enabled (triggered by pushing 'Button 1' in QRoboticsCenter)
    // Correction step
    Matrix<1> distance_measurement;                                     // define a vector of length 1
    distance_measurement(0) = getFrontDistance();                       // front distance
    CorrectionUpdate(distance_measurement, _xhat, _nu);     // do the correction step -> update _xhat, _nu
  }
  writeValue(8, _xhat(0)); // a posteriori state estimate
  writeValue(9, _nu(0)); // innovation

  if(controlEnabled()) {   // only do this if controller is enabled (triggered by pushing 'Button 0' in QRoboticsCenter)

    // UNCOMMENT AND COMPLETE LINES BELOW TO IMPLEMENT POSITION CONTROLLER
    // float desired_position = readValue(0);      // use channel 0 to provide the constant position reference
    // xref(0) = ? ;                               // transform desired_position to the state reference (make sure units are consistent)
    // K(0) = ? ;                                  // state feedback gain K, to design
    // desired_velocity = K * (xref - _xhat);      // calculate the state feedback signal, (i.e. the input for the velocity controller)

    // // UNCOMMENT AND COMPLETE LINES BELOW TO IMPLEMENT VELOCITY CONTROLLER
    // ...
    // ...                                          // implement your velocity controller here (assignment 2), such that the motors track velocity v
    // ...
    // volt_A =
    // volt_B =

    //// COMMENT OR REMOVE LINES BELOW ONCE YOU IMPLEMENT THE VELOCITY CONTROLLER
    volt_A = 0.0;
    volt_B = 0.0;

    // Send wheel speed command
    setVoltageMotorA(volt_A);
    setVoltageMotorB(volt_B);
  }
  else                      // do nothing since control is disabled
  {
    desired_velocity(0) = 0.0;
    setVoltageMotorA(0.0);
    setVoltageMotorB(0.0);
  }

  // Sate estimation
  if(StateEstimationEnabled()) {   // only do this if controller is enabled (triggered by pushing 'Button 1' in QRoboticsCenter)
    // Prediction step
    PredictionUpdate(desired_velocity, _xhat);                    // do the prediction step -> update _xhat
  }
  // writeValue(8, _xhat(0)); // a priori state estimate
  
  // Send useful outputs to QRC
  writeValue(0, volt_A);
  writeValue(1, volt_B);
  writeValue(2, desired_velocity(0));
  writeValue(3, getPositionMotorA());
  writeValue(4, getPositionMotorB());
  writeValue(5, getSpeedMotorA());
  writeValue(6, getSpeedMotorB());
  writeValue(7, getFrontDistance());


}

void Robot::resetController(){
  // Set all errors and control signals in the memory back to 0
  // ...
  // ...
}

void Robot::resetStateEstimator() {
  // // UNCOMMENT AND MODIFIES LINES BELOW TO IMPLEMENT THE RESET OF THE STATE ESTIMATOR
  // // Initialize state estimate
  // _xhat(0) = 0.0;     // Change this according to your experiments
}

bool Robot::controlEnabled() {
  return _button_states[0];       // The control is enabled if the state of button 0 is true
}

bool Robot::StateEstimationEnabled() {
  return _button_states[1];
}

void Robot::button0callback() {
  if(toggleButton(0)) {           // Switches the state of button 0 and checks if the new state is true
    resetController();
    message("Controller reset and enabled.");    // Display a message in the status bar of QRoboticsCenter
  }
  else {
    message("Control disabled.");
  }
  // button1callback();           // If you want to toggle the estimator together with the controller
}

void Robot::button1callback() {
  if(toggleButton(1)){
      resetStateEstimator();            // Reset the state estimator
      message("State estimator reset and enabled.");
  }
  else
  {
    message("State estimator disabled.");
  }
}

void Robot::button2callback() {
    init();                         // Reset the MECOtron and reinitialize the Robot object
    resetController();              // Reset the Controller
    _button_states[0] = 0;
    resetStateEstimator();            // Reset the state estimator
    _button_states[1] = 0;
    message("MECOtron reinitialized.");
}
