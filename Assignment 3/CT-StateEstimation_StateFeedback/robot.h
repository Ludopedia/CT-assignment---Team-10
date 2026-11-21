#ifndef ROBOT_H
#define ROBOT_H

/*
 * ROBOT Class
 *
 * Class incorporating the robot. This class is used to define state machines,
 * control algorithms, sensor readings,...
 * It should be interfaced with the communicator to send data to the world.
 *
 */

#include "mecotron.h" // Include MECOTRON header
#include <BasicLinearAlgebra.h> // Include BasicLinearAlgebra to make matrix manipulations easier
#include "state_estimator.h" // Include template to make state estimation implementation easier

class Robot : public MECOtron {
  private:

    // Class variables

    // Velocity controller
    // ...

    // State estimation
    Matrix<1> _xhat;      // state estimate vector
    Matrix<1> _nu;        // innovation vector

    // Position controller
    Matrix<1> xref;       // reference state
    Matrix<1,1> K;        // state feedback gain
    Matrix<1> desired_velocity; //control signal

  public:
    // Constructor
    Robot() { }

    void control();

    // General functions
    bool init();  // Set up the robot

    bool controlEnabled();
    bool StateEstimationEnabled();

    void resetController();
    void resetStateEstimator();

    void button0callback();
    void button1callback();
    void button2callback();

};

#endif // ROBOT_H
