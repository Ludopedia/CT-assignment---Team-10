#ifndef ROBOT_H
#define ROBOT_H

#include "mecotron.h" 

class Robot : public MECOtron {
  public:
    Robot() { }

    void control();
    bool init();  
    bool controlEnabled();

    void button0callback();
    void button1callback();
};

#endif // ROBOT_2A_H
