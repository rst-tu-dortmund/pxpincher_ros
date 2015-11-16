#ifndef SIMULATION_H
#define SIMULATION_H

#include <vector>
#include "sensor_msgs/JointState.h"

#include "ros/ros.h"

#include "boost/thread/mutex.hpp"

class Simulation
{
public:
    Simulation();
    Simulation(std::vector<double> offsets);

    sensor_msgs::JointState performSimulationStep(double duration);
    void setQDot(std::vector<double> qDot);
    sensor_msgs::JointState getCurrentState();

private:

    double q1Old, q2Old, q3Old, q4Old, q5Old;
    std::vector<double> offsets_, qDots_;
    sensor_msgs::JointState currentState_;
};

#endif // SIMULATION_H
