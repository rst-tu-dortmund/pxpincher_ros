#include "pxpincher_cpp/simulation.h"

Simulation::Simulation()
{
    offsets_ = {0.0,0.0,0.0,0.0,0.0};
    qDots_ = {0.0,0.0,0.0,0.0,0.0};

    currentState_.header.stamp = ros::Time::now();
    currentState_.velocity = qDots_;
    currentState_.position = offsets_;
}

Simulation::Simulation(std::vector<double> offsets):
    offsets_(offsets)
{
    qDots_ = {0.0,0.0,0.0,0.0,0.0};

    currentState_.header.stamp = ros::Time::now();
    currentState_.velocity = qDots_;
    currentState_.position = offsets_;
}

void Simulation::setQDot(std::vector<double> qDot)
{
    qDots_ = qDot;
}

sensor_msgs::JointState Simulation::performSimulationStep(double duration)
{
    std::vector<std::string> names = {"J1","J2","J3","J4","J5"};

    for(int i = 0; i < qDots_.size(); ++i){
        offsets_[i] += qDots_[i]*duration;
    }


    currentState_.header.stamp = ros::Time::now();
    currentState_.name = names;
    currentState_.position = offsets_;
    currentState_.velocity = qDots_;

    return currentState_;
}

sensor_msgs::JointState Simulation::getCurrentState()
{
    return currentState_;
}
