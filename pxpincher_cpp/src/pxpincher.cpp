#include "pxpincher_cpp/pxpincher.h"

PxPincher::PxPincher():
    comm_(paramObject_.port_,paramObject_.baud_),
    sim_(paramObject_.simulation_),
    rate_(paramObject_.rate_),
    controllerManager_(this),
    last_(ros::Time::now()),
    nHandle_("PXPincher")
{
    std::size_t no_joints = paramObject_.names_.size();
    
    joint_info_.resize(no_joints);
    
    initRobot();
    
    
    // create ros_control handles
    for (int i = 0; i < no_joints; ++i)
    {
         hardware_interface::JointStateHandle state_handle_joint( paramObject_.names_[i] , &joint_info_[i].pos, &joint_info_[i].vel, &joint_info_[i].eff);
         jnt_state_interface_.registerHandle(state_handle_joint);
         
         hardware_interface::JointHandle pos_handle_joint(jnt_state_interface_.getHandle(paramObject_.names_[i]),  &joint_info_[i].cmd); // TODO: maybe use handle from above
         jnt_position_interface_.registerHandle(pos_handle_joint);
    }
    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_position_interface_);

    statePublisher_ = nHandle_.advertise<sensor_msgs::JointState>("/joint_states",1); // joint_states topic is always root
    diagnosticPublisher_ = nHandle_.advertise<pxpincher_msgs::pxpincher_diagnostic>("Diagnostics",1);
    simSubscriber_ = nHandle_.subscribe("JointCMDSimulation",1, &PxPincher::simulationCallback,this);

}

PxPincher::~PxPincher()
{
    // TODO Shut Down Robot by using emergency shut down function
    // Stop all movement
    // Hold torque for n seconds
    // Turn off torque

    comm_.close();
}

void PxPincher::start()
{
    ros::Rate rate(rate_);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    while(ros::ok()){
        update();
        ros::spinOnce();
        rate.sleep();
    }
}

void PxPincher::update()
{
    if(!sim_){
        // Get current servo status
        std::vector<ServoStatus> stati;
        for (int id : paramObject_.ids_)
        {
            stati.emplace_back(id);
        }
        protocol_.readServoStatus(stati,comm_);

        // Fill control variables
        fillControlRegister(stati);

        // Do control step
        calculateControlStep();

        // Write control command
        performAction();

        // Publish current information
        statePublisher_.publish(getJointState());
        diagnosticPublisher_.publish(getDiagnostics(stati));
    }else{
        // Publish current information
        statePublisher_.publish(simObject_.performSimulationStep(1/rate_));
    }
}

void PxPincher::fillControlRegister(const std::vector<ServoStatus>& stati)
{
    if (stati.size() != joint_info_.size())
    {
        ROS_ERROR("Cannot fill controll register. Number of stati obtained from the hardware communication node does not match number of joints.");
        return;
    }
    
    int idx = 0;
    for (const ServoStatus& status : stati)
    {
        joint_info_[idx].pos = tick2rad( status.position_ + paramObject_.offsets_[idx]);
        joint_info_[idx].vel = tick2rads( status.speed_ );
        ++idx;
    }
}

void PxPincher::calculateControlStep()
{
    ros::Time now = ros::Time::now();
    ros::Duration period = last_ - now;
    controllerManager_.update(now,period,false);

    last_ = now;
}

void PxPincher::performAction()
{
    //protocol_.setGoalPosition(ids,positions,comm_);
    for(const JointInfo& joint : joint_info_){
        ROS_INFO("Value: %f",joint.cmd);
    }
}


double PxPincher::tick2rad(int position)
{
    return conversionFactor1*position;
}


std::vector<double> PxPincher::tick2rad(const std::vector<int>& positions)
{
    std::vector<double> rads;
    rads.reserve(positions.size());
    
    for(int elem : positions){
        rads.push_back(tick2rad(elem));
    }
    return rads;
}

double PxPincher::tick2rads(int speed)
{
    return conversionFactor2*speed;
}

std::vector<double> PxPincher::tick2rads(const std::vector<int>& speeds)
{
    std::vector<double> rads;
    rads.reserve(speeds.size());

    for(int elem : speeds){
        rads.push_back(tick2rads(elem));
    }
    return rads;
}


double PxPincher::convVoltage(int volt)
{
    return double(volt)/10.0;
}

std::vector<double> PxPincher::convVoltage(const std::vector<int>& volt)
{
    std::vector<double> newVolt;
    newVolt.reserve(volt.size());

    for(int elem : volt){
        newVolt.push_back(convVoltage(elem));
    }
    return newVolt;
}


sensor_msgs::JointState PxPincher::getJointState()
{
    sensor_msgs::JointState jointStates;

    jointStates.header.stamp = ros::Time::now();
          
    int idx = 0;
    for (const JointInfo& joint : joint_info_)
    {
        jointStates.name.push_back( paramObject_.names_[idx] );
        jointStates.position.push_back( joint.pos );
        jointStates.velocity.push_back( joint.vel );
        ++idx;
    }
    
    return jointStates;
}


pxpincher_msgs::pxpincher_diagnostic PxPincher::getDiagnostics(const std::vector<ServoStatus>& stati)
{
    pxpincher_msgs::pxpincher_diagnostic diag;
    
    diag.header.stamp = ros::Time::now();

    int idx = 0;
    for (const ServoStatus& status : stati)
    {
        diag.name.push_back( paramObject_.names_[idx] );
        diag.temperature.push_back( (double) status.temperature_ );
        diag.voltage.push_back( convVoltage( status.voltage_ ) );
        ++idx;
    }

    return diag;
}

void PxPincher::simulationCallback(const sensor_msgs::JointStateConstPtr &state)
{
    simObject_.setQDot(state->velocity);
}


void PxPincher::initRobot(){

    std::vector<UBYTE> ids = paramObject_.ids_;
    std::vector<int> cwLimits = paramObject_.cwlimits_;
    std::vector<int> ccwLimits = paramObject_.ccwlimits_;
    std::vector<int> speeds = paramObject_.speeds_;

    protocol_.setCCWAngleLimit(ids,ccwLimits,comm_);
    ros::Duration(0.01).sleep();

    protocol_.setCWAngleLimit(ids,cwLimits,comm_);
    ros::Duration(0.01).sleep();

    protocol_.setGoalSpeed(ids,speeds,comm_);
    ros::Duration(0.01).sleep();

    // Drive to Home-Position
    std::vector<int> values = {511, 511, 511, 511, 215};
    protocol_.setGoalPosition(ids,values,comm_);
    ros::Duration(0.01).sleep();
}

