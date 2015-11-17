#include "pxpincher_cpp/pxpincher.h"

PxPincher::PxPincher():
    comm_(params_.port_,params_.baud_),
    sim_(params_.simulation_),
    rate_(params_.rate_),
    controller_manager_(this),
    last_(ros::Time::now()),
    nhandle_("PXPincher")
{
    std::size_t no_joints = params_.names_.size();
    
    joint_data_.resize(no_joints);
    
    initRobot();
    
    
    // create ros_control handles
    for (int i = 0; i < no_joints; ++i)
    {
         hardware_interface::JointStateHandle state_handle_joint( params_.names_[i] , &joint_data_[i].pos, &joint_data_[i].vel, &joint_data_[i].eff);
         jnt_state_interface_.registerHandle(state_handle_joint);
         
         hardware_interface::JointHandle pos_handle_joint(jnt_state_interface_.getHandle(params_.names_[i]),  &joint_data_[i].cmd); // TODO: maybe use handle from above
         jnt_position_interface_.registerHandle(pos_handle_joint);
    }
    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_position_interface_);

    state_publisher_ = nhandle_.advertise<sensor_msgs::JointState>("/joint_states",1); // joint_states topic is always root
    diagnostic_publisher_ = nhandle_.advertise<pxpincher_msgs::pxpincher_diagnostic>("Diagnostics",1);
    sim_subscriber_ = nhandle_.subscribe("JointCMDSimulation",1, &PxPincher::simulationCallback,this);

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
        for (int id : params_.ids_)
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
        state_publisher_.publish(getJointState());
        diagnostic_publisher_.publish(getDiagnostics(stati));
    }else{
        // Publish current information
        state_publisher_.publish(sim_object_.performSimulationStep(1/rate_));
    }
}

void PxPincher::fillControlRegister(const std::vector<ServoStatus>& stati)
{
    if (stati.size() != joint_data_.size())
    {
        ROS_ERROR("Cannot fill controll register. Number of stati obtained from the hardware communication node does not match number of joints.");
        return;
    }
    
    int idx = 0;
    for (const ServoStatus& status : stati)
    {
        joint_data_[idx].pos = tick2rad( status.position_ + params_.offsets_[idx]);
        joint_data_[idx].vel = tick2rads( status.speed_ );
        ++idx;
    }
}

void PxPincher::calculateControlStep()
{
    ros::Time now = ros::Time::now();
    ros::Duration period = last_ - now;
    controller_manager_.update(now,period,false);

    last_ = now;
}

void PxPincher::performAction()
{
    //protocol_.setGoalPosition(ids,positions,comm_);
    for(const JointData& joint : joint_data_){
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
    std::vector<double> new_volt;
    new_volt.reserve(volt.size());

    for(int elem : volt){
        new_volt.push_back(convVoltage(elem));
    }
    return new_volt;
}


sensor_msgs::JointState PxPincher::getJointState()
{
    sensor_msgs::JointState jointStates;

    jointStates.header.stamp = ros::Time::now();
          
    int idx = 0;
    for (const JointData& joint : joint_data_)
    {
        jointStates.name.push_back( params_.names_[idx] );
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
        diag.name.push_back( params_.names_[idx] );
        diag.temperature.push_back( (double) status.temperature_ );
        diag.voltage.push_back( convVoltage( status.voltage_ ) );
        ++idx;
    }

    return diag;
}

void PxPincher::simulationCallback(const sensor_msgs::JointStateConstPtr &state)
{
    sim_object_.setQDot(state->velocity);
}


void PxPincher::initRobot(){

    std::vector<UBYTE> ids = params_.ids_;
    std::vector<int> cw_limits = params_.cwlimits_;
    std::vector<int> ccw_limits = params_.ccwlimits_;
    std::vector<int> speeds = params_.speeds_;

    protocol_.setCCWAngleLimit(ids,ccw_limits,comm_);
    ros::Duration(0.01).sleep();

    protocol_.setCWAngleLimit(ids,cw_limits,comm_);
    ros::Duration(0.01).sleep();

    protocol_.setGoalSpeed(ids,speeds,comm_);
    ros::Duration(0.01).sleep();

    // Drive to Home-Position
    std::vector<int> values = {511, 511, 511, 511, 215};
    protocol_.setGoalPosition(ids,values,comm_);
    ros::Duration(0.01).sleep();
}

