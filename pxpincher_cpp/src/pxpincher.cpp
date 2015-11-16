#include "pxpincher_cpp/pxpincher.h"

PxPincher::PxPincher():
    comm_(paramObject_.port_,paramObject_.baud_),
    sim_(paramObject_.simulation_),
    rate_(paramObject_.rate_),
    controllerManager_(this),
    last_(ros::Time::now()),
    nHandle_("PXPincher")
{
    initRobot();

    hardware_interface::JointStateHandle stateHandleJoint1("J1",&pos[0], &vel[0],&eff[0]);
    hardware_interface::JointStateHandle stateHandleJoint2("J2",&pos[1], &vel[1],&eff[1]);
    hardware_interface::JointStateHandle stateHandleJoint3("J3",&pos[2], &vel[2],&eff[2]);
    hardware_interface::JointStateHandle stateHandleJoint4("J4",&pos[3], &vel[3],&eff[3]);
    hardware_interface::JointStateHandle stateHandleJoint5("J5",&pos[4], &vel[4],&eff[4]);


    jntStateInterface.registerHandle(stateHandleJoint1);
    jntStateInterface.registerHandle(stateHandleJoint2);
    jntStateInterface.registerHandle(stateHandleJoint3);
    jntStateInterface.registerHandle(stateHandleJoint4);
    jntStateInterface.registerHandle(stateHandleJoint5);

    registerInterface(&jntStateInterface);

    hardware_interface::JointHandle posHandleJoint1(jntStateInterface.getHandle("J1"),&cmd[0]);
    hardware_interface::JointHandle posHandleJoint2(jntStateInterface.getHandle("J2"),&cmd[0]);
    hardware_interface::JointHandle posHandleJoint3(jntStateInterface.getHandle("J3"),&cmd[0]);
    hardware_interface::JointHandle posHandleJoint4(jntStateInterface.getHandle("J4"),&cmd[0]);
    hardware_interface::JointHandle posHandleJoint5(jntStateInterface.getHandle("J5"),&cmd[0]);

    jntPositionInterface.registerHandle(posHandleJoint1);
    jntPositionInterface.registerHandle(posHandleJoint2);
    jntPositionInterface.registerHandle(posHandleJoint3);
    jntPositionInterface.registerHandle(posHandleJoint4);
    jntPositionInterface.registerHandle(posHandleJoint5);

    registerInterface(&jntPositionInterface);

    statePublisher_ = nHandle_.advertise<sensor_msgs::JointState>("JointStates",1);
    diagnosticPublisher_ = nHandle_.advertise<pxpincher_rst_msgs::pxpincher_rst_diagnostic>("Diagnostics",1);
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
        std::vector<ServoStatus> stati = {ServoStatus(1), ServoStatus(2), ServoStatus(3), ServoStatus(4), ServoStatus(5)};
        protocol_.readServoStatus(stati,comm_);

        // Fill control variables
        fillControlRegister(stati);

        // Do control step
        calculateControlStep();

        // Write control command
        performAction();

        // Publish current information
        statePublisher_.publish(getJointState(stati));
        diagnosticPublisher_.publish(getDiagnostics(stati));
    }else{
        // Publish current information
        statePublisher_.publish(simObject_.performSimulationStep(1/rate_));
    }
}

void PxPincher::fillControlRegister(std::vector<ServoStatus> stati)
{
    std::vector<int> positions = {stati[0].position_, stati[1].position_, stati[2].position_, stati[3].position_, stati[4].position_};
    std::vector<int> speeds = {stati[0].speed_, stati[1].speed_, stati[2].speed_, stati[3].speed_, stati[4].speed_};

    std::vector<double> positionDbl = tick2rad(positions);
    std::vector<double> speedDbl = tick2rads(speeds);

    for(int i = 0; i < positionDbl.size(); ++i){
        pos[i] = positionDbl[i];
        vel[i] = speedDbl[i];
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
    std::vector<int> positions = {(int) floor(cmd[1]),(int) floor(cmd[2]),(int) floor(cmd[3]),(int) floor(cmd[4]),(int) floor(cmd[5])};
    std::vector<UBYTE> ids = paramObject_.ids_;

    //protocol_.setGoalPosition(ids,positions,comm_);
    for(int temp : positions){
        ROS_INFO("Value: %d",temp);
    }
}


std::vector<double> PxPincher::tick2rad(std::vector<int> positions)
{
    std::vector<double> rads;
    rads.reserve(positions.size());

    for(int elem : positions){
        rads.push_back(conversionFactor1*elem);
    }
    return rads;
}


std::vector<double> PxPincher::tick2rads(std::vector<int> speeds)
{
    std::vector<double> rads;
    rads.reserve(speeds.size());

    for(int elem : speeds){
        rads.push_back(conversionFactor2*elem);
    }
    return rads;
}


std::vector<double> PxPincher::convVoltage(std::vector<int> volt)
{
    std::vector<double> newVolt;
    newVolt.reserve(volt.size());

    for(int elem : volt){
        newVolt.push_back(elem/10);
    }
    return newVolt;
}


sensor_msgs::JointState PxPincher::getJointState(std::vector<ServoStatus> stati)
{
    sensor_msgs::JointState jointStates;
    std::vector<std::string> names = {"J1","J2","J3","J4","J5"};

    std::vector<int> positions = {stati[0].position_, stati[1].position_, stati[2].position_, stati[3].position_, stati[4].position_};
    std::vector<int> speeds = {stati[0].speed_, stati[1].speed_, stati[2].speed_, stati[3].speed_, stati[4].speed_};

    jointStates.header.stamp = ros::Time::now();
    jointStates.name = names;
    jointStates.position = tick2rad(positions);
    jointStates.velocity = tick2rads(speeds);

    return jointStates;
}


pxpincher_rst_msgs::pxpincher_rst_diagnostic PxPincher::getDiagnostics(std::vector<ServoStatus> stati)
{
    pxpincher_rst_msgs::pxpincher_rst_diagnostic diag;
    std::vector<std::string> names = {"J1","J2","J3","J4","J5"};

    diag.header.stamp = ros::Time::now();
    diag.name = names;


    std::vector<int> voltage = {stati[0].voltage_, stati[1].voltage_, stati[2].voltage_, stati[3].voltage_, stati[4].voltage_};
    std::vector<double> temperature = {(double)stati[0].temperature_, (double)stati[1].temperature_, (double)stati[2].temperature_, (double)stati[3].temperature_, (double)stati[4].temperature_};

    diag.temperature = temperature;
    diag.voltage = convVoltage(voltage);

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

