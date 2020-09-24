#ifndef OROCOS_TEST_COMP
#define OROCOS_TEST_COMP

#include <rtt/RTT.hpp>
#include <Eigen/Dense>
#include <string>
#include <boost/filesystem.hpp>
#define CART_DOF_SIZE 6
#define ROBOT_DOF_SIZE 7
#include "reflexxes/ReflexxesAPI.h"
#include "reflexxes/RMLPositionFlags.h"
#include "reflexxes/RMLPositionInputParameters.h"
#include "reflexxes/RMLPositionOutputParameters.h"

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/segment.hpp>
#include <kdl/frames.hpp>

#include "yaml-cpp/yaml.h"
#include <fstream>

#include <sys/socket.h>
#include <bits/stdc++.h>
#include <jsoncpp/json/json.h>

#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/kinematics/JointAccelerations.hpp>
#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/geometry/Pose.hpp>
#include <rst-rt/dynamics/Wrench.hpp>

#include <trac_ik.hpp>


class Test_Component: public RTT::TaskContext
{
public:

    Test_Component(std::string const& name);
    ~Test_Component();
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();
    int func();


private:

    void initializeVariables();
    void initializePorts();
    TRAC_IK::TRAC_IK *trac_ik;

    double t;
    int var;

    Eigen::VectorXf x_current; // XYZRPY
    Eigen::VectorXf q_current;

    RTT::InputPort<int> input_int_port;
    int input_int_var;
    RTT::FlowStatus input_int_flow;

    RTT::InputPort<double> input_double_port;
    double input_double_var;
    RTT::FlowStatus input_double_flow;

    RTT::InputPort<std::string> input_string_port;
    std::string input_string_var;
    RTT::FlowStatus input_string_flow;

    RTT::InputPort<bool> input_bool_port;
    bool input_bool_var;
    RTT::FlowStatus input_bool_flow;


    int ResultValue;
    ReflexxesAPI* RML;
    RMLPositionInputParameters* IP;
    RMLPositionOutputParameters* OP;
    RMLPositionFlags Flags;

    int ResultValueLine;
    ReflexxesAPI* RMLLine;
    RMLPositionInputParameters* IPLine;
    RMLPositionOutputParameters* OPLine;
    RMLPositionFlags FlagsLine;

    KDL::Frame frame;
    KDL::Vector vec;
    KDL::Rotation rot;

    rstrt::geometry::Pose pose;
    rstrt::kinematics::JointAngles joint_angles;

    double getSimulationTime();
    
};

#endif // OROCOS_TEST_COMP
