#include "orocos_test_comp.hpp"
#include <rtt/Component.hpp>

Test_Component::Test_Component(std::string const &name) : RTT::TaskContext(name)
{
    addOperation("func", &Test_Component::func, this, RTT::ClientThread).doc("function");
    addOperation("getSimulationTime", &Test_Component::getSimulationTime, this, RTT::ClientThread).doc("getSimulationTime");
    addProperty("var",var);

}

Test_Component::~Test_Component()
{
}

void Test_Component::initializeVariables()
{
    var = 1;

    x_current.setZero(CART_DOF_SIZE); // XYZRPY
    q_current.setZero(ROBOT_DOF_SIZE);

    joint_angles.angles.setZero(ROBOT_DOF_SIZE) ;

}


bool Test_Component::configureHook()
{

    RML = new ReflexxesAPI(ROBOT_DOF_SIZE, getPeriod());
    IP = new RMLPositionInputParameters(ROBOT_DOF_SIZE);
    OP = new RMLPositionOutputParameters(ROBOT_DOF_SIZE);

    initializeVariables();
    initializePorts();

    return true;
}

void Test_Component::updateHook()
{

    if (input_int_port.readNewest(input_int_var) == RTT::NewData)
    {
        var = input_int_var;
        RTT::log(RTT::Warning)<< "var: "<< var << RTT::endlog();
    }

    x_current = x_current * 0.01;
    q_current = q_current * 0.01;


}

bool Test_Component::startHook()
{
    return true;
}

void Test_Component::stopHook()
{
}

void Test_Component::cleanupHook()
{
}

int Test_Component::func()
{
    var++;
    return var;
}

void Test_Component::initializePorts()
{
    // Ports Definitions !!

    input_int_flow = RTT::NoData;
    input_int_port.setName("input_int_port");
    input_int_port.doc("input_int_port");
    ports()->addPort(input_int_port);

    input_double_flow = RTT::NoData;
    input_double_port.setName("input_double_port");
    input_double_port.doc("input_double_port");
    ports()->addPort(input_double_port);

    input_string_flow = RTT::NoData;
    input_string_port.setName("input_string_port");
    input_string_port.doc("input_string_port");
    ports()->addPort(input_string_port);

    input_bool_flow = RTT::NoData;
    input_bool_port.setName("input_bool_port");
    input_bool_port.doc("input_bool_port");
    ports()->addPort(input_bool_port);

}

double Test_Component::getSimulationTime()
{
    return 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
}


ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(Test_Component)
