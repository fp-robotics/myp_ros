#include "ExampleClass.h"

using namespace example_ns;

// constructor
ExampleClass::ExampleClass(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "example_cpp");
    
    // Create our node handle
    n = new ros::NodeHandle();
    
    // Assign the subscribers to their topics
    n->subscribe("joint_angles", 1000, &ExampleClass::joint_callback, this);
    n->subscribe("posture", 1000, &ExampleClass::posture_callback, this);

    // Assign the service client objects we will use to call the services
    move_joint_client = n->serviceClient<myp_ros::MoveJoint>("move_joint");
    move_tool_client = n->serviceClient<myp_ros::MoveTool>("move_tool");
    
    // Wait till the services are online
    move_joint_client.waitForExistence(ros::Duration(-1));
    move_tool_client.waitForExistence(ros::Duration(-1));

}

// joint callback
void ExampleClass::joint_callback(const myp_ros::JointAngles::ConstPtr& msg)
{
    // do something with the message data
    float joint_1 = msg->data[0];
}

// posture callback
void ExampleClass::posture_callback(const myp_ros::Posture::ConstPtr& msg)
{
    // do something with the message data
    float x = msg->xyz[0];
}

// wrapper for the move_joint_client with all arguments
std::string ExampleClass::move_joint(int len, std::string actuator_ids[], float position[],
    float velocity[], float acceleration[], bool not_block, 
        bool relative)
{
    myp_ros::MoveJoint mj_srv;
    mj_srv.request.actuator_ids.assign(&actuator_ids[0], &actuator_ids[0]+len);
    mj_srv.request.position.assign(&position[0], &position[0]+len);
    mj_srv.request.velocity.assign(&velocity[0], &velocity[0]+len);
    mj_srv.request.acceleration.assign(&acceleration[0], &acceleration[0]+len);
    mj_srv.request.not_block = not_block;
    mj_srv.request.relative = relative;
    move_joint_client.call(mj_srv);
    return mj_srv.response.message;
}
        
// wrapper for the move_joint_client with all arguemnts except ids
std::string ExampleClass::move_joint(int len, float position[], float velocity[], 
    float acceleration[], bool not_block, bool relative)
{
    myp_ros::MoveJoint mj_srv;
    mj_srv.request.position.assign(&position[0], &position[0]+len);
    mj_srv.request.velocity.assign(&velocity[0], &velocity[0]+len);
    mj_srv.request.acceleration.assign(&acceleration[0], &acceleration[0]+len);
    mj_srv.request.not_block = not_block;
    mj_srv.request.relative = relative;
    move_joint_client.call(mj_srv);
    return mj_srv.response.message;
}

// wrapper for the move_joint_client without acceleration
std::string ExampleClass::move_joint(int len, float position[], float velocity[], 
    bool not_block, bool relative)
{
    myp_ros::MoveJoint mj_srv;
    mj_srv.request.position.assign(&position[0], &position[0]+len);
    mj_srv.request.velocity.assign(&velocity[0], &velocity[0]+len);
    mj_srv.request.not_block = not_block;
    mj_srv.request.relative = relative;
    move_joint_client.call(mj_srv);
    return mj_srv.response.message;
}

// wrapper for the move_joint_client without velocity or acceleration
std::string ExampleClass::move_joint(int len, float position[], bool not_block,
    bool relative)
    
{
    myp_ros::MoveJoint mj_srv;
    mj_srv.request.position.assign(&position[0], &position[0]+len);
    mj_srv.request.not_block = not_block;
    mj_srv.request.relative = relative;
    move_joint_client.call(mj_srv);
    return mj_srv.response.message;
}

// wrapper for the move_tool_client with all arguments
std::string ExampleClass::move_tool(float x, float y, float z, float orientation[],
    float velocity, float acceleration, bool not_block,
        bool relative)

{
    myp_ros::MoveTool mt_srv;
    mt_srv.request.x = x;
    mt_srv.request.y = y;
    mt_srv.request.z = z;
    mt_srv.request.orientation.assign(&orientation[0], &orientation[0]+3);
    mt_srv.request.velocity = velocity;
    mt_srv.request.acceleration = acceleration;
    mt_srv.request.not_block = not_block;
    mt_srv.request.relative = relative;
    move_tool_client.call(mt_srv);
    return mt_srv.response.message;
}
        
// wrapper for the move_tool_client with all arguemnts except orientation
std::string ExampleClass::move_tool(float x, float y, float z, float velocity,
    float acceleration, bool not_block, bool relative)
{
    myp_ros::MoveTool mt_srv;
    mt_srv.request.x = x;
    mt_srv.request.y = y;
    mt_srv.request.z = z;
    mt_srv.request.velocity = velocity;
    mt_srv.request.acceleration = acceleration;
    mt_srv.request.not_block = not_block;
    mt_srv.request.relative = relative;
    move_tool_client.call(mt_srv);
    return mt_srv.response.message;
}

void ExampleClass::run()
{
    // Create some arrays with the values we will use
    std::string aids[6] = {"1", "2", "3", "4", "5", "6"};
    float pos1[6] = {30, 30, 30, 0, 0, 10};
    float pos2[6] = {0, 0, 0, 0, 0, 0};
    float vel[6] = {80, 80, 80, 80, 80, 80};
    float acc[6] = {80, 80, 80, 80, 80, 80};
    float ori[3] = {0, 0, 0};
    
    ROS_INFO("%s", "Moving joints");
    ROS_INFO("%s", move_joint(6, pos1).c_str());
    ROS_INFO("%s", "Moving back");
    ROS_INFO("%s", move_joint(6, pos2, vel).c_str());
    ROS_INFO("%s", "Moving joints");
    ROS_INFO("%s", move_joint(6, pos1, vel, acc).c_str());
    ROS_INFO("%s", "Moving back with a non-blocking and a 1 second sleep");
    ROS_INFO("%s", move_joint(6, aids, pos2, vel, acc, true).c_str());
    ros::Duration(1.0).sleep();
        
    ROS_INFO("%s", "Moving tool");
    ROS_INFO("%s", move_tool(100, 100, 1000).c_str());
    ROS_INFO("%s", "Moving tool with non-blocking and a 1 second sleep");
    ROS_INFO("%s", move_tool(-100, 100, 1000, 30, 30, true).c_str());
    ros::Duration(1.0).sleep();
    ROS_INFO("%s", "Moving tool to interupt");
    ROS_INFO("%s", move_tool(-100, -100, 1000, ori, 30, 30, false).c_str());
    ROS_INFO("%s", "Moving back");
    ROS_INFO("%s", move_joint(6, aids, pos2, vel, acc, true).c_str());
    ROS_INFO("Subscribers are still spinning, press CTRL+C to exit.");
    
}
