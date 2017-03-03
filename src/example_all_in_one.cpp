#include "ros/ros.h"
#include "myp_ros/MoveJoint.h"
#include <cstdlib>
#include <string>

/*
This example shows how to use the move_joint service in two ways using a single cpp file.
We assume you have already connected to the robot (you can do this within a program by calling
the "connect" service or via the command line via "rosservice call connect real PRob2R normal"

*/

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "example_cpp");
    
    // Create our node handle
    ros::NodeHandle n;

    // Instantiate a service object for the MoveJoint service
    // NOTE: all values are defaulted to zero (ints/floats), false (bools) or empty arrays
    myp_ros::MoveJoint mj_srv; 

    // Create some arrays with the values we will use
    std::string aids[6] = {"1", "2", "3", "4", "5", "6"};
    float pos1[6] = {30, 30, 30, 0, 0, 10};
    float pos2[6] = {0, 0, 0, 0, 0, 0};
    float vel[6] = {80, 80, 80, 80, 80, 80};
    float acc[6] = {80, 80, 80, 80, 80, 80};

    // There are two ways to call a service in ROS, the "bare" way and using a node handle.

    /*
    BARE METHOD
    Here we directly call a service without creating a client.
    This method is better used when the service will not be called many times.
    */

    // Load in the request values (check srv/MoveJoint.srv for request members)
    mj_srv.request.actuator_ids.assign(&aids[0], &aids[0]+6); // message arrays are vectors
    mj_srv.request.position.assign(&pos1[0], &pos1[0]+6);
    // NOTE: By leaving velocity and acceleration empty, myp will use their default values

    // Wait till the service is online
    ros::service::waitForService("move_joint", -1);

    // Directly call the service
    if (ros::service::call("move_joint", mj_srv))
    {
        // If the request was successfully sent and a response was received print the response
        printf("%s\n", mj_srv.response.message.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service move_joint");
        return 1;
    }
    
    /*
    HANDLE METHOD
    This method involves more set up but then allows you to call the same service from
    a single object, allowing the service to be treated more like a class function.
    */

    // Load in the request values (let's explicitly set the velocity, acceleration and non_blocking)
    mj_srv.request.position.assign(&pos2[0], &pos2[0]+6);
    mj_srv.request.velocity.assign(&vel[0], &vel[0]+6);
    mj_srv.request.acceleration.assign(&acc[0], &acc[0]+6);
    mj_srv.request.not_block = true;
    
    // Create the service client object we will use to call the service
    ros::ServiceClient mj_client = n.serviceClient<myp_ros::MoveJoint>("move_joint");

    
    // Wait till the service is online
    mj_client.waitForExistence(ros::Duration(-1));
    
    // Call the service
    if (mj_client.call(mj_srv))
    {
        // If the request was successfully sent and a response was received print the response
        printf("%s\n", mj_srv.response.message.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service move_joint");
        return 1;
    }

    return 0;
}

