#include "ros/ros.h"
#include "myp_ros/MoveJoint.h"
#include "myp_ros/MoveTool.h"
#include "myp_ros/JointAngles.h"
#include "myp_ros/Posture.h"
#include <cstdlib>
#include <string>

namespace example_ns{
    class ExampleClass
    {
        private:
            // Node handle pointer, subscribers and service clients
            ros::NodeHandle *n;
            ros::Subscriber joint_sub;
            ros::Subscriber posture_sub;
            ros::ServiceClient move_joint_client;
            ros::ServiceClient move_tool_client;
            
        public:
            // constructor
            ExampleClass(int argc, char **argv);
            
            // callbacks for subscribers
            void joint_callback(const myp_ros::JointAngles::ConstPtr& msg);
            void posture_callback(const myp_ros::Posture::ConstPtr& msg);
            
            // wrapper for the move_joint_client with all arguments
            std::string move_joint(int len, std::string actuator_ids[], float position[],
                float velocity[], float acceleration[], bool not_block=false, 
                    bool relative=false);
                    
            // wrapper for the move_joint_client with all arguemnts except ids
            std::string move_joint(int len, float position[], float velocity[], 
                float acceleration[], bool not_block=false, bool relative=false);
            
            // wrapper for the move_joint_client without acceleration
            std::string move_joint(int len, float position[], float velocity[], 
                bool not_block=false, bool relative=false);
            
            // wrapper for the move_joint_client without velocity or acceleration
            std::string move_joint(int len, float position[], bool not_block=false,
                bool relative=false);
            
            // wrapper for the move_tool_client with all arguments
            std::string move_tool(float x, float y, float z, float orientation[],
                float velocity=0, float acceleration=0, bool not_block=false,
                    bool relative=false);
                    
            // wrapper for the move_tool_client with all arguemnts except orientation
            std::string move_tool(float x, float y, float z, float velocity=0,
                float acceleration=0, bool not_block=false, bool relative=false);
                
            // run function
            void run();
    };
}
