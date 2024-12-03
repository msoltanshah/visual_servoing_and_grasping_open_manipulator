/*
    Visual Servoing Grasping Node
    Author: Mohammad Soltanshah
    Email: m.soltanshah1990@gmail.com
*/

#include <math.h>
#include<array>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <uam_msgs/StartVS.h>
#include <uam_msgs/GrabIt.h>
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include "open_manipulator_msgs/SetJointPosition.h"

#define PI 3.14159265

namespace VSGrasping
{
    class VSGraspingNode
	{
        protected:
            ros::NodeHandle nh_;
            ros::NodeHandle private_nh_;
            ros::Subscriber object_pose_sub, start_VS_sub, grab_it_sub;
            ros::ServiceClient goal_task_space_client_, goal_joint_space_path_client_;
            
        private: 
            void init(void);
            void createSubscribers(void);
            void createServices(void);
            void object_pose_cb(const geometry_msgs::PoseStamped& msg);
            void start_VS_cb(const uam_msgs::StartVS& msg);
            void grab_it_cb(const uam_msgs::GrabIt& msg);
            bool setTaskSpacePath(std::vector<double> kinematics_pose, double path_time);
            bool setTaskSpacePathGrab(std::vector<double> kinematics_pose, double path_time);
            bool setToolControl(std::vector<double> joint_angle, double path_time);
            geometry_msgs::PoseStamped object_pose_message;
            uam_msgs::StartVS start_VS_message;
            uam_msgs::GrabIt grab_it_message;
            geometry_msgs::PoseStamped target_pose_command_message;
            double vs_grasping_loop_rate;
            bool manual_grasping;
            std::vector<double> kinematics_pose;
            double path_time_track;
            double path_time_grab;

            std::vector<double> track_vector_0;
            std::vector<double> grab_vector_0;
            std::vector<double> joint_angle_track, joint_angle_grab;

            std::array<std::array<double,4>,4> T_0_O, T_0_B, T_B_C, T_C_O;
            std::array<std::array<double,3>,3> R_0_B, R_B_C, R_C_O;
            std::array<std::array<double,3>,3> R_B_C_z, R_B_C_x, R_B_C_y, R_C_O_z, R_C_O_x;

            double p_O_x = 0.10, p_O_y = 0.0, p_O_z = 0.0;
            double shift_x = 0.05, shift_y = 0.0, shift_z = 0.0;
            double point_x, point_y, point_z;

            std::vector<double> t_0_B = { 0.11, 0, 0}, t_B_C = { 0.15, 0.45, 0.2};

            std::vector<double> calculateTransformation(double p_O_x_num, double p_O_y_num, double p_O_z_num, const geometry_msgs::PoseStamped& object_pose_message);
            std::array<std::array<double,4>,4> matMult_4x4(std::array<std::array<double,4>,4> a, std::array<std::array<double,4>,4> b);
            std::array<std::array<double,3>,3> matMult_3x3(std::array<std::array<double,3>,3> a, std::array<std::array<double,3>,3> b);
            std::vector<double> matVecMult(std::array<std::array<double,4>,4> c, std::vector<double> d);
       
        public:
            bool is_new_object_pose, is_new_start_VS_message, is_new_grab_it_message;
            void run(void);
            // Defining the class constructor
            VSGraspingNode(int argc, char** argv)
            :private_nh_("~")
            {
                init();
            }
    };

}
