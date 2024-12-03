/*
    Visual Servoing Grasping Node
    Author: Mohammad Soltanshah
    Email: m.soltanshah1990@gmail.com
*/

#include <vs_grasping_node.h>

using namespace VSGrasping;

void VSGraspingNode::object_pose_cb(const geometry_msgs::PoseStamped& msg){
    object_pose_message = msg;
    is_new_object_pose = true;
}

void VSGraspingNode::start_VS_cb(const uam_msgs::StartVS& msg){
    start_VS_message = msg;
    is_new_start_VS_message = true;
}

void VSGraspingNode::grab_it_cb(const uam_msgs::GrabIt& msg){
    grab_it_message = msg;
    is_new_grab_it_message = true;
}

void VSGraspingNode::init(void)
{
    private_nh_.param("vs_grasping_loop_rate", vs_grasping_loop_rate, double(5));
    private_nh_.param("manual_graspoing", manual_grasping, bool(true));

    createSubscribers();
    createServices();
}

void VSGraspingNode::createSubscribers(void)
{
    // Subscriber Creation
    object_pose_sub = nh_.subscribe("/visp_auto_tracker/object_position", 10, &VSGraspingNode::object_pose_cb, this);
    start_VS_sub = nh_.subscribe("/start_VS", 10, &VSGraspingNode::start_VS_cb, this);
    grab_it_sub = nh_.subscribe("/grab_it", 10, &VSGraspingNode::grab_it_cb, this);
}

void VSGraspingNode::createServices(void)
{
    // Service Client Creation
    goal_task_space_client_ = nh_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_position_only");
    goal_joint_space_path_client_ = nh_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");

}

std::array<std::array<double,4>,4> VSGraspingNode::matMult_4x4(std::array<std::array<double,4>,4> a, std::array<std::array<double,4>,4> b)
{
    std::array<std::array<double,4>,4> c_array;
    int i, j, k;

    // Initializing elements of matrix mult to 0.
    for(i = 0; i < 4; ++i)
        for(j = 0; j < 4; ++j)
        {
            c_array[i][j]=0;
        }
    
    // Multiplying matrix a and b and storing in array mult.
    for(i = 0; i < 4; ++i)
        for(j = 0; j < 4; ++j)
            for(k = 0; k < 4; ++k)
            {
                c_array[i][j] += a[i][k] * b[k][j];
            }
    
    return c_array;
}

std::array<std::array<double,3>,3> VSGraspingNode::matMult_3x3(std::array<std::array<double,3>,3> a, std::array<std::array<double,3>,3> b)
{
    std::array<std::array<double,3>,3> c_array;
    int i, j, k;

    // Initializing elements of matrix mult to 0.
    for(i = 0; i < 3; ++i)
        for(j = 0; j < 3; ++j)
        {
            c_array[i][j]=0;
        }
    
    // Multiplying matrix a and b and storing in array mult.
    for(i = 0; i < 3; ++i)
        for(j = 0; j < 3; ++j)
            for(k = 0; k < 3; ++k)
            {
                c_array[i][j] += a[i][k] * b[k][j];
            }
    
    return c_array;
}

std::vector<double> VSGraspingNode::matVecMult(std::array<std::array<double,4>,4> c, std::vector<double> d)
{
    std::vector<double> c_vector = { 0, 0, 0, 0 };
    int i, j, k;

    // Initializing elements of matrix mult to 0.
    for(i = 0; i < 4; ++i)
    {
        c_vector[i]=0;
    }
            
    // Multiplying matrix a and b and storing in array mult.
    for(i = 0; i < 4; ++i)
    {
            for(j = 0; j < 4; ++j)
            {
                c_vector[i] += c[i][j] * d[j];
            }  
    }
        
    
    return c_vector;
}

std::vector<double> VSGraspingNode::calculateTransformation(double p_O_x_num, double p_O_y_num, double p_O_z_num, const geometry_msgs::PoseStamped& object_pose_message)
{
    int i, j, k;
    std::vector<double> X_vector_num = { 0, 0, 0, 0 };
    std::vector<double> p_O_vector_num = { p_O_x_num, p_O_y_num, p_O_z_num, 1 };
    // std::vector<double> p_O_vector_num = { p_O_x, p_O_y, p_O_z, 1 };
    // double p_0_x_num, p_0_y_num, p_0_z_num;

    // C to O
    R_C_O_z[0][0] = cos(-PI/2); R_C_O_z[0][1] = -sin(-PI/2); R_C_O_z[0][2] = 0;
    R_C_O_z[1][0] = sin(-PI/2); R_C_O_z[1][1] = cos(-PI/2);  R_C_O_z[1][2] = 0;
    R_C_O_z[2][0] = 0;          R_C_O_z[2][1] = 0;           R_C_O_z[2][2] = 1;

    R_C_O_x[0][0] = 1; R_C_O_x[0][1] = 0;         R_C_O_x[0][2] = 0;
    R_C_O_x[1][0] = 0; R_C_O_x[1][1] = cos(PI/2); R_C_O_x[1][2] = -sin(PI/2);
    R_C_O_x[2][0] = 0; R_C_O_x[2][1] = sin(PI/2); R_C_O_x[2][2] = cos(PI/2);

    R_C_O = matMult_3x3(R_C_O_z, R_C_O_x);

    T_C_O[0][0] = R_C_O[0][0]; T_C_O[0][1] = R_C_O[0][1]; T_C_O[0][2] = R_C_O[0][2]; T_C_O[0][3] = object_pose_message.pose.position.z; 
    T_C_O[1][0] = R_C_O[1][0]; T_C_O[1][1] = R_C_O[1][1]; T_C_O[1][2] = R_C_O[1][2]; T_C_O[1][3] = -object_pose_message.pose.position.x;
    T_C_O[2][0] = R_C_O[2][0]; T_C_O[2][1] = R_C_O[2][1]; T_C_O[2][2] = R_C_O[2][2]; T_C_O[2][3] = -object_pose_message.pose.position.y;
    T_C_O[3][0] = 0;           T_C_O[3][1] = 0;           T_C_O[3][2] = 0;           T_C_O[3][3] = 1;

    for(i = 0; i < 4; ++i)
    for(j = 0; j < 4; ++j)
    {
        if(abs(T_C_O[i][j]) < 1e-05)
            T_C_O[i][j] = 0;
    }

    // B to C
    R_B_C_z[0][0] = cos(-PI/2); R_B_C_z[0][1] = -sin(-PI/2); R_B_C_z[0][2] = 0;
    R_B_C_z[1][0] = sin(-PI/2); R_B_C_z[1][1] = cos(-PI/2);  R_B_C_z[1][2] = 0;
    R_B_C_z[2][0] = 0;          R_B_C_z[2][1] = 0;           R_B_C_z[2][2] = 1;

    R_B_C = R_B_C_z;

    T_B_C[0][0] = R_B_C[0][0]; T_B_C[0][1] = R_B_C[0][1]; T_B_C[0][2] = R_B_C[0][2]; T_B_C[0][3] = t_B_C[0]; 
    T_B_C[1][0] = R_B_C[1][0]; T_B_C[1][1] = R_B_C[1][1]; T_B_C[1][2] = R_B_C[1][2]; T_B_C[1][3] = t_B_C[1];
    T_B_C[2][0] = R_B_C[2][0]; T_B_C[2][1] = R_B_C[2][1]; T_B_C[2][2] = R_B_C[2][2]; T_B_C[2][3] = t_B_C[2];
    T_B_C[3][0] = 0;           T_B_C[3][1] = 0;           T_B_C[3][2] = 0;           T_B_C[3][3] = 1;

    for(i = 0; i < 4; ++i)
    for(j = 0; j < 4; ++j)
    {
        if(abs(T_B_C[i][j]) < 1e-05)
            T_B_C[i][j] = 0;
    }

    // 0 to B
    R_0_B[0][0] = 1; R_0_B[0][1] = 0; R_0_B[0][2] = 0;
    R_0_B[1][0] = 0; R_0_B[1][1] = 1; R_0_B[1][2] = 0;
    R_0_B[2][0] = 0; R_0_B[2][1] = 0; R_0_B[2][2] = 1;

    T_0_B[0][0] = R_0_B[0][0]; T_0_B[0][1] = R_0_B[0][1]; T_0_B[0][2] = R_0_B[0][2]; T_0_B[0][3] = t_0_B[0]; 
    T_0_B[1][0] = R_0_B[1][0]; T_0_B[1][1] = R_0_B[1][1]; T_0_B[1][2] = R_0_B[1][2]; T_0_B[1][3] = t_0_B[1];
    T_0_B[2][0] = R_0_B[2][0]; T_0_B[2][1] = R_0_B[2][1]; T_0_B[2][2] = R_0_B[2][2]; T_0_B[2][3] = t_0_B[2];
    T_0_B[3][0] = 0;           T_0_B[3][1] = 0;           T_0_B[3][2] = 0;           T_0_B[3][3] = 1;

    for(i = 0; i < 4; ++i)
    for(j = 0; j < 4; ++j)
    {
        if(abs(T_0_B[i][j]) < 1e-05)
            T_0_B[i][j] = 0;
    }

    T_0_O = matMult_4x4(matMult_4x4(T_0_B, T_B_C),T_C_O);

    X_vector_num = matVecMult(T_0_O, p_O_vector_num); 

    return X_vector_num;
}

bool VSGraspingNode::setTaskSpacePath(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv_task_space;

  srv_task_space.request.end_effector_name = "gripper";

  srv_task_space.request.kinematics_pose.pose.position.x = kinematics_pose[0];
  srv_task_space.request.kinematics_pose.pose.position.y = kinematics_pose[1];
  srv_task_space.request.kinematics_pose.pose.position.z = kinematics_pose[2];

//   srv.request.kinematics_pose.pose.orientation.w = 0;
//   srv.request.kinematics_pose.pose.orientation.x = 0;
//   srv.request.kinematics_pose.pose.orientation.y = 0;
//   srv.request.kinematics_pose.pose.orientation.z = 0;

  srv_task_space.request.path_time = path_time;

  if(goal_task_space_client_.call(srv_task_space))
  {
    return srv_task_space.response.is_planned;
  }
  return false;
}

// bool VSGraspingNode::setTaskSpacePathGrab(std::vector<double> kinematics_pose, double path_time)
// {
//   open_manipulator_msgs::SetKinematicsPose srv_task_space_grab;

//   srv_task_space_grab.request.end_effector_name = "gripper";

//   srv_task_space_grab.request.kinematics_pose.pose.position.x = kinematics_pose[0];
//   srv_task_space_grab.request.kinematics_pose.pose.position.y = kinematics_pose[1];
//   srv_task_space_grab.request.kinematics_pose.pose.position.z = kinematics_pose[2];

// //   srv.request.kinematics_pose.pose.orientation.w = 0;
// //   srv.request.kinematics_pose.pose.orientation.x = 0;
// //   srv.request.kinematics_pose.pose.orientation.y = 0;
// //   srv.request.kinematics_pose.pose.orientation.z = 0;

//   srv_task_space_grab.request.path_time = path_time;

//   if(goal_task_space_client_.call(srv_task_space_grab))
//   {
//     return srv_task_space_grab.response.is_planned;
//   }
//   return false;
// }

bool VSGraspingNode::setToolControl(std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv_tool_control;
  srv_tool_control.request.joint_position.joint_name.push_back("gripper");
  srv_tool_control.request.joint_position.position = joint_angle;
  srv_tool_control.request.path_time = path_time;

  if(goal_joint_space_path_client_.call(srv_tool_control))
  {
    return srv_tool_control.response.is_planned;
  }
  return false;
}

void VSGraspingNode::run(void)
{
    ros::Rate loop_rate(vs_grasping_loop_rate);
    ros::Duration duration(0.2);

    is_new_object_pose = false;
    is_new_start_VS_message = false;
    is_new_grab_it_message = false;

    while (ros::ok())
    {

		if (start_VS_message.start_vs)
        {
            if (is_new_object_pose)
            {
                if (!grab_it_message.grab_it)
                { 
                    ROS_INFO("grab_it off");
                    path_time_track = 1;
                    point_x = p_O_x + shift_x;
                    point_y = p_O_y + shift_y;
                    point_z = p_O_z + shift_z;
                    track_vector_0 = calculateTransformation(point_x, point_y, point_z, object_pose_message);

                    std::cout << "X_0_track = " << track_vector_0[0] << std::endl;
                    std::cout << "Y_0_track = " << track_vector_0[1] << std::endl;
                    std::cout << "Z_0_track = " << track_vector_0[2] << std::endl;

                    setTaskSpacePath(track_vector_0, path_time_track);

                    joint_angle_track.push_back(0.01);
                    setToolControl(joint_angle_track, path_time_track);
                } 
                if (grab_it_message.grab_it)
                {
                    ROS_INFO("grab_it on");
                    path_time_grab = 0.5;
                    grab_vector_0 = calculateTransformation(p_O_x, p_O_y, p_O_z, object_pose_message);

                    std::cout << "X_0_grab = " << grab_vector_0[0] << std::endl;
                    std::cout << "Y_0_grab = " << grab_vector_0[1] << std::endl;
                    std::cout << "Z_0_grab = " << grab_vector_0[2] << std::endl;

                    setTaskSpacePath(grab_vector_0, path_time_track);

                    duration.sleep();

                    joint_angle_grab.push_back(-0.005);
                    setToolControl(joint_angle_grab, path_time_grab);
                }
            } 
        }

        is_new_object_pose = false;
        is_new_start_VS_message = false;
        is_new_grab_it_message = false;
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}