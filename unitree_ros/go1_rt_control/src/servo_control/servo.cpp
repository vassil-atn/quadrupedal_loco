/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
Edited by Jiatao Ding, email: jtdingx@gmail.com
************************************************************************/

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include "body.h"
#include "servo.h"
#include <Eigen/Dense>
#include "kinematics/Kinematics.h"
#include "sensor_msgs/JointState.h"
#include "Robotpara/robot_const_para_config.h"

using namespace std;
using namespace unitree_model;

bool start_up = true;

#define PI 3.1415926

sensor_msgs::JointState joint2simulation;




class multiThread
{
public:
    multiThread(string rname){
        robot_name = rname;
        imu_sub = nm.subscribe("/trunk_imu", 1, &multiThread::imuCallback, this);
        footForce_sub[0] = nm.subscribe("/visual/FR_foot_contact/the_force", 1, &multiThread::FRfootCallback, this);
        footForce_sub[1] = nm.subscribe("/visual/FL_foot_contact/the_force", 1, &multiThread::FLfootCallback, this);
        footForce_sub[2] = nm.subscribe("/visual/RR_foot_contact/the_force", 1, &multiThread::RRfootCallback, this);
        footForce_sub[3] = nm.subscribe("/visual/RL_foot_contact/the_force", 1, &multiThread::RLfootCallback, this);
        servo_sub[0] = nm.subscribe("/" + robot_name + "_gazebo/FR_hip_controller/state", 1, &multiThread::FRhipCallback, this);
        servo_sub[1] = nm.subscribe("/" + robot_name + "_gazebo/FR_thigh_controller/state", 1, &multiThread::FRthighCallback, this);
        servo_sub[2] = nm.subscribe("/" + robot_name + "_gazebo/FR_calf_controller/state", 1, &multiThread::FRcalfCallback, this);
        servo_sub[3] = nm.subscribe("/" + robot_name + "_gazebo/FL_hip_controller/state", 1, &multiThread::FLhipCallback, this);
        servo_sub[4] = nm.subscribe("/" + robot_name + "_gazebo/FL_thigh_controller/state", 1, &multiThread::FLthighCallback, this);
        servo_sub[5] = nm.subscribe("/" + robot_name + "_gazebo/FL_calf_controller/state", 1, &multiThread::FLcalfCallback, this);
        servo_sub[6] = nm.subscribe("/" + robot_name + "_gazebo/RR_hip_controller/state", 1, &multiThread::RRhipCallback, this);
        servo_sub[7] = nm.subscribe("/" + robot_name + "_gazebo/RR_thigh_controller/state", 1, &multiThread::RRthighCallback, this);
        servo_sub[8] = nm.subscribe("/" + robot_name + "_gazebo/RR_calf_controller/state", 1, &multiThread::RRcalfCallback, this);
        servo_sub[9] = nm.subscribe("/" + robot_name + "_gazebo/RL_hip_controller/state", 1, &multiThread::RLhipCallback, this);
        servo_sub[10] = nm.subscribe("/" + robot_name + "_gazebo/RL_thigh_controller/state", 1, &multiThread::RLthighCallback, this);
        servo_sub[11] = nm.subscribe("/" + robot_name + "_gazebo/RL_calf_controller/state", 1, &multiThread::RLcalfCallback, this);
    
        //// topic with first-layer MPC
        nrt_mpc_gait_subscribe_ = nm.subscribe("/MPC/Gait", 10, &multiThread::nrt_gait_sub_operation, this);
        gait_des_sub_ = nm.subscribe("/rtMPC/traj", 10,&multiThread::rt_gait_sub_operation, this);
             
    
    
    }

    void imuCallback(const sensor_msgs::Imu & msg)
    { 
        lowState.imu.quaternion[0] = msg.orientation.w;
        lowState.imu.quaternion[1] = msg.orientation.x;
        lowState.imu.quaternion[2] = msg.orientation.y;
        lowState.imu.quaternion[3] = msg.orientation.z;

        lowState.imu.gyroscope[0] = msg.angular_velocity.x;
        lowState.imu.gyroscope[1] = msg.angular_velocity.y;
        lowState.imu.gyroscope[2] = msg.angular_velocity.z;
        
        lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
        lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
        lowState.imu.accelerometer[2] = msg.linear_acceleration.z;
        
    }

    void FRhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[0].mode = msg.mode;
        lowState.motorState[0].q = msg.q;
        lowState.motorState[0].dq = msg.dq;
        lowState.motorState[0].tauEst = msg.tauEst;
        
    }

    void FRthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[1].mode = msg.mode;
        lowState.motorState[1].q = msg.q;
        lowState.motorState[1].dq = msg.dq;
        lowState.motorState[1].tauEst = msg.tauEst;

    }

    void FRcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[2].mode = msg.mode;
        lowState.motorState[2].q = msg.q;
        lowState.motorState[2].dq = msg.dq;
        lowState.motorState[2].tauEst = msg.tauEst;

    }

    void FLhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[3].mode = msg.mode;
        lowState.motorState[3].q = msg.q;
        lowState.motorState[3].dq = msg.dq;
        lowState.motorState[3].tauEst = msg.tauEst;

    }

    void FLthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[4].mode = msg.mode;
        lowState.motorState[4].q = msg.q;
        lowState.motorState[4].dq = msg.dq;
        lowState.motorState[4].tauEst = msg.tauEst;

    }

    void FLcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[5].mode = msg.mode;
        lowState.motorState[5].q = msg.q;
        lowState.motorState[5].dq = msg.dq;
        lowState.motorState[5].tauEst = msg.tauEst;

    }

    void RRhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[6].mode = msg.mode;
        lowState.motorState[6].q = msg.q;
        lowState.motorState[6].dq = msg.dq;
        lowState.motorState[6].tauEst = msg.tauEst;

    }

    void RRthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[7].mode = msg.mode;
        lowState.motorState[7].q = msg.q;
        lowState.motorState[7].dq = msg.dq;
        lowState.motorState[7].tauEst = msg.tauEst;

    }

    void RRcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[8].mode = msg.mode;
        lowState.motorState[8].q = msg.q;
        lowState.motorState[8].dq = msg.dq;
        lowState.motorState[8].tauEst = msg.tauEst;

    }

    void RLhipCallback(const unitree_legged_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[9].mode = msg.mode;
        lowState.motorState[9].q = msg.q;
        lowState.motorState[9].dq = msg.dq;
        lowState.motorState[9].tauEst = msg.tauEst;
    }

    void RLthighCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[10].mode = msg.mode;
        lowState.motorState[10].q = msg.q;
        lowState.motorState[10].dq = msg.dq;
        lowState.motorState[10].tauEst = msg.tauEst;
    }

    void RLcalfCallback(const unitree_legged_msgs::MotorState& msg)
    {
        lowState.motorState[11].mode = msg.mode;
        lowState.motorState[11].q = msg.q;
        lowState.motorState[11].dq = msg.dq;
        lowState.motorState[11].tauEst = msg.tauEst;
    }

    void FRfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[0].x = msg.wrench.force.x;
        lowState.eeForce[0].y = msg.wrench.force.y;
        lowState.eeForce[0].z = msg.wrench.force.z;
        lowState.footForce[0] = msg.wrench.force.z;
    }

    void FLfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[1].x = msg.wrench.force.x;
        lowState.eeForce[1].y = msg.wrench.force.y;
        lowState.eeForce[1].z = msg.wrench.force.z;
        lowState.footForce[1] = msg.wrench.force.z;
    }

    void RRfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[2].x = msg.wrench.force.x;
        lowState.eeForce[2].y = msg.wrench.force.y;
        lowState.eeForce[2].z = msg.wrench.force.z;
        lowState.footForce[2] = msg.wrench.force.z;
    }

    void RLfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[3].x = msg.wrench.force.x;
        lowState.eeForce[3].y = msg.wrench.force.y;
        lowState.eeForce[3].z = msg.wrench.force.z;
        lowState.footForce[3] = msg.wrench.force.z;
    }


    //// ====================================== real-time mpc control //////////
    void nrt_gait_sub_operation(const sensor_msgs::JointState::ConstPtr &msg)
    {
	for (int jx = 0; jx<100; jx++)
	{
	    slow_mpc_gait(jx) = msg->position[jx]; 
	}
	mpc_gait_flag = slow_mpc_gait(99);
    }
    
    
    void rt_gait_sub_operation(const sensor_msgs::JointState::ConstPtr &msg)
    {
	for (int jx = 0; jx<51; jx++)
	{
	    fast_mpc_gait(jx) = msg->position[36+jx]; 
	}
	fast_mpc_gait_flag = msg->position[99];
	count_in_mpc_max = msg->position[98];
	
    }    
    
  
  


private:
    ros::NodeHandle nm;
    ros::Subscriber servo_sub[12], footForce_sub[4], imu_sub, nrt_mpc_gait_subscribe_, gait_des_sub_;


    string robot_name;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "unitree_gazebo_servo");

    string robot_name;
    ros::param::get("/robot_name", robot_name);
    cout << "robot_name: " << robot_name << endl;
    

    gait_mode = 102;
    x_offset = 0.01;
    y_offset = 0.75;


    //desired joint angles
    FR_angle_des.setZero(); FL_angle_des.setZero(); RR_angle_des.setZero(); RL_angle_des.setZero(); 
    //measure angles
    FR_angle_mea.setZero(); FL_angle_mea.setZero(); RR_angle_mea.setZero(); RL_angle_mea.setZero();

    // desired foot location reatlive to body center;
    FR_foot_relative_des.setZero(); FL_foot_relative_des.setZero();
    RR_foot_relative_des.setZero(); RL_foot_relative_des.setZero();

    // measured foot location reatlive to body center;
    FR_foot_relative_mea.setZero(); FL_foot_relative_mea.setZero();
    RR_foot_relative_mea.setZero(); RL_foot_relative_mea.setZero(); 
    
    // desired body posioin and rotation
    body_p_Homing.setZero();
    FR_foot_Homing.setZero();
    FL_foot_Homing.setZero();
    RR_foot_Homing.setZero();
    RL_foot_Homing.setZero();


    body_p_des.setZero();
    body_r_des.setZero();
    FR_foot_des.setZero(); 
    FL_foot_des.setZero(); 
    RR_foot_des.setZero(); 
    RL_foot_des.setZero();
   

////////////////////////// state estimation///////////////////////
    support_flag = 0; /////left: 0; right: 1; double: 2

    fz_double = gait::mass * gait::_g /2; 
    fz_limit = gait::force_z_limt;
    omega_sensor = sqrt(gait::_g/gait::Z_c);

        
    support_pos_sensor[0] = 0; ///left support by default
    support_pos_sensor[1] = gait::RobotParaClass_HALF_HIP_WIDTH; ///left support by default
    support_pos_sensor[2] = 0; ///left support by default
    com_sensor[0] = 0; /////assuming com is 10cm above the pelvis
    com_sensor[1] = 0;
    com_sensor[2] = (gait::Z_c);
    com_sensor_hip[0] = 0; /////assuming com is 10cm above the pelvis
    com_sensor_hip[1] = 0;
    com_sensor_hip[2] = (gait::Z_c);	

    rfoot_pose_sensor[0] = 0;
    rfoot_pose_sensor[1] = -gait::RobotParaClass_HALF_HIP_WIDTH;
    rfoot_pose_sensor[2] = 0;
    lfoot_pose_sensor[0] = 0;
    lfoot_pose_sensor[1] = gait::RobotParaClass_HALF_HIP_WIDTH;
    lfoot_pose_sensor[2] = 0;


    using_ft_sensor = false;       

    for(int i = 0; i < 3; i++){

        com_sensor_pre[i] = 0;
        com_des[i] = 0;
        com_des_pre[i] = 0;
        rfoot_des[i] = 0;
        lfoot_des[i] = 0;
        theta_des[i] = 0;
        theta_des_pre[i] = 0;
        rfoot_theta_des[i] = 0;
        lfoot_theta_des[i] = 0;

        comv_sensor[i] = 0;
        coma_sensor[i] = 0;
        zmp_sensor[i] = 0;
        zmp_ref[i] = 0;
        dcm_ref[i]= 0;
        dcm_sensor[i] = 0;        

    }

    com_des[2] = gait::Z_c;

    rfoot_des[1] = -gait::RobotParaClass_HALF_HIP_WIDTH;
    lfoot_des[1] = gait::RobotParaClass_HALF_HIP_WIDTH;

    comav_butterworth.setZero();
    theta_default.setZero();


    L_com.setZero();
    com_estkine.setZero();
    cop_estkine.setZero();
    theta_estkine.setZero();
    thetaa_estkine.setZero();
    Fr_estkine = 0;
    Fl_estkine = 0;
    comv_estkine.setZero();
    dob_result.setZero();
    J_ini_xx_est = 0;
    J_ini_yy_est = 0;
    thetav_estkine.setZero();




////////////////////// real-time mpc loop /////////////////////////	
    count_in_mpc_max = 1000;
    count_in_rt_loop = 0;

	joint2simulationx.position.resize(100);
	state_to_MPC.position.resize(25);
	state_feedback.setZero();
	slow_mpc_gait.setZero();   
	mpc_gait_flag = 0;
	mpc_gait_flag_old = 0;
	slow_mpc_gait_inte.setZero(); 
	
	fast_mpc_gait_flag = 0;
	fast_mpc_gait_flag_old = 0;
	fast_mpc_gait.setZero();	
	
	
	COM_in1.setZero(); 
	COM_in2.setZero(); 
	COMxyz_ref.setZero(); 
	COM_ref2.setZero(); 
	COM_in1(2) = COM_in2(2) = COMxyz_ref(2) = COM_ref2(2) = gait::RobotPara_Z_C; 

	COMv_ref.setZero(); 

	COMacc_in1.setZero();  
	COMacc_in2.setZero();  
	COMacc_ref.setZero();  
	COMaccv_ref.setZero();  
	COMacc_ref2.setZero();  

	FootL_in1.setZero();  
	FootL_in2.setZero();  
	FootL_ref.setZero();  
	FootL_ref2.setZero();  
	FootL_in1(1) = FootL_in2(1)= FootL_ref(1) = FootL_ref2(1) = gait::RobotParaClass_HALF_HIP_WIDTH; 
	FootLv_ref.setZero(); 

	FootR_in1.setZero();  
	FootR_in2.setZero();  
	FootR_ref.setZero();  
	FootR_in1(1) = FootR_in2(1)= FootR_ref(1) = FootR_ref2(1) = -gait::RobotParaClass_HALF_HIP_WIDTH; 
	FootRv_ref.setZero(); 

	zmp_in1.setZero(); zmp_in2.setZero(); zmpxyz_ref.setZero(); zmpv_ref.setZero(); zmp_ref2.setZero();
	dcm_in1.setZero(); dcm_in2.setZero(); dcmxyz_ref.setZero(); dcmv_ref.setZero(); dcm_ref2.setZero();

	body_in1.setZero(); body_in2.setZero(); body_ref.setZero(); bodyv_ref.setZero(); body_ref2;
	rfootrpy_in1.setZero(); rfootrpy_in2.setZero(); rfootrpy_ref.setZero(); rfootrpyv_ref.setZero(); rfootrpy_ref2.setZero();
	lfootrpy_in1.setZero(); lfootrpy_in2.setZero(); lfootrpy_ref.setZero(); lfootrpyv_ref.setZero(); lfootrpy_ref2.setZero();



	PelvisPos.setZero();  
	body_thetax.setZero();  
	LeftFootPosx.setZero(); 
	RightFootPosx.setZero();  
	dcmxyz_ref.setZero();  
	F_L.setZero();  
	F_R.setZero(); 
	M_L.setZero();  
	M_R.setZero(); 
	LeftFootRPY.setZero();  
	RightFootRPY.setZero(); 
	
	rpy_mpc_body.setZero();
	rpy_mpc_body(2) = COM_ref2(2);

	rfoot_inter.setZero();
	rfoot_inter(1) = FootR_ref2(1);

	lfoot_inter.setZero();
	lfoot_inter(1) = FootL_ref2(1);

	bodytheta_inter.setZero();

	rftheta_inter.setZero(); 
	lftheta_inter.setZero();

	zmp_inter.setZero(); 
	dcm_inter.setZero();

	zmp_mpc_ref.setZero(); rfoot_mpc_ref.setZero(); lfoot_mpc_ref.setZero(); bodyangle_mpc_ref.setZero(); comacc_mpc_ref.setZero();
	bodyangle_mpc.setZero();
	bodyangle_state.setZero();

	count_in_rt_loop = 0;
	count_in_rt_mpc = 0;

	count_inteplotation = 0;
	count_inteplotation_fast = 0;
	t_int = 0;
	dtx = gait::t_program_cyclic;   
	n_t_int = (int) round(gait::dt_mpc_slow /dtx); 
	n_t_int_fast = (int) round(gait::dt_mpc_fast /dtx); 


	_mass = gait::mass;
	_j_ini = gait::J_ini;

	_Zsc = 0;
	_ggg = gait::g;	
	
	//// gait filter
    f_sample_comx1 = 1/gait::t_program_cyclic;
	fcutoff_comx1 = 4;
	fcutoff_comx2 = 5;
	butterworthLPF1.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF2.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF3.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF4.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF5.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF6.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF7.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF8.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF9.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF10.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF11.init(f_sample_comx1,fcutoff_comx2);	
	butterworthLPF12.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF13.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF14.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF15.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF16.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF17.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF18.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF19.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF20.init(f_sample_comx1,fcutoff_comx1);
	butterworthLPF21.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF22.init(f_sample_comx1,fcutoff_comx1);
	
	fcutoff_comx3 = 5;
	butterworthLPF23.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF24.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF25.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF26.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF27.init(f_sample_comx1,fcutoff_comx3);	
	butterworthLPF28.init(f_sample_comx1,fcutoff_comx3);	
/*	butterworthLPF29.init(f_sample_comx1,fcutoff_comx1);	
	butterworthLPF30.init(f_sample_comx1,fcutoff_comx1);*/	
    
    pitch_angle_W = 0;

    n_period = round(gait::t_period / gait::t_program_cyclic); 




  
    multiThread listen_publish_obj(robot_name);
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    usleep(300000); // must wait 300ms, to get first state

    ros::NodeHandle n;
    ros::Publisher lowState_pub; //for rviz visualization
    ros::Publisher gait_data_pub; // for data_analysis
    ros::Publisher control_to_rtmpc_pub_; /// state feedback to rtmpc
    
    rt_frequency = 1000; /// frequency of lower_level control
    time_programming = 1.0/rt_frequency;
    ros::Rate loop_rate(rt_frequency);

    joint2simulation.position.resize(100);
    n_count = 0;
    stand_duration = 5; /// stand up: 2s
    
    


   

    // the following nodes have been initialized by "gazebo.launch"
    lowState_pub = n.advertise<unitree_legged_msgs::LowState>("/" + robot_name + "_gazebo/lowState/state", 1);
    servo_pub[0] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = n.advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_calf_controller/command", 1);
    gait_data_pub = n.advertise<sensor_msgs::JointState>("go1_gait_data",10);
    control_to_rtmpc_pub_ = n.advertise<sensor_msgs::JointState>("/control2rtmpc/state", 10);
    ///// a must : parameter initailization for robot control
    paramInit();

    while (ros::ok()){
        // publisher measured state:
        lowState_pub.publish(lowState);/// for data analysis
        
        ////////////////////////// state estimation ///////////////////////////////////////////////

        // Forward kinematics: for kinematic-based state estimation
        FR_angle_mea(0,0) = lowState.motorState[0].q;
        FR_angle_mea(1,0) = lowState.motorState[1].q;
        FR_angle_mea(2,0) = lowState.motorState[2].q;
        FL_angle_mea(0,0) = lowState.motorState[3].q;
        FL_angle_mea(1,0) = lowState.motorState[4].q;
        FL_angle_mea(2,0) = lowState.motorState[5].q;
        RR_angle_mea(0,0) = lowState.motorState[6].q;
        RR_angle_mea(1,0) = lowState.motorState[7].q;
        RR_angle_mea(2,0) = lowState.motorState[8].q;
        RL_angle_mea(0,0) = lowState.motorState[9].q;
        RL_angle_mea(1,0) = lowState.motorState[10].q;
        RL_angle_mea(2,0) = lowState.motorState[11].q;
        
        /// relative to body center
        FR_foot_relative_mea = Kine.Forward_kinematics(FR_angle_mea, 0);
        FL_foot_relative_mea = Kine.Forward_kinematics(FL_angle_mea, 1);
        RR_foot_relative_mea = Kine.Forward_kinematics(RR_angle_mea, 2);
        RL_foot_relative_mea = Kine.Forward_kinematics(RL_angle_mea, 3);


        for (int jx = 0; jx<25; jx++)
        {
        state_to_MPC.position[jx] = state_feedback(jx,0);
        }        

        control_to_rtmpc_pub_.publish(state_to_MPC); //// state feedback to MPC; 1+5*p,v,a+3*3*position;
        
        


        //////////////////// gait control loop/////////////////////////////////////////////////////
        /// *****************joint cmd generation*******************////
        double targetPos[12]  = {0, 0.87, -1.5, 0, 0.87, -1.5, 0, 0.87, -1.5, 0, 0.87, -1.5};  
        // reference angle generation: a simple test
        if (n_count*time_programming <= stand_duration) 
        {
            //****************Homing_pose*******************
            for(int j=0; j<12; j++) lastPos[j] = lowState.motorState[j].q;

            percent = pow((n_count*time_programming)/stand_duration,2);
            for(int j=0; j<12; j++){
                lowCmd.motorCmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent; 
            }

            FR_angle_des(0,0) = lowCmd.motorCmd[0].q;
            FR_angle_des(1,0) = lowCmd.motorCmd[1].q;
            FR_angle_des(2,0) = lowCmd.motorCmd[2].q;
            FL_angle_des(0,0) = lowCmd.motorCmd[3].q;
            FL_angle_des(1,0) = lowCmd.motorCmd[4].q;
            FL_angle_des(2,0) = lowCmd.motorCmd[5].q;
            RR_angle_des(0,0) = lowCmd.motorCmd[6].q;
            RR_angle_des(1,0) = lowCmd.motorCmd[7].q;
            RR_angle_des(2,0) = lowCmd.motorCmd[8].q;
            RL_angle_des(0,0) = lowCmd.motorCmd[9].q;
            RL_angle_des(1,0) = lowCmd.motorCmd[10].q;
            RL_angle_des(2,0) = lowCmd.motorCmd[11].q;   

            FR_foot_des = FR_foot_relative_des = Kine.Forward_kinematics(FR_angle_des, 0);
            FL_foot_des = FL_foot_relative_des = Kine.Forward_kinematics(FL_angle_des, 1);
            RR_foot_des = RR_foot_relative_des = Kine.Forward_kinematics(RR_angle_des, 2);
            RL_foot_des = RL_foot_relative_des = Kine.Forward_kinematics(RL_angle_des, 3);  
            
            /// computing the homing body position:
            body_p_Homing(2,0) = body_p_des(2,0) = - (FR_foot_relative_des(2,0) + FL_foot_relative_des(2,0) + RR_foot_relative_des(2,0) + RL_foot_relative_des(2,0))/4;                   
            FR_foot_des(2) = 0;
            FL_foot_des(2) = 0;
            RR_foot_des(2) = 0;
            RL_foot_des(2) = 0;

            FR_foot_Homing = FR_foot_des;
            FL_foot_Homing = FL_foot_des;
            RR_foot_Homing = RR_foot_des;
            RL_foot_Homing = RL_foot_des;
        }
        else   
        {
             ///****************squat_down: only for test*******************
/*            percent = 0.2 * (cos(n_count*time_programming - stand_duration) + 4);
            for(int j=0; j<12; j++){
                lowCmd.motorCmd[j].q = lastPos[j]*percent; 
            }

            // Forward kinematics_test:
            FR_angle_des(0,0) = lowCmd.motorCmd[0].q;
            FR_angle_des(1,0) = lowCmd.motorCmd[1].q;
            FR_angle_des(2,0) = lowCmd.motorCmd[2].q;
            FL_angle_des(0,0) = lowCmd.motorCmd[3].q;
            FL_angle_des(1,0) = lowCmd.motorCmd[4].q;
            FL_angle_des(2,0) = lowCmd.motorCmd[5].q;
            RR_angle_des(0,0) = lowCmd.motorCmd[6].q;
            RR_angle_des(1,0) = lowCmd.motorCmd[7].q;
            RR_angle_des(2,0) = lowCmd.motorCmd[8].q;
            RL_angle_des(0,0) = lowCmd.motorCmd[9].q;
            RL_angle_des(1,0) = lowCmd.motorCmd[10].q;
            RL_angle_des(2,0) = lowCmd.motorCmd[11].q;

            FR_foot_relative_des = Kine.Forward_kinematics(FR_angle_des, 0);
            FL_foot_relative_des = Kine.Forward_kinematics(FL_angle_des, 1);
            RR_foot_relative_des = Kine.Forward_kinematics(RR_angle_des, 2);
            RL_foot_relative_des = Kine.Forward_kinematics(RL_angle_des, 3); */

            ///// locally inverse kinematics: local kinematics: using the reference trajectory generated by the FK;
/*             Eigen::Matrix<double,3,1>  q_ini;
            q_ini(0,0) = lastPos[0];
            q_ini(1,0) = lastPos[1];
            q_ini(2,0) = lastPos[2];
            FR_angle_des = Kine.Inverse_kinematics(FR_foot_relative_des,q_ini,0);

            q_ini(0,0) = lastPos[3];
            q_ini(1,0) = lastPos[4];
            q_ini(2,0) = lastPos[5];
            FL_angle_des = Kine.Inverse_kinematics(FL_foot_relative_des,q_ini,1);

            q_ini(0,0) = lastPos[6];
            q_ini(1,0) = lastPos[7];
            q_ini(2,0) = lastPos[8]; 
            RR_angle_des = Kine.Inverse_kinematics(RR_foot_relative_des,q_ini,2);

            q_ini(0,0) = lastPos[9];
            q_ini(1,0) = lastPos[10];
            q_ini(2,0) = lastPos[11];
            RL_angle_des = Kine.Inverse_kinematics(RL_foot_relative_des,q_ini,3); */

            /// hand tuned gait: global inverse kinematics test://///////////////////
            // ******************** body movement test *********************////
            /* body_p_des(0,0) = body_p_Homing(0,0) + (0.01 * (sin(n_count*time_programming - stand_duration)));
            body_p_des(1,0) = (0.1 * (cos(n_count*time_programming - stand_duration + PI/2)));
            body_r_des(0,0) = (0.15 * (sin(n_count*time_programming - stand_duration)));
            body_r_des(1,0) = (0.1 * (sin(n_count*time_programming - stand_duration)));  */
            

            // //  // ******************** troting: just a quick test *********************////
            // int nT = 400;   //// 0.001*500=0.5s;
            // int n_count_y;
            // n_count_y = n_count - (int)stand_duration/time_programming;

            // if (( n_count_y % nT) == nT/2)
            // {
            //     cout<< "body_p_des_hight:"<<body_p_Homing(2)<<endl;
            //     cout<< "leg_distance_x:"<< FR_foot_relative_des(0,0) - RR_foot_relative_des(0,0)<<endl;
            //     cout<< "leg_distance_y:"<< RL_foot_relative_des(1,0) - RR_foot_relative_des(1,0)<<endl;
            // }
            
            // if ( ( n_count_y % nT) <= nT/2 ) // first half period
            // {
            //     FR_foot_des(2,0) = RL_foot_des(2,0) = 0.04 * sin((n_count_y % nT) / ((nT/2.0))  * PI);
            // }
            // else             ///later half period
            // {
            //     FL_foot_des(2,0) = RR_foot_des(2,0) = 0.04 * sin(((n_count_y % nT) - (nT/2)) / ((nT/2.0))  * PI);
            // }
            
            
            ///////////////////////////////////// MPC gait planner test //////////////////////////
            /////// rt loop counter 
            count_in_rt_loop += 1;  
            t_int = ((int) round(count_in_rt_loop/n_t_int));
            state_feedback(0,0) = t_int;

            
            //// non -real-time intepoloation: data filter: from hierachical convex optimization:

            com_des[0] = butterworthLPF1.filter(fast_mpc_gait(0,0));
            com_des[1] = butterworthLPF2.filter(fast_mpc_gait(1,0));
            com_des[2] = butterworthLPF3.filter(fast_mpc_gait(2,0));
            
            theta_des[0] = 0;
            theta_des[1] = 0;	    
            theta_des[2] = 0;            
            // theta_des[0] = butterworthLPF4.filter(fast_mpc_gait(36));
            // theta_des[1] = butterworthLPF5.filter(fast_mpc_gait(37));	    
            // theta_des[2] = 0;
            
            rfoot_theta_des[0] = butterworthLPF12.filter(fast_mpc_gait(31));
            rfoot_theta_des[1] = butterworthLPF13.filter(fast_mpc_gait(32));
            rfoot_theta_des[2] = butterworthLPF14.filter(fast_mpc_gait(33));		
            lfoot_theta_des[0] = butterworthLPF15.filter(fast_mpc_gait(28));
            lfoot_theta_des[1] = butterworthLPF16.filter(fast_mpc_gait(29));
            lfoot_theta_des[2] = butterworthLPF17.filter(fast_mpc_gait(30));		    
            
            rfoot_des[0] = butterworthLPF6.filter(fast_mpc_gait(9));
            rfoot_des[1] = butterworthLPF7.filter(fast_mpc_gait(10));
            rfoot_des[2] = butterworthLPF8.filter(fast_mpc_gait(11));

            
            lfoot_des[0] = butterworthLPF9.filter(fast_mpc_gait(6));
            lfoot_des[1] = butterworthLPF10.filter(fast_mpc_gait(7));
            lfoot_des[2] = butterworthLPF11.filter(fast_mpc_gait(8));
    
            zmp_ref[0] = butterworthLPF18.filter(fast_mpc_gait(12));
            zmp_ref[1] = butterworthLPF19.filter(fast_mpc_gait(13));  		    
            dcm_ref[0] = butterworthLPF20.filter(fast_mpc_gait(34));
            dcm_ref[1] = butterworthLPF21.filter(fast_mpc_gait(35));
            
            if (count_in_rt_loop * gait::t_program_cyclic >= 0.6)
            {
                switch (gait_mode)
                {
                case 101:  ////biped walking
                    body_p_des[0] = com_des[0];
                    body_p_des[1] = com_des[1] * y_offset;
                    body_p_des[2] = com_des[2];

                    body_r_des[0] = theta_des[0];
                    body_r_des[1] = theta_des[1];
                    body_r_des[2] = theta_des[2];

                    //// right two legs move synchronous
                    FR_foot_des[0] = FR_foot_Homing[0] + rfoot_des[0];
                    FR_foot_des[1] = FR_foot_Homing[1] + rfoot_des[1] + gait::RobotParaClass_HALF_HIP_WIDTH;
                    FR_foot_des[2] = FR_foot_Homing[2] + rfoot_des[2];
                    RR_foot_des[0] = RR_foot_Homing[0] + rfoot_des[0];
                    RR_foot_des[1] = RR_foot_Homing[1] + rfoot_des[1] + gait::RobotParaClass_HALF_HIP_WIDTH;
                    RR_foot_des[2] = RR_foot_Homing[2] + rfoot_des[2];

                    //// left two legs move synchronous
                    FL_foot_des[0] = FL_foot_Homing[0] + lfoot_des[0];
                    FL_foot_des[1] = FL_foot_Homing[1] + lfoot_des[1] - gait::RobotParaClass_HALF_HIP_WIDTH;
                    FL_foot_des[2] = FL_foot_Homing[2] + lfoot_des[2];
                    RL_foot_des[0] = RL_foot_Homing[0] + lfoot_des[0];
                    RL_foot_des[1] = RL_foot_Homing[1] + lfoot_des[1] - gait::RobotParaClass_HALF_HIP_WIDTH;
                    RL_foot_des[2] = RL_foot_Homing[2] + lfoot_des[2];


                    break;
                case 102:  ///troting
                    body_p_des[0] = com_des[0];
                    body_p_des[1] = com_des[1] * 0.15;
                    body_p_des[2] = com_des[2];

                    body_r_des[0] = theta_des[0];
                    body_r_des[1] = theta_des[1];
                    body_r_des[2] = theta_des[2];

                    //// FR, RL two legs move synchronous
                    FR_foot_des[0] = FR_foot_Homing[0] + rfoot_des[0];
                    FR_foot_des[1] = FR_foot_Homing[1] + rfoot_des[1] + gait::RobotParaClass_HALF_HIP_WIDTH;
                    FR_foot_des[2] = FR_foot_Homing[2] + rfoot_des[2];
                    RL_foot_des[0] = RL_foot_Homing[0] + rfoot_des[0];
                    RL_foot_des[1] = RL_foot_Homing[1] + rfoot_des[1] + gait::RobotParaClass_HALF_HIP_WIDTH;
                    RL_foot_des[2] = RL_foot_Homing[2] + rfoot_des[2];

                    //// FL, RR two legs move synchronous
                    FL_foot_des[0] = FL_foot_Homing[0] + lfoot_des[0];
                    FL_foot_des[1] = FL_foot_Homing[1] + lfoot_des[1] - gait::RobotParaClass_HALF_HIP_WIDTH;
                    FL_foot_des[2] = FL_foot_Homing[2] + lfoot_des[2];
                    RR_foot_des[0] = RR_foot_Homing[0] + lfoot_des[0];
                    RR_foot_des[1] = RR_foot_Homing[1] + lfoot_des[1] - gait::RobotParaClass_HALF_HIP_WIDTH;
                    RR_foot_des[2] = RR_foot_Homing[2] + lfoot_des[2];                


                    break;
                case 103:  ///gallop: alter the  x-y direction
                    body_p_des[0] = com_des[0];
                    body_p_des[1] = com_des[1] * 0.1;
                    body_p_des[2] = com_des[2];
                    
                    // pitch angle generation //////
                    
                    pitch_angle_W = 2 * gait::pi / (2 * gait::t_period);

                    body_r_des[0] = theta_des[0];
                    ///// 
                    if (count_in_rt_loop - 3* n_period <= 0)
                    {
                        body_r_des[0] = theta_des[1];
                    }
                    else
                    {
                      body_r_des[1] = -(0.1 * (sin(pitch_angle_W * (count_in_rt_loop - 3* n_period) * gait::t_program_cyclic))); /// pitch angle//
                    }
                    
                    body_r_des[2] = theta_des[2];                    

                    //// fore two legs move synchronous
                    FR_foot_des[0] = FR_foot_Homing[0] + rfoot_des[0];
                    FR_foot_des[1] = FR_foot_Homing[1] + rfoot_des[1] + gait::RobotParaClass_HALF_HIP_WIDTH;
                    FR_foot_des[2] = FR_foot_Homing[2] + rfoot_des[2];
                    FL_foot_des[0] = FL_foot_Homing[0] + rfoot_des[0];
                    FL_foot_des[1] = FL_foot_Homing[1] + rfoot_des[1] + gait::RobotParaClass_HALF_HIP_WIDTH;
                    FL_foot_des[2] = FL_foot_Homing[2] + rfoot_des[2];

                    //// rear two legs move synchronous
                    RR_foot_des[0] = RR_foot_Homing[0] + lfoot_des[0];
                    RR_foot_des[1] = RR_foot_Homing[1] + lfoot_des[1] - gait::RobotParaClass_HALF_HIP_WIDTH;
                    RR_foot_des[2] = RR_foot_Homing[2] + lfoot_des[2];             
                    RL_foot_des[0] = RL_foot_Homing[0] + lfoot_des[0];
                    RL_foot_des[1] = RL_foot_Homing[1] + lfoot_des[1] - gait::RobotParaClass_HALF_HIP_WIDTH;
                    RL_foot_des[2] = RL_foot_Homing[2] + lfoot_des[2];  

                    break;            
                default:
                    break;
                }
            }


            
           
            Eigen::Matrix<double,3,1>  q_ini;
            q_ini = FR_angle_des;
            FR_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,FR_foot_des,q_ini,0);
            // FR_angle_des = Kine.Inverse_kinematics_optimisation(FR_foot_des,q_ini,0);

            q_ini = FL_angle_des;
            FL_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,FL_foot_des,q_ini,1);
            // FL_angle_des = Kine.Inverse_kinematics_optimisation(FL_foot_des,q_ini,1);

            q_ini = RR_angle_des;
            RR_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,RR_foot_des,q_ini,2);
            // RR_angle_des = Kine.Inverse_kinematics_optimisation(RR_foot_des,q_ini,2);

            q_ini = RL_angle_des;
            RL_angle_des = Kine.Inverse_kinematics_g(body_p_des,body_r_des,RL_foot_des,q_ini,3);    
            // RL_angle_des = Kine.Inverse_kinematics_optimisation(RL_foot_des,q_ini,3);       

            
             // desired angle: generatee by inverse kinematics
            lowCmd.motorCmd[0].q = FR_angle_des(0,0);
            lowCmd.motorCmd[1].q = FR_angle_des(1,0);
            lowCmd.motorCmd[2].q = FR_angle_des(2,0);
            lowCmd.motorCmd[3].q = FL_angle_des(0,0);
            lowCmd.motorCmd[4].q = FL_angle_des(1,0);
            lowCmd.motorCmd[5].q = FL_angle_des(2,0);
            lowCmd.motorCmd[6].q = RR_angle_des(0,0);
            lowCmd.motorCmd[7].q = RR_angle_des(1,0);
            lowCmd.motorCmd[8].q = RR_angle_des(2,0); 
            lowCmd.motorCmd[9].q = RL_angle_des(0,0);
            lowCmd.motorCmd[10].q = RL_angle_des(1,0);
            lowCmd.motorCmd[11].q = RL_angle_des(2,0);         
        }

        ////////////////////////////////////////////////////////////////////////
        // joint command pub to servors:
        for(int m=0; m<12; m++){
            servo_pub[m].publish(lowCmd.motorCmd[m]);
        } 

         ///********************* data saving ************************************///////
        joint2simulation.header.stamp = ros::Time::now();
        for(int j=0; j<12; j++){
                joint2simulation.position[j] = lowCmd.motorCmd[j].q; // desired joint angles; 
        }
        for(int j=0; j<12; j++)
        {
            joint2simulation.position[12+j] = lowState.motorState[j].q;   // measured joint angles;
        } 

        for(int j=0; j<3; j++)
        {
            joint2simulation.position[24+j] = FR_foot_des(j,0);   // desired FR Leg position;
        }         
        for(int j=0; j<3; j++)
        {
            joint2simulation.position[27+j] = FL_foot_des(j,0);   // desired position;
        }
        for(int j=0; j<3; j++)
        {
            joint2simulation.position[30+j] = RR_foot_des(j,0);   // desired position;
        }
        for(int j=0; j<3; j++)
        {
            joint2simulation.position[33+j] = RL_foot_des(j,0);   // desired position;
        } 

        for(int j=0; j<3; j++)
        {
            joint2simulation.position[36+j] = FR_foot_relative_mea(j,0) + body_p_des(j,0);   // measured position;
        }         
        for(int j=0; j<3; j++)
        {
            joint2simulation.position[39+j] = FL_foot_relative_mea(j,0) + body_p_des(j,0);   // measured position;
        }
        for(int j=0; j<3; j++)
        {
            joint2simulation.position[42+j] = RR_foot_relative_mea(j,0) + body_p_des(j,0);   // measured position;
        }
        for(int j=0; j<3; j++)
        {
            joint2simulation.position[45+j] = RL_foot_relative_mea(j,0) + body_p_des(j,0);   // measured position;
        } 
        
        /// body_pos_des
        for(int j=0; j<3; j++)
        {
            joint2simulation.position[48+j] = body_p_des(j,0);   // desired body position;
        }
        /// body_R_des
        for(int j=0; j<3; j++)
        {
            joint2simulation.position[51+j] = body_r_des(j,0);   // desired body ori;
        }                                   


        gait_data_pub.publish(joint2simulation);

        
        
        ros::spinOnce();

        loop_rate.sleep();
        n_count++;   
        
    }
    
    return 0;
}
