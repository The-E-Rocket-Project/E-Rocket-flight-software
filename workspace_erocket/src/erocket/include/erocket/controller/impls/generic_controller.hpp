#pragma once
#include <rclcpp/rclcpp.hpp>
#include <erocket/constants.hpp>
#include <erocket/frame_transforms.h>

#include <erocket/controller/state.hpp>
#include <erocket/controller/setpoint.hpp>
#include <erocket/controller/allocator.hpp>
#include <erocket/controller/impls/attitude_pid.hpp>
#include <erocket/vehicle_constants.hpp>
#include <erocket/msg/generic_controller_debug.hpp>
#include <cmath>
#include "attd_fun.h"

using namespace erocket::frame_transforms;
using namespace erocket::msg;
using namespace erocket::constants::controller_generic;
using namespace std;

class GenericController
{
public:
    GenericController(
        rclcpp::Node* node, 
        rclcpp::QoS qos, 
        std::shared_ptr<StateAggregator> state_aggregator, 
        std::shared_ptr<SetpointAggregator> setpoint_aggregator,
        std::shared_ptr<VehicleConstants> vehicle_constants
    ): 
        state_aggregator_(state_aggregator),
        setpoint_aggregator_(setpoint_aggregator),
        vehicle_constants_(vehicle_constants),
        attitude_controller_{node, qos, state_aggregator_, setpoint_aggregator_, vehicle_constants_},
        debug_publisher_{node->create_publisher<GenericControllerDebug>("generic_controller/debug", qos)},
        clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
    {
        // Get parameters from yaml file
        node->declare_parameter<double>(CONTROLLER_GENERIC_FREQUENCY_HERTZ_PARAM);
        double controller_freq = node->get_parameter(CONTROLLER_GENERIC_FREQUENCY_HERTZ_PARAM).as_double();
        dt_ = 1.0 / controller_freq;

        node->declare_parameter<std::vector<double>>(CONTROLLER_GENERIC_KP_PARAM);
        node->declare_parameter<std::vector<double>>(CONTROLLER_GENERIC_KV_PARAM);
        node->declare_parameter<std::vector<double>>(CONTROLLER_GENERIC_LAMBDA1_PARAM);
        node->declare_parameter<std::vector<double>>(CONTROLLER_GENERIC_LAMBDA2_PARAM);
        node->declare_parameter<std::vector<double>>(CONTROLLER_GENERIC_LAMBDA3_PARAM);

        Eigen::Vector3d kp_diag = Eigen::Map<const Eigen::Vector3d>(node->get_parameter(CONTROLLER_GENERIC_KP_PARAM).as_double_array().data(), 3);
        Eigen::Vector3d kv_diag = Eigen::Map<const Eigen::Vector3d>(node->get_parameter(CONTROLLER_GENERIC_KV_PARAM).as_double_array().data(), 3);
        Eigen::Vector3d Lambda1_diag = Eigen::Map<const Eigen::Vector3d>(node->get_parameter(CONTROLLER_GENERIC_LAMBDA1_PARAM).as_double_array().data(), 3);
        Eigen::Vector3d Lambda2_diag = Eigen::Map<const Eigen::Vector3d>(node->get_parameter(CONTROLLER_GENERIC_LAMBDA2_PARAM).as_double_array().data(), 3);
        Eigen::Vector3d Lambda3_diag = Eigen::Map<const Eigen::Vector3d>(node->get_parameter(CONTROLLER_GENERIC_LAMBDA3_PARAM).as_double_array().data(), 3);

        Kp_ = kp_diag.asDiagonal();
        Kv_ = kv_diag.asDiagonal();
        Lambda1_= Lambda1_diag.asDiagonal();
        Lambda2_ = Lambda2_diag.asDiagonal();
        Lambda3_ = Lambda3_diag.asDiagonal();

        node->declare_parameter<double>(CONTROLLER_GENERIC_KR_PARAM);
        kr_ = node->get_parameter(CONTROLLER_GENERIC_KR_PARAM).as_double();
        node->declare_parameter<double>(CONTROLLER_GENERIC_HR_PARAM);
        hr_ = node->get_parameter(CONTROLLER_GENERIC_HR_PARAM).as_double();
        node->declare_parameter<double>(CONTROLLER_GENERIC_KW_PARAM);
        kw_ = node->get_parameter(CONTROLLER_GENERIC_KW_PARAM).as_double();
        node->declare_parameter<double>(CONTROLLER_GENERIC_HW_PARAM);
        hw_ = node->get_parameter(CONTROLLER_GENERIC_HW_PARAM).as_double();
    }

    /*
        * @brief Compute the control input based on the PID controller formula
        * @return The computed control input
    */
    AllocatorInput compute() {
        auto state = state_aggregator_->get_state();
        auto sp = setpoint_aggregator_->get_position_setpoint();
        auto att_sp = setpoint_aggregator_->get_attitude_setpoint();

        Eigen::Vector3d pd       = sp.position;        // pd   
        Eigen::Vector3d pd_dot   = sp.velocity;       // pd_dot
        Eigen::Vector3d pd_2dot  = sp.acceleration;   // pd_2dot
        Eigen::Vector3d pd_3dot  = sp.jerk;           // pd_3dot
        Eigen::Vector3d pd_4dot  = sp.snap;           // pd_4dot

        double yaw_sp = att_sp.attitude[2];

        Eigen::Vector3d p        = state.position;    // p
        Eigen::Vector3d v        = state.velocity;    // v
        Eigen::Matrix3d R        = state.rotation_matrix; // R
        Eigen::Vector3d euler_angles = state.euler_angles; // R
        Eigen::Vector3d om       = state.angular_rate;  // om

        /*
            0.1. rotate necessary vectors to controller frame
            0.2. obtain rotation matrix in controller frame
            0.3. extract euler angles from rotation matrix and give to controller
        */

        // static rotation matrix platform -> controller
        // equal in inertial and body frames
        Eigen::Matrix3d Omega_cp;
        Omega_cp.col(0) = Eigen::Vector3d(0.0, 0.0, -1.0); // controller x -> platform -z
        Omega_cp.col(1) = Eigen::Vector3d(-1.0, 0.0, 0.0); // controller y -> platform -x
        Omega_cp.col(2) = Eigen::Vector3d(0.0, 1.0, 0.0);  // controller z -> platform y

        // rotation matrix in controller frame
        Eigen::Matrix3d R_c;
        R_c = Omega_cp.transpose()* R * Omega_cp;

        double mass = vehicle_constants_->mass_of_system_;
        double ell = vehicle_constants_->lever_arm_;
        double g = vehicle_constants_->gravitational_acceleration_;
        Eigen::Matrix3d J;
        double J_axial = 0.008;
        J.col(0) = Eigen::Vector3d(J_axial, 0.0, 0.0);
        J.col(1) = Eigen::Vector3d(0.0, vehicle_constants_->moment_of_inertia_, 0.0);
        J.col(2) = Eigen::Vector3d(0.0, 0.0, vehicle_constants_->moment_of_inertia_);
       
        // extract euler angles from R_c
        double psi = -std::asin(R_c(0,1));
        double th = std::atan2(R_c(0,2)/std::cos(psi), R_c(0,0)/std::cos(psi));
        double cphi = R_c(1,1)/std::cos(psi);
        double sphi = R_c(2,1)/std::cos(psi);

        Eigen::Vector3d p_c, dp_c;
        euler_c << cphi, sphi, th, psi;
        p_c = Omega_cp.transpose() * p;
        dp_c = Omega_cp.transpose() * v;
        omega_c = Omega_cp.transpose() * om;

        // trajectory in controller frame
        Eigen::Matrix<double, 3, 5> pd35_c; // pd, pd_dot, pd_2dot, pd_3dot, pd_4dot
        Eigen::Vector3d pd_c, dpd_c, ddpd_c, d3pd_c, d4pd_c;
        pd_c = Omega_cp.transpose() * pd;
        dpd_c = Omega_cp.transpose() * pd_dot;
        ddpd_c = Omega_cp.transpose() * pd_2dot;
        d3pd_c = Omega_cp.transpose() * pd_3dot;
        d4pd_c = Omega_cp.transpose() * pd_4dot;
        pd35_c.col(0) = pd_c;
        pd35_c.col(1) = dpd_c;
        pd35_c.col(2) = ddpd_c;
        pd35_c.col(3) = d3pd_c;
        pd35_c.col(4) = d4pd_c;

        ep_int(0) = ep_int(0) + (p_c(0)-pd_c(0))*dt_;
        ep_int(1) = ep_int(1) + (p_c(1)-pd_c(1))*dt_;
        ep_int(2) = ep_int(2) + (p_c(2)-pd_c(2))*dt_;

        /*
            1. use state and setpoint to compute tracking errors
            2. apply control equations, with gains from yaml
            3. populate output structs
        */
        // use Kp and Kv as inner loop PD controller gains
        // use Lambda1, Lambda2, Lambda3 as outer loop PID gains


        /*
        // pure integrator for Ts = 10 ms
        double n_int[3] = { 0.0050000000, 0.0050000000, 0.0000000000 }; // num
        double d_int[3] = { 1.0000000000, -1.0000000000, 0.0000000000 }; // den
        ep_int(0) = filter_stuff(n_int, d_int, u_x_, y_x_, p_c(0)-pd_c(0));
        ep_int(1) = filter_stuff(n_int, d_int, u_y_, y_y_, p_c(1)-pd_c(1));
        ep_int(2) = filter_stuff(n_int, d_int, u_z_, y_z_, p_c(2)-pd_c(2));
        */

        // use only 0:1 entries of lambda as outer loop gains
        Eigen::Matrix2d Lambda1_2d, Lambda2_2d, Lambda3_2d;
        Lambda1_2d = Lambda1_.topLeftCorner<2,2>();
        Lambda2_2d = Lambda2_.topLeftCorner<2,2>();
        Lambda3_2d = Lambda3_.topLeftCorner<2,2>();

        tvc_cmd = compute_inner_loop(p_c, dp_c, pd35_c, euler_c, omega_c, thd3, psid3, mass, J, ell, g, Kp_, Kv_, kw_, ep_int(0), Mx);

        // roll control
        phid3(0) = -yaw_sp; phid3(1) = 0.0; phid3(2) = 0.0;
        double cphid = std::cos(phid3(0));
        double sphid = std::sin(phid3(0));
        double e_phi = std::atan2((sphi*cphid - cphi*sphid), (cphi*cphid + sphi*sphid));
        // integrate roll error for roll control
        roll_int = roll_int + e_phi*dt_;
        Mx = compute_roll_control(euler_c, omega_c, phid3, roll_int, J, tvc_cmd, ell, kr_, hr_, hw_);


        //att_d = compute_outer_loop(p_c, dp_c, euler_c, pd35_c, ep_int, tvc_cmd, mass, Lambda1_2d, Lambda2_2d, Lambda3_2d);

        double phi_ddot, phi_dot, phi_v;
        double ey_dddot, ey_ddot, ey_dot, ey, ey_int_v;
        double ez_dddot, ez_ddot, ez_dot, ez, ez_int_v;
        double ddyd_ddot, ddyd_dot, ddyd_v, ddzd_ddot, ddzd_dot, ddzd_v;
        double u1;


        Eigen::Vector3d tvc_cmd_debug;
        tvc_cmd_debug(0) = tvc_cmd(0);
        tvc_cmd_debug(1) = 0.0;
        tvc_cmd_debug(2) = 0.0;
        convert_stuff(p_c, euler_c, dp_c, pd35_c, ep_int, tvc_cmd_debug, Mx, omega_c, mass, g, J, ell,
            phi_ddot, phi_dot, phi_v,
            ey_dddot, ey_ddot, ey_dot, ey, ey_int_v,
            ez_dddot, ez_ddot, ez_dot, ez, ez_int_v,
            ddyd_ddot, ddyd_dot, ddyd_v,
            ddzd_ddot, ddzd_dot, ddzd_v,
            u1);

        double out1_attd[6];

        attd_fun(phi_ddot, phi_dot, phi_v,
            ey_dddot, ey_ddot, ey_dot, ey, ey_int_v,
            ez_dddot, ez_ddot, ez_dot, ez, ez_int_v,
            ddyd_ddot, ddyd_dot, ddyd_v,
            ddzd_ddot, ddzd_dot, ddzd_v,
            mass,
            u1,
            Lambda1_2d(0,0), Lambda1_2d(1,1),
            Lambda2_2d(0,0), Lambda2_2d(1,1),
            Lambda3_2d(0,0), Lambda3_2d(1,1),
            out1_attd);

        thd3(0)  = out1_attd[0];
        psid3(0) = out1_attd[1];
        thd3(1)  = out1_attd[2];
        psid3(1) = out1_attd[3];
        thd3(2)  = out1_attd[4];
        psid3(2) = out1_attd[5];



        
        ////////////////////////////////////////////////////////////////////////////7
        // APPROACH 1: simple derivative with low pass filter
        /*
        // :: derivator + low pass w/ wc = 2.50 rad/s and xi = 0.80:
        double n[] = { 0.0306325624, 0.0000000000, -0.0306325624 }; // num
        double d[] = { 1.0000000000, -1.9601776689, 0.9607903201 }; // den

        double dthd = 0.0; double dpsid = 0.0;
        double ddthd = 0.0; double ddpsid = 0.0;
        // eventually, consider the  derivation!!!
        dthd = filter_stuff(n,d,u_thd_,y_thd_,att_d(0));
        dpsid = filter_stuff(n,d,u_psid_,y_psid_,att_d(1));
        ddthd = filter_stuff(n,d,u_thd2_,y_thd2_,dthd);
        ddpsid = filter_stuff(n,d,u_psid2_,y_psid2_,dpsid);
        */
        ////////////////////////////////////////////////////////////////////////////7
        // APPROACH 2: second-order low-pass observer
/*       double wc = kr_; // cutoff frequency
        double xi = hr_; // damping ratio

        double ddthd = wc*wc*(att_d(0) - thd3(0)) - 2.0*xi*wc*thd3(1);
        thd3(1) = thd3(1) + ddthd*dt_;
        thd3(0) = thd3(0) + thd3(1)*dt_;
        thd3(2) = ddthd*0.0;
        //thd3(0) = att_d(0); // for debug

        double ddpsid = wc*wc*(att_d(1) - psid3(0)) - 2.0*xi*wc*psid3(1);
        psid3(1) = psid3(1) + ddpsid*dt_;
        psid3(0) = psid3(0) + psid3(1)*dt_;
        psid3(2) = ddpsid*0.0; */
        //psid3(0) = att_d(1); // for debug */



        // for debug
        //thd3(2) = 0;
        //psid3(2) = 0;

        // prepare TVC for output: rotate back to platform frame
        //Eigen::Vector3d u;
        u = Omega_cp * tvc_cmd;

        // --- Map u into your allocator outputs ---
        // NOTE: this mapping is vehicle-specific. The Matlab returned u is [u1 u2 u3]'

        // double u3 = 1.0; // dummy yaw input
        // auto attitude_output = attitude_controller_.compute(u3); // <<-- replace with your own yaw controller. u3 does not matter in this case :D

        double tau_yaw, tau_delta_bar;
        tau_yaw = Mx* (u(2)/u.norm());
        tau_delta_bar = tau_yaw/J_axial; 

        output_.thrust_vector = u;
        //output_.tau_delta_bar = attitude_output.tau_delta_bar; // <<-- replace with your own yaw controller
        output_.tau_delta_bar = tau_delta_bar;
        

        publish_debug();
        return output_;
    }

private:
    std::shared_ptr<StateAggregator> state_aggregator_;
    std::shared_ptr<SetpointAggregator> setpoint_aggregator_;
    std::shared_ptr<VehicleConstants> vehicle_constants_;

    AttitudePIDController attitude_controller_;
    rclcpp::Publisher<GenericControllerDebug>::SharedPtr debug_publisher_;

    rclcpp::Clock::SharedPtr clock_;
    double dt_;
    AllocatorInput output_;

    // Gains and matrices (mirror MATLAB types)
    Eigen::Matrix3d Kp_ = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d Kv_ = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d Lambda1_ = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d Lambda2_ = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d Lambda3_ = Eigen::Matrix3d::Zero();

    // variables
    Eigen::Vector3d u = Eigen::Vector3d::Zero();
    Eigen::Vector3d ep = Eigen::Vector3d::Zero();  
    Eigen::Vector3d ev = Eigen::Vector3d::Zero();  
    Eigen::Vector3d er = Eigen::Vector3d::Zero();    
    Eigen::Vector3d ew = Eigen::Vector3d::Zero();  

    // vehicle params
    double hr_;
    double kr_;
    double kw_;
    double hw_;

    Eigen::Vector4d euler_c = Eigen::Vector4d::Zero();
    Eigen::Vector3d omega_c = Eigen::Vector3d::Zero();
    Eigen::Vector2d att_d   = Eigen::Vector2d::Zero();
    Eigen::Vector3d tvc_cmd = Eigen::Vector3d::Zero();
    Eigen::Vector3d phid3  = Eigen::Vector3d::Zero();
    Eigen::Vector3d thd3   = Eigen::Vector3d::Zero();
    Eigen::Vector3d psid3  = Eigen::Vector3d::Zero();
    Eigen::Vector3d ep_int = Eigen::Vector3d::Zero(); // integral of position error
    double Mx = 0.0;
    double roll_int = 0.0;

    // filter states for derivation
    double u_x_[3] = {0.0,0.0,0.0};
    double y_x_[3] = {0.0,0.0,0.0};

    double u_y_[3] = {0.0,0.0,0.0};
    double y_y_[3] = {0.0,0.0,0.0};

    double u_z_[3] = {0.0,0.0,0.0};
    double y_z_[3] = {0.0,0.0,0.0};

    double u_thd_[3] = {0.0,0.0,0.0}; // pitch input history
    double y_thd_[3] = {0.0,0.0,0.0}; // pitch derivative output history
    double u_psid_[3] = {0.0,0.0,0.0}; // yaw input history
    double y_psid_[3] = {0.0,0.0,0.0}; // yaw derivative output history

    double u_thd2_[3] = {0.0,0.0,0.0}; // pitch derivative input history (for acceleration)
    double y_thd2_[3] = {0.0,0.0,0.0}; // pitch acceleration output history
    double u_psid2_[3] = {0.0,0.0,0.0}; // yaw derivative input history (for acceleration)
    double y_psid2_[3] = {0.0,0.0,0.0}; // yaw acceleration output history


    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Logger logger_ = rclcpp::get_logger("generic_controller");

    void convert_stuff(
        const Eigen::Vector3d& p,                  // position
        const Eigen::Vector4d& euler,              // euler angles [cphi, sphi, theta, psi]
        const Eigen::Vector3d& dp,                 // velocity
        const Eigen::Matrix<double, 3, 5>& pd_3x5, // desired trajectory: pos, vel, acc, jerk, snap
        const Eigen::Vector3d& p_int,              // integral of position error
        const Eigen::Vector3d& tvc,                // thrust vector command
        double M,                                  // M vector  
        const Eigen::Vector3d& omega,              // angular rates
        double m,                                  // mass
        double g,                                  // gravity
        const Eigen::Matrix3d& J,                  // inertia matrix
        double ell,                                // lever arm
        double& phi_ddot,
        double& phi_dot,
        double& phi_v,
        double& ey_dddot,
        double& ey_ddot,
        double& ey_dot,
        double& ey,
        double& ey_int_v,
        double& ez_dddot,
        double& ez_ddot,
        double& ez_dot,
        double& ez,
        double& ez_int_v,
        double& ddyd_ddot,
        double& ddyd_dot,
        double& ddyd_v,
        double& ddzd_ddot,
        double& ddzd_dot,
        double& ddzd_v,
        double& u1
    ){
        double cphi = euler(0);
        double sphi = euler(1);
        double th   = euler(2);
        double psi  = euler(3);

        Eigen::Matrix3d R;
        R <<
            std::cos(th)*std::cos(psi),
            -std::sin(psi),
            std::cos(psi)*std::sin(th),

            std::sin(th)*sphi + std::cos(th)*cphi*std::sin(psi),
            cphi*std::cos(psi),
            cphi*std::sin(th)*std::sin(psi) - std::cos(th)*sphi,

            std::cos(th)*sphi*std::sin(psi) - cphi*std::sin(th),
            std::cos(psi)*sphi,
            std::cos(th)*cphi + std::sin(th)*sphi*std::sin(psi);

        Eigen::Matrix3d Q;
        Q <<
            std::cos(th)/std::cos(psi), 0.0, std::sin(th)/std::cos(psi),
            std::cos(th)*std::tan(psi), 1.0, std::sin(th)*std::tan(psi),
            -std::sin(th), 0.0, std::cos(th);

        Eigen::Vector3d deuler;
        deuler = Q * omega;
        double dphi = deuler(0);
        double dth  = deuler(1);
        double dpsi = deuler(2);
        phi_dot = dphi;


        Eigen::Matrix3d dotQ;
        dotQ <<
            (dpsi*std::cos(th)*std::sin(psi) - dth*std::cos(psi)*std::sin(th))/(std::cos(psi)*std::cos(psi)), 0.0,
            (dth*std::cos(th)*std::cos(psi) + dpsi*std::sin(th)*std::sin(psi))/(std::cos(psi)*std::cos(psi)),

            (dpsi*std::cos(th) - dth*std::cos(psi)*std::sin(th)*std::sin(psi))/(std::cos(psi)*std::cos(psi)), 0.0,
            (dpsi*std::sin(th) + dth*std::cos(th)*std::cos(psi)*std::sin(psi))/(std::cos(psi)*std::cos(psi)),

            -dth*std::cos(th), 0.0,
            -dth*std::sin(th);

        double p_rate = omega(0);
        double q_rate = omega(1);
        double r_rate = omega(2);

        // S matrix
        Eigen::Matrix3d S;
        S <<
            0.0,   -r_rate,  q_rate,
            r_rate, 0.0,    -p_rate,
        -q_rate, p_rate,  0.0;

        Eigen::Matrix3d S_ell;
        S_ell <<
            0.0, 0.0, 0.0,
            0.0, 0.0,  ell,
            0.0,-ell, 0.0;

        // zeta
        Eigen::Matrix3d Jinv = J.inverse();
        Eigen::Vector3d zeta = (Q * Jinv * S * J - dotQ) * omega; // for debug, ignore gyroscopic effects

        Eigen::Vector3d ddp = (R * tvc - g * Eigen::Vector3d(1.0,0.0,0.0)) / m;
        Eigen::Vector3d d3p = R * S * tvc / m;

        Eigen::Vector3d ddeuler = Q * Jinv * (S_ell * tvc + M*tvc/tvc.norm()) - zeta;
        phi_ddot = ddeuler(0);
        
        phi_v = std::atan2(sphi, cphi); // THINK ABOUT THIS DEPARAM !!!!!!!!!!!!!!!!!!!!!!!!!!!!

        Eigen::Vector3d pd = pd_3x5.col(0);
        Eigen::Vector3d dpd = pd_3x5.col(1);
        Eigen::Vector3d ddpd = pd_3x5.col(2);
        Eigen::Vector3d d3pd = pd_3x5.col(3);
        Eigen::Vector3d d4pd = pd_3x5.col(4);

        // lateral errors
        ey = p(1) - pd(1);
        ey_dot = dp(1) - dpd(1);
        ey_ddot = ddp(1) - ddpd(1);
        ey_dddot = d3p(1) - d3pd(1);
        ey_int_v = p_int(1);
        ez = p(2) - pd(2);
        ez_dot = dp(2) - dpd(2);
        ez_ddot = ddp(2) - ddpd(2);
        ez_dddot = d3p(2) - d3pd(2);
        ez_int_v = p_int(2);

        ddyd_v = ddpd(1);
        ddyd_dot = d3pd(1);
        ddyd_ddot = d4pd(1);
        ddzd_v = ddpd(2);
        ddzd_dot = d3pd(2);
        ddzd_ddot = d4pd(2);

        u1 = tvc(0);

    }

    Eigen::Vector3d compute_inner_loop(
        const Eigen::Vector3d& p,
        const Eigen::Vector3d& dp,
        const Eigen::Matrix<double, 3, 5>& pd35,
        const Eigen::Vector4d& euler,
        const Eigen::Vector3d& omega,
        const Eigen::Vector3d& thd3,
        const Eigen::Vector3d& psid3,
        double mass,
        const Eigen::Matrix3d& J,
        double ell,
        double g,
        const Eigen::Matrix3d& kp,
        const Eigen::Matrix3d& kv,
        double ki,
        double p_int_x,
        double Mx)
    {

        double cphi = euler(0);
        double sphi = euler(1);
        double th   = euler(2);
        double psi  = euler(3);

        Eigen::Matrix3d R;
        R <<
            std::cos(th)*std::cos(psi),
            -std::sin(psi),
            std::cos(psi)*std::sin(th),

            std::sin(th)*sphi + std::cos(th)*cphi*std::sin(psi),
            cphi*std::cos(psi),
            cphi*std::sin(th)*std::sin(psi) - std::cos(th)*sphi,

            std::cos(th)*sphi*std::sin(psi) - cphi*std::sin(th),
            std::cos(psi)*sphi,
            std::cos(th)*cphi + std::sin(th)*sphi*std::sin(psi);

        Eigen::Matrix3d Q;
        Q <<
            std::cos(th)/std::cos(psi), 0.0, std::sin(th)/std::cos(psi),
            std::cos(th)*std::tan(psi), 1.0, std::sin(th)*std::tan(psi),
            -std::sin(th), 0.0, std::cos(th);

        Eigen::Vector3d deuler;
        deuler = Q * omega;
        double dphi = deuler(0);
        double dth  = deuler(1);
        double dpsi = deuler(2);


        Eigen::Matrix3d dotQ;
        dotQ <<
            (dpsi*std::cos(th)*std::sin(psi) - dth*std::cos(psi)*std::sin(th))/(std::cos(psi)*std::cos(psi)), 0.0,
            (dth*std::cos(th)*std::cos(psi) + dpsi*std::sin(th)*std::sin(psi))/(std::cos(psi)*std::cos(psi)),

            (dpsi*std::cos(th) - dth*std::cos(psi)*std::sin(th)*std::sin(psi))/(std::cos(psi)*std::cos(psi)), 0.0,
            (dpsi*std::sin(th) + dth*std::cos(th)*std::cos(psi)*std::sin(psi))/(std::cos(psi)*std::cos(psi)),

            -dth*std::cos(th), 0.0,
            -dth*std::sin(th);



        // scalar extraction
        double x = p(0);
        double dx = dp(0);
        double xd = pd35(0,0);
        double dxd = pd35(0,1);
        double ddxd = pd35(0,2);

        double p_rate = omega(0);
        double q_rate = omega(1);
        double r_rate = omega(2);

        ep(0) = x - xd;
        ev(0) = dx - dxd;
        er(1) = th - thd3(0);
        ew(1) = dth - thd3(1);
        ep(2) = psi - psid3(0);
        ev(2) = dpsi - psid3(1);

        // S matrix
        Eigen::Matrix3d S;
        S <<
            0.0,   -r_rate,  q_rate,
            r_rate, 0.0,    -p_rate,
        -q_rate, p_rate,  0.0;

        Eigen::Matrix3d S_ell;
        S_ell <<
            0.0, 0.0, 0.0,
            0.0, 0.0,  ell,
            0.0,-ell, 0.0;

        // zeta
        Eigen::Matrix3d Jinv = J.inverse();
        Eigen::Vector3d zeta = (Q * Jinv * S * J - dotQ) * omega; 

        // tracking errors
        Eigen::Vector3d xi;
        Eigen::Vector3d dxi;

        xi << x-xd, th-thd3(0), psi-psid3(0);
        dxi << dx-dxd, dth-thd3(1), dpsi-psid3(1);

        // gi matrix
        Eigen::RowVector3d gi_x = (Eigen::RowVector3d() << 1.0, 0.0, 0.0).finished() * R / mass;
        Eigen::Matrix<double,2,3> E;
        E << 0, 1, 0,
            0, 0, 1;  

        Eigen::Matrix<double,2,3> gi_th_psi;
        gi_th_psi = E * Q * Jinv * S_ell;

        Eigen::Matrix3d gi;
        gi.row(0) = gi_x;
        gi.block<2,3>(1,0) = gi_th_psi;


        Eigen::Matrix3d gi_inv = gi.inverse();

        // eta
        Eigen::Vector3d eta;
        eta(0) = g + ddxd;
        
        // Compute eta_th_psi = [0 1 0; 0 0 1]*(zeta + M*Q*J^(-1)*[-1;0;0]) + [thd3(2); psid3(2)]
        Eigen::Vector3d M_term = Mx * Q * Jinv * Eigen::Vector3d(-1.0, 0.0, 0.0);
        Eigen::Vector3d zeta_M = zeta + M_term;
        eta(1) = zeta_M(1) + thd3(2);
        eta(2) = zeta_M(2) + psid3(2);

        // tvc command
        Eigen::Vector3d tvc;
        tvc = gi_inv * (-kp*xi - kv*dxi - Eigen::Vector3d(ki * p_int_x, 0.0, 0.0) + eta);

        return tvc; // Newtons 3x1 vector in body frame
    }


    /*
    Eigen::Vector2d compute_outer_loop(
        const Eigen::Vector3d& p,
        const Eigen::Vector3d& dp,
        const Eigen::Vector4d& euler,
        const Eigen::Matrix<double,3,5>& pd35,
        const Eigen::Vector3d& p_int,
        const Eigen::Vector3d& tvc,
        double mass,
        const Eigen::Matrix2d& kp,
        const Eigen::Matrix2d& kd,
        const Eigen::Matrix2d& ki)
    {
        double cphi = euler(0);
        double sphi = euler(1);
        double th   = euler(2);
        double psi  = euler(3);

        double dy = dp(1);
        double dz = dp(2);
        double y = p(1);
        double z = p(2);

        double yd = pd35(1,0);
        double dyd = pd35(1,1);
        double ddyd = pd35(1,2);

        double zd = pd35(2,0);
        double dzd = pd35(2,1);
        double ddzd = pd35(2,2);

        double u1 = tvc(0);
        double u2 = tvc(1);
        double u3 = tvc(2);

        ep(1) = y - yd;
        ep(2) = z - zd;
        ev(1) = dy - dyd;
        ev(2) = dz - dzd;

        // errors
        Eigen::Vector2d xo, dxo, xo_int;
        xo << y-yd, z-zd;
        dxo << dy-dyd, dz-dzd;
        xo_int << p_int(1), p_int(2);

        Eigen::Matrix2d Rx_2d;
        Rx_2d <<
            cphi, -sphi,
            sphi,  cphi;

        Eigen::Vector2d law;
        law =
            -ki * xo_int
            -kp * xo
            -kd * dxo
            +Eigen::Vector2d(ddyd, ddzd);

        Eigen::Vector2d ff;
        ff = mass * Rx_2d.inverse() * law;

        // :: pitch
        double rho2 = std::sqrt(u1*u1 + u3*u3);
        double alp2 = std::atan2(u3, u1);
        double thd  = -std::asin(ff(1) / rho2) + alp2;

        // :: yaw
        double aux = u1*std::cos(th) + u3*std::sin(th);
        double rho1  = std::sqrt(aux*aux + u2*u2);
        double alp1  = std::atan2(u2, aux);
        double psid  = std::asin(ff(0) / rho1) - alp1;

        Eigen::Vector2d att_d;
        att_d << thd, psid; // radians, desired pitch and yaw

        // return dummy references for debug
        //att_d << 0.1, 0.2;
        
        return att_d;
    }
    */
    

    double compute_roll_control(
        const Eigen::Vector4d& euler,
        const Eigen::Vector3d& omega,
        const Eigen::Vector3d& phid3,
        double roll_int,
        const Eigen::Matrix3d& J,
        const Eigen::Vector3d& tvc_cmd,
        double ell,
        double kp,
        double kd,
        double ki
    ){
        double cphi = euler(0);
        double sphi = euler(1);
        double th   = euler(2);
        double psi  = euler(3);

        
        Eigen::Matrix3d R;
        R <<
            std::cos(th)*std::cos(psi),
            -std::sin(psi),
            std::cos(psi)*std::sin(th),

            std::sin(th)*sphi + std::cos(th)*cphi*std::sin(psi),
            cphi*std::cos(psi),
            cphi*std::sin(th)*std::sin(psi) - std::cos(th)*sphi,

            std::cos(th)*sphi*std::sin(psi) - cphi*std::sin(th),
            std::cos(psi)*sphi,
            std::cos(th)*cphi + std::sin(th)*sphi*std::sin(psi);

        Eigen::Matrix3d Q;
        Q <<
            std::cos(th)/std::cos(psi), 0.0, std::sin(th)/std::cos(psi),
            std::cos(th)*std::tan(psi), 1.0, std::sin(th)*std::tan(psi),
            -std::sin(th), 0.0, std::cos(th);

        Eigen::Vector3d deuler;
        deuler = Q * omega;
        double dphi = deuler(0);
        double dth  = deuler(1);
        double dpsi = deuler(2);


        Eigen::Matrix3d dotQ;
        dotQ <<
            (dpsi*std::cos(th)*std::sin(psi) - dth*std::cos(psi)*std::sin(th))/(std::cos(psi)*std::cos(psi)), 0.0,
            (dth*std::cos(th)*std::cos(psi) + dpsi*std::sin(th)*std::sin(psi))/(std::cos(psi)*std::cos(psi)),

            (dpsi*std::cos(th) - dth*std::cos(psi)*std::sin(th)*std::sin(psi))/(std::cos(psi)*std::cos(psi)), 0.0,
            (dpsi*std::sin(th) + dth*std::cos(th)*std::cos(psi)*std::sin(psi))/(std::cos(psi)*std::cos(psi)),

            -dth*std::cos(th), 0.0,
            -dth*std::sin(th);

        double p_rate = omega(0);
        double q_rate = omega(1);
        double r_rate = omega(2);

        // S matrix
        Eigen::Matrix3d S;
        S <<
            0.0,   -r_rate,  q_rate,
            r_rate, 0.0,    -p_rate,
        -q_rate, p_rate,  0.0;

        Eigen::Matrix3d S_ell;
        S_ell <<
            0.0, 0.0, 0.0,
            0.0, 0.0,  ell,
            0.0,-ell, 0.0;

        // zeta
        Eigen::Matrix3d Jinv = J.inverse();
        Eigen::Vector3d zeta = (Q * Jinv * S * J - dotQ) * omega; 

        double phid   = phid3(0);
        double dphid  = phid3(1);
        double ddphid = phid3(2);

        Eigen::RowVector3d row;
        row << 1.0, 0.0, 0.0;

        double thrust = tvc_cmd.norm();
        double g, g_inv, eta;

        g = (row*Q*Jinv*tvc_cmd/thrust).value();
        g_inv = 1.0/g;

        eta = -(row* (Q*Jinv*S_ell*tvc_cmd + zeta)).value() + ddphid;

        double cphid = std::cos(phid);
        double sphid = std::sin(phid);
        double e_phi = std::atan2((sphi*cphid - cphi*sphid), (cphi*cphid + sphi*sphid));

        double M = g_inv * (-kp*e_phi - kd*(dphi-dphid) - ki*roll_int + eta);

        return M;
    }

    double filter_stuff(const double n[3], const double d[3],
                        double u[3], double y[3], double input) {
        // shift input history
        u[2] = u[1];
        u[1] = u[0];
        u[0] = input;

        // shift output history
        y[2] = y[1];
        y[1] = y[0];

        // compute new output
        y[0] = (n[0]*u[0] + n[1]*u[1] + n[2]*u[2]
            - d[1]*y[1] - d[2]*y[2]) /d[0];

        return y[0];
    }


    void publish_debug() {
        auto msg = GenericControllerDebug();
        msg.stamp = clock_->now();

        auto state = state_aggregator_->get_state();
        auto setpoint = setpoint_aggregator_->get_position_setpoint();

        Eigen::Map<Eigen::Vector3d>(msg.position.data()) = state.position;
        Eigen::Map<Eigen::Vector3d>(msg.position_setpoint.data()) = setpoint.position;
        Eigen::Map<Eigen::Vector3d>(msg.velocity.data()) = state.velocity;
        Eigen::Map<Eigen::Vector3d>(msg.velocity_setpoint.data()) = setpoint.velocity;
        Eigen::Map<Eigen::Vector3d>(msg.acceleration.data()) = state.acceleration;
        Eigen::Map<Eigen::Vector3d>(msg.acceleration_setpoint.data()) = setpoint.acceleration;

        Eigen::Map<Eigen::Vector3d>(msg.ep.data()) = ep;
        Eigen::Map<Eigen::Vector3d>(msg.ev.data()) = ev;
        Eigen::Map<Eigen::Vector3d>(msg.er.data()) = er;
        Eigen::Map<Eigen::Vector3d>(msg.ew.data()) = ew;
        Eigen::Map<Eigen::Vector3d>(msg.u.data()) = u;

        Eigen::Map<Eigen::Vector4d>(msg.euler_c.data()) = euler_c;
        Eigen::Map<Eigen::Vector3d>(msg.omega_c.data()) = frame_transforms::radians_to_degrees(omega_c);
        Eigen::Map<Eigen::Vector3d>(msg.thd3.data()) = frame_transforms::radians_to_degrees(thd3);
        Eigen::Map<Eigen::Vector3d>(msg.psid3.data()) = frame_transforms::radians_to_degrees(psid3);
        Eigen::Map<Eigen::Vector3d>(msg.ep_int.data()) = ep_int;
        Eigen::Map<Eigen::Vector3d>(msg.phid3.data()) = frame_transforms::radians_to_degrees(phid3);

        debug_publisher_->publish(msg);
    }
};
