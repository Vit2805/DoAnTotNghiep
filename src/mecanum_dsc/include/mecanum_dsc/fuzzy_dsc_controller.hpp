#ifndef FUZZY_DSC_CONTROLLER_HPP_
#define FUZZY_DSC_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <vector>
#include <array>

// --- ĐỊNH NGHĨA CÁC MỨC NGÔN NGỮ MỜ (5 MỨC) ---
// AL: Âm Lớn, AN: Âm Nhỏ, K: Không (Zero), DN: Dương Nhỏ, DL: Dương Lớn
enum FuzzyLevel { AL = 0, AN = 1, K = 2, DN = 3, DL = 4 };

// --- CLASS XỬ LÝ LOGIC MỜ (THEO ẢNH BẠN CUNG CẤP) ---
class FuzzyEngine {
public:
    FuzzyEngine();

    // Hàm tính toán Gain C1 và C2 dựa trên e và de
    void compute_gains(double e, double de, double &out_C1, double &out_C2);

private:
    // Hàm thuộc hình tam giác (Triangular Membership Function)
    double trimf(double x, double a, double b, double c);

    // Bảng luật (Rule Base) 5x5 - Tra cứu theo [e_index][de_index]
    // Giá trị trả về là FuzzyLevel đầu ra (RN, N, TB, L, RL)
    int rule_table_C1[5][5]; 
    
    // Giá trị thực (Crisp value) cho các mức đầu ra của C1 và C2
    // Dựa trên ảnh bảng giá trị: RN, N, TB, L, RL
    double output_vals_C1[5]; 
    double output_vals_C2[5];

    // Thông số các tập mờ đầu vào (Input Membership Parameters)
    // Cần tinh chỉnh cái này cho khớp với thực tế robot
    struct InputParams {
        double AL_center, AN_center, K_center, DN_center, DL_center;
    } params_e, params_de;
};

// --- CLASS CONTROLLER CHÍNH ---
class FuzzyDSCController : public rclcpp::Node {
public:
    FuzzyDSCController();

private:
    // ROS Interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables
    nav_msgs::msg::Odometry current_odom_;
    double current_yaw_ = 0.0;
    
    // Filter State cho DSC (Biến nhớ alpha_f)
    double s_filter_x = 0.0;
    double s_filter_y = 0.0;
    double s_filter_th = 0.0;

    // Sai số cũ để tính đạo hàm de (cho Fuzzy)
    double prev_ex = 0.0, prev_ey = 0.0, prev_eth = 0.0;

    // Parameters
    double tau_filter_;
    double mass_, damping_;

    // Đối tượng Fuzzy
    FuzzyEngine fuzzy_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void control_loop();
    
    // Hàm DSC cho 1 trục
    double compute_single_axis(
        double q_ref, double dq_ref, 
        double q_act, double dq_act, 
        double &prev_error_for_de,
        double M_hat, double D_hat, 
        double &filter_state, 
        double dt, bool is_angle);
};

#endif