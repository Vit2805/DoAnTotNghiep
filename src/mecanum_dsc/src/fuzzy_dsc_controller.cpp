#include "mecanum_dsc/fuzzy_dsc_controller.hpp"
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

// ==========================================================
// PHẦN 1: HIỆN THỰC BỘ FUZZY (QUAN TRỌNG NHẤT)
// ==========================================================

FuzzyEngine::FuzzyEngine() {
    // 1. CẤU HÌNH GIÁ TRỊ ĐẦU RA (Theo ảnh image_f03dab.png)
    // Mức: RN (Rất nhỏ), N (Nhỏ), TB (Trung bình), L (Lớn), RL (Rất lớn)
    // Mapping: 0->RN, 1->N, 2->TB, 3->L, 4->RL
    
    // C1: {2, 4.2, 6.5, 8, 10}
    output_vals_C1[0] = 2.0;  output_vals_C1[1] = 4.2;  output_vals_C1[2] = 6.5; 
    output_vals_C1[3] = 8.0;  output_vals_C1[4] = 10.0;

    // C2/C3: {10, 15, 20, 25, 30}
    output_vals_C2[0] = 10.0; output_vals_C2[1] = 15.0; output_vals_C2[2] = 20.0; 
    output_vals_C2[3] = 25.0; output_vals_C2[4] = 30.0;

    // 2. CẤU HÌNH BẢNG LUẬT (Theo ảnh image_f03da8.png)
    // Hàng: e1 (AL, AN, K, DN, DL) -> Index i
    // Cột: de1 (AL, AN, K, DN, DL) -> Index j
    // Giá trị: Output Level (0=RN, 1=N, 2=TB, 3=L, 4=RL)
    
    // Ma trận Rule cho C1 (Dựa vào ảnh bảng luật suy diễn cho c1)
    // Chú ý: Cần nhìn kỹ ảnh để điền đúng (Ví dụ hàng K: RL, N, TB, N, RL)
    // Dưới đây là mô phỏng theo logic ảnh (Đường chéo chính Gain nhỏ, Sai số lớn Gain lớn)
    int raw_rules[5][5] = {
        // de: AL, AN, K,  DN, DL
        {2, 1, 0, 1, 2}, // e: AL (TB, N, RN, N, TB) - Ví dụ theo ảnh
        {3, 2, 1, 2, 3}, // e: AN (L, TB, N, TB, N)
        {4, 3, 2, 3, 4}, // e: K  (RL, L, TB, L, RL)  <- Dòng giữa quan trọng nhất
        {3, 2, 1, 2, 3}, // e: DN
        {2, 1, 0, 1, 2}  // e: DL
    };
    // COPY giá trị vào mảng class
    for(int i=0; i<5; i++) 
        for(int j=0; j<5; j++) 
            rule_table_C1[i][j] = raw_rules[i][j];

    // 3. CẤU HÌNH TẬP MỜ ĐẦU VÀO (Theo hình tam giác image_f03dc5.png)
    // Bạn cần điều chỉnh các số này cho phù hợp với đơn vị mét/rad của robot
    // Ví dụ: Sai số 0.5m là Lớn hay Nhỏ?
    params_e  = {-1.0, -0.5, 0.0, 0.5, 1.0}; // Đơn vị: mét
    params_de = {-1.0, -0.5, 0.0, 0.5, 1.0}; // Đơn vị: m/s
}

double FuzzyEngine::trimf(double x, double a, double b, double c) {
    return std::max(0.0, std::min((x - a)/(b - a), (c - x)/(c - b)));
}

void FuzzyEngine::compute_gains(double e, double de, double &out_C1, double &out_C2) {
    // Bước 1: Mờ hóa (Fuzzification) - Tính độ thuộc mu cho e và de
    double mu_e[5], mu_de[5];
    
    // Mẹo: Dùng mảng params để code gọn
    double p_e[] = {params_e.AL_center, params_e.AN_center, params_e.K_center, params_e.DN_center, params_e.DL_center};
    // Với tam giác đều, chân trái = center - width, chân phải = center + width. 
    // Giả sử width = 0.5 (khoảng cách giữa các đỉnh)
    double w = 0.5; 
    
    for(int i=0; i<5; i++) {
        mu_e[i]  = trimf(e,  p_e[i]-w,  p_e[i],  p_e[i]+w);
        mu_de[i] = trimf(de, p_e[i]-w,  p_e[i],  p_e[i]+w); // Tạm dùng chung param
    }

    // Bước 2 & 3: Luật hợp thành & Giải mờ (Defuzzification - Weighted Average)
    // Phương pháp: Sum(Min(mu_e, mu_de) * Output_Value) / Sum(Min(mu_e, mu_de))
    
    double num_C1 = 0.0, den_C1 = 0.0;
    double num_C2 = 0.0, den_C2 = 0.0;

    for (int i = 0; i < 5; i++) {       // Duyệt qua các tập e
        for (int j = 0; j < 5; j++) {   // Duyệt qua các tập de
            
            // Độ kích hoạt của luật (Activation strength) - Dùng phép MIN
            double alpha = std::min(mu_e[i], mu_de[j]);
            
            if (alpha > 0) {
                // Tra bảng luật để biết đầu ra là mức nào (RN..RL)
                int out_idx = rule_table_C1[i][j];
                
                // Cộng dồn cho C1
                double val_C1 = output_vals_C1[out_idx];
                num_C1 += alpha * val_C1;
                den_C1 += alpha;

                // Cộng dồn cho C2 (Dùng chung luật hoặc bảng riêng tùy thiết kế)
                // Ở đây giả sử dùng chung logic mức độ nhưng map sang giá trị C2
                double val_C2 = output_vals_C2[out_idx];
                num_C2 += alpha * val_C2;
                den_C2 += alpha;
            }
        }
    }

    // Kết quả cuối cùng
    out_C1 = (den_C1 > 0) ? (num_C1 / den_C1) : output_vals_C1[2]; // Default TB
    out_C2 = (den_C2 > 0) ? (num_C2 / den_C2) : output_vals_C2[2];
}


// ==========================================================
// PHẦN 2: NODE ROS 2 & THUẬT TOÁN DSC
// ==========================================================

FuzzyDSCController::FuzzyDSCController() : Node("fuzzy_dsc_node") {
    // Parameters
    this->declare_parameter("tau_filter", 0.05);
    this->declare_parameter("mass", 10.0);
    this->declare_parameter("damping", 0.5);

    // Init ROS
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&FuzzyDSCController::odom_callback, this, _1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), 
        std::bind(&FuzzyDSCController::control_loop, this));
}

void FuzzyDSCController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = *msg;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double r, p;
    m.getRPY(r, p, current_yaw_);
}

// --- HÀM DSC KẾT HỢP FUZZY (Theo sơ đồ image_57814d.png) ---
double FuzzyDSCController::compute_single_axis(
    double q_ref, double dq_ref, 
    double q_act, double dq_act, 
    double &prev_error,
    double M_hat, double D_hat, 
    double &s_filter, 
    double dt, bool is_angle) 
{
    // 1. Tính sai số bám e1 (Khối đầu tiên bên trái sơ đồ)
    double e1 = q_act - q_ref;
    if (is_angle) e1 = atan2(sin(e1), cos(e1));

    // 2. Tính đạo hàm sai số (dot_e1) để đưa vào Fuzzy
    double de1 = (e1 - prev_error) / dt;
    prev_error = e1; // Lưu lại cho vòng sau

    // 3. GỌI BỘ FUZZY (Khối Fuzzy màu trắng trong sơ đồ)
    // Đầu vào: e1, de1 -> Đầu ra: C1_adaptive, C2_adaptive
    double C1, C2;
    fuzzy_.compute_gains(e1, de1, C1, C2);

    // 4. Tính tín hiệu điều khiển ảo Beta (Calculate_Beta)
    // Theo công thức: beta = v_ref - C1 * e1
    double beta = dq_ref - C1 * e1;

    // 5. Bộ lọc bậc nhất (Filter - Khối giữa sơ đồ)
    // Dynamic: tau * dot(alpha) + alpha = beta
    double prev_alpha = s_filter;
    double alpha_dot = (beta - prev_alpha) / tau_filter_;
    s_filter = prev_alpha + alpha_dot * dt; // Tích phân ra alpha mới (af)

    // 6. Tính sai lệch vận tốc e2 (Tính mặt trượt S)
    double e2 = dq_act - s_filter;

    // 7. Tính t_eq (Động học ngược) + t_sw (Switching với Gain C2 từ Fuzzy)
    // tau = M * dot(alpha) + D * v_act - C2 * e2
    double tau = M_hat * alpha_dot + D_hat * dq_act - C2 * e2;

    return tau;
}

void FuzzyDSCController::control_loop() {
    // Update params
    this->get_parameter("tau_filter", tau_filter_);
    this->get_parameter("mass", mass_);
    this->get_parameter("damping", damping_);
    double dt = 0.05;

    // Lấy trạng thái hiện tại
    double x = current_odom_.pose.pose.position.x;
    double y = current_odom_.pose.pose.position.y;
    double vx = current_odom_.twist.twist.linear.x; // Body frame velocity
    double vy = current_odom_.twist.twist.linear.y;
    double w = current_odom_.twist.twist.angular.z;

    // QUAN TRỌNG: Chuyển đổi sai số vị trí sang Body Frame (như đã bàn ở các câu trước)
    // Giả sử Ref = [0,0,0]
    double ex_global = x - 0.0; 
    double ey_global = y - 0.0;
    
    double c = cos(current_yaw_);
    double s = sin(current_yaw_);
    double ex_body =  c * ex_global + s * ey_global;
    double ey_body = -s * ex_global + c * ey_global;

    // --- TÍNH TOÁN 3 TRỤC ---
    
    // Trục X
    double fx = compute_single_axis(0, 0, ex_body, vx, prev_ex, mass_, damping_, s_filter_x, dt, false);
    
    // Trục Y
    double fy = compute_single_axis(0, 0, ey_body, vy, prev_ey, mass_, damping_, s_filter_y, dt, false);
    
    // Trục Yaw
    double mz = compute_single_axis(0, 0, current_yaw_, w, prev_eth, mass_*0.1, damping_, s_filter_th, dt, true);

    // Chuyển Lực -> Vận tốc lệnh
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = std::clamp(vx + (fx/mass_)*dt, -0.5, 0.5);
    cmd.linear.y = std::clamp(vy + (fy/mass_)*dt, -0.5, 0.5);
    cmd.angular.z = std::clamp(w + (mz/(mass_*0.1))*dt, -1.0, 1.0);

    cmd_vel_pub_->publish(cmd);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FuzzyDSCController>());
    rclcpp::shutdown();
    return 0;
}