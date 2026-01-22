import math

class FuzzyPID:
    def __init__(self, 
                 base_kp=0.6,    # [GIẢM] Giảm Kp mặc định (Cũ 1.2 -> 0.6)
                 base_ki=1.5,    # Tăng Ki để bù sai số từ từ
                 base_kd=0.05,   
                 max_out=1.0, 
                 error_map_max=0.5,
                 max_accel=2.0): # [MỚI] Giới hạn gia tốc (m/s^2)
        
        # Cài đặt cơ bản
        self.base_kp = base_kp
        self.base_ki = base_ki
        self.base_kd = base_kd
        
        # Giới hạn
        self.max_out = max_out
        self.min_out = -max_out
        self.error_max = error_map_max
        self.max_accel = max_accel # Gia tốc tối đa cho phép
        
        # Biến trạng thái
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_output = 0.0     # [MỚI] Lưu đầu ra trước đó để làm mượt
        
        # Bộ lọc cho khâu D (Tránh giật khi nhiễu encoder)
        self.alpha_d = 0.1         # Hệ số lọc (0.0 < alpha < 1.0). Càng nhỏ càng mượt.
        self.filtered_deriv = 0.0
        
        # Biến debug
        self.dynamic_kp = 0.0
        self.dynamic_kd = 0.0

    def _fuzzify(self, error):
        """
        Chuyển đổi sai số thành độ thuộc (Membership function)
        """
        abs_err = abs(error)
        normalized_err = min(abs_err / self.error_max, 1.0)
        
        mu_large = normalized_err          
        mu_small = 1.0 - normalized_err    
        
        return mu_small, mu_large

    def _inference_rules(self, mu_small, mu_large):
        """
        [CẬP NHẬT] LUẬT MỜ NHẸ NHÀNG HƠN
        Thay vì nhân 2.5 lần, chỉ thay đổi nhẹ nhàng để tránh sốc.
        """
        
        # 1. Trường hợp GẦN (Small Error): Chạy êm
        kp_sm = self.base_kp * 1.0
        kd_sm = self.base_kd * 1.2   # Tăng nhẹ D để phanh êm
        ki_sm = self.base_ki * 1.0
        
        # 2. Trường hợp XA (Large Error): Tăng tốc vừa phải
        # [QUAN TRỌNG] Giảm hệ số này xuống! 
        # Cũ: 2.5 (Quá gắt) -> Mới: 1.5 (Vừa phải)
        kp_lg = self.base_kp * 1.5   
        kd_lg = self.base_kd * 0.8   
        ki_lg = 0.0                  
        
        # Trung bình trọng số
        dyn_kp = (mu_small * kp_sm) + (mu_large * kp_lg)
        dyn_kd = (mu_small * kd_sm) + (mu_large * kd_lg)
        dyn_ki = (mu_small * ki_sm) + (mu_large * ki_lg)
        
        return dyn_kp, dyn_ki, dyn_kd

    def compute(self, setpoint, current, dt):
        """
        Hàm tính toán chính
        """
        if dt <= 0.0001: return 0.0
        
        # 1. Tính sai số
        error = setpoint - current
        
        # 2. FUZZY TUNING
        mu_small, mu_large = self._fuzzify(error)
        kp, ki, kd = self._inference_rules(mu_small, mu_large)
        
        self.dynamic_kp = kp
        self.dynamic_kd = kd
        
        # 3. Tính toán PID
        # P term
        p_out = kp * error
        
        # I term (Tích phân)
        self.integral += error * dt
        # Anti-windup
        if self.integral > self.max_out: self.integral = self.max_out
        elif self.integral < self.min_out: self.integral = self.min_out
        i_out = ki * self.integral
        
        # D term (Đạo hàm có lọc nhiễu - Low Pass Filter)
        raw_derivative = (error - self.prev_error) / dt
        self.filtered_deriv = (self.alpha_d * raw_derivative) + ((1 - self.alpha_d) * self.filtered_deriv)
        d_out = kd * self.filtered_deriv
        
        # Tổng hợp đầu ra thô
        raw_output = p_out + i_out + d_out
        self.prev_error = error
        
        # 4. [MỚI] OUTPUT RAMPING (Làm mượt đầu ra)
        # Giới hạn tốc độ thay đổi của output (Slew Rate Limiter)
        # delta_limit = gia_tốc_max * dt
        max_change = self.max_accel * dt
        
        if raw_output > self.last_output + max_change:
            output = self.last_output + max_change
        elif raw_output < self.last_output - max_change:
            output = self.last_output - max_change
        else:
            output = raw_output

        # 5. Kẹp bão hòa (Saturation)
        if output > self.max_out: output = self.max_out
        elif output < self.min_out: output = self.min_out
        
        self.last_output = output
        
        return output
    
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_output = 0.0
        self.filtered_deriv = 0.0