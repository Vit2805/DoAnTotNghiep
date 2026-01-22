



import sys 
import termios 
import tty 
import select 
from typing import Optional 

import rclpy 
from rclpy .node import Node 
from rclpy .parameter import Parameter 
from rcl_interfaces .msg import ParameterDescriptor 
from geometry_msgs .msg import Twist 

HELP =r"""
---------------- TELEOP (2 CHẾ ĐỘ) ----------------
[ SNAP MODE ]  (đặt ngay tốc độ = ±max×scale)
  W/S: tiến / lùi
  A/D: trái / phải (strafe)
  Q/E: quay trái / quay phải

[ INCREMENTAL MODE ]  (tích tốc – bấm nhiều lần sẽ cộng dồn)
  I/K: +vx / -vx (tiến / lùi)
  J/L: +vy / -vy (trái / phải)
  U/O: +wz / -wz (quay trái / quay phải)
  m / .: giảm / tăng scale (độ nhạy bước)
  SPACE: dừng (vx=vy=wz=0)

P: in trạng thái hiện tại   |   H hoặc ?: hiện trợ giúp   |   Ctrl+C: thoát
----------------------------------------------------
"""

class TeleopWASDV2 (Node ):
    def __init__ (self ):
        super ().__init__ ("teleop_wasd_v2")


        self .R =self ._get_required_double ("robot.geometry.R")
        self .lx =self ._get_required_double ("robot.geometry.lx")
        self .ly =self ._get_required_double ("robot.geometry.ly")
        self .L =self .lx +self .ly 

        self .wheel_max_rpm =self ._get_required_double ("robot.drivetrain.wheel_max_rpm")
        self .limit_scale_safety =self ._get_required_double ("robot.drivetrain.limit_scale_safety")


        self .max_vx =self ._get_optional_double ("bridge_limits.max_vx")
        self .max_vy =self ._get_optional_double ("bridge_limits.max_vy")
        self .max_wz =self ._get_optional_double ("bridge_limits.max_wz")
        self ._auto_compute_limits_if_needed ()


        self .rate_hz =self ._get_int ("teleop.rate_hz",20 )
        self .scale =self ._clip (self ._get_double ("teleop.init_scale",0.5 ),0.0 ,1.0 )
        self .step_scale =self ._get_double ("teleop.step_scale",0.1 )
        self .inc_lin_frac =self ._clip (self ._get_double ("teleop.inc_lin_frac",0.10 ),0.0 ,1.0 )
        self .inc_ang_frac =self ._clip (self ._get_double ("teleop.inc_ang_frac",0.10 ),0.0 ,1.0 )


        self .pub =self .create_publisher (Twist ,"cmd_vel",10 )
        self .timer =self .create_timer (1.0 /float (max (1 ,self .rate_hz )),self ._on_timer_pub )


        self .vx =0.0 
        self .vy =0.0 
        self .wz =0.0 

        self .get_logger ().info (HELP )
        self .get_logger ().info (self ._readable_limits ())
        self ._print_status ()


        self ._old_term_attrs =termios .tcgetattr (sys .stdin .fileno ())
        tty .setcbreak (sys .stdin .fileno ())
        self .key_timer =self .create_timer (0.02 ,self ._poll_keyboard )


    def _get_required_double (self ,name :str )->float :
        desc =ParameterDescriptor (dynamic_typing =True )
        self .declare_parameter (name ,None ,descriptor =desc )
        p =self .get_parameter (name )
        if p .type_ ==Parameter .Type .NOT_SET or p .value is None :
            self .get_logger ().error (f"Thiếu tham số bắt buộc '{name}' trong YAML.")
            raise RuntimeError (f"Missing required param: {name}")
        try :
            return float (p .value )
        except Exception :
            self .get_logger ().error (f"Tham số '{name}' phải là số (float). Giá trị: {p.value}")
            raise 

    def _get_double (self ,name :str ,default :float )->float :
        self .declare_parameter (name ,float (default ))
        p =self .get_parameter (name )
        return float (p .value )if p .type_ !=Parameter .Type .NOT_SET else float (default )

    def _get_int (self ,name :str ,default :int )->int :
        self .declare_parameter (name ,int (default ))
        p =self .get_parameter (name )
        return int (p .value )if p .type_ !=Parameter .Type .NOT_SET else int (default )

    def _get_optional_double (self ,name :str )->Optional [float ]:
        desc =ParameterDescriptor (dynamic_typing =True )
        self .declare_parameter (name ,None ,descriptor =desc )
        p =self .get_parameter (name )
        v =p .value 
        if v is None :
            return None 
        if isinstance (v ,str ):
            vs =v .strip ().lower ()
            if vs in ("null","none","nan",""):
                return None 
            try :
                return float (v )
            except ValueError :
                self .get_logger ().warn (f"Param {name}='{v}' không phải số, bỏ qua -> auto tính.")
                return None 
        try :
            return float (v )
        except Exception :
            self .get_logger ().warn (f"Param {name}={v} không ép float được, bỏ qua -> auto tính.")
            return None 


    def _auto_compute_limits_if_needed (self ):
        omega_wheel_max =2.0 *3.141592653589793 *(max (0.0 ,self .wheel_max_rpm )/60.0 )
        v_lin =self .R *omega_wheel_max 
        w_z =(self .R /max (1e-9 ,self .L ))*omega_wheel_max 
        v_lin_safe =v_lin *self .limit_scale_safety 
        w_z_safe =w_z *self .limit_scale_safety 
        if self .max_vx is None :
            self .max_vx =v_lin_safe 
        if self .max_vy is None :
            self .max_vy =v_lin_safe 
        if self .max_wz is None :
            self .max_wz =w_z_safe 


    def _on_timer_pub (self ):
        msg =Twist ()
        msg .linear .x =self .vx 
        msg .linear .y =self .vy 
        msg .angular .z =self .wz 
        self .pub .publish (msg )

    def _poll_keyboard (self ):

        dr ,_ ,_ =select .select ([sys .stdin ],[],[],0 )
        if not dr :
            return 
        ch =sys .stdin .read (1 )
        if not ch :
            return 

        c =ch .lower ()

        if c =='w':
            self .vx =self .scale *self .max_vx 
        elif c =='s':
            self .vx =-self .scale *self .max_vx 
        elif c =='a':
            self .vy =self .scale *self .max_vy 
        elif c =='d':
            self .vy =-self .scale *self .max_vy 
        elif c =='q':
            self .wz =self .scale *self .max_wz 
        elif c =='e':
            self .wz =-self .scale *self .max_wz 


        elif c =='i':
            self .vx =self ._clamp (self .vx +self .scale *self .inc_lin_frac *self .max_vx ,
            -self .max_vx ,self .max_vx )
        elif c =='k':
            self .vx =self ._clamp (self .vx -self .scale *self .inc_lin_frac *self .max_vx ,
            -self .max_vx ,self .max_vx )
        elif c =='j':
            self .vy =self ._clamp (self .vy +self .scale *self .inc_lin_frac *self .max_vy ,
            -self .max_vy ,self .max_vy )
        elif c =='l':
            self .vy =self ._clamp (self .vy -self .scale *self .inc_lin_frac *self .max_vy ,
            -self .max_vy ,self .max_vy )
        elif c =='u':
            self .wz =self ._clamp (self .wz +self .scale *self .inc_ang_frac *self .max_wz ,
            -self .max_wz ,self .max_wz )
        elif c =='o':
            self .wz =self ._clamp (self .wz -self .scale *self .inc_ang_frac *self .max_wz ,
            -self .max_wz ,self .max_wz )


        elif c =='m':
            self .scale =self ._clip (self .scale -self .step_scale ,0.0 ,1.0 )
            self ._print_status ()
        elif c =='.':
            self .scale =self ._clip (self .scale +self .step_scale ,0.0 ,1.0 )
            self ._print_status ()
        elif c ==' ':
            self .vx =self .vy =self .wz =0.0 
        elif c =='p':
            self ._print_status ()
        elif c in ('h','?'):
            self .get_logger ().info (HELP )
        else :

            return 

        self .get_logger ().info (
        f"cmd: vx={self.vx:.4f} m/s, vy={self.vy:.4f} m/s, wz={self.wz:.4f} rad/s  (scale={self.scale:.2f})"
        )


    @staticmethod 
    def _clip (x :float ,mn :float ,mx :float )->float :
        return mn if x <mn else mx if x >mx else x 

    @staticmethod 
    def _clamp (x :float ,mn :float ,mx :float )->float :
        return mn if x <mn else mx if x >mx else x 

    def _readable_limits (self )->str :
        return (f"[Limits] max_vx={self.max_vx:.4f} m/s, "
        f"max_vy={self.max_vy:.4f} m/s, "
        f"max_wz={self.max_wz:.4f} rad/s")

    def _print_status (self ):
        self .get_logger ().info (self ._readable_limits ())
        self .get_logger ().info (f"[Teleop] scale={self.scale:.2f} | "
        f"inc_lin_step={self.inc_lin_frac*self.scale:.2f}×max_lin, "
        f"inc_ang_step={self.inc_ang_frac*self.scale:.2f}×max_ang")


    def destroy_node (self ):
        try :
            termios .tcsetattr (sys .stdin .fileno (),termios .TCSADRAIN ,self ._old_term_attrs )
        except Exception :
            pass 
        return super ().destroy_node ()


def main ():
    rclpy .init ()
    node =TeleopWASDV2 ()
    try :
        rclpy .spin (node )
    except KeyboardInterrupt :
        pass 
    finally :
        node .destroy_node ()
        rclpy .shutdown ()


if __name__ =="__main__":
    main ()
