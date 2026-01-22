

import math 
from typing import List ,Optional ,Tuple 

import rclpy 
from rclpy .node import Node 
from rclpy .duration import Duration 
from rcl_interfaces .msg import ParameterDescriptor 
from sensor_msgs .msg import LaserScan 
from geometry_msgs .msg import TransformStamped 
import tf2_ros 
from tf2_ros import TransformException 


def yaw_from_quat (x :float ,y :float ,z :float ,w :float )->float :
    siny_cosp =2.0 *(w *z +x *y )
    cosy_cosp =1.0 -2.0 *(y *y +z *z )
    return math .atan2 (siny_cosp ,cosy_cosp )


def point_in_polygon (px :float ,py :float ,poly_xy :List [float ])->bool :
    if not poly_xy or len (poly_xy )<6 :
        return False 
    n =len (poly_xy )//2 
    inside =False 
    j =n -1 
    for i in range (n ):
        xi ,yi =poly_xy [2 *i ],poly_xy [2 *i +1 ]
        xj ,yj =poly_xy [2 *j ],poly_xy [2 *j +1 ]
        intersect =((yi >py )!=(yj >py ))and (px <(xj -xi )*(py -yi )/(yj -yi +1e-12 )+xi )
        if intersect :
            inside =not inside 
        j =i 
    return inside 


def parse_box (box_param )->Optional [Tuple [float ,float ,float ,float ]]:
    if box_param is None :
        return None 
    if isinstance (box_param ,(list ,tuple ))and len (box_param )==4 :
        return float (box_param [0 ]),float (box_param [1 ]),float (box_param [2 ]),float (box_param [3 ])
    if isinstance (box_param ,str ):
        try :
            parts =[float (p .strip ())for p in box_param .replace ('[','').replace (']','').split (',')]
            if len (parts )==4 :
                return parts [0 ],parts [1 ],parts [2 ],parts [3 ]
        except Exception :
            return None 
    return None 


def parse_polygon (poly_param )->List [float ]:

    if poly_param is None :
        return []

    if isinstance (poly_param ,(list ,tuple )):
        try :
            vals =[float (v )for v in poly_param ]
            return vals if len (vals )>=6 and len (vals )%2 ==0 else []
        except Exception :
            return []

    if isinstance (poly_param ,str ):
        s =poly_param .strip ()
        s =s .replace ('[','').replace (']','')

        try :
            vals =[float (p .strip ())for p in s .split (',')if p .strip ()!='']
            if len (vals )>=6 and len (vals )%2 ==0 :
                return vals 
        except Exception :
            pass 

        try :
            pairs =[]
            for token in s .split (','):
                parts =token .strip ().split ()
                if len (parts )==2 :
                    pairs .extend ([float (parts [0 ]),float (parts [1 ])])
            return pairs if len (pairs )>=6 and len (pairs )%2 ==0 else []
        except Exception :
            return []

    return []


class ScanMerger360 (Node ):
    def __init__ (self ):
        super ().__init__ ('scan_merger_360')
        desc =ParameterDescriptor (dynamic_typing =True )

        self .declare_parameter ('front_topic','/scan_front',descriptor =desc )
        self .declare_parameter ('rear_topic','/scan_rear',descriptor =desc )
        self .declare_parameter ('target_frame','base_link',descriptor =desc )
        self .declare_parameter ('output_topic','/scan_360',descriptor =desc )

        self .declare_parameter ('angle_min',-math .pi ,descriptor =desc )
        self .declare_parameter ('angle_max',math .pi ,descriptor =desc )
        self .declare_parameter ('angle_increment',math .radians (1.0 ),descriptor =desc )
        self .declare_parameter ('range_min',0.05 ,descriptor =desc )
        self .declare_parameter ('range_max',12.0 ,descriptor =desc )
        self .declare_parameter ('publish_rate_hz',10.0 ,descriptor =desc )
        self .declare_parameter ('use_inf',True ,descriptor =desc )

        self .declare_parameter ('front_mask_box',[0.0 ,0.40 ,-0.1 ,0.30 ],descriptor =desc )
        self .declare_parameter ('rear_mask_box',[0.0 ,0.40 ,-0.1 ,0.30 ],descriptor =desc )
        self .declare_parameter ('body_polygon',[],descriptor =desc )

        self .front_topic =self .get_parameter ('front_topic').get_parameter_value ().string_value 
        self .rear_topic =self .get_parameter ('rear_topic').get_parameter_value ().string_value 
        self .target_frame =self .get_parameter ('target_frame').get_parameter_value ().string_value 
        self .output_topic =self .get_parameter ('output_topic').get_parameter_value ().string_value 

        self .angle_min =float (self .get_parameter ('angle_min').value )
        self .angle_max =float (self .get_parameter ('angle_max').value )
        self .angle_inc =float (self .get_parameter ('angle_increment').value )
        self .range_min =float (self .get_parameter ('range_min').value )
        self .range_max =float (self .get_parameter ('range_max').value )
        self .use_inf =bool (self .get_parameter ('use_inf').value )


        if self .angle_inc <=0.0 or self .angle_max <=self .angle_min :
            self .get_logger ().error (
            f"Invalid angle config: angle_min={self.angle_min}, "
            f"angle_max={self.angle_max}, angle_inc={self.angle_inc}. "
            "Reset to defaults: [-pi, +pi], inc=1 deg."
            )
            self .angle_min =-math .pi 
            self .angle_max =math .pi 
            self .angle_inc =math .radians (1.0 )

        self .pub_rate_hz =float (self .get_parameter ('publish_rate_hz').value )
        self .body_polygon =parse_polygon (self .get_parameter ('body_polygon').value )
        self .front_mask =parse_box (self .get_parameter ('front_mask_box').value )
        self .rear_mask =parse_box (self .get_parameter ('rear_mask_box').value )

        self .tf_buffer =tf2_ros .Buffer (cache_time =Duration (seconds =5.0 ))
        self .tf_listener =tf2_ros .TransformListener (self .tf_buffer ,self )

        self .sub_front =self .create_subscription (LaserScan ,self .front_topic ,self ._on_front ,10 )
        self .sub_rear =self .create_subscription (LaserScan ,self .rear_topic ,self ._on_rear ,10 )
        self .pub =self .create_publisher (LaserScan ,self .output_topic ,10 )

        self .last_front :Optional [LaserScan ]=None 
        self .last_rear :Optional [LaserScan ]=None 

        period =1.0 /max (1e-3 ,self .pub_rate_hz )
        self .timer =self .create_timer (period ,self ._tick )

        self .get_logger ().info (
        f"scan_merger_360: target_frame={self.target_frame}, out={self.output_topic}, "
        f"bins={int(round((self.angle_max - self.angle_min) / self.angle_inc))}, "
        f"front='{self.front_topic}', rear='{self.rear_topic}'"
        )
        if self .body_polygon :
            self .get_logger ().info (f"Body polygon vertices: {len(self.body_polygon) // 2} points")

    def _on_front (self ,msg :LaserScan ):
        self .last_front =msg 

    def _on_rear (self ,msg :LaserScan ):
        self .last_rear =msg 

    def _tick (self ):

        if self .angle_inc <=0.0 or self .angle_max <=self .angle_min :
            return 

        scans =[s for s in [self .last_front ,self .last_rear ]if s is not None ]
        if not scans :
            return 

        bins =int (round ((self .angle_max -self .angle_min )/self .angle_inc ))
        angle_max_exact =self .angle_min +bins *self .angle_inc 
        if angle_max_exact <self .angle_max -1e-6 :
            bins +=1 
        ranges =[math .inf ]*bins 

        for scan in scans :
            self ._accumulate_scan (scan ,ranges )

        if self .body_polygon :
            for i in range (bins ):
                r =ranges [i ]
                if not math .isfinite (r ):
                    continue 
                ang =self .angle_min +i *self .angle_inc 
                x =r *math .cos (ang )
                y =r *math .sin (ang )
                if point_in_polygon (x ,y ,self .body_polygon ):
                    ranges [i ]=math .inf 

        out =LaserScan ()
        now =self .get_clock ().now ().to_msg ()
        out .header .frame_id =self .target_frame 
        out .header .stamp =now 
        out .angle_min =self .angle_min 
        out .angle_max =self .angle_min +(bins -1 )*self .angle_inc 
        out .angle_increment =self .angle_inc 
        out .time_increment =0.0 
        out .scan_time =1.0 /self .pub_rate_hz 
        out .range_min =self .range_min 
        out .range_max =self .range_max 
        out .ranges =[]
        for r in ranges :
            if math .isfinite (r )and (self .range_min <=r <=self .range_max ):
                out .ranges .append (r )
            else :
                out .ranges .append (math .inf if self .use_inf else 0.0 )
        self .pub .publish (out )

    def _accumulate_scan (self ,scan :LaserScan ,ranges_out :List [float ]):
        src =scan .header .frame_id 
        try :
            t :TransformStamped =self .tf_buffer .lookup_transform (
            self .target_frame ,src ,scan .header .stamp ,timeout =Duration (seconds =0.05 )
            )
        except TransformException as e :
            self .get_logger ().warning (f"TF {src}->{self.target_frame} fail: {e}")
            return 

        tx =t .transform .translation .x 
        ty =t .transform .translation .y 
        qx =t .transform .rotation .x 
        qy =t .transform .rotation .y 
        qz =t .transform .rotation .z 
        qw =t .transform .rotation .w 
        yaw =yaw_from_quat (qx ,qy ,qz ,qw )
        cy =math .cos (yaw )
        sy =math .sin (yaw )

        mask_box =None 
        if src .endswith ('lidar_front')or src =='lidar_front':
            mask_box =self .front_mask 
        elif src .endswith ('lidar_rear')or src =='lidar_rear':
            mask_box =self .rear_mask 

        a =scan .angle_min 
        inc =scan .angle_increment 
        n =len (scan .ranges )
        for i in range (n ):
            r =scan .ranges [i ]
            if not math .isfinite (r )or r <scan .range_min or r >scan .range_max :
                continue 
            ang =a +i *inc 

            lx =r *math .cos (ang )
            ly =r *math .sin (ang )

            if mask_box is not None :
                xmin ,xmax ,ymin ,ymax =mask_box 
                if (xmin <=lx <=xmax )and (ymin <=ly <=ymax ):
                    continue 

            bx =cy *lx -sy *ly +tx 
            by =sy *lx +cy *ly +ty 

            rr =math .hypot (bx ,by )
            if rr <self .range_min or rr >self .range_max :
                continue 

            theta =math .atan2 (by ,bx )
            idx =int (round ((theta -self .angle_min )/self .angle_inc ))
            if 0 <=idx <len (ranges_out ):
                if rr <ranges_out [idx ]:
                    ranges_out [idx ]=rr 


def main ():
    rclpy .init ()
    node =ScanMerger360 ()
    try :
        rclpy .spin (node )
    except KeyboardInterrupt :
        pass 
    finally :
        node .destroy_node ()
        rclpy .shutdown ()


if __name__ =='__main__':
    main ()

