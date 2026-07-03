#include "main.h"
#include "control_task.h"
#include "control_process.hpp"
#include "pid_controller.h"
#include "chassis_task.h"
#include "topic_pool.h"
#include "topics.hpp"
#include <cmath>
#include <cstdint>
#include <math.h>
#include "pick_hand.hpp"
#include "weapon_hand.hpp"
#include "lift.hpp"
#include "omni_ir.hpp"
#include "control_action.hpp"

class TrajectChassis{
public:

    float_t watch_1;
    float_t watch_2;
    float_t watch_3;
    float_t watch_4;
    float_t watch_5;

    bool Traj_complete_Flag;
    bool PointTrack_linear_complete_Flag;
    bool PointTrack_omega_complete_Flag; 

    // =====================================================
    //  轨迹运行函数
    // =====================================================
    void Run(pub_chassis_cmd Now) {
        now.x = Now.linear_x_;
        now.y = Now.linear_y_;
        now.yaw = Now.omega_;

        dt = DWT_GetDeltaT(&DWT_CNT);

        Update_s(Traj_Mode);
        Update_s_ref(Traj_Mode);
        Update_TarjSpeed();
        
        if(Traj_complete_Flag == true){
            Update_TrackSpeed_linear(Traj_s);
        }else {
            Update_TrackSpeed_linear(Traj_s_ref);
        }
        Update_TrackSpeed_omega(Traj_s_ref);

        output_w.vx = Traj_wff.vx + track_w.vx;
        output_w.vy = Traj_wff.vy + track_w.vy;
        output_w.w  = Traj_wff.w  + track_w.w;

        watch_1 = Traj_wff.w;
        watch_2 = track_w.w;
        watch_3 = ref.x;
        watch_4 = ref.y;
        watch_5 = ref.yaw;
        
        s_last = s_now;

        float_t cos_yaw = cosf(Get_predict_yaw()*kDegToRad);
        float_t sin_yaw = sinf(Get_predict_yaw()*kDegToRad);

        output_b.vx = output_w.vx*cos_yaw + output_w.vy*sin_yaw;
        output_b.vy =-output_w.vx*sin_yaw + output_w.vy*cos_yaw;
        output_b.w  = output_w.w;

    }

    // =====================================================
    //  返回最终速度参数
    // =====================================================
    pub_chassis_cmd Get_output_b(){
        pub_chassis_cmd output{};
        output.linear_x_ = output_b.vx;
        output.linear_y_ = output_b.vy;
        output.omega_    = output_b.w;
        return output;
    }

    // =====================================================
    //  设置目标值
    // =====================================================
    void Set_Ref(pub_chassis_cmd Ref,RobotMode_t mode,speed_data speed_In){
        if(fabsf(Ref.linear_x_ - ref.x) > 0.0015f || fabsf(Ref.linear_y_ - ref.y) > 0.0015f ||fabsf(Ref.omega_ - ref.yaw) > 0.001f){
            ref.x = Ref.linear_x_;
            ref.y = Ref.linear_y_;
            ref.yaw = Ref.omega_;
            Traject = speed_In;
            Traj_complete_Flag = false;
            PointTrack_omega_complete_Flag = false;
            PointTrack_linear_complete_Flag = false;
            TrajGenerate(mode);
        }
    }



    // =====================================================
    //  轨迹生成函数
    // =====================================================
    void TrajGenerate(RobotMode_t mode) {

        start = ref_last;
        ref_last = ref;

        if(mode == RobotMode_t::MF){
            if(fabsf(start.x) > 2.1f-0.1f && fabsf(start.x) < 8.1f+0.1f){
                Traj_Mode = MF;
            }else {
                Traj_Mode = Normal;
            }
        }else {
            Traj_Mode = Normal;
        }

        s_ref = 0.0f;
        s_now = 0.0f;
        s_last = 0.0f;

        v_output = 0.0f;

        PID_Reset(&track_path_xy);
        PID_Reset(&track_lateral_xy);
        PID_Reset(&track_omega);

        L = sqrt((ref.x - start.x)*(ref.x - start.x) + (ref.y - start.y)*(ref.y - start.y));
        if(L > 0.001f){
            tx = (ref.x - start.x)/L;
            ty = (ref.y - start.y)/L;
            nx = -ty ;
            ny =  tx ;

            dx_ds = tx;
            dy_ds = ty;
            dyaw_ds = 0.0f;
        }else {
            dx_ds = 0.0f;
            dy_ds = 0.0f;
            dyaw_ds = 0.0f;

            L = 0.001f;//防止后续除零风险
        }
        
    }

    // =====================================================
    //  更新s信息函数
    // =====================================================
    float_t Update_s(RobotMode_t mode){
        //假设P为当前点，A为起始点，B为终点
        float_t dx_PA = now.x - start.x;
        float_t dy_PA = now.y - start.y;

        //判断轨迹是否完成
        if(L - s_now < 0.02f){
            //如果完成就不会更新s，让后续不再进入轨迹
            s_now = L;
            Traj_complete_Flag = true;
        }else{
            //取最近点更新s，但这里实际是根据当前点和起始点、终点的关系来计算的
            s_now = dx_PA*tx + dy_PA*ty;
            if(s_now - s_last > 0.2f){
                s_now = s_last + 0.2f;
            }else if(s_now - s_last < -0.05f){
                s_now = s_last - 0.05f;
            }
        }

        if(s_now < 0.0f){
            s_now = 0.0f;
        }else if(s_now > L){
            s_now = L;
        };

        // 根据新获得的s更新轨迹点
        Traj_s.x   = Get_x_s(s_now);
        Traj_s.y   = Get_y_s(s_now);
        Traj_s_last.x   = Get_x_s(s_last);
        Traj_s_last.y   = Get_y_s(s_last);
        switch (mode){
            case RobotMode_t::Normal:
                Traj_s.yaw = Get_yaw_s(s_now);
                Traj_s_last.yaw = Get_yaw_s(s_last);
                break;
            case RobotMode_t::MF:
                Traj_s_last.yaw = Traj_s.yaw;
                Traj_s.yaw = Get_MF_yaw(Traj_s,s_now);
                break;
            default :
                break;
        }
        
        //防止除零错误
        if(s_now - s_last > 0.001f){
            dx_ds   = (Traj_s.x - Traj_s_last.x)/(s_now - s_last);
            dy_ds   = (Traj_s.y - Traj_s_last.y)/(s_now - s_last);
            // dyaw_ds = Warp_ToRange(Traj_s.yaw - Traj_s_last.yaw,-180.0f,180.0f)/(s_now - s_last);
        }
        return s_now;
    }

    // =====================================================
    //  更新s_ref信息函数
    // =====================================================
    float_t Update_s_ref(RobotMode_t mode){
        float s_ref_last = s_ref;
        float Traj_s_ref_yaw_last = Traj_s_ref.yaw;
        s_ref = s_ref + v_output*dt;

        if(s_ref > s_now + 0.35f){
            s_ref = s_now + 0.35f;
        }

        if(s_ref < s_now - 0.20f){
            s_ref = s_now - 0.20f;
        }
        
        if(s_ref < 0.0f){
            s_ref = 0.0f;
        }else if(s_ref > L){
            s_ref = L;
        };

        // 根据新获得的s_ref更新轨迹点
        Traj_s_ref.x   = Get_x_s(s_ref);
        Traj_s_ref.y   = Get_y_s(s_ref);
        switch (mode){
            case RobotMode_t::Normal:
                Traj_s_ref.yaw = Get_yaw_s(s_ref);
                break;
            case RobotMode_t::MF:
                Traj_s_ref.yaw = Get_MF_yaw(Traj_s_ref,s_ref);
                break;
            default :
                break;
        }
        if(s_ref - s_ref_last > 0.001f){
            dyaw_ds = Warp_ToRange(Traj_s_ref.yaw - Traj_s_ref_yaw_last,-180.0f,180.0f)/(s_ref - s_ref_last);
        }

        return s_ref;
    }

    // =====================================================
    //  轨迹速度更新函数
    // =====================================================
    void Update_TarjSpeed(){

        v_Acc = v_output + Traject.Acc_linear*dt;
        v_Dec = sqrt(2.0f*Traject.Dec_linear*(L - s_now));

        if(Traject.v_Max < v_Acc && Traject.v_Max < v_Dec){
            v_output = Traject.v_Max;
        }else if(v_Acc > v_Dec){
            v_output = v_Dec;
        }else {
            v_output = v_Acc;
        };

        if(fabsf(dyaw_ds)*kDegToRad*v_output > Traject.w_Max && fabsf(dyaw_ds) > 0.01f){
            v_output = Traject.w_Max/(fabsf(dyaw_ds)*kDegToRad);
        }

        Traj_wff.vx = dx_ds*v_output;
        Traj_wff.vy = dy_ds*v_output;
        Traj_wff.w  = dyaw_ds*kDegToRad*v_output;

    }

    // =====================================================
    //  点跟踪平移速度更新函数
    // =====================================================
    void Update_TrackSpeed_linear(chassis_position Ref_tmp){
        float_t e_x = Ref_tmp.x - now.x;
        float_t e_y = Ref_tmp.y - now.y;
        float_t e_xy = sqrt(e_x*e_x + e_y*e_y);
        float_t e_path = tx*e_x + ty*e_y;
        float_t e_lateral = nx*e_x + ny*e_y;
        if(Traj_complete_Flag == true){
            if(e_xy < 0.03f){
                static uint8_t count = 0 ;
                if(count >= 10){
                    PointTrack_linear_complete_Flag = true;
                    count = 0;
                }else {
                    count ++;
                }
            }
        }
        float_t path_output = PID_Calculate(&track_path_xy, 0.0f, e_path);
        float_t lateral_output = PID_Calculate(&track_lateral_xy, 0.0f, e_lateral);
        track_w.vx = path_output*tx + lateral_output*nx;
        track_w.vy = path_output*ty + lateral_output*ny;
    }

    // =====================================================
    //  点跟踪角速度更新函数
    // =====================================================
    void Update_TrackSpeed_omega(chassis_position Ref_tmp){
        float_t e_yaw = Warp_ToRange(Ref_tmp.yaw - now.yaw,-180.0f,180.0f);
        if(Traj_complete_Flag == true){
            if(fabsf(e_yaw) < 0.5f){
                static uint8_t count = 0 ;
                if(count >= 10){
                    PointTrack_omega_complete_Flag = true;
                    count = 0;
                }else {
                    count ++;
                }
            }
        }
        float_t track_omega_pid_output = kDegToRad * PID_Calculate(&track_omega, 0.0f, e_yaw);
        track_w.w = track_omega_pid_output;
    }


    // =====================================================
    //  MF_yaw（s）函数
    // =====================================================
    float_t Get_MF_yaw(chassis_position Traj_s_tmp,float s_tmp){
        float x = Traj_s_tmp.x;
        float y = Traj_s_tmp.y;
        float yaw = Traj_s_tmp.yaw;

        if(fabsf(x - 5.1f*field_side) < 0.1f || fabs(x) <= 2.1f){
            if(y >= 0.1f && y < 0.9f)       {yaw = (-90.0f*field_side)*(sinf(M_PI_2*((2*y-1.0f)/0.8f)) + 1.0f)/2.0f;}
            else if(y >= 0.9f && y < 4.1f)  {yaw = -90.0f*field_side;}
            else if(y >= 4.1f && y <= 4.9f) {yaw = Warp_ToRange(180.0f - (-90.0f*field_side),-180.0f,180.0f)*(sinf(M_PI_2*((2*y-9.0f)/0.8f)) + 1.0f)/2.0f + (-90.0f*field_side);}

        }else if(fabsf(x - 8.1f*field_side) < 0.1f || fabs(x) >= 8.1f){
            if(y >= 0.1f && y < 0.9f)       {yaw = (90.0f*field_side)*(sinf(M_PI_2*((2*y-1.0f)/0.8f)) + 1.0f)/2.0f;}
            else if(y >= 0.9f && y < 4.1f)  {yaw = 90.0f*field_side;}
            else if(y >= 4.1f && y <= 4.9f) {yaw = Warp_ToRange(180.0f - (90.0f*field_side),-180.0f,180.0f)*(sinf(M_PI_2*((2*y-9.0f)/0.8f)) + 1.0f)/2.0f + (90.0f*field_side);}

        }else if(fabsf(x) > 2.1f && fabsf(x) < 8.1f){
            if(fabsf(y - 0.1f) < 0.05f)      {yaw = 0.0f;}
            else if(fabsf(y - 4.9f) < 0.1f) {yaw = 180.0f;}
        }
        return Warp_ToRange(yaw, -180.0f, 180.0f);
    }

    // =====================================================
    //  x（s）函数
    // =====================================================
    float_t Get_x_s(float_t s){
        float_t x_tmp = start.x + s*(ref.x - start.x)/L;
        return x_tmp;
    }

    // =====================================================
    //  y（s）函数
    // =====================================================
    float_t Get_y_s(float_t s){
        float_t y_tmp = start.y + s*(ref.y - start.y)/L;
        return y_tmp;
    }

    // =====================================================
    //  yaw（s）函数
    // =====================================================
    float_t Get_yaw_s(float_t s){
        float_t yaw_tmp = start.yaw + s*(ref.yaw - start.yaw)/L;
        return yaw_tmp;
    }

    float_t s_ref;
    float_t s_now;
    float_t s_last;

    RobotMode_t Traj_Mode = Normal;

    float_t L;

    chassis_position Traj_s_ref{};
    chassis_position Traj_s{};
    chassis_position Traj_s_last{};

    chassis_position ref{};
    chassis_position ref_last{};
    chassis_position start{};
    chassis_position now{};

    float_t tx;
    float_t ty;
    float_t tyaw;

    float_t nx;
    float_t ny;
    float_t nyaw;

    float_t dx_ds;
    float_t dy_ds;
    float_t dyaw_ds;

    speed_data Traject{1.2f,0.9f,2.1f,M_PI*0.6f};

    // float_t Acc_linear = 1.2f;
    // float_t Dec_linear = 0.9f;
    // float_t v_Max = 2.1f;
    // float_t w_Max = M_PI*0.6f;

    float_t v_Acc;
    float_t v_Dec;

    float_t w_Acc;
    float_t w_Dec;

    float_t v_output;

    chassis_speed Traj_wff;
    chassis_speed track_w;
    chassis_speed output_w;
    chassis_speed output_b;

    float_t dt;
    uint32_t DWT_CNT;

    PID_t track_path_xy{.Kp = 4.88f,.Ki = 0.01f,.Kd = 0.55f,.MaxOut = 0.95*MAX_VELOCITY_LINEAR,.DeadBand = 0.005f,.Improve = NONE};
    PID_t track_lateral_xy{.Kp = 3.0f,.Ki = 0.03f,.Kd = 0.35f,.MaxOut = 0.95*MAX_VELOCITY_LINEAR,.DeadBand = 0.005f,.Improve = NONE};
    PID_t track_omega{.Kp = 5.30f,.Ki = 0.1f,.Kd = 0.55f,.MaxOut = MAX_VELOCITY_ANGULAR*0.75*180.0/M_PI,.IntegralLimit = 50000.0f,.DeadBand = 0.1f,.Improve = Integral_Limit};

    
private:
};