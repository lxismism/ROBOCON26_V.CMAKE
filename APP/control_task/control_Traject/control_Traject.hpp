#include "main.h"
#include "control_task.h"
#include "control_process.hpp"
#include "pid_controller.h"
#include "chassis_task.h"
#include "topic_pool.h"
#include "topics.hpp"
#include <cmath>
#include <cstdint>
#include "pick_hand.hpp"
#include "weapon_hand.hpp"
#include "lift.hpp"
#include "omni_ir.hpp"

class TrajectChassis{
public:

    // =====================================================
    //  轨迹运行函数
    // =====================================================
    void Run(pub_chassis_cmd Now) {
        now.x = Now.linear_x_;
        now.y = Now.linear_y_;
        now.yaw = Now.omega_;

        if(Ref_change_Flag == true){
            TrajGenerate();
            Ref_change_Flag = false;
            Traj_complete_Flag = false;
        }

        if(Traj_complete_Flag == false){
            Update_s();
        }else {
            s_now = L;
        }
        
        Update_TarjSpeed();
        Update_TrackSpeed(Traj_s);

        output_w.vx = Traj_wff.vx + track_w.vx;
        output_w.vy = Traj_wff.vy + track_w.vy;
        output_w.w = Traj_wff.w + track_w.w;

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
    void Set_Ref(pub_chassis_cmd Ref){
        if(fabsf(Ref.linear_x_ - ref.x) > 0.001f || fabsf(Ref.linear_y_ - ref.y) > 0.001f ||fabsf(Ref.omega_ - ref.yaw) > 0.001f){
            ref.x = Ref.linear_x_;
            ref.y = Ref.linear_y_;
            ref.yaw = Ref.omega_;
            Ref_change_Flag = true;
        }
    }

private:

    // =====================================================
    //  轨迹生成函数
    // =====================================================
    void TrajGenerate() {

        start = ref_last;
        ref_last = ref;

        s_now = 0.0f;
        s_last = 0.0f;

        v_output = 0.0f;

        PID_Reset(&track_xy);
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
        }
        
    }

    // =====================================================
    //  轨迹速度更新函数
    // =====================================================
    void Update_TarjSpeed(){
        if(s_now < 0.0f){
            s_now = 0.0f;
        }else if(s_now > L){
            s_now = L;
        };
        // v_Acc = sqrt(2.0f*Acc*s_now);
        Acc_dt = DWT_GetDeltaT(&Acc_DWT_CNT);
        v_Acc = v_output + Acc*Acc_dt;
        v_Dec = sqrt(2.0f*Dec*(L - s_now));
        if(v_Max < v_Acc && v_Max < v_Dec){
            v_output = v_Max;
        }else if(v_Acc > v_Dec){
            v_output = v_Dec;
        }else {
            v_output = v_Acc;
        };
        Traj_wff.vx = dx_ds*v_output;
        Traj_wff.vy = dy_ds*v_output;
        Traj_wff.w  = dyaw_ds*kDegToRad*v_output;
    }

    // =====================================================
    //  点跟踪速度更新函数
    // =====================================================
    void Update_TrackSpeed(chassis_position Ref_tmp){
        float_t e_x = Ref_tmp.x - now.x;
        float_t e_y = Ref_tmp.y - now.y;
        float_t e_xy = sqrt(e_x*e_x + e_y*e_y);
        float_t e_yaw = Warp_ToRange(Ref_tmp.yaw - now.yaw,-180.0f,180.0f);

        float_t track_xy_pid_output = PID_Calculate(&track_xy, 0.0f, e_xy);
        float_t track_omega_pid_output = kDegToRad * PID_Calculate(&track_omega, 0.0f, e_yaw);

        if(e_xy > 0.005f){
            track_w.vx = track_xy_pid_output * (e_x/e_xy);
            track_w.vy = track_xy_pid_output * (e_y/e_xy);
        }else {
            track_w.vx = 0.0f;
            track_w.vy = 0.0f; 
        }

        track_w.w = track_omega_pid_output;
    }

    // =====================================================
    //  更新s信息函数
    // =====================================================
    float_t Update_s(){
        //假设P为当前点，A为起始点，B为终点
        float_t dx_PA = now.x - start.x;
        float_t dy_PA = now.y - start.y;

        // float dx_BA = ref.x - start.x;
        // float dy_BA = ref.y - start.y;

        //取最近点更新s，但这里实际是根据当前点和起始点、终点的关系来计算的
        s_now = dx_PA*tx + dy_PA*ty;
        if(s_now - s_last > 0.2f){
            s_now = s_last + 0.2f;
        }else if(s_now - s_last < -0.05f){
            s_now = s_last - 0.05f;
        }

        //判断轨迹是否完成
        if(L - s_now < 0.05f){
            s_now = L;
            Traj_complete_Flag = true;
        }

        // 根据新获得的s更新轨迹点
        Traj_s.x   = Get_x_s(s_now);
        Traj_s.y   = Get_y_s(s_now);
        Traj_s.yaw = Get_yaw_s(s_now);

        Traj_s_last.x   = Get_x_s(s_last);
        Traj_s_last.y   = Get_y_s(s_last);
        Traj_s_last.yaw = Get_yaw_s(s_last);

        //防止除零错误
        if(s_now - s_last > 0.001f){
            dx_ds   = (Traj_s.x - Traj_s_last.x)/(s_now - s_last);
            dy_ds   = (Traj_s.y - Traj_s_last.y)/(s_now - s_last);
            dyaw_ds = Warp_ToRange(Traj_s.yaw - Traj_s_last.yaw,-180.0f,180.0f)/(s_now - s_last);
        }
        
        s_last = s_now;

        return s_now;
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
        float_t yaw_tmp;
        if(s < L/2){
            yaw_tmp = start.yaw;
        }else {
            yaw_tmp = start.yaw + (s - L/2)*(ref.yaw - start.yaw)/(L/2);
        }
        return yaw_tmp;
    }
    
    float_t s_now;
    float_t s_last;

    float_t L;

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

    const float_t Acc = 2.5f;
    const float_t Dec = 1.8f;
    const float_t v_Max = 1.5f;
    float_t v_Acc;
    float_t v_Dec;
    float_t v_output;

    chassis_speed Traj_wff;
    chassis_speed track_w;
    chassis_speed output_w;
    chassis_speed output_b;

    bool Ref_change_Flag = false;
    
    bool Traj_complete_Flag = true;

    float_t Acc_dt;
    uint32_t Acc_DWT_CNT;

    PID_t track_xy{.Kp = 4.68f,.Ki = 0.03f,.Kd = 0.75f,.MaxOut = 0.95*MAX_VELOCITY_LINEAR,.DeadBand = 0.005f,.Improve = NONE};
    PID_t track_omega{.Kp = 2.10f,.Ki = 0.22f,.Kd = 0.08f,.MaxOut = MAX_VELOCITY_ANGULAR*0.75*180.0/M_PI,.IntegralLimit = 50000.0f,.DeadBand = 0.1f,.Improve = Integral_Limit};

};