#include "targetGenerator.h"

static float sidewall_control_value = 0.0f;
static float frontwall_control_value = 0.0f;
static uint8_t sen_diff = 3;

static float low_pass_velocity = 0.0f;
static float velocity_present = 0.0f;

void setPIDGain( t_PID_param *param, float kp, float ki, float kd )
{
  param->kp = kp;
  param->ki = ki;
  param->kd = kd;  
}

void setSenDiff( uint8_t _diff )
{
  sen_diff = _diff;
}

void PIDControl( t_run *ideal, t_run *left, t_run *right, t_deviation *left_deviation, t_deviation *right_deviation, 
                t_PID_param *gain, t_trapezoid *trape, t_duty *duty, int8_t rotation_control )
{
  int32_t duty_left, duty_right;
  float left_error, right_error;
  float kp, ki, kd;
  float left_p, right_p;
  float left_i, right_i;
  float left_d, right_d;
  kp = gain->kp;
  ki = gain->ki;
  kd = gain->kd;

  if ( trape->back_rightturn_flag == 1 ){
    left->velocity = -1.0f * ( left->velocity + right->velocity ) / 2.0f;
    right->velocity = left->velocity;
  } else {
    left->velocity = ( left->velocity + right->velocity ) / 2.0f;
    right->velocity = left->velocity;
  }

  if ( rotation_control == 0 ){
    low_pass_velocity = right->velocity * 0.95f + velocity_present * 0.05f;
    velocity_present = right->velocity;
  } else {
    low_pass_velocity = 0.0f;
  }

  

  // 壁制御を入力
  if ( rotation_control == 1 ){
    left->velocity += sidewall_control_value;
    right->velocity += sidewall_control_value;
  } else {
    //left->velocity += frontwall_control_value;
    //right->velocity += frontwall_control_value;
    low_pass_velocity += frontwall_control_value;
  }

  // 現在の偏差を計算 
  if ( rotation_control == 1 ){
    left_error = ( ideal->velocity - left->velocity );
    right_error = ( ideal->velocity - right ->velocity );
  } else {
    left_error = ideal->velocity - low_pass_velocity;
    right_error = ideal->velocity - low_pass_velocity;
  }
  

  left_p = left_error * kp;
  right_p = right_error * kp;

  // 積分値を計算
  left_deviation->cumulative += left_error * dt;
  right_deviation->cumulative += right_error * dt;

  left_i = left_deviation->cumulative * ki;
  right_i = right_deviation->cumulative * ki;

  left_d = ( left_error - left_deviation->difference ) * kd;
  right_d = ( right_error - right_deviation->difference ) * kd;

  // 一つ前の値を計算( 微分用 )
  left_deviation->difference = left_error;
  right_deviation->difference = right_error;

  duty_left = (int32_t)(left_p + left_i + left_d);
  duty_right = (int32_t)(right_p + right_i + right_d);

  if ( rotation_control == 1 ){
    duty_left = -1 * duty_left; // 回転方向逆にする 
  }

  if ( trape->back_rightturn_flag == 1 ){
    duty_left = -1 * duty_left;
    duty_right = -1 * duty_right;
  }

  duty->left += duty_left;
  duty->right += duty_right;

}

void sideWallControl( void )
{
  // sidewall_control_valueをいじる
  // kp については無理やり角速度になおせるような値を求める。
  // 横壁制御フラグが1のときのみ制御を行う
  float sen_error = 0.0f;

  if ( sidewall_control_flag == 1 && (sen_l.diff < sen_diff) && (sen_r.diff < sen_diff) && (translation_ideal.velocity > 100.0f) ){
    if ( sen_l.is_wall == 1 && sen_r.is_wall == 1 ){
      sen_error = (float)( ( sen_l.now - sen_l.reference ) - ( sen_r.now - sen_r.reference ) );
      if ( sen_error > 100.0f ){
        sen_error = 100.0f;
      } else if ( sen_error < -100.0f ){
        sen_error = -100.0f;
      }
      sidewall_control_value = sensor_gain.kp * sen_error;
    } else if ( sen_l.is_wall == 1 && sen_r.is_wall == 0 ){
      sen_error = (float)( sen_l.now - sen_l.reference );
      if ( sen_error > 100.0f ){
        sen_error = 100.0f;
      } else if ( sen_error < -100.0f ){
        sen_error = -100.0f;
      }
      sidewall_control_value = 2.0f * sensor_gain.kp * sen_error;
    } else if( sen_r.is_wall == 1 && sen_l.is_wall == 0 ){
      sen_error = (float)( sen_r.now - sen_r.reference );
      if ( sen_error > 100.0f ){
        sen_error = 100.0f;
      } else if ( sen_error < -100.0f ){
        sen_error = -100.0f;
      }
      sidewall_control_value = -2.0f * sensor_gain.kp * sen_error;
    } else {
      sidewall_control_value = 0.0f;
    }
  } else if ( dirwall_control_flag == 1 ){
    sidewall_control_value = 0.0f;
    // 4つのセンサのそれぞれの値の閾値を決めてそれに対して制御量を気持ち与える。
    // 2019 1/25 fl: , l: , fr: , r:
    if ( sen_fl.now > 110 && sen_fl.diff < 50 ){
      sidewall_control_value = (float)0.6f * ( sen_fl.now - 90 );
    } else if ( sen_l.now > 700 && sen_l.diff_1ms < 70 ){
      sidewall_control_value = (float)0.6f * ( sen_l.now - 660 );
    } else if ( sen_fr.now > 100 && sen_fr.diff < 50 ){
      sidewall_control_value = (float)-0.6f * ( sen_fr.now - 80 );
    } else if ( sen_r.now > 700 && sen_r.diff_1ms < 70 ){
      sidewall_control_value = (float)-0.6f * ( sen_r.now - 660 );
    }
  } else {
    sidewall_control_value = 0.0f;
  }

}

void frontWallControl( void )
{
  // 前壁については普通にゲイン調整して合わせること。
  // 前壁制御フラグが1のときのみ制御を行う
  if ( frontwall_control_flag == 1 && sen_front.is_wall == 1 && right_real.velocity < 200.0f ){
    frontwall_control_value = (float) 0.5f * (sen_front.now - sen_front.reference);
  } else {
    frontwall_control_value = 0.0f;
  }
}

void feedForwardTranslation( float left_vel, float right_vel, float accel, float velocity, t_duty *duty, float vBat, uint8_t backright_flag )
{
  float center_vel = 0.0f; 
  float motor_reverse_v = 0.0f;
  float out_power = 0.0f;
  float out_duty = 0.0f;
  float real_accel = 0.0f;

  if ( accel != 0.0f ){
    // 中心速度を求め、mm/s から m/s に変換する
    center_vel = (left_vel + right_vel) / 2000.0f;
    
    // モーターの逆起電力を計算する
    // モーターの回転数 * 逆起電力定数
    //motor_reverse_v = 30.0f * GEAR_RATION * center_vel / (3.141592f * MACHINE_WHEEL_RADIUS ) * MOTOR_REVERSE_VOLTAGE_CONSTANT; 
    motor_reverse_v = 0.44404229122f * center_vel;

    real_accel = accel / 1000.0f; // accel mm/ss から m/ssに変更
    // 並進方向の計算
    //out_power = MACHINE_WHEEL_RADIUS * MACHINE_WEIGHT * accel / ( 2.0f * GEAR_RATION); 0.00011333333f
    // out_powerに摩擦力を考慮した値を追加で足している 
    // 0.3 m/s: duty 42 0.5 m/s: 66, 1.0 m/s: 220
    if ( accel > 0.0f ){
      out_power = (real_accel + 2.7f) * 0.00011333333f;
    } else if ( accel == 0.0f ){
      out_power = 0.0f;
    } else {
      out_power = real_accel * 0.00011333333f;
    }
    
    // 出力すべき値の計算
    //out_duty = ( MOTOR_RESISTOR * out_power / MOTOR_TORQUE_CONSTANT+  motor_reverse_v ) / Vbat;
    if ( backright_flag == 0 ){
      out_duty = ( 857.58f * out_power + motor_reverse_v ) / vBat;
    } else {
      out_duty = 0.0f;
    }
    
    duty->left += (int32_t)(out_duty * 400);
    duty->right += (int32_t)(out_duty * 400);
  }
  
}