#include "motion.h"

#include "run.h"

#include "maze.h"

void setSlaromOffset( t_slarom_parameter *slarom, float left_in, float left_out, float right_in, 
                      float right_out, float ang_accel, float max_ang_vel )
{
  slarom->left.in = left_in;
  slarom->left.out = left_in;
  slarom->right.in = right_in;
  slarom->right.out = right_out;
  slarom->angular_accel = ang_accel;
  slarom->max_angular_velocity = max_ang_vel;
}

void setNormalRunParam( t_normal_param *param, float accel, float max_vel )
{
  param->accel = accel;
  param->velocity = max_vel;
}

void adjFront( float accel, float run_vel )
{ 
  wall_out_flag = 1;          // 壁切れを読むことを許可
  sidewall_control_flag = 1;  // 壁制御有効
  setStraight( ADJ_FRONT_DISTANCE, accel, run_vel, 0.0f, run_vel );
  waitStraight();
}

void adjBack( void )
{
  setStraight( -25.0f, 2000.0f, 100.0f, 0.0f, 0.0f );
  waitStraight();
}

void straightOneBlock( float run_vel )
{
  wall_out_flag = 2;          // 壁切れを読むことを許可
  sidewall_control_flag = 1;  // 壁制御有効
  setStraight( ONE_BLOCK_DISTANCE, 0.0f, run_vel, run_vel, run_vel );
  waitStraight();
}

void straightHalfBlockStop( float accel , float run_vel )
{
  sidewall_control_flag = 1;    // 壁制御有効
  frontwall_control_flag = 1;
  setStraight( HALF_BLOCK_DISTANCE, accel, run_vel, run_vel, 0.0f );
  waitStraight(); 
  waitMotion( 300 );
}

void straightHalfBlockStart( float accel , float run_vel )
{
  sidewall_control_flag = 1;    // 壁制御有効
  frontwall_control_flag = 1;
  setStraight( HALF_BLOCK_DISTANCE, accel, run_vel, 0.0f, run_vel );
  waitStraight(); 
}

void pivoTurnLeft( float accel, float run_vel )
{
  setRotation( 90.0f, accel, run_vel, 0.0f );
  waitRotation();
  waitMotion( 300 );
}

void pivoTurnRight( float accel, float run_vel )
{
  setRotation( -90.0f, accel, run_vel, 0.0f );
  waitRotation();
  waitMotion( 300 );
}

void pivoTurn180( float accel, float run_vel )
{
  setRotation( 180.0f, accel, run_vel, 0.0f );
  waitRotation();
  waitMotion( 300 );
}

void slaromLeft( float run_vel )
{
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( slarom300.left.in, 0.0f, run_vel, run_vel, run_vel );
  waitStraight();
  setRotation( 90.0f, slarom300.angular_accel, slarom300.max_angular_velocity, run_vel );
  waitRotation();
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( slarom300.left.out, 0.0f, run_vel, run_vel, run_vel );
  waitSlaromOut();
}

void slaromRight( float run_vel )
{
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( slarom300.right.in, 0.0f, run_vel, run_vel, run_vel );
  waitStraight();
  setRotation( -90.0f, slarom300.angular_accel, slarom300.max_angular_velocity, run_vel );
  waitRotation();
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( slarom300.right.out, 0.0f, run_vel, run_vel, run_vel );
  waitSlaromOut();
}

// to do 最短のやつを作りまくる
void runStraight( float accel , float distance, float start_vel, float run_vel, float end_vel )
{
  setStraight( distance, accel, run_vel, start_vel, end_vel );
  waitStraight(); 
}

// 中心から90度
void slaromCenterLeft( float accel )
{
  sidewall_control_flag = 1;    // 壁制御有効
  while( sen_l.now > sen_l.threshold );
  translation_ideal.distance = 8.0f;
  setStraight( 41.0f, 8000.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
  setRotation( 90.0f, 10000.0f, 700.0f, 400.0f );
  waitRotation();
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 46.0f, 0.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
}

void slaromCenterRight( float accel )
{
  sidewall_control_flag = 1;    // 壁制御有効
  while( sen_r.now > sen_r.threshold );
  translation_ideal.distance = 8.8f;
  setStraight( 41.0f, 8000.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
  setRotation( -90.0f, 10000.0f, 700.0f, 400.0f );
  waitRotation();
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 46.0f, 0.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
}

// 中心から180度
void slaromCenterLeft180( float accel )
{
  sidewall_control_flag = 1;    // 壁制御有効
  while( sen_l.now > sen_l.threshold );
  translation_ideal.distance = 8.0f;
  setStraight( 30.0f, 8000.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
  setRotation( 180.0f, 12000.0f, 510.0f, 400.0f );
  waitRotation();
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 33.0f, 0.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
}

void slaromCenterRight180( float accel )
{
  sidewall_control_flag = 1;    // 壁制御有効
  while( sen_r.now > sen_r.threshold );
  translation_ideal.distance = 8.0f;
  setStraight( 30.0f, 8000.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
  setRotation( -180.0f, 12000.0f, 510.0f, 400.0f );
  waitRotation();
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 33.0f, 0.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
}

// 中心から45度
void slaromCenterLeft45( float accel )
{
  sidewall_control_flag = 1;    // 壁制御有効
  while( sen_l.now > sen_l.threshold );
  translation_ideal.distance = 8.0f;
  setStraight( 21.0f, 8000.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
  setRotation( 45.0f, 16000.0f, 700.0f, 400.0f );
  waitRotation();
  dirwall_control_flag = 1;
  setStraight( 42.0f, 0.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
}

void slaromCenterRight45( float accel )
{
  sidewall_control_flag = 1;    // 壁制御有効
  while( sen_r.now > sen_r.threshold );
  translation_ideal.distance = 8.0f;
  setStraight( 21.0f, 8000.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
  setRotation( -45.0f, 16000.0f, 700.0f, 400.0f );
  waitRotation();
  dirwall_control_flag = 1;
  setStraight( 42.0f, 0.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
}

// 中心から135度
void slaromCenterLeft135( float accel )
{
  sidewall_control_flag = 1;    // 壁制御有効
  while( sen_l.now > sen_l.threshold );
  translation_ideal.distance = 8.0f;
  setStraight( 42.0f, 8000.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
  setRotation( 135.0f, 12000.0f, 700.0f, 400.0f );
  waitRotation();
  dirwall_control_flag = 1;
  setStraight( 38.0f, 0.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
}

void slaromCenterRight135( float accel )
{
  sidewall_control_flag = 1;    // 壁制御有効
  while( sen_r.now > sen_r.threshold );
  translation_ideal.distance = 8.8f;
  setStraight( 42.0f, 8000.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
  setRotation( -135.0f, 12000.0f, 700.0f, 400.0f );
  waitRotation();
  dirwall_control_flag = 1;
  setStraight( 38.0f, 0.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
}

// 斜め90度 ( V90 )
void slaromLeftV90( void )
{
  dirwall_control_flag = 1;
  while( sen_l.now > sen_l.threshold && translation_ideal.distance < 15.0f );
  if ( translation_ideal.distance < 15.0f ){
    translation_ideal.distance = 2.0f;
  }
  
  setStraight( 21.0f, 0.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
  setRotation( 90.0f, 14000.0f, 800.0f, 400.0f );
  waitRotation();
  dirwall_control_flag = 1;
  setStraight( 27.0f, 0.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();  
}

void slaromRightV90( void )
{
  dirwall_control_flag = 1;
  while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
  if ( translation_ideal.distance < 15.0f ){
    translation_ideal.distance = 3.2f;
  }
  setStraight( 21.0f, 0.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
  setRotation( -90.0f, 14000.0f, 800.0f, 400.0f );
  waitRotation();
  dirwall_control_flag = 1;
  setStraight( 27.0f, 0.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();  
}

// 斜めから復帰
void slaromReturnDiaLeft45( void )
{
  dirwall_control_flag = 1;
  while( sen_l.now > sen_l.threshold && translation_ideal.distance < 15.0f );
  if ( translation_ideal.distance < 15.0f ){
    translation_ideal.distance = 2.0f;
  }
  setStraight( 36.0f, 8000.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
  setRotation( 45.0f, 12000.0f, 700.0f, 400.0f );
  waitRotation();
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 21.0f, 0.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
}

void slaromReturnDiaRight45( void )
{
  dirwall_control_flag = 1;
  while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
  if ( translation_ideal.distance < 15.0f ){
    translation_ideal.distance = 3.2f;
  }
  setStraight( 36.0f, 8000.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
  setRotation( -45.0f, 12000.0f, 700.0f, 400.0f );
  waitRotation();
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 21.0f, 0.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
}

// 斜めから135度ターン復帰
void slaromReturnDiaLeft135( void )
{
  dirwall_control_flag = 1;
  while( sen_l.now > sen_l.threshold && translation_ideal.distance < 15.0f );
  if ( translation_ideal.distance < 15.0f ){
    translation_ideal.distance = 2.0f;
  }
  setStraight( 34.0f, 8000.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
  setRotation( 135.0f, 12000.0f, 700.0f, 400.0f );
  waitRotation();
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 46.0f, 0.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
}

void slaromReturnDiaRight135( void )
{
  dirwall_control_flag = 1;
  while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
  if ( translation_ideal.distance < 15.0f ){
    translation_ideal.distance = 3.2f;
  }
  
  setStraight( 34.0f, 8000.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
  setRotation( -135.0f, 12000.0f, 700.0f, 400.0f );
  waitRotation();
  sidewall_control_flag = 1;    // 壁制御有効
  setStraight( 46.0f, 0.0f, 400.0f, 400.0f, 400.0f );
  waitStraight();
}