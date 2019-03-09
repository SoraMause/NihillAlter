#include "mode.h"

#include "stm32f4xx_hal.h"
#include "spi.h"
#include "tim.h"

#include <stdio.h>

#include "run.h"
#include "motion.h"
#include "targetGenerator.h"

#include "timer.h"

#include "maze.h"
#include "agent.h"

#include "buzzer.h"
#include "led.h"
#include "flash.h"
#include "function.h"
#include "logger.h"

#include "mazeRun.h"

// ゴール座標の設定
static uint8_t goal_x = 7;
static uint8_t goal_y = 7;
static uint8_t maze_goal_size = 1;

void modeSelect( int8_t mode )
{

  mode_init();
  log_init();

  switch( mode ){
    case 0:
      mode0();
      break;

    case 1:
      mode1();
      break;

    case 2:
      mode2();
      break;

    case 3:
      mode3();
      break;
    
    case 4:
      mode4();
      break;

    case 5:
      mode5();
      break;
    
    case 6:
      mode6();
      break;

    case 7:
      mode7();
      break;

    case 8:
      mode8();
      break;

    default:
      break;
  }
  adcEnd();
  HAL_Delay( 300 );
  translation_ideal.accel = 0.0;
	translation_ideal.velocity = 0.0;
  setControlFlag( 0 );
  mode_counter = 0;
  mode_distance = 0.0f;
  certainLedOut( 0 );
}

void mode_init( void )
{
  printf("\r\n");
  failSafe_flag = 0;
  mode_distance = 0.0f;
  mode_counter |= 0x80;
	translation_ideal.accel = 0.0;
	translation_ideal.velocity = 0.0;
	translation_ideal.distance = 0.0;
	rotation_ideal.accel = 0.0;
	rotation_ideal.distance = 0.0;
	rotation_ideal.velocity = 0.0;
	rotation_trape_param.back_rightturn_flag = 0;
	rotation_deviation.cumulative = 0.0;
  // to do search param と fast paramで分けれるようにする
  setSlaromOffset( &slarom300, 8.0f, 8.5f, 8.0f, 8.5f, 9000.0f, 720.0f );

  setPIDGain( &translation_gain, 0.8f, 30.0f, 0.0f );  
  setPIDGain( &rotation_gain, 0.9f, 60.0f, 0.0f ); 
  setPIDGain( &sensor_gain, 0.2f, 0.0f, 0.0f );

  // sensor 値設定
  setSensorConstant( &sen_front, 500, 100 );
  // 区画中心　前壁 195
  setSensorConstant( &sen_l, 215, 100 );
  setSensorConstant( &sen_r, 195, 100 );

  certainLedOut( 0x03 );
  waitMotion( 100 );
  certainLedOut( LED_OFF );
  waitMotion( 100 );
  certainLedOut( 0x03 );
  waitMotion( 100 );
  certainLedOut( LED_OFF );
  waitMotion( 100 );
}

void startAction( void )
{
  while( getPushsw() == 0 );
  buzzerSetMonophonic( A_SCALE, 200 );
  HAL_Delay( 300 );
  adcStart();
  while( sen_front.now < 200 );
  buzzerSetMonophonic( C_H_SCALE, 300 );
  adcEnd();
  waitMotion( 500 );
  MPU6500_z_axis_offset_calc_start();
  while( MPU6500_calc_check() ==  0 );
  adcStart();
  waitMotion( 100 );
  setLogFlag( 1 );
  setControlFlag( 1 );
}

void writeFlashData( t_walldata *wall )
{
  certainLedOut( 0x01 );
wall->save = 1;

  for ( int i = 0; i <= MAZE_HALF_MAX_SIZE; i++ ){
    wall->column_known[i] = 0xffffffff;
    wall->row_known[i] = 0xffffffff;
  }

  writeFlash( start_address, (uint8_t*)wall, sizeof( t_walldata ) );
}

void loadWallData( t_walldata *wall )
{
  loadFlash( start_address, (uint8_t *)wall, sizeof( t_walldata ) );
}

// 足立法探索
void mode0( void )
{
  setNormalRunParam( &run_param, 4000.0f, 300.0f );       // 加速度、探索速度指定
  setNormalRunParam( &rotation_param, 5400.0f, 450.0f );  // 角加速度、角速度指定
  wall_Init( &wall_data, MAZE_HALF_MAX_SIZE );
  wallBIt_Init( &wall_bit, MAZE_HALF_MAX_SIZE );
  setMazeGoalSize( maze_goal_size );
  positionReset( &mypos );

  startAction();

  adachiSearchRun( goal_x, goal_y, &run_param, &rotation_param, &wall_data, &wall_bit, &mypos, MAZE_HALF_MAX_SIZE );
  adcEnd();
  setControlFlag( 0 );
  writeFlashData( &wall_bit );
  adcStart();
  setControlFlag( 1 );
  adachiSearchRun( 0, 0, &run_param, &rotation_param, &wall_data, &wall_bit, &mypos, MAZE_HALF_MAX_SIZE );
  adcEnd();
  setControlFlag( 0 );
  writeFlashData( &wall_bit );
  
}

// 足立法最短(斜めなし)
void mode1( void )
{
  setNormalRunParam( &run_param, 4000.0f, 300.0f );       // 加速度、速度指定
  setNormalRunParam( &rotation_param, 6300.0f, 540.0f );  // 角加速度、角速度指定

  loadWallData( &wall_data );
  agentSetShortRoute( goal_x, goal_y, &wall_data, MAZE_HALF_MAX_SIZE, 0, 0 );
  
  positionReset( &mypos );
  startAction();

  adachiFastRun( &run_param, &rotation_param );

  setControlFlag( 0 );
}

// 足立法最短( 斜めあり )
void mode2( void )
{
  int8_t speed_count = 0;
  int8_t _straight = 0;
  int8_t counter = 0;

  loadWallData( &wall_data );
  positionReset( &mypos );

  certainLedOut( 0x0c );
  waitMotion( 300 );
  certainLedOut( LED_OFF );

  mode_distance = 0.0f;

  while( getPushsw() == 0 ){
    printf( "mode_counter = %d, mode_distance = %f\r", counter, mode_distance );

    if ( mode_distance > 30.0f ){
      mode_distance = 0.0f;
      counter++;
      if( counter > 4 ) counter = 0;
      buzzermodeSelect( counter );
      waitMotion( 200 );
      certainLedOut( counter );
    }

    if ( mode_distance < -30.0f ){
      mode_distance = 0.0f;
      counter--;
      if( counter < 0 ) counter = 4;
      buzzermodeSelect( counter );      
      waitMotion( 200 );
      certainLedOut( counter );
    }
  }

  if ( counter % 2 == 0 ){
      _straight = 0;
  } else {
      _straight = 1;
  }
  

  if ( mode_counter < 2 ){
    speed_count = PARAM_400;
    setNormalRunParam( &run_param, 8000.0f, 400.0f );       // 加速度、速度指定
    setNormalRunParam( &rotation_param, 6300.0f, 450.0f );  // 角加速度、角速度指定
    setPIDGain( &translation_gain, 1.4f, 40.0f, 0.0f );   
  } else {
    speed_count = PARAM_500;
    setNormalRunParam( &run_param, 10000.0f, 500.0f );       // 加速度、速度指定
    setNormalRunParam( &rotation_param, 6300.0f, 450.0f );  // 角加速度、角速度指定
    setPIDGain( &translation_gain, 1.6f, 45.0f, 0.0f );   
  }
  

  if ( agentDijkstraRoute( goal_x, goal_y, &wall_data, MAZE_HALF_MAX_SIZE, _straight, speed_count, 0 ) == 0 ){
    return;
  }

  startAction();

  if ( speed_count < 2 ){
    adachiFastRunDiagonal400( &run_param, &rotation_param );
  } else {
    adachiFastRunDiagonal500( &run_param, &rotation_param );
  }
  
  
  // debug 
  waitMotion( 2000 );
  while( getPushsw() == 0 );
  showLog();  

}

// 各種センサー値確認,最短パスチェック
void mode3( void )
{
  loadWallData( &wall_data );
  HAL_Delay( 300 );
  adcStart();
  cnt_motion = 0;
  while( 1 ){
    printf( "fl : %4d, l: %4d, r: %4d, fr: %4d, front : %4d\r",sen_fl.now, sen_l.now, sen_r.now, sen_fr.now, sen_front.now );
    if ( getPushsw() == 1 && cnt_motion >= 2000 ) break;
  }
  adcEnd();
  if ( wall_data.save == 1 ){
    agentSetShortRoute( goal_x, goal_y, &wall_data, MAZE_HALF_MAX_SIZE, 1, 0 );
    agentDijkstraRoute( goal_x, goal_y, &wall_data, MAZE_HALF_MAX_SIZE, 0, PARAM_400, 1 );
  }
  
}

// knwon search
void mode4( void )
{
  setNormalRunParam( &run_param, 4000.0f, 300.0f );       // 加速度、探索速度指定
  setNormalRunParam( &rotation_param, 5400.0f, 450.0f );  // 角加速度、角速度指定
  wall_Init( &wall_data, MAZE_HALF_MAX_SIZE );
  wallBIt_Init( &wall_bit, MAZE_HALF_MAX_SIZE );
  setMazeGoalSize( maze_goal_size );
  positionReset( &mypos );

  startAction();

  adachiSearchRunKnown( goal_x, goal_y, &run_param, &rotation_param, &wall_data, &wall_bit, &mypos, MAZE_HALF_MAX_SIZE );
  adcEnd();
  setControlFlag( 0 );
  writeFlashData( &wall_bit );
  setVirtualGoal( MAZE_HALF_MAX_SIZE, &wall_data );

  mazeUpdateMap( 0, 0, &wall_data, MAZE_HALF_MAX_SIZE );

  if ( checkAllSearch() == 1 ){
    waitMotion( 100 );
    
  } else {
    adcStart();
    setControlFlag( 1 );
    adachiSearchRunKnown( 0, 0, &run_param, &rotation_param, &wall_data, &wall_bit, &mypos, MAZE_HALF_MAX_SIZE );
    adcEnd();
    setControlFlag( 0 );
    writeFlashData( &wall_bit );
  }

  #if 0
  if ( mypos.x == 0 && mypos.y == 0 ){
    setNormalRunParam( &run_param, 8000.0f, 1000.0f );       // 加速度、速度指定
    setNormalRunParam( &rotation_param, 6300.0f, 450.0f );  // 角加速度、角速度指定

    loadWallData( &wall_data );
    
    if ( agentDijkstraRoute( goal_x, goal_y, &wall_data, MAZE_HALF_MAX_SIZE, 0, PARAM_1000, 0 ) == 0 ){
      return;
    }

    positionReset( &mypos );

    buzzerSetMonophonic( C_H_SCALE, 100 );
    HAL_Delay( 110 );
    buzzerSetMonophonic( C_H_SCALE, 100 );
    HAL_Delay( 110 );

    MPU6500_z_axis_offset_calc_start();
    while( MPU6500_calc_check() ==  0 );
    adcStart();
    waitMotion( 100 );
    
    setControlFlag( 1 );

    adachiFastRunDiagonal1000( &run_param, &rotation_param );
  }
  #endif

}

// 直進、超進地チェック
void mode5( void )
{
  startAction();
  sidewall_control_flag = 1;  // 壁制御有効
  adjFront( 4000.0f, 300.0f );
  slaromLeft( 300.0f );
  straightHalfBlockStop( 4000.0f, 300.0f );
  waitStraight();

  setLogFlag( 0 );
  setControlFlag( 0 );
  while( getPushsw() == 0 );
  showLog();
}

// 回転方向のチェック
void mode6( void )
{
  startAction();
  setRotation( 180.0f, 6300.0f, 540.0f, 0.0f );
  waitRotation();
  waitMotion( 300 );
  setRotation( 180.0f, 6300.0f, 540.0f, 0.0f );
  waitRotation();
  setLogFlag( 0 );
  setControlFlag( 0 );
  while( getPushsw() == 0 );
  showLog();
}

// スラロームチェック
void mode7( void )
{
  
  buzzerSetMonophonic( NORMAL, 200 );
  HAL_Delay(300); 
  startAction();
  // turn param
  setNormalRunParam( &rotation_param, 6300.0f, 450.0f );  // 角加速度、角速度指定
  setPIDGain( &translation_gain, 1.6f, 45.0f, 0.0f );  
  
  setStraight( 105.0f, 10000.0f, 500.0f, 0.0f, 500.0f );
  waitStraight();

  setStraight( 90.0f, 10000.0f, 500.0f, 500.0f, 0.0f );
  waitStraight();


  setLogFlag( 0 );
  waitMotion( 300 );
  setControlFlag( 0 );

  while( getPushsw() == 0 );
  showLog();
  
}

// 直進、回転組み合わせチェック 超進地旋回
void mode8( void )
{
  buzzerSetMonophonic( NORMAL, 200 );
  HAL_Delay(300); 
  startAction();
  translation_ideal.velocity = 0.0f;
  rotation_ideal.velocity = 0.0f;
  while( 1 );
}

