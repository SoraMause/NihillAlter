#include "search.h"

#include "maze.h"
#include "agent.h"

#include "tim.h"

#include "variable.h"

#include "mode.h"

#include "motion.h"
#include "run.h"
#include "timer.h"
#include "buzzer.h"
#include "led.h"
#include "logger.h"

#define SEARCH_MAX_TIME 300000

void adachiSearchRun( int8_t gx, int8_t gy, t_normal_param *translation, t_normal_param *rotation, t_walldata *wall, t_walldata *bit, t_position *pos, uint8_t maze_scale )
{

  int8_t next_dir = front;

  // もし、ゴール座標が探索ならマシンの動作時間を0にする。 
  if ( gx != 0 && gy != 0 ){
    cnt_act = 0;
  }

  mazeUpdatePosition( front, pos );
  adjFront( translation->accel, translation->velocity );
  while( pos->x != gx || pos->y != gy ){
    addWall( pos, wall );
    addWall( pos, bit ); 
    mazeUpdateMap( gx, gy, wall, maze_scale );
    next_dir = getNextDir( pos->direction,pos->x, pos->y, wall, maze_scale );

    switch( next_dir ){
      case front:
        mazeUpdatePosition( front, pos );
        straightOneBlock( translation->velocity );
        break;

      case left:
        mazeUpdatePosition( left, pos );
        slaromLeft( translation->velocity );
        //straightHalfBlockStop( translation->accel, translation->velocity );
        //pivoTurnLeft( rotation->accel, rotation->velocity );
        //straightHalfBlockStart( translation->accel, translation->velocity );
        break;

      case right:
        mazeUpdatePosition( right, pos );
        slaromRight( translation->velocity );
        //straightHalfBlockStop( translation->accel, translation->velocity );
        //pivoTurnRight( rotation->accel, rotation->velocity );
        //straightHalfBlockStart( translation->accel, translation->velocity );
        break;

      case rear:
        straightHalfBlockStop( translation->accel, translation->velocity );
        mazeUpdatePosition( rear, pos );
        pivoTurn180( rotation->accel, rotation->velocity );
        adjBack();
        adjFront( translation->accel, translation->velocity );
        break;

      case pivo_rear:
        straightHalfBlockStop( translation->accel, translation->velocity );
        mazeUpdatePosition( rear, pos );
        pivoTurn180( rotation->accel, rotation->velocity );
        straightHalfBlockStart( translation->accel, translation->velocity );
        break;
    }

    // 探索時間が2分30秒以上たっていた場合打ち切り。
    if( cnt_act > SEARCH_MAX_TIME && gx == 0 && gy == 0 ) break;
  }
    
  addWall( pos, wall );
  addWall( pos, bit ); 
  straightHalfBlockStop( translation->accel, translation->velocity );
  waitMotion( 300 );
  if ( mypos.x == gx && mypos.y == gy ){
    pivoTurn180( rotation->accel, rotation->velocity );
    adjBack();
    mypos.direction = (mypos.direction + 2) % 4;
  } 
  
  setControlFlag( 0 );
  buzzerSetMonophonic( C_H_SCALE, 100 );
  waitMotion( 150 );
}

void adachiSearchRunKnown( int8_t gx, int8_t gy, t_normal_param *translation, t_normal_param *rotation, t_walldata *wall, t_walldata *bit, t_position *pos, uint8_t maze_scale )
{
  int8_t next_dir = front;
  int8_t block = 0;

  // もし、ゴール座標が探索ならマシンの動作時間を0にする。 
  if ( gx != 0 && gy != 0 ){
    cnt_act = 0;
  }

  mazeUpdatePosition( front, pos );
  adjFront( translation->accel, translation->velocity );
  while( pos->x != gx || pos->y != gy ){
    addWall( pos, wall );
    addWall( pos, bit ); 
    mazeUpdateMap( gx, gy, wall, maze_scale );
    next_dir = getNextDirKnown( pos->direction,pos->x, pos->y, wall, bit, maze_scale );

    if ( next_dir == front ){
        mazeUpdatePosition( front, pos );
        straightOneBlock( translation->velocity );
    } else if ( next_dir == left ){
        mazeUpdatePosition( left, pos );
        slaromLeft( translation->velocity );
    } else if ( next_dir == right ){
        mazeUpdatePosition( right, pos );
        slaromRight( translation->velocity );
    } else if ( next_dir == rear ){
      straightHalfBlockStop( translation->accel, translation->velocity );
      mazeUpdatePosition( rear, pos );
      pivoTurn180( rotation->accel, rotation->velocity );
      adjBack();
      adjFront( translation->accel, translation->velocity );
    } else if ( next_dir == pivo_rear ){
      straightHalfBlockStop( translation->accel, translation->velocity );
      mazeUpdatePosition( rear, pos );
      pivoTurn180( rotation->accel, rotation->velocity );
      straightHalfBlockStart( translation->accel, translation->velocity );
    } else if ( next_dir > 10 ){
        sidewall_control_flag = 1;  // 壁制御有効
        block = next_dir - 10;
        mazeUpdatePosition( next_dir, pos );
        setStraight( ONE_BLOCK_DISTANCE * block,translation->accel, translation->velocity + 100.0f, translation->velocity, translation->velocity );
        waitSearchStraight();
    }

    // 探索時間が2分30秒以上たっていた場合打ち切り。
    if( cnt_act > SEARCH_MAX_TIME && gx == 0 && gy == 0 ) break;

    if ( checkAllSearch() == 1 && gx == 0 && gy == 0 ) break;

  }
    
  addWall( pos, wall );
  addWall( pos, bit ); 
  straightHalfBlockStop( translation->accel, translation->velocity );
  if ( mypos.x == gx && mypos.y == gy && sen_fl.is_wall == 1 ){
    pivoTurn180( rotation->accel, rotation->velocity );
    adjBack();
    mypos.direction = (mypos.direction + 2) % 4;
  } else {
    adjBack();
  }
  setControlFlag( 0 );
  buzzerSetMonophonic( C_H_SCALE, 100 );
  waitMotion( 150 );
}

void adachiFastRun( t_normal_param *translation, t_normal_param *rotation )
{
  while( motion_queue[motion_last] != END ){
    switch( motion_queue[motion_last] ){
      case SET_STRAIGHT:
        sidewall_control_flag = 1;  // 壁制御有効
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );
        break;

      case SLAROM_LEFT:
        slaromLeft( translation->velocity );
        break;

      case SLAROM_RIGHT:
        slaromRight( translation->velocity );
        break;

      case FRONTPD_DELAY:
        waitMotion( 300 );
        break;

      case SET_FRONT_PD_STRAIGHT:
        // 前壁制御有効にする
        frontwall_control_flag = 1;
        sidewall_control_flag = 1;
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );        
        break;
    } // end switch 
    motion_last++;
  }

  waitMotion( 100 );
  buzzerSetMonophonic( NORMAL, 100 );
  waitMotion( 100 );
}

void adachiFastRunDiagonal400( t_normal_param *translation, t_normal_param *rotation )
{
  
  setControlFlag( 0 );
  setLogFlag( 0 );
  waitMotion( 1000 );
  setLogFlag( 1 );
  setControlFlag( 1 );
  
  while( motion_queue[motion_last] != END ){
    switch( motion_queue[motion_last] ){
      case SET_STRAIGHT:
        certainLedOut( LED_OFF );
        sidewall_control_flag = 1;  // 壁制御有効
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );
        break;

      case SET_DIA_STRAIGHT:
        certainLedOut( 0x01 );
        dirwall_control_flag = 1;
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );
        dirwall_control_flag = 0;
        break;

      // 中心から90度
      case CENRTER_SLAROM_LEFT:
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
        break;

      case CENRTER_SLAROM_RIGHT:
        sidewall_control_flag = 1;    // 壁制御有効
        while( sen_r.now > sen_r.threshold );
        translation_ideal.distance = 8.4f;
        setStraight( 41.0f, 8000.0f, 400.0f, 400.0f, 400.0f );
        waitStraight();
        setRotation( -90.0f, 10000.0f, 700.0f, 400.0f );
        waitRotation();
        sidewall_control_flag = 1;    // 壁制御有効
        setStraight( 46.0f, 0.0f, 400.0f, 400.0f, 400.0f );
        waitStraight();
        break;

      // 中心から180度
      case SLAROM_LEFT_180:
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
        break;

      case SLAROM_RIGHT_180:
        sidewall_control_flag = 1;    // 壁制御有効
        while( sen_r.now > sen_r.threshold );
        translation_ideal.distance = 8.4f;
        setStraight( 30.0f, 8000.0f, 400.0f, 400.0f, 400.0f );
        waitStraight();
        setRotation( -180.0f, 12000.0f, 510.0f, 400.0f );
        waitRotation();
        sidewall_control_flag = 1;    // 壁制御有効
        setStraight( 33.0f, 0.0f, 400.0f, 400.0f, 400.0f );
        waitStraight();
        break;

      // 中心から45度
      case DIA_CENTER_LEFT:
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
        break;

      case DIA_CENTER_RIGHT:
        sidewall_control_flag = 1;    // 壁制御有効
        while( sen_r.now > sen_r.threshold );
        translation_ideal.distance = 8.4f;
        setStraight( 21.0f, 8000.0f, 400.0f, 400.0f, 400.0f );
        waitStraight();
        setRotation( -45.0f, 16000.0f, 700.0f, 400.0f );
        waitRotation();
        dirwall_control_flag = 1;
        setStraight( 42.0f, 0.0f, 400.0f, 400.0f, 400.0f );
        waitStraight();
        break;

      // 中心から135度
      case DIA_CENTER_LEFT_135:
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
        break;

      case DIA_CENTER_RIGHT_135:
        sidewall_control_flag = 1;    // 壁制御有効
        while( sen_r.now > sen_r.threshold );
        translation_ideal.distance = 8.4f;
        setStraight( 42.0f, 8000.0f, 400.0f, 400.0f, 400.0f );
        waitStraight();
        setRotation( -135.0f, 12000.0f, 700.0f, 400.0f );
        waitRotation();
        dirwall_control_flag = 1;
        setStraight( 38.0f, 0.0f, 400.0f, 400.0f, 400.0f );
        waitStraight();
        break;

      // 斜め90度 ( V90 )
      case DIA_LEFT_TURN:
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
        break;

      case DIA_RIGHT_TURN:
        dirwall_control_flag = 1;
        while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
        if ( translation_ideal.distance < 15.0f ){
          translation_ideal.distance = 2.8f;
        }
        setStraight( 21.0f, 0.0f, 400.0f, 400.0f, 400.0f );
        waitStraight();
        setRotation( -90.0f, 14000.0f, 800.0f, 400.0f );
        waitRotation();
        dirwall_control_flag = 1;
        setStraight( 27.0f, 0.0f, 400.0f, 400.0f, 400.0f );
        waitStraight();  
        break;

      // 斜めから復帰
      case RETURN_DIA_LEFT:
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
        break;

      case RETURN_DIA_RIGHT:
        dirwall_control_flag = 1;
        while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
        if ( translation_ideal.distance < 15.0f ){
          translation_ideal.distance = 2.8f;
        }
        setStraight( 36.0f, 8000.0f, 400.0f, 400.0f, 400.0f );
        waitStraight();
        setRotation( -45.0f, 12000.0f, 700.0f, 400.0f );
        waitRotation();
        sidewall_control_flag = 1;    // 壁制御有効
        setStraight( 21.0f, 0.0f, 400.0f, 400.0f, 400.0f );
        waitStraight();
        break;

      // 斜めから135度ターン復帰
      case RETURN_DIA_LEFT_135:
        dirwall_control_flag = 1;
        while( sen_l.now > sen_l.threshold && translation_ideal.distance < 15.0f );
        if ( translation_ideal.distance < 15.0f ){
          translation_ideal.distance = 2.8f;
        }
        setStraight( 34.0f, 8000.0f, 400.0f, 400.0f, 400.0f );
        waitStraight();
        setRotation( 135.0f, 12000.0f, 700.0f, 400.0f );
        waitRotation();
        sidewall_control_flag = 1;    // 壁制御有効
        setStraight( 46.0f, 0.0f, 400.0f, 400.0f, 400.0f );
        waitStraight();
        break;

      case RETURN_DIA_RIGHT_135:
        dirwall_control_flag = 1;
        while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
        if ( translation_ideal.distance < 15.0f ){
          translation_ideal.distance = 2.8f;
        }
        
        setStraight( 34.0f, 8000.0f, 400.0f, 400.0f, 400.0f );
        waitStraight();
        setRotation( -135.0f, 12000.0f, 700.0f, 400.0f );
        waitRotation();
        sidewall_control_flag = 1;    // 壁制御有効
        setStraight( 46.0f, 0.0f, 400.0f, 400.0f, 400.0f );
        waitStraight();
        break;

      case FRONTPD_DELAY:
        // 前壁制御有効にする
        frontwall_control_flag = 1;
        waitMotion( 100 );
        break;

      case DELAY:
        waitMotion( 50 );
        break;

      case SET_FRONT_PD_STRAIGHT:
        // 前壁制御有効にする
        frontwall_control_flag = 1;
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );        
        break;

      default:
        break;
    } // end switch 
    motion_last++;
  }

  buzzerSetMonophonic( NORMAL, 100 );
  setControlFlag( 0 );
  waitMotion( 100 );
}

void adachiFastRunDiagonal500( t_normal_param *translation, t_normal_param *rotation )
{
  
  setControlFlag( 0 );
  setLogFlag( 0 );
  waitMotion( 1000 );
  setLogFlag( 1 );
  setControlFlag( 1 );
  
  while( motion_queue[motion_last] != 0 ){
    switch( motion_queue[motion_last] ){
      case SET_STRAIGHT:
        certainLedOut( LED_OFF );
        sidewall_control_flag = 1;  // 壁制御有効
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );
        break;

      case SET_DIA_STRAIGHT:
        certainLedOut( 0x01 );
        dirwall_control_flag = 1;
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );
        dirwall_control_flag = 0;
        break;

      // 中心から90度
      case CENRTER_SLAROM_LEFT:
        certainLedOut( LED_OFF );
        sidewall_control_flag = 1;    // 壁制御有効
        while( sen_l.now > sen_l.threshold );
        translation_ideal.distance = 7.5f;
        sidewall_control_flag = 1;
        setStraight( 29.0f, translation->accel, 500.0f, 500.0f, 500.0f );
        waitStraight();
        setRotation( 90.0f, 10000.0f, 700.0f, 500.0f );
        waitRotation();
        sidewall_control_flag = 1;    // 壁制御有効
        setStraight( 35.0f, 0.0f, 500.0f, 500.0f, 500.0f );
        waitStraight();
        break;

      case CENRTER_SLAROM_RIGHT:
        sidewall_control_flag = 1;    // 壁制御有効
        while( sen_r.now > sen_r.threshold );
        translation_ideal.distance = 8.0f;
        sidewall_control_flag = 1;
        setStraight( 29.0f, translation->accel, 500.0f, 500.0f, 500.0f );
        waitStraight();
        setRotation( -90.0f, 10000.0f, 700.0f, 500.0f );
        waitRotation();
        sidewall_control_flag = 1;    // 壁制御有効
        setStraight( 35.0f, 0.0f, 500.0f, 500.0f, 500.0f );
        waitStraight();
        break;

      // 中心から180度
      case SLAROM_LEFT_180:
        certainLedOut( LED_OFF );
        sidewall_control_flag = 1;    // 壁制御有効
        while( sen_l.now > sen_l.threshold );
        translation_ideal.distance = 7.5f;
        setStraight( 30.0f, translation->accel, 500.0f, 500.0f, 500.0f );
        waitStraight();
        setRotation( 180.0f, 12000.0f, 650.0f, 500.0f );
        waitRotation();
        sidewall_control_flag = 1;    // 壁制御有効
        setStraight( 34.0f, 0.0f, 500.0f, 500.0f, 500.0f );
        waitStraight();
        break;

      case SLAROM_RIGHT_180:
        certainLedOut( LED_OFF );
        sidewall_control_flag = 1;    // 壁制御有効
        while( sen_r.now > sen_r.threshold );
        translation_ideal.distance = 8.0f;
        setStraight( 30.0f, translation->accel, 500.0f, 500.0f, 500.0f );
        waitStraight();
        setRotation( -180.0f, 12000.0f, 650.0f, 500.0f );
        waitRotation();
        sidewall_control_flag = 1;    // 壁制御有効
        setStraight( 34.0f, 0.0f, 500.0f, 500.0f, 500.0f );
        waitStraight();
        break;

      // 中心から45度
      case DIA_CENTER_LEFT:
        certainLedOut( LED_OFF );
        sidewall_control_flag = 1;    // 壁制御有効
        while( sen_l.now > sen_l.threshold );
        translation_ideal.distance = 7.5f;
        setStraight( 15.5f, translation->accel, 500.0f, 500.0f, 500.0f );
        waitStraight();
        setRotation( 45.0f, 16000.0f, 750.0f, 500.0f );
        waitRotation();
        dirwall_control_flag = 1;
        setStraight( 38.0f, 0.0f, 500.0f, 500.0f, 500.0f );
        waitStraight();
        break;

      case DIA_CENTER_RIGHT:
        certainLedOut( LED_OFF );
        sidewall_control_flag = 1;    // 壁制御有効
        while( sen_r.now > sen_r.threshold );
        translation_ideal.distance = 8.0f;
        setStraight( 15.5f, translation->accel, 500.0f, 500.0f, 500.0f );
        waitStraight();
        setRotation( -45.0f, 16000.0f, 750.0f, 500.0f );
        waitRotation();
        dirwall_control_flag = 1;
        setStraight( 38.0f, 0.0f, 500.0f, 500.0f, 500.0f );
        waitStraight();
        break;

      // 中心から135度
      case DIA_CENTER_LEFT_135:
        certainLedOut( LED_OFF );
        sidewall_control_flag = 1;    // 壁制御有効
        while( sen_l.now > sen_l.threshold );
        translation_ideal.distance = 7.5f;
        setStraight( 33.0f, translation->accel, 500.0f, 500.0f, 500.0f );
        waitStraight();
        setRotation( 135.0f, 16000.0f, 800.0f, 500.0f );
        waitRotation();
        dirwall_control_flag = 1;
        setStraight( 31.0f, 0.0f, 500.0f, 500.0f, 500.0f );
        waitStraight();
        break;

      case DIA_CENTER_RIGHT_135:
        certainLedOut( LED_OFF );
        sidewall_control_flag = 1;    // 壁制御有効
        while( sen_r.now > sen_r.threshold );
        translation_ideal.distance = 8.0f;
        setStraight( 33.0f, translation->accel, 500.0f, 500.0f, 500.0f );
        waitStraight();
        setRotation( -135.0f, 16000.0f, 800.0f, 500.0f );
        waitRotation();
        dirwall_control_flag = 1;
        setStraight( 31.0f, 0.0f, 500.0f, 500.0f, 500.0f );
        waitStraight();
        break;

      // 斜め90度 ( V90 )
      case DIA_LEFT_TURN:
        certainLedOut( 0x03 );
        dirwall_control_flag = 1;
        while( sen_l.now > sen_l.threshold && translation_ideal.distance < 10.0f );
        if ( translation_ideal.distance < 10.0f ){
          translation_ideal.distance = 2.0f;
        }
        
        setStraight( 13.0f, 0.0f, 500.0f, 500.0f, 500.0f );
        waitStraight();
        setRotation( 90.0f, 16000.0f, 800.0f, 500.0f );
        waitRotation();
        dirwall_control_flag = 1;
        setStraight( 19.0f, 0.0f, 500.0f, 500.0f, 500.0f );
        waitStraight();  
        break;

      case DIA_RIGHT_TURN:
        certainLedOut( 0x03 );
        dirwall_control_flag = 1;
        while( sen_r.now > sen_r.threshold && translation_ideal.distance < 10.0f );
        if ( translation_ideal.distance < 10.0f ){
          translation_ideal.distance = 3.0f;
        }
        setStraight( 13.0f, 0.0f, 500.0f, 500.0f, 500.0f );
        waitStraight();
        setRotation( -90.0f, 16000.0f, 800.0f, 500.0f );
        waitRotation();
        dirwall_control_flag = 1;
        setStraight( 19.0f, 0.0f, 500.0f, 500.0f, 500.0f );
        waitStraight();  
        break;

      // 斜めから復帰
      case RETURN_DIA_LEFT:
        certainLedOut( LED_OFF );
        dirwall_control_flag = 1;
        while( sen_l.now > sen_l.threshold && translation_ideal.distance < 15.0f );
        if ( translation_ideal.distance < 15.0f ){
          translation_ideal.distance = 2.0f;
        }
        setStraight( 34.0f, 8000.0f, 500.0f, 500.0f, 500.0f );
        waitStraight();
        setRotation( 45.0f, 16000.0f, 800.0f, 500.0f );
        waitRotation();
        sidewall_control_flag = 1;    // 壁制御有効
        setStraight( 19.0f, 0.0f, 500.0f, 500.0f, 500.0f );
        waitStraight();
        break;

      case RETURN_DIA_RIGHT:
        certainLedOut( LED_OFF );
        dirwall_control_flag = 1;
        while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
        if ( translation_ideal.distance < 15.0f ){
          translation_ideal.distance = 3.0f;
        }
        setStraight( 34.0f, 8000.0f, 500.0f, 500.0f, 500.0f );
        waitStraight();
        setRotation( -45.0f, 16000.0f, 800.0f, 500.0f );
        waitRotation();
        sidewall_control_flag = 1;    // 壁制御有効
        setStraight( 19.0f, 0.0f, 500.0f, 500.0f, 500.0f );
        waitStraight();
        break;

      // 斜めから135度ターン復帰
      case RETURN_DIA_LEFT_135:
        certainLedOut( LED_OFF );
        dirwall_control_flag = 1;
        while( sen_l.now > sen_l.threshold && translation_ideal.distance < 15.0f );
        if ( translation_ideal.distance < 15.0f ){
          translation_ideal.distance = 2.0f;
        }
        setStraight( 25.0f, 8000.0f, 500.0f, 500.0f, 500.0f );
        waitStraight();
        setRotation( 135.0f, 16000.0f, 800.0f, 500.0f );
        waitRotation();
        sidewall_control_flag = 1;    // 壁制御有効
        setStraight( 38.0f, 0.0f, 500.0f, 500.0f, 500.0f );
        waitStraight();
        break;

      case RETURN_DIA_RIGHT_135:
        certainLedOut( LED_OFF );
        dirwall_control_flag = 1;
        while( sen_r.now > sen_r.threshold && translation_ideal.distance < 15.0f );
        if ( translation_ideal.distance < 15.0f ){
          translation_ideal.distance = 3.0f;
        }
        
        setStraight( 25.0f, 8000.0f, 500.0f, 500.0f, 500.0f );
        waitStraight();
        setRotation( -135.0f, 16000.0f, 800.0f, 500.0f );
        waitRotation();
        sidewall_control_flag = 1;    // 壁制御有効
        setStraight( 38.0f, 0.0f, 500.0f, 500.0f, 500.0f );
        waitStraight();
        break;

      case FRONTPD_DELAY:
        // 前壁制御有効にする
        frontwall_control_flag = 1;
        waitMotion( 100 );
        break;

      case DELAY:
        waitMotion( 50 );
        break;

      case SET_FRONT_PD_STRAIGHT:
        // 前壁制御有効にする
        frontwall_control_flag = 1;
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );        
        break;

      default:
        break;
    } // end switch 
    motion_last++;
  }

  buzzerSetMonophonic( NORMAL, 100 );
  setControlFlag( 0 );
  setLogFlag( 0 );
  waitMotion( 100 );
}

void adachiFastRunDiagonal600( t_normal_param *translation, t_normal_param *rotation )
{
  
  setControlFlag( 0 );
  setLogFlag( 0 );
  waitMotion( 1000 );
  setLogFlag( 1 );
  setControlFlag( 1 );
  
  while( motion_queue[motion_last] != 0 ){
    switch( motion_queue[motion_last] ){
      case SET_STRAIGHT:
        certainLedOut( LED_OFF );
        sidewall_control_flag = 1;  // 壁制御有効
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );
        break;

      case SET_DIA_STRAIGHT:
        certainLedOut( 0x01 );
        dirwall_control_flag = 1;
        if ( motion_queue[motion_last+1] == DIA_LEFT_TURN || motion_queue[motion_last+1] == DIA_RIGHT_TURN ){
          runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                              fast_path[motion_last].speed, 500.0f );
        } else {
          runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );
        }
        break;

      // 中心から90度
      case CENRTER_SLAROM_LEFT:
        certainLedOut( LED_OFF );
        slaromCenterLeft( translation->accel );
        break;

      case CENRTER_SLAROM_RIGHT:
        certainLedOut( LED_OFF );
        slaromCenterRight( translation->accel );
        break;

      // 中心から180度
      case SLAROM_LEFT_180:
        certainLedOut( LED_OFF );
        slaromCenterLeft180( translation->accel );
        break;

      case SLAROM_RIGHT_180:
        certainLedOut( LED_OFF );
        slaromCenterRight180( translation->accel );
        break;

      // 中心から45度
      case DIA_CENTER_LEFT:
        certainLedOut( LED_OFF );
        slaromCenterLeft45( translation->accel );
        break;

      case DIA_CENTER_RIGHT:
        certainLedOut( LED_OFF );
        slaromCenterRight45( translation->accel );
        break;

      // 中心から135度
      case DIA_CENTER_LEFT_135:
        certainLedOut( LED_OFF );
        slaromCenterLeft135( translation->accel );
        break;

      case DIA_CENTER_RIGHT_135:
        certainLedOut( LED_OFF );
        slaromCenterRight135( translation->accel );
        break;

      // 斜め90度 ( V90 )
      case DIA_LEFT_TURN:
        certainLedOut( 0x03 );
        slaromLeftV90();
        break;

      case DIA_RIGHT_TURN:
        certainLedOut( 0x03 );
        slaromRightV90();
        break;

      // 斜めから復帰
      case RETURN_DIA_LEFT:
        certainLedOut( LED_OFF );
        slaromReturnDiaLeft45();
        break;

      case RETURN_DIA_RIGHT:
        certainLedOut( LED_OFF );
        slaromReturnDiaRight45();
        break;

      // 斜めから135度ターン復帰
      case RETURN_DIA_LEFT_135:
        certainLedOut( LED_OFF );
        slaromReturnDiaLeft135();
        break;

      case RETURN_DIA_RIGHT_135:
        certainLedOut( LED_OFF );
        slaromReturnDiaRight135();
        break;

      case FRONTPD_DELAY:
        // 前壁制御有効にする
        frontwall_control_flag = 1;
        waitMotion( 100 );
        break;

      case DELAY:
        waitMotion( 50 );
        break;

      case SET_FRONT_PD_STRAIGHT:
        // 前壁制御有効にする
        frontwall_control_flag = 1;
        runStraight( translation->accel, fast_path[motion_last].distance, fast_path[motion_last].start_speed, 
                    fast_path[motion_last].speed, fast_path[motion_last].end_speed );        
        break;

      default:
        break;
    } // end switch 
    motion_last++;
  }

  buzzerSetMonophonic( NORMAL, 100 );
  setControlFlag( 0 );
  setLogFlag( 0 );
  waitMotion( 100 );
}