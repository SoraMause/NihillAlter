#include "led.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "gpio.h"

///////////////////////////////////////////////////////////////////////
// read push switch(tail) 
// [argument] nothing
// [Substitutiong] nothing
// [return] ON 1 OFF 0
///////////////////////////////////////////////////////////////////////
uint8_t getPushsw( void )
{
  if ( HAL_GPIO_ReadPin( pushsw_GPIO_Port ,pushsw_Pin ) == 0 ){
    return 1;
  } else {
    return 0;
  }
}

///////////////////////////////////////////////////////////////////////
// certain led ( tail )
// [argument] LED_RIGHT_SIDE,LED_LEFT_SIDE,LED_BOTH,LED_OFF
// [Substitutiong] nothing
// [return] nothing
///////////////////////////////////////////////////////////////////////
void certainLedOut( uint8_t led )
{
  uint8_t _led1 = ( ~led & 0x01 );
  uint8_t _led2 = ( ~led & 0x02 ) >> 1;
  uint8_t _led3 = ( ~led & 0x04 ) >> 2;
  uint8_t _led4 = ( ~led & 0x08 ) >> 3;  
  HAL_GPIO_WritePin( led1_GPIO_Port, led1_Pin, _led1 );
  HAL_GPIO_WritePin( led2_GPIO_Port, led2_Pin, _led2 );
  HAL_GPIO_WritePin( led3_GPIO_Port, led3_Pin, _led3 );
  HAL_GPIO_WritePin( led4_GPIO_Port, led4_Pin, _led4 );
}