#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

// led の光らせ方
#define LED_OFF         0x00

uint8_t getPushsw( void );
void certainLedOut( uint8_t led );

#ifdef __cplusplus
 }
#endif

#endif /* __LED_H */