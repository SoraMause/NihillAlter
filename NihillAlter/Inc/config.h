#ifndef __CONFIG_H
#define __CONFIG_H

// macro define
#define dt 0.001f
//---------------------------------------------------------------------
// マシンデータ( 物理パラメータ )
//---------------------------------------------------------------------
#define MACHINE_WHEEL_RADIUS      0.00675f
// マシンのトレッド
#define MACHINE_TREAD_WIDTH       0.0195f
// マシンの重さ
#define MACHINE_WEIGHT            0.020f
// ギア比
#define GEAR_RATION               4.33f // 39 / 9 = 4.333

// タイヤが一回転するまでのエンコーダの値
#define MACHINE_ENC_CNT_PER_ROT   4096

#endif /* __CONFIG_H */