#ifndef __SYNC_H__
#define __SYNC_H__
#include "bsp_gpio.h"

#define CHANNEL_MAX 4
typedef struct
{
  //def of TriggerMode
  int offset_;               // pulse offset
  int first_trigger_edge_;   //first raise edge in one second
  int current_trigger_edge_; //current raise edge in one second
  int first_reset_edge_;     //first reset edge in one second
  int current_reset_edge_;   //current reset edge in one second
  int width_;                // pulse width
  int gap_;                  // 1/freq
  int freq_;                 // freq
  int down_mod_, up_mod_;
  enum TriggerMode
  {
    RiseEdge = 0,
    FallEdge = 1,
    DoubleEdge = 2
  } mode_; // for start indicator RiseEdge means high gate FallEdge means low gate DoubleEdge means Double Paulse
  int enable;
} Channel;

inline void TogPin(int i)
{
  switch (i)
  {
  case 0:
    CHANNEL1_TOGGLE();
    break;
  case 1:
    CHANNEL2_TOGGLE();
    break;
  case 2:
    CHANNEL3_TOGGLE();
    break;
  case 3:
    CHANNEL4_TOGGLE();
    break;
  default:
    break;
  }
}

inline void SetPin(int i)
{
  switch (i)
  {
  case 0:
    CHANNEL1_HIGH();
    break;
  case 1:
    CHANNEL2_HIGH();
    break;
  case 2:
    CHANNEL3_HIGH();
    break;
  case 3:
    CHANNEL4_HIGH();
    break;
  default:
    break;
  }
}

inline void ResetPin(int i)
{
  switch (i)
  {
  case 0:
    CHANNEL1_LOW();
    break;
  case 1:
    CHANNEL2_LOW();
    break;
  case 2:
    CHANNEL3_LOW();
    break;
  case 3:
    CHANNEL4_LOW();
    break;
  default:
    break;
  }
}


extern int count_;
extern Channel channel[CHANNEL_MAX];
extern Channel clock_channel;
extern Channel start_indicator;
extern int starting;
extern int has_start;
extern int has_set_config;

#endif // __SYNC_H__