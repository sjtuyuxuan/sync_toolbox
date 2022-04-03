extern "C" {
#include "stm32h7xx_hal.h"
#include "string.h"
#include "bsp_usart.h"
#include "bsp_gpio.h"
#include "bsp_systic.h"
}
#include "sync.h"

int count_ = 0;
Channel channel[CHANNEL_MAX];
Channel clock_channel;
Channel start_indicator;
int starting = 0;
int has_start = 0;
int has_set_config = 0;

__IO uint8_t aRxBuffer[MAX_COUNT]={0};
__IO uint16_t RxCount=0;

uint8_t TextSend[1000];

int main(void)
{
  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();
  SystemClock_Config();
  MX_USARTx_Init();
	OUTPUT_GPIO_Init();
  __HAL_UART_ENABLE_IT(&husartx,UART_IT_RXNE);
	
  while (1)
  {
  }
}

void ConfigChannel(Channel *c)
{
  c->width_ /= (1000000 / TICPS);                                                                                        // change unit to 10us
  c->offset_ /= (1000000 / TICPS);                                                                                       // change unit to 10us
  c->gap_ = TICPS / c->freq_;                                                                                     // calculate gap
  c->first_trigger_edge_ = c->offset_ >= 0 ? c->offset_ : c->offset_ + c->gap_;                                          // calculate first raise edge
  c->current_trigger_edge_ = c->offset_ >= 0 ? c->offset_ : TICPS + c->offset_;                                          // calculate current raise edge
  c->first_reset_edge_ = c->offset_ + c->width_ >= 0 ? c->offset_ + c->width_ : c->offset_ + c->width_ + c->gap_;        // calculate first raise edge
  c->current_reset_edge_ = c->offset_ + c->width_ >= 0 ? c->offset_ + c->width_ : TICPS + c->offset_ + c->width_;        // calculate current raise edge
}

extern "C" {
void RxCallback(uint16_t len)
{
  if (len == 3|| len == 4)
  {
    if (has_set_config)
    {
      if ((char)aRxBuffer[1] == 's' && !has_start)
        starting = 1;
      if ((char)aRxBuffer[1] == 'e')
      {
        ; // bip
        if (has_start)
        {
          has_start = 0;
          for (int i = 0; i < CHANNEL_MAX; ++i)
          {
            if (channel[i].enable)
            {
              if (channel[i].mode_ == Channel::FallEdge)
                SetPin(i);
              else
                ResetPin(i);
            }
          }
          if (start_indicator.enable)
          {
            if (start_indicator.mode_ == Channel::RiseEdge)
              START_LOW();
            else if (start_indicator.mode_ == Channel::FallEdge)
              START_HIGH();
            else
            {
              START_HIGH();
              // FIXME:delay width
              START_LOW();
            }
          }
        }
      }
    }
    else
    {
      sprintf((char *)TextSend,"Please Config Before Start\n");
      HAL_UART_Transmit(&husartx,TextSend,strlen((char *)TextSend),1000);
    }
  }
  else if (len == 125 || len == 126 || len == 127)
  {
    // get information
    sscanf((char *)aRxBuffer, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
           &channel[0].enable, (int *)&channel[0].mode_, &channel[0].freq_, &channel[0].offset_, &channel[0].width_,
           &channel[1].enable, (int *)&channel[1].mode_, &channel[1].freq_, &channel[1].offset_, &channel[1].width_,
           &channel[2].enable, (int *)&channel[2].mode_, &channel[2].freq_, &channel[2].offset_, &channel[2].width_,
           &channel[3].enable, (int *)&channel[3].mode_, &channel[3].freq_, &channel[3].offset_, &channel[3].width_,
           &clock_channel.enable, (int *)&clock_channel.mode_, &clock_channel.freq_, &clock_channel.offset_, &clock_channel.width_,
           &start_indicator.enable, (int *)&start_indicator.mode_, &start_indicator.freq_, &start_indicator.offset_, &start_indicator.width_);
    // calculate
    for (size_t i = 0; i < CHANNEL_MAX; i++)
    {
      ConfigChannel(&channel[i]);
    }
    ConfigChannel(&clock_channel);
    ConfigChannel(&start_indicator);

    // init
    if (start_indicator.enable)
    {
      if (start_indicator.mode_ == Channel::RiseEdge || start_indicator.mode_ == Channel::DoubleEdge)
        START_LOW();
      if (start_indicator.mode_ == Channel::FallEdge)
        START_HIGH();
    }
    if (clock_channel.enable)
    {
      if (clock_channel.mode_ == Channel::RiseEdge || clock_channel.mode_ == Channel::DoubleEdge)
        CLOCK_LOW();
      if (clock_channel.mode_ == Channel::FallEdge)
        CLOCK_HIGH();
    }
    if (channel[0].enable)
    {
      if (channel[0].mode_ == Channel::RiseEdge || channel[0].mode_ == Channel::DoubleEdge)
        CHANNEL1_LOW();
      if (channel[0].mode_ == Channel::FallEdge)
        CHANNEL1_HIGH();
    }
    if (channel[1].enable)
    {
      if (channel[1].mode_ == Channel::RiseEdge || channel[1].mode_ == Channel::DoubleEdge)
        CHANNEL2_LOW();
      if (channel[1].mode_ == Channel::FallEdge)
        CHANNEL2_HIGH();
    }
    if (channel[2].enable)
    {
      if (channel[2].mode_ == Channel::RiseEdge || channel[2].mode_ == Channel::DoubleEdge)
        CHANNEL3_LOW();
      if (channel[2].mode_ == Channel::FallEdge)
        CHANNEL3_HIGH();
    }
    if (channel[3].enable)
    {
      if (channel[3].mode_ == Channel::RiseEdge || channel[3].mode_ == Channel::DoubleEdge)
        CHANNEL4_LOW();
      if (channel[3].mode_ == Channel::FallEdge)
        CHANNEL4_HIGH();
    }

    // debug
    sprintf((char *)TextSend, "The channel 1 is %d\nthe mode is: %d\nThe freq is: %d\nThe offset is: %d\nThe width is: %d\n Gap is: %d\n First edge is: %d\n Current edge is :%d\n",
            channel[0].enable, channel[0].mode_, channel[0].freq_, channel[0].offset_, channel[0].width_, channel[0].gap_, channel[0].first_trigger_edge_, channel[0].current_trigger_edge_);
    HAL_UART_Transmit(&husartx,TextSend,strlen((char *)TextSend),2000);
    sprintf((char *)TextSend, "The channel 2 is %d\nthe mode is: %d\nThe freq is: %d\nThe offset is: %d\nThe width is: %d\n Gap is: %d\n First edge is: %d\n Current edge is :%d\n",
            channel[1].enable, channel[1].mode_, channel[1].freq_, channel[1].offset_, channel[1].width_, channel[1].gap_, channel[1].first_trigger_edge_, channel[1].current_trigger_edge_);
    HAL_UART_Transmit(&husartx,TextSend,strlen((char *)TextSend),2000);
    sprintf((char *)TextSend, "The channel 3 is %d\nthe mode is: %d\nThe freq is: %d\nThe offset is: %d\nThe width is: %d\n Gap is: %d\n First edge is: %d\n Current edge is :%d\n",
            channel[2].enable, channel[2].mode_, channel[2].freq_, channel[2].offset_, channel[2].width_, channel[2].gap_, channel[2].first_trigger_edge_, channel[2].current_trigger_edge_);
    HAL_UART_Transmit(&husartx,TextSend,strlen((char *)TextSend),2000);
    sprintf((char *)TextSend, "The channel 4 is %d\nthe mode is: %d\nThe freq is: %d\nThe offset is: %d\nThe width is: %d\n Gap is: %d\n First edge is: %d\n Current edge is :%d\n",
            channel[3].enable, channel[3].mode_, channel[3].freq_, channel[3].offset_, channel[3].width_, channel[3].gap_, channel[3].first_trigger_edge_, channel[3].current_trigger_edge_);
    HAL_UART_Transmit(&husartx,TextSend,strlen((char *)TextSend),2000);
    sprintf((char *)TextSend, "The clock channel is %d\nthe mode is: %d\nThe freq is: %d\nThe offset is: %d\nThe width is: %d\n Gap is: %d\n First edge is: %d\n Current edge is :%d\n",
            clock_channel.enable, clock_channel.mode_, clock_channel.freq_, clock_channel.offset_, clock_channel.width_, clock_channel.gap_, clock_channel.first_trigger_edge_, clock_channel.current_trigger_edge_);
    HAL_UART_Transmit(&husartx,TextSend,strlen((char *)TextSend),2000);
    sprintf((char *)TextSend, "The start indicator channel is %d\nthe mode is: %d\nThe freq is: %d\nThe offset is: %d\nThe width is: %d\n Gap is: %d\n First edge is: %d\n Current edge is :%d\n",
            start_indicator.enable, start_indicator.mode_, start_indicator.freq_, start_indicator.offset_, start_indicator.width_, start_indicator.gap_, start_indicator.first_trigger_edge_, start_indicator.current_trigger_edge_);
    HAL_UART_Transmit(&husartx,TextSend,strlen((char *)TextSend),2000);
    has_set_config = 1;
  }
  else
  {
      sprintf((char *)TextSend,"WRONG LENGTH PLZ CHECK THE INPUT%d\n",len);
      HAL_UART_Transmit(&husartx,TextSend,strlen((char *)TextSend),1000);
  }
}
}
