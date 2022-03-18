#include "NU32.h" // constants, funcs for startup and UART
#include <stdio.h>
#include <string.h>

#define MSG_LEN 200
#define CHANNEL_MAX 4
#define SECOND_COUNT 100000
#define CLPIN LATFbits.LATF0
#define SIPIN LATFbits.LATF1
#define C1PIN LATEbits.LATE0
#define C2PIN LATEbits.LATE1
#define C3PIN LATEbits.LATE2
#define C4PIN LATEbits.LATE3

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

int count_ = 0;
Channel channel[CHANNEL_MAX];
Channel clock_channel;
Channel start_indicator;
int starting = 0;
int has_start = 0;
int has_set_config = 0;

void SetupInterrupts(void);
void SetupOutput(void);
void WaitInstruction(void);
void ConfigChannel(Channel *c);
void SetPin(int i);
void ResetPin(int i);
void InvPin(int i);

void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1ISR(void)
{ // INT step 1: the ISR
  if (count_ == SECOND_COUNT)
  {
    count_ = 0;
  }

  if (count_ == SECOND_COUNT / 2 && starting)
  {
    NU32_WriteUART3("Start\n");
    starting = 0;
    has_start = 1;
  }
  if (start_indicator.enable)
  {
    if (count_ == start_indicator.current_trigger_edge_ && has_start)
    {
      if (start_indicator.mode_ == RiseEdge || start_indicator.mode_ == DoubleEdge)
        SIPIN = 1; //set bit
      if (start_indicator.mode_ == FallEdge)
        SIPIN = 0; //reset bit
    }
  }

  if (clock_channel.enable)
  {
    if (count_ == clock_channel.current_trigger_edge_)
    {
      clock_channel.current_trigger_edge_ += clock_channel.gap_;
      if (clock_channel.current_trigger_edge_ + clock_channel.gap_ > SECOND_COUNT + clock_channel.first_trigger_edge_)
        clock_channel.current_trigger_edge_ = clock_channel.first_trigger_edge_;
      if (clock_channel.mode_ == DoubleEdge)
        CLPIN = !CLPIN;
      else if (clock_channel.mode_ == RiseEdge)
        CLPIN = 1;
      else
        CLPIN = 0;
    }
    if (count_ == clock_channel.current_reset_edge_)
    {
      clock_channel.current_reset_edge_ += clock_channel.gap_;
      if (clock_channel.current_reset_edge_ + clock_channel.gap_ > SECOND_COUNT + clock_channel.first_reset_edge_)
        clock_channel.current_reset_edge_ = clock_channel.first_reset_edge_;
      if (clock_channel.mode_ == RiseEdge)
        CLPIN = 0;
      else if (clock_channel.mode_ == FallEdge)
        CLPIN = 1;
    }
  }
  if (has_start)
    for (int i = 0; i < CHANNEL_MAX; ++i)
    {
      if (channel[i].enable)
      {
        if (count_ == channel[i].current_trigger_edge_)
        {
          channel[i].current_trigger_edge_ += channel[i].gap_;
          if (channel[i].current_trigger_edge_ + channel[i].gap_ > SECOND_COUNT + channel[i].first_trigger_edge_)
            channel[i].current_trigger_edge_ = channel[i].first_trigger_edge_;
          if (channel[i].mode_ == DoubleEdge)
            InvPin(i);
          else if (channel[i].mode_ == RiseEdge)
            SetPin(i);
          else
            ResetPin(i);
        }
        if (count_ == channel[i].current_reset_edge_)
        {
          channel[i].current_reset_edge_ += channel[i].gap_;
          if (channel[i].current_reset_edge_ + channel[i].gap_ > SECOND_COUNT + channel[i].first_reset_edge_)
            channel[i].current_reset_edge_ = channel[i].first_reset_edge_;
          if (channel[i].mode_ == RiseEdge)
            ResetPin(i);
          else if (channel[i].mode_ == FallEdge)
            SetPin(i);
        }
      }
    }

  count_ += 1;
  IFS0bits.T1IF = 0; // clear interrupt flag
}

int main(void)
{
  NU32_Startup();    // cache on, interrupts on, LED/button init, UART init
  SetupInterrupts(); // setup interrupts
  SetupOutput();
  while (1)
  {
    WaitInstruction();
  }
  return 0;
}

void SetupInterrupts(void)
{
  __builtin_disable_interrupts(); // disable interrupts at CPU
  PR1 = 99;
  TMR1 = 0;
  T1CONbits.TCKPS = 1;           // prescaler to 8
  T1CONbits.TGATE = 0;           //             not gated input (the default)
  T1CONbits.TCS = 0;             //             PCBLK input (the default)
  T1CONbits.ON = 1;              //             turn on Timer1
  IPC1bits.T1IP = 5;             // INT step 4: priority
  IPC1bits.T1IS = 0;             //             subpriority
  IFS0bits.T1IF = 0;             // INT step 5: clear interrupt flag
  IEC0bits.T1IE = 1;             // INT step 6: enable interrupt
  __builtin_enable_interrupts(); // INT step 7: enable interrupts at CPU
}

void SetupOutput(void)
{
  TRISE = 0x0000;   // set E0-3 as digital output
  ODCESET = 0x0000; // No open drain port
}

void WaitInstruction(void)
{
  char msg[MSG_LEN];
  char msg_send[MSG_LEN];
  NU32_ReadUART3(msg, MSG_LEN);
  if (strlen(msg) == 3)
  {
    if (has_set_config)
    {
      if (msg[1] == 's' && !has_start)
        starting = 1;
      if (msg[1] == 'e')
      {
        ; // bip
        if (has_start)
        {
          has_start = 0;
          for (int i = 0; i < CHANNEL_MAX; ++i)
          {
            if (channel[i].enable)
            {
              if (channel[i].mode_ == FallEdge)
                SetPin(i);
              else
                ResetPin(i);
            }
          }
          if (start_indicator.enable)
          {
            if (start_indicator.mode_ == RiseEdge)
              SIPIN = 0;
            else if (start_indicator.mode_ == FallEdge)
              SIPIN = 1;
            else
            {
              SIPIN = 1;
              // FIXME:delay width
              SIPIN = 0;
            }
          }
        }
      }
    }
    else
    {
      NU32_WriteUART3("Please Config Before Start\n");
    }
  }
  else if (strlen(msg) == 125 || strlen(msg) == 126)
  {
    memset(msg_send, 0, sizeof msg_send);
    sprintf(msg_send, "%d", strlen(msg));
    NU32_WriteUART3(msg_send);
    // get information
    sscanf(msg, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
           &channel[0].enable, &channel[0].mode_, &channel[0].freq_, &channel[0].offset_, &channel[0].width_,
           &channel[1].enable, &channel[1].mode_, &channel[1].freq_, &channel[1].offset_, &channel[1].width_,
           &channel[2].enable, &channel[2].mode_, &channel[2].freq_, &channel[2].offset_, &channel[2].width_,
           &channel[3].enable, &channel[3].mode_, &channel[3].freq_, &channel[3].offset_, &channel[3].width_,
           &clock_channel.enable, &clock_channel.mode_, &clock_channel.freq_, &clock_channel.offset_, &clock_channel.width_,
           &start_indicator.enable, &start_indicator.mode_, &start_indicator.freq_, &start_indicator.offset_, &start_indicator.width_);
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
      if (start_indicator.mode_ == RiseEdge || start_indicator.mode_ == DoubleEdge)
        SIPIN = 0; //reset bit
      if (start_indicator.mode_ == FallEdge)
        SIPIN = 1; //set bit
    }
    if (clock_channel.enable)
    {
      if (clock_channel.mode_ == RiseEdge || clock_channel.mode_ == DoubleEdge)
        CLPIN = 0; //reset bit
      if (clock_channel.mode_ == FallEdge)
        CLPIN = 1; //set bit
    }
    if (channel[0].enable)
    {
      if (channel[0].mode_ == RiseEdge || channel[0].mode_ == DoubleEdge)
        C1PIN = 0; //reset bit
      if (channel[0].mode_ == FallEdge)
        C1PIN = 1; //set bit
    }
    if (channel[1].enable)
    {
      if (channel[1].mode_ == RiseEdge || channel[1].mode_ == DoubleEdge)
        C2PIN = 0; //reset bit
      if (channel[1].mode_ == FallEdge)
        C2PIN = 1; //set bit
    }
    if (channel[2].enable)
    {
      if (channel[2].mode_ == RiseEdge || channel[2].mode_ == DoubleEdge)
        C3PIN = 0; //reset bit
      if (channel[2].mode_ == FallEdge)
        C3PIN = 1; //set bit
    }
    if (channel[3].enable)
    {
      if (channel[3].mode_ == RiseEdge || channel[3].mode_ == DoubleEdge)
        C4PIN = 0; //reset bit
      if (channel[3].mode_ == FallEdge)
        C4PIN = 1; //set bit
    }

    // debug
    sprintf(msg_send, "The channel 1 is %d\nthe mode is: %d\nThe freq is: %d\nThe offset is: %d\nThe width is: %d\n Gap is: %d\n First edge is: %d\n Current edge is :%d\n",
            channel[0].enable, channel[0].mode_, channel[0].freq_, channel[0].offset_, channel[0].width_, channel[0].gap_, channel[0].first_trigger_edge_, channel[0].current_trigger_edge_);
    NU32_WriteUART3(msg_send);
    sprintf(msg_send, "The channel 2 is %d\nthe mode is: %d\nThe freq is: %d\nThe offset is: %d\nThe width is: %d\n Gap is: %d\n First edge is: %d\n Current edge is :%d\n",
            channel[1].enable, channel[1].mode_, channel[1].freq_, channel[1].offset_, channel[1].width_, channel[1].gap_, channel[1].first_trigger_edge_, channel[1].current_trigger_edge_);
    NU32_WriteUART3(msg_send);
    sprintf(msg_send, "The channel 3 is %d\nthe mode is: %d\nThe freq is: %d\nThe offset is: %d\nThe width is: %d\n Gap is: %d\n First edge is: %d\n Current edge is :%d\n",
            channel[2].enable, channel[2].mode_, channel[2].freq_, channel[2].offset_, channel[2].width_, channel[2].gap_, channel[2].first_trigger_edge_, channel[2].current_trigger_edge_);
    NU32_WriteUART3(msg_send);
    sprintf(msg_send, "The channel 4 is %d\nthe mode is: %d\nThe freq is: %d\nThe offset is: %d\nThe width is: %d\n Gap is: %d\n First edge is: %d\n Current edge is :%d\n",
            channel[3].enable, channel[3].mode_, channel[3].freq_, channel[3].offset_, channel[3].width_, channel[3].gap_, channel[3].first_trigger_edge_, channel[3].current_trigger_edge_);
    NU32_WriteUART3(msg_send);
    sprintf(msg_send, "The clock channel is %d\nthe mode is: %d\nThe freq is: %d\nThe offset is: %d\nThe width is: %d\n Gap is: %d\n First edge is: %d\n Current edge is :%d\n",
            clock_channel.enable, clock_channel.mode_, clock_channel.freq_, clock_channel.offset_, clock_channel.width_, clock_channel.gap_, clock_channel.first_trigger_edge_, clock_channel.current_trigger_edge_);
    NU32_WriteUART3(msg_send);
    sprintf(msg_send, "The start indicator channel is %d\nthe mode is: %d\nThe freq is: %d\nThe offset is: %d\nThe width is: %d\n Gap is: %d\n First edge is: %d\n Current edge is :%d\n",
            start_indicator.enable, start_indicator.mode_, start_indicator.freq_, start_indicator.offset_, start_indicator.width_, start_indicator.gap_, start_indicator.first_trigger_edge_, start_indicator.current_trigger_edge_);
    NU32_WriteUART3(msg_send);
    has_set_config = 1;
  }
  else
  {
    NU32_WriteUART3("WRONG LENGTH PLZ CHECK THE INPUT");
  }
}

void ConfigChannel(Channel *c)
{
  c->width_ /= (1000000 / SECOND_COUNT);                                                                                 // change unit to 10us
  c->offset_ /= (1000000 / SECOND_COUNT);                                                                                // change unit to 10us
  c->gap_ = SECOND_COUNT / c->freq_;                                                                                     // calculate gap
  c->first_trigger_edge_ = c->offset_ >= 0 ? c->offset_ : c->offset_ + c->gap_;                                          // calculate first raise edge
  c->current_trigger_edge_ = c->offset_ >= 0 ? c->offset_ : SECOND_COUNT + c->offset_;                                   // calculate current raise edge
  c->first_reset_edge_ = c->offset_ + c->width_ >= 0 ? c->offset_ + c->width_ : c->offset_ + c->width_ + c->gap_;        // calculate first raise edge
  c->current_reset_edge_ = c->offset_ + c->width_ >= 0 ? c->offset_ + c->width_ : SECOND_COUNT + c->offset_ + c->width_; // calculate current raise edge
}

inline void SetPin(int i)
{
  switch (i)
  {
  case 0:
    C1PIN = 1;
    break;
  case 1:
    C2PIN = 1;
    break;
  case 2:
    C3PIN = 1;
    break;
  case 3:
    C4PIN = 1;
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
    C1PIN = 0;
    break;
  case 1:
    C2PIN = 0;
    break;
  case 2:
    C3PIN = 0;
    break;
  case 3:
    C4PIN = 0;
    break;
  default:
    break;
  }
}

inline void InvPin(int i)
{
  switch (i)
  {
  case 0:
    C1PIN = !C1PIN;
    break;
  case 1:
    C2PIN = !C2PIN;
    break;
  case 2:
    C3PIN = !C3PIN;
    break;
  case 3:
    C4PIN = !C4PIN;
    break;
  default:
    break;
  }
}