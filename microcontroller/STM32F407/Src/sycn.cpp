#include "sync.h"
#include "bsp_systic.h"

extern "C" {
void HAL_SYSTICK_Callback(void)
{
    if (count_ == TICPS)
    {
        count_ = 0;
    }

    if (count_ == TICPS / 2 && starting)
    {
        starting = 0;
        has_start = 1;
    }
    if (start_indicator.enable)
    {
        if (count_ == start_indicator.current_trigger_edge_ && has_start)
        {
            if (start_indicator.mode_ == Channel::RiseEdge || start_indicator.mode_ == Channel::DoubleEdge)
                START_HIGH();
            if (start_indicator.mode_ == Channel::FallEdge)
                START_LOW();
        }
    }

    if (clock_channel.enable)
    {
        if (count_ == clock_channel.current_trigger_edge_)
        {
            clock_channel.current_trigger_edge_ += clock_channel.gap_;
            if (clock_channel.current_trigger_edge_ + clock_channel.gap_ > TICPS + clock_channel.first_trigger_edge_)
                clock_channel.current_trigger_edge_ = clock_channel.first_trigger_edge_;
            if (clock_channel.mode_ == Channel::DoubleEdge)
                CLOCK_TOGGLE();
            else if (clock_channel.mode_ == Channel::RiseEdge)
                CLOCK_HIGH();
            else
                CLOCK_LOW();
        }
        if (count_ == clock_channel.current_reset_edge_)
        {
            clock_channel.current_reset_edge_ += clock_channel.gap_;
            if (clock_channel.current_reset_edge_ + clock_channel.gap_ > TICPS + clock_channel.first_reset_edge_)
                clock_channel.current_reset_edge_ = clock_channel.first_reset_edge_;
            if (clock_channel.mode_ == Channel::RiseEdge)
                CLOCK_LOW();
            else if (clock_channel.mode_ == Channel::FallEdge)
                CLOCK_HIGH();
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
                    if (channel[i].current_trigger_edge_ + channel[i].gap_ > TICPS + channel[i].first_trigger_edge_)
                        channel[i].current_trigger_edge_ = channel[i].first_trigger_edge_;
                    if (channel[i].mode_ == Channel::DoubleEdge)
                        TogPin(i);
                    else if (channel[i].mode_ == Channel::RiseEdge)
                        SetPin(i);
                    else
                        ResetPin(i);
                }
                if (count_ == channel[i].current_reset_edge_)
                {
                    channel[i].current_reset_edge_ += channel[i].gap_;
                    if (channel[i].current_reset_edge_ + channel[i].gap_ > TICPS + channel[i].first_reset_edge_)
                        channel[i].current_reset_edge_ = channel[i].first_reset_edge_;
                    if (channel[i].mode_ == Channel::RiseEdge)
                        ResetPin(i);
                    else if (channel[i].mode_ == Channel::FallEdge)
                        SetPin(i);
                }
            }
        }

    count_ += 1;
}
}