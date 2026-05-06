#include "track.h"
#include "ti_msp_dl_config.h"

#define TRACK_BLACK_LEVEL 0
#define TRACK_SENSOR_COUNT 8

static const uint32_t track_pins[TRACK_SENSOR_COUNT] = {
    GPIO_TRACK_IR1_PIN,
    GPIO_TRACK_IR2_PIN,
    GPIO_TRACK_IR3_PIN,
    GPIO_TRACK_IR4_PIN,
    GPIO_TRACK_IR5_PIN,
    GPIO_TRACK_IR6_PIN,
    GPIO_TRACK_IR7_PIN,
    GPIO_TRACK_IR8_PIN
};

static const int track_weights[TRACK_SENSOR_COUNT] = {
    -700, -500, -300, -100, 100, 300, 500, 700
};

static int read_pin(uint32_t pin)
{
    return DL_GPIO_readPins(GPIO_TRACK_PORT, pin) ? 1 : 0;
}

static int is_black_level(int level)
{
    return level == TRACK_BLACK_LEVEL;
}

void track_init(void)
{
    /* GPIO inputs are configured by SysConfig in SYSCFG_DL_init(). */
}

track_sample_t track_read(void)
{
    track_sample_t sample = {0};
    int weighted_sum = 0;
    int i;

    for (i = 0; i < TRACK_SENSOR_COUNT; i++)
    {
        int level = read_pin(track_pins[i]);

        if (level)
        {
            sample.raw |= (uint8_t)(1U << i);
        }

        if (is_black_level(level))
        {
            sample.black_mask |= (uint8_t)(1U << i);
            sample.black_count++;
            weighted_sum += track_weights[i];
        }
    }

    if (sample.black_count > 0)
    {
        sample.error = weighted_sum / sample.black_count;
    }

    return sample;
}

int track_get_error(void)
{
    return track_read().error;
}

int track_is_lost(void)
{
    return track_read().black_count == 0;
}

int track_is_cross(void)
{
    return track_read().black_count >= 6;
}
