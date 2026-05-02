#include "track.h"
#include "ti_msp_dl_config.h"

#define IR1     DL_GPIO_PIN_4
#define IR2     DL_GPIO_PIN_5
#define IR3     DL_GPIO_PIN_6
#define IR4     DL_GPIO_PIN_7
#define IR5     DL_GPIO_PIN_8
#define IR6     DL_GPIO_PIN_9
#define IR7     DL_GPIO_PIN_10
#define IR8     DL_GPIO_PIN_11

static int read_pin(uint32_t pin)
{
    return DL_GPIO_readPins(GPIOA, pin) ? 1 : 0;
}

void track_init(void)
{
    DL_GPIO_initDigitalInput(IR1);
    DL_GPIO_initDigitalInput(IR2);
    DL_GPIO_initDigitalInput(IR3);
    DL_GPIO_initDigitalInput(IR4);
    DL_GPIO_initDigitalInput(IR5);
    DL_GPIO_initDigitalInput(IR6);
    DL_GPIO_initDigitalInput(IR7);
    DL_GPIO_initDigitalInput(IR8);
}

int track_get_error(void)
{
    int err = 0;
    if(read_pin(IR1)) err -= 8;
    if(read_pin(IR2)) err -= 6;
    if(read_pin(IR3)) err -= 4;
    if(read_pin(IR4)) err -= 2;
    if(read_pin(IR5)) err += 2;
    if(read_pin(IR6)) err += 4;
    if(read_pin(IR7)) err += 6;
    if(read_pin(IR8)) err += 8;
    return err;
}

int track_is_lost(void)
{
    return (read_pin(IR1)==0 && read_pin(IR2)==0 && read_pin(IR3)==0 && read_pin(IR4)==0 &&
            read_pin(IR5)==0 && read_pin(IR6)==0 && read_pin(IR7)==0 && read_pin(IR8)==0);
}

int track_is_cross(void)
{
    return (read_pin(IR1) && read_pin(IR2) && read_pin(IR3) && read_pin(IR4) &&
            read_pin(IR5) && read_pin(IR6) && read_pin(IR7) && read_pin(IR8));
}