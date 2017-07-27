#ifndef BUTTON_H_
#define BUTTON_H_

// Default Hue for Fn
#define BTN_FN1_BR              90
#define BTN_FN1_CCT             3000

#define BTN_FN2_BR              20
#define BTN_FN2_CCT             3500

#define BTN_FN3_BR              75
#define BTN_FN3_CCT             5000
#define BTN_FN3_W               0
#define BTN_FN3_R               230
#define BTN_FN3_G               32  
#define BTN_FN3_B               80

#define BTN_FN4_BR              85
#define BTN_FN4_CCT             6000

// Key list
typedef enum
{
    keylstUp,
    keylstDown,
    keylstLeft,
    keylstRight,
    keylstCenter,
    keylstFn1,
    keylstFn2,
    keylstFn3,
    keylstFn4,
    keylstFLASH,
    keylstLASER,
    keylstDummy
} keylist_t;

typedef enum button_timer_status_e
{
    BUTTON_STATUS_INIT = 0,
    BUTTON_STATUS_LESS_2S,
    BUTTON_STATUS_MORE_2S,
    BUTTON_STATUS_MORE_5S,
    BUTTON_STATUS_DOUBLE_TRACK
} button_timer_status_t;

typedef enum button_event_e
{
    BUTTON_INVALID = 0,
    BUTTON_SHORT_PRESS,
    BUTTON_DOUBLE_PRESS,
    BUTTON_LONG_HOLD,
    BUTTON_LONG_PRESS,
    BUTTON_VERY_LONG_HOLD,
    BUTTON_VERY_LONG_PRESS,
    DOUBLE_BTN_TRACK
} button_event_t;

#define IS_VALID_BUTTON(x)              ((x) >= keylstLeft && (x) < keylstDummy)

void SelectDeviceLED(uint8_t _dev);

void button_event_handler(uint8_t _pin);
void button_init(void);
void SetFlashlight(uint8_t _st);
void SetLasterBeam(uint8_t _st);
void LED_Blink(bool _flash, bool _fast);

#endif // BUTTON_H_
