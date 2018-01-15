#ifndef BUTTON_H_
#define BUTTON_H_


// Default Hue for Fn
#define BTN_FN1_BR              90
#define BTN_FN1_CCT             3000
#define BTN_FN1_W               0
#define BTN_FN1_R               255
#define BTN_FN1_G               0  
#define BTN_FN1_B               0
#define BTN_FN1_SC              0

#define BTN_FN2_BR              20
#define BTN_FN2_CCT             3500
#define BTN_FN2_W               0
#define BTN_FN2_R               0
#define BTN_FN2_G               255  
#define BTN_FN2_B               0
#define BTN_FN2_SC              0

#define BTN_FN3_BR              75
#define BTN_FN3_CCT             5000
#define BTN_FN3_W               0
#define BTN_FN3_R               0
#define BTN_FN3_G               0  
#define BTN_FN3_B               255
#define BTN_FN3_SC              0

#define BTN_FN4_BR              85
#define BTN_FN4_CCT             6000
#define BTN_FN4_W               0
#define BTN_FN4_R               205
#define BTN_FN4_G               0  
#define BTN_FN4_B               205
#define BTN_FN4_SC              0

#define BTN_FN5_BR              65
#define BTN_FN5_CCT             4500
#define BTN_FN5_W               0
#define BTN_FN5_R               227
#define BTN_FN5_G               31  
#define BTN_FN5_B               51
#define BTN_FN5_SC              0

#define BTN_FN6_BR              65
#define BTN_FN6_CCT             4000
#define BTN_FN6_W               0
#define BTN_FN6_R               255
#define BTN_FN6_G               128  
#define BTN_FN6_B               0
#define BTN_FN6_SC              0

#define BTN_FN7_BR              50
#define BTN_FN7_CCT             5500
#define BTN_FN7_W               0
#define BTN_FN7_R               0
#define BTN_FN7_G               255  
#define BTN_FN7_B               255
#define BTN_FN7_SC              0

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

#define IS_VALID_BUTTON(x)              ((x) >= keylstUp && (x) < keylstDummy)

void SelectDeviceLED(uint8_t _dev);

void button_event_handler(uint8_t _pin);
void button_init(void);
void SetFlashlight(uint8_t _st);
void SetLasterBeam(uint8_t _st);
void LED_Blink(bool _flash, bool _fast);

#endif // BUTTON_H_
