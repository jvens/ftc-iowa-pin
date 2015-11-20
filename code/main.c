/*
 * File:   main.c
 * Author: vens
 *
 * Created on October 17, 2015, 7:44 PM
 */


#include <xc.h>

#define OUT
#define IN

typedef enum bool
{
    false=0, true
}bool;


const unsigned short a1[] = 
{
    12,
    0x5001,
    0x5002,
    0x5004,
    0x5008,
    0x5010,
    0x5020,
    0x5040,
    0x5080,
    0x5100,
    0x5200,
    0x5400,
    0x5800
};

unsigned short const * animation_table[] = {&a1};
unsigned short number_of_animations = 1;

/////////////////////////////////////////////////
//                                             //
//  FEATURE SELECTION                          //
//                                             //
/////////////////////////////////////////////////

//#define ENABLE_UART
//#define ENABLE_TOUCH
//#define ENABLE_DAC

//#define ENABLE_CIRCULAR_BUFFER

//-- dependencies
#ifdef ENABLE_UART
#ifndef ENABLE_CIRCULAR_BUFFER
#define ENABLE_CIRCULAR_BUFFER
#endif
#endif

/////////////////////////////////////////////////
//                                             //
//   INTERRUPT HANDLER                         //
//                                             //
/////////////////////////////////////////////////





/////////////////////////////////////////////////
//                                             //
//   PROCESSOR CORE FEATURES                   //
//                                             //
/////////////////////////////////////////////////

/**
 * Initialize the processors core features.
 * @details configure the processor to use the internal oscillator at 500kHz and set the WDT
 *          time out to 1ms.
 */
inline static void proc_init(void)
{
    // init oscillator
    OSCCON = 0x3A; // PLL disabled, f=500kHz, Internal osc
    
    // init watch dog timer
    WDTCON = 0x00; // set WDT time out to 1ms
}

/**
 * Enable the watchdog timer.
 */
inline void wdt_enable(void)
{
    WDTCON |= 0x01;
}

/**
 * Disable the watchdog timer.
 */
inline void wdt_disable(void)
{
    WDTCON &= ~0x01;
}

/////////////////////////////////////////////////
//                                             //
//   CIRCULAR BUFFER                           //
//                                             //
/////////////////////////////////////////////////

#ifdef ENABLE_CIRCULAR_BUFFER
struct circular_buffer
{
    char *buffer;
    unsigned char read_index;
    unsigned char write_index;
    unsigned char count;
    unsigned char size;
};

void cb_init(struct circular_buffer * cb, char * buffer, unsigned char size)
{
    cb->buffer = buffer;
    cb->read_index = 0;
    cb->write_index = 0;
    cb->count = 0;
    cb->size = size;
    return;
}

unsigned char cb_push(struct circular_buffer * cb, char data)
{
    if(cb->count == cb->size)
        return -1;
    
    cb->buffer[cb->write_index++] = data;
    cb->write_index %= cb->size;
    cb->count++;
    return cb->count;
}

unsigned char cb_pop(struct circular_buffer * cb, OUT char * data)
{
    unsigned char count;
    //proc_interrupt_disable();
    if(cb->count == 0)
        return -1;
    
    *data = cb->buffer[cb->read_index++];
    cb->read_index %= cb->size;
    cb->count --;
    count = cb->count;
    //proc_interrupt_enable();
    return count;
}

#endif

/////////////////////////////////////////////////
//                                             //
//   UART                                      //
//                                             //
/////////////////////////////////////////////////
/*
/// @todo catch errors

#ifdef ENABLE_UART
#define UART_RX_BUFFER_SIZE 32

static struct UART_DATA
{
    char rx_data[UART_RX_BUFFER_SIZE];
    struct circular_buffer rx_buffer;
}uart;
#endif
*/
/**
 * Initialize the UART module
 *
inline void uart_init(void)
{
#ifdef ENABLE_UART
    cb_init(&uart.rx_buffer, uart.rx_data, UART_RX_BUFFER_SIZE);
    
    BAUD1CON = 0x00;
    
    RC1STA = 0x90; // enabled, async mode
#else
    return;
#endif
}
*/
/**
 * Callback called anytime a new byte is available to read from the UART.  
 * @details This function is called by the ISR anytime new data is available to
 *          read from the UART module.  It grabs the new data and saves it, but
 *          does not process it.  The data is stored in the circular buffer
 *          uart.rx_buffer.
 *
inline static void uart_rx(void)
{
#ifdef ENABLE_UART
    unsigned char error = RCSTA;
    // if not error
    cb_push(&uart.rx_buffer, RCREG);
#else
    return;
#endif
}

static void uart_loop(void)
{
#ifdef ENABLE_UART
    static char string[16];
    static unsigned char i = 0;
    unsigned char count;
    char data;
    for(count = cb_pop(&uart.rx_buffer, &data);count > 0; count = cb_pop(&uart.rx_buffer, &data))
    {
        string[i++] = data;
        if(data == '\n')
        {
            // handle data
            i = 0;
        }
        else if(i == 16)
        {
            // buffer overrun
            
            i = 0;
            break;
        }
    }
#else
    return;
#endif
}
*/
/////////////////////////////////////////////////
//                                             //
//   DAC                                       //
//                                             //
/////////////////////////////////////////////////
/*
enum dac_source
{
    
};

static void dac_init(void)
{
#warning dac_init not implemented
}

static void dac_set_source(enum dac_source source)
{
    
}

static void dac_turn_on(void)
{
    
}

static void dac_turn_off(void)
{
    
}

static void dac_set(unsigned char value)
{
    
}
*/
/////////////////////////////////////////////////
//                                             //
//   GPIOs                                     //
//                                             //
/////////////////////////////////////////////////

/**
 * @brief Represents the registers for a GPIO bank.
 * @todo make these pointers constant
 */
struct gpio_reg
{
    /// The input read value
    volatile unsigned char * port;
    /// The direction, 0=output, 1=input
    volatile unsigned char * tris;
    /// The output latch value
    volatile unsigned char * lat;
    /// The input level to be considered 1
    volatile unsigned char * inlvl;
    /// Open drain enabled when 1
    volatile unsigned char * odcon;
    /// Turn on slewrate control
    volatile unsigned char * slrcon;
    /// enable weak pullups
    volatile unsigned char * wpu;
};

/// GPIO bank A
const struct gpio_reg GPIOA = {&PORTA, &TRISA, &LATA, &INLVLA, &ODCONA, &SLRCONA, &WPUA};
/// GPIO bank B
const struct gpio_reg GPIOC = {&PORTC, &TRISC, &LATC, &INLVLC, &ODCONC, &SLRCONC, &WPUC};

/**
 * @brief A single GPIO pin
 */
struct gpio_pin
{
    /// The register bank the pin is connected to
    const struct gpio_reg const * reg;
    /// The pin on the bank that is this GPIO
    unsigned char pin;
};

/**
 * Named GPIOs for this probject
 */
typedef enum gpio
{
    SEL0, SEL1, RED0, RED1, WHITE0, WHITE1, BLUE0, BLUE1
}gpio;

/// The GPIOs for this project
const struct gpio_pin gpios[] = 
{
    {&GPIOA, 0}, // SEL0
    {&GPIOA, 1}, // SEL1
    {&GPIOC, 0}, // RED0
    {&GPIOC, 1}, // RED1
    {&GPIOC, 2}, // WHITE0
    {&GPIOC, 4}, // WHITE1
    {&GPIOA, 4}, // BLUE0
    {&GPIOA, 5}  // BLUE1
};

typedef enum gpio_mode
{
    INPUT, OUTPUT, HIGHZ
}gpio_mode;

typedef enum gpio_value
{
    LOW = 0, HIGH
}gpio_value;

/// @def Set the @param bit of the @param byte to @param value
#define SET_BIT(byte, bit, value) (value ? byte |= 1 << bit : byte &= ~(1 << bit))

inline void gpio_setMode(gpio g, gpio_mode mode)
{
    switch (mode)
    {
        case INPUT:
        case HIGHZ:
            SET_BIT(*gpios[g].reg->tris, gpios[g].pin, 1);
            break;
        case OUTPUT:
            SET_BIT(*gpios[g].reg->tris, gpios[g].pin, 0);
    }
}

inline void gpio_setValue(gpio g, gpio_value value)
{
    SET_BIT(*gpios[g].reg->lat, gpios[g].pin, value);
}


inline void gpio_setPullup(gpio g, bool value)
{
    SET_BIT(*gpios[g].reg->wpu, gpios[g].pin, value);
}

inline void gpio_setOpenDrain(gpio g, bool value)
{
    SET_BIT(*gpios[g].reg->odcon, gpios[g].pin, value);
}

inline void gpio_limitSlewRate(gpio g, bool value)
{
    SET_BIT(*gpios[g].reg->slrcon, gpios[g].pin, value);
}

inline char gpio_readValue(gpio g)
{
    return (*gpios[g].reg->port & (1 << gpios[g].pin));
}

/////////////////////////////////////////////////
//                                             //
//   ANIMATION                                 //
//                                             //
/////////////////////////////////////////////////

struct animation
{
    unsigned short frames[128];         ///< Array of frame data
    unsigned char frame_number;         ///< number of current frame
    unsigned char number_of_frames;     ///< total number of frames in the array
}animation;

unsigned short animation_getNextFrame()
{
    animation.frame_number += 1;
    animation.frame_number %= animation.number_of_frames;
    return animation.frames[animation.frame_number];
}

void animation_load(unsigned short number)
{
    unsigned short const * a = animation_table[number];
    unsigned short i;
    animation.number_of_frames = a[0];
    for(i=0; i < animation.number_of_frames; i++)
    {
        animation.frames[i] = a[i+1];
    }
    animation.frame_number = 0;
}

/////////////////////////////////////////////////
//                                             //
//   LEDs                                      //
//                                             //
/////////////////////////////////////////////////

struct display
{
    unsigned red    : 4;
    unsigned white  : 4;
    unsigned blue   : 4;
};

enum led_state
{
    LED_STATE_RED0,
    LED_STATE_RED1,
    LED_STATE_WHITE0,
    LED_STATE_WHITE1,
    LED_STATE_BLUE0,
    LED_STATE_BLUE1
};

struct led_state_machine
{
    struct display current_display;
    unsigned char frame_length;         ///< The number of times this frame should be shown (0=allways show)
    unsigned char frame_time;           ///< The number of times this frame has been shown
    enum led_state state;               ///< State of the finite state machine
    unsigned short current_frame;
    
}led_state_machine;



void led_init()
{
    gpio_setMode(SEL0, HIGHZ);
    gpio_setMode(SEL1, HIGHZ);
    gpio_setMode(RED0, HIGHZ);
    gpio_setMode(RED1, HIGHZ);
    gpio_setMode(WHITE0, HIGHZ);
    gpio_setMode(WHITE1, HIGHZ);
    gpio_setMode(BLUE0, HIGHZ);
    gpio_setMode(BLUE1, HIGHZ);
    
    gpio_setPullup(SEL0, false);
    gpio_setPullup(SEL1, false);
    gpio_setPullup(RED0, false);
    gpio_setPullup(RED1, false);
    gpio_setPullup(WHITE0, false);
    gpio_setPullup(WHITE1, false);
    gpio_setPullup(BLUE0, false);
    gpio_setPullup(BLUE1, false);
    
    gpio_setOpenDrain(SEL0, true);
    gpio_setOpenDrain(SEL1, true);
    gpio_setOpenDrain(RED0, true);
    gpio_setOpenDrain(RED1, true);
    gpio_setOpenDrain(WHITE0, true);
    gpio_setOpenDrain(WHITE1, true);
    gpio_setOpenDrain(BLUE0, true);
    gpio_setOpenDrain(BLUE1, true);
    
    gpio_setValue(SEL0, LOW);
    gpio_setValue(SEL1, LOW);
    gpio_setValue(RED0, LOW);
    gpio_setValue(RED1, LOW);
    gpio_setValue(WHITE0, LOW);
    gpio_setValue(WHITE1, LOW);
    gpio_setValue(BLUE0, LOW);
    gpio_setValue(BLUE1, LOW);
}

void led_turnOff(gpio g)
{
    gpio_setMode(g, HIGHZ);
    gpio_setMode(SEL0, HIGHZ);
    gpio_setMode(SEL1, HIGHZ);
}

void led_turnOn(gpio g, unsigned char sel0, unsigned char sel1)
{
    gpio_setMode(SEL0, sel0 ? OUTPUT : HIGHZ);
    gpio_setMode(SEL1, sel1 ? OUTPUT : HIGHZ);
    gpio_setMode(g, OUTPUT);
}

void led_loadNextFrame(void)
{
    unsigned short frame = animation_getNextFrame();
    led_state_machine.current_display.red = frame & 0x000F;
    led_state_machine.current_display.white=(frame & 0x00F0) >> 4;
    led_state_machine.current_display.blue = (frame & 0x0F00) >> 8;
    led_state_machine.frame_length = (frame & 0xF000) >> 8;          // note that this is multiplied by 16 => 1 length ~= 100ms
    led_state_machine.frame_time = 0;
}

void led_loop()
{
    switch(led_state_machine.state)
    {
        case LED_STATE_RED0:
            led_turnOff(BLUE1);
            led_turnOn(RED0, led_state_machine.current_display.red & 0x1, led_state_machine.current_display.red & 0x2);
            led_state_machine.state = LED_STATE_RED1;
            break;
        case LED_STATE_RED1:
            led_turnOff(RED0);
            led_turnOn(RED1, led_state_machine.current_display.red & 0x4, led_state_machine.current_display.red & 0x8);
            led_state_machine.state = LED_STATE_WHITE0;
            break;
        case LED_STATE_WHITE0:
            led_turnOff(RED1);
            led_turnOn(WHITE0, led_state_machine.current_display.white & 0x1, led_state_machine.current_display.white & 0x2);
            led_state_machine.state = LED_STATE_WHITE1;
            break;
        case LED_STATE_WHITE1:
            led_turnOff(WHITE0);
            led_turnOn(WHITE1, led_state_machine.current_display.white & 0x4, led_state_machine.current_display.white & 0x8);
            led_state_machine.state = LED_STATE_BLUE0;
            break;
        case LED_STATE_BLUE0:
            led_turnOff(WHITE1);
            led_turnOn(BLUE0, led_state_machine.current_display.blue & 0x1, led_state_machine.current_display.blue & 0x2);
            led_state_machine.state = LED_STATE_BLUE1;
            break;
        case LED_STATE_BLUE1:
            led_turnOff(BLUE0);
            led_turnOn(BLUE1, led_state_machine.current_display.blue & 0x4, led_state_machine.current_display.blue & 0x8);
            led_state_machine.state = LED_STATE_RED0;
            led_state_machine.frame_time ++;
            if(led_state_machine.frame_length && led_state_machine.frame_time == led_state_machine.frame_length)
            {
                led_loadNextFrame();
            }
            break;
    }
}



/////////////////////////////////////////////////
//                                             //
//   MAIN                                      //
//                                             //
/////////////////////////////////////////////////

void main(void) {
    proc_init();
    // dac_init();
    led_init();
    // uart_init();
    // touch_init();
    
    animation_load(0);
    
    wdt_enable();
    while(1)    // this loop is approxamently every 1ms
    {
        // set led values
        led_loop();
        //uart_loop();
    //     proc_sleep();
    }
    
    
    return;
}

