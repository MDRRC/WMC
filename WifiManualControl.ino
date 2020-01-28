/***********************************************************************************************************************
   @file
   @ingroup
   @brief
 **********************************************************************************************************************/

/***********************************************************************************************************************
   I N C L U D E S
 **********************************************************************************************************************/
#include "Bounce2.h"
#include "Loclib.h"
#include "WmcTft.h"
#include "Z21Slave.h"
#include "app_cfg.h"
#include "fsmlist.hpp"
#include "wmc_event.h"

/***********************************************************************************************************************
   E X P O R T E D   S Y M B O L   D E F I N I T I O N S (defines, typedefs)
 **********************************************************************************************************************/
#define WMC_PULSE_SWITCH_UPDATE_TIME 50
#define encoder0PinA D5
#define encoder0PinB D6

/***********************************************************************************************************************
   D A T A   D E C L A R A T I O N S (exported, local)
 **********************************************************************************************************************/

Bounce WmcPowerOffOnButton      = Bounce();
Bounce WmcPulseSwitchPushButton = Bounce();

unsigned long WmcStartMsPulseSwitchPushButton;
unsigned long WmcUpdateTimer3Seconds;
unsigned long WmcUpdatePulseSwitch;
unsigned long WmcUpdateTimer5msec;
unsigned long WmcUpdateTimer50msec;
unsigned long WmcUpdateTimer100msec;
unsigned long WmcUpdateTimer500msec;

// buffer to hold incoming and outgoing packets
bool turnedWhilePressed      = false;
int16_t encoder0PosActual    = 0;
volatile int16_t encoder0Pos = 0;

uint8_t WmcButton;

updateEvent3sec wmcUpdateEvent3Sec;
pushButtonsEvent wmcPushButtonEvent;
pulseSwitchEvent wmcPulseSwitchEvent;
updateEvent5msec wmcUpdateEvent5msec;
updateEvent50msec wmcUpdateEvent50msec;
updateEvent100msec wmcUpdateEvent100msec;
updateEvent500msec wmcUpdateEvent500msec;

/***********************************************************************************************************************
   L O C A L   F U N C T I O N S
 **********************************************************************************************************************/

/***********************************************************************************************************************
 */
static bool WmcUpdate5msec(void)
{
    bool Result = false;

    if (millis() - WmcUpdateTimer5msec > 5)
    {
        Result               = true;
        WmcUpdateTimer5msec = millis();
        send_event(wmcUpdateEvent5msec);
    }

    return (Result);
}

/***********************************************************************************************************************
 */
static bool WmcUpdate50msec(void)
{
    bool Result = false;

    if (millis() - WmcUpdateTimer50msec > 50)
    {
        Result               = true;
        WmcUpdateTimer50msec = millis();
        send_event(wmcUpdateEvent50msec);
    }

    return (Result);
}

/***********************************************************************************************************************
 */
static bool WmcUpdate100msec(void)
{
    bool Result = false;

    if (millis() - WmcUpdateTimer100msec >= 100)
    {
        Result                = true;
        WmcUpdateTimer100msec = millis();
        send_event(wmcUpdateEvent100msec);
    }

    return (Result);
}

/***********************************************************************************************************************
 */
static bool WmcUpdate500msec(void)
{
    bool Result = false;

    if (millis() - WmcUpdateTimer500msec >= 500)
    {
        Result                = true;
        WmcUpdateTimer500msec = millis();
        send_event(wmcUpdateEvent500msec);
    }

    return (Result);
}

/***********************************************************************************************************************
 */
static bool WmcUpdate3Sec(void)
{
    bool Result = false;
    if (millis() - WmcUpdateTimer3Seconds > 3000)
    {
        Result                 = true;
        WmcUpdateTimer3Seconds = millis();
        send_event(wmcUpdateEvent3Sec);
    }

    return (Result);
}

/***********************************************************************************************************************
 */
void doEncoderA()
{
    // look for a low-to-high on channel A
    if (digitalRead(encoder0PinA) == HIGH)
    {
        // check channel B to see which way encoder is turning
        if (digitalRead(encoder0PinB) == LOW)
        {
            // CW
            encoder0Pos++;
        }
        else
        {
            // CCW
            encoder0Pos--;
        }
    }
    else // must be a high-to-low edge on channel A
    {
        // check channel B to see which way encoder is turning
        if (digitalRead(encoder0PinB) == HIGH)
        {
            // CW
            encoder0Pos++;
        }
        else
        {
            // CCW
            encoder0Pos--;
        }
    }
}

/***********************************************************************************************************************
 */
int8_t DecoderUpdate(void)
{
    int8_t Delta = 0;
    if (encoder0Pos != encoder0PosActual)
    {
        Delta = (int8_t)(encoder0Pos - encoder0PosActual);
        encoder0PosActual = encoder0Pos;
    }

    return (Delta);
}

/***********************************************************************************************************************
   E X P O R T E D   F U N C T I O N S
 **********************************************************************************************************************/

/**
 ******************************************************************************
   @brief
   @param
   @return     None
 ******************************************************************************/
void setup()
{
    Serial.begin(115200);

    /* Init the pulse / rotary encoder. */
    pinMode(encoder0PinA, INPUT_PULLUP);
    pinMode(encoder0PinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderA, FALLING);

    /* Init debounces for switches. */
    pinMode(D7, INPUT_PULLUP);
    WmcPulseSwitchPushButton.interval(100);
    WmcPulseSwitchPushButton.attach(D7);

    pinMode(D0, INPUT_PULLUP);
    WmcPowerOffOnButton.interval(100);
    WmcPowerOffOnButton.attach(D0);

    /* Init timers. */
    WmcUpdateTimer3Seconds          = millis();
    WmcUpdatePulseSwitch            = millis();
    WmcUpdateTimer50msec            = millis();
    WmcUpdateTimer100msec           = millis();
    WmcUpdateTimer500msec           = millis();
    WmcUpdateTimer3Seconds          = millis();
    WmcStartMsPulseSwitchPushButton = millis();

    /* Kick the state machine. */
    fsm_list::start();
}

/**
 ******************************************************************************
   @brief
   @param
   @return     None
 ******************************************************************************/
void loop()
{
    int8_t Delta = 0;

    /* Check for timed events. */
    WmcUpdate5msec();
    WmcUpdate50msec();
    WmcUpdate100msec();
    WmcUpdate500msec();
    WmcUpdate3Sec();

    /* Update button status. */
    WmcPowerOffOnButton.update();
    WmcPulseSwitchPushButton.update();

    /* Check for button changes and generate evbent if required. */
    if (WmcPowerOffOnButton.rose() == true)
    {
        wmcPushButtonEvent.Button = button_power;
        send_event(wmcPushButtonEvent);
    }

    /* Pulse switch push button handling. When releasing button check time and generate event based on time.*/
    if (WmcPulseSwitchPushButton.fell() == true)
    {
        WmcStartMsPulseSwitchPushButton = millis();
    }
    else if (WmcPulseSwitchPushButton.rose() == true)
    {
        if (turnedWhilePressed == false)
        {
            if (millis() - WmcStartMsPulseSwitchPushButton > 3000)
            {
                wmcPulseSwitchEvent.Status = pushedlong;
                send_event(wmcPulseSwitchEvent);
            }
            else if (millis() - WmcStartMsPulseSwitchPushButton < 300)
            {
                wmcPulseSwitchEvent.Status = pushedShort;
                send_event(wmcPulseSwitchEvent);
            }
            else if (millis() - WmcStartMsPulseSwitchPushButton < 1100)
            {
                wmcPulseSwitchEvent.Status = pushedNormal;
                send_event(wmcPulseSwitchEvent);
            }
        }
        else
        {
            turnedWhilePressed = false;
        }
    }
    else if ((millis() - WmcUpdatePulseSwitch) > WMC_PULSE_SWITCH_UPDATE_TIME)
    {
        /* Update pulse switch turn events if turned.*/
        WmcUpdatePulseSwitch = millis();
        Delta                = DecoderUpdate();
        if (Delta != 0)
        {
            if (WmcPulseSwitchPushButton.read() == LOW)
            {
                turnedWhilePressed         = true;
                wmcPulseSwitchEvent.Status = pushturn;
            }
            else
            {
                wmcPulseSwitchEvent.Status = turn;
            }

            wmcPulseSwitchEvent.Delta = Delta;

            send_event(wmcPulseSwitchEvent);
        }
    }
}
