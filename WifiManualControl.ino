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
#define AnalogIn A0

/***********************************************************************************************************************
   D A T A   D E C L A R A T I O N S (exported, local)
 **********************************************************************************************************************/

Bounce WmcPowerOffOnButton      = Bounce();
Bounce WmcPulseSwitchPushButton = Bounce();

unsigned long WmcStartMsPulseSwitchPushButton;
unsigned long WmcUpdateTimer3Seconds;
unsigned long WmcUpdatePulseSwitch;
unsigned long WmcUpdateTimer50msec;
unsigned long WmcUpdateTimer100msec;
unsigned long WmcUpdateTimer500msec;

// buffer to hold incoming and outgoing packets
bool turnedWhilePressed           = false;
unsigned int encoder0PosActual    = 0;
volatile unsigned int encoder0Pos = 0;

uint8_t WmcButton;

updateEvent3sec wmcUpdateEvent3Sec;
pushButtonsEvent wmcPushButtonEvent;
pulseSwitchEvent wmcPulseSwitchEvent;
updateEvent50msec wmcUpdateEvent50msec;
updateEvent100msec wmcUpdateEvent100msec;
updateEvent500msec wmcUpdateEvent500msec;

/***********************************************************************************************************************
   L O C A L   F U N C T I O N S
 **********************************************************************************************************************/

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
            encoder0Pos = encoder0Pos + 1;
        }
        else
        {
            // CCW
            encoder0Pos = encoder0Pos - 1;
        }
    }
    else // must be a high-to-low edge on channel A
    {
        // check channel B to see which way encoder is turning
        if (digitalRead(encoder0PinB) == HIGH)
        {
            // CW
            encoder0Pos = encoder0Pos + 1;
        }
        else
        {
            // CCW
            encoder0Pos = encoder0Pos - 1;
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
        Delta             = encoder0Pos - encoder0PosActual;
        encoder0PosActual = encoder0Pos;
    }

    return (Delta);
}

/***********************************************************************************************************************
 */
static uint8_t WmcFunctionButtons(void)
{
    int readingIn;
    uint8_t Stat = 255;

    /* Read analog once every 100msec, reading it each time in loop disturbs the
       Wifi... */
    if (millis() % 100 == 0)
    {
        readingIn = analogRead(AnalogIn);

        /* Check for a pressed button. No check on two or more pressed buttons! */
        if (readingIn < 30)
        {
#if APP_CFG_PCB_VERSION == APP_CFG_PCB_VERSION_REV01
            WmcButton = 4;
#else
            WmcButton = 0;
#endif
        }
        else if ((readingIn > 480) && (readingIn < 500))
        {
#if APP_CFG_PCB_VERSION == APP_CFG_PCB_VERSION_REV01
            WmcButton = 5;
#else
            WmcButton = 1;
#endif
        }
        else if ((readingIn > 645) && (readingIn < 665))
        {
#if APP_CFG_PCB_VERSION == APP_CFG_PCB_VERSION_REV01
            WmcButton = 2;
#else
            WmcButton = 2;
#endif
        }
        else if ((readingIn > 722) && (readingIn < 742))
        {
#if APP_CFG_PCB_VERSION == APP_CFG_PCB_VERSION_REV01
            WmcButton = 3;
#else
            WmcButton = 3;
#endif
        }
        else if ((readingIn > 771) && (readingIn < 791))
        {
#if APP_CFG_PCB_VERSION == APP_CFG_PCB_VERSION_REV01
            WmcButton = 0;
#else
            WmcButton = 4;
#endif
        }
        else if ((readingIn > 802) && (readingIn < 822))
        {
#if APP_CFG_PCB_VERSION == APP_CFG_PCB_VERSION_REV01
            WmcButton = 1;
#else
            WmcButton = 5;
#endif
        }

        /* Check for released button. Only if button released and valid
         * button press return it.*/
        if (readingIn > 950)
        {
            if (WmcButton != 255)
            {
                Stat      = WmcButton;
                WmcButton = 255;
            }
        }
    }
    return (Stat);
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
    int8_t Delta         = 0;
    uint8_t buttonActual = 255;

    /* Check for timed events. */
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

    /* Function buttons on analog input.*/
    buttonActual = WmcFunctionButtons();
    if (buttonActual != 255)
    {
        /* Generate event if a button is pressed and released. */
        wmcPushButtonEvent.Button = static_cast<pushButtons>(buttonActual);
        send_event(wmcPushButtonEvent);
    }
}
