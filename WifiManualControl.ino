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
#include "WmcCli.h"
#include "WmcTft.h"
#include "Z21Slave.h"
#include "fsmlist.hpp"
#include "wmc_event.h"

/***********************************************************************************************************************
   E X P O R T E D   S Y M B O L   D E F I N I T I O N S (defines, typedefs)
 **********************************************************************************************************************/
#define UDP_TIMEOUT 3000
#define encoder0PinA D5
#define encoder0PinB D6
#define AnalogIn A0

/***********************************************************************************************************************
   D A T A   D E C L A R A T I O N S (exported, local)
 **********************************************************************************************************************/

WmcCli WmcCommandLine;

Bounce WmcPowerOffOnButton      = Bounce();
Bounce WmcPulseSwitchPushButton = Bounce();

unsigned long WmcStartMs;
unsigned long WmcStartMsPulseSwitchPushButton;
unsigned long WmcStartMsKeepAlive;
unsigned long WmcUpdateMs;
unsigned long WmcUpdatePulseSwitch;
unsigned long WmcUpdateEvent500msec;

// buffer to hold incoming and outgoing packets
bool turnedWhilePressed           = false;
unsigned int encoder0PosActual    = 0;
volatile unsigned int encoder0Pos = 0;

uint8_t WmcButton;

updateEvent3sec wmcUpdateEvent3Sec;
pushButtonsEvent wmcPushButtonEvent;
pulseSwitchEvent wmcPulseSwitchEvent;
updateEvent50msec wmcUpdateEvent50msec;
updateEvent500msec wmcUpdateEvent500msec;

/***********************************************************************************************************************
   L O C A L   F U N C T I O N S
 **********************************************************************************************************************/

/***********************************************************************************************************************
 */
static bool WmcUpdate3Sec(void)
{
    bool Result = false;
    if (millis() - WmcStartMsKeepAlive > 3000)
    {
        Result              = true;
        WmcStartMsKeepAlive = millis();
    }

    return (Result);
}

/***********************************************************************************************************************
 */
static bool WmcUpdate50msec(void)
{
    bool Result = false;

    if (millis() - WmcUpdateMs > 50)
    {
        Result      = true;
        WmcUpdateMs = millis();
    }

    return (Result);
}

/***********************************************************************************************************************
 */
static bool WmcUpdate500msec(void)
{
    bool Result = false;

    if (millis() - WmcUpdateEvent500msec >= 500)
    {
        Result                = true;
        WmcUpdateEvent500msec = millis();
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
        if (readingIn < 10)
        {
            WmcButton = 0;
        }
        else if ((readingIn > 480) && (readingIn < 500))
        {
            WmcButton = 1;
        }
        else if ((readingIn > 645) && (readingIn < 665))
        {
            WmcButton = 2;
        }
        else if ((readingIn > 722) && (readingIn < 742))
        {
            WmcButton = 3;
        }
        else if ((readingIn > 771) && (readingIn < 791))
        {
            WmcButton = 4;
        }
        else if ((readingIn > 802) && (readingIn < 822))
        {
            WmcButton = 5;
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
    WmcCommandLine.Init();

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
    WmcStartMs                      = millis();
    WmcStartMsKeepAlive             = millis();
    WmcUpdatePulseSwitch            = millis();
    WmcUpdateEvent500msec           = millis();
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
    if (WmcUpdate50msec() == true)
    {
        send_event(wmcUpdateEvent50msec);
    }

    if (WmcUpdate500msec() == true)
    {
        send_event(wmcUpdateEvent500msec);
    }

    if (WmcUpdate3Sec() == true)
    {
        send_event(wmcUpdateEvent3Sec);
    }

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
    else if ((millis() - WmcUpdatePulseSwitch) > 75)
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

    /* Command line update */
    WmcCommandLine.Update();
}
