/***********************************************************************************************************************
   @file
   @ingroup
   @brief
 **********************************************************************************************************************/

/***********************************************************************************************************************
   I N C L U D E S
 **********************************************************************************************************************/
#include "Bounce2.h"
#include "ESP8266WiFi.h"
#include "Event.h"
#include "Loclib.h"
#include "WmcCli.h"
#include "WmcTft.h"
#include "Z21Slave.h"
#include <WiFiUdp.h>

/***********************************************************************************************************************
   E X P O R T E D   S Y M B O L   D E F I N I T I O N S (defines, typedefs)
 **********************************************************************************************************************/
#define UDP_TIMEOUT 2000
#define encoder0PinA D5
#define encoder0PinB D6
#define AnalogIn A0
/**
   States of the application.
*/
typedef enum
{
    WMC_STATE_INIT_CONNECT = 0,
    WMC_STATE_INIT_CONNECT_RESPONSE,
    WMC_STATE_INIT_BROADCAST,
    WMC_STATE_INIT_STATUS_GET,
    WMC_STATE_INIT_STATUS_GET_RESPONSE,
    WMC_STATE_INIT_FIRMWARE_GET,
    WMC_STATE_INIT_FIRMWARE_GET_RESPONSE,
    WMC_STATE_INIT_GET_LOCO_INFO,
    WMC_STATE_INIT_GET_LOCO_INFO_RESPONSE,
    WMC_STATE_INIT_GET_LOCO_MODE,
    WMC_STATE_INIT_GET_LOCO_MODE_RESPONSE,
    WMC_STATE_POWER_OFF_SET,
    WMC_STATE_POWER_OFF,
    WMC_STATE_POWER_ON_SET,
    WMC_STATE_POWER_ON,
    WMC_STATE_MENU_SET,
    WMC_STATE_MENU,
    WMC_STATE_LOC_ADD_SET,
    WMC_STATE_LOC_ADD,
    WMC_STATE_LOC_ADD_FUNCTION_SET,
    WMC_STATE_LOC_ADD_FUNCTION,
    WMC_STATE_LOC_ADD_STORE,
    WMC_STATE_LOC_CHANGE_SET,
    WMC_STATE_LOC_CHANGE,
    WMC_STATE_LOC_REMOVE_SET,
    WMC_STATE_LOC_REMOVE,
    WMC_STATE_LOC_EXIT,
    WMC_STATE_CONNECTION_FAILED,
} WMC_STATE;

/***********************************************************************************************************************
   D A T A   D E C L A R A T I O N S (exported, local)
 **********************************************************************************************************************/

char ssid[] = "ssid_name";     // your network SSID (name)
char pass[] = "ssid_password"; // your network password

unsigned int WmcLocalPort; // local port to listen on

WMC_STATE WmcState;
WMC_STATE WmcStateRequest;
IPAddress WmcUdpIp(192, 168, 2, 112);

WiFiUDP WifiUdp;
LocLib WmcLocLib;
WmcTft WmcDisplay;
WmcCli WmcCommandLine;
Z21Slave WmcSlave;
EventManager WmcEvtMngr;

Bounce WmcPowerOffOnButton      = Bounce();
Bounce WmcPulseSwitchPushButton = Bounce();

byte WmcPacketBuffer[40]; // buffer to hold incoming and outgoing packets

bool TrackPower;
unsigned long WmcStartMs;
unsigned long WmcStartMsPulseSwitchPushButton;
unsigned long WmcStartMsKeepAlive;
Z21Slave::locInfo* WmcLocInfoReceived;
Z21Slave::locInfo WmcLocInfoReceivedPrevious;
Z21Slave::locInfo WmcLocInfoControl;

unsigned int encoder0PosActual    = 0;
volatile unsigned int encoder0Pos = 0;

uint8_t WmcButton;
bool WmcLocSelection = false;

uint16_t WmcLocAddressAdd;
uint16_t WmcLocAddFunction;
uint8_t WmcLocAddFunctionAssignment[5];

/**
   Prototype
*/

static void WmcTransmitData(const uint8_t* DataTxPtr, uint8_t DataTxLength);

/**
   Event handler / listener.
*/
struct Z21Listener : public EventTask
{
    using EventTask::execute;
    uint8_t* DataTransmitPtr;
    uint8_t assignedFunctions[5];
    uint8_t locIndex;
    uint8_t Index;
    bool UpdateAll = false;

    void execute(Event evt)
    {
        Z21Slave::EventData extra = (Z21Slave::EventData)(evt.extra);

        switch (extra)
        {
        case Z21Slave::none: break;
        case Z21Slave::trackPowerOn:
            TrackPower = true;
            switch (WmcState)
            {
            case WMC_STATE_INIT_CONNECT:
            case WMC_STATE_INIT_CONNECT_RESPONSE: WmcStateRequest = WMC_STATE_INIT_BROADCAST; break;
            case WMC_STATE_INIT_STATUS_GET:
            case WMC_STATE_INIT_STATUS_GET_RESPONSE: WmcStateRequest = WMC_STATE_INIT_GET_LOCO_INFO; break;
            case WMC_STATE_INIT_BROADCAST:
            case WMC_STATE_INIT_FIRMWARE_GET:
            case WMC_STATE_INIT_FIRMWARE_GET_RESPONSE:
            case WMC_STATE_INIT_GET_LOCO_INFO:
            case WMC_STATE_INIT_GET_LOCO_INFO_RESPONSE:
            case WMC_STATE_INIT_GET_LOCO_MODE:
            case WMC_STATE_INIT_GET_LOCO_MODE_RESPONSE:
            case WMC_STATE_POWER_ON_SET:
            case WMC_STATE_POWER_ON:
            case WMC_STATE_MENU_SET:
            case WMC_STATE_MENU:
            case WMC_STATE_LOC_ADD_SET:
            case WMC_STATE_LOC_ADD:
            case WMC_STATE_LOC_ADD_FUNCTION_SET:
            case WMC_STATE_LOC_ADD_FUNCTION:
            case WMC_STATE_LOC_ADD_STORE:
            case WMC_STATE_LOC_CHANGE_SET:
            case WMC_STATE_LOC_CHANGE:
            case WMC_STATE_LOC_REMOVE_SET:
            case WMC_STATE_LOC_REMOVE:
            case WMC_STATE_LOC_EXIT:
                // Do nothing, complete init cyle or stay in menu!
                break;
            default: WmcStateRequest = WMC_STATE_POWER_ON_SET; break;
            }
            break;
        case Z21Slave::trackPowerOff:
            TrackPower = false;
            switch (WmcState)
            {
            case WMC_STATE_INIT_CONNECT:
            case WMC_STATE_INIT_CONNECT_RESPONSE: WmcStateRequest = WMC_STATE_INIT_BROADCAST; break;
            case WMC_STATE_INIT_STATUS_GET:
            case WMC_STATE_INIT_STATUS_GET_RESPONSE: WmcStateRequest = WMC_STATE_INIT_GET_LOCO_INFO; break;
            case WMC_STATE_INIT_BROADCAST:
            case WMC_STATE_INIT_FIRMWARE_GET:
            case WMC_STATE_INIT_FIRMWARE_GET_RESPONSE:
            case WMC_STATE_INIT_GET_LOCO_INFO:
            case WMC_STATE_INIT_GET_LOCO_INFO_RESPONSE:
            case WMC_STATE_INIT_GET_LOCO_MODE:
            case WMC_STATE_INIT_GET_LOCO_MODE_RESPONSE:
            case WMC_STATE_POWER_OFF_SET:
            case WMC_STATE_POWER_OFF:
            case WMC_STATE_MENU_SET:
            case WMC_STATE_MENU:
            case WMC_STATE_LOC_ADD_SET:
            case WMC_STATE_LOC_ADD:
            case WMC_STATE_LOC_ADD_FUNCTION_SET:
            case WMC_STATE_LOC_ADD_FUNCTION:
            case WMC_STATE_LOC_ADD_STORE:
            case WMC_STATE_LOC_CHANGE_SET:
            case WMC_STATE_LOC_CHANGE:
            case WMC_STATE_LOC_REMOVE_SET:
            case WMC_STATE_LOC_REMOVE:
            case WMC_STATE_LOC_EXIT:
                // Do nothing, complete init cyle or stay in menu!
                break;
            default:
                if (WmcLocLib.SpeedGet() != 0)
                {
                    WmcLocLib.SpeedUpdate(0);

                    locIndex = WmcLocLib.CheckLoc(WmcLocInfoControl.Address);

                    for (Index = 0; Index < 5; Index++)
                    {
                        assignedFunctions[Index] = WmcLocLib.FunctionAssignedGet(Index);
                    }

                    WmcDisplay.UpdateLocInfo(WmcLocInfoReceived, &WmcLocInfoControl, assignedFunctions, false);
                }
                WmcStateRequest = WMC_STATE_POWER_OFF_SET;
                break;
            }
            break;
        case Z21Slave::lanVersionResponse:
        case Z21Slave::fwVersionInfoResponse:
            switch (WmcState)
            {
            case WMC_STATE_POWER_OFF_SET:
            case WMC_STATE_POWER_OFF:
            case WMC_STATE_POWER_ON_SET:
            case WMC_STATE_POWER_ON:
            case WMC_STATE_MENU_SET:
            case WMC_STATE_MENU:
            case WMC_STATE_LOC_ADD_SET:
            case WMC_STATE_LOC_ADD:
            case WMC_STATE_LOC_ADD_FUNCTION_SET:
            case WMC_STATE_LOC_ADD_FUNCTION:
            case WMC_STATE_LOC_ADD_STORE:
            case WMC_STATE_LOC_CHANGE_SET:
            case WMC_STATE_LOC_CHANGE:
            case WMC_STATE_LOC_REMOVE_SET:
            case WMC_STATE_LOC_REMOVE:
            case WMC_STATE_LOC_EXIT:
                // Do nothing, complete init cyle or stay in menu!
                break;
            default: WmcStateRequest = WMC_STATE_INIT_GET_LOCO_INFO; break;
            }
            break;

        case Z21Slave::locinfo:

            WmcLocInfoReceived = WmcSlave.LanXLocoInfo();

            if ((WmcState == WMC_STATE_INIT_GET_LOCO_INFO_RESPONSE) || (WmcState == WMC_STATE_LOC_EXIT))
            {
                if (WmcLocInfoReceived->Address != 0)
                {
                    switch (WmcLocInfoReceived->Steps)
                    {
                    case Z21Slave::locDecoderSpeedSteps14: WmcLocLib.DecoderStepsUpdate(LocLib::decoderStep14); break;
                    case Z21Slave::locDecoderSpeedSteps28: WmcLocLib.DecoderStepsUpdate(LocLib::decoderStep28); break;
                    case Z21Slave::locDecoderSpeedSteps128: WmcLocLib.DecoderStepsUpdate(LocLib::decoderStep128); break;
                    case Z21Slave::locDecoderSpeedStepsUnknown:
                        WmcLocLib.DecoderStepsUpdate(LocLib::decoderStep28);
                        break;
                    }

                    WmcLocLib.SpeedUpdate(WmcLocInfoReceived->Speed);

                    /* Show screen after init cycle. */
                    WmcDisplay.Clear();

                    if (TrackPower == false)
                    {
                        WmcStateRequest = WMC_STATE_POWER_OFF_SET;
                    }
                    else
                    {
                        WmcStateRequest = WMC_STATE_POWER_ON_SET;
                    }

                    locIndex = WmcLocLib.CheckLoc(WmcLocInfoControl.Address);

                    for (Index = 0; Index < 5; Index++)
                    {
                        assignedFunctions[Index] = WmcLocLib.FunctionAssignedGet(Index);
                    }
                    WmcLocInfoControl.Functions = 0;
                    WmcDisplay.UpdateLocInfo(WmcLocInfoReceived, &WmcLocInfoControl, assignedFunctions, true);
                    memcpy(&WmcLocInfoControl, WmcLocInfoReceived, sizeof(Z21Slave::locInfo));
                }
            }
            else
            {
                if (WmcLocInfoReceived->Address == WmcLocLib.GetActualLocAddress())
                {
                    if (WmcLocSelection == true)
                    {
                        switch (WmcLocInfoReceived->Steps)
                        {
                        case Z21Slave::locDecoderSpeedSteps14:
                            WmcLocLib.DecoderStepsUpdate(LocLib::decoderStep14);
                            break;
                        case Z21Slave::locDecoderSpeedSteps28:
                            WmcLocLib.DecoderStepsUpdate(LocLib::decoderStep28);
                            break;
                        case Z21Slave::locDecoderSpeedSteps128:
                            WmcLocLib.DecoderStepsUpdate(LocLib::decoderStep128);
                            break;
                        case Z21Slave::locDecoderSpeedStepsUnknown:
                            WmcLocLib.DecoderStepsUpdate(LocLib::decoderStep28);
                            break;
                        }

                        locIndex = WmcLocLib.CheckLoc(WmcLocInfoReceived->Address);
                        if (locIndex != 255)
                        {
                            for (Index = 0; Index < 5; Index++)
                            {
                                assignedFunctions[Index] = WmcLocLib.FunctionAssignedGet(Index);
                            }

                            WmcLocInfoReceivedPrevious.Functions = 0;
                        }
                    }

                    WmcLocLib.SpeedUpdate(WmcLocInfoReceived->Speed);
                    WmcDisplay.UpdateLocInfo(
                        WmcLocInfoReceived, &WmcLocInfoReceivedPrevious, assignedFunctions, WmcLocSelection);
                    memcpy(&WmcLocInfoReceivedPrevious, WmcLocInfoReceived, sizeof(Z21Slave::locInfo));
                    DecoderUpdate();
                }
            }
            break;
        case Z21Slave::txDataPresent:
            DataTransmitPtr = WmcSlave.GetDataTx();
            WmcTransmitData(DataTransmitPtr, DataTransmitPtr[0]);
            break;
        case Z21Slave::unknown: break;
        }
    }
} Z21Listener;

/***********************************************************************************************************************
   L O C A L   F U N C T I O N S
 **********************************************************************************************************************/

/***********************************************************************************************************************
 */
static void WmcCheckForRxData(void)
{
    int WmcPacketBufferLength = 0;

    if (WifiUdp.parsePacket())
    {
        // We've received a packet, read the data from it into the buffer
        WmcPacketBufferLength = WifiUdp.read(WmcPacketBuffer, 40);
        if (WmcPacketBufferLength != 0)
        {
            // Process the data.
            WmcSlave.ProcesDataRx(WmcPacketBuffer, sizeof(WmcPacketBuffer));
        }
    }
}

/***********************************************************************************************************************
 */
static bool WmcResponseTimeOut(void)
{
    bool Result = false;

    if ((millis() - WmcStartMs) > UDP_TIMEOUT)
    {
        Result     = true;
        WmcStartMs = millis();
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
static void WmcTransmitData(const uint8_t* DataTxPtr, uint8_t DataTxLength)
{
    WifiUdp.beginPacket(WmcUdpIp, WmcLocalPort);
    WifiUdp.write(DataTxPtr, DataTxLength);
    WifiUdp.endPacket();
}

/***********************************************************************************************************************
 */
static uint8_t WmcFunctionButtons(void)
{
    int readingIn;
    uint8_t Stat = 255;

    /* Read analog once every 100msec, reading it eacht ime in loop disturns the
     * Wifi... */
    if (millis() % 100 == 0)
    {
        readingIn = analogRead(AnalogIn);

        // Check for a pressed button.
        if (readingIn < 10)
        {
            WmcButton = 0;
        }
        else if ((readingIn > 322) && (readingIn < 335))
        {
            WmcButton = 1;
        }
        else if ((readingIn > 487) && (readingIn < 495))
        {
            WmcButton = 2;
        }
        else if ((readingIn > 667) && (readingIn < 677))
        {
            WmcButton = 3;
        }
        else if ((readingIn > 788) && (readingIn < 800))
        {
            WmcButton = 4;
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
    // Init variables.
    uint8_t WifiConnectCnt          = 0;
    TrackPower                      = false;
    WmcState                        = WMC_STATE_INIT_CONNECT;
    WmcStateRequest                 = WMC_STATE_INIT_CONNECT;
    WmcStartMsPulseSwitchPushButton = 0;
    WmcStartMsKeepAlive             = millis();
    WmcLocalPort                    = 21105;
    WmcLocAddressAdd                = 0;

    Serial.begin(115200);

    WmcEvtMngr.subscribe(Subscriber("1", &Z21Listener));
    WmcSlave.SetEventDestination(WmcEvtMngr, "1", 1);

    WmcLocLib.Init();
    WmcDisplay.Init();
    WmcCommandLine.Init();

    WmcDisplay.UpdateStatus("Connecting to Wifi", true, WmcTft::color_yellow);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);

    while ((WiFi.status() != WL_CONNECTED) && (WifiConnectCnt < 120))
    {
        WmcDisplay.WifiConnectUpdate(WifiConnectCnt);
        WifiConnectCnt++;
        delay(500);
    }

    if (WifiConnectCnt < 120)
    {
        /* Init the pulse / rotary encoder. */
        pinMode(encoder0PinA, INPUT);
        pinMode(encoder0PinB, INPUT);
        attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderA, FALLING);

        /* Init debounces for switches. */
        pinMode(D7, INPUT_PULLUP);
        WmcPulseSwitchPushButton.interval(100);
        WmcPulseSwitchPushButton.attach(D7);

        pinMode(D0, INPUT_PULLUP);
        WmcPowerOffOnButton.interval(100);
        WmcPowerOffOnButton.attach(D0);

        // start udp
        WmcDisplay.UpdateStatus("Connecting to central", true, WmcTft::color_yellow);
        WifiUdp.begin(WmcLocalPort);
    }
    else
    {
        WmcDisplay.WifiConnectFailed();
        WmcStateRequest = WMC_STATE_CONNECTION_FAILED;
    }

    WmcStartMsKeepAlive = millis();
}

/**
 ******************************************************************************
   @brief
   @param
   @return     None
 ******************************************************************************/
void loop()
{
    int8_t Delta              = 0;
    uint8_t Function          = 0;
    uint8_t buttonActual      = 255;
    bool LocAddUpdateScreen   = false;
    bool LocAddUpdateFunction = false;
    uint8_t Index             = 0;

    WmcCheckForRxData();

    WmcPowerOffOnButton.update();
    WmcPulseSwitchPushButton.update();
    buttonActual = WmcFunctionButtons();

    WmcCommandLine.Update();

    if (WmcState != WmcStateRequest)
    {
        WmcState = WmcStateRequest;
    }

    if (millis() - WmcStartMsKeepAlive > 3000)
    {
        WmcStartMsKeepAlive = millis();

        switch (WmcState)
        {
        case WMC_STATE_POWER_OFF:
        case WMC_STATE_POWER_ON: WmcSlave.LanXGetLocoInfo(WmcLocLib.GetActualLocAddress()); break;
        case WMC_STATE_MENU_SET:
        case WMC_STATE_MENU:
        case WMC_STATE_LOC_ADD_SET:
        case WMC_STATE_LOC_ADD:
        case WMC_STATE_LOC_ADD_FUNCTION_SET:
        case WMC_STATE_LOC_ADD_FUNCTION:
        case WMC_STATE_LOC_ADD_STORE:
        case WMC_STATE_LOC_CHANGE_SET:
        case WMC_STATE_LOC_CHANGE:
        case WMC_STATE_LOC_REMOVE_SET:
        case WMC_STATE_LOC_REMOVE:
        case WMC_STATE_LOC_EXIT:
            // In menu modes ask for status so connection does not drop off.
            WmcSlave.LanGetStatus();
            break;
        default: break;
        }
    }

    switch (WmcState)
    {
    case WMC_STATE_INIT_CONNECT:
        // Try to start communication with Z21
        WmcSlave.LanGetStatus();

        // Now lets' check if we got some data back with some delay...
        WmcStartMs      = millis();
        WmcStateRequest = WMC_STATE_INIT_CONNECT_RESPONSE;
        break;
    case WMC_STATE_INIT_CONNECT_RESPONSE:
        if (WmcResponseTimeOut() == true)
        {
            WmcSlave.LanGetStatus();
        }
        break;
    case WMC_STATE_INIT_BROADCAST:
        WmcSlave.LanSetBroadCastFlags(0x01);
        delay(200);
        WmcStateRequest = WMC_STATE_INIT_STATUS_GET;
        break;
    case WMC_STATE_INIT_STATUS_GET:
        WmcStartMs = millis();
        WmcSlave.LanGetStatus();
        WmcStateRequest = WMC_STATE_INIT_STATUS_GET_RESPONSE;
        break;
    case WMC_STATE_INIT_STATUS_GET_RESPONSE:
        if (WmcResponseTimeOut() == true)
        {
            WmcSlave.LanGetStatus();
        }
        break;
    case WMC_STATE_INIT_FIRMWARE_GET:
        WmcStartMs = millis();
        WmcSlave.LanGetVersion();
        WmcStateRequest = WMC_STATE_INIT_FIRMWARE_GET_RESPONSE;
        break;
    case WMC_STATE_INIT_FIRMWARE_GET_RESPONSE:
        if (WmcResponseTimeOut() == true)
        {
            WmcSlave.LanGetVersion();
        }
        break;
    case WMC_STATE_INIT_GET_LOCO_INFO:
        WmcStartMs = millis();
        WmcSlave.LanXGetLocoInfo(WmcLocLib.GetActualLocAddress());
        WmcStateRequest = WMC_STATE_INIT_GET_LOCO_INFO_RESPONSE;
        break;
    case WMC_STATE_INIT_GET_LOCO_INFO_RESPONSE:
        if (WmcResponseTimeOut() == true)
        {
            WmcSlave.LanXGetLocoInfo(WmcLocLib.GetActualLocAddress());
        }
        break;
    case WMC_STATE_INIT_GET_LOCO_MODE:
        WmcStartMs = millis();
        WmcSlave.LanGetLocoMode(WmcLocLib.GetActualLocAddress());
        WmcStateRequest = WMC_STATE_INIT_GET_LOCO_MODE_RESPONSE;
        break;
    case WMC_STATE_INIT_GET_LOCO_MODE_RESPONSE:
        if (WmcResponseTimeOut() == true)
        {
            WmcSlave.LanGetLocoMode(WmcLocLib.GetActualLocAddress());
        }
        break;
    case WMC_STATE_POWER_OFF_SET:
        DecoderUpdate();

        WmcStartMsKeepAlive             = millis();
        WmcStartMsPulseSwitchPushButton = millis();

        WmcDisplay.UpdateStatus("PowerOff", false, WmcTft::color_red);
        WmcDisplay.UpdateSelectedAndNumberOfLocs(WmcLocLib.GetActualSelectedLocIndex(), WmcLocLib.GetNumberOfLocs());

        WmcStateRequest = WMC_STATE_POWER_OFF;
        break;
    case WMC_STATE_POWER_OFF:
        Delta = DecoderUpdate();
        if (WmcPowerOffOnButton.rose() == true)
        {
            WmcSlave.LanSetTrackPowerOn();
        }
        else if (WmcPulseSwitchPushButton.fell() == true)
        {
            WmcStartMsPulseSwitchPushButton = millis();
        }
        else if (WmcPulseSwitchPushButton.rose() == true)
        {
            if (WmcLocSelection == true)
            {
                WmcLocSelection = false;
            }
            else
            {
                if (millis() - WmcStartMsPulseSwitchPushButton > 3000)
                {
                    WmcStateRequest = WMC_STATE_MENU_SET;
                }
            }
        }
        else if (Delta != 0)
        {
            if (WmcPulseSwitchPushButton.read() == LOW)
            {
                /* Select another locomotive. */
                WmcLocSelection = true;
                WmcLocLib.GetNextLoc(Delta);
                WmcSlave.LanXGetLocoInfo(WmcLocLib.GetActualLocAddress());
                WmcDisplay.UpdateSelectedAndNumberOfLocs(
                    WmcLocLib.GetActualSelectedLocIndex(), WmcLocLib.GetNumberOfLocs());
            }
        }
        break;
    case WMC_STATE_POWER_ON_SET:
        WmcStartMsKeepAlive = millis();
        WmcDisplay.UpdateStatus("PowerOn", false, WmcTft::color_green);
        WmcDisplay.UpdateSelectedAndNumberOfLocs(WmcLocLib.GetActualSelectedLocIndex(), WmcLocLib.GetNumberOfLocs());
        WmcStateRequest = WMC_STATE_POWER_ON;
        break;
    case WMC_STATE_POWER_ON:
        if (WmcPowerOffOnButton.rose() == true)
        {
            WmcSlave.LanSetTrackPowerOff();
        }
        else if (WmcPulseSwitchPushButton.fell() == true)
        {
            WmcStartMsPulseSwitchPushButton = millis();
        }
        else if (buttonActual != 255)
        {
            Function = WmcLocLib.FunctionAssignedGet(buttonActual);
            WmcLocLib.FunctionToggle(Function);
            if (WmcLocLib.FunctionStatusGet(Function) == LocLib::functionOn)
            {
                WmcSlave.LanXSetLocoFunction(WmcLocLib.GetActualLocAddress(), Function, Z21Slave::on);
            }
            else
            {
                WmcSlave.LanXSetLocoFunction(WmcLocLib.GetActualLocAddress(), Function, Z21Slave::off);
            }
        }
        else if (WmcPulseSwitchPushButton.rose() == true)
        {
            if (WmcLocSelection == false)
            {
                if (millis() - WmcStartMsPulseSwitchPushButton < 300)
                {
                    if (WmcLocLib.SpeedSet(0) == true)
                    {
                        if (WmcLocLib.DirectionGet() == LocLib::directionForward)
                        {
                            WmcLocInfoControl.Direction = Z21Slave::locDirectionForward;
                        }
                        else
                        {
                            WmcLocInfoControl.Direction = Z21Slave::locDirectionBackward;
                        }

                        WmcLocInfoControl.Speed = 0;

                        switch (WmcLocLib.DecoderStepsGet())
                        {
                        case LocLib::decoderStep14: WmcLocInfoControl.Steps = Z21Slave::locDecoderSpeedSteps14; break;
                        case LocLib::decoderStep28: WmcLocInfoControl.Steps = Z21Slave::locDecoderSpeedSteps28; break;
                        case LocLib::decoderStep128: WmcLocInfoControl.Steps = Z21Slave::locDecoderSpeedSteps128; break;
                        }
                        WmcSlave.LanXSetLocoDrive(&WmcLocInfoControl);
                        ;
                    }
                }
                else
                {
                    WmcLocLib.DirectionToggle();

                    if (WmcLocLib.DirectionGet() == LocLib::directionForward)
                    {
                        WmcLocInfoControl.Direction = Z21Slave::locDirectionForward;
                    }
                    else
                    {
                        WmcLocInfoControl.Direction = Z21Slave::locDirectionBackward;
                    }

                    switch (WmcLocLib.DecoderStepsGet())
                    {
                    case LocLib::decoderStep14: WmcLocInfoControl.Steps = Z21Slave::locDecoderSpeedSteps14; break;
                    case LocLib::decoderStep28: WmcLocInfoControl.Steps = Z21Slave::locDecoderSpeedSteps28; break;
                    case LocLib::decoderStep128: WmcLocInfoControl.Steps = Z21Slave::locDecoderSpeedSteps128; break;
                    }
                    WmcSlave.LanXSetLocoDrive(&WmcLocInfoControl);
                }
            }
            else
            {
                WmcLocSelection                 = false;
                WmcStartMsPulseSwitchPushButton = millis();
            }
        }
        else
        {
            Delta = DecoderUpdate();
            if (Delta != 0)
            {
                if (WmcPulseSwitchPushButton.read() == LOW)
                {
                    /* Select another locomotive. */
                    WmcLocSelection = true;
                    WmcLocLib.GetNextLoc(Delta);
                    WmcSlave.LanXGetLocoInfo(WmcLocLib.GetActualLocAddress());
                    WmcDisplay.UpdateSelectedAndNumberOfLocs(
                        WmcLocLib.GetActualSelectedLocIndex(), WmcLocLib.GetNumberOfLocs());
                }
                else
                {
                    if (WmcLocLib.SpeedSet(Delta) == true)
                    {
                        WmcLocInfoControl.Speed = WmcLocLib.SpeedGet();
                        if (WmcLocLib.DirectionGet() == LocLib::directionForward)
                        {
                            WmcLocInfoControl.Direction = Z21Slave::locDirectionForward;
                        }
                        else
                        {
                            WmcLocInfoControl.Direction = Z21Slave::locDirectionBackward;
                        }

                        WmcLocInfoControl.Address = WmcLocLib.GetActualLocAddress();

                        switch (WmcLocLib.DecoderStepsGet())
                        {
                        case LocLib::decoderStep14: WmcLocInfoControl.Steps = Z21Slave::locDecoderSpeedSteps14; break;
                        case LocLib::decoderStep28: WmcLocInfoControl.Steps = Z21Slave::locDecoderSpeedSteps28; break;
                        case LocLib::decoderStep128: WmcLocInfoControl.Steps = Z21Slave::locDecoderSpeedSteps128; break;
                        }

                        WmcSlave.LanXSetLocoDrive(&WmcLocInfoControl);
                    }
                }
            }
        }
        break;
    case WMC_STATE_MENU_SET:
        WmcDisplay.ShowMenu();
        WmcStateRequest = WMC_STATE_MENU;
        break;
    case WMC_STATE_MENU:
        if (buttonActual != 255)
        {
            switch (buttonActual)
            {
            case 1:
                WmcStateRequest  = WMC_STATE_LOC_ADD_SET;
                WmcLocAddressAdd = WmcLocLib.GetActualLocAddress();
                break;
            case 2: WmcStateRequest = WMC_STATE_LOC_CHANGE_SET; break;
            case 3: WmcStateRequest = WMC_STATE_LOC_REMOVE_SET; break;
            case 4:
                WmcStateRequest = WMC_STATE_LOC_EXIT;
                WmcSlave.LanXGetLocoInfo(WmcLocLib.GetActualLocAddress());
                break;
            default: break;
            }
        }
        else if (WmcPowerOffOnButton.rose() == true)
        {
            WmcStateRequest = WMC_STATE_LOC_EXIT;
            WmcSlave.LanXGetLocoInfo(WmcLocLib.GetActualLocAddress());
        }
        break;
    case WMC_STATE_LOC_ADD_SET:
        WmcDisplay.Clear();
        WmcDisplay.UpdateStatus("ADD LOC", true, WmcTft::color_green);
        WmcDisplay.ShowLocSymbolFw();
        WmcDisplay.ShowlocAddress(WmcLocAddressAdd, WmcTft::color_green);
        WmcDisplay.UpdateSelectedAndNumberOfLocs(WmcLocLib.GetActualSelectedLocIndex(), WmcLocLib.GetNumberOfLocs());
        WmcStateRequest = WMC_STATE_LOC_ADD;
        break;
    case WMC_STATE_LOC_ADD:
        if (WmcPowerOffOnButton.rose() == true)
        {
            WmcStateRequest = WMC_STATE_MENU_SET;
        }
        else if (WmcPulseSwitchPushButton.rose() == true)
        {
            // Before continuing check if the loc is present...
            if (WmcLocLib.CheckLoc(WmcLocAddressAdd) != 255)
            {
                WmcDisplay.ShowlocAddress(WmcLocAddressAdd, WmcTft::color_red);
            }
            else
            {
                WmcStateRequest = WMC_STATE_LOC_ADD_FUNCTION_SET;
            }
        }
        else
        {
            Delta = DecoderUpdate();
            if (Delta != 0)
            {
                if (Delta > 0)
                {
                    LocAddUpdateScreen = true;
                    WmcLocAddressAdd++;
                    if (WmcLocAddressAdd == 10000)
                    {
                        WmcLocAddressAdd = 1;
                    }
                }
                else
                {
                    LocAddUpdateScreen = true;
                    WmcLocAddressAdd--;
                    if (WmcLocAddressAdd == 0)
                    {
                        WmcLocAddressAdd = 9999;
                    }
                }
            }
            else if (buttonActual != 255)
            {
                switch (buttonActual)
                {
                case 1:
                    WmcLocAddressAdd++;
                    LocAddUpdateScreen = true;
                    break;
                case 2:
                    WmcLocAddressAdd += 10;
                    LocAddUpdateScreen = true;
                    break;
                case 3:
                    WmcLocAddressAdd += 100;
                    LocAddUpdateScreen = true;
                    break;
                case 4:
                    WmcLocAddressAdd += 1000;
                    LocAddUpdateScreen = true;
                    break;
                case 0: WmcLocAddressAdd = 1; LocAddUpdateScreen = true;
                default: break;
                }
            }

            if (LocAddUpdateScreen == true)
            {
                if (WmcLocAddressAdd > 9999)
                {
                    WmcLocAddressAdd -= 10000;
                }

                WmcDisplay.ShowlocAddress(WmcLocAddressAdd, WmcTft::color_green);
            }
        }
        break;
    case WMC_STATE_LOC_ADD_FUNCTION_SET:
        WmcDisplay.UpdateStatus("FUNCTIONS", true, WmcTft::color_green);
        WmcLocAddFunction = 0;
        for (Index = 0; Index < 5; Index++)
        {
            WmcLocAddFunctionAssignment[Index] = Index;
        }

        WmcDisplay.FunctionAddSet();
        WmcDisplay.UpdateSelectedAndNumberOfLocs(WmcLocLib.GetActualSelectedLocIndex(), WmcLocLib.GetNumberOfLocs());

        WmcStateRequest = WMC_STATE_LOC_ADD_FUNCTION;
        break;
    case WMC_STATE_LOC_ADD_FUNCTION:
        if (WmcPowerOffOnButton.rose() == true)
        {
            WmcStateRequest = WMC_STATE_MENU_SET;
        }
        else if (WmcPulseSwitchPushButton.rose() == true)
        {
            WmcStateRequest = WMC_STATE_LOC_ADD_STORE;
        }
        else
        {
            Delta = DecoderUpdate();
            if (Delta != 0)
            {
                if (Delta > 0)
                {
                    LocAddUpdateScreen = true;
                    WmcLocAddFunction++;
                    if (WmcLocAddFunction == 29)
                    {
                        WmcLocAddFunction = 0;
                    }
                }
                else
                {
                    LocAddUpdateScreen = true;
                    if (WmcLocAddFunction == 0)
                    {
                        WmcLocAddFunction = 28;
                    }
                    else
                    {
                        WmcLocAddFunction--;
                    }
                }
            }
            else if (buttonActual != 255)
            {
                switch (buttonActual)
                {
                case 0:
                    WmcLocAddFunctionAssignment[buttonActual] = WmcLocAddFunction;
                    LocAddUpdateFunction                      = true;
                    break;
                case 1:
                case 2:
                case 3:
                case 4:
                    /* Light can only be assigned to button 0. */
                    if (WmcLocAddFunction != 0)
                    {
                        WmcLocAddFunctionAssignment[buttonActual] = WmcLocAddFunction;
                        LocAddUpdateFunction                      = true;
                    }
                    break;
                default: LocAddUpdateFunction = false; break;
                }
            }

            if (LocAddUpdateScreen == true)
            {
                WmcDisplay.FunctionAddUpdate(WmcLocAddFunction);
            }
            if (LocAddUpdateFunction == true)
            {
                WmcDisplay.UpdateFunction(buttonActual, WmcLocAddFunctionAssignment[buttonActual]);
            }
        }
        break;
    case WMC_STATE_LOC_CHANGE_SET:
        WmcDisplay.Clear();
        WmcLocAddFunction = 0;
        WmcLocAddressAdd  = WmcLocLib.GetActualLocAddress();
        WmcDisplay.UpdateStatus("CHANGE FUNC", true, WmcTft::color_green);
        WmcDisplay.ShowLocSymbolFw();
        WmcDisplay.ShowlocAddress(WmcLocAddressAdd, WmcTft::color_green);
        WmcDisplay.FunctionAddUpdate(WmcLocAddFunction);
        WmcDisplay.UpdateSelectedAndNumberOfLocs(WmcLocLib.GetActualSelectedLocIndex(), WmcLocLib.GetNumberOfLocs());

        for (Index = 0; Index < 5; Index++)
        {
            WmcLocAddFunctionAssignment[Index] = WmcLocLib.FunctionAssignedGet(Index);
            WmcDisplay.UpdateFunction(Index, WmcLocAddFunctionAssignment[Index]);
        }
        WmcStateRequest = WMC_STATE_LOC_CHANGE;
        break;
    case WMC_STATE_LOC_CHANGE:
        if (WmcPowerOffOnButton.rose() == true)
        {
            WmcStateRequest = WMC_STATE_MENU_SET;
        }
        else if (WmcPulseSwitchPushButton.rose() == true)
        {
            if (WmcLocSelection == true)
            {
                WmcLocSelection = false;
            }
            else
            {
                WmcLocLib.StoreLoc(WmcLocAddressAdd, WmcLocAddFunctionAssignment, LocLib::storeChange);
                WmcDisplay.ShowlocAddress(WmcLocAddressAdd, WmcTft::color_yellow);
            }
        }
        else
        {
            Delta = DecoderUpdate();
            if (WmcPulseSwitchPushButton.read() == LOW)
            {
                /* Select another locomotive. */
                if (Delta != 0)
                {
                    WmcLocAddressAdd   = WmcLocLib.GetNextLoc(Delta);
                    LocAddUpdateScreen = true;
                    WmcLocSelection    = true;

                    WmcDisplay.UpdateSelectedAndNumberOfLocs(
                        WmcLocLib.GetActualSelectedLocIndex(), WmcLocLib.GetNumberOfLocs());

                    for (Index = 0; Index < 5; Index++)
                    {
                        WmcLocAddFunctionAssignment[Index] = WmcLocLib.FunctionAssignedGet(Index);
                        WmcDisplay.UpdateFunction(Index, WmcLocAddFunctionAssignment[Index]);
                    }

                    WmcDisplay.ShowlocAddress(WmcLocAddressAdd, WmcTft::color_green);

                    DecoderUpdate();
                }
            }
            else
            {
                if (Delta != 0)
                {
                    if (Delta > 0)
                    {
                        LocAddUpdateScreen = true;
                        WmcLocAddFunction++;
                        if (WmcLocAddFunction >= 29)
                        {
                            WmcLocAddFunction = 0;
                        }
                    }
                    else
                    {
                        LocAddUpdateScreen = true;
                        if (WmcLocAddFunction == 0)
                        {
                            WmcLocAddFunction = 28;
                        }
                        else
                        {
                            WmcLocAddFunction--;
                        }
                    }
                }
                else if (buttonActual != 255)
                {
                    switch (buttonActual)
                    {
                    case 0:
                        WmcLocAddFunctionAssignment[buttonActual] = WmcLocAddFunction;
                        LocAddUpdateFunction                      = true;
                        break;
                    case 1:
                    case 2:
                    case 3:
                    case 4:
                        /* Light can only be assigned to button 0. */
                        if (WmcLocAddFunction != 0)
                        {
                            WmcLocAddFunctionAssignment[buttonActual] = WmcLocAddFunction;
                            LocAddUpdateFunction                      = true;
                        }
                        break;
                    default: LocAddUpdateFunction = false; break;
                    }
                }
            }

            if (LocAddUpdateScreen == true)
            {
                WmcDisplay.FunctionAddUpdate(WmcLocAddFunction);
                DecoderUpdate();
            }

            if (LocAddUpdateFunction == true)
            {
                WmcDisplay.UpdateFunction(buttonActual, WmcLocAddFunctionAssignment[buttonActual]);
                DecoderUpdate();
            }
        }
        break;
    case WMC_STATE_LOC_ADD_STORE:
        WmcLocLib.StoreLoc(WmcLocAddressAdd, WmcLocAddFunctionAssignment, LocLib::storeAdd);
        WmcLocLib.LocBubbleSort();
        WmcLocAddressAdd++;
        WmcStateRequest = WMC_STATE_LOC_ADD_SET;
        break;
    case WMC_STATE_LOC_REMOVE_SET:
        WmcDisplay.Clear();
        WmcLocAddressAdd = WmcLocLib.GetActualLocAddress();
        WmcDisplay.UpdateStatus("DELETE", true, WmcTft::color_green);
        WmcDisplay.ShowLocSymbolFw();
        WmcDisplay.ShowlocAddress(WmcLocAddressAdd, WmcTft::color_green);
        WmcDisplay.UpdateSelectedAndNumberOfLocs(WmcLocLib.GetActualSelectedLocIndex(), WmcLocLib.GetNumberOfLocs());
        WmcStateRequest = WMC_STATE_LOC_REMOVE;
        break;
    case WMC_STATE_LOC_REMOVE:
        Delta = DecoderUpdate();
        if (WmcPowerOffOnButton.rose() == true)
        {
            WmcStateRequest = WMC_STATE_MENU_SET;
        }
        else if (WmcPulseSwitchPushButton.rose() == true)
        {
            WmcLocLib.RemoveLoc(WmcLocAddressAdd);
            WmcDisplay.UpdateSelectedAndNumberOfLocs(
                WmcLocLib.GetActualSelectedLocIndex(), WmcLocLib.GetNumberOfLocs());
            WmcLocAddressAdd   = WmcLocLib.GetActualLocAddress();
            LocAddUpdateScreen = true;
        }
        else if (Delta != 0)
        {
            WmcLocAddressAdd = WmcLocLib.GetNextLoc(Delta);
            WmcDisplay.UpdateSelectedAndNumberOfLocs(
                WmcLocLib.GetActualSelectedLocIndex(), WmcLocLib.GetNumberOfLocs());
            LocAddUpdateScreen = true;
        }

        if (LocAddUpdateScreen == true)
        {
            WmcDisplay.ShowlocAddress(WmcLocAddressAdd, WmcTft::color_green);
        }
        break;
    case WMC_STATE_LOC_EXIT:
    case WMC_STATE_CONNECTION_FAILED: break;
    }
}
