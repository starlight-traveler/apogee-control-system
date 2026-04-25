/*!
 * \file            main.c
 * \author          SBG Systems
 * \date            July 27, 2021
 *
 * \brief           C code sample demonstrating the usage of sbgECom and sbgInsRestApi.
 *
 * This simple C program:
 *   - Detects a device using sbgECom and sbgInsRestApi.
 *   - Configures the device to output data at 10 Hz.
 *   - Displays received data on the console.
 *
 * The code uses a serial connection.
 * It is compatible with:
 *   - ELLIPSE devices (firmware v3 and above)
 *   - High Performance INS (HPINS) products
 *   - PULSE Inertial Measurement Units (IMUs)
 * 
 * 
 * The code is written in plain ANSI C and is designed to work on any POSIX 
 * and Windows system. With minor modifications, it can also run on bare-metal devices.
 * 
 * Dependencies:
 *   - Uses the open-source cJSON library by Dave Gamble (https://github.com/DaveGamble/cJSON)
 *     to parse incoming JSON replies from the device.
 *
 * \copyright       Copyright (C) 2007-2026, SBG Systems SAS. All rights reserved.
 * \beginlicense    The MIT license
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * \endlicense
 */

// Standard Library
#include <signal.h>
#include <string.h>

// sbgCommonLib headers
#include <sbgCommon.h>

// sbgECom headers
#include <sbgEComLib.h>

// Local headers
#include "cJSON.h"
#include "restApiHelper.h"

//----------------------------------------------------------------------//
//- Constant definitions                                               -//
//----------------------------------------------------------------------//

/*!
 * Program name.
 */
#define PROGRAM_NAME                                        "sbgEComExample"

/*!
 * Time to wait before trying to connect to the device following a software reboot
 */
#define SBG_ECOM_EXAMPLE_BOOT_TIME_MS                               (500)

//----------------------------------------------------------------------//
//- Gloval private members                                             -//
//----------------------------------------------------------------------//

/*!
 * Global variable used to gracefull exit the example when the user hit CTRL+C
 */
volatile sig_atomic_t gShouldExit = 0;

//----------------------------------------------------------------------//
//- Private methods                                                    -//
//----------------------------------------------------------------------//

/*!
 * Callback definition used to print log error messages.
 *
 * \param[in]   pFileName                   The file in which the log was triggered.
 * \param[in]   pFunctionName               The function where the log was triggered.
 * \param[in]   line                        The line in the file where the log was triggered.
 * \param[in]   pCategory                   Category for this log or "None" if no category has been specified.
 * \param[in]   logType                     Associated log message level.
 * \param[in]   errorCode                   Associated error code or SBG_NO_ERROR for INFO & DEBUG level logs.
 * \param[in]   pMessage                    The message to log.
 */
static void sbgEComExamplePrintMsgLog(const char *pFileName, const char *pFunctionName, uint32_t line, const char *pCategory, SbgDebugLogType type, SbgErrorCode errorCode, const char *pMessage)
{
    const char                          *pTypeStr;

    assert(pFileName);
    assert(pFunctionName);
    assert(pCategory);
    assert(pMessage);

    SBG_UNUSED_PARAMETER(pFileName);
    SBG_UNUSED_PARAMETER(line);

    pTypeStr    = sbgDebugLogTypeToStr(type);

    if (errorCode == SBG_NO_ERROR)
    {
        fprintf(stderr, "%-7s %s: %s\n", pTypeStr, pFunctionName, pMessage);
    }
    else
    {
        fprintf(stderr, "%-7s %s %s: %s\n", pTypeStr, sbgErrorCodeToString(errorCode), pFunctionName, pMessage);
    }
}

/*!
 * Callback used to handle received sbgECom logs.
 *
 * \param[in]   pECom                   SbgECom instance.
 * \param[in]   msgClass                Class of the received message.
 * \param[in]   msg                     Received message ID.
 * \param[in]   pLogData                Received data.
 * \param[in]   pUserArg                Optional user argument.
 * \return                              SBG_NO_ERROR if successful.
 */
static SbgErrorCode sbgEComExampleOnLogReceived(SbgEComHandle *pECom, SbgEComClass msgClass, SbgEComMsgId msg, const SbgEComLogUnion *pLogData, void *pUserArg)
{
    SBG_UNUSED_PARAMETER(pECom);
    SBG_UNUSED_PARAMETER(pUserArg);

    assert(pLogData);

    if (msgClass == SBG_ECOM_CLASS_LOG_ECOM_0)
    {
        switch (msg)
        {
        case SBG_ECOM_LOG_IMU_SHORT:
            printf("IMU  : %" PRIu32 "\t%" PRIu16 "\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\n",
                pLogData->imuShort.timeStamp,
                pLogData->imuShort.status,
                sbgEComLogImuShortGetDeltaVelocity(&pLogData->imuShort, 0),
                sbgEComLogImuShortGetDeltaVelocity(&pLogData->imuShort, 1),
                sbgEComLogImuShortGetDeltaVelocity(&pLogData->imuShort, 2),
                sbgRadToDegf(sbgEComLogImuShortGetDeltaAngle(&pLogData->imuShort, 0)),
                sbgRadToDegf(sbgEComLogImuShortGetDeltaAngle(&pLogData->imuShort, 1)),
                sbgRadToDegf(sbgEComLogImuShortGetDeltaAngle(&pLogData->imuShort, 2)),
                sbgEComLogImuShortGetTemperature(&pLogData->imuShort));
            break;

        case SBG_ECOM_LOG_EKF_EULER:
            printf("EULER: %" PRIu32 "\t%" PRIu16 "\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\n",
                pLogData->ekfEulerData.timeStamp,
                pLogData->ekfEulerData.status,
                sbgRadToDegf(pLogData->ekfEulerData.euler[0]),
                sbgRadToDegf(pLogData->ekfEulerData.euler[1]),
                sbgRadToDegf(pLogData->ekfEulerData.euler[2]),
                sbgRadToDegf(pLogData->ekfEulerData.eulerStdDev[0]),
                sbgRadToDegf(pLogData->ekfEulerData.eulerStdDev[1]),
                sbgRadToDegf(pLogData->ekfEulerData.eulerStdDev[2]),
                sbgRadToDegf(pLogData->ekfEulerData.magDeclination));
            break;
        default:
            break;
        }
    }

    return SBG_NO_ERROR;
}

/*!
 * Receive logs and only exit upon user request.
 *
 * \param[in]   pECom                   SbgECom instance.
 */
static void sbgEComExampleProcessIncoming(SbgEComHandle *pECom)
{
    assert(pECom);

    sbgEComSetReceiveLogCallback(pECom, sbgEComExampleOnLogReceived, NULL);

    while (!gShouldExit)
    {
        SbgErrorCode                     errorCode;

        errorCode = sbgEComHandle(pECom);

        if (errorCode == SBG_NOT_READY)
        {
            sbgSleep(1);
        }
    }
}

/*!
 * Get and print product info.
 *
 * \param[in]   pECom                   SbgECom instance.
 * \return                              SBG_NO_ERROR if successful.
 */
static SbgErrorCode sbgEComExamplePrintDeviceInfo(SbgEComHandle *pECom)
{
    SbgEComCmdApiReply               reply;
    SbgErrorCode                     errorCode;

    assert(pECom);

    sbgEComPurgeIncoming(pECom);

    sbgEComCmdApiReplyConstruct(&reply);

    errorCode = sbgEComCmdApiGet(pECom, "/api/v1/info", NULL, &reply);

    if (errorCode == SBG_NO_ERROR)
    {
        cJSON   *pJsonNode = cJSON_Parse(reply.pContent);

        if (pJsonNode)
        {
            DeviceInfo      deviceInfo;

            errorCode = restApiHelperParseDeviceInfo(pJsonNode, &deviceInfo);

            if (errorCode == SBG_NO_ERROR)
            {
                printf("          product code: %s\n", deviceInfo.productCode);
                printf("         serial number: %s\n", deviceInfo.serialNumber);
                printf("     hardware revision: %s\n", deviceInfo.hwRevision);
                printf(" manufacturing version: %s\n", deviceInfo.mnfVersion);
                printf("      firmware version: %s\n", deviceInfo.fmwVersion);
                printf("    bootLoader version: %s\n", deviceInfo.btVersion);
            
                printf("\n");
            }
            else
            {
                SBG_LOG_ERROR(errorCode, "Received JSON is malformed");
            }
            
            cJSON_Delete(pJsonNode);
        }
        else
        {
            errorCode = SBG_ERROR;
            SBG_LOG_ERROR(errorCode, "Unable to parse the JSON content");
        }
    }
    else
    {
        SBG_LOG_ERROR(errorCode, "unable to retrieve product info");
    }

    sbgEComCmdApiReplyDestroy(&reply);

    return errorCode;
}

/*!
 * Save new settings to FLASH memory and reboot the device
 *
 * \param[in]   pECom                   SbgECom instance.
 * \return                              SBG_NO_ERROR if successful.
 */
static SbgErrorCode sbgEComExampleSaveAndReboot(SbgEComHandle *pECom)
{
    SbgErrorCode                     errorCode;
    SbgEComCmdApiReply               reply;
    
    assert(pECom);

    sbgEComCmdApiReplyConstruct(&reply);

    //
    // Save to FLASH memory
    //
    errorCode = sbgEComCmdApiPost(pECom, "/api/v1/settings/save", NULL, NULL, &reply);

    if (errorCode == SBG_NO_ERROR)
    {
        //
        // Check the reply status code that is based on HTTP codes (200 - OK)
        //
        if (reply.statusCode == 200)
        {
            SBG_LOG_DEBUG("settings saved to FLASH memory");
        }
        else
        {
            errorCode = SBG_INVALID_PARAMETER;
            SBG_LOG_ERROR(errorCode, "unable to save settings to FLASH memory");

            restApiHelperPrintErrorDetails(reply.pContent);
        }
    }
    else
    {
        SBG_LOG_ERROR(errorCode, "unable to save settings to FLASH memory");
    }

    //
    // Reboot the device for the new settings to be applied
    //
    errorCode = sbgEComCmdApiPost(pECom, "/api/v1/system/reboot", NULL, NULL, &reply);

    if (errorCode == SBG_NO_ERROR)
    {
        //
        // Check the reply status code that is based on HTTP codes (200 - OK)
        //
        if (reply.statusCode == 200)
        {
            SBG_LOG_INFO("device is rebooting...");
            sbgSleep(SBG_ECOM_EXAMPLE_BOOT_TIME_MS);
        }
        else
        {
            errorCode = SBG_INVALID_PARAMETER;
            SBG_LOG_ERROR(errorCode, "unable to reboot the device");

            restApiHelperPrintErrorDetails(reply.pContent);
        }
    }
    else
    {
        SBG_LOG_ERROR(errorCode, "unable to reboot the device");
    }

    sbgEComCmdApiReplyDestroy(&reply);

    return errorCode;
}

/*!
 * Configures the device to output IMU Short messages on Port A.
 * 
 * This function demonstrates how to modify device settings using the SBG REST API.  
 * It also verifies the device response to determine if a reboot is required for 
 * the updated settings to take effect.
 * 
 * \param[in]   pECom                   SbgECom instance.
 * \param[out]  pNeedReboot             Returns true if the applied settings require the device to reboot.
 * \return                              SBG_NO_ERROR if the configuration was successful.
 */
static SbgErrorCode sbgEComExampleConfigurePortA(SbgEComHandle *pECom, bool *pNeedReboot)
{
    SbgEComCmdApiReply               reply;
    SbgErrorCode                     errorCode;

    assert(pECom);
    assert(pNeedReboot);

    sbgEComCmdApiReplyConstruct(&reply);

    //
    // Enable SBG_ECOM_LOG_IMU_SHORT on Port A at 10Hz
    //
    errorCode = sbgEComCmdApiPost(pECom, "/api/v1/settings/output/comA/messages/imuShort", NULL, "\"100ms\"", &reply);

    if (errorCode == SBG_NO_ERROR)
    {
        //
        // Check if the POST request has been successfully exectued (HTTP code 200 - OK)
        //
        if (reply.statusCode == 200)
        {
            errorCode = restApiHelperParseSuccess(reply.pContent, pNeedReboot);

            if (errorCode != SBG_NO_ERROR)
            {
                SBG_LOG_ERROR(errorCode, "unable to parse answer from the product");
            }
        }
        else
        {
            errorCode = SBG_INVALID_PARAMETER;
            SBG_LOG_ERROR(errorCode, "unable to configure IMU short log at 10Hz");

            restApiHelperPrintErrorDetails(reply.pContent);
        }
    }
    else
    {
        SBG_LOG_ERROR(errorCode, "unable to configure IMU short log at 10Hz");
    }

    sbgEComCmdApiReplyDestroy(&reply);
    
    return errorCode;
}

/*!
 * Detect and print product information, configure it over Port A and read data.
 *
 * \param[in]   pInterface              Interface.
 * \return                              SBG_NO_ERROR if successful.
 */
static SbgErrorCode sbgEComExampleExecute(SbgInterface *pInterface)
{
    SbgErrorCode                         errorCode;
    SbgEComHandle                        comHandle;

    assert(pInterface);

    errorCode = sbgEComInit(&comHandle, pInterface);

    if (errorCode == SBG_NO_ERROR)
    {
        errorCode = sbgEComExamplePrintDeviceInfo(&comHandle);

        if (errorCode == SBG_NO_ERROR)
        {
            bool    needReboot;

            //
            // Showcase device configuration using the sbgInsRestApi over the serial Port A.
            //
            errorCode = sbgEComExampleConfigurePortA(&comHandle, &needReboot);
            
            //
            // If new settings require a reboot to take effect, save to FLASH and reboot the device
            //
            if ( (errorCode == SBG_NO_ERROR) && (needReboot) )
            {
                errorCode = sbgEComExampleSaveAndReboot(&comHandle);

                if (errorCode != SBG_NO_ERROR)
                {
                    SBG_LOG_ERROR(errorCode, "unable to save new settings and reboot the device");
                }
            }
        }

        if (errorCode == SBG_NO_ERROR)
        {
            sbgEComPurgeIncoming(&comHandle);
            sbgEComExampleProcessIncoming(&comHandle);
        }

        sbgEComClose(&comHandle);
    }

    return errorCode;
}

/*!
 * Intercept Ctrl+C events and Terminal close events to exit gracefully.
 * 
 * \param[in]   signalId                Signal identifier that has triggered the event.
 */
static void signalHandler(int signalId)
{
    SBG_UNUSED_PARAMETER(signalId);

    //
    // Notify main processing loop to stop the processing
    //
    gShouldExit = 1;
}

/*!
 * Register all signals events used to exit the logger gracefully
 */
static void registerSignalsEvents(void)
{
    signal(SIGINT,      signalHandler);
    signal(SIGABRT,     signalHandler);
    signal(SIGTERM,     signalHandler);

#ifdef SIGHUP
    signal(SIGHUP,      signalHandler);
#endif
    
#ifdef SIGBREAK
    signal(SIGBREAK,    signalHandler);
#endif
}

//----------------------------------------------------------------------//
// Main program entry point                                             //
//----------------------------------------------------------------------//

/*!
 * Program entry point.
 *
 * \param[in]   argc                        Number of input arguments.
 * \param[in]   argv                        Input arguments as an array of strings.
 * \return                                  EXIT_SUCCESS if successful.
 */
int main(int argc, char **argv)
{
    int                                  exitCode = EXIT_FAILURE;
    SbgErrorCode                         errorCode;
    SbgInterface                         ecomInterface;

    sbgCommonLibSetLogCallback(sbgEComExamplePrintMsgLog);
    registerSignalsEvents();

    if (argc == 2)
    {
        if ( (strcmp(argv[1], "-h") == 0) || (strcmp(argv[1], "--help") == 0) )
        {
            printf("Usage: %s SERIAL_DEVICE SERIAL_BAUDRATE\n", PROGRAM_NAME);
            printf("\n");
            printf("sbgECom simple C example that displays device info and prints outputs using a serial interface.\n");
            return EXIT_SUCCESS;
        }
        else if ( (strcmp(argv[1], "-v") == 0) || (strcmp(argv[1], "--version") == 0) )
        {
            printf("%s\n", sbgEComGetVersionAsString());
            return EXIT_SUCCESS;
        }
    }

    if (argc == 3)
    {
        errorCode = sbgInterfaceSerialCreate(&ecomInterface, argv[1], atoi(argv[2]));

        if (errorCode == SBG_NO_ERROR)
        {
            printf("Welcome to the sbgECom v%s C code sample.\n", sbgEComGetVersionAsString());
            printf("You can exit this example by pressing the CTRL+C keys.\n");
            puts("");

            errorCode = sbgEComExampleExecute(&ecomInterface);

            if (errorCode == SBG_NO_ERROR)
            {
                exitCode = EXIT_SUCCESS;
            }

            sbgInterfaceDestroy(&ecomInterface);
        }
        else
        {
            SBG_LOG_ERROR(errorCode, "unable to open the serial interface");
        }
    }
    else
    {
        fprintf(stderr, "Usage: %s SERIAL_DEVICE SERIAL_BAUDRATE\n", PROGRAM_NAME);
    }

    return exitCode;
}
