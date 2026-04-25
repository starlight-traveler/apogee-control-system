/*!
 * \file            main.c
 * \author          SBG Systems
 * \date            28/03/2014
 *
 * \brief           C example that showcase ELLIPSE configuration and log parsing.
 *
 * This small example demonstrates how to initialize the sbgECom library
 * to read data from an Ellipse using callbacks.
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

// sbgCommonLib headers
#include <sbgCommon.h>
#include <version/sbgVersion.h>

// sbgECom headers
#include <sbgEComLib.h>

//----------------------------------------------------------------------//
//- Private methods                                                    -//
//----------------------------------------------------------------------//

typedef struct _EllipseLegacyContext
{
    uint32_t    imuCount;
    uint32_t    magCount;
    uint32_t    eulerCount;
    uint32_t    quatCount;
    bool        missingEulerReported;
    bool        missingQuatReported;
} EllipseLegacyContext;

/*!
 * Return a readable string for an EKF solution mode.
 *
 * \param[in]   solutionMode                         EKF solution mode.
 * \return                                          NULL terminated string.
 */
static const char *solutionModeToString(SbgEComSolutionMode solutionMode)
{
    switch (solutionMode)
    {
    case SBG_ECOM_SOL_MODE_UNINITIALIZED:
        return "UNINITIALIZED";
    case SBG_ECOM_SOL_MODE_VERTICAL_GYRO:
        return "VERTICAL_GYRO";
    case SBG_ECOM_SOL_MODE_AHRS:
        return "AHRS";
    case SBG_ECOM_SOL_MODE_NAV_VELOCITY:
        return "NAV_VELOCITY";
    case SBG_ECOM_SOL_MODE_NAV_POSITION:
        return "NAV_POSITION";
    default:
        return "UNKNOWN";
    }
}

/*!
 * Print a one shot diagnostic when the link works but no Euler logs are received.
 *
 * \param[in,out]   pContext                        Example runtime context.
 */
static void reportMissingEulerIfNeeded(EllipseLegacyContext *pContext)
{
    assert(pContext);

    if ((!pContext->missingEulerReported) && (pContext->eulerCount == 0u) && (pContext->imuCount >= 25u))
    {
        printf("\nNo EKF_EULER logs received yet, but IMU data is streaming.\n");
        printf("This usually means the serial link works and the issue is on the device side:\n");
        printf("the EKF may still be uninitialized, heading may be unavailable, or the legacy unit rejected the EKF_EULER output configuration.\n\n");

        pContext->missingEulerReported = true;
    }
}

/*!
 * Print a one shot diagnostic when the link works but no quaternion logs are received.
 *
 * \param[in,out]   pContext                        Example runtime context.
 */
static void reportMissingQuatIfNeeded(EllipseLegacyContext *pContext)
{
    assert(pContext);

    if ((!pContext->missingQuatReported) && (pContext->quatCount == 0u) && (pContext->imuCount >= 25u))
    {
        printf("\nNo EKF_QUAT logs received yet, but IMU data is streaming.\n");
        printf("This usually means the serial link works and the issue is on the device side:\n");
        printf("the EKF may still be uninitialized, or the legacy unit rejected the EKF_QUAT output configuration.\n\n");

        pContext->missingQuatReported = true;
    }
}

/*!
 * Read back and print the configured output mode for one log.
 *
 * \param[in]   pECom                               SbgECom instance.
 * \param[in]   msgId                               Log id.
 * \param[in]   pLogName                            Human readable log name.
 */
static void printOutputConfiguration(SbgEComHandle *pECom, SbgEComMsgId msgId, const char *pLogName)
{
    SbgErrorCode            errorCode;
    SbgEComOutputMode       outputMode;

    assert(pECom);
    assert(pLogName);

    errorCode = sbgEComCmdOutputGetConf(pECom, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, msgId, &outputMode);

    if (errorCode == SBG_NO_ERROR)
    {
        printf("%s output mode: %u\n", pLogName, (unsigned int)outputMode);
    }
    else
    {
        SBG_LOG_WARNING(errorCode, "Unable to read back output configuration");
    }
}

/*!
 * Callback definition called each time a new log is received.
 * 
 * \param[in]   pHandle                                 Valid handle on the sbgECom instance that has called this callback.
 * \param[in]   msgClass                                Class of the message we have received
 * \param[in]   msg                                     Message ID of the log received.
 * \param[in]   pLogData                                Contains the received log data as an union.
 * \param[in]   pUserArg                                Optional user supplied argument.
 * \return                                              SBG_NO_ERROR if the received log has been used successfully.
 */
static SbgErrorCode onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgEComLogUnion *pLogData, void *pUserArg)
{
    EllipseLegacyContext    *pContext;

    assert(pLogData);

    SBG_UNUSED_PARAMETER(pHandle);

    pContext = (EllipseLegacyContext*)pUserArg;

    if (msgClass == SBG_ECOM_CLASS_LOG_ECOM_0)
    {
        //
        // Handle separately each received data according to the log ID
        //
        switch (msg)
        {
        case SBG_ECOM_LOG_IMU_DATA:
            if (pContext)
            {
                pContext->imuCount++;
            }

            printf("IMU  : t=%" PRIu32 " acc=%8.4f %8.4f %8.4f m/s^2 gyro=%8.4f %8.4f %8.4f rad/s status=0x%04" PRIX16 "\n",
                pLogData->imuData.timeStamp,
                pLogData->imuData.accelerometers[0],   pLogData->imuData.accelerometers[1],   pLogData->imuData.accelerometers[2],
                pLogData->imuData.gyroscopes[0],       pLogData->imuData.gyroscopes[1],       pLogData->imuData.gyroscopes[2],
                pLogData->imuData.status);

            if (pContext)
            {
                reportMissingEulerIfNeeded(pContext);
                reportMissingQuatIfNeeded(pContext);
            }
            break;

        case SBG_ECOM_LOG_MAG:
            if (pContext)
            {
                pContext->magCount++;
            }

            printf("MAG  : t=%" PRIu32 " mag=%8.4f %8.4f %8.4f a.u. acc=%8.4f %8.4f %8.4f m/s^2 status=0x%04" PRIX16 "\n",
                pLogData->magData.timeStamp,
                pLogData->magData.magnetometers[0],    pLogData->magData.magnetometers[1],    pLogData->magData.magnetometers[2],
                pLogData->magData.accelerometers[0],   pLogData->magData.accelerometers[1],   pLogData->magData.accelerometers[2],
                pLogData->magData.status);
            break;

        case SBG_ECOM_LOG_EKF_EULER:
            if (pContext)
            {
                pContext->eulerCount++;
            }

            printf("EULER: t=%" PRIu32 " mode=%s att=%u hdg=%u align=%u mag=%u rpy=%7.2f %7.2f %7.2f deg std=%5.2f %5.2f %5.2f deg status=0x%08" PRIX32 "\n",
                pLogData->ekfEulerData.timeStamp,
                solutionModeToString(sbgEComLogEkfGetSolutionMode(pLogData->ekfEulerData.status)),
                (unsigned int)((pLogData->ekfEulerData.status & SBG_ECOM_SOL_ATTITUDE_VALID) != 0u),
                (unsigned int)((pLogData->ekfEulerData.status & SBG_ECOM_SOL_HEADING_VALID) != 0u),
                (unsigned int)((pLogData->ekfEulerData.status & SBG_ECOM_SOL_ALIGN_VALID) != 0u),
                (unsigned int)((pLogData->ekfEulerData.status & SBG_ECOM_SOL_MAG_REF_USED) != 0u),
                sbgRadToDegf(pLogData->ekfEulerData.euler[0]),          sbgRadToDegf(pLogData->ekfEulerData.euler[1]),          sbgRadToDegf(pLogData->ekfEulerData.euler[2]),
                sbgRadToDegf(pLogData->ekfEulerData.eulerStdDev[0]),    sbgRadToDegf(pLogData->ekfEulerData.eulerStdDev[1]),    sbgRadToDegf(pLogData->ekfEulerData.eulerStdDev[2]),
                pLogData->ekfEulerData.status);
            break;

        case SBG_ECOM_LOG_EKF_QUAT:
            if (pContext)
            {
                pContext->quatCount++;
            }

            printf("QUAT : t=%" PRIu32 " mode=%s att=%u hdg=%u align=%u mag=%u q=%7.4f %7.4f %7.4f %7.4f std=%5.2f %5.2f %5.2f deg status=0x%08" PRIX32 "\n",
                pLogData->ekfQuatData.timeStamp,
                solutionModeToString(sbgEComLogEkfGetSolutionMode(pLogData->ekfQuatData.status)),
                (unsigned int)((pLogData->ekfQuatData.status & SBG_ECOM_SOL_ATTITUDE_VALID) != 0u),
                (unsigned int)((pLogData->ekfQuatData.status & SBG_ECOM_SOL_HEADING_VALID) != 0u),
                (unsigned int)((pLogData->ekfQuatData.status & SBG_ECOM_SOL_ALIGN_VALID) != 0u),
                (unsigned int)((pLogData->ekfQuatData.status & SBG_ECOM_SOL_MAG_REF_USED) != 0u),
                pLogData->ekfQuatData.quaternion[0],   pLogData->ekfQuatData.quaternion[1],   pLogData->ekfQuatData.quaternion[2],   pLogData->ekfQuatData.quaternion[3],
                sbgRadToDegf(pLogData->ekfQuatData.eulerStdDev[0]),    sbgRadToDegf(pLogData->ekfQuatData.eulerStdDev[1]),    sbgRadToDegf(pLogData->ekfQuatData.eulerStdDev[2]),
                pLogData->ekfQuatData.status);
            break;
        default:
            break;
        }
    }

    return SBG_NO_ERROR;
}

/*!
 * Get and print product info.
 *
 * \param[in]   pECom                   SbgECom instance.
 * \return                              SBG_NO_ERROR if successful.
 */
static SbgErrorCode getAndPrintProductInfo(SbgEComHandle *pECom)
{
    SbgErrorCode                    errorCode;
    SbgEComDeviceInfo               deviceInfo;

    assert(pECom);

    //
    // Get device information
    //
    errorCode = sbgEComCmdGetInfo(pECom, &deviceInfo);

    //
    // Display device information if no error
    //
    if (errorCode == SBG_NO_ERROR)
    {
        char    calibVersionStr[32];
        char    hwRevisionStr[32];
        char    fmwVersionStr[32];      

        sbgVersionToStringEncoded(deviceInfo.calibationRev, calibVersionStr, sizeof(calibVersionStr));
        sbgVersionToStringEncoded(deviceInfo.hardwareRev, hwRevisionStr, sizeof(hwRevisionStr));
        sbgVersionToStringEncoded(deviceInfo.firmwareRev, fmwVersionStr, sizeof(fmwVersionStr));

        printf("      Serial Number: %09"PRIu32"\n",    deviceInfo.serialNumber);
        printf("       Product Code: %s\n",             deviceInfo.productCode);
        printf("  Hardware Revision: %s\n",             hwRevisionStr);
        printf("   Firmware Version: %s\n",             fmwVersionStr);
        printf("     Calib. Version: %s\n",             calibVersionStr);
        printf("\n");
    }
    else
    {
        SBG_LOG_WARNING(errorCode, "Unable to retrieve device information");
    }

    return errorCode;
}

/*!
 * Execute the ellipseMinimal example given an opened and valid interface.
 * 
 * \param[in]   pInterface                          Interface used to communicate with the device.
 * \return                                          SBG_NO_ERROR if successful.
 */
static SbgErrorCode ellipseMinimalProcess(SbgInterface *pInterface)
{
    SbgErrorCode            errorCode = SBG_NO_ERROR;
    SbgEComHandle           comHandle;
    EllipseLegacyContext    context;
        
    assert(pInterface);

    memset(&context, 0, sizeof(context));

    //
    // Create the sbgECom library and associate it with the created interfaces
    //
    errorCode = sbgEComInit(&comHandle, pInterface);

    //
    // Test that the sbgECom has been initialized
    //
    if (errorCode == SBG_NO_ERROR)
    {
        //
        // Welcome message
        //
        printf("Welcome to the ELLIPSE minimal example.\n");
        printf("sbgECom version %s\n\n", SBG_E_COM_VERSION_STR);

        //
        // Query and display produce info, don't stop if there is an error
        //
        getAndPrintProductInfo(&comHandle);

        //
        // Showcase how to configure some output logs to 25 Hz, don't stop if there is an error
        //
        errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, SBG_ECOM_OUTPUT_MODE_DIV_8);

        if (errorCode != SBG_NO_ERROR)
        {
            SBG_LOG_WARNING(errorCode, "Unable to configure SBG_ECOM_LOG_IMU_DATA log");
        }

        errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG, SBG_ECOM_OUTPUT_MODE_DIV_8);

        if (errorCode != SBG_NO_ERROR)
        {
            SBG_LOG_WARNING(errorCode, "Unable to configure SBG_ECOM_LOG_MAG log");
        }

        errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_EULER, SBG_ECOM_OUTPUT_MODE_DIV_8);

        if (errorCode != SBG_NO_ERROR)
        {
            SBG_LOG_WARNING(errorCode, "Unable to configure SBG_ECOM_LOG_EKF_EULER log");
        }

        errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT, SBG_ECOM_OUTPUT_MODE_DIV_8);

        if (errorCode != SBG_NO_ERROR)
        {
            SBG_LOG_WARNING(errorCode, "Unable to configure SBG_ECOM_LOG_EKF_QUAT log");
        }

        printOutputConfiguration(&comHandle, SBG_ECOM_LOG_IMU_DATA, "IMU_DATA");
        printOutputConfiguration(&comHandle, SBG_ECOM_LOG_MAG, "MAG");
        printOutputConfiguration(&comHandle, SBG_ECOM_LOG_EKF_EULER, "EKF_EULER");
        printOutputConfiguration(&comHandle, SBG_ECOM_LOG_EKF_QUAT, "EKF_QUAT");

        //
        // Define callbacks for received data and display header
        //
        sbgEComSetReceiveLogCallback(&comHandle, onLogReceived, &context);
        printf("Streaming IMU, MAG, EKF_EULER and EKF_QUAT logs.\n");
        printf("If IMU or MAG appears but EULER/QUAT does not, the link is alive and the EKF/output status on the sensor is the problem.\n\n");

        //
        // Loop until the user exist
        //
        while (1)
        {
            //
            // Try to read a frame
            //
            errorCode = sbgEComHandle(&comHandle);

            //
            // Test if we have to release some CPU (no frame received)
            //
            if (errorCode == SBG_NOT_READY)
            {
                //
                // Release CPU
                //
                sbgSleep(1);
            }
            else if (errorCode != SBG_NO_ERROR)
            {
                SBG_LOG_ERROR(errorCode, "Unable to process incoming sbgECom logs");
            }
        }

        //
        // Close the sbgECom library
        //
        sbgEComClose(&comHandle);
    }
    else
    {
        SBG_LOG_ERROR(errorCode, "Unable to initialize the sbgECom library");
    }

    return errorCode;
}

//----------------------------------------------------------------------//
//  Main program                                                        //
//----------------------------------------------------------------------//

/*!
 * Program entry point usage: ellipseMinimal COM1 921600
 * 
 * \param[in]   argc                    Number of input arguments.
 * \param[in]   argv                    Input arguments as an array of strings.
 * \return                              EXIT_SUCCESS if successful.
 */
int main(int argc, char** argv)
{
    SbgErrorCode        errorCode = SBG_NO_ERROR;
    SbgInterface        sbgInterface;
    int                 exitCode;

    SBG_UNUSED_PARAMETER(argc);
    SBG_UNUSED_PARAMETER(argv);

    if (argc == 3)
    {
        //
        // Create a serial interface to communicate with the PULSE
        //
        errorCode = sbgInterfaceSerialCreate(&sbgInterface, argv[1], atoi(argv[2]));

        if (errorCode == SBG_NO_ERROR)
        {
            errorCode = ellipseMinimalProcess(&sbgInterface);

            if (errorCode == SBG_NO_ERROR)
            {
                exitCode = EXIT_SUCCESS;
            }
            else
            {
                exitCode = EXIT_FAILURE;
            }

            sbgInterfaceDestroy(&sbgInterface);
        }
        else
        {
            SBG_LOG_ERROR(errorCode, "unable to open serial interface");
            exitCode = EXIT_FAILURE;
        }
    }
    else
    {
        printf("Invalid input arguments, usage: ellipseLegacy SERIAL_DEVICE SERIAL_BAUDRATE\n");
        exitCode = EXIT_FAILURE;
    }
    
    return exitCode;
}
