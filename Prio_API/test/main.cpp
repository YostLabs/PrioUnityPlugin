/********************************************//**
 * This file should not be included for compiling the Prio API.
 * This file is mainly a testing ground for various functionality of the API and Prio devices.
 *
 * Copyright 1998-2014, YEI Corporation
 * This Source Code Form is subject to the terms of the YEI 3-Space Open
 * Source License available online at:
 * http://www.yeitechnology.com/yei-3-space-open-source-license
 ***********************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory>
#include <windows.h>
#include <chrono>
#include <thread>
#include <vector>
#include "prio_api_export.h"
#include "prio_api.hpp"
#include "serial.h"
#include <fstream>
#include <iostream>

void prio_basestation_test()
{
    prio_device_id bs_device;
    prio_device_id hub_device;
    prio_ComPort port;
    port.port_name = new char[64];
    PRIO_ERROR error;

    printf("====Creating a Prio Device from Search====\n");
    prio_findPorts(PRIO_BS);

    if (prio_getPort(port.port_name, 0, &port.device_type) == PRIO_NO_ERROR)
    {
        error = prio_createBaseStation(port.port_name, &bs_device);

        if (error)
        {
            printf("Failed to create PrioVR on %s!\n", port.port_name);
            prio_deinitAPI();
            return;
        }

        error = prio_bs_getWirelessHub(bs_device, &hub_device);

        if (error)
        {
            printf("Failed to create PrioVR on %s!\n", port.port_name);
            prio_deinitAPI();
            return;
        }
    }
    else
    {
        printf("Failed to get the port!");
        prio_deinitAPI();
        return;
    }
    printf("====Created a BaseStation====\n");


    char firmwareVerison[13];
    error = prio_bs_getFirmwareVersion(bs_device, firmwareVerison);
    if (error != PRIO_NO_ERROR)
    {
        printf("ERROR with get firmware version: %s\n", prio_error_string[error]);
    }
    else
    {
        printf("Basestatiom firmware version %s\n", firmwareVerison);
    }

    char hardwareVerison[33];
    error = prio_bs_getHardwareVersion(bs_device, hardwareVerison);
    if (error != PRIO_NO_ERROR)
    {
        printf("ERROR with get firmware version: %s\n", prio_error_string[error]);
    }
    else
    {
        printf("Basestation hardware version %s\n", hardwareVerison);
    }

    error = prio_hub_getFirmwareVersion(hub_device, firmwareVerison);
    if (error != PRIO_NO_ERROR)
    {
        printf("ERROR with get firmware version.\n", prio_error_string[error]);
    }
    else
    {
        printf("Hub firmware version %s\n", firmwareVerison);
    }

    error = prio_hub_getHardwareVersion(hub_device, hardwareVerison);
    if (error != PRIO_NO_ERROR)
    {
        printf("ERROR with get firmware version.\n", prio_error_string[error]);
    }
    else
    {
        printf("Hub hardware version %s\n", hardwareVerison);
    }

    U32 serialNumber = 1000;
    error = prio_bs_getSerialNumberAtLogicalID(bs_device,240,&serialNumber);
    if (error != PRIO_NO_ERROR)
    {
        printf("ERROR with get serial ID: %s\n", prio_error_string[error]);
    }
    else
    {
        printf("Serial ID %d\n", serialNumber);
    }

    error = prio_bs_getFirmwareVersionAtLogicalID(bs_device,0,firmwareVerison);
    if (error != PRIO_NO_ERROR)
    {
        printf("ERROR with get firmware version: %s\n", prio_error_string[error]);
    }
    else
    {
        printf("Sensor firmware version %s\n", firmwareVerison);
    }

    error = prio_bs_getHardwareVersionAtLogicalID(bs_device,0,hardwareVerison);
    if (error != PRIO_NO_ERROR)
    {
        printf("ERROR with get firmware version: %s\n", prio_error_string[error]);
    }
    else
    {
        printf("Sensor hardware version %s\n", hardwareVerison);
    }

    printf("================================\n");

    prio_deinitAPI();
    printf("\nFinished press Enter to continue");
    getchar();
    return;
}

void prio_hub_test()
{
    prio_device_id bs_device;
    prio_device_id hub_device;
    prio_ComPort port;
    port.port_name = new char[64];
    PRIO_ERROR error;

    printf("====Creating a Prio Device from Search====\n");
    prio_findPorts(PRIO_BS);

    if (prio_getPort(port.port_name, 0, &port.device_type) == PRIO_NO_ERROR)
    {
        error = prio_createBaseStation(port.port_name, &bs_device);

        if (error)
        {
            printf("Failed to create PrioVR on %s!\n", port.port_name);
            return;
        }

        error = prio_bs_getWirelessHub(bs_device, &hub_device);

        if (error)
        {
            printf("Failed to create PrioVR on %s!\n", port.port_name);
            return;
        }
    }
    else
    {
        printf("Failed to get the port!\n");
        return;
    }
    printf("====Created a BaseStation====\n");

    /* U32 serialNumber = 255;
    error = prio_hub_getSerialNumberAtLogicalID(hub_device, 0, &serialNumber);
    if (error != PRIO_NO_ERROR)
    {
        printf("ERROR with Hub Button: %s\n", prio_error_string[error]);
    }
    else
    {
        printf("Got Serial Number of %d\n", serialNumber);
    }
    char firmwareVerison[13];
    error = prio_hub_getFirmwareVersionAtLogicalID(hub_device, 0, firmwareVerison);
    if (error != PRIO_NO_ERROR)
    {
        printf("ERROR with get firmware version: %s\n", prio_error_string[error]);
    }
    else
    {
        printf("Sensor firmware version %s\n", firmwareVerison);
    }
    char hardwareVerison[33];
    error = prio_hub_getHardwareVersionAtLogicalID(hub_device, 0, hardwareVerison);
    if (error != PRIO_NO_ERROR)
    {
        printf("ERROR with get firmware version: %s\n", prio_error_string[error]);
    }
    else
    {
        printf("Sensor hardware version %s\n", hardwareVerison);
    }*/

	/*float scale[9] = { 1,2,3,4,5,6,7,8,9 };
	float bias[3] = { 1,2,3 };

	error = prio_hub_setMagnetometerCalibParams(hub_device, 0, scale, bias);
	error = prio_hub_setMagnetometerCalibParams(hub_device, 1, scale, bias);
	error = prio_hub_setMagnetometerCalibParams(hub_device, 7, scale, bias);
	error = prio_hub_setMagnetometerCalibParams(hub_device, 8, scale, bias);
	error = prio_hub_setMagnetometerCalibParams(hub_device, 9, scale, bias);
	error = prio_hub_setMagnetometerCalibParams(hub_device, 10, scale, bias);
	error = prio_hub_setMagnetometerCalibParams(hub_device, 14, scale, bias);
	error = prio_hub_setMagnetometerCalibParams(hub_device, 15, scale, bias);
	error = prio_hub_setMagnetometerCalibParams(hub_device, 16, scale, bias);
	error = prio_hub_setMagnetometerCalibParams(hub_device, 17, scale, bias);
	error = prio_hub_setMagnetometerCalibParams(hub_device, 21, scale, bias);

	if (error)
	{
		printf("Error on set Magnetometer  %s!\n", prio_error_string[error]);
	}*/
	float scale[9] = { 1,2,3,4,5,6,7,8,9 };
	float bias[3] = { 1,2,3 };

	error = prio_hub_getMagnetometerCalibParams(hub_device, 0, scale, bias);
	if (error)
	{
		printf("Error on get Magnetometer  %s!\n", prio_error_string[error]);
	}
	else
	{
		cout << scale[0] << scale[1] << scale[2] << scale[3] << scale[4] << scale[5] << scale[6] << scale[7] << scale[8] << bias[0] << bias[1] << bias[2] << endl;
	}

    prio_deinitAPI();
    printf("\nFinished press Enter to continue");
    getchar();
    return;
}

void prio_record_test()
{
    prio_device_id bs_device;
    prio_device_id hub_device;
    prio_ComPort port;
	port.port_name = new char[64];
    PRIO_ERROR error;

    printf("====Creating a Prio Device from Search====\n");
    prio_findPorts(PRIO_BS);

    if (prio_getPort(port.port_name, 0, &port.device_type) == PRIO_NO_ERROR)
    {
        error = prio_createBaseStation(port.port_name, &bs_device);

        if (error)
        {
            printf("Failed to create PrioVR on %s!\n", port.port_name);
            return;
        }

        error = prio_bs_getWirelessHub(bs_device, &hub_device);

        if (error)
        {
            printf("Failed to create PrioVR on %s!\n", port.port_name);
            return;
        }
    }
    else
    {
        printf("Failed to get the port!\n");
        return;
    }
    printf("====Created a BaseStation====\n");

    prio_hub_setupRecordingOptions(hub_device, 10, TRUE);

    bool wrap_mode = 0;
    prio_hub_getRecordingWrappingMode(hub_device, &wrap_mode);
    if (error != PRIO_NO_ERROR)
    {
        printf("Error with get RecordedSamples Length: %s\n", prio_error_string[error]);
        getchar();
        return;
    }
    else
    {
        printf("Got Wrapping mode of: %d\n", wrap_mode);
    }


    prio_bs_startRecording(bs_device, PRIO_STREAM_UNTARED_ORIENTATION_AS_QUATERNION, 0, PRIO_STREAM_DURATION_INFINITE);

    Sleep(10000);

    prio_bs_stopRecording(bs_device);

    printf("====Printing Results====\n");

    PrioHeader headerData[5000];
    U8 packetData[32*5000];
    U32 packet_count = 0; 

    error = prio_hub_getLengthOfRecordedSamples(hub_device, &packet_count);

    if (error != PRIO_NO_ERROR)
    {
        printf("Error with get RecordedSamples Length: %s\n", prio_error_string[error]);
        getchar();
        return;
    }
    else
    {
        printf("Got Length of: %d\n", packet_count);
    }

    /*if (error != PRIO_NO_ERROR || packet_count > 10)
    {
        packet_count = 10;
    }*/

    error = prio_hub_getRecordedSamples(hub_device, headerData, packetData, packet_count);

    if (error != PRIO_NO_ERROR)
    {
        printf("Error with get RecordedSamples: %s", prio_error_string[error]);
        getchar();
        return;
    }

    ofstream recordedData;
    recordedData.open("RecordedData.txt");

    for (int i = 0; i < (int)packet_count; i++)
    {
        PrioHeader tmp = headerData[i];
        recordedData << "====Header Data at Index: " << i << "====" << endl;
        recordedData << "TimeStamp: " << (int) tmp.SensorTimestamp << endl;
        recordedData << "Battery Level: " << (int)tmp.BatteryLevel << endl;
        recordedData << "Hub Button: " << (int)tmp.HubButton << endl;
        recordedData << "Joystick 1 X axis: " << (int)tmp.Joystick1.x_Axis << endl;
        recordedData << "Joystick 1 y axis: " << (int)tmp.Joystick1.y_Axis << endl;
        recordedData << "Joystick 1 Trigger: " << (int)tmp.Joystick1.Trigger << endl;
        recordedData << "Joystick 1 Button State: " << (int)tmp.Joystick1.ButtonState << endl;
        recordedData << "Joystick 2 X axis: " << (int)tmp.Joystick2.x_Axis << endl;
        recordedData << "Joystick 2 y axis: " << (int)tmp.Joystick2.y_Axis << endl;
        recordedData << "Joystick 2 Trigger: " << (int)tmp.Joystick2.Trigger << endl;
        recordedData << "Joystick 2 Button State: " << (int)tmp.Joystick2.ButtonState << endl;
        recordedData << "========================================" << endl;
        
        recordedData << "====Raw Sensor Data====" << endl;
        for (int j = 0; j < 2; j++)
        {
            float tmp[4];
            memcpy(tmp, packetData + (i * 32) + j*16, 16);
            recordedData << tmp[0] << ", " << tmp[1] << ", " << tmp[2] << ", " << tmp[3] << endl;
        }
        recordedData << "====End of Raw Sensor Data===" << endl;
    }
    recordedData.close();

    //Testing get packet at index
    PrioHeader headerDataIndex;
    U8 packetDataIndex[32];

    error = prio_hub_getRecordedSampleAtIndex(hub_device, &headerDataIndex, packetDataIndex, 3);

    if (error != PRIO_NO_ERROR)
    {
        printf("Error with get RecordedSamplesAtIndex: %s", prio_error_string[error]);
        getchar();
        return;
    }
    
    recordedData.open("RecordedDataAtInd.txt");

    recordedData << "====Header Data====" << endl;
    recordedData << "TimeStamp: " << (int)headerDataIndex.SensorTimestamp << endl;
    recordedData << "Battery Level: " << (int)headerDataIndex.BatteryLevel << endl;
    recordedData << "Hub Button: " << (int)headerDataIndex.HubButton << endl;
    recordedData << "Joystick 1 X axis: " << (int)headerDataIndex.Joystick1.x_Axis << endl;
    recordedData << "Joystick 1 y axis: " << (int)headerDataIndex.Joystick1.y_Axis << endl;
    recordedData << "Joystick 1 Trigger: " << (int)headerDataIndex.Joystick1.Trigger << endl;
    recordedData << "Joystick 1 Button State: " << (int)headerDataIndex.Joystick1.ButtonState << endl;
    recordedData << "Joystick 2 X axis: " << (int)headerDataIndex.Joystick2.x_Axis << endl;
    recordedData << "Joystick 2 y axis: " << (int)headerDataIndex.Joystick2.y_Axis << endl;
    recordedData << "Joystick 2 Trigger: " << (int)headerDataIndex.Joystick2.Trigger << endl;
    recordedData << "Joystick 2 Button State: " << (int)headerDataIndex.Joystick2.ButtonState << endl;
    recordedData << "========================================" << endl;

    recordedData << "====Raw Sensor Data====" << endl;
    for (int j = 0; j < 2; j++)
    {
        float tmp[4];
        memcpy(tmp, packetData + j*16, 16);
        recordedData << tmp[0] << ", " << tmp[1] << ", " << tmp[2] << ", " << tmp[3] << endl;
    }
    recordedData << "====End of Raw Sensor Data===" << endl;

    recordedData.close();

    prio_deinitAPI();
    return;
}

void prio_stream_test()
{
    prio_device_id bs_device;
    prio_device_id hub_device;
    prio_ComPort port;
    port.port_name = new char[64];
    PRIO_ERROR error;

    printf("====Creating a Prio Device from Search====\n");
    prio_findPorts(PRIO_BS);

    if (prio_getPort(port.port_name, 0, &port.device_type) == PRIO_NO_ERROR)
    {
        error = prio_createBaseStation(port.port_name, &bs_device);

        if (error)
        {
            printf("Failed to create PrioVR on %s!\n", port.port_name);
            prio_deinitAPI();
            return;
        }

        error = prio_bs_getWirelessHub(bs_device, &hub_device);

        if (error)
        {
            printf("Failed to create PrioVR on %s!\n", port.port_name);
            prio_deinitAPI();
            return;
        }
    }
    else
    {
        printf("Failed to get the port!");
        prio_deinitAPI();
        return;
    }
    printf("====Created a BaseStation====\n");
    

    error = prio_bs_startStreaming(bs_device, PRIO_STREAM_UNTARED_ORIENTATION_AS_QUATERNION, 0, PRIO_STREAM_DURATION_INFINITE);

    if (error != PRIO_NO_ERROR)
    {
        printf("ERROR with get start streaming: %s\n", prio_error_string[error]);
        prio_deinitAPI();
        return;
    }
    else
    {
        printf("Started Streaming!\n");
    }

    for (int i = 0; i < 20; i++)
    {
        printf("====================\n");
        float updateRate = 0;
        prio_hub_getStreamUpdateRate(hub_device, &updateRate);
        printf("Average update rate: %f", updateRate);
        printf("====================\n");

        prio_StreamHeaderData header_data;
        U8 packet_data[64];
        float packet_quat[16];

        error = prio_hub_getLastStreamingPacket(hub_device, &header_data, &packet_data[0]);
        if (error == PRIO_NO_ERROR)
        {
            memcpy(packet_quat, packet_data, 64);

            printf("====Header Data====\n");
            printf("Battery Level: %d\n", header_data.battery_level);
            printf("Button State: %d\n", header_data.hub_button);

            printf("Left Joystick x-axis: %d\n", header_data.joystick1.x_Axis);
            printf("Left Joystick y-axis: %d\n", header_data.joystick1.y_Axis);
            printf("Left Joystick trigger: %d\n", header_data.joystick1.Trigger);
            printf("Left Joystick button state: %d\n", header_data.joystick1.ButtonState);

            printf("Right Joystick x-axis: %d\n", header_data.joystick2.x_Axis);
            printf("Right Joystick y-axis: %d\n", header_data.joystick2.y_Axis);
            printf("Right Joystick trigger: %d\n", header_data.joystick2.Trigger);
            printf("Right Joystick button state: %d\n", header_data.joystick2.ButtonState);

            printf("====Sensor Data====\n");
            printf("(%.3f,%.3f,%.3f,%.3f)\n", packet_quat[0], packet_quat[1], packet_quat[2], packet_quat[3]);
            printf("(%.3f,%.3f,%.3f,%.3f)\n", packet_quat[4], packet_quat[5], packet_quat[6], packet_quat[7]);
            printf("(%.3f,%.3f,%.3f,%.3f)\n", packet_quat[8], packet_quat[9], packet_quat[10], packet_quat[11]);
            printf("(%.3f,%.3f,%.3f,%.3f)\n", packet_quat[12], packet_quat[13], packet_quat[14], packet_quat[15]);
            printf("====================\n");
        }
        else
        {
            printf("ERROR with get last streaming packet: %s\n", prio_error_string[error]);
        }
        getchar();
    }

    error = prio_bs_stopStreaming(bs_device);

    prio_deinitAPI();
    return;
}

void prio_hub_stream_wired()
{
    // An unique identifier used by the PrioVR API to identify the PrioVR Hub
    prio_device_id hub_device;
    // The communication port of a PrioVR Hub
    prio_ComPort port;
    // Allocate an char array of 64 chars for the port name
    port.port_name = new char[64];
    // An index into the array of communication ports found by the Prio API
    uint8_t com_offset = 0;

    // Find all of the communication ports that correspond to PrioVR Hub
    prio_findPorts(PRIO_HUB);
    // Get the communication port at the index of com_offset
    PRIO_ERROR error = prio_getPort(port.port_name, com_offset, &port.device_type);
    // Create the Hub
    error = prio_createHub(port.port_name, &hub_device);
    if (error)
    {
        printf("Failed to create PrioVR on %s! %s\n", port.port_name, prio_error_string[error]);
        return;
    }

    U8 _active_sensors[5];
    _active_sensors[0] = 255;
    _active_sensors[1] = 255;
    _active_sensors[2] = 255;
    _active_sensors[3] = 255;
    _active_sensors[4] = 255;

    U8 actual_length = 0;
    prio_hub_getActiveSensors(hub_device, _active_sensors, 10, &actual_length);

	float scale[9] = { 3,4,3,3,3,3,3,3,3 };
	float bias[3] = { 100,100,100 };

	prio_hub_setMagnetometerCalibParams(hub_device, 7, scale, bias);
	prio_hub_commitSensorSettings(hub_device, 7);


    // Start streaming data from the Prio suit
    error = prio_hub_startStreamingWired(hub_device, PRIO_STREAM_UNTARED_ORIENTATION_AS_QUATERNION, 0, PRIO_STREAM_DURATION_INFINITE);
    if (error)
    {
        printf("Failed to create PrioVR on %s! %s\n", port.port_name, prio_error_string[error]);
        return;
    }

    Sleep(100);

    prio_StreamHeaderData header_data;
    U8 packet_data[64] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    float packet_quat[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    for (int i = 0; i < 10; i++)
    {
        error = prio_hub_getLastStreamingPacket(hub_device, &header_data, &packet_data[0]);
        if (error == PRIO_NO_ERROR)
        {
            memcpy(packet_quat, packet_data, 64);

            printf("====Header Data====\n");
            printf("Battery Level: %d\n", header_data.battery_level);
            printf("Battery Status: %d\n", header_data.battery_status);
            printf("Button State: %d\n", header_data.hub_button);

            printf("Left Joystick x-axis: %d\n", header_data.joystick1.x_Axis);
            printf("Left Joystick y-axis: %d\n", header_data.joystick1.y_Axis);
            printf("Left Joystick trigger: %d\n", header_data.joystick1.Trigger);
            printf("Left Joystick button state: %d\n", header_data.joystick1.ButtonState);

            printf("Right Joystick x-axis: %d\n", header_data.joystick2.x_Axis);
            printf("Right Joystick y-axis: %d\n", header_data.joystick2.y_Axis);
            printf("Right Joystick trigger: %d\n", header_data.joystick2.Trigger);
            printf("Right Joystick button state: %d\n", header_data.joystick2.ButtonState);

            printf("====Sensor Data====\n");
            printf("(%.3f,%.3f,%.3f,%.3f)\n", packet_quat[0], packet_quat[1], packet_quat[2], packet_quat[3]);
            printf("(%.3f,%.3f,%.3f,%.3f)\n", packet_quat[4], packet_quat[5], packet_quat[6], packet_quat[7]);
            printf("(%.3f,%.3f,%.3f,%.3f)\n", packet_quat[8], packet_quat[9], packet_quat[10], packet_quat[11]);
            printf("(%.3f,%.3f,%.3f,%.3f)\n", packet_quat[12], packet_quat[13], packet_quat[14], packet_quat[15]);
            printf("====================\n");
            float updateRate = 0;
            prio_hub_getStreamUpdateRate(hub_device, &updateRate);
            printf("Average update rate: %f", updateRate);
            printf("====================\n");
        }
        else
        {
            printf("ERROR with get last streaming packet: %s\n", prio_error_string[error]);
        }
        getchar();
    }

    // Stops the streaming of the PrioVR Hub
    error = prio_hub_stopStreamingWired(hub_device);
    //printf("Stop Streaming Error: %s\n", prio_error_string[error]);
    float packet_quats[4] = { 0, 0, 0, 0 };
    for (int i = 0; i < 3; i++) // For as long as you need suit data for
    {
        error = prio_hub_getUntaredOrientationAsQuaternion(hub_device, 0, 4, &packet_quats[0]);
        if (error == PRIO_NO_ERROR)
        {
            printf("(%f,%f,%f,%f)\n", packet_quats[0], packet_quats[1], packet_quats[2], packet_quats[3]);
            getchar();
        }
    }

	U8 joystick_1_data[4] = { 128, 128, 0, 0 };
	U8 joystick_2_data[4] = { 128, 128, 0, 0 };
	U8 joystick_1_id = 255;
	U8 joystick_2_id = 255;

	U8 hub_button_state = 0;

	error = prio_hub_getAllControllerData(hub_device, &joystick_1_id, &joystick_2_id, joystick_1_data, joystick_2_data, &hub_button_state);

    // Deinitialize the Prio API
    prio_deinitAPI();
}

void prio_auto_pair_hub()
{
	prio_device_id bs_device;
	prio_device_id hub_device;
	prio_ComPort port;
	port.port_name = new char[64];
	PRIO_ERROR error;

	printf("====Creating a Prio Device from Search====\n");
	prio_findPorts(PRIO_BS);

	if (prio_getPort(port.port_name, 0, &port.device_type) == PRIO_NO_ERROR)
	{
		error = prio_createBaseStation(port.port_name, &bs_device);

		if (error)
		{
			printf("Failed to create PrioVR on %s!\n", port.port_name);
			prio_deinitAPI();
			return;
		}

		//error = prio_bs_getWirelessHub(bs_device, &hub_device);

		if (error)
		{
			printf("Failed to create PrioVR on %s!\n", port.port_name);
			prio_deinitAPI();
			return;
		}
	}
	else
	{
		printf("Failed to get the port!");
		prio_deinitAPI();
		return;
	}
	printf("====Created a BaseStation====\n");


	char firmwareVerison[13];
	error = prio_bs_getFirmwareVersion(bs_device, firmwareVerison);
	if (error != PRIO_NO_ERROR)
	{
		printf("ERROR with get firmware version: %s\n", prio_error_string[error]);
	}
	else
	{
		printf("Basestatiom firmware version %s\n", firmwareVerison);
	}

	char hardwareVerison[33];
	error = prio_bs_getHardwareVersion(bs_device, hardwareVerison);
	if (error != PRIO_NO_ERROR)
	{
		printf("ERROR with get firmware version: %s\n", prio_error_string[error]);
	}
	else
	{
		printf("Basestation hardware version %s\n", hardwareVerison);
	}

	/*error = prio_hub_getFirmwareVersion(hub_device, firmwareVerison);
	if (error != PRIO_NO_ERROR)
	{
		printf("ERROR with get firmware version.\n", prio_error_string[error]);
	}
	else
	{
		printf("Hub firmware version %s\n", firmwareVerison);
	}

	error = prio_hub_getHardwareVersion(hub_device, hardwareVerison);
	if (error != PRIO_NO_ERROR)
	{
		printf("ERROR with get firmware version.\n", prio_error_string[error]);
	}
	else
	{
		printf("Hub hardware version %s\n", hardwareVerison);
	}*/

	U8 channel;
	error = prio_bs_getWirelessChannel(bs_device, &channel);
	if (error != PRIO_NO_ERROR)
	{
		printf("ERROR with get firmware version.\n", prio_error_string[error]);
	}
	else
	{
		printf("BS channel: %d\n", channel);
	}

	U16 pan_id;
	error = prio_bs_getWirelessPanID(bs_device, &pan_id);
	if (error != PRIO_NO_ERROR)
	{
		printf("ERROR with get firmware version.\n", prio_error_string[error]);
	}
	else
	{
		printf("BS channel: %d\n", pan_id);
	}

	/*error = prio_hub_getWirelessChannel(hub_device, &channel);
	if (error != PRIO_NO_ERROR)
	{
		printf("ERROR with get firmware version.\n", prio_error_string[error]);
	}
	else
	{
		printf("Hub channel: %d\n", channel);
	}

	error = prio_hub_getWirelessPanID(hub_device, &pan_id);
	if (error != PRIO_NO_ERROR)
	{
		printf("ERROR with get firmware version.\n", prio_error_string[error]);
	}
	else
	{
		printf("Hub channel: %d\n", pan_id);
	}*/


	prio_bs_autoPairWithHub(bs_device);

	Sleep(100);

	error = prio_bs_getFirmwareVersion(bs_device, firmwareVerison);
	if (error != PRIO_NO_ERROR)
	{
		printf("ERROR with get firmware version: %s\n", prio_error_string[error]);
	}
	else
	{
		printf("Basestatiom firmware version %s\n", firmwareVerison);
	}

	error = prio_bs_getHardwareVersion(bs_device, hardwareVerison);
	if (error != PRIO_NO_ERROR)
	{
		printf("ERROR with get firmware version: %s\n", prio_error_string[error]);
	}
	else
	{
		printf("Basestation hardware version %s\n", hardwareVerison);
	}

	error = prio_bs_getWirelessHub(bs_device, &hub_device);

	if (error)
	{
		printf("Failed to create PrioVR on %s!\n", port.port_name);
		prio_deinitAPI();
		return;
	}

	error = prio_hub_getFirmwareVersion(hub_device, firmwareVerison);
	if (error != PRIO_NO_ERROR)
	{
		printf("ERROR with get firmware version.\n", prio_error_string[error]);
	}
	else
	{
		printf("Hub firmware version %s\n", firmwareVerison);
	}

	error = prio_hub_getHardwareVersion(hub_device, hardwareVerison);
	if (error != PRIO_NO_ERROR)
	{
		printf("ERROR with get firmware version.\n", prio_error_string[error]);
	}
	else
	{
		printf("Hub hardware version %s\n", hardwareVerison);
	}

	error = prio_bs_getWirelessChannel(hub_device, &channel);
	if (error != PRIO_NO_ERROR)
	{
		printf("ERROR with get firmware version.\n", prio_error_string[error]);
	}
	else
	{
		printf("BS channel: %d\n", channel);
	}

	error = prio_bs_getWirelessPanID(hub_device, &pan_id);
	if (error != PRIO_NO_ERROR)
	{
		printf("ERROR with get firmware version.\n", prio_error_string[error]);
	}
	else
	{
		printf("BS channel: %d\n", pan_id);
	}

	error = prio_hub_getWirelessChannel(hub_device, &channel);
	if (error != PRIO_NO_ERROR)
	{
		printf("ERROR with get firmware version.\n", prio_error_string[error]);
	}
	else
	{
		printf("Hub channel: %d\n", channel);
	}

	error = prio_hub_getWirelessPanID(hub_device, &pan_id);
	if (error != PRIO_NO_ERROR)
	{
		printf("ERROR with get firmware version.\n", prio_error_string[error]);
	}
	else
	{
		printf("Hub channel: %d\n", pan_id);
	}

	printf("================================\n");

	prio_deinitAPI();
	printf("\nFinished press Enter to continue");
	getchar();
	return;
}

int main()
{
    //prio_basestation_test();
    //prio_hub_test();
    prio_record_test();
    //prio_stream_test();
    //prio_hub_stream_wired();
	//prio_auto_pair_hub();
    system("pause");
    return 0;
}