#include "yost_tss_connection_processor.hpp"
#include "yost_skeleton_core_api.hpp"

namespace yost
{
    // TssConnection
    TssConnection::TssConnection()
    {
        _type = PROCESSOR_TYPE_TSS;

		vector<TssComPort> ports;
		tss_findSensorPorts(TSS_FIND_ALL_KNOWN);
		result = TSS_NO_ERROR;
		while (result == TSS_NO_ERROR)
		{
			char port_name[64];
			U16 sensor_type;
			U8 connection_type;
			result = tss_getNextSensorPort(port_name, &sensor_type, &connection_type);
			if (result == TSS_NO_ERROR)
			{
				TssComPort temp;
				temp.port_name = port_name;
				temp.device_type = sensor_type;
				ports.push_back(temp);
			}
		}

        for (auto device : ports)
        {
            if (device.device_type & TSS_DONGLE)
            {
				tss_device_id out_id;
				result = tss_createDongle(device.port_name.c_str(), &out_id);	
				if (result == TSS_NO_ERROR)
				{
					Dongle d;
					d.id = out_id;
					d.port_name = device.port_name;
					_active_dongles.push_back(d);
				}
				else
					continue;

                vector<tss_device_id> sensors;
                for (U8 i = 0; i < 15; i++)
                {
                    printf("Looking for Wireless Sensor: %d\n", i);
					tss_device_id sensor;
                    U32 sNum;
					U32 tStamp;
					
					Dongle dongle = _active_dongles.back();
					result = tss_dongle_getSerialNumberAtLogicalID(dongle.id, i, &sNum, &tStamp);

                    if (result != TSS_NO_ERROR || sNum == 0)
                    {
                        continue;
                    }

					result = tss_dongle_getWirelessSensor(dongle.id, i, &sensor);
                    if (result == TSS_NO_ERROR)
                    {
						Sensor s;
						s.id = sensor;
						s.sensor_type = TSS_WIRELESS;
						s.port_name = "Invalid";
						s.serial_number = sNum;
                        _active_sensors.push_back(s);
                    }
                }
            }
            else if (device.device_type & (TSS_FIND_ALL_KNOWN-TSS_BOOTLOADER))
            {
				U16 sensor_type;
				tss_device_id out_id;
				result = tss_createSensor(device.port_name.c_str(), &out_id);
				if (result == TSS_NO_ERROR)
				{
					if (tss_sensor_getSensorType(out_id, &sensor_type) == TSS_NO_ERROR)
					{
						U32 sNum = 0;
						U32 tStamp = 0;
						result = tss_sensor_getSerialNumber(out_id, &sNum, &tStamp);
						if (result == TSS_NO_ERROR)
						{
							Sensor s;
							s.id = out_id;
							s.port_name = device.port_name;
							s.serial_number = sNum;
							s.sensor_type = sensor_type;
							_active_sensors.push_back(s);
						}
					}
				}
            }
        }

        if (_active_sensors.size() <= 0)
        {
            _is_connected = false;
        }
        else
        {
            _is_connected = true;
        }
        if (!_is_connected)
        {
            disconnectDevice();
        }
    }

    TssConnection::~TssConnection()
    {
        disconnectDevice();
        if (_is_connected)
        {
            //prio_removeBaseStation(_device);
        }
    }

    void TssConnection::runProcess()
    {
        if (!_is_connected)
        {
            disconnectDevice();
            reconnectDevice();
        }
        if (!_is_connected || !_is_mapped || !_is_streaming)
        {
            return;
        }

        std::shared_ptr<Bone> bone;
        for (auto device : _active_sensors)
        {			
			if (tss_sensor_isConnected(device.id) == TSS_NO_ERROR)
			{	
				bone = _skeleton->getBone(_skeleton_id_map[device.serial_number]);
				TSS_Stream_Packet packet;
				result = tss_sensor_getLastStreamingPacket(device.id, &packet);
				if (result == TSS_NO_ERROR && bone != nullptr)
				{
					Orient new_orient, tmp_orient, finalOrient;
					std::map<std::string, Orient> pose = _skeleton->getSkeletonPose();

					std::vector<std::string> boneNames = _skeleton->getBoneNameList();

					//bone->setRawOrientation(packet.taredOrientQuat);
					//new_orient = packet.taredOrientQuat;
					Orient calibration_offset = bone->getCalibrationOffset();
					orientMul((Orient*)packet.taredOrientQuat, &calibration_offset, &tmp_orient);
					Orient calibration_tare = bone->getCalibrationTare();
					orientMul(&calibration_tare, &tmp_orient, &new_orient);
					//
					Orient poseOffset = pose[bone->getName()];
					orientInverse(&poseOffset);
					//
					orientMul(&new_orient, &poseOffset, &finalOrient);
					bone->setRawOrientation(finalOrient);
				}
			}
        }
        _skeleton->getRootBone()->update();
    }

    void TssConnection::reconnectDevice()
    {
        for (auto device : _active_sensors)
        {			
            if (tss_sensor_isConnected(device.id) != TSS_NO_ERROR)
            {
				if (device.port_name != "Invalid")
					tss_sensor_openPort(device.id, device.port_name.c_str());
            }
        }
        _is_connected = true;
    }

    void TssConnection::disconnectDevice()
    {
        stopStreaming();
        for (auto device : _active_dongles)
        {
			tss_dongle_closePort(device.id);            
        }
        
        for (auto device : _active_sensors)
        {
			U8 is_wireless = 0;
			result = tss_sensor_isWireless(device.id, &is_wireless);
            if (result != TSS_NO_ERROR || is_wireless != 1)
            {
				tss_sensor_closePort(device.id);                
            }
        }
        _is_connected = false;
        _is_mapped = false;
        _is_streaming = false;
        _is_recording = false;
        _stream_data.reset();
    }

    // This needs to be called before runProcess
    void TssConnection::startStreaming()
    {
        //memset(_stream_slots, 0xff, sizeof(_stream_slots));
        _stream_slots[0] = TSS_STREAM_TARED_ORIENTATION_AS_QUATERNION;
        _stream_data_size = 0;

        for (auto device : _active_dongles)
        {
			U32 data_flags = TSS_STREAM_TARED_ORIENTATION_AS_QUATERNION;
			// TODO: _stream_slots[0]
			result = tss_dongle_enableAllSensorsAndStartStreaming(device.id, data_flags, 1, TSS_STREAM_DURATION_INFINITE);            
        }

       /* for (auto device : _active_sensors)
        {
            if (!device->_isStreaming && !device->_isWireless)
            {
                result = device->startStreamingWired(_stream_slots[0], 1, TSS_STREAM_DURATION_INFINITE);
            }
        }*/
        if (result == TSS_NO_ERROR)
        {
            _is_streaming = true;
            //result;// = prio_getFullLengthOfStreamData(_device, &_stream_data_size);
            if (result == TSS_NO_ERROR)
            {
            }
        }
        yost::sleep_ms(1000);
    }

    void TssConnection::stopStreaming()
    {
        for (auto device : _active_dongles)
        {            
			result = tss_dongle_stopStreaming(device.id);
        }

        for (auto device : _active_sensors)
        {
			U8 is_wireless = 0;
			result = tss_sensor_isWireless(device.id, &is_wireless);
			if (result == TSS_NO_ERROR && is_wireless == 0)
			{				
				result = tss_sensor_stopStreamingWired(device.id);
			}
        }
    }

    // The calibration is calculated using the Hub sensor and applied to the other sensors
    // This needs to be called before runProcess
    bool TssConnection::calibrate(float wait_time)
    {
        _is_calibrated = false;

        if (!_is_connected)
        {
            reconnectDevice();
        }

        std::unique_ptr<float[]> accel3(new float[3]);
        std::unique_ptr<float[]> untared_quats(new float[_active_sensors_len * 4]);
        std::map<int32_t, Orient> avg_orients;
        result = TSS_ERROR_COMMAND_FAILURE;
        U8 i, j;

        setupSkeletonMap();

        if (!_is_mapped)
        {
            return _is_calibrated;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds((uint32_t)(wait_time * 1000)));

        Orient* tempQuat = new Orient();
        std::map<int32_t, Orient>::iterator avg_it;
        std::map<int32_t, Orient>::iterator end_avg_it = avg_orients.end();

        for (i = 0; i < 10; i++)
        {
            for (j = 0; j < _active_sensors.size(); j++)
            {
				U32 tStamp;                
				result = tss_sensor_getUntaredOrientationAsQuaternion(_active_sensors[j].id, (float*)tempQuat, &tStamp);

                if (result == TSS_NO_ERROR)
                {
                    if (avg_orients.find(_active_sensors[j].serial_number) == end_avg_it)
                    {
                        avg_orients[_active_sensors[j].serial_number].data[0] = tempQuat->data[0];
                        avg_orients[_active_sensors[j].serial_number].data[1] = tempQuat->data[1];
                        avg_orients[_active_sensors[j].serial_number].data[2] = tempQuat->data[2];
                        avg_orients[_active_sensors[j].serial_number].data[3] = tempQuat->data[3];
                    }
                    else
                    {
                        avg_orients[_active_sensors[j].serial_number].data[0] += tempQuat->data[0];
                        avg_orients[_active_sensors[j].serial_number].data[1] += tempQuat->data[1];
                        avg_orients[_active_sensors[j].serial_number].data[2] += tempQuat->data[2];
                        avg_orients[_active_sensors[j].serial_number].data[3] += tempQuat->data[3];
                    }
                }
            }
        }

        for (auto quat : avg_orients)
        {
            orientNormalize(&quat.second);
        }

        Orient anchor_placement_orient(0, 0.7071f, 0.7071f, 0);
        orientInverse(&anchor_placement_orient);
        orientNormalize(&anchor_placement_orient);

        // Check if the Hub sensor is in the list of orientations
        //U32 anchor_idx = _skeleton_id_order[0];
        U32 anchor_sensor_idx = _skeleton_id_order[0];

        if (avg_orients.find(anchor_sensor_idx) == avg_orients.end())
        {
            return _is_calibrated;
        }

        Orient anchor_sensor_orient = avg_orients[anchor_sensor_idx];

        // Need to offest the Hub sensor orientation to match the space of the API (Left-handed: X-right, Y-up, Z-forward)
        Orient anchor_reference_orient;
        orientMul(&anchor_sensor_orient, &anchor_placement_orient, &anchor_reference_orient);

        // Calculate the offset for the Z-axis of rotation
        Vector3 z_axis(0, 0, 1);
        orientRotate(&anchor_reference_orient, &z_axis);

        // Calculate the offset for the X-axis of rotation
        Vector3 x_axis;
        Vector3 vector_y = Vector3(0, 1, 0);
        vectorCross(&vector_y, &z_axis, &x_axis);

        Vector3 vector_x = Vector3(1, 0, 0);
        anchor_reference_orient = getOrientFromVectors(&vector_x, &x_axis);

        // Apply offest to all sensors
        std::map<std::string, Orient> pose = _skeleton->getSkeletonPose();
        std::map<std::string, Orient>::iterator pose_it;
        std::map<std::string, Orient>::iterator end_pose_it = pose.end();
        Orient base_pose_orient;
        Orient pose_orient;

        std::shared_ptr<Bone> bone;
        uint32_t bone_device_id;
        Orient curr_bone_orient;
        Orient curr_bone_offset;
        Orient curr_bone_tare;

        for (pose_it = pose.begin(); pose_it != end_pose_it; pose_it++)
        {
            bone = _skeleton->getBone((*pose_it).first);
            if (bone == nullptr)
            {
                return _is_calibrated;
            }
            base_pose_orient = (*pose_it).second;
            orientMul(&anchor_reference_orient, &base_pose_orient, &pose_orient);

            bone_device_id = bone->getDeviceId();

            if (avg_orients.find(bone_device_id) != end_avg_it)
            {
                curr_bone_orient = avg_orients[bone_device_id];
                orientInverse(&curr_bone_orient);

                orientMul(&curr_bone_orient, &pose_orient, &curr_bone_offset);
                orientNormalize(&curr_bone_offset);
                bone->setCalibrationOffset(curr_bone_offset);

                curr_bone_tare = anchor_reference_orient;
                orientInverse(&curr_bone_tare);
                orientNormalize(&curr_bone_tare);
                bone->setCalibrationTare(curr_bone_tare);
            }
        }

        // Update the skeleton before continuing on so positions will be valid
        bone = _skeleton->getRootBone();
        if (bone != nullptr)
        {
            bone->update();
        }

        _is_calibrated = true;
        return _is_calibrated;
    }

    // This needs to be called before runProcess
    void TssConnection::setupSkeletonMap()
    {
        resetSkeletonMap();

        std::string xml_data = _skeleton->utilityGetDeviceXmlMap();
        if (xml_data.size() == 0)
        {
            _is_mapped = false;
        }
        else
        {
            rapidxml::xml_document<> xml_doc;
            rapidxml::xml_node<>* root_node;
            rapidxml::xml_node<>* suit_node;

            // Parse the string using the rapidxml file parsing library into xml_doc
            xml_doc.parse<0>(&xml_data[0]);

            // Find our root node
            root_node = xml_doc.first_node();

            // Get our suit layout
            suit_node = root_node->first_node();

            // Interate over the sensor assignments
            U32 boneOrder = 0;
            _skeleton->getRootBone()->setAllChildrenUpdateType(YOST_SKELETON_BONE_UPDATE_FUSED);
            for (rapidxml::xml_node<>* sensor_node = suit_node->first_node(); sensor_node != nullptr; sensor_node = sensor_node->next_sibling())
            {
                uint32_t serial = std::stoul(sensor_node->first_attribute("SERIAL")->value(), nullptr, 16);
                std::string bone_name = sensor_node->first_attribute("BONE")->value();
                _skeleton_id_map[serial] = bone_name;
                _skeleton_id_order[boneOrder] = serial;
                std::shared_ptr<Bone> bone = _skeleton->getBone(bone_name);
                bone->setDeviceId(serial);
                bone->setUpdateType(YOST_SKELETON_BONE_UPDATE_DEVICE);
                //bone->setAllChildrenUpdateType(YOST_SKELETON_BONE_UPDATE_FUSED); Commented out for testing NiCK 8/17/2015

                boneOrder++;
            }
            removeUnusedSensors();

            _is_mapped = true;
        }
    }

    U32 TssConnection::removeUnusedSensors()
    {
        U32 numRemovedSensors = 0;
        bool isPaired = false;
        for (unsigned int i = 0; i < _active_sensors.size(); i++)
        {
            isPaired = _skeleton_id_map.find(_active_sensors[i].serial_number) != _skeleton_id_map.end();
            if (!isPaired)
            {
                _active_sensors.erase(_active_sensors.begin()+i);
                numRemovedSensors++;
            }
        }

        _is_connected = (_active_sensors.size() > 0) ? true : false;

        return numRemovedSensors;
    }
}
