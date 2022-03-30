#include "yost_connection_processor.hpp"
#include "threespace_api_export.h"

namespace yost
{    
	struct TssComPort
	{
		std::string port_name;
		U16 device_type;
	};

	struct Sensor
	{
		tss_device_id id;
		U32 serial_number;
		U16 sensor_type;
		std::string port_name;
	};

	struct Dongle
	{
		tss_device_id id;			
		std::string port_name;
	};

	// Class for 3-Space device connections
    class TssConnection : public YostConnection
    {
    public:
        TssConnection();
        ~TssConnection();

        void runProcess();

        // Device functions
        void reconnectDevice();
        void disconnectDevice();

        void startStreaming();
        void stopStreaming();

        void clearRecordedSamples();

        // Skeleton calls
        bool calibrate(float wait_time = 0.0f);
        void setupSkeletonMap();
        U32 removeUnusedSensors();

    private:
		TSS_ERROR result = TSS_NO_ERROR;
        uint8_t _active_sensors_len;

        vector<Sensor> _active_sensors;
        vector<Dongle> _active_dongles;
    };
};

