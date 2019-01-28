#ifndef DXL_DRIVER_H
#define DXL_DRIVER_H

/*
    Base class for Dynamixel motor driver (dynamixel protocol 2.0 only)
*/

#include "dynamixel_sdk/dynamixel_sdk.h"
#include <vector>

#define DXL_LEN_ONE_BYTE   1
#define DXL_LEN_TWO_BYTES  2
#define DXL_LEN_FOUR_BYTES 4

#define GROUP_SYNC_REDONDANT_ID     10
#define GROUP_SYNC_READ_RX_FAIL     11
#define LEN_ID_DATA_NOT_SAME        20

#define PING_WRONG_MODEL_NUMBER     30

class DxlDriver {

    protected:
        dynamixel::PortHandler *portHandler;
        dynamixel::PacketHandler *packetHandler;

        int syncWrite1Byte  (uint8_t address, std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list);
        int syncWrite2Bytes (uint8_t address, std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list);
        int syncWrite4Bytes (uint8_t address, std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list);

        int read1Byte       (uint8_t address, uint8_t id, uint32_t *data);
        int read2Bytes      (uint8_t address, uint8_t id, uint32_t *data);
        int read4Bytes      (uint8_t address, uint8_t id, uint32_t *data);
        int syncRead        (uint8_t address, uint8_t data_len, std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list);

    public:
        DxlDriver(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler);
        
        int scan(std::vector<uint8_t> &id_list);
        int ping(uint8_t id);
        int getModelNumber(uint8_t id, uint16_t *dxl_model_number);
        int reboot(uint8_t id);

        /*
         * Virtual functions below - to override
         *
         * --> All functions return communication result
         */

        virtual int checkModelNumber(uint8_t id) = 0;

        // eeprom write
        virtual int changeId            (uint8_t id, uint8_t new_id) = 0;
        virtual int changeBaudRate      (uint8_t id, uint32_t new_baudrate) = 0;
        virtual int setReturnDelayTime  (uint8_t id, uint32_t return_delay_time) = 0;
        virtual int setLimitTemperature (uint8_t id, uint32_t temperature) = 0;
        virtual int setMaxTorque        (uint8_t id, uint32_t torque) = 0;
        virtual int setReturnLevel      (uint8_t id, uint32_t return_level) = 0;
        virtual int setAlarmShutdown    (uint8_t id, uint32_t alarm_shutdown) = 0;

        // eeprom read
        virtual int readReturnDelayTime  (uint8_t id, uint32_t *return_delay_time) = 0;
        virtual int readLimitTemperature (uint8_t id, uint32_t *limit_temperature) = 0;
        virtual int readMaxTorque        (uint8_t id, uint32_t *max_torque) = 0;
        virtual int readReturnLevel      (uint8_t id, uint32_t *return_level) = 0;
        virtual int readAlarmShutdown    (uint8_t id, uint32_t *alarm_shutdown) = 0;

        // ram write
        virtual int setTorqueEnable   (uint8_t id, uint32_t torque_enable) = 0;
        virtual int setLed            (uint8_t id, uint32_t led_value) = 0;
        virtual int setGoalPosition   (uint8_t id, uint32_t position) = 0;
        virtual int setGoalVelocity   (uint8_t id, uint32_t velocity) = 0;
        virtual int setGoalTorque     (uint8_t id, uint32_t torque) = 0;

        virtual int syncWriteLed          (std::vector<uint8_t> &id_list, std::vector<uint32_t> &led_list) = 0;
        virtual int syncWriteTorqueEnable (std::vector<uint8_t> &id_list, std::vector<uint32_t> &torque_enable_list) = 0; 
        virtual int syncWritePositionGoal (std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list) = 0;
        virtual int syncWriteVelocityGoal (std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list) = 0;
        virtual int syncWriteTorqueGoal   (std::vector<uint8_t> &id_list, std::vector<uint32_t> &torque_list) = 0;

        // ram read
        virtual int readPosition       (uint8_t id, uint32_t *present_position) = 0;
        virtual int readVelocity       (uint8_t id, uint32_t *present_velocity) = 0;
        virtual int readLoad           (uint8_t id, uint32_t *present_load) = 0;
        virtual int readTemperature    (uint8_t id, uint32_t *temperature) = 0;
        virtual int readVoltage        (uint8_t id, uint32_t *voltage) = 0;
        virtual int readHardwareStatus (uint8_t id, uint32_t *hardware_status) = 0;

        virtual int syncReadPosition       (std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list) = 0;
        virtual int syncReadVelocity       (std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list) = 0;
        virtual int syncReadLoad           (std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list) = 0;
        virtual int syncReadTemperature    (std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list) = 0;
        virtual int syncReadVoltage        (std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list) = 0;
        virtual int syncReadHwErrorStatus  (std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list) = 0;
};

#endif
