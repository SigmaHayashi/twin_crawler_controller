#ifndef NIDEC_MOTOR_H
#define NIDEC_MOTOR_H

#include <string>
#include <stdint.h>

class NidecMotor{
    public:
    NidecMotor(int fd, int id_pc, int id_motor);

    struct MotorResponse{
        uint8_t *raw_data;
        int command;
        bool result;
        std::string result_message;
        uint8_t ack;
        std::string ack_message;
        long data;
    };

    enum ControlMode{
        Release = 0x00,
        Position = 0x01,
        Speed = 0x04,
        Torque = 0x10
    };

    bool update();
    struct MotorResponse readResponse();

    enum Command{
        run_,
        stop_,
        emmergencyStop_,
        breakCommand_,
        servoOn_,
        servoOff_,
        getErrorInfo_,
        resetError_,
        checkConnection_,

        readDeviceID_,
        readControlMode_,
        writeControlMode_,
        offsetEncoder_,

        rollBySpeed_,
        readSpeed_
    };

    void run();
    void stop();
    void emmergencyStop();
    void breakCommand();
    void servoOn();
    void servoOff();
    void getErrorInfo();
    void resetError();
    void checkConnection();

    void readDeviceID();
    void readControlMode();
    void writeControlMode(ControlMode mode);
    void offsetEncoder();

    //速度制御
    void rollBySpeed(int rpm);
    void readSpeed();

    private:
    NidecMotor *motor;
    int fd, id_pc, id_motor;
    uint8_t read_buf[128], analyze_buf[128];
    int read_buf_index;

    int new_command;
    uint8_t new_operation_command;
    uint16_t new_attribute_command;

    struct AnalyzedData{
        uint8_t *raw_data;
        uint8_t send_from;
        uint8_t send_to;
        uint16_t data_length;
        uint8_t operation_command;
        uint16_t attribute_command;
        uint8_t *data;
        uint8_t check_sum;
    };
    AnalyzedData analyzed_data;

    uint8_t calcCheckSum(uint8_t *data, int check_sum_length);
    void writeData(int data_length, uint8_t operation_command, uint16_t attribute_command = 0, uint8_t *data = 0);
    void readData();
    void analyzeReadData(uint8_t *read_data, int length);

    void returnACK(uint8_t *data);
};

#endif
