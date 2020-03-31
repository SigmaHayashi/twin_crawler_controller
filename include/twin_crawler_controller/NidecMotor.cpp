#include "NidecMotor.h"

#include <unistd.h>
#include <stdio.h>

NidecMotor::NidecMotor(int fd, int id_pc, int id_motor){
    this->fd = fd;
    this->id_pc = id_pc;
    this->id_motor = id_motor;
    this->read_buf_index = 0;
    motor = this;

    for(int i = 0; i < 128; i++){
        read_buf[i] = analyze_buf[i] = 0;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool NidecMotor::update(){
    readData();
    
    int start_idx, end_idx;
    bool start_buffering = false;
    bool end_buffering = false;

    /*
    printf("read_buf : ");
    for(int i = 0; i < 128; i++){
        printf("%02x", read_buf[i]);
    }
    printf("\n");
    */
    
    for(int i = 0; i < 128; i ++){
        if(!start_buffering && (read_buf[i] == 0x7e)){
            start_idx = i;
            start_buffering = true;
        }
        if(start_buffering && !end_buffering && (read_buf[i] == 0x7f)){
            end_idx = i;
            end_buffering = true;
            break;
        }
    }
    if(!start_buffering || !end_buffering){
        return false;
    }

    for(int i = 0; i < end_idx - start_idx + 1; i++){
        analyze_buf[i] = read_buf[start_idx + i];
    }
    analyzeReadData(analyze_buf, end_idx - start_idx + 1);
    
    for(int i = 0; i < 128; i ++){
        read_buf[i] = 0;
        read_buf_index = 0;
    }

    return true;
}

struct NidecMotor::MotorResponse NidecMotor::readResponse(){
    MotorResponse response;

    response.raw_data = analyzed_data.raw_data;

    // 自動で送信されたエラー情報を受け取り
    if((analyzed_data.operation_command & 0x3f) == 0x11){
        response.command = getErrorInfo_;
        response.command_str = "Error Information";
        response.data = 0;
        
        char nak_code[20];
        int nak_code_int = 0;
        for(int i = 0; i < analyzed_data.data_length - 5; i++){
            nak_code_int = nak_code_int << 8 | analyzed_data.data[i];
        }
        sprintf(nak_code, "0x%08x", nak_code_int);
        response.message = "Error Code : " + std::string(nak_code);

        return response;
    }

    // エラー情報読み込みコマンドの返答
    if((analyzed_data.operation_command & 0x3f) == 0x10){
        response.command = getErrorInfo_;
        response.command_str = "Error Information";
        
        response.data = 0;
        
        char nak_code[20];
        int nak_code_int = 0;
        for(int i = 0; i < analyzed_data.data_length - 5; i++){
            nak_code_int = nak_code_int << 8 | analyzed_data.data[i];
        }
        sprintf(nak_code, "0x%08x", nak_code_int);
        response.message = "Error Code : " + std::string(nak_code);

        return response;
    }

    // 通常のフロー
    // データをセット
    response.data = 0;
    for(int i = 0; i < analyzed_data.data_length - 5; i++){
        response.data = response.data << 8 | analyzed_data.data[i];
    }

    // コマンドをセット＆コマンドごとの対応
    if((analyzed_data.operation_command & 0x3f) == 0x03){
        response.command = run_;
        response.command_str = "run";
    }
    else if((analyzed_data.operation_command & 0x3f) == 0x08){
        response.command = stop_;
        response.command_str = "stop";
    }
    else if((analyzed_data.operation_command & 0x3f) == 0x09){
        response.command = emmergencyStop_;
        response.command_str = "emmergencyStop";
    }
    else if((analyzed_data.operation_command & 0x3f) == 0x0a){
        response.command = breakCommand_;
        response.command_str = "breakCommand";
    }
    else if((analyzed_data.operation_command & 0x3f) == 0x0e){
        response.command = servoOn_;
        response.command_str = "servoOn";
    }
    else if((analyzed_data.operation_command & 0x3f) == 0x0f){
        response.command = servoOff_;
        response.command_str = "servoOff";
    }
    else if((analyzed_data.operation_command & 0x3f) == 0x10){
        response.command = getErrorInfo_;
        response.command_str = "getErrorInfo";
    }
    else if((analyzed_data.operation_command & 0x3f) == 0x12){
        response.command = resetError_;
        response.command_str = "resetError";
    }
    else if((analyzed_data.operation_command & 0x3f) == 0x22){
        response.command = checkConnection_;
        response.command_str = "checkConnection";
    }
    else if((analyzed_data.operation_command & 0x3f) == 0x01 && analyzed_data.attribute_command == 0x0004){
        response.command = readDeviceID_;
        response.command_str = "readDeviceID";
    }
    else if((analyzed_data.operation_command & 0x3f) == 0x01 && analyzed_data.attribute_command == 0x0001){
        response.command = readControlMode_;
        response.command_str = "readControlMode";
    }
    else if((analyzed_data.operation_command & 0x3f) == 0x05 && analyzed_data.attribute_command == 0x0001){
        response.command = writeControlMode_;
        response.command_str = "writeControlMode";
    }
    else if((analyzed_data.operation_command & 0x3f) == 0x01 && analyzed_data.attribute_command == 0x0011){
        response.command = readPosition_;
        response.command_str = "readPosition";
        response.data = response.data >> 7;
    }
    else if((analyzed_data.operation_command & 0x3f) == 0x05 && analyzed_data.attribute_command == 0x001d){
        response.command = writePosition_;
        response.command_str = "writePosition";
        response.data = response.data >> 7;
    }
    else if((analyzed_data.operation_command & 0x3f) == 0x05 && analyzed_data.attribute_command == 0x001f){
        response.command = offsetEncoder_;
        response.command_str = "offsetEncoder";
    }
    else if((analyzed_data.operation_command & 0x3f) == 0x05 && analyzed_data.attribute_command == 0x0022){
        response.command = rollBySpeed_;
        response.command_str = "rollBySpeed";
        response.data = response.data >> 12;
    }
    else if((analyzed_data.operation_command & 0x3f) == 0x01 && analyzed_data.attribute_command == 0x0021){
        response.command = readSpeed_;
        response.command_str = "readSpeed";
        response.data = response.data >> 12;
    }
    else{
        response.command = NotDefined_;
        response.command_str = "Not Defined";
    }

    // エラーメッセージなどの対応
    bool success = true;
    std::string error_message = "";
    if((analyzed_data.operation_command & 0x3f) != motor->new_operation_command){
        success = false;
        error_message += "Returned Different Operation Command. ";
    }
    if(analyzed_data.data_length > 3 && analyzed_data.attribute_command != motor->new_attribute_command){
        success = false;
        error_message += "Returned Different Attribute Command. ";
    }
    if(analyzed_data.operation_command & 0xc0 != 0xc0 && analyzed_data.operation_command & 0xc0 != 0x80){
        success = false;
        error_message += "Returned NAK. ";
    }

   if(success){
       response.message = "Success.";
   }
   else{
       response.message = error_message;
   }

    return response;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void NidecMotor::run(){
   motor->new_command = run_;
   motor->new_operation_command = 0x03;
   writeData(3, motor->new_operation_command);
}

void NidecMotor::stop(){
   motor->new_command = stop_;
   motor->new_operation_command = 0x08;
   writeData(3, motor->new_operation_command);
}

void NidecMotor::emmergencyStop(){
   motor->new_command = emmergencyStop_;
   motor->new_operation_command = 0x09;
   writeData(3, motor->new_operation_command);
}

void NidecMotor::breakCommand(){
   motor->new_command = breakCommand_;
   motor->new_operation_command = 0x0a;
   writeData(3, motor->new_operation_command);
}

void NidecMotor::servoOn(){
   motor->new_command = servoOn_;
   motor->new_operation_command = 0x0e;
   writeData(3, motor->new_operation_command);
}

void NidecMotor::servoOff(){
   motor->new_command = servoOff_;
   motor->new_operation_command = 0x0f;
   writeData(3, motor->new_operation_command);
}

void NidecMotor::getErrorInfo(){
   motor->new_command = getErrorInfo_;
   motor->new_operation_command = 0x10;
   writeData(3, motor->new_operation_command);
}

void NidecMotor::resetError(){
   motor->new_command = resetError_;
   motor->new_operation_command = 0x12;
   writeData(3, motor->new_operation_command);
}

void NidecMotor::checkConnection(){
   motor->new_command = checkConnection_;
   motor->new_operation_command = 0x22;
   writeData(3, motor->new_operation_command);
}

void NidecMotor::readDeviceID(){
    motor->new_command = readDeviceID_;
    motor->new_operation_command = 0x01;
    motor->new_attribute_command = 0x0004;
    writeData(5, motor->new_operation_command, motor->new_attribute_command);
}

void NidecMotor::readControlMode(){
    motor->new_command = readControlMode_;
    motor->new_operation_command = 0x01;
    motor->new_attribute_command = 0x0001;
    writeData(5, motor->new_operation_command, motor->new_attribute_command);
}

void NidecMotor::writeControlMode(ControlMode mode){
    motor->new_command = writeControlMode_;
    motor->new_operation_command = 0x05;
    motor->new_attribute_command = 0x0001;
    uint8_t data;
    switch (mode){
        case Release:
        case Position:
        case Speed:
        case Torque:
        data = mode;
        break;
        
        default:
        printf("Control mode error : %d\n", mode);
        return;
    }
    writeData(5 + 1, motor->new_operation_command, motor->new_attribute_command, &data);
}

void NidecMotor::readPosition(){
    motor->new_command = readPosition_;
    motor->new_operation_command = 0x01;
    motor->new_attribute_command = 0x0011;
    writeData(5, motor->new_operation_command, motor->new_attribute_command);
}

void NidecMotor::writePosition(int pos){
    motor->new_command = writePosition_;
    motor->new_operation_command = 0x05;
    motor->new_attribute_command = 0x001d;

    pos = pos << 7;
    uint8_t data[4];
    data[0] = pos >> 24;
    data[1] = pos >> 16;
    data[2] = pos >> 8;
    data[3] = pos >> 0;

    writeData(5 + 4, motor->new_operation_command, motor->new_attribute_command, data);
}

void NidecMotor::offsetEncoder(){
    motor->new_command = offsetEncoder_;
    motor->new_operation_command = 0x05;
    motor->new_attribute_command = 0x001f;
    writeData(5, motor->new_operation_command, motor->new_attribute_command);
}

//速度制御
void NidecMotor::rollBySpeed(int rpm){
    motor->new_command = rollBySpeed_;
    motor->new_operation_command = 0x05;
    motor->new_attribute_command = 0x0022;

    rpm = rpm << 12;
    uint8_t data[4];
    data[0] = rpm >> 24;
    data[1] = rpm >> 16;
    data[2] = rpm >> 8;
    data[3] = rpm >> 0;
    
    writeData(5 + 4, motor->new_operation_command, motor->new_attribute_command, data);
}

void NidecMotor::readSpeed(){
    motor->new_command = readSpeed_;
    motor->new_operation_command = 0x01;
    motor->new_attribute_command = 0x0021;
    writeData(5, motor->new_operation_command, motor->new_attribute_command);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t NidecMotor::calcCheckSum(uint8_t *data, int check_sum_length){
    uint8_t sum = 0x00;
    for(int i = 0; i < check_sum_length; i++){
        sum += data[i];
    }
    sum = ~sum;
    return sum;
}

void NidecMotor::writeData(int data_length, uint8_t operation_command, uint16_t attribute_command, uint8_t *data){
    uint8_t data_send[data_length + 5];
    int idx = 0;
    data_send[idx++] = 0x7e;

    uint8_t data_common[data_length + 2];
    int i = 0;
    data_common[i++] = (uint8_t)id_pc;
    data_common[i++] = (uint8_t)id_motor;
    data_common[i++] = (uint8_t)(data_length >> 8 & 0xff);
    data_common[i++] = (uint8_t)(data_length & 0xff);
    data_common[i++] = operation_command;
    if(data_length > 3){
        data_common[i++] = (uint8_t)(attribute_command >> 8);
        data_common[i++] = (uint8_t)attribute_command;
    }
    if(data_length > 5){
        for(int j = 0; j < sizeof(data); j++){
            data_common[i++] = data[j];
        }
    }
    for(int j = 0; j < sizeof(data_common); j++){
        data_send[idx++] = data_common[j];
    }
    data_send[idx++] = calcCheckSum(data_common, data_length + 2);
    data_send[idx++] = 0x7f;
    
    printf("\nWriteData : 0x");
    for(int i = 0; i < data_length + 5; i++){
        printf("%02x ", data_send[i]);
    }
    printf("\n");
    
    write(fd, data_send, data_length + 5);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void NidecMotor::readData(){
    uint8_t serial_buf[128];
    //printf("read_start\n");
    int serial_size = read(fd, serial_buf, 128);
    //printf("read_end\n");
    for(int i = 0; i < serial_size; i++){
        read_buf[read_buf_index++] = serial_buf[i];
    }
}

void NidecMotor::analyzeReadData(uint8_t *read_data, int length){
    printf("\nAnalyzeData : 0x");
    for(int i = 0; i < length; i ++){
        printf("%02x ", read_data[i]);
    }
    printf("\n");
    
    analyzed_data.raw_data = read_data;
    analyzed_data.send_from = read_data[1];
    analyzed_data.send_to = read_data[2];
    analyzed_data.data_length = (uint16_t)read_data[3] << 8 | (uint16_t)read_data[4];
    analyzed_data.operation_command = read_data[5];
    if(analyzed_data.data_length > 3){
        analyzed_data.attribute_command = (uint16_t)read_data[6] << 8 | (uint16_t)read_data[7];
        
        if(analyzed_data.data_length > 5){
            analyzed_data.data = &read_data[8];
        }
    }
    analyzed_data.check_sum = read_data[3 + analyzed_data.data_length];

    if(analyzed_data.operation_command == 0x11){
        motor->returnACK(analyzed_data.data);
    }

    printf("\n");
    printf("send_from : 0x%02x\n", analyzed_data.send_from);
    printf("send_to : 0x%02x\n", analyzed_data.send_to);
    printf("data_length : 0x%04x\n", analyzed_data.data_length);
    printf("operation_command : 0x%02x\n", analyzed_data.operation_command);
    if(analyzed_data.data_length > 3){
        printf("attribute_command : 0x%04x\n", analyzed_data.attribute_command);
        if(analyzed_data.data_length > 5){
            printf("data : 0x");
            for(int i = 0; i < analyzed_data.data_length - 5; i++){
                printf("%02x", analyzed_data.data[i]);
            }
            printf("\n");
        }
    }
    printf("check_sum : 0x%02x\n", analyzed_data.check_sum);
}

void NidecMotor::returnACK(uint8_t *data){
    writeData(9, 0x91, 0xffff, data);
}
