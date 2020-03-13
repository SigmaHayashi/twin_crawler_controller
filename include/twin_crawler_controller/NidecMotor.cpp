#include "NidecMotor.h"

#include <unistd.h>

NidecMotor::NidecMotor(int fd, int id_pc, int id_motor){
    this->fd = fd;
    this->id_pc = id_pc;
    this->id_motor = id_motor;
    this->read_buf_index = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool NidecMotor::update(){
    readData();
    
    int start_idx, end_idx;
    bool start_buffering = false;
    bool end_buffering = false;

    for(int i = 0; i < 128; i ++){
        if(!start_buffering && read_buf[i] == 0x7e){
            start_idx = i;
            start_buffering = true;
        }
        if(!end_buffering && read_buf[i] == 0x7f){
            end_idx = i;
            end_buffering = true;
            break;
        }
    }
    if(!start_buffering || !end_buffering){
        return false;
    }

    uint8_t analyze_buf[128];
    for(int i = 0; i < end_idx - start_idx + 1 - i; i++){
        analyze_buf[i] = read_buf[start_idx + i];
    }
    //analyzed_data = analyzeReadData(analyze_buf);
    analyzeReadData(analyze_buf);
    
    for(int i = 0; i < 128; i ++){
        //read_buf[i] = analyze_buf[i] = 0;
        read_buf[i] = 0;
        read_buf_index = 0;
    }

    /*
    if(analyzed_data.send_to != id_pc || analyzed_data.send_from != id_motor){
        return false;
    }
    */

    return true;
}

struct NidecMotor::MotorResponse NidecMotor::readResponse(){
    MotorResponse response;

    response.raw_data = analyzed_data.raw_data;

    response.result = true;
    std::string error_message = "";
    if(analyzed_data.operation_command != new_operation_command){
        response.result = false;
        error_message += "Returned Different Operation Command\n";
    }
    if(analyzed_data.data_length > 3 && analyzed_data.attribute_command != new_attribute_command){
        response.result = false;
        error_message += "Returned Diffetent Attribute Command\n";
    }
    if(analyzed_data.operation_command & 0xc0 != 0xc0 && analyzed_data.operation_command & 0xc0 != 0x80){
        response.result = false;
        error_message += "Returned NAK\n";
    }

    response.ack = analyzed_data.operation_command;
    if(analyzed_data.operation_command & 0xc0 == 0xc0){
        response.ack_message = "Complete";
    }
    else if(analyzed_data.operation_command & 0xc0 == 0x80){
        response.ack_message = "ACK";
    }
    else{
        char nak_code[10];
        sprintf(nak_code, "0x%x", *analyzed_data.data);
        response.ack_message = "NAK : " + std::string(nak_code);
    }

    //response.data = *analyzed_data.data;
    response.data = 0;
    for(int i = 0; i < analyzed_data.data_length - 5; i++){
        response.data << 8 | analyzed_data.data[i];
    }

    return response;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void NidecMotor::run(){
    /*
    uint8_t operation_command = 0x03;

    writeData(3, operation_command);
    new_operation_command = operation_command;
    */
   new_operation_command = 0x03;
   writeData(3, new_operation_command);
}

void NidecMotor::stop(){
   new_operation_command = 0x08;
   writeData(3, new_operation_command);
}

void NidecMotor::emmergencyStop(){
   new_operation_command = 0x09;
   writeData(3, new_operation_command);
}

void NidecMotor::breakCommand(){
   new_operation_command = 0x0a;
   writeData(3, new_operation_command);
}

void NidecMotor::servoOn(){
   new_operation_command = 0x0e;
   writeData(3, new_operation_command);
}

void NidecMotor::servoOff(){
   new_operation_command = 0x0f;
   writeData(3, new_operation_command);
}

void NidecMotor::getErrorInfo(){
   new_operation_command = 0x10;
   writeData(3, new_operation_command);
}

void NidecMotor::resetError(){
   new_operation_command = 0x12;
   writeData(3, new_operation_command);
}

void NidecMotor::checkConnection(){
   new_operation_command = 0x22;
   writeData(3, new_operation_command);
}

void NidecMotor::readDeviceID(){
    /*
    uint8_t operation_command = 0x01;
    uint16_t attribute_command = 0x0004;

    writeData(5, operation_command, attribute_command);
    new_operation_command = operation_command;
    new_attribute_command = attribute_command;
    */
    new_operation_command = 0x01;
    new_attribute_command = 0x0004;
    writeData(5, new_operation_command, new_attribute_command);

    /*
    MotorResponse response;
    response.result = 0;
    response.result_message = "sample";
    return response;
    */
}

void NidecMotor::readControlMode(){
    new_operation_command = 0x01;
    new_attribute_command = 0x0001;
    writeData(5, new_operation_command, new_attribute_command);
}

void NidecMotor::writeControlMode(ControlMode mode){
    new_operation_command = 0x05;
    new_attribute_command = 0x0001;
    uint8_t data;
    switch (mode){
        case ControlMode::Release:
        data = 0x00;
        break;

        case ControlMode::Position:
        data = 0x01;
        break;

        case ControlMode::Speed:
        data = 0x04;
        break;

        case ControlMode::Torque:
        data = 0x10;
        break;

        default:
        printf("Control mode error : %d\n", mode);
        return;
    }
    writeData(5, new_operation_command, new_attribute_command, &data);
}

void NidecMotor::offsetEncoder(){
    new_operation_command = 0x05;
    new_attribute_command = 0x001f;
    writeData(5, new_operation_command, new_attribute_command);
}

//速度指令モードの確認
void NidecMotor::spinMotor(int rpm){
    new_operation_command = 0x05;
    new_attribute_command = 0x0022;
    
    rpm = rpm << 12;
    uint8_t data[4];
    data[0] = rpm >> 12;
    data[1] = rpm >> 8;
    data[2] = rpm >> 4;
    data[3] = rpm >> 0;
    
    writeData(5, new_operation_command, new_attribute_command, data);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t NidecMotor::calcChechSum(uint8_t *data){
    uint8_t sum = 0x00;
    for(int i = 0; i < sizeof(data); i++){
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
    data_send[idx++] = calcChechSum(data_common);
    data_send[idx++] = 0x7f;
    
    //printf("WriteData : %x", data_send);
    printf("CheckSum : %02x\n", calcChechSum(data_common));
    printf("WriteData : 0x");
    for(int i = 0; i < data_length + 5; i++){
        printf("%02x", data_send[i]);
    }
    printf("\n");
    
    write(fd, data_send, data_length + 5);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void NidecMotor::readData(){
    uint8_t serial_buf[128];
    int serial_size = read(fd, serial_buf, 128);
    for(int i = 0; i < serial_size; i++){
        read_buf[read_buf_index++] = serial_buf[i];
    }
}

//NidecMotor::AnalyzedData NidecMotor::analyzeReadData(uint8_t *read_data){
void NidecMotor::analyzeReadData(uint8_t *read_data){
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
    /*
    uint8_t data_data[analyzed_data.data_length - 5];
    for(int i = 0; i < analyzed_data.data_length - 5; i++){
        data_data[i] = read_data[8 + i];
    }
    analyzed_data.data = data_data;
    */
    /*
    if(analyzed_data.data_length > 5){
        analyzed_data.data = &read_data[8];
    }
    */
    analyzed_data.check_sum = read_data[3 + analyzed_data.data_length];
}

/*
void NidecMotor::getDataData(){

}

void NidecMotor::getAck(){

}
*/
