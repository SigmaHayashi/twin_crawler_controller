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
            //printf("start_ids : %d\n", start_idx);
            start_buffering = true;
        }
        if(start_buffering && !end_buffering && (read_buf[i] == 0x7f)){
            end_idx = i;
            //printf("end_idx : %d\n", end_idx);
            end_buffering = true;
            break;
        }
    }
    if(!start_buffering || !end_buffering){
        return false;
    }

    //uint8_t analyze_buf[128];
    //printf("analyze_buf : ");
    //for(int i = 0; i < end_idx - start_idx + 1 - i; i++){
    for(int i = 0; i < end_idx - start_idx + 1; i++){
        analyze_buf[i] = read_buf[start_idx + i];
        //printf("%02x ", analyze_buf[i]);
    }
    //printf("\n");
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
   /*
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
    */
    return true;
}

struct NidecMotor::MotorResponse NidecMotor::readResponse(){
    MotorResponse response;

    response.raw_data = analyzed_data.raw_data;

    response.result = true;
    std::string error_message = "";
    //printf("%02x, %02x\n", analyzed_data.operation_command & 0x3f, motor->new_operation_command);
    if((analyzed_data.operation_command & 0x3f) != motor->new_operation_command){
        response.result = false;
        error_message += "Returned Different Operation Command\n";
    }
    //printf("%02x, %02x\n", analyzed_data.attribute_command, motor->new_attribute_command);
    if(analyzed_data.data_length > 3 && analyzed_data.attribute_command != motor->new_attribute_command){
        response.result = false;
        error_message += "Returned Diffetent Attribute Command\n";
    }
    if(analyzed_data.operation_command & 0xc0 != 0xc0 && analyzed_data.operation_command & 0xc0 != 0x80){
        response.result = false;
        error_message += "Returned NAK\n";
    }

    if(response.result){
        response.result_message = "Success";
    }
    else{
        response.result_message = error_message;
    }

    response.ack = analyzed_data.operation_command;
    /*
    printf("operation_command & 0xc0 : %02x\n", analyzed_data.operation_command & 0xc0);
    if(0x80 == 0xc0){
        printf("0x80 = 0xc0\n");
    }
    */
    if((analyzed_data.operation_command & 0xc0) == 0xc0){
        response.ack_message = "Complete";
    }
    else if((analyzed_data.operation_command & 0xc0) == 0x80){
        response.ack_message = "ACK";
    }
    else{
        char nak_code[10];
        sprintf(nak_code, "0x%x", *analyzed_data.data);
        response.ack_message = "NAK : " + std::string(nak_code);
    }

    //response.data = *analyzed_data.data;
    response.data = 0;
    //printf("data : ");
    for(int i = 0; i < analyzed_data.data_length - 5; i++){
        response.data = response.data << 8 | analyzed_data.data[i];
        //printf("%02x\n", analyzed_data.data[i]);
        //printf("response.data : %ld\n", response.data);
    }
    //printf("\n");
    //printf("response.data : %ld\n", response.data);

    if(response.result){
        if(motor->new_attribute_command == 0x0022){
            //printf("%ld\n", response.data);
            //int data = (int)response.data;
            //printf("%d\n", data);
            //response.data = (long)(data >> 12);
            //printf("%ld\n", response.data);
            response.data = (int)response.data >> 12;
        }
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
   motor->new_operation_command = 0x03;
   writeData(3, motor->new_operation_command);
}

void NidecMotor::stop(){
   motor->new_operation_command = 0x08;
   writeData(3, motor->new_operation_command);
}

void NidecMotor::emmergencyStop(){
   motor->new_operation_command = 0x09;
   writeData(3, motor->new_operation_command);
}

void NidecMotor::breakCommand(){
   motor->new_operation_command = 0x0a;
   writeData(3, motor->new_operation_command);
}

void NidecMotor::servoOn(){
   motor->new_operation_command = 0x0e;
   writeData(3, motor->new_operation_command);
}

void NidecMotor::servoOff(){
   motor->new_operation_command = 0x0f;
   writeData(3, motor->new_operation_command);
}

void NidecMotor::getErrorInfo(){
   motor->new_operation_command = 0x10;
   writeData(3, motor->new_operation_command);
}

void NidecMotor::resetError(){
   motor->new_operation_command = 0x12;
   writeData(3, motor->new_operation_command);
}

void NidecMotor::checkConnection(){
   motor->new_operation_command = 0x22;
   writeData(3, motor->new_operation_command);
}

void NidecMotor::readDeviceID(){
    /*
    uint8_t operation_command = 0x01;
    uint16_t attribute_command = 0x0004;

    writeData(5, operation_command, attribute_command);
    new_operation_command = operation_command;
    new_attribute_command = attribute_command;
    */
    motor->new_operation_command = 0x01;
    motor->new_attribute_command = 0x0004;
    writeData(5, motor->new_operation_command, motor->new_attribute_command);

    /*
    MotorResponse response;
    response.result = 0;
    response.result_message = "sample";
    return response;
    */
}

void NidecMotor::readControlMode(){
    motor->new_operation_command = 0x01;
    motor->new_attribute_command = 0x0001;
    writeData(5, motor->new_operation_command, motor->new_attribute_command);
}

void NidecMotor::writeControlMode(ControlMode mode){
    motor->new_operation_command = 0x05;
    motor->new_attribute_command = 0x0001;
    uint8_t data;
    switch (mode){
        /*
        case Release:
        data = 0x00;
        break;

        case Position:
        data = 0x01;
        break;

        case Speed:
        data = 0x04;
        break;

        case Torque:
        data = 0x10;
        break;
        */
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

void NidecMotor::offsetEncoder(){
    motor->new_operation_command = 0x05;
    motor->new_attribute_command = 0x001f;
    writeData(5, motor->new_operation_command, motor->new_attribute_command);
}

//速度指令モードの確認
void NidecMotor::spinMotor(int rpm){
    motor->new_operation_command = 0x05;
    motor->new_attribute_command = 0x0022;

    //printf("%08x\n", rpm);
    rpm = rpm << 12;
    //printf("%08x\n", rpm);
    uint8_t data[4];
    data[0] = rpm >> 24;
    data[1] = rpm >> 16;
    data[2] = rpm >> 8;
    data[3] = rpm >> 0;
    //printf("%02x %02x %02x %02x\n", data[0], data[1], data[2], data[3]);
    
    writeData(5 + 4, motor->new_operation_command, motor->new_attribute_command, data);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t NidecMotor::calcCheckSum(uint8_t *data, int check_sum_length){
    uint8_t sum = 0x00;
    //printf("sizeof(data) : %d\n", sizeof(data));
    //for(int i = 0; i < sizeof(data); i++){
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
    
    //printf("WriteData : %x", data_send);
    printf("CheckSum : %02x\n", calcCheckSum(data_common, data_length + 2));
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
    //printf("read_start\n");
    int serial_size = read(fd, serial_buf, 128);
    //printf("read_end\n");
    for(int i = 0; i < serial_size; i++){
        read_buf[read_buf_index++] = serial_buf[i];
    }
}

//NidecMotor::AnalyzedData NidecMotor::analyzeReadData(uint8_t *read_data){
void NidecMotor::analyzeReadData(uint8_t *read_data){
    printf("AnalyzeData : 0x");
    for(int i = 0; i < 128; i ++){
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

/*
void NidecMotor::getDataData(){

}

void NidecMotor::getAck(){

}
*/
