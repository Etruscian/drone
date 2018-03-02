#include <mbed.h>
#include <transceiver.h>
#include <iostream>
#include <bitset>

uint8_t Transceiver::initialize(configStruct config, dataStruct *data)
{
    firstPacketReceived = false;
    _radio.powerUp();
    radioInterrupt.fall(this, &Transceiver::interruptHandler);

    // Check is status register is correct, if not, a full reboot is needed :(
    uint8_t statRegister = _radio.getStatusRegister();
    if ((statRegister != 0x08) && (statRegister != 0x0e) && (statRegister != 0x0f))
    {
        // _radio.disable();
        // _radio.powerDown();

        // _radio.setRegister(,0x07);
        std::cout << std::hex << bitset<8>(statRegister) << std::endl;
        return 1;
    }
    _radio.setRfFrequency(2400 + config.radioConfig.channel);
    _radio.setTransferSize(config.radioConfig.transferSize);
    _radio.setCrcWidth(16);
    _radio.enableAutoAcknowledge(NRF24L01P_PIPE_P0);
    _radio.setRxAddress(config.radioConfig.rxAddress);
    _radio.setTxAddress(config.radioConfig.txAddress);
    _radio.setAirDataRate(NRF24L01P_DATARATE_250_KBPS);
    _radio.setReceiveMode();

    _radio.enable();
    transferSize = config.radioConfig.transferSize;
    rxData = new char[transferSize];
    dataPtr = data;
    return 0;
}

void Transceiver::powerDown(){
    _radio.disable();
    _radio.powerDown();
}

bool Transceiver::messageAvailable()
{
    return _radio.readable(NRF24L01P_PIPE_P0);
}

void Transceiver::interruptHandler(void){
    // Serial serialConnection(USBTX,USBRX);
    status = _radio.getStatusRegister();
    
    // if (status == 0)
    // { //data not ready?
    //     while (status == 0)
    //     {
    //         status = _radio.getStatusRegister();
    //     }
    // }
    if (status & 1)
    { // TX FIFO full
        _radio.disable();
        _radio.flushTX();
    }
    if (status & 16)
    { // max TX retransmits
        _radio.disable();
        _radio.flushTX();
        _radio.setRegister(0x07, 16);
    }
    if (status & 32)
    { // TX sent (ACK package available if autoAck is enabled)
        _radio.disable();
        _radio.flushTX();
        _radio.setRegister(0x07, 32);
    }
    if (status & 64)
    { // RX received
        firstPacketReceived = true;
        // do {
            _radio.read((status & 14) >> 1, rxData, transferSize);
            _radio.setRegister(0x07, 64);
        // } while (!(_radio.getRegister(0x17) & 1));
        // 
        // (*dataPtr).remote.throttle = 0;//(uint16_t)rxData[1] << 8 | rxData[0];
        // (*dataPtr).remote.roll = (float)((int16_t)(rxData[3] << 8 | rxData[2]));
        // (*dataPtr).remote.pitch = (float)((int16_t)(rxData[5] << 8 | rxData[4]));
        // (*dataPtr).remote.yaw = (float)((int16_t)(rxData[7] << 8 | rxData[6]));
        // (*dataPtr).acroMode = 0;//((bool)rxData[9] >> 1) & 0x01;
        // (*dataPtr).armMotor = 1;//(bool)rxData[9] & 0x01;
    }
    // _radio.writeAcknowledgePayload((status & 14) >> 1, dataPtr->batteryLevel.u, 4);
}

void Transceiver::receive(int pipe, char *buffer, uint8_t length)
{
    if (_radio.readable())
        _radio.read(pipe, buffer, length);
}

uint8_t Transceiver::movingAvg(uint8_t *ptrArrNumbers, uint16_t *ptrSum, uint8_t pos, uint16_t len, uint8_t nextNum)
{
  //Subtract the oldest number from the prev sum, add the new number
  *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
  //Assign the nextNum to the position in the array
  ptrArrNumbers[pos] = nextNum;
  //return the average
  return *ptrSum / len;
}

void Transceiver::update(void)
{
    if (messageAvailable())
    {
        packetReceived = 100;
        receive(NRF24L01P_PIPE_P0, rxData, transferSize);
        (*dataPtr).remote.throttle = (uint16_t)rxData[1] << 8 | rxData[0];
        (*dataPtr).remote.roll = (float)((int16_t)(rxData[3] << 8 | rxData[2]));
        (*dataPtr).remote.pitch = (float)((int16_t)(rxData[5] << 8 | rxData[4]));
        (*dataPtr).remote.yaw = (float)((int16_t)(rxData[7] << 8 | rxData[6]));
        (*dataPtr).acroMode = 0;//((bool)rxData[9] >> 1) & 0x01;
        (*dataPtr).armMotor = 1;//(bool)rxData[9] & 0x01;
        (*dataPtr).remote.missedPackets = 0;
    }
    else
    {
        packetReceived = 0;
        (*dataPtr).remote.missedPackets++;
    }
    signalStrength = movingAvg(signalStrengthArray, &sum, pos, sizeof(signalStrengthArray), packetReceived);
    pos++;
    
    if (signalStrength <= 50){
        if (signalLostTimeout <= 400)
            signalLostTimeout++;
    }
    else if (signalStrength > 50){
        if (signalLostTimeout >=1)
            signalLostTimeout--;
    }

    if (signalLostTimeout >=200){
        // std::cout << (uint16_t)signalLostTimeout << std::endl;
        (*dataPtr).remote.signalLost = true;
    } else if (signalLostTimeout <=10)
        (*dataPtr).remote.signalLost = false;
}

void Transceiver::setAcknowledgePayload(int pipe)
{
    _radio.writeAcknowledgePayload(pipe, dataPtr->batteryLevel.u, 2);
}