#include <mbed.h>
#include <transceiver.h>

uint8_t Transceiver::initialize(configStruct config, dataStruct *data)
{
    packetReceived = false;

    // Check is status register is correct, if not, a full reboot is needed :(
    uint8_t statRegister = _radio.getStatusRegister();
    if ((statRegister != 0x08) && (statRegister != 0x0e) && (statRegister != 0x0f))
    {
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
    _radio.powerUp();
    _radio.enable();
    transferSize = config.radioConfig.transferSize;
    rxBuffer = new char[transferSize];
    dataPtr = data;

    prescaler[0] = config.controllerConfig.prescaler[0];
    prescaler[1] = config.controllerConfig.prescaler[1];
    prescaler[2] = config.controllerConfig.prescaler[2];
    return 0;
}

void Transceiver::interruptHandler(void){
    status = _radio.getStatusRegister();
    
    if (status == 0)
    { //data not ready?
        while (status == 0)
        {
            status = _radio.getStatusRegister();
        }
    }
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
        signalStrength = movingAvg(signalStrengthArray, &sum, pos, sizeof(signalStrengthArray), 0);
        pos++;
        if (pos >= sizeof(signalStrengthArray) - 1){
            pos = 0;
        }
    }
    if (status & 32)
    { // TX sent (ACK package available if autoAck is enabled)
        _radio.disable();
        _radio.flushTX();
        _radio.setRegister(0x07, 32);
        signalStrength = movingAvg(signalStrengthArray, &sum, pos, sizeof(signalStrengthArray), 100);
        pos++;
        if (pos >= sizeof(signalStrengthArray) - 1){
            pos = 0;
        }
    }
    if (status & 64)
    { // RX received
        (*dataPtr).newPacket = true;
        _radio.read((status & 14) >> 1, rxBuffer, transferSize);
        _radio.setRegister(0x07, 64);
        send(0,(char * )(*dataPtr).batteryLevel.u,4);
        for (int i = 0; i<15;i++){
            rxData[i/4].c[i & 3] = rxBuffer[i];
        }
        (*dataPtr).remote.throttle = rxData[0].f;
        (*dataPtr).remote.roll = rxData[1].f * prescaler[0];
        (*dataPtr).remote.pitch = rxData[2].f * prescaler[1];
        (*dataPtr).remote.yaw = rxData[3].f * prescaler[2];
        (*dataPtr).armMotor =((bool)rxBuffer[16] >> 1) & 0x01;
        (*dataPtr).acroMode = (bool)rxBuffer[16] & 0x01;
    }
    
}

void Transceiver::send(uint8_t pipe, char * buffer, uint8_t length){
    _radio.write(pipe, buffer, length);
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
