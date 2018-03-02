#include <nRF24L01P.h>
#include <config.hpp>

class Transceiver{
    private:
        nRF24L01P _radio;
        uint8_t transferSize;
        char * rxData;
        dataStruct * dataPtr;
        uint8_t pos, status;
        uint8_t movingAvg(uint8_t *ptrArrNumbers, uint16_t *ptrSum, uint8_t pos, uint16_t len, uint8_t nextNum);
        uint8_t signalStrengthArray[256];
        uint16_t sum;
        volatile uint8_t packetReceived;
        volatile uint8_t signalStrength;
        uint16_t signalLostTimeout;
        InterruptIn radioInterrupt;
        void interruptHandler(void);

    public:
        Transceiver(PinName mosi, PinName miso, PinName sck, PinName csn, PinName ce): _radio(mosi,miso,sck,csn,ce), radioInterrupt(p10){};
        uint8_t initialize(configStruct config, dataStruct * data);
        bool messageAvailable();
        void send();
        void receive(int pipe,char* buffer,uint8_t length);
        void update();
        void setAcknowledgePayload(int pipe);
        void powerDown();
        bool firstPacketReceived;
    };
