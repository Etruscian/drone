#include <mbed.h>
#include "config.hpp"

class SerialHandler
{
  public:
    SerialHandler() : _connection(USBTX, USBRX){};
    void initialize(dataStruct * data, configStruct * config)
    {
        _data = data;
        _config = config;
        _connection.baud(115200);
        _connection.attach(callback(this, &SerialHandler::rxInterruptHandler), Serial::RxIrq);
        _connection.putc('a');
        // _connection.attach(callback(this, &SerialHandler::txInterruptHandler), Serial::TxIrq);
    };

  private:
    Serial _connection;
    // LocalFileSystem local;
    dataStruct * _data;
    configStruct * _config;
    char rxBuffer[256], txBuffer[256];
    uint8_t rxBufferPosition, txBufferPosition, interpreterPosition;

    void rxInterruptHandler(void)
    {   
        rxBuffer[rxBufferPosition] = _connection.getc();
        if (rxBuffer[rxBufferPosition] == '\n')
        {
            interpretData();
        }
        else
        {
            rxBufferPosition++;
        }
    };
    void txInterruptHandler(void){
        // _connection.putc(txBuffer[txBufferPosition]);
        // txBufferPosition++;
    };

    void interpretData(void)
    {
        while (interpreterPosition <= rxBufferPosition)
        {
            switch (rxBuffer[interpreterPosition])
            {
            case '0':{
                _connection.putc('a');
                _connection.printf("Wilbur\n");
                rxBufferPosition = 0;
                interpreterPosition = 0;
                return;
            }
            case '1':{
                _connection.printf("%u\n", (*_config).radioConfig.channel);
                _connection.printf("%lx\n", (*_config).radioConfig.txAddress);
                _connection.printf("%u\n", (*_config).tickerFrequency);
                _connection.printf("%u\n", (*_config).tickerFrequency);
                rxBufferPosition = 0;
                interpreterPosition = 0;
                return;
            }
            case '2':
                {
                LocalFileSystem local("local");
                dictionary *dir = iniparser_load("/local/config.ini");
                FILE *file = fopen("/local/config.ini", "w");
                
                char formattedString[15];
                interpreterPosition++;
                sprintf(formattedString, "%c%c%c", rxBuffer[interpreterPosition++], rxBuffer[interpreterPosition++], rxBuffer[interpreterPosition++]);
                iniparser_set(dir, "radio:channel", formattedString);
                sprintf(formattedString, "0x00%c%c%c%c%c%c%c%c", rxBuffer[interpreterPosition++], rxBuffer[interpreterPosition++], rxBuffer[interpreterPosition++], rxBuffer[interpreterPosition++], rxBuffer[interpreterPosition++], rxBuffer[interpreterPosition++], rxBuffer[interpreterPosition++], rxBuffer[interpreterPosition++]);
                iniparser_set(dir, "radio:txaddress", formattedString);
                iniparser_set(dir, "radio:rxaddress", formattedString);
                sprintf(formattedString, "%c%c%c%c", rxBuffer[interpreterPosition++], rxBuffer[interpreterPosition++], rxBuffer[interpreterPosition++], rxBuffer[interpreterPosition++]);
                iniparser_set(dir, "misc:tickerfrequency", formattedString);
                iniparser_dump_ini(dir, file);
                fclose(file);
                
                rxBufferPosition = 0;
                interpreterPosition = 0;
                return;
                }
            default:
                break;
            }
            interpreterPosition++;
        }
    }
};