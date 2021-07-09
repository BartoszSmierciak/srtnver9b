#ifndef ENCODER_H
#define ENCODER_H

#include <modbus/modbus.h>
#include <stdint.h>




typedef struct angle 
{
    double deg;
    double degm;
    double degs;
    int d;
    int dm;
    int ds;
    double hour;
    double hourm;
    double hours;
    int h;
    int hm;
    int hs;
    
}angle;



typedef struct EncoderReg{
        const uint16_t PositionH                      ;
        const uint16_t PositionL                      ;
        const uint16_t ActualReverseState             ;
        const uint16_t TermResetState                 ;
        const uint16_t SpeedH                         ;
        const uint16_t SpeedL                         ;
        const uint16_t LimitSwitchState               ;
        const uint16_t PhysicalSTResolutionH          ;
        const uint16_t PhysicalSTResolutionL          ;
        const uint16_t PhysicalMTResolutionH          ;
        const uint16_t PhysicalMTResolutionL          ;
        const uint16_t ScalingEnabled                 ;
        const uint16_t STResolutionH                  ;
        const uint16_t STResolutionL                  ;
        const uint16_t TotResolutionH                 ;
        const uint16_t TotResolutionL                 ;
        const uint16_t PresetH                        ;
        const uint16_t PresetL                        ;
        const uint16_t OffsetH                        ;
        const uint16_t OffsetL                        ;
        const uint16_t CountDirection                 ;
        const uint16_t SpeedMode                      ;
        const uint16_t SpeedFilter                    ;
        const uint16_t LimitSwitchEnable              ;
        const uint16_t LowLimitSwitchH                ;
        const uint16_t LowLimitSwitchL                ;
        const uint16_t HighLimitSwitchH               ;
        const uint16_t HighLimitSwitchL               ;
        const uint16_t Delay                          ;
        const uint16_t ErrorReg                       ;
        const uint16_t DeviceResetStore               ;
        const uint16_t Parameters                     ;
        const uint16_t AutoStore                      ;
        const uint16_t RestoreAllParameters           ;
        const uint16_t RestoreAplicationParameters    ;
        const uint16_t AutoTest                       ;
        const uint16_t SoftwareVersion                ;
        const uint16_t SerialNumberH                  ;
        const uint16_t SerialNumberL                  ;
        const uint16_t LifeCycleCounterH              ;
        const uint16_t LifeCycleCounterL              ;
        const uint16_t RollCounter                    ;
        const uint16_t Baudrate                       ;
        const uint16_t NumberData                     ;
        const uint16_t Parity                         ;
        const uint16_t Stopbits                       ;
        const uint16_t CommUpdate                     ;
        const uint16_t NodeAddress                    ;
        const uint16_t NodeUpdate                     ;
        const uint16_t AutoBaudEnable                 ;
        const uint16_t AutoBaudTimeout                ;
        const uint16_t RestoreBusParameters           ;
        const uint16_t Termination                    ;
        const uint16_t TermUpdate                     ;
}EncoderReg;

EncoderReg EncoderRegisters;
//typedef int (*GR)(uint32_t *, struct _encoder *);
//typedef int (*GD)(double *,  struct _encoder *);

//typedef struct _encoder{
//        int slaveAddress;
//        GR GetPosition;
//        GD GetAngle;

//}Encoder;


//Encoder encoder[2];

//Encoder EncoderInit(int slaveAddress);

//void EncodersInit();


/*!
* Initialisation of encoder. Set parameters of connections and connect to encoder
* \param device
* \param baud
* \param parity
* \param data_bit
* \param stop_bit
* \return 0 on success
*/
int ModbusInit( const char *device, int baud, char parity, int data_bit, int stop_bit);

int ModbusRead(int slaveAddress, int regAddress, uint16_t *reg);

int ModbusWrite(int slaveAddress, int regAddress, uint16_t value);

//Close modbus connection
void ModbusClose(void);

int GetPosition(int slaveAddress, uint32_t *reg32);


int GetRegister(int slaveAddress, const uint16_t regAddress, uint32_t *reg16);
/*!
* Get encoder reverse state.
* Read 8 bit (MSB 0xF000) register of reverse state.
* Actual State CW = 0, CCW = 1
* Default is 0
* \param slaveAddress The node adress of the encoder
* \return Reverse state on success
*/
int GetPositionH(int slaveAddress, uint32_t *reg16);

int GetPositionL(int slaveAddress, uint32_t *reg16);

int GetSTAngle(int slaveAddress, double *angle);

int read_en_az_pos(double *az);

int read_en_el_pos(double *el);

int GetActualReverseState(int slaveAddress);

//8 bit MSB 0xF000
//Termination on = 1, off = 0
//Default is 1
int GetTermResetState(int slaveAddress);

//2*32 bit MSB 0xFF00 LSB 0x00FF
int GetSpeed(int slaveAddress);

//8bit 0x000F
//default 0
int GetLimitSwitchState(int slaveAddress);

//2*32 bit MSB 0xFF00 LSB 0x00FF
//defalt
//H 0
//L 8192
int GetPhysicalSTResolution(int slaveAddress);

//2*32 bit MSB 0xFF00 LSB 0x00FF 
//defalt
//H 0
//L 4096
int GetPhysicalMTResolution(int slaveAddress);

/*!
* Get scaling enabled.
* 8 bit 0x000F
* \return TODO
*/
int GetScalingEnabled(int slaveAddress);

/*!
* Set scaling enabled.
* \param slaveAddress
* \param scalingEnabled
*/
int SetScalingEnabled(int slaveAddress, uint32_t scalingEnabled);

int GetSTResolution(int slaveAddress);

int GetTotResolution(int slaveAddress);

int SetTotResolution(int slaveAddress, uint32_t totResolution);

int GetPreset(int slaveAddress);

int SetPreEncoderSet(int slaveAddress, uint32_t preEncoderSet);

int GetOffset(int slaveAddress);

int GetCountDirection(int slaveAddress);

int SetCountDirection(int slaveAddress, uint32_t countDirection);

int GetSpeedMode(int slaveAddress);

int SetSpeedMode(int slaveAddress, uint32_t speedMode);

int GetSpeedFilter(int slaveAddress);

int SetSpeedFilter(int slaveAddress, uint32_t speedFilter);

int GetLimitSwitchEnable(int slaveAddress);

int SetLimitSwitchEnable(int slaveAddress, uint32_t limitSwitchEnable);

int GetLowLimitSwitch(int slaveAddress);

int SetLowLimitSwitch(int slaveAddress, uint32_t lowLimitSwitch);

int GetHighLimitSwitch(int slaveAddress);

int SetHighLimitSwitch(int slaveAddress, uint32_t highLimitSwitch);

int GetDelay(int slaveAddress);

int SetDelay(int slaveAddress, uint32_t delay);

int GetErrorReg(int slaveAddress);

int SetErrorReg(int slaveAddress, uint32_t errorReg);

int GetDeviceResetStore(int slaveAddress);

int SetDeviceReEncoderSetStore(int slaveAddress, uint32_t deviceReEncoderSetStore);

int GetParameters(int slaveAddress);

int SetParameters(int slaveAddress, uint32_t parameters);

int GetAutoStore(int slaveAddress);

int SetAutoStore(int slaveAddress, uint32_t autostore);

int GetRestoreAllParameters(int slaveAddress);

int SetRestoreAllParameters(int slaveAddress, uint32_t restoreAllParameters);

int GetRestoreAplicationParameters(int slaveAddress);

void SteRestoteAplicationParameters(int slaveAddress, uint32_t restoreAplicationParameters);

int GetAutoTest(int slaveAddress);

int SetAutoTest(int slaveAddress, uint32_t autoTest);

int GetSoftwareVersion(int slaveAddress);

int GetSerialNumber(int slaveAddress);

int GetLifeCycleCounter(int slaveAddress);

int GetRollCounter(int slaveAddress);

int GetBaudrate(int slaveAddress);

int SetBaudrate(int slaveAddress, uint32_t baudrate);

int GetNumberData(int slaveAddress);

int SetNumberData(int slaveAddress, uint32_t numberData);

int GetParity(int slaveAddress);

int SetParity(int slaveAddress, uint32_t parity);

int GetStopbits(int slaveAddress);

int SetStopbits(int slaveAddress, uint32_t stopbits);

int GetCommUpdate(int slaveAddress);

int SetCommUpdate(int slaveAddress, uint32_t commUpdate);

int GetNodeAddress(int slaveAddress);

int SetNodeAddress(int slaveAddress, uint32_t nodeAddress);

int GetNodeUpdate(int slaveAddress);

int SetNodeUpdate(int slaveAddress, uint32_t nodeUpdate);

int GetAutoBaudEnable(int slaveAddress);

int SetAutoBaudEnable(int slaveAddress, uint32_t autoBaudEnable);

int GetAutoBaudTimeout(int slaveAddress);

int SetAutoBaudTimeout(int slaveAddress, uint32_t autoBaudTimeout);

int GetRestoreBusParameters(int slaveAddress);

int SetRestoreBusParameters(int slaveAddress, uint32_t restoreBusParameters);

int GetTermination(int slaveAddress);

int SetTermination(int slaveAddress, uint32_t termination);

int GetTermUpdate(int slaveAddress);

int SetTermUpdate(int slaveAddress, uint32_t termUpdate);

#endif

