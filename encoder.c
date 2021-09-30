/*
* @file encoder.c
* @author Bartosz Śmierciak
* @Date 2021-05-31
*
* @brief A brief description of encoder.h.
*/

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <modbus/modbus.h>
#include "encoder.h"
//#include "Registers.h"

//modbus contex
modbus_t *ctx;


//void EncodersInit()
//{
//    encoder[0] = EncoderInit(127);
//    encoder[1] = EncoderInit(126);
//}

EncoderReg EncoderRegisters = {
    .PositionH                      = 0x0001 ,//0x9C42 ,
    .PositionL                      = 0x0002 ,//0x9C43 ,
    .ActualReverseState             = 0x0003 ,//0x9C44 ,
    .TermResetState                 = 0x0004 ,//0x9C45 ,
    .SpeedH                         = 0x0005 ,//0x9C46 ,
    .SpeedL                         = 0x0006 ,//0x9C47 ,
    .LimitSwitchState               = 0x0007 ,//0x9C48 ,
    .PhysicalSTResolutionH          = 0x000C ,//0x9C4D ,
    .PhysicalSTResolutionL          = 0x000D ,//0x9C4E ,
    .PhysicalMTResolutionH          = 0x000E ,//0x9C4F ,
    .PhysicalMTResolutionL          = 0x000F ,//0x9C50 ,
    .ScalingEnabled                 = 0x0010 ,//0x9C51 ,
    .STResolutionH                  = 0x0011 ,//0x9C52 ,
    .STResolutionL                  = 0x0012 ,//0x9C53 ,
    .TotResolutionH                 = 0x0013 ,//0x9C54 ,
    .TotResolutionL                 = 0x0014 ,//0x9C55 ,
    .PresetH                        = 0x0015 ,//0x9C56 ,
    .PresetL                        = 0x0016 ,//0x9C57 ,
    .OffsetH                        = 0x0017 ,//0x9C58 ,
    .OffsetL                        = 0x0018 ,//0x9C59 ,
    .CountDirection                 = 0x0019 ,//0x9C5A ,
    .SpeedMode                      = 0x001A ,//0x9C5B ,
    .SpeedFilter                    = 0x001B ,//0x9C5C ,
    .LimitSwitchEnable              = 0x001C ,//0x9C5D ,
    .LowLimitSwitchH                = 0x001D ,//0x9C5E ,
    .LowLimitSwitchL                = 0x001E ,//0x9C5F ,
    .HighLimitSwitchH               = 0x001F ,//0x9C60 ,
    .HighLimitSwitchL               = 0x0020 ,//0x9C61 ,
    .Delay                          = 0x0021 ,//0x9C62 ,
    .ErrorReg                       = 0x0022 ,//0x9C63 ,
    .DeviceResetStore               = 0x0023 ,//0x9C63 ,
    .Parameters                     = 0x0024 ,//0x9C64 ,
    .AutoStore                      = 0x0025 ,//0x9C65 ,
    .RestoreAllParameters           = 0x0026 ,//0x9C66 ,
    .RestoreAplicationParameters    = 0x0027 ,//0x9C67 ,
    .AutoTest                       = 0x0028 ,//0x9C68 ,
    .SoftwareVersion                = 0x0029 ,//0x9C69 ,
    .SerialNumberH                  = 0x002A ,//0x9C70 ,
    .SerialNumberL                  = 0x002B ,//0x9C71 ,
    .LifeCycleCounterH              = 0x0031 ,//0x9C72 ,
    .LifeCycleCounterL              = 0x0032 ,//0x9C73 ,
    .RollCounter                    = 0x0033 ,//0x9C74 ,
    .Baudrate                       = 0x0100 ,//0x9D41 ,
    .NumberData                     = 0x0101 ,//0x9D42 ,
    .Parity                         = 0x0102 ,//0x9D43 ,
    .Stopbits                       = 0x0103 ,//0x9D44 ,
    .CommUpdate                     = 0x0104 ,//0x9D45 ,
    .NodeAddress                    = 0x0105 ,//0x9D46 ,
    .NodeUpdate                     = 0x0106 ,//0x9D47 ,
    .AutoBaudEnable                 = 0x0107 ,//0x9D48 ,
    .AutoBaudTimeout                = 0x0108 ,//0x9D49 ,
    .RestoreBusParameters           = 0x0109 ,//0x9D4A ,
    .Termination                    = 0x010C ,//0x9D4D ,
    .TermUpdate                     = 0x010D ,//0x9D4E ,
};

//Encoder EncoderInit(int slaveAddress)
//{
//    Encoder _encoder;
//    _encoder.slaveAddress = slaveAddress;
//    _encoder.GetPosition = GetPosition;
//    _encoder.GetAngle = GetSTAngle;
 
//    return _encoder;
//}



int ModbusInit(const char *device, int baud, char parity, int data_bit, int stop_bit)
{
    
    uint32_t resTimeSec = 0;
    uint32_t resTimeuSec = 600000;

    ctx = modbus_new_rtu(device, baud, parity, data_bit, stop_bit);
    if (modbus_connect(ctx) == -1)
    {
        //fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_close(ctx);
        modbus_free(ctx);
        return -1;
    }
    if (NULL == ctx)
    {
        //printf("Unable to create modbus context\n");
        modbus_close(ctx);
        modbus_free(ctx);
        return -2;
    }
    //printf("OK.\nCreated modbus context\n");
    /* Get response timeout */
    //modbus_get_response_timeout(ctx, &tv_sec, &tv_usec); 
    //printf("Default response timeout: %d sec %d usec \n", tv_sec, tv_usec );

    /* Set response timeout */
    modbus_set_response_timeout(ctx, resTimeSec, resTimeuSec); 
    //modbus_get_response_timeout(ctx, &tv_sec, &tv_usec); 
    //printf("Set response timeout:     %d sec %d usec \n", tv_sec, tv_usec );
    return 0;
}

int ModbusRead(int slaveAddress, int regAddress, uint16_t *reg)
{
    //Set the Modbus address of the remote slave
    int ans; 
    ans = modbus_set_slave(ctx, slaveAddress);
    if (ans == -1) 
    {
        //fprintf(stderr, "Invalid slave ID: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }
    ans = modbus_read_registers(ctx, regAddress, 1, reg);
    
    if (ans != 1)
    {
        //fprintf(stderr, "Failed to read modbus. Num: %d, error: %s\n", ans, modbus_strerror(errno));
        return -2;
    }
    return 0;
}

int ModbusWrite(int slaveAddress, int regAddress, uint16_t value)
{
    int ans;
	
    ans = modbus_set_slave(ctx, slaveAddress);
    if (ans == -1) 
    {
        //fprintf(stderr, "Invalid slave ID: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    ans = modbus_write_register(ctx, regAddress, value);
    if (ans != 1)
    {
        return -2;
        //printf("ERROR modbus_write_register (%d)\n", modbusAnswear);
        //printf("Address = %d, value = %d (0x%X)\n", regAddress, value, value);
    }
    return 0;
}

void ModbusClose(void)
{
    modbus_close(ctx);
    modbus_free(ctx);
}

int Get2Registers(int slaveAddress, const uint16_t regAddressL, const uint16_t regAddressH, uint32_t *reg32)
{
    int ans;
    uint16_t reg16H = 0, reg16L = 0;
    ans = ModbusRead(slaveAddress, regAddressH, &reg16H);
    
    if (ans == -1) 
    {
        //fprintf(stderr, "Invalid slave ID: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }
    if (ans == -2)
    {
        //fprintf(stderr, "Failed to read modbus. Num: %d, error: %s\n", num, modbus_strerror(errno));
        return -2;
    }
    ans = ModbusRead(slaveAddress, regAddressL, &reg16L);
    if (ans == -1) 
    {
        //fprintf(stderr, "Invalid slave ID: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }
    if (ans == -2)
    {
        //fprintf(stderr, "Failed to read modbus. Num: %d, error: %s\n", num, modbus_strerror(errno));
        return -2;
    }
    uint32_t reg32H = reg16H;
    uint32_t reg32L = reg16L;
    *reg32 = (reg32H << 16) | reg32L;
    return 0;
}
<<<<<<< HEAD
/*
int GetRegister(int slaveAddress, const uint16_t regAddress, uint32_t *reg16)
=======

/*int GetRegister(int slaveAddress, const uint16_t regAddress, uint16_t *reg16)
>>>>>>> 1fc433dea014639993d9c6463126db2c95c71edd
{
    int ans;
    ans = ModbusRead(slaveAddress, regAddress, reg16);
    if (ans == -1) 
    {
        //fprintf(stderr, "Invalid slave ID: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }
    if (ans == -2)
    {
        //fprintf(stderr, "Failed to read modbus. Num: %d, error: %s\n", num, modbus_strerror(errno));
        return -2;
    }
    return 0;
}
*/
int GetPosition(int slaveAddress, uint32_t *reg32)
{
    int ans;
    ans = Get2Registers(slaveAddress, EncoderRegisters.PositionL, EncoderRegisters.PositionH, reg32);
    return ans;
}

/*
int GetPositionH(int slaveAddress, uint32_t *reg16)
{
    int ans;
    ans = GetRegister(slaveAddress, EncoderRegisters.PositionH, reg16);
    return ans;
}

int GetPositionL(int slaveAddress, uint32_t *reg16)
{
    int ans;
    ans = GetRegister(slaveAddress, EncoderRegisters.PositionL, reg16);
    return ans;
}
*/
int GetSTAngle(int slaveAddress, double *angle)
{
    int ans;
    uint32_t _Position = 0, _STResolution = 0;
    ans = Get2Registers(slaveAddress, EncoderRegisters.PositionL, EncoderRegisters.PositionH, &_Position);
    ans = Get2Registers(slaveAddress, EncoderRegisters.STResolutionL, EncoderRegisters.STResolutionH, &_STResolution);
    *angle = (((double)(_Position % _STResolution) / _STResolution))*360.0;
    
    return ans;
}

int read_en_az_pos(double *az)
{   
    return GetSTAngle(127, az);
}

int read_en_el_pos(double *el)
{
    return GetSTAngle(126, el);
}
/*
angle Angles(double STAngle)
{
    angle angle;
    angle.deg = STAngle;
    angle.d = (int)angle.deg;
    angle.degm = (angle.deg - angle.d) * 60.0;
    angle.dm = (int)angle.degm;
    angle.degs = (angle.degm - angle.dm) * 60.0;
    angle.ds = (int)angle.degs;
    angle.hour = angle.deg /15.0;
    angle.h = (int)angle.hour;
    angle.hourm = (angle.hour - angle.h) * 60.0;
    angle.hm = (int)angle.hourm;
    angle.hours = (angle.hourm - angle.hm) * 60.0;
    angle.hs = (int)angle.hours;
    return angle;
}

int read_en_az_pos(double *az)
{
    int status = 0;
    double reg32;
    //status = read_encoder(usb0, &reg32);               // odczyt pozycji w az
    // *az = reg32 * 360.0 / 65535.0 + d1.en_az_offset;   // zamiana na stopnie i dodanie offsetu
    return status 

}

int read_en_el_poz(double *el)
{

}
int GetAngleMT(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].PositionH = ModbusRead(slaveAddress, RegPositionH);
    _encoder[slaveAddress].PositionL = ModbusRead(slaveAddress, RegPositionL);
    _encoder[slaveAddress].Position = (_encoder[slaveAddress].PositionH ) << 16 | _encoder[slaveAddress].PositionL;
    return _encoder[slaveAddress].Position;
}

int GetActualReverseState(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].ActualReverseState = ModbusRead(slaveAddress, RegActualReverseState);
    return _encoder[slaveAddress].ActualReverseState;
}

int GetTermResetState(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].TermResetState = ModbusRead(slaveAddress, RegTermResetState);
    return _encoder[slaveAddress].TermResetState;
}

int GetSpeed(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].SpeedH = ModbusRead(slaveAddress, RegSpeedH);
    _encoder[slaveAddress].SpeedL = ModbusRead(slaveAddress, RegSpeedL);
    _encoder[slaveAddress].Speed = (_encoder[slaveAddress].SpeedH) << 16 | _encoder[slaveAddress].SpeedL;
    return _encoder[slaveAddress].Speed;
}

int GetLimitSwitchState(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].LimitSwitchState = ModbusRead(slaveAddress, RegLimitSwitchState);
    return _encoder[slaveAddress].LimitSwitchState;
}

int GetPhysicalSTResolution(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].PhysicalSTResolutionH = ModbusRead(slaveAddress, RegPhysicalSTResolutionH);
    _encoder[slaveAddress].PhysicalSTResolutionL = ModbusRead(slaveAddress, RegPhysicalSTResolutionL);
    _encoder[slaveAddress].PhysicalSTResolution = (_encoder[slaveAddress].PhysicalSTResolutionH) << 16 | _encoder[slaveAddress].PhysicalSTResolutionL;
    return _encoder[slaveAddress].PhysicalSTResolution;
}

int GetPhysicalMTResolution(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].PhysicalMTResolutionH = ModbusRead(slaveAddress, RegPhysicalMTResolutionH);
    _encoder[slaveAddress].PhysicalMTResolutionL = ModbusRead(slaveAddress, RegPhysicalMTResolutionL);
    _encoder[slaveAddress].PhysicalMTResolution = (_encoder[slaveAddress].PhysicalMTResolutionH) << 16 | _encoder[slaveAddress].PhysicalMTResolutionL;
    return _encoder[slaveAddress].PhysicalMTResolution;
}

int GetScalingEnabled(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].ScalingEnabled = ModbusRead(slaveAddress, RegScalingEnabled);
    return _encoder[slaveAddress].ScalingEnabled;
}

int SetScalingEnabled(int slaveAddress, uint32_t scalingEnabled)
{
    _encoder[slaveAddress].ScalingEnabled = scalingEnabled;
    EncoderWriteModbus(slaveAddress, RegScalingEnabled, _encoder[slaveAddress].ScalingEnabled);
}

int GetSTResolution(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].STResolutionH = ModbusRead(slaveAddress, RegSTResolutionH);
    _encoder[slaveAddress].STResolutionL = ModbusRead(slaveAddress, RegSTResolutionL);
    _encoder[slaveAddress].STResolution = (_encoder[slaveAddress].STResolutionH) << 16 | _encoder[slaveAddress].STResolutionL;
    return _encoder[slaveAddress].STResolution;
}

int GetTotResolution(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].TotResolutionH = ModbusRead(slaveAddress, RegTotResolutionH);
    _encoder[slaveAddress].TotResolutionL = ModbusRead(slaveAddress, RegTotResolutionL);
    _encoder[slaveAddress].TotResolution = (_encoder[slaveAddress].TotResolutionH) << 16 | _encoder[slaveAddress].TotResolutionL;
    return _encoder[slaveAddress].TotResolution;
}

int SetTotResolution(int slaveAddress, uint32_t totResolution)
{
    _encoder[slaveAddress].TotResolution = totResolution;
    EncoderWriteModbus(slaveAddress, RegTotResolutionL, _encoder[slaveAddress].TotResolution | 0x00FF);
    EncoderWriteModbus(slaveAddress, RegTotResolutionH, (_encoder[slaveAddress].TotResolution | 0xFF00) >> 16);
}

int GetPreset(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].PresetH = ModbusRead(slaveAddress, RegPresetH);
    _encoder[slaveAddress].PresetL = ModbusRead(slaveAddress, RegPresetL);
    _encoder[slaveAddress].Preset = (_encoder[slaveAddress].PresetH) << 16 | _encoder[slaveAddress].PresetL;
    return _encoder[slaveAddress].Preset;
}

int SetPreset(int slaveAddress, uint32_t preset)
{
    _encoder[slaveAddress].Preset = preset;
    EncoderWriteModbus(slaveAddress, RegPresetL, _encoder[slaveAddress].Preset | 0x00FF);
    EncoderWriteModbus(slaveAddress, RegPresetH, (_encoder[slaveAddress].Preset | 0xFF00) >> 16);
    
}

int GetOffset(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].OffsetH = ModbusRead(slaveAddress, RegOffsetH);
    _encoder[slaveAddress].OffsetL = ModbusRead(slaveAddress, RegOffsetL);
    _encoder[slaveAddress].Offset = (_encoder[slaveAddress].OffsetH) << 16 | _encoder[slaveAddress].OffsetL;
    return _encoder[slaveAddress].Offset;
}

int GetCountDirection(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].CountDirection = ModbusRead(slaveAddress, RegCountDirection);
    return _encoder[slaveAddress].CountDirection;
}

int SetCountDirection(int slaveAddress, uint32_t countDirection)
{
    _encoder[slaveAddress].CountDirection = countDirection;
    EncoderWriteModbus(slaveAddress, RegCountDirection, _encoder[slaveAddress].CountDirection);
}

int GetSpeedMode(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].SpeedMode = ModbusRead(slaveAddress, RegSpeedMode);
    return _encoder[slaveAddress].SpeedMode;
}

int SetSpeedMode(int slaveAddress, uint32_t speedMode)
{
    _encoder[slaveAddress].SpeedMode = speedMode;
    EncoderWriteModbus(slaveAddress, RegSpeedMode, _encoder[slaveAddress].SpeedMode);
}

int GetSpeedFilter(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].SpeedFilter = ModbusRead(slaveAddress, RegSpeedFilter);
    return _encoder[slaveAddress].SpeedFilter;
}

int SetSpeedFilter(int slaveAddress, uint32_t speedFilter)
{
    _encoder[slaveAddress].SpeedFilter = speedFilter;
    EncoderWriteModbus(slaveAddress, RegSpeedFilter, _encoder[slaveAddress].SpeedFilter);
}

int GetLimitSwitchEnable(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].LimitSwitchEnable = ModbusRead(slaveAddress, RegLimitSwitchEnable);
    return _encoder[slaveAddress].LimitSwitchEnable;
}

int SetLimitSwitchEnable(int slaveAddress, uint32_t limitSwitchEnable)
{
    _encoder[slaveAddress].LimitSwitchEnable = limitSwitchEnable;
    EncoderWriteModbus(slaveAddress, RegLimitSwitchEnable, _encoder[slaveAddress].LimitSwitchEnable);
}

int GetLowLimitSwitch(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].LowLimitSwitchH = ModbusRead(slaveAddress, RegLowLimitSwitchH);
    _encoder[slaveAddress].LowLimitSwitchL = ModbusRead(slaveAddress, RegLowLimitSwitchL);
    _encoder[slaveAddress].LowLimitSwitch = (_encoder[slaveAddress].LowLimitSwitchH) << 16 | _encoder[slaveAddress].LowLimitSwitchL;
    return _encoder[slaveAddress].LowLimitSwitch;
}

int SetLowLimitSwitch(int slaveAddress, uint32_t lowLimitSwitch)
{
    _encoder[slaveAddress].LowLimitSwitch = lowLimitSwitch;
    EncoderWriteModbus(slaveAddress, RegLowLimitSwitchL, _encoder[slaveAddress].LowLimitSwitch | 0x00FF);
    EncoderWriteModbus(slaveAddress, RegLowLimitSwitchH, (_encoder[slaveAddress].LowLimitSwitch | 0xFF00) >> 16);
}

int GetHighLimitSwitch(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].HighLimitSwitchH = ModbusRead(slaveAddress, RegHighLimitSwitchH);
    _encoder[slaveAddress].HighLimitSwitchL = ModbusRead(slaveAddress, RegHighLimitSwitchL);
    _encoder[slaveAddress].HighLimitSwitch = (_encoder[slaveAddress].HighLimitSwitchH) << 16 | _encoder[slaveAddress].HighLimitSwitchL;
    return _encoder[slaveAddress].HighLimitSwitch;
}

int SetHighLimitSwitch(int slaveAddress, uint32_t highLimitSwitch)
{
    _encoder[slaveAddress].HighLimitSwitch = highLimitSwitch;
    EncoderWriteModbus(slaveAddress, RegHighLimitSwitchL, _encoder[slaveAddress].HighLimitSwitch | 0x00FF);
    EncoderWriteModbus(slaveAddress, RegHighLimitSwitchH, (_encoder[slaveAddress].HighLimitSwitch | 0x00FF ) >> 16);
}

int GetDelay(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].Delay = ModbusRead(slaveAddress, RegDelay);
    return _encoder[slaveAddress].Delay;
}

int SetDelay(int slaveAddress, uint32_t delay)
{
    _encoder[slaveAddress].Delay = delay;
    EncoderWriteModbus(slaveAddress, RegDelay, _encoder[slaveAddress].Delay);
}

int GetErrorReg(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].ErrorReg = ModbusRead(slaveAddress, RegErrorReg);
    return _encoder[slaveAddress].ErrorReg;
}

int SetErrorReg(int slaveAddress, uint32_t errorReg)
{
    _encoder[slaveAddress].ErrorReg = errorReg;
    EncoderWriteModbus(slaveAddress, RegErrorReg, _encoder[slaveAddress].ErrorReg);
}

int GetDeviceResetStore(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].DeviceResetStore = ModbusRead(slaveAddress, RegDeviceResetStore);
    return _encoder[slaveAddress].DeviceResetStore;
}

int SetDeviceResetStore(int slaveAddress, uint32_t deviceResetStore)
{
    _encoder[slaveAddress].DeviceResetStore = deviceResetStore;
    EncoderWriteModbus(slaveAddress, RegDeviceResetStore, _encoder[slaveAddress].DeviceResetStore);
}

int GetParameters(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].Parameters = ModbusRead(slaveAddress, RegParameters);
    return _encoder[slaveAddress].Parameters;
}

int SetParameters(int slaveAddress, uint32_t parameters)
{
    _encoder[slaveAddress].Parameters = parameters;
    EncoderWriteModbus(slaveAddress, RegParameters, _encoder[slaveAddress].Parameters);
}

int GetAutoStore(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].AutoStore = ModbusRead(slaveAddress, RegAutoStore);
    return _encoder[slaveAddress].AutoStore;
}

int SetAutoStore(int slaveAddress, uint32_t autoStore)
{
    _encoder[slaveAddress].AutoStore = autoStore;
    EncoderWriteModbus(slaveAddress, RegAutoStore, _encoder[slaveAddress].AutoStore);
}

int GetRestoreAllParameters(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].RestoreAllParameters = ModbusRead(slaveAddress, RegRestoreAllParameters);
    return _encoder[slaveAddress].RestoreAllParameters;
}

int SetRestoreAllParameters(int slaveAddress, uint32_t restoreAllParameters)
{
    _encoder[slaveAddress].RestoreAllParameters = restoreAllParameters;
    EncoderWriteModbus(slaveAddress, RegRestoreAllParameters, _encoder[slaveAddress].RestoreAllParameters);
}

int GetRestoreAplicationParameters(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].RestoreAplicationParameters = ModbusRead(slaveAddress, RegRestoreAplicationParameters);
    return _encoder[slaveAddress].RestoreAplicationParameters;
}

void SteRestoteAplicationParameters(int slaveAddress, uint32_t restoreAplicationParameters)
{
    _encoder[slaveAddress].RestoreAplicationParameters = restoreAplicationParameters;
    EncoderWriteModbus(slaveAddress, RegRestoreAplicationParameters, _encoder[slaveAddress].RestoreAplicationParameters);
}

int GetAutoTest(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].AutoTest = ModbusRead(slaveAddress, RegAutoTest);
    return _encoder[slaveAddress].AutoTest;
}

int SetAutoTest(int slaveAddress, uint32_t autoTest)
{
    _encoder[slaveAddress].AutoTest = autoTest;
    EncoderWriteModbus(slaveAddress, RegAutoTest, _encoder[slaveAddress].AutoTest);
}
    
int GetSoftwareVersion(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].SoftwareVersion = ModbusRead(slaveAddress, RegSoftwareVersion);
    return _encoder[slaveAddress].SoftwareVersion;
}
        
int GetSerialNumber(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].SerialNumberH = ModbusRead(slaveAddress, RegSerialNumberH);
    _encoder[slaveAddress].SerialNumberL = ModbusRead(slaveAddress, RegSerialNumberL);
    _encoder[slaveAddress].SerialNumber = (_encoder[slaveAddress].SerialNumberH) << 16 | _encoder[slaveAddress].SerialNumberL;
    return _encoder[slaveAddress].SerialNumber;
}
    
int GetLifeCycleCounter(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].LifeCycleCounterH = ModbusRead(slaveAddress, RegLifeCycleCounterH);
    _encoder[slaveAddress].LifeCycleCounterL = ModbusRead(slaveAddress, RegLifeCycleCounterL);
    _encoder[slaveAddress].LifeCycleCounter = (_encoder[slaveAddress].LifeCycleCounterH) << 16 | _encoder[slaveAddress].LifeCycleCounterL;
    return _encoder[slaveAddress].LifeCycleCounter;
}
    
int GetRollCounter(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].RollCounter = ModbusRead(slaveAddress, RegRollCounter);
    return _encoder[slaveAddress].RollCounter;
}
        
int GetBaudrate(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].Baudrate = ModbusRead(slaveAddress, RegBaudrate);
    return _encoder[slaveAddress].Baudrate;
}

int SetBaudrate(int slaveAddress, uint32_t baudrate)
{
    _encoder[slaveAddress].Baudrate = baudrate;
    EncoderWriteModbus(slaveAddress, RegBaudrate, _encoder[slaveAddress].Baudrate);
}

int GetNumberData(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].NumberData = ModbusRead(slaveAddress, RegNumberData);
    return _encoder[slaveAddress].NumberData;
}

int SetNumberData(int slaveAddress, uint32_t numberData)
{
    _encoder[slaveAddress].NumberData = numberData;
    EncoderWriteModbus(slaveAddress, RegNumberData, _encoder[slaveAddress].NumberData);
}

uint32_t EncoderParity(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].Parity = ModbusRead(slaveAddress, RegParity);
    return _encoder[slaveAddress].Parity;
}

int SetParity(int slaveAddress, uint32_t parity)
{
    _encoder[slaveAddress].Parity = parity;
    EncoderWriteModbus(slaveAddress, RegParity, _encoder[slaveAddress].Parity);
}

int GetStopbits(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].Stopbits = ModbusRead(slaveAddress, RegStopbits);
    return _encoder[slaveAddress].Stopbits;
}

int SetStopbits(int slaveAddress, uint32_t stopbits)
{
    _encoder[slaveAddress].Stopbits = stopbits;
    EncoderWriteModbus(slaveAddress, RegStopbits, _encoder[slaveAddress].Stopbits);
}

int GetCommUpdate(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].CommUpdate = ModbusRead(slaveAddress, RegCommUpdate);
    return _encoder[slaveAddress].CommUpdate;
}

int SetCommUpdate(int slaveAddress, uint32_t commUpdate)
{
    _encoder[slaveAddress].CommUpdate = commUpdate;
    EncoderWriteModbus(slaveAddress, RegCommUpdate, _encoder[slaveAddress].CommUpdate);
}

int GetNodeAddress(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].NodeAddress = ModbusRead(slaveAddress, RegNodeAddress);
    return _encoder[slaveAddress].NodeAddress;
}

int SetNodeAddress(int slaveAddress, uint32_t nodeAddress)
{
    _encoder[slaveAddress].NodeAddress = nodeAddress;
    EncoderWriteModbus(slaveAddress, RegNodeAddress, _encoder[slaveAddress].NodeAddress);
}

int GetNodeUpdate(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].NodeUpdate = ModbusRead(slaveAddress, RegNodeUpdate);
    return _encoder[slaveAddress].NodeUpdate;
}

int SetNodeUpdate(int slaveAddress, uint32_t nodeUpdate)
{
    _encoder[slaveAddress].NodeUpdate = nodeUpdate;
    EncoderWriteModbus(slaveAddress, RegNodeUpdate, _encoder[slaveAddress].NodeUpdate);
}

int GetAutoBaudEnable(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].AutoBaudEnable = ModbusRead(slaveAddress, RegAutoBaudEnable);
    return _encoder[slaveAddress].AutoBaudEnable;
}

int SetAutoBaudEnable(int slaveAddress, uint32_t autoBaudEnable)
{
    _encoder[slaveAddress].AutoBaudEnable = autoBaudEnable;
    EncoderWriteModbus(slaveAddress, RegAutoBaudEnable, _encoder[slaveAddress].AutoBaudEnable);
}

int GetAutoBaudTimeout(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].AutoBaudTimeout = ModbusRead(slaveAddress, RegAutoBaudTimeout);
    return _encoder[slaveAddress].AutoBaudTimeout;
}

int SetAutoBaudTimeout(int slaveAddress, uint32_t autoBaudTimeout)
{
    _encoder[slaveAddress].AutoBaudTimeout = autoBaudTimeout;
    EncoderWriteModbus(slaveAddress, RegAutoBaudTimeout, _encoder[slaveAddress].AutoBaudTimeout);
}

int GetRestoreBusParameters(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].RestoreBusParameters = ModbusRead(slaveAddress, RegRestoreBusParameters);
    return _encoder[slaveAddress].RestoreBusParameters;
}

int SetRestoreBusParameters(int slaveAddress, uint32_t restoreBusParameters)
{
    _encoder[slaveAddress].RestoreBusParameters = restoreBusParameters;
    EncoderWriteModbus(slaveAddress, RegRestoreBusParameters, _encoder[slaveAddress].RestoreBusParameters);
}

int GetTermination(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].Termination = ModbusRead(slaveAddress, RegTermination);
    return _encoder[slaveAddress].Termination;
}

int SetTermination(int slaveAddress, uint32_t termination)
{
    _encoder[slaveAddress].Termination = termination;
    EncoderWriteModbus(slaveAddress, RegTermination, _encoder[slaveAddress].Termination);
}

int GetTermUpdate(int slaveAddress, uint32_t *reg32)
{
    _encoder[slaveAddress].TermUpdate = ModbusRead(slaveAddress, RegTermUpdate);
    return _encoder[slaveAddress].TermUpdate;
}

int SetTermUpdate(int slaveAddress, uint32_t termUpdate)
{
    _encoder[slaveAddress].TermUpdate = termUpdate;
    EncoderWriteModbus(slaveAddress, RegTermUpdate, _encoder[slaveAddress].TermUpdate);
}
*/
