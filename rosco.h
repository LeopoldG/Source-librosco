#ifndef ROSCO_H
#define ROSCO_H

/** \file mems.h
 * Header file defining the librosco functions, structs, and enums.
 */

#include <stdint.h>
#include <stdbool.h>

#if defined(WIN32)
  #include <windows.h>
#else
  #include <pthread.h>
  #include <errno.h>
#endif

#define DEBUG_P

#ifdef DEBUG_P
  #define dprintf_err printf
#else
  #define dprintf_err
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define IAC_MAXIMUM 0xB4

/**
 * These general commands are used to request data and clear fault codes.
 */
enum mems_data_command
{
    MEMS_ReqData7D      = 0x7D,
    MEMS_ReqData80      = 0x80,
    MEMS_ClearFaults    = 0xCC,
    MEMS_Heartbeat      = 0xF4,
    MEMS_GetIACPosition = 0xFB,
    MEMS_Reset_ECU       = 0xAF,
    MEMS_Reset_ADJ       = 0xFA,
    MEMS_Reset_EMI       = 0x0F,
    MEMS_Save           = 0x81

};

/**
 * These commands are used to test actuators on the car.
 * Although some commands have on/off pairs (for controlling relays),
 * MEMS 1.6 (as fitted to the Mini SPi) will automatically shut off these
 * these actuators after a short period of time (< 1s). The "off" command,
 * if sent, will be acknowledged, but there is apparently no action taken.
 */
enum mems_actuator_command
{
    MEMS_FuelPumpOn     = 0x11,
    MEMS_FuelPumpOff    = 0x01,
    MEMS_PTCRelayOn     = 0x12,
    MEMS_PTCRelayOff    = 0x02,
    MEMS_ACRelayOn      = 0x13,
    MEMS_ACRelayOff     = 0x03,
    MEMS_PurgeValveOn   = 0x18,
    MEMS_PurgeValveOff  = 0x08,
    MEMS_O2HeaterOn     = 0x19,
    MEMS_O2HeaterOff    = 0x09,
    MEMS_BoostValveOn   = 0x1B,
    MEMS_BoostValveOff  = 0x0B,
    MEMS_Fan1On         = 0x1D,
    MEMS_Fan1Off        = 0x0D,
    MEMS_Fan2On         = 0x1E,
    MEMS_Fan2Off        = 0x0E,
    MEMS_TestInjectors  = 0xF7,
    MEMS_FireCoil       = 0xF8,
    MEMS_OpenIAC        = 0xFD,
    MEMS_CloseIAC       = 0xFE,
     MEMS_FuelTrimPlus = 0x79,
     MEMS_FuelTrimMinus = 0x7A,
     MEMS_IdleDecayPlus = 0x89,
     MEMS_IdleDecayMinus = 0x8A,
     MEMS_IdleSpeedPlus = 0x91,
     MEMS_IdleSpeedMinus = 0x92,
    MEMS_IgnitionAdvancePlus = 0x93,
    MEMS_IgnitionAdvanceMinus = 0x94
  
};

typedef enum mems_actuator_command actuator_cmd;

/**
 * Data sequence returned by the ECU in reply to the command 0x7D.
 */
typedef struct
{
  uint8_t bytes_in_frame;                 //  '0x00'
  uint8_t ignition_switch;                //  '0x01'
  uint8_t throttle_angle;                 //  '0x02'
  uint8_t uk6;                            //  '0x03'
  uint8_t air_fuel_ratio;                 //  '0x04'
  uint8_t fault_code0;                    //  '0x05'
  uint8_t lambda_voltage;                 //  '0x06'
  uint8_t lambda_sensor_frequency;        //  '0x07'
  uint8_t lambda_sensor_dutycycle;        //  '0x08'
  uint8_t lambda_sensor_status;           //  '0x09'
  uint8_t closed_loop;                    //  '0x0A'
  uint8_t long_term_fuel_trim;            //  '0x0B'
  uint8_t short_term_fuel_trim;           //  '0x0C'
  uint8_t carbon_canister_dutycycle;      //  '0x0D'
  uint8_t fault_code1;                    //  '0x0E'
  uint8_t idle_base_pos;                  //  '0x0F'
  uint8_t uk7;                            //  '0x10'
  uint8_t uk8;                            //  '0x11'
  uint8_t ignition_advance2;              //  '0x12'
  uint8_t uk9;				  //  '0x13'
  uint8_t idle_error2;			  //  '0x14'
  uint8_t uk10;				  //  '0x15'
  uint8_t fault_code4;			  //  '0x16'
  uint8_t uk11;				  //  '0x17'
  uint8_t uk12;				  //  '0x18'
  uint8_t uk13;				  //  '0x19'
  uint8_t uk14;				  //  '0x20'
  uint8_t uk15;				  //  '0x21'
  uint8_t uk16;				  //  '0x22'
  uint8_t uk1A;				  //  '0x1A'
  uint8_t uk1B;				  //  '0x1B'
  uint8_t uk1C;				  //  '0x1C'
} mems_data_frame_7d;

/**
 * Data sequence returned by the ECU in reply to the command 0x80.
 */
typedef struct
{
    uint8_t bytes_in_frame;				//'0x00'
    uint8_t engine_rpm_hi;				//'0x01'
    uint8_t engine_rpm_lo;				//'0X02'
    uint8_t coolant_temp;				//'0x03'
    uint8_t ambient_temp;				//'0x04'
    uint8_t intake_air_temp;			        //'0x05'
    uint8_t fuel_temp;					//'0x06'
    uint8_t map_kpa;					//'0x07'
    uint8_t battery_voltage;			        //'0x08'
    uint8_t throttle_pot;				//'0x09'
    uint8_t idle_switch;				//'0x0A'
    uint8_t uk1;				        //'0x0B'
    uint8_t park_neutral_switch;		        //'0x0C'
    uint8_t dtc0;				        //'0x0D'
    uint8_t dtc1;                                       //'0x0E'
    uint8_t idle_set_point;				//'0x0F'
    uint8_t idle_hot;					//'0x10'
    uint8_t uk2;				        //'0x11'
    uint8_t iac_position;			        //'0x12'
    uint8_t idle_error_hi;				//'0x13'
    uint8_t idle_error_lo;				//'0x14'
    uint8_t ignition_advance_offset;	                //'0x15'
    uint8_t ignition_advance;			        //'0x16'
    uint8_t coil_time_hi;				//'0x17'
    uint8_t coil_time_lo;				//'0x18'
    uint8_t crancs;					//'0x19'
    uint8_t uk4;					//'0x1A'
    uint8_t uk5;					//'0x1B'
} mems_data_frame_80;

/**
 * Compact structure containing only the relevant data from the ECU.
 */
typedef struct
{
    uint16_t engine_rpm;   //0x01-2
    uint8_t coolant_temp_f; 	//0x03
    uint8_t ambient_temp_f;//0x04
    uint8_t intake_air_temp_f;//0x05
    uint8_t fuel_temp_f;//0x06
    float map_kpa;//0x07
    float battery_voltage;//0x08
    float throttle_pot_voltage;//0x09
    uint8_t idle_switch;//0x0A
    uint8_t uk1;//0x0B
    uint8_t park_neutral_switch;//0x0C
    uint8_t dtc0;//0x0D
    uint8_t dtc1;//0x0E
    uint8_t idle_set_point;//0x0F
    uint8_t idle_hot;//0x10
    uint8_t uk2;//0x11
    uint8_t iac_position;//0x12
    uint16_t idle_error;//0x13-14
    uint8_t ignition_advance_offset;//0x15
    float ignition_advance;//0x16
    float coil_time;//0x17-18
    uint8_t crancs;//0x19
    uint8_t uk4;//0x1A
    uint8_t uk5;//0x1B
    uint8_t fault_codes;
    //******dframe 7**********************************
    uint8_t ignition_switch;//0x01
    uint8_t throttle_angle;//0x02
    uint8_t uk6;//0x03
    float air_fuel_ratio;//0x04
    uint8_t fault_code0;//0x05
    uint16_t lambda_voltage_mv;//0x06
    uint16_t lambda_sensor_frequency;//0x07
    uint16_t lambda_sensor_dutycycle;//0x08
    uint16_t lambda_sensor_status;//0x09
    uint8_t closed_loop;//0x0A
    uint8_t long_term_fuel_trim;//0x0B
    uint8_t short_term_fuel_trim;//0x0C
    uint8_t carbon_canister_dutycycle;//0x0D
    uint8_t fault_code1;//0x0E
    uint8_t idle_base_pos;////0x0F
    uint8_t uk7;//0x10
    uint8_t uk8;//0x11
    uint8_t ignition_advance2;//0x12
    uint8_t uk9;//0x13
    uint8_t idle_error2;//0x14
    uint8_t uk10;//0x15
    uint8_t fault_code4;//0x16
    uint8_t uk11;//0x17
    uint8_t uk12;//0x18
    uint8_t uk13;//0x19
    uint8_t uk14;//0x1A
    uint8_t uk15;//0x1B
    uint8_t uk16;//0x1C
    uint8_t uk1A;//0x1D
    uint8_t uk1B;//0x1E
    uint8_t uk1C;//0x1F
}
mems_data;

/**
 * Major/minor/patch version numbers for this build of the library
 */
typedef struct
{
  //! Major version number
  uint8_t major;
  //! Minor version number
  uint8_t minor;
  //! Patch version number
  uint8_t patch;
} librosco_version;

/**
 * Contains information about the state of the current connection to the ECU.
 */
typedef struct
{
#if defined(WIN32)
    //! Descriptor for the serial port device
    HANDLE sd;
    //! Lock to prevent multiple simultaneous open/close/read/write operations
    HANDLE mutex;
#else
    //! Descriptor for the serial port device
    int sd;
    //! Lock to prevent multiple simultaneous open/close/read/write operations
    pthread_mutex_t mutex;
#endif
} mems_info;

void mems_init(mems_info* info);
bool mems_init_link(mems_info* info, uint8_t* d0_response_buffer);
void mems_cleanup(mems_info* info);
bool mems_connect(mems_info* info, const char* devPath);
void mems_disconnect(mems_info* info);
bool mems_is_connected(mems_info* info);
bool mems_read_raw(mems_info* info, mems_data_frame_80* frame80, mems_data_frame_7d* frame7d);
bool mems_read(mems_info* info, mems_data* data);
bool mems_read_iac_position(mems_info* info, uint8_t* position);
bool mems_move_iac(mems_info* info, uint8_t desired_pos);
bool mems_test_actuator(mems_info* info, actuator_cmd cmd, uint8_t* data);
bool mems_clear_faults();
bool mems_reset_ECU();
bool mems_reset_ADJ();
bool mems_reset_EMI();
bool mems_heartbeat(mems_info* info);
bool mems_write_interactive(mems_info* info);
bool mems_fuel_trim_plus(mems_info* info,uint8_t* data);
bool mems_fuel_trim_minus(mems_info* info,uint8_t* data);
bool mems_idle_decay_plus(mems_info* info,uint8_t* data);
bool mems_idle_decay_minus(mems_info* info,uint8_t* data);
bool mems_idle_speed_plus(mems_info* info,uint8_t* data);
bool mems_idle_speed_minus(mems_info* info,uint8_t* data);
bool mems_ignition_advance_plus(mems_info* info,uint8_t* data);
bool mems_ignition_advance_minus(mems_info* info,uint8_t* data);
bool mems_interactive_mode(mems_info* info, uint8_t* response_buffer);
bool mems_Purge_Valve_On(mems_info* info,uint8_t* data);
bool mems_Purge_Valve_Off(mems_info* info,uint8_t* data);
bool mems_O2Heater_On(mems_info* info,uint8_t* data);
bool mems_O2Heater_Off(mems_info* info,uint8_t* data);
bool mems_Fan1_On(mems_info* info,uint8_t* data);
bool mems_Fan1_Off(mems_info* info,uint8_t* data);
bool mems_Fan2_On(mems_info* info,uint8_t* data);
bool mems_Fan2_Off(mems_info* info,uint8_t* data);
bool mems_Boost_Valve_On(mems_info* info,uint8_t* data);
bool mems_Boost_Valve_Off(mems_info* info,uint8_t* data);
bool mems_Save();
librosco_version mems_get_lib_version();

/* Closing brace for 'extern "C"' */
#ifdef __cplusplus
}
#endif

#endif // MEMS_H

