#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <libgen.h>
#include "rosco.h"

enum command_idx
{
  MC_Read = 0,
  MC_Read_Raw = 1,
  MC_Read_IAC = 2,
  MC_PTC = 3,
  MC_FuelPump = 4,
  MC_IAC_Close = 5,
  MC_IAC_Open = 6,
  MC_AC = 7,
  MC_Coil = 8,
  MC_Injectors = 9,
  MC_Purge_Valve_On = 10,
  MC_Purge_Valve_Off = 11,
  MC_O2Heater_On = 12,
  MC_O2Heater_Off =13,
  MC_Boost_Valve_On = 14,
  MC_Boost_Valve_Off = 15,
  MC_Fan1_On = 16,
  MC_Fan1_Off = 17,
  MC_Fan2_On = 18,
  MC_Fan2_Off = 19,
    MC_fuel_trim_plus = 20,
    MC_fuel_trim_minus = 21,
    MC_idle_decay_plus = 22,
    MC_idle_decay_minus = 23,
    MC_idle_speed_plus = 24,
    MC_idle_speed_minus = 25,
    MC_ignition_advance_plus = 26,
    MC_ignition_advance_minus = 27,
    MC_Interactive = 28,
    MC_Save=29,
    MC_Reset_Emission=30,
    MC_Reset_Adjustment=31,
    MC_Reset_ECU=32,
    MC_Num_Commands = 33
};


static const char* commands[] = { "read",
  "read-raw",
  "read-iac",
  "ptc",
  "fuelpump",
  "iac-close",
  "iac-open",
  "ac",
  "coil",
  "injectors",
  "Purge_Valve_On",
  "Purge_Valve_Off",
  "O2Heater_On",
  "O2Heater_Off",
  "Boost_Valve_On",
  "Boost_Valve_Off",
  "Fan1_On",
  "Fan1_Off",
  "Fan2_On",
  "Fan2_Off",
  "fuel_trim_plus",
  "fuel_trim_minus",
  "idle_decay_plus",
  "idle_decay_minus",
  "idle_speed_plus",
  "idle_speed_minus",
  "ignition_advance_plus",
  "ignition_advance_minus",
  "interactive",
  "Reset_Ecu",
  "Reset_Adjustment",
  "Reset_Emission",
  "mems_Save"

};

void printbuf(uint8_t* buf, unsigned int count)
{
  unsigned int idx = 0;

  while (idx < count)
  {
    idx += 1;
    printf("%02X ", buf[idx - 1]);
    if (idx % 16 == 0)
    {
      printf("\n");
    }
  }
  printf("\n");
}


int16_t readserial(mems_info* info, uint8_t* buffer, uint16_t quantity)
{
  int16_t bytesread = -1;

#if defined (WIN32)
  DWORD w32BytesRead = 0;

  if ((ReadFile(info->sd, (UCHAR *) buffer, quantity, &w32BytesRead, NULL) == TRUE) && (w32BytesRead > 0))
  {
    bytesread = w32BytesRead;
  }
#else
  bytesread = read(info->sd, buffer, quantity);
#endif

  return bytesread;
}


int16_t writeserial(mems_info* info, uint8_t* buffer, uint16_t quantity)
{
  int16_t byteswritten = -1;

#if defined(WIN32)
  DWORD w32BytesWritten = 0;

  if ((WriteFile(info->sd, (UCHAR*) buffer, quantity, &w32BytesWritten, NULL) == TRUE) &&
      (w32BytesWritten == quantity))
  {
    byteswritten = w32BytesWritten;
  }
#else
  byteswritten = write(info->sd, buffer, quantity);
#endif

  return byteswritten;
}

bool interactive_mode(mems_info* info, uint8_t* response_buffer)
{
  size_t icmd_size = 8;
  char* icmd_buf_ptr;
  uint8_t icmd;
  ssize_t bytes_read = 0;
  ssize_t total_bytes_read = 0;
  bool quit = false;

  if ((icmd_buf_ptr = (char*)malloc(icmd_size)) != NULL)
  {
    printf("Enter a command (in hex) or 'quit'.\n> ");
    while (!quit && (fgets(icmd_buf_ptr, icmd_size, stdin) != NULL))
    {
      if ((strncmp(icmd_buf_ptr, "q", 1) == 0) ||
          (strncmp(icmd_buf_ptr, "quit", 4) == 0))
      {
        quit = true;
      }
      else if (icmd_buf_ptr[0] != '\n' && icmd_buf_ptr[1] != '\r')
      {
        icmd = strtoul(icmd_buf_ptr, NULL, 16);
        if ((icmd >= 0) && (icmd <= 0xff))
        {
          if (writeserial(info, &icmd, 1) == 1)
          {
            bytes_read = 0;
            total_bytes_read = 0;
            do
            {
              bytes_read = readserial(info, response_buffer + total_bytes_read, 1);
              total_bytes_read += bytes_read;
            } while (bytes_read > 0);

            if (total_bytes_read > 0)
            {
               printbuf(response_buffer, total_bytes_read);
            }
            else
            {
              printf("No response from ECU.\n");
            }
          }
          else
          {
            printf("Error: failed to write command byte to serial port.\n");
          }
        }
        else
        {
          printf("Error: command must be between 0x00 and 0xFF.\n");
        }
        printf("> ");
      }
      else
      {
        printf("> ");
      }
    }

    free(icmd_buf_ptr);
  }
  else
  {
    printf("Error allocating command buffer memory.\n");
  }
}


int main(int argc, char **argv)
{
  bool success = false;
  int cmd_idx = 0;
  mems_data data;
  mems_data_frame_80 frame80;
  mems_data_frame_7d frame7d;
  librosco_version ver;
  mems_info info;
  uint8_t* frameptr;
  uint8_t bufidx;
  uint8_t readval = 0;
  uint8_t iac_limit_count = 80; // number of times to re-send an IAC move command when

  // the ECU is already reporting that the valve has
  // reached its requested position
  int read_loop_count = 1;
  bool read_inf = false;

  // this is twice as large as the micro's on-chip ROM, so it's probably sufficient
  uint8_t response_buffer[16384];

  char win32devicename[16];

  ver = mems_get_lib_version();

  if (argc < 3)
  {
    printf("readmems using librosco v%d.%d.%d\n", ver.major, ver.minor, ver.patch);
    printf("Diagnostic utility using ROSCO protocol for MEMS 1.6 systems\n");
    printf("Usage: %s <serial device> <command> [read-loop-count]\n", basename(argv[0]));
    printf(" where <command> is one of the following:\n");
    for (cmd_idx = 0; cmd_idx < MC_Num_Commands; ++cmd_idx)
    {
      printf("\t%s\n", commands[cmd_idx]);
    }
    printf(" and [read-loop-count] is either a number or 'inf' to read forever.\n");

    return 0;
  }

  while ((cmd_idx < MC_Num_Commands) && (strcasecmp(argv[2], commands[cmd_idx]) != 0))
  {
    cmd_idx += 1;
  }

  if (cmd_idx >= MC_Num_Commands)
  {
    printf("Invalid command: %s\n", argv[2]);
    return -1;
  }

  if (argc >= 4)
  {
    if (strcmp(argv[3], "inf") == 0)
    {
      read_inf = true;
    }
    else
    {
      read_loop_count = strtoul(argv[3], NULL, 0);
    }
  }

  if (cmd_idx != MC_Interactive)
  {
    printf("Running command: %s\n", commands[cmd_idx]);
  }

  mems_init(&info);

#if defined(WIN32)
  // correct for microsoft's legacy nonsense by prefixing with "\\.\"
  strcpy(win32devicename, "\\\\.\\");
  strncat(win32devicename, argv[1], 16);
  if (mems_connect(&info, win32devicename))
#else
  if (mems_connect(&info, argv[1]))
#endif
  {
    if (mems_init_link(&info, response_buffer))
    {
      printf("ECU responded to D0 command with: %02X %02X %02X %02X\n\n",
             response_buffer[0], response_buffer[1], response_buffer[2], response_buffer[3]);

      switch (cmd_idx)
      {
      case MC_Read:
        while (read_inf || (read_loop_count-- > 0))
        {
          if (mems_read(&info, &data))
          {
              printf("RPM: %u\n"
                     "Coolant (deg F): %u\n"
                     "Ambient (deg F): %u\n"
                     "Intake air (deg F): %u\n"
                     "Fuel temp (deg F): %u\n"
                     "MAP (kPa): %f\n"
                     "Main voltage: %f\n"
                     "Throttle pot voltage: %f\n"
                     "Idle switch:%u\n"
                     "unknown1: %u\n"
                     "Park/neutral switch: %u\n"
                     "Fault codes:%u\n"
                     "idlesetpoint:%u\n"
                     "idle_hot:%u\n"
                     "unknown2: %u\n"
                     "IAC position:%u\n"
                     "idle error:%u\n"
                     "iggnition_advance_offset: %u\n"
                     "ignition_advance:%u\n"
                     "coil_time:%u\n"
                     "crancsensor:%u\n"
                     "unknown4:%u\n"
                     "unknown5:%u\n "
                     "ignitionswitch:%u\n "
                     "throttle_angle:%u\n"
                     "unknown6:%u\n"
                     "air_fuel_ratio:%u\n"
                     "faultcode0:%u\n"
                     "lambda_voltage:%u\n"
                     "lambda_sensor_frequency:%u\n"
                     "lambda_sensor_dutycycle:%u\n"
                     "lambda_sensor_status:%u\n"
                     "closed_loop:%u\n"
                     "longterm fueltrim:%u\n"
                     "shortterm_fueltrim:%u\n"
                     "carboncanisterdudycycle:%u\n"
                     "faultcode1:%u\n"
                     "idle_base_position:%u\n"
                     "unknown7:%u\n"
                     "unknown8:%u\n"
                     "unknown9:%u\n"
                     "ignition advance:%u\n"
                     "unknown9:%u\n"
                     "idleerror2:%u\n"
                     "unknown10:%u\n "
                     "faultcode4:%u\n"
                     "unknown11:%u\n"
                     "unknown12:%u\n"
                     "unknown13:%u\n"
                     "unknown14:%u\n"
                     "unknown15:%u\n"
                     "unknown16:%u\n"
                     "unknown17:%u\n"
                     "unknown18:%u\n"
                     "unknown19:%u\n"
                     "-------------\n",
                     data.engine_rpm,
                     data.coolant_temp_f,
                     data.ambient_temp_f,
                     data.intake_air_temp_f,
                     data.fuel_temp_f,
                     data.map_kpa,
                     data.battery_voltage,
                     data.throttle_pot_voltage,
                     data.idle_switch,
                     data.uk1,
                     data.park_neutral_switch,
                     data.fault_codes,
                     data.idle_set_point,
                     data.idle_hot,
                     data.uk2,
                     data.iac_position,
                     data.idle_error,
                     data.ignition_advance_offset,
                     data.ignition_advance,
                     data.coil_time,
                     data.crancs,
                     data.uk4,
                     data.uk5,
                     data.ignition_switch,
                     data.throttle_angle,
                     data.uk6,
                     data.air_fuel_ratio,
                     data.fault_code0,
                     data.lambda_voltage_mv,
                     data.lambda_sensor_frequency,
                     data.lambda_sensor_dutycycle,
                     data.lambda_sensor_status,
                     data.closed_loop,
                     data.long_term_fuel_trim,
                     data.short_term_fuel_trim,
                     data.carbon_canister_dutycycle,
                     data.fault_code1,
                     data.idle_base_pos,
                     data.uk7,
                     data.uk8,
                     data.ignition_advance2,
                     data.uk9,
                     data.idle_error2,
                     data.uk10,
                     data.fault_code4,
                     data.uk11,
                     data.uk12,
                     data.uk13,
                     data.uk14,
                     data.uk15,
                     data.uk16,
                     data.uk1A,
                     data.uk1B,
                     data.uk1C);




            success = true;
          }
        }
        break;

      case MC_Read_Raw:
        while (read_inf || (read_loop_count-- > 0))
        {
          if (mems_read_raw(&info, &frame80, &frame7d))
          {
            frameptr = (uint8_t*)&frame80;
            printf("80: ");
            for (bufidx = 0; bufidx < sizeof(mems_data_frame_80); ++bufidx)
            {
              printf("%02X ", frameptr[bufidx]);
            }
            printf("\n");

            frameptr = (uint8_t*)&frame7d;
            printf("7D: ");
            for (bufidx = 0; bufidx < sizeof(mems_data_frame_7d); ++bufidx)
            {
              printf("%02X ", frameptr[bufidx]);
            }
            printf("\n");

            success = true;
          }
        }
        break;

      case MC_Read_IAC:
        if (mems_read_iac_position(&info, &readval))
        {
          printf("0x%02X\n", readval);
          success = true;
        }
        break;

      case MC_PTC:
        if (mems_test_actuator(&info, MEMS_PTCRelayOn, NULL))
        {
          sleep(2);
          success = mems_test_actuator(&info, MEMS_PTCRelayOff, NULL);
        }
        break;

      case MC_FuelPump:
        if (mems_test_actuator(&info, MEMS_FuelPumpOn, NULL))
        {
          sleep(2);
          success = mems_test_actuator(&info, MEMS_FuelPumpOff, NULL);
        }
        break;

      case MC_IAC_Close:
        do
        {
          success = mems_test_actuator(&info, MEMS_CloseIAC, &readval);

          // For some reason, diagnostic tools will continue to send send the
          // 'close' command many times after the IAC has already reached the
          // fully-closed position. Emulate that behavior here.
          if (success && (readval == 0x00))
          {
            iac_limit_count -= 1;
          }
        } while (success && iac_limit_count);
        break;

      case MC_IAC_Open:
        // The SP Rover 1 pod considers a value of 0xB4 to represent an opened
        // IAC valve, so repeat the open command until the valve is opened to
        // that point.
        do
        {
          success = mems_test_actuator(&info, MEMS_OpenIAC, &readval);
        } while (success && (readval < 0xB4));


      case MC_AC:
        if (mems_test_actuator(&info, MEMS_ACRelayOn, NULL))
        {
          sleep(2);
          success = mems_test_actuator(&info, MEMS_ACRelayOff, NULL);
        }
        break;

      case MC_Coil:
        success = mems_test_actuator(&info, MEMS_FireCoil, NULL);
        break;

      case MC_Injectors:
        success = mems_test_actuator(&info, MEMS_TestInjectors, NULL);
        break;



      case MC_fuel_trim_plus:
          success = mems_test_actuator(&info,MEMS_FuelTrimPlus,NULL);
          break;

      case MC_fuel_trim_minus:
          success = mems_test_actuator(&info,MEMS_FuelTrimMinus,NULL);
          break;

      case MC_idle_decay_plus:
          success = mems_test_actuator(&info,MEMS_IdleDecayPlus,NULL);
          break;

      case MC_idle_decay_minus:
          success = mems_test_actuator(&info,MEMS_IdleDecayMinus,NULL);
          break;

      case MC_idle_speed_plus:
          success = mems_test_actuator(&info,MEMS_IdleSpeedPlus,NULL);
          break;

      case MC_idle_speed_minus:
          success = mems_test_actuator(&info,MEMS_IdleSpeedMinus,NULL);
          break;

      case MC_ignition_advance_plus:
          success = mems_test_actuator(&info,MEMS_IgnitionAdvancePlus,NULL);
          break;

      case MC_ignition_advance_minus:
          success = mems_test_actuator(&info,MEMS_IgnitionAdvanceMinus,NULL);
          break;
      case MC_Purge_Valve_On:

          if (mems_test_actuator(&info, MEMS_PurgeValveOn, NULL))
          {
            sleep(2);
            success = mems_test_actuator(&info, MEMS_PurgeValveOff, NULL);
          }
          break;


      case MC_O2Heater_On:


            success = mems_test_actuator(&info, MEMS_O2HeaterOn, NULL);

          break;


      case MC_Boost_Valve_On:

          if (mems_test_actuator(&info, MEMS_BoostValveOn, NULL))
          {
            sleep(2);
            success = mems_test_actuator(&info, MEMS_BoostValveOff, NULL);
          }
          break;


      case MC_Fan1_On:

          if (mems_test_actuator(&info, MEMS_Fan1On, NULL))
          {
            sleep(2);
            success = mems_test_actuator(&info, MEMS_Fan1Off, NULL);
          }
          break;


      case MC_Fan2_On:
          if (mems_test_actuator(&info, MEMS_Fan2On, NULL))
          {
            sleep(2);
            success = mems_test_actuator(&info, MEMS_Fan2Off, NULL);
          }
          break;



      case MC_O2Heater_Off:


          {

            success = mems_test_actuator(&info, MEMS_O2HeaterOff, NULL);
          }
          break;


      case MC_Interactive:

              {
            success= interactive_mode(&info,response_buffer);
              }
            break;
      case MC_Save:

              {
            success= mems_Save(&info);
              }
            break;
      case MC_Reset_Adjustment:

              {
            success= mems_reset_ADJ(&info);
              }
            break;
      case MC_Reset_Emission:

              {
            success= mems_reset_EMI(&info);
              }
            break;
      case MC_Reset_ECU:

              {
            success= mems_reset_ECU(&info);
              }
            break;






      default:
        printf("Error: invalid command\n");
        break;
      }
    }
    else
    {
      printf("Error in initialization sequence.\n");
    }
    mems_disconnect(&info);
  }
  else
  {
#if defined(WIN32)
    printf("Error: could not open serial device (%s).\n", win32devicename);
#else
    printf("Error: could not open serial device (%s).\n", argv[1]);
#endif
  }

  mems_cleanup(&info);

  return success ? 0 : -2;
}
