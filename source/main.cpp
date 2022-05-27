/**
*******************************************************************************
* @file    main.cpp
* @author  AST / Central Lab
* @version V1.0.0
* @date    June-2017
* @brief   Teseo Location Hello World
*
*******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
********************************************************************************
*/

#include "mbed.h"
#include "Teseo.h"
#include "GPSProvider.h"

#include "geofence_config.h"

#define DEBUG_RX_PIN   D0
#define DEBUG_TX_PIN   D1

/* appliation commands */
typedef enum AppCmd {
    APP_CMD_IDLE,           // No special command
    APP_CMD_HELP,           // Show the supported commands
    APP_CMD_START,          // Start location
    APP_CMD_STOP,           // Stop location
    APP_CMD_GETLASTLOC,     // Get last location
    APP_CMD_ENGEOFENCE,     // Enable Geofence
    APP_CMD_CONFGEOFENCE,   // Config Geofence
    APP_CMD_GEOFENCEREQ,    // Request Geofence status
    APP_CMD_ENODO,          // Enable Odometer
    APP_CMD_STARTODO,       // Start Odometer system
    APP_CMD_STOPODO,        // Stop Odometer system
    APP_CMD_ENDATALOG,      // Enable Datalog
    APP_CMD_CONFDATALOG,    // Config Datalog
    APP_CMD_STARTDATALOG,   // Start Datalog
    APP_CMD_STOPDATALOG,    // Stop Datalog
    APP_CMD_ERASEDATALOG,   // Erase Datalog
    APP_CMD_VERBOSE,        // Enable verbose mode
    APP_CMD_RESET,          // Debug command, pull reset pin high level
    APP_CMD_GET_DEVICE_INFO // Get Device Info
} eAppCmd;

static void _AppShowCmd(void);
static void _ConsoleRxHandler(void);
static void _AppCmdProcess(char *pCmd);
static void _AppShowLastPosition(const GPSProvider::LocationUpdateParams_t *lastLoc);
static void _AppEnGeofence(const bool isGeofenceSupported);
static void _AppGeofenceCfg(void);
static void _AppEnOdometer(const bool isOdometerSupported);
static void _AppEnDatalogging(const bool isDataloggingSupported);
static void _AppDatalogCfg(void);

static void _ExecAppCmd(void);

static int sAppCmd = APP_CMD_IDLE;
static eGeofenceId geofenceId;

static BufferedSerial serialDebug(DEBUG_TX_PIN, DEBUG_RX_PIN, 115200);
#define TESEO_APP_LOG_INFO(...) printf(__VA_ARGS__)

#define WARNING_NOT_RUN_MSG TESEO_APP_LOG_INFO("GNSS is not running. Please, type 'start' to make it runnable.\r\n");
#define WARNING_ALREADY_RUN_MSG TESEO_APP_LOG_INFO("GNSS is already running.\r\n");

static GPSProvider gnss;
static bool gnssRunning = false;
static int level = 1;

FileHandle *mbed::mbed_override_console(int fd)
{
    return &serialDebug;
}

void
locationHandler(const GPSProvider::LocationUpdateParams_t *params)
{
  if (params->valid) {
    /* application specific handling of location data; */
    TESEO_APP_LOG_INFO("locationHandler...\r\n");
  }
}

void
geofenceCfg(int ret_code)
{
  TESEO_APP_LOG_INFO("geofenceCfg...\r\n");
}

void
geofenceStatus(const GPSProvider::GeofenceStatusParams_t *params, int ret_code)
{
  /* application specific handling of geofencing status data; */
  TESEO_APP_LOG_INFO("geofenceStatus...\r\n");
}

static void
_ConsoleRxHandler(void)
{
  static char cmd[32] = {0};
  char        ch;
  char        nl = '\n';

  while(true) {
        while (!serialDebug.readable()) {
            ThisThread::yield(); // Allow other threads to run
        }
        serialDebug.read(&ch, 1);
        serialDebug.write((const void *)&ch, 1);
        if (ch == '\r') {
            serialDebug.write((const void *)&nl, 1);
            if (strlen(cmd) > 0) {
                _AppCmdProcess(cmd);
                memset(cmd, 0, sizeof(cmd));
            }
        } else {
            cmd[strlen(cmd)] = ch;
        }
  }
}

static
void
_ExecAppCmd(void)
{
  while (true) {
    //TESEO_APP_LOG_INFO("main thread!!!\r\n");
    switch (sAppCmd) {
    case APP_CMD_HELP:
      sAppCmd = APP_CMD_IDLE;
      _AppShowCmd();
      break;

    case APP_CMD_IDLE:
      if(gnssRunning) {
        //TESEO_APP_LOG_INFO("process.\r\n");
        gnss.process();
      }
      break;

    case APP_CMD_START:
      sAppCmd = APP_CMD_IDLE;
      if(gnssRunning) {
        WARNING_ALREADY_RUN_MSG;
      } else {
        TESEO_APP_LOG_INFO("start gnss.\r\n");
        gnss.start();
        gnssRunning = true;
      }
      break;
    case APP_CMD_STOP:
      sAppCmd = APP_CMD_IDLE;
      if(!gnssRunning) {
        WARNING_NOT_RUN_MSG;
      } else {
        TESEO_APP_LOG_INFO("stop gnss.\r\n");
        gnss.stop();
        gnssRunning = false;
      }
      break;
    case APP_CMD_RESET:
      sAppCmd = APP_CMD_IDLE;
      if(!gnssRunning) {
        WARNING_NOT_RUN_MSG;
      } else {
        TESEO_APP_LOG_INFO("reset on.\r\n");
        gnss.reset();
      }
      break;
    case APP_CMD_GETLASTLOC:
      sAppCmd = APP_CMD_IDLE;
      if(!gnssRunning) {
        WARNING_NOT_RUN_MSG;
      } else {
        TESEO_APP_LOG_INFO("get last loc.\r\n");
        _AppShowLastPosition(gnss.getLastLocation());
      }
      break;
    case APP_CMD_ENGEOFENCE:
      sAppCmd = APP_CMD_IDLE;
      if(!gnssRunning) {
        WARNING_NOT_RUN_MSG;
      } else {
        TESEO_APP_LOG_INFO("enable geofence.\r\n");
        _AppEnGeofence(gnss.isGeofencingSupported());
      }
      break;
    case APP_CMD_CONFGEOFENCE:
      sAppCmd = APP_CMD_IDLE;
      if(!gnssRunning) {
        WARNING_NOT_RUN_MSG;
      } else {
        TESEO_APP_LOG_INFO("config geofence.\r\n");
        _AppGeofenceCfg();
      }
      break;
    case APP_CMD_GEOFENCEREQ:
      sAppCmd = APP_CMD_IDLE;
      if(!gnssRunning) {
        WARNING_NOT_RUN_MSG;
      } else {
        TESEO_APP_LOG_INFO("request geofence status.\r\n");
        gnss.geofenceReq();
      }
      break;
    case APP_CMD_ENODO:
      sAppCmd = APP_CMD_IDLE;
      if(!gnssRunning) {
        WARNING_NOT_RUN_MSG;
      } else {
        TESEO_APP_LOG_INFO("enable odometer.\r\n");
        _AppEnOdometer(gnss.isOdometerSupported());
      }
      break;
    case APP_CMD_STARTODO:
      sAppCmd = APP_CMD_IDLE;
      if(!gnssRunning) {
        WARNING_NOT_RUN_MSG;
      } else {
        TESEO_APP_LOG_INFO("start odo subystem.\r\n");
        gnss.startOdo(1);
      }
      break;
    case APP_CMD_STOPODO:
      sAppCmd = APP_CMD_IDLE;
      if(!gnssRunning) {
        WARNING_NOT_RUN_MSG;
      } else {
        TESEO_APP_LOG_INFO("stop odo subystem.\r\n");
        gnss.stopOdo();
      }
      break;
    case APP_CMD_ENDATALOG:
      sAppCmd = APP_CMD_IDLE;
      if(!gnssRunning) {
        WARNING_NOT_RUN_MSG;
      } else {
        TESEO_APP_LOG_INFO("enable datalog.\r\n");
        _AppEnDatalogging(gnss.isDataloggingSupported());
      }
      break;
    case APP_CMD_CONFDATALOG:
      sAppCmd = APP_CMD_IDLE;
      if(!gnssRunning) {
        WARNING_NOT_RUN_MSG;
      } else {
        TESEO_APP_LOG_INFO("config datalog.\r\n");
        _AppDatalogCfg();
      }
      break;
    case APP_CMD_STARTDATALOG:
      sAppCmd = APP_CMD_IDLE;
      if(!gnssRunning) {
        WARNING_NOT_RUN_MSG;
      } else {
        TESEO_APP_LOG_INFO("start datalog.\r\n");
        gnss.startDatalog();
      }
      break;
    case APP_CMD_STOPDATALOG:
      sAppCmd = APP_CMD_IDLE;
      if(!gnssRunning) {
        WARNING_NOT_RUN_MSG;
      } else {
        TESEO_APP_LOG_INFO("stop datalog.\r\n");
        gnss.stopDatalog();
      }
      break;
    case APP_CMD_ERASEDATALOG:
      sAppCmd = APP_CMD_IDLE;
      if(!gnssRunning) {
        WARNING_NOT_RUN_MSG;
      } else {
        TESEO_APP_LOG_INFO("erase datalog.\r\n");
        gnss.eraseDatalog();
      }
      break;
    case APP_CMD_VERBOSE:
      sAppCmd = APP_CMD_IDLE;
      if(!gnssRunning) {
        WARNING_NOT_RUN_MSG;
      } else {
        TESEO_APP_LOG_INFO("set verbose mode.\r\n");
        gnss.setVerboseMode(level);
      }
      break;
    case APP_CMD_GET_DEVICE_INFO:
      sAppCmd = APP_CMD_IDLE;
      if(!gnssRunning) {
        WARNING_NOT_RUN_MSG;
      } else {
        TESEO_APP_LOG_INFO("get device info.\r\n");
        if(gnss.haveDeviceInfo()) {
          TESEO_APP_LOG_INFO("%s", gnss.getDeviceInfo());
        }
        TESEO_APP_LOG_INFO("\r\n");
      }
      break;
    }
    ThisThread::yield(); // Allow other threads to run
  }
}

int main() {
  Thread consoleThread;
  Thread cmdThread;

  TESEO_APP_LOG_INFO("Starting GNSS...\r\n");

  consoleThread.start(_ConsoleRxHandler);
  
  gnss.reset();
  gnss.onLocationUpdate(locationHandler);
  TESEO_APP_LOG_INFO("Success to new GNSS.\r\n");

  _AppShowCmd();
  cmdThread.start(_ExecAppCmd);
  
  while(1) {
    ThisThread::yield();
  }
  
}

static void
_AppShowLastPosition(const GPSProvider::LocationUpdateParams_t *lastLoc)
{
  char msg[256];
  GPSProvider::LocationUpdateParams_t lastLocation = *lastLoc;

  if(lastLocation.valid == true) {
    sprintf(msg,"Latitude:\t\t[ %.0f' %d'' ]\n\r",
            (lastLocation.lat - ((int)lastLocation.lat % 100)) / 100, 
            ((int)lastLocation.lat % 100));          
    TESEO_APP_LOG_INFO("%s", msg);
    
    sprintf(msg,"Longitude:\t\t[ %.0f' %d'' ]\n\r",
            (lastLocation.lon - ((int)lastLocation.lon % 100)) / 100, 
            ((int)lastLocation.lon % 100));          
    TESEO_APP_LOG_INFO("%s", msg);
    
    sprintf(msg,"Altitude:\t\t[ %.2f ]\n\r",
            lastLocation.altitude);
    TESEO_APP_LOG_INFO("%s", msg);
    
    sprintf(msg,"Satellites locked:\t[ %d ]\n\r",
            lastLocation.numGPSSVs);
    TESEO_APP_LOG_INFO("%s", msg);
    
    sprintf(msg, "UTC:\t\t\t[ %d ]\n\r",
            (int)lastLocation.utcTime);
    TESEO_APP_LOG_INFO("%s", msg);
    
  } else {
    sprintf(msg, "Last position wasn't valid.\n\n\r");
    TESEO_APP_LOG_INFO("%s", msg);
  }
}

static void
_AppEnGeofence(const bool isGeofenceSupported)
{
  if(isGeofenceSupported) {
    gps_provider_error_t ret = gnss.enableGeofence();
    if(ret == GPS_ERROR_NONE) {
      TESEO_APP_LOG_INFO("Enabling Geofencing subsystem...\n\r");
    }
  } else {
    TESEO_APP_LOG_INFO("Geofencing is not supported!\n\r");
  }

}

static void
_AppGeofenceCfg(void)
{
  GPSGeofence gf;

  switch (geofenceId) {
  case LecceId:
    gf.setGeofenceCircle(STLecce);
    break;
  case CataniaId:
    gf.setGeofenceCircle(Catania);
    break;
  }
  /*
  GPSGeofence::GeofenceCircle_t c = gf.getGeofenceCircle();
  printf("_AppGeofenceCfg id=%d\r\n", c.id);
  printf("_AppGeofenceCfg en=%d\r\n", c.enabled);
  printf("_AppGeofenceCfg tol=%d\r\n", c.tolerance);
  printf("_AppGeofenceCfg lat=%.2f\r\n", c.lat);
  printf("_AppGeofenceCfg lon=%.2f\r\n", c.lon);
  printf("_AppGeofenceCfg radius=%.2f\r\n", c.radius);
  */
  
  GPSGeofence *geofenceTable[] = {&gf};

  gnss.onGeofenceCfgMessage(geofenceCfg);
  gnss.onGeofenceStatusMessage(geofenceStatus);
  gps_provider_error_t ret = gnss.configGeofences(geofenceTable, sizeof(geofenceTable)/sizeof(GPSGeofence *));
  if(ret == GPS_ERROR_NONE) {
    TESEO_APP_LOG_INFO("Configuring Geofence circles...\n\r");
  }

}

static void
_AppEnOdometer(const bool isOdometerSupported)
{
  if(isOdometerSupported) {
    gps_provider_error_t ret = gnss.enableOdo();
    if(ret == GPS_ERROR_NONE) {
      TESEO_APP_LOG_INFO("Enabling Odometer subsystem...\n\r");
    }
  } else {
    TESEO_APP_LOG_INFO("Odometer is not supported!\n\r");
  }

}

static void
_AppEnDatalogging(const bool isDataloggingSupported)
{
  if(isDataloggingSupported) {
    gps_provider_error_t ret = gnss.enableDatalog();
    if(ret == GPS_ERROR_NONE) {
      TESEO_APP_LOG_INFO("Enabling Datalog subsystem...\n\r");
    }
  } else {
    TESEO_APP_LOG_INFO("Datalog is not supported!\n\r");
  }

}

static void
_AppDatalogCfg(void)
{
  bool          enableBufferFullAlarm = false;
  bool          enableCircularBuffer = true;
  unsigned      minRate = 5;
  unsigned      minSpeed = 0;
  unsigned      minPosition = 0;
  int           logMask = 1;

  GPSDatalog dl(enableBufferFullAlarm,
                enableCircularBuffer,
                minRate,
                minSpeed,
                minPosition,
                logMask);

  gps_provider_error_t ret = gnss.configDatalog(&dl);
  if(ret == GPS_ERROR_NONE) {
    TESEO_APP_LOG_INFO("Configuring Datalog...\n\r");
  }
}

static void
_AppShowCmd(void)
{
    TESEO_APP_LOG_INFO("Location commands:\r\n");
    TESEO_APP_LOG_INFO("    help         - help to show supported commands\r\n");
    TESEO_APP_LOG_INFO("    start        - begin location app\r\n");
    TESEO_APP_LOG_INFO("    stop         - end location app\r\n");
    TESEO_APP_LOG_INFO("    getlastloc   - get last location\r\n");
    TESEO_APP_LOG_INFO("    en-geo       - enable Geofence\r\n");
    TESEO_APP_LOG_INFO("    geo-c        - config Geofence [c=l Lecce, c=t Catania]\r\n");
    TESEO_APP_LOG_INFO("    req-geo      - request Geofence status\r\n");
    TESEO_APP_LOG_INFO("    en-odo       - enable Odoemter\r\n");
    TESEO_APP_LOG_INFO("    start-odo    - start Ododmeter [demo distance 1m]\r\n");
    TESEO_APP_LOG_INFO("    stop-odo     - stop Ododmeter\r\n");
    TESEO_APP_LOG_INFO("    en-datalog   - enable Datalog\r\n");
    TESEO_APP_LOG_INFO("    cfg-dl       - config Datalog\r\n");
    TESEO_APP_LOG_INFO("    start-dl     - start Datalog\r\n");
    TESEO_APP_LOG_INFO("    stop-dl      - stop Datalog\r\n");
    TESEO_APP_LOG_INFO("    erase-dl     - erase Datalog\r\n");
    TESEO_APP_LOG_INFO("    verbose-l    - nmea msg verbose mode [l=1 normal, l=2 debug]\r\n");
    TESEO_APP_LOG_INFO("    reset        - reset GNSS\r\n");
    TESEO_APP_LOG_INFO("    getdevinfo   - get device info\r\n");
}

static void
_AppCmdProcess(char *pCmd)
{
    if (strcmp(pCmd, "help") == 0) {
        sAppCmd = APP_CMD_HELP;
    } else if (strcmp(pCmd, "start") == 0) {
        sAppCmd = APP_CMD_START;
    } else if (strcmp(pCmd, "stop") == 0) {
        sAppCmd = APP_CMD_STOP;
    } else if (strcmp(pCmd, "getlastloc") == 0) {
        sAppCmd = APP_CMD_GETLASTLOC;
    } else if (strcmp(pCmd, "en-geo") == 0) {
        sAppCmd = APP_CMD_ENGEOFENCE;
    } else if (strcmp(pCmd, "geo-l") == 0) {
        geofenceId = LecceId;
        sAppCmd = APP_CMD_CONFGEOFENCE;
    } else if (strcmp(pCmd, "geo-t") == 0) {
        geofenceId = CataniaId;
        sAppCmd = APP_CMD_CONFGEOFENCE;
    } else if (strcmp(pCmd, "req-geo") == 0) {
        sAppCmd = APP_CMD_GEOFENCEREQ;
    } else if (strcmp(pCmd, "en-odo") == 0) {
        sAppCmd = APP_CMD_ENODO;
    } else if (strcmp(pCmd, "start-odo") == 0) {
        sAppCmd = APP_CMD_STARTODO;
    } else if (strcmp(pCmd, "stop-odo") == 0) {
        sAppCmd = APP_CMD_STOPODO;
    } else if (strcmp(pCmd, "en-datalog") == 0) {
        sAppCmd = APP_CMD_ENDATALOG;
    } else if (strcmp(pCmd, "cfg-dl") == 0) {
        sAppCmd = APP_CMD_CONFDATALOG;
    } else if (strcmp(pCmd, "start-dl") == 0) {
        sAppCmd = APP_CMD_STARTDATALOG;
    } else if (strcmp(pCmd, "stop-dl") == 0) {
        sAppCmd = APP_CMD_STOPDATALOG;
    } else if (strcmp(pCmd, "erase-dl") == 0) {
        sAppCmd = APP_CMD_ERASEDATALOG;
    } else if (strcmp(pCmd, "verbose-1") == 0) {
        level = 1;
        sAppCmd = APP_CMD_VERBOSE;
    } else if (strcmp(pCmd, "verbose-2") == 0) {
        level = 2;
        sAppCmd = APP_CMD_VERBOSE;
    } else if (strcmp(pCmd, "reset") == 0) {
        sAppCmd = APP_CMD_RESET;
    } else if (strcmp(pCmd, "getdevinfo") == 0) {
        sAppCmd = APP_CMD_GET_DEVICE_INFO;
    } else {
        TESEO_APP_LOG_INFO("\r\nUnknown command %s\r\n", pCmd);
    }

    TESEO_APP_LOG_INFO("\r\n");
}
