/******************************************************************************

                        4Drawing software - main

  Copyright (C) <2014>  <DFRobot>   www.dfrobot.com

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Contact: leo.yan.cn@gmail.com

 ******************************************************************************
  Author        : Leo Yan
  Created       : 2014/6
  Last Modified :
  Description   :  The library is depend on Arduino environment.
  Function List :

******************************************************************************/


#ifndef _WEATHER_STATION_H
#define _WEATHER_STATION_H

//#include "arduino.h"
#include "public.h"
#include "dht.h"
#include <BMP085.h>
#include <LiquidCrystal_I2C.h>



/********************************************************************************/
/**user can redefine**/
#define SLEEP_OVERTIME 12  //unit:s
#define DATA_SAVE_DAYS 2   //Save two day's data ( today and yesterday)
#define SAMPLE_TIMES_PERDAY 24  //sample data every hour

#define END_HOURTIME_ONEDAY 20  //range: [0-23]
#define START_HOURTIME_ONEDAY ((END_HOURTIME_ONEDAY+1)%24)  //range: [0-23]

#define RESTART_DISPLAY_OVERTIME 3 //minute


/**pin define**/
#define PALETTE_PIN_BLK 13
#define PALETTE_PIN_SW  7

#define STAION_PIN_BUTTON 8    //PCINT18
#define STAION_PIN_RTC_INT 10  //PCINT4   if modify this , ioDisableMask must be modified


#define PIN_UART  253
#define PIN_I2C   252
#define PIN_NULL  255



/**Sensor**/
#define FUNCTION_SENSOR_DHT22    1
#define FUNCTION_SENSOR_BMP085   2
#define FUNCTION_SENSOR_GP2Y1010 3


/**DATA**/
#define DATA_TEMPERATURE   0
#define DATA_HUMIDITY      1
#define DATA_AIR_PRESSURE  2
#define DATA_PM10          3

#define DATA_TYPE_MAXNUM 4  


typedef struct
{
    uint8_t type;         //keyword
    uint8_t sensorID;
    uint8_t attachPin;
    uint8_t auxiliaryPin;
}DataConfig;

class Sensor;

typedef struct
{
    boolean valid;       //false = blank record
    uint8_t dataType;    //
    Sensor *p_sensor;    //
}MeasureDataMgt;


/** Event **/
#define EVENT_SAMPLE_TIMER   0x0
#define EVENT_BUTTON         0x1

#define EVENT_STATE_SET 0
#define EVENT_STATE_DOING 1
#define EVENT_STATE_DONE 2


/** Action **/

#define ACTION_SAMPLE_DATA   0   //use it as index of a_actionList
#define ACTION_SAVE_DATA     1
#define ACTION_DISPLAY       2
#define ACTION_SEND_DATA     3
#define ACTION_NUMBER        4

#define STATE_INIT 0
#define STATE_DOING 1
#define STATE_DONE 2

#define DISPLAY_STATE_INIT 0
#define DISPLAY_STATE_NEXT 1


typedef struct
{
    uint8_t state;       //init, doing, done
    uint8_t index;       //different fuctiong
}ActionMgt;

/**button**/
#define BUTTON_STATE_DOWN HIGH
#define BUTTON_STATE_UP   LOW


/**display**/
typedef struct
{
    uint8_t state;
    boolean dateShowFlag;
    uint8_t nowDataIndex;
    boolean statisticFlag; //display statistic data
    uint8_t days;
    uint8_t measureIndex;
    String rowInfo; 
}DisplayMgt;

/**Swith(SW) define**/
#define SW_MAKE HIGH
#define SW_RUN  LOW


/** System **/


#define SYS_STATE_INIT 0
#define SYS_STATE_ACTION 1
#define SYS_STATE_WAITING 2
#define SYS_STATE_GOTOSLEEP 3
#define SYS_STATE_SLEEPING 4




/****/
#define PALETTE_PORT_NUM 4
#define PALETTE_PIN_MVCC  4


class WeatherStation
{
public:
    WeatherStation();
    void init( DataConfig *p_dataConfig, uint8_t dataNum );
    void run();
    void processRTCAlarm();
    void processButton();
    void setEvent( uint8_t pin );

private:
    volatile uint8_t systemState;

    uint8_t measureNum;
    MeasureDataMgt a_measureData[DATA_TYPE_MAXNUM];
    ActionMgt a_actionList[ACTION_NUMBER];

    uint8_t runHours;  //running hours from start

    uint16_t currentYear;
    uint8_t currentMonth;
    uint8_t currentDate;
    uint8_t currentHour;
    uint8_t currentMinute;
    uint8_t currentSecond;

    uint8_t startTime;
    uint8_t overTime;    //uint:  s

    volatile boolean wakeFlag;  //volatile

    volatile uint8_t eventRTCState;
    volatile uint8_t eventButtonState;

    uint8_t buttonPressHours; //minute 
    uint8_t buttonPressMinute; //hour + minute 
    

    
    uint8_t switchState;


    DisplayMgt displayMgt;


    static const uint8_t ioPort[PALETTE_PORT_NUM];
    static const uint8_t ioDisableMask[PALETTE_PORT_NUM];

    uint8_t ioPreDDRValue[PALETTE_PORT_NUM];
    uint8_t ioPrePORTValue[PALETTE_PORT_NUM];


    void addAction( uint8_t actionIndex );
    uint8_t getActionState();


    void disableBeforeSleep();
    void enableAfterSleep();
    void sleep();
    void displayNext();
    void setCurrentDate();
    void clrDisplayMgt();

};





/************************/
/**EEPROM SPACE
    uint8_t validFlag;   //used 
    uint8_t todayIndex;   //yesterday = todayIndex -1  ...
    uint8_t a_year[DATA_SAVE_DAYS];  2 
    uint8_t a_month[DATA_SAVE_DAYS]; 2
    uint8_t a_date[DATA_SAVE_DAYS];  2     
    int16_t a_dataHouse[DATA_SAVE_DAYS][DATA_TYPE_MAXNUM][SAMPLE_TIMES_PERDAY];
    **/
    /*
*/
#define DATA_WRITING 0
#define DATA_VALID 0xA5


#define STORE_ADDR_VALIDFLAG  0
#define STORE_ADDR_TODAYINDEX ( STORE_ADDR_VALIDFLAG + 1 )
#define STORE_ADDR_YEAR(x) ( STORE_ADDR_TODAYINDEX + 1 + (x)*2 )
#define STORE_ADDR_MONTH(x) ( STORE_ADDR_YEAR(DATA_SAVE_DAYS) + (x) )
#define STORE_ADDR_DAYDATE(x) ( STORE_ADDR_MONTH(DATA_SAVE_DAYS) + (x) )
#define STORE_ADDR_DATA(x, y, z) ( STORE_ADDR_DAYDATE(DATA_SAVE_DAYS) + ((((x) * DATA_TYPE_MAXNUM * SAMPLE_TIMES_PERDAY ) + ((y) * SAMPLE_TIMES_PERDAY ) + (z)) * 2))

#if ( STORE_ADDR_DATA(DATA_SAVE_DAYS-1, DATA_TYPE_MAXNUM-1, SAMPLE_TIMES_PERDAY-1) > 1024 )   //eeprom size
#error "There is not enough EEPROM space!"
#endif



class DataWarehouse
{
public:
    DataWarehouse();
    void init( uint16_t nowYear, uint8_t nowMonth, uint8_t nowDate, uint8_t nowHour );
    void putData( int16_t data, uint8_t dataType, uint8_t hour );
    int16_t getData( uint8_t dataType, uint8_t dayIndex, uint8_t hour );
    int16_t getDayAverage( uint8_t dataType, uint8_t dayIndex, uint8_t *p_number = NULL );
    int16_t getDayMax( uint8_t dataType, uint8_t dayIndex, uint8_t *p_hour=NULL);
    int16_t getDayMin( uint8_t dataType, uint8_t dayIndex, uint8_t *p_hour=NULL);
    void addDataTypeIndex( uint8_t dataType);
    void updateDate( uint16_t year, uint8_t month, uint8_t date );
    uint8_t getDate( int8_t daysBeforeToday );

#if _DEBUG
    void printData( uint8_t days );
#endif

private:


    uint8_t a_dataTypeIndex[DATA_TYPE_MAXNUM];

    inline uint8_t hour2Index( uint8_t hour){ return ((24+hour) - (END_HOURTIME_ONEDAY + 1))%24;};
    inline uint8_t index2Hour( uint8_t index){ return (END_HOURTIME_ONEDAY + 1 + index)%24;};

    uint8_t getDataTypeIndex( uint8_t dataType );
    
    int16_t readData( uint8_t dayIndex, uint8_t dataTypeIndex, uint8_t hourIndex); 
    void writeData( int16_t data, uint8_t dayIndex, uint8_t dataTypeIndex, uint8_t hourIndex); 
    void writeDateInfo( uint8_t index, uint16_t year, uint8_t month, uint8_t date ); 

};










/********************************************************************************/



#define SENSOR_MAXNUM 4



class Sensor
{
public:
    Sensor();
    Sensor* creatObject( uint8_t ID, uint8_t pin, uint8_t auxiliaryPin );
    Sensor* getObject( uint8_t pin );

    virtual void start();
    virtual void stop();
    virtual boolean run();
    virtual const char * getDataName( uint8_t dataType ){return "";};
    virtual int16_t getValue( uint8_t dataType ){return INVALID_INT16;};
    virtual String formatValue( int16_t value, uint8_t dataType ){return "?";};
    //virtual float getPercentage( uint8_t dataType = 0 );


protected:
    uint8_t attachPin;
    boolean runEnable;

private:


    typedef struct
    {
        uint8_t attachPin;   //keyword
        Sensor *pObject;
    }SensorObject_stru;

    uint8_t objectNum;
    SensorObject_stru objectList[SENSOR_MAXNUM];


    void rigisterObject( uint8_t pin, Sensor *pObject );


};




class DHT22Sensor: public Sensor
{
public:
    DHT22Sensor(uint8_t pin);

    virtual void start();
    virtual void stop();
    virtual boolean run();
    virtual int16_t getValue( uint8_t dataType );
    virtual String formatValue( int16_t value, uint8_t dataType );
    virtual const char * getDataName( uint8_t dataType );

private:
    dht *pDriver;

    int16_t temperature;
    int16_t humidity;
    
    boolean execute();

};

class BMP085Sensor : public Sensor
{
public:
    BMP085Sensor(uint8_t pin);

    virtual void start();
    virtual void stop();
    virtual boolean run();
    virtual int16_t getValue( uint8_t dataType = DATA_AIR_PRESSURE );
    virtual String formatValue( int16_t value, uint8_t dataType );
    virtual const char * getDataName( uint8_t dataType );

private:
    BMP085 *pDriver;
    int16_t pressureValue;
    
};

/****/

#define GP2Y1010_SAMPLE_TIME 280  //uint: us


class GP2Y1010Sensor: public Sensor
{
public:
    GP2Y1010Sensor(uint8_t voutPin, uint8_t ledPin );

    virtual void start();
    virtual void stop();
    virtual boolean run();
    virtual int16_t getValue( uint8_t dataType = DATA_AIR_PRESSURE );
    virtual String formatValue( int16_t value, uint8_t dataType );
    virtual const char * getDataName( uint8_t dataType );

private:
    uint8_t auxiliaryPin;
    int16_t dustDensity;  //ug/m3
    
};



/********************************************************************************/



/********************************************************************************/
extern WeatherStation g_station;

extern void showErrInfo( String &Info );
extern void PCattachInterrupt(uint8_t pin);
extern void dettachUSB();

#endif
