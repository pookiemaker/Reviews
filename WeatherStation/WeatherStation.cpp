/******************************************************************************

                        4Drawing software - Sensor

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


#include <avr/sleep.h>
#include <avr/eeprom.h>

#include "public.h"
#include <Wire.h>
#include "DS3231.h"


#include "WeatherStation.h"


#define LCD_COL_NUM 16
#define LCD_ROW_NUM 2

//uint8_t retarrow[8] ={0x16,0x09,0x08,0x08,0x08,0x09,0x06};


LiquidCrystal_I2C lcd(0x20,LCD_COL_NUM,LCD_ROW_NUM);
Sensor sensorMgt;
DataWarehouse dataHouse;
DS3231 RTCObj;

bool g_RTCIntFlag = false;


/*
Event:     action

RTC alarm:  sample data,  store data, set overtime
Button 1st:  sample data, display current data, set overtime
Button N: display N data, set overtime
Overtime: sleep


Action:
 sample data:  timestamp,  sample once in limited time
 store data:
 display:
 set overtime:


ActionList:



class:  sensor   data -


*/


/* data

No.    0 = current,  1

date

 time
 temperature: -40.0 to 80.0
 humidity: 0-99.9%
 Pressure: 300.0 to 1100.0hPa
 others



 getData
 showData(time, )
 saveData
 clearData



*/




/** **/


WeatherStation g_station;


const uint8_t WeatherStation::ioPort[PALETTE_PORT_NUM] = { 3,  6, 2, 4 };   //close MVCC last
const uint8_t WeatherStation::ioDisableMask[PALETTE_PORT_NUM] = { 0b0, 0b0, 0b01010000, 0b0 };  //some pin doesn't need to disable


WeatherStation::WeatherStation()
{
    uint8_t index, i;

    for ( index = 0; index < DATA_TYPE_MAXNUM; index++ )
    {
        a_measureData[index].valid = false;
    }

    runHours = 0;
    currentHour = START_HOURTIME_ONEDAY;
    currentDate = 1;
    eventRTCState = EVENT_STATE_DONE;
    eventButtonState = EVENT_STATE_DONE;
    overTime = 0;
    systemState = SYS_STATE_WAITING;
    wakeFlag = false;

    for ( i = 0; i < ACTION_NUMBER; i++ )
    {
        a_actionList[i].state = STATE_INIT;
    }

    clrDisplayMgt();

}



void WeatherStation::init( DataConfig *p_dataConfig, uint8_t dataNum )
{
    uint8_t i, index;
    DataConfig *p_dataConfigRecord = NULL;

    /**init config table**/
    p_dataConfigRecord = p_dataConfig;
    index = 0;

    DBG_PRINTLN("measure table:");

    for( i = 0; i < dataNum; i++ )
    {
        a_measureData[index].p_sensor = sensorMgt.creatObject( p_dataConfigRecord->sensorID, p_dataConfigRecord->attachPin, p_dataConfigRecord->auxiliaryPin );
        if ( NULL == a_measureData[index].p_sensor )
        {
            String info;

            info = "Error: Rule No.";
            info += (i+1);

            showErrInfo( info );

            break;
        }

        a_measureData[index].dataType = p_dataConfigRecord->type;
        a_measureData[index].valid = true;

        dataHouse.addDataTypeIndex(a_measureData[index].dataType);

        DBG_PRINTLN_VAR(index, DEC);
        DBG_PRINTLN_VAR((word)a_measureData[index].p_sensor, HEX);

        index++;
        p_dataConfigRecord++;


    }

    measureNum = index;

    //todo enable power
    digitalWrite( PALETTE_PIN_MVCC, LOW );
    pinMode( PALETTE_PIN_MVCC, OUTPUT );
    delay(500);  //waiting power stable

    /**init devices**/
	Wire.begin();
	RTCObj.begin();
    lcd.init();
    lcd.backlight();

    setCurrentDate();
    dataHouse.init( currentYear, currentMonth, currentDate, currentHour );

    sensorMgt.start();

    /**get the state of Switch**/
    pinMode(PALETTE_PIN_SW, INPUT_PULLUP);
    switchState = digitalRead(PALETTE_PIN_SW);

    /**attach interrupt**/
    pinMode(STAION_PIN_RTC_INT, INPUT_PULLUP);
    pinMode(STAION_PIN_BUTTON, INPUT );

    PCattachInterrupt( STAION_PIN_RTC_INT );
    PCattachInterrupt( STAION_PIN_BUTTON );

    RTCObj.clearINTStatus();

#if _DEBUG
	RTCObj.enableInterrupts(EveryMinute);
#else
	RTCObj.enableInterrupts(EveryHour);
#endif

    /**Start**/
    pinMode(PALETTE_PIN_BLK, OUTPUT);
    digitalWrite(PALETTE_PIN_BLK,HIGH);

    startTime = (uint8_t)(millis()/1000);

    /**emulate pushing button**/
    setEvent( STAION_PIN_BUTTON );


}


void WeatherStation::run()
{
    uint8_t dataType;
    int16_t data;
    boolean ret;

    /*event*/

    if (g_RTCIntFlag)  //it must be at the front of      processRTCAlarm()
    {
        g_RTCIntFlag = false;
        RTCObj.clearINTStatus();
    }

    if ( EVENT_STATE_SET == eventRTCState )
    {
        processRTCAlarm();
        eventRTCState = EVENT_STATE_DOING;
    }

    if ( EVENT_STATE_SET == eventButtonState )
    {
        processButton();
        eventButtonState = EVENT_STATE_DOING;
    }


    if ( SYS_STATE_WAITING == systemState )
    {
        if ( STATE_DOING == getActionState())  //high priority
        {
            systemState = SYS_STATE_ACTION;
        }
        else if ( (uint8_t)((millis()/1000) - startTime) >= overTime )
        {
            systemState = SYS_STATE_GOTOSLEEP;

            DBG_PRINTLN("goto sleep");

            /**to minimize the time to sleep state, so there is not sleep state case **/
            if ( (SW_RUN == digitalRead(PALETTE_PIN_SW)) && (SW_MAKE == switchState) )
            {
#if !_DEBUG
                dettachUSB();
#endif
                switchState = SW_RUN;
            }


            disableBeforeSleep();

            wakeFlag = false;

            sleep();

            enableAfterSleep();
            systemState = SYS_STATE_WAITING;

            DBG_PRINTLN_VAR(systemState,DEC);

        }
        else
        {
            //nothing
        }
    }
    else if ( SYS_STATE_ACTION == systemState )
    {

        /*action*/

        if ( STATE_DOING == a_actionList[ACTION_SAMPLE_DATA].state )
        {
            DBG_PRINTLN("**Sample");
            ret = sensorMgt.run();

            if ( ret )
            {
                a_actionList[ACTION_SAMPLE_DATA].state = STATE_DONE;
            }
            else
            {
                delay(10);
            }
        }

        if ( STATE_DOING == a_actionList[ACTION_SAVE_DATA].state )
        {
            if ( STATE_DONE == a_actionList[ACTION_SAMPLE_DATA].state )
            {
                DBG_PRINTLN("**Save");
                for ( uint8_t i = 0; i < DATA_TYPE_MAXNUM; i++ )
                {
                    if ( false == a_measureData[i].valid )
                    {
                        break;
                    }

                    dataType = a_measureData[i].dataType;
                    data = a_measureData[i].p_sensor->getValue( dataType );

                    dataHouse.putData( data, dataType, currentHour );
                }

                a_actionList[ACTION_SAVE_DATA].state = STATE_DONE;

            }

        }

        if ( STATE_DOING == a_actionList[ACTION_DISPLAY].state )
        {
            if (STATE_DONE == a_actionList[ACTION_SAMPLE_DATA].state)
            {
                displayMgt.state = DISPLAY_STATE_NEXT;
            }

            if ( DISPLAY_STATE_NEXT == displayMgt.state )
            {
                DBG_PRINTLN("**Display");

                displayNext();

                a_actionList[ACTION_DISPLAY].state = STATE_DONE;
            }

        }


        if ( STATE_DONE == getActionState() )
        {
            startTime = millis()/1000;

            if( (END_HOURTIME_ONEDAY == currentHour) && (EVENT_STATE_DOING == eventRTCState) )
            {   //todo:  muti thread
                DBG_PRINTLN_VAR(currentHour,DEC);
                dataHouse.updateDate( currentYear, currentMonth, currentDate );
            }

            eventRTCState = EVENT_STATE_DONE;
            eventButtonState = EVENT_STATE_DONE;

            for ( uint8_t i = 0; i < ACTION_NUMBER; i++ )  //todo
            {
                a_actionList[i].state = STATE_INIT;
            }

            systemState = SYS_STATE_WAITING;

            DBG_PRINTLN_VAR(systemState,DEC);

        }

    }



#if 0
DateTime now = RTCObj.now();
Serial.println(now.minute(), DEC);
Serial.print(now.year(), DEC);
Serial.print('/');
Serial.print(now.month(), DEC);
Serial.print('/');
Serial.print(now.date(), DEC);
Serial.print(' ');
Serial.print(now.hour(), DEC);
Serial.print(':');
Serial.print(now.minute(), DEC);
Serial.print(':');
Serial.print(now.second(), DEC);
Serial.println(' ');
Serial.println(RTCObj.getTemperature());


#endif

}

void WeatherStation::addAction( uint8_t actionIndex )
{
    if ( STATE_INIT == a_actionList[actionIndex].state )
    {
        a_actionList[actionIndex].state = STATE_DOING;
        a_actionList[actionIndex].index = 0;
    }
}

/**
0123456789012345
��ʼ

-
 Sample data...
  **DFRobot**
-
20:00  -19.9C
 99%RH 1005hPa
-
31th-C  A=-19.9C
H=-19.9C L=-19.9C

31th-RH A=99%
H=99% L=99%

31th-hPa A=1005
H=1005 L=1005

-
30th-C  A=-19.9C
H=-19.9C L=-19.9C

30th-RH A=99%
H=99% L=99%

30th-hPa A=1005
H=1005 L=1005


TEMP(C)

T or TEMP
P or
RH

**/
void WeatherStation::displayNext()
{
    uint8_t i;
    uint8_t dataType;
    uint8_t lcdCurrentRow;
    uint8_t date;
    int16_t data;

    uint8_t &days = displayMgt.days;
    uint8_t &measureIndex = displayMgt.measureIndex;
    bool &dateShowFlag = displayMgt.dateShowFlag;
    uint8_t &nowDataIndex = displayMgt.nowDataIndex;

    String &rowInfo = displayMgt.rowInfo;
    String unitWord;

    /**display current**/

    DBG_PRINTLN_VAR(days,DEC);
    DBG_PRINTLN_VAR(nowDataIndex,DEC);
    DBG_PRINTLN_VAR(measureIndex,DEC);

    lcd.clear();
    lcdCurrentRow = 0;

    /**display date**/
    if ( !dateShowFlag )
    {
        rowInfo = "";
        rowInfo += currentYear;
        rowInfo += "/";
        rowInfo += currentMonth;
        rowInfo += "/";
        rowInfo += currentDate;

        rowInfo += " ";
        rowInfo += currentHour;
        rowInfo += ":";
        rowInfo += currentMinute;

        lcd.setCursor( 0, lcdCurrentRow++ );
        lcd.print( rowInfo );

        rowInfo = " ";  //one space before data

        dateShowFlag = true;
    }


    /**display current data**/
    if ( dateShowFlag && (nowDataIndex < measureNum) )
    {
        for ( i = nowDataIndex; i < measureNum; i++ )
        {
            if ( false == a_measureData[i].valid )
            {
                break;
            }

            dataType = a_measureData[i].dataType;
            data = a_measureData[i].p_sensor->getValue( dataType );

            unitWord = a_measureData[i].p_sensor->formatValue( data, dataType );

            if ( (rowInfo.length() + unitWord.length()) > LCD_COL_NUM )
            {
                lcd.setCursor( 0, lcdCurrentRow++ );
                lcd.print( rowInfo );

                rowInfo = " ";

                if (lcdCurrentRow >= LCD_ROW_NUM)
                {
                    break;
                }
            }

            rowInfo += unitWord;
            rowInfo += " ";

        }


        /*display the last row*/
        if ( rowInfo.length() > 2 )  //valid str > 2
        {
            lcd.setCursor( 0, lcdCurrentRow );
            lcd.print( rowInfo );

            rowInfo = "";
        }

        nowDataIndex = i;
        return;
    }


    /**display previous statistic**/
    if ( nowDataIndex >= measureNum )
    {
        uint8_t number;
        Sensor *p_sensor;

        date = dataHouse.getDate(days);

        if ( date > 31 )
        {
            clrDisplayMgt();

            lcd.print( "  ** no data **" );
        }
        else
        {
            static int8_t strLeftIndex = 0;
            int8_t strRightIndex;

            if ( 0 == rowInfo.length())
            {
                const char * dataName;
                uint8_t hour;

                rowInfo = "";

                rowInfo += date;
                rowInfo +="th ";

                dataType = a_measureData[measureIndex].dataType;

                p_sensor = a_measureData[measureIndex].p_sensor;

                if ( NULL != p_sensor )
                {
                    dataName = p_sensor->getDataName(dataType);
                    rowInfo += dataName;
                    rowInfo += " ";

                    data = dataHouse.getDayAverage( dataType, days, &number );
                    rowInfo += number;
                    rowInfo += "data";
                    rowInfo += '\n';

                    if ( number > 0)
                    {
                        unitWord = p_sensor->formatValue( data, dataType );
                        rowInfo += " Avg:";
                        rowInfo += unitWord;
                        rowInfo += '\n';

                        data = dataHouse.getDayMin( dataType, days, &hour );
                        unitWord = p_sensor->formatValue( data, dataType );
                        rowInfo += " ";
                        rowInfo += unitWord;
                        rowInfo += " ";
                        rowInfo += hour;
                        rowInfo += ":00";
                        rowInfo += "\n";

                        data = dataHouse.getDayMax( dataType, days, &hour );
                        unitWord = p_sensor->formatValue( data, dataType );
                        rowInfo += " ";
                        rowInfo += unitWord;
                        rowInfo += " ";
                        rowInfo += hour;
                        rowInfo += ":00";
                        rowInfo += "\n";
                    }

                }

                strLeftIndex = 0;
            }

            do
            {
                strRightIndex = rowInfo.indexOf('\n', strLeftIndex);

                if ( strRightIndex < 0 )
                {
                    break;
                }

                lcd.setCursor( 0, lcdCurrentRow++ );
                lcd.print( rowInfo.substring( strLeftIndex, strRightIndex ) );
                displayMgt.statisticFlag = true;

                strLeftIndex = (strRightIndex + 1);

                if ( lcdCurrentRow >= LCD_ROW_NUM )
                {
                    break;
                }

            }while(1);


            if ( ((strRightIndex + 1) >= rowInfo.length()) || (strRightIndex < 0) )
            {
                rowInfo = "";

                measureIndex++;

                if ( measureIndex >= measureNum )
                {
                    days++;
                    measureIndex = 0;
                }

                if ( days >= DATA_SAVE_DAYS )
                {
                    clrDisplayMgt();
                }

            }


        }

    }

}

/**
Event:     action

RTC alarm:  sample data,  store data, set overtime
Button 1st:  sample data, display current data, set overtime
Button N: display N data, set overtime
Overtime: sleep

**/

void WeatherStation::processRTCAlarm()
{

    runHours++;

    /**eliminate disturbance**/
#if !_DEBUG
    setCurrentDate();

    if ( currentMinute <= 2 )
    {

#else
        currentHour++;
        currentHour = currentHour % 24;

        if ( 0 == currentHour )
        {
            currentDate++;
            currentDate = currentDate % 31;
        }

        DBG_PRINTLN_VAR(overTime,DEC);
#endif

        addAction(ACTION_SAMPLE_DATA);
        addAction(ACTION_SAVE_DATA);

#if !_DEBUG
    }
#endif


}

void WeatherStation::processButton()
{
    #define GOTOSLEEP_TIME 2 //s

    setCurrentDate();
    buttonPressHours = runHours;
    buttonPressMinute = currentMinute;

    if ( DISPLAY_STATE_INIT == displayMgt.state )
    {
        lcd.backlight();

        lcd.setCursor(0,0);
        lcd.print("Weather Station");  //16
        lcd.setCursor(0,1);
        lcd.print("*** DFRobot ***");  //16

        addAction(ACTION_SAMPLE_DATA);
    }

    addAction(ACTION_DISPLAY);


    overTime = SLEEP_OVERTIME;
    /**to avoid RTC interrupt when goto sleep **/
    if ( (59 == currentMinute) && ( (currentSecond + overTime + GOTOSLEEP_TIME) >= 60 ))
    {
        overTime += (GOTOSLEEP_TIME + 1);
    }


}

void WeatherStation::setCurrentDate()
{
    DateTime nowDate;

    nowDate = RTCObj.now();

    currentYear = nowDate.year();
    currentMonth = nowDate.month();
    currentDate = nowDate.date();
    currentHour = nowDate.hour();
    currentMinute = nowDate.minute();
    currentSecond = nowDate.second();
}


void WeatherStation::clrDisplayMgt()
{
    displayMgt.state = DISPLAY_STATE_INIT;
    displayMgt.dateShowFlag = false;
    displayMgt.nowDataIndex = 0;
    displayMgt.statisticFlag = false;
    displayMgt.days = 1;
    displayMgt.measureIndex = 0;
}

uint8_t WeatherStation::getActionState()
{
    uint8_t i;
    uint8_t countDoing, countDone;

    countDoing = 0;
    countDone = 0;

    for ( i = 0; i < ACTION_NUMBER; i++ )
    {
        if ( STATE_DOING == a_actionList[i].state )
        {
            countDoing++;
            break;
        }
        else if ( STATE_DONE == a_actionList[i].state )
        {
            countDone++;
        }
    }


    if ( countDoing > 0 )
    {
        return STATE_DOING;
    }
    else if ( countDone > 0 )
    {
        return STATE_DONE;
    }
    else
    {
        return STATE_INIT;
    }


}

void WeatherStation::disableBeforeSleep()
{
    volatile uint8_t *out, *ddr;
    uint8_t i;

    setCurrentDate();

    lcd.noBacklight();

    sensorMgt.stop();

    overTime = 0;

    delay(100);

    /**close ports**/
    for( i = 0; i < PALETTE_PORT_NUM; i++ )
    {
        ddr = portModeRegister(ioPort[i]);
        out = portOutputRegister(ioPort[i]);

        ioPreDDRValue[i] = *ddr;
        ioPrePORTValue[i] = *out;

        (*ddr) &= ioDisableMask[i];
        (*out) &= ioDisableMask[i];
    }


}


void WeatherStation::enableAfterSleep()
{
    uint8_t i;
    volatile uint8_t *out, *ddr;
    uint16_t diffMinutes;

    //enable power
    digitalWrite( PALETTE_PIN_MVCC, LOW );
    pinMode( PALETTE_PIN_MVCC, OUTPUT );
    delay(50);  //waiting power stable

    /**recovery ports**/
    for( i = 0; i < PALETTE_PORT_NUM; i++ )
    {
        ddr = portModeRegister(ioPort[i]);
        out = portOutputRegister(ioPort[i]);

        *ddr = ioPreDDRValue[i];
        *out = ioPrePORTValue[i];
    }


    sensorMgt.start();
    lcd.init();  //need sometimes

    if ( EVENT_STATE_SET == eventButtonState )
    {
        lcd.backlight();

        setCurrentDate();

        diffMinutes = (uint16_t)(runHours - buttonPressHours) * 60 + currentMinute - buttonPressMinute;

        if ( (!displayMgt.statisticFlag) || (diffMinutes >= RESTART_DISPLAY_OVERTIME) )
        {
            clrDisplayMgt();
        }
    }


    for ( i = 0; i < ACTION_NUMBER; i++ )
    {
        a_actionList[i].state = STATE_INIT;
    }

    startTime = (uint8_t)(millis()/1000);

}




void WeatherStation::sleep()         // here we put the arduino to sleep
{

    /* Now is the time to set the sleep mode. In the Atmega8 datasheet
     * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
     * there is a list of sleep modes which explains which clocks and
     * wake up sources are available in which sleep mode.
     *
     * In the avr/sleep.h file, the call names of these sleep modes are to be found:
     *
     * The 5 different modes are:
     *     SLEEP_MODE_IDLE         -the least power savings
     *     SLEEP_MODE_ADC
     *     SLEEP_MODE_PWR_SAVE
     *     SLEEP_MODE_STANDBY
     *     SLEEP_MODE_PWR_DOWN     -the most power savings
     *
     * For now, we want as much power savings as possible, so we
     * choose the according
     * sleep mode: SLEEP_MODE_PWR_DOWN
     *
     */
    if( SW_RUN == switchState )
    {
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here

        sleep_enable();          // enables the sleep bit in the mcucr register

                                 // so sleep is possible. just a safety pin
    }

    /* Now it is time to enable an interrupt. We do it here so an
     * accidentally pushed interrupt button doesn't interrupt
     * our running program. if you want to be able to run
     * interrupt code besides the sleep function, place it in
     * setup() for example.
     *
     * In the function call attachInterrupt(A, B, C)
     * A   can be either 0 or 1 for interrupts on pin 2 or 3.
     *
     * B   Name of a function you want to execute at interrupt for A.
     *
     * C   Trigger mode of the interrupt pin. can be:
     *             LOW        a low level triggers
     *             CHANGE     a change in level triggers
     *             RISING     a rising edge of a level triggers
     *             FALLING    a falling edge of a level triggers
     *
     * In all but the IDLE sleep modes only LOW can be used.
     */

    systemState = SYS_STATE_SLEEPING;

    if( SW_MAKE == switchState )
    {
        do
        {
            delay(200);
            DBG_PRINTLN_VAR(wakeFlag, DEC);
        }while(!wakeFlag);
    }
    else
    {
        do
        {
            sleep_mode();   // here the device is actually put to sleep!!
                        // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
            delay(1);  //if there is no delay, wakeFlag always false, why?
        }while(!wakeFlag);

        sleep_disable();         // first thing after waking from sleep:  disable sleep...

    }





}

void WeatherStation::setEvent( uint8_t pin )
{

    if ( SYS_STATE_GOTOSLEEP == systemState)
    {
        return;
    }

    wakeFlag = true;

    if ( (STAION_PIN_RTC_INT == pin) && (EVENT_STATE_DONE == eventRTCState) )
    {
        eventRTCState = EVENT_STATE_SET;
    }
    else if ( (STAION_PIN_BUTTON == pin) && (EVENT_STATE_DONE == eventButtonState) )
    {
        eventButtonState = EVENT_STATE_SET;
    }
    else
    {
        //nothing
    }

}



/*
 * attach an interrupt to a specific pin using pin change interrupts.
    pin =  arduino digital pin number
 */



void PCattachInterrupt(uint8_t pin) {
  uint8_t bit;
  uint8_t port;

  port = digitalPinToPort(pin);
  // map pin to PCIR register

  bit = digitalPinToBitMask(pin);

  switch (port)
  {
    case 2:  //PB
        PCMSK0 |= bit;
        PCICR |= 0x01 << 0;
        break;
    case 3:
        PCMSK1 |= bit;
        PCICR |= 0x01 << 1;
        break;
    case 4:
        PCMSK2 |= bit;
        PCICR |= 0x01 << 2;
        break;

    default:
        ;//nothing
  }

}



/***ISR thread***/
ISR(PCINT0_vect)
{
    if ( digitalRead( STAION_PIN_BUTTON ) == BUTTON_STATE_DOWN )
    {
        g_station.setEvent( STAION_PIN_BUTTON );
    }

    if ( digitalRead(STAION_PIN_RTC_INT) == LOW )
    {
        g_RTCIntFlag = true;
        g_station.setEvent( STAION_PIN_RTC_INT );
    }

}

void dettachUSB()
{

#ifdef UDCON
    DBG_END;

	UDCON = 1;							// disable attach resistor
    USBCON = 1<<FRZCLK;                 // Disable USB interface & Disable PLL
	PLLCSR = 0;		   				    // Disable PLL
	UHWCON = 0;                      //Disable USB pad regulator
#endif
}


#define ERROR_INDICATOR 13

void showErrInfo( String &info )
{
    uint8_t i;
    boolean state;

    pinMode( ERROR_INDICATOR, OUTPUT );

    Serial.begin(9600);

    state = HIGH;

    while(1)
    {
        for ( i = 0; i < 8; i++ )
        {
          digitalWrite( ERROR_INDICATOR, state );
          delay( 100 );
          state = !state;
        }

        if ( Serial )
        {
            Serial.println(info);
        }
    }
}


/*****/
uint32_t days2hours( uint16_t days, uint8_t hour )
{
    return (uint32_t)days * 24 + hour;
}

DataWarehouse::DataWarehouse()
{
    uint8_t i;

    for ( i = 0; i < DATA_TYPE_MAXNUM; i++ )
    {
        a_dataTypeIndex[i] = INVALID_UINT8;
    }
}

void DataWarehouse::init( uint16_t nowYear, uint8_t nowMonth, uint8_t nowDate, uint8_t nowHour )
{
    uint16_t year;
    uint8_t month;
    uint8_t date;

    uint8_t todayIndex;
    boolean valid;

#if _DEBUG
    for ( uint8_t i = 0; i < 20; i++ )
    {
        Serial.print(i);
        Serial.print(": ");
        Serial.println( eeprom_read_byte((const uint8_t *)i), HEX);
    }

#endif

    valid = true;

    if ( DATA_VALID == eeprom_read_byte((const uint8_t *)STORE_ADDR_VALIDFLAG ) )
    {
        todayIndex = eeprom_read_byte((const uint8_t *)STORE_ADDR_TODAYINDEX);

        if ( todayIndex < DATA_SAVE_DAYS )
        {
            uint32_t nowHours, storeHours;

            year = eeprom_read_word((const uint16_t *)STORE_ADDR_YEAR(todayIndex));
            month = eeprom_read_byte((const uint8_t *)STORE_ADDR_MONTH(todayIndex));
            date = eeprom_read_byte((const uint8_t *)STORE_ADDR_DAYDATE(todayIndex));

            storeHours = days2hours( date2days( year, month, date ), END_HOURTIME_ONEDAY);
            nowHours = days2hours( date2days( nowYear, nowMonth, nowDate ), nowHour);

            if ( (nowHours - storeHours) > 12 )
            {
                valid = false;
            }
        }
        else
        {
            valid = false;
        }

    }
    else
    {
        valid = false;
    }

    if ( !valid )
    {
        uint8_t i,j,k;

        eeprom_write_byte( (uint8_t *)STORE_ADDR_VALIDFLAG, ~DATA_VALID );

        for ( i = 0; i < DATA_SAVE_DAYS; i++ )
        {
            writeDateInfo( i, INVALID_UINT16, INVALID_UINT8, INVALID_UINT8);

            for ( j = 0; j < DATA_TYPE_MAXNUM; j++ )
            {
                for ( k = 0; k < SAMPLE_TIMES_PERDAY; k++ )
                {
    				writeData( INVALID_INT16, i, j, k );
                }

            }
        }

        todayIndex = 0;

        eeprom_write_byte((uint8_t *)STORE_ADDR_TODAYINDEX, todayIndex);
        writeDateInfo( todayIndex, nowYear, nowMonth, nowDate );   //record start date, only used internally

        eeprom_write_byte( (uint8_t *)STORE_ADDR_VALIDFLAG, DATA_VALID );


    }

#if _DEBUG
    for ( uint8_t i = 0; i < 20; i++ )
    {
        Serial.print(i);
        Serial.print(": ");
        Serial.println( eeprom_read_byte((const uint8_t *)i), HEX);
    }

#endif


}

#if _DEBUG
void DataWarehouse::printData( uint8_t days )
{
    uint8_t j,k;
    uint8_t index;

    index = (eeprom_read_byte((const uint8_t *)STORE_ADDR_TODAYINDEX) + DATA_SAVE_DAYS - days)%DATA_SAVE_DAYS;

    Serial.print("==Show data of index=");
    Serial.print(index);
    Serial.print(" date=");
    Serial.println(eeprom_read_byte((const uint8_t *)STORE_ADDR_DAYDATE(index)));



        for ( j = 0; j < DATA_TYPE_MAXNUM; j++ )
        {
            Serial.print(" Type ");
            Serial.print(a_dataTypeIndex[j]);
            Serial.print(":");

            for ( k = 0; k < SAMPLE_TIMES_PERDAY; k++ )
            {
                Serial.print(k);
                Serial.print("=");

                Serial.print(readData(index, j, k));
                Serial.print(" ");
            }

            Serial.println("");
        }

}

#endif

void DataWarehouse::addDataTypeIndex( uint8_t dataType)
{
    static uint8_t index = 0;

    DBG_ASSERT(index < DATA_TYPE_MAXNUM);

    a_dataTypeIndex[index] = dataType;
    index++;
}

void DataWarehouse::updateDate( uint16_t year, uint8_t month, uint8_t date )
{
    uint8_t j, k;
    uint8_t todayIndex;


    todayIndex = eeprom_read_byte( (const uint8_t *)STORE_ADDR_TODAYINDEX );

    eeprom_write_byte( (uint8_t *)STORE_ADDR_VALIDFLAG, ~DATA_VALID );
    writeDateInfo( todayIndex, year, month, date );   //record end date, display end date

    todayIndex = (todayIndex+1) % DATA_SAVE_DAYS;
    DBG_PRINTLN_VAR(todayIndex,DEC);

    eeprom_write_byte( (uint8_t *)STORE_ADDR_TODAYINDEX, todayIndex );
    writeDateInfo( todayIndex, year, month, date );   //record start date, only used internally

    for ( j = 0; j < DATA_TYPE_MAXNUM; j++ )
    {
        for ( k = 0; k < SAMPLE_TIMES_PERDAY; k++ )
        {
			writeData( INVALID_INT16, todayIndex, j, k );
        }
    }

    eeprom_write_byte( (uint8_t *)STORE_ADDR_VALIDFLAG, DATA_VALID );


#if _DEBUG
    printData(1);
#endif


}



int16_t DataWarehouse::readData( uint8_t dayIndex, uint8_t dataTypeIndex, uint8_t hourIndex)
{
    uint16_t addr;
	int16_t data;

	addr = STORE_ADDR_DATA(dayIndex, dataTypeIndex, hourIndex);

	data = (int16_t)eeprom_read_word((const uint16_t *)addr);

	return data;
}

void DataWarehouse::writeData( int16_t data, uint8_t dayIndex, uint8_t dataTypeIndex, uint8_t hourIndex)
{
    uint16_t addr;

	addr = STORE_ADDR_DATA(dayIndex, dataTypeIndex, hourIndex);

	eeprom_write_word((uint16_t *)addr, (uint16_t)data);

#if 0
    DBG_PRINTLN_VAR(dayIndex,DEC);
    DBG_PRINTLN_VAR(dataTypeIndex,DEC);
    DBG_PRINTLN_VAR(hourIndex,DEC);
    Serial.print("-Addr=");
    Serial.print(addr);
    Serial.print(" data=");
	Serial.println((int16_t)eeprom_read_word((uint16_t *)addr));
#endif
}


void DataWarehouse::writeDateInfo( uint8_t index, uint16_t year, uint8_t month, uint8_t date )
{
    eeprom_write_word( (uint16_t *)STORE_ADDR_YEAR(index), year );
    eeprom_write_byte( (uint8_t *)STORE_ADDR_MONTH(index), month );
    eeprom_write_byte( (uint8_t *)STORE_ADDR_DAYDATE(index), date );
}

void DataWarehouse::putData( int16_t data, uint8_t dataType, uint8_t hour )
{
    uint8_t todayIndex;
    uint8_t dataTypeIndex;
    uint8_t hourIndex;

    todayIndex = eeprom_read_byte((const uint8_t *)STORE_ADDR_TODAYINDEX);
    dataTypeIndex = getDataTypeIndex(dataType);
    hourIndex = hour2Index(hour);


    eeprom_write_byte( (uint8_t *)STORE_ADDR_VALIDFLAG, ~DATA_VALID );
	writeData( data, todayIndex, dataTypeIndex, hourIndex );
    eeprom_write_byte( (uint8_t *)STORE_ADDR_VALIDFLAG, DATA_VALID );


    DBG_PRINT("putData:")
    DBG_PRINTLN_VAR(dataType, DEC);
    DBG_PRINTLN_VAR(data, DEC);
}

//todo,  count
int16_t DataWarehouse::getDayAverage( uint8_t dataType, uint8_t daysBeforeToday, uint8_t *p_number)
{

    uint8_t dayIndex, hourIndex;
    uint8_t dataTypeIndex;
    uint8_t count;
    int16_t data;
    int32_t sum;

    dayIndex = ((eeprom_read_byte((const uint8_t *)STORE_ADDR_TODAYINDEX) + DATA_SAVE_DAYS) - daysBeforeToday) % DATA_SAVE_DAYS;
    dataTypeIndex = getDataTypeIndex(dataType);
    count = 0;
    sum = 0;

    for ( hourIndex = 0; hourIndex < SAMPLE_TIMES_PERDAY; hourIndex++ )
    {
        data = readData( dayIndex, dataTypeIndex, hourIndex);

        if ( INVALID_INT16 != data )
        {
            sum += data;
            count++;
        }
    }


    DBG_PRINTLN_VAR(sum,DEC);
    DBG_PRINTLN_VAR(count,DEC);

    if ( NULL != p_number )
    {
        *p_number = count;
    }

    return ( 0 == count ) ? INVALID_INT16 : (sum/count);

}

int16_t DataWarehouse::getDayMax( uint8_t dataType, uint8_t daysBeforeToday, uint8_t *p_hour)
{
    uint8_t dayIndex, hourIndex;
    uint8_t dataTypeIndex;
    uint8_t maxHourIndex;
    int16_t data;
    int16_t max;
    boolean firstFlag;

    dayIndex = ((eeprom_read_byte((const uint8_t *)STORE_ADDR_TODAYINDEX) + DATA_SAVE_DAYS) - daysBeforeToday) % DATA_SAVE_DAYS;
    dataTypeIndex = getDataTypeIndex(dataType);
    firstFlag = false;

    for ( hourIndex = 0; hourIndex < SAMPLE_TIMES_PERDAY; hourIndex++ )
    {
        data = readData( dayIndex, dataTypeIndex, hourIndex);

        if ( INVALID_INT16 != data )
        {
            if ( (data > max) || (!firstFlag) )
            {
                max = data;
                maxHourIndex = hourIndex;
            }

            firstFlag = true;
        }
    }

    if ( p_hour != NULL )
    {
        *p_hour = index2Hour(maxHourIndex);
    }

    return ( firstFlag ) ? max : INVALID_INT16;

}

int16_t DataWarehouse::getDayMin( uint8_t dataType, uint8_t daysBeforeToday, uint8_t *p_hour)
{
    uint8_t dayIndex, hourIndex;
    uint8_t dataTypeIndex;
    boolean firstFlag;
    uint8_t minHourIndex;
    int16_t data;
    int16_t min;

    dayIndex = ((eeprom_read_byte((const uint8_t *)STORE_ADDR_TODAYINDEX) + DATA_SAVE_DAYS) - daysBeforeToday) % DATA_SAVE_DAYS;
    dataTypeIndex = getDataTypeIndex(dataType);
    firstFlag = false;

    for ( hourIndex = 0; hourIndex < SAMPLE_TIMES_PERDAY; hourIndex++ )
    {
        data = readData( dayIndex, dataTypeIndex, hourIndex);

        if ( INVALID_INT16 != data )
        {
            if ( (data < min) || (!firstFlag) )
            {
                minHourIndex = hourIndex;
                min = data;
            }

            firstFlag = true;
        }
    }

    if ( p_hour != NULL )
    {
        *p_hour = index2Hour(minHourIndex);
    }

    return ( firstFlag ) ? min : INVALID_INT16;

}

uint8_t DataWarehouse::getDate( int8_t daysBeforeToday )
{
    uint8_t dayIndex;

    dayIndex = ((eeprom_read_byte((const uint8_t *)STORE_ADDR_TODAYINDEX) + DATA_SAVE_DAYS) - daysBeforeToday) % DATA_SAVE_DAYS;

    return eeprom_read_byte((const uint8_t *)STORE_ADDR_DAYDATE(dayIndex));

}


uint8_t DataWarehouse::getDataTypeIndex( uint8_t dataType )
{
    uint8_t i, index;

    index = INVALID_UINT8;

    for ( i = 0; i < DATA_TYPE_MAXNUM; i++ )
    {
        if ( INVALID_UINT8 == a_dataTypeIndex[i] )
        {
            break;
        }
        else
        {
            if ( dataType == a_dataTypeIndex[i] )
            {
                index = i;
            }
        }
    }

    DBG_ASSERT(index < DATA_TYPE_MAXNUM);

    return index;

}


/**Sensor**/
Sensor::Sensor()
{
    runEnable = false;
    objectNum = 0;
}

Sensor* Sensor::creatObject( uint8_t ID, uint8_t pin, uint8_t auxiliaryPin)
{
    Sensor *pObject = NULL;


    /**search**/
    pObject = getObject( pin );

    if ( pObject )
    {
        return pObject;
    }

    if ( objectNum >= SENSOR_MAXNUM )
    {
        return NULL;
    }

    /**creat **/

    switch ( ID )
    {
        case FUNCTION_SENSOR_DHT22:
            pObject = new DHT22Sensor( pin );
            break;

        case FUNCTION_SENSOR_BMP085:
            pObject = new BMP085Sensor( pin );
            break;

        case FUNCTION_SENSOR_GP2Y1010:
            pObject = new GP2Y1010Sensor( pin, auxiliaryPin );
            break;

        default:
           ; //nothing

    }


    /**store **/

    if ( pObject )
    {
        rigisterObject( pin, pObject );
        objectNum++;  //must be increase after Rigister
    }

    return pObject;
}


Sensor* Sensor::getObject( uint8_t pin )
{
    uint8_t i;
    Sensor *pObject = NULL;


    for ( i = 0; i < objectNum ; i++ )
    {
        if ( pin == objectList[i].attachPin )
        {
            pObject = objectList[i].pObject;
            break;
        }
    }

    return pObject;

}


void Sensor::rigisterObject( uint8_t pin, Sensor *pObject )
{

    if ( objectNum >= SENSOR_MAXNUM )
    {
        return;
    }

    objectList[objectNum].attachPin = pin;
    objectList[objectNum].pObject = pObject;
}

void Sensor::start()
{
    uint8_t i;
    Sensor *pObject = NULL;


    for ( i = 0; i < objectNum ; i++ )
    {
        pObject = objectList[i].pObject;
        if ( NULL != pObject )
        {
            pObject->start();
        }
    }

    runEnable = true;

}


boolean Sensor::run()
{
    uint8_t i;
    boolean ret;

    Sensor *pObject = NULL;

    ret = true;

    if ( !runEnable )
    {
        return false;
    }

    for ( i = 0; i < objectNum ; i++ )
    {
        //todo,  add balance
        pObject = objectList[i].pObject;
        if ( NULL != pObject )
        {
            if ( false == pObject->run())
            {
                ret = false;
            }
        }
    }

    return ret;


}


void Sensor::stop()
{
    uint8_t i;
    Sensor *pObject = NULL;

    //runEnable = false;

    for ( i = 0; i < objectNum ; i++ )
    {
        //todo,  add balance
        pObject = objectList[i].pObject;
        if ( NULL != pObject )
        {
            pObject->stop();
        }
    }
}


DHT22Sensor::DHT22Sensor(uint8_t pin)
{
    attachPin = pin;
    pDriver = new dht;
}

void DHT22Sensor::start( )
{
    temperature = INVALID_INT16;
    humidity = INVALID_INT16;
}

void DHT22Sensor::stop( )
{

}

boolean DHT22Sensor::run()
{
    int16_t ret;


    execute(); //read data is the last data, so execute twice.

    if ( execute())
    {
        temperature = (int16_t)(pDriver->temperature*10);
        humidity = (int16_t)(pDriver->humidity*10);
        ret = true;
    }
    else
    {
        temperature = INVALID_INT16;
        humidity = INVALID_INT16;
        ret = false;

    }

    return true;
}

#define DHT_TRY_NUM 25   //must < 255
boolean DHT22Sensor::execute()
{
    int16_t ret;
    uint8_t i;

    for ( i = 0; i < DHT_TRY_NUM; i++ )
    {
        ret = (int8_t)pDriver->read22( attachPin );
        if ( DHTLIB_OK == ret )
        {
            return true;
        }

        delay(100);
    }

    DBG_PRINT("DHT sample: Errcode=");
    DBG_PRINTLN(ret);

    return false;
}



int16_t DHT22Sensor::getValue( uint8_t dataType )
{
    int16_t val;

    if ( DATA_TEMPERATURE == dataType )
    {
        val = temperature;
    }
    else if ( DATA_HUMIDITY == dataType )
    {
        val = humidity;
    }
    else
    {
        val = INVALID_INT16;
        DBG_WARNING(0);
    }

    return val;


}


String DHT22Sensor::formatValue( int16_t value, uint8_t dataType )
{
    String info;

    if ( INVALID_INT16 == value )
    {
        info = "*";
    }
    else
    {
        info = "";
        info += value/10;
        info += ".";
        info += value%10;
    }

    if ( DATA_TEMPERATURE == dataType )
    {
        info += "C";
    }
    else if ( DATA_HUMIDITY == dataType )
    {
        info += "%";
    }
    else
    {
        info += "?";
        DBG_WARNING(0);
    }

    return info;

}

const char * DHT22Sensor::getDataName( uint8_t dataType )
{
    const char *name;

    if ( DATA_TEMPERATURE == dataType )
    {
         name = "TEMP";
    }
    else if ( DATA_HUMIDITY == dataType )
    {
        name = "RH";
    }
    else
    {
        name = "?";
        DBG_WARNING(0);
    }

    return name;
}

/****/
BMP085Sensor::BMP085Sensor(uint8_t pin)
{
    attachPin = pin;
    pDriver = new BMP085();
}

void BMP085Sensor::start( )
{
    Wire.begin();
    pDriver->init();

    pressureValue = INT16_MAX;
}

void BMP085Sensor::stop( )
{

}

boolean BMP085Sensor::run()
{
    int32_t value;


    pDriver->getPressure( &value );

    value = (value + 50)/100;  //Pa -> hPa

    pressureValue = ( value > INT16_MAX ) ? INVALID_INT16 : (int16_t)value;

    return true;
}

int16_t BMP085Sensor::getValue( uint8_t dataType )
{
    return pressureValue;
}


String BMP085Sensor::formatValue( int16_t value, uint8_t dataType )
{

    String info;

    if ( INVALID_INT16 == value )
    {
        info = "*";
    }
    else
    {
        info = "";
        info += value;
    }

    info += "hPa";

    return ( info );
}

const char * BMP085Sensor::getDataName( uint8_t dataType )
{
    return "BARO";
}


/****/
GP2Y1010Sensor::GP2Y1010Sensor(uint8_t voutPin, uint8_t ledPin)
{
    attachPin = voutPin;
    auxiliaryPin = ledPin;
}

void GP2Y1010Sensor::start( )
{
    dustDensity = INT16_MAX;

    pinMode(auxiliaryPin, OUTPUT);
    digitalWrite(auxiliaryPin,HIGH);
}

void GP2Y1010Sensor::stop( )
{
    digitalWrite(auxiliaryPin,LOW);
    pinMode(auxiliaryPin, INPUT);
}

boolean GP2Y1010Sensor::run()
{
    uint16_t value;

    digitalWrite(auxiliaryPin,LOW); // power on the LED
    delayMicroseconds(GP2Y1010_SAMPLE_TIME);

    value = (uint16_t)analogRead(attachPin); // read the dust value

    digitalWrite(auxiliaryPin,HIGH); // power on the LED

    /**  **/
    dustDensity = 0.83*(float)value - 100;

    if ( dustDensity < 0 )
    {
        dustDensity = 0;
    }
    else if ( dustDensity > 520 )
    {
        dustDensity = 520;
    }


    DBG_PRINT("Dust raw value=");
    DBG_PRINTLN(value);

    return true;
}

int16_t GP2Y1010Sensor::getValue( uint8_t dataType )
{
    return dustDensity;
}


String GP2Y1010Sensor::formatValue( int16_t value, uint8_t dataType )
{

    String info;

    if ( INVALID_INT16 == value )
    {
        info = "*";
    }
    else
    {
        info = "";
        info += value;
    }

    info += "ug/m3";

    return ( info );
}

const char * GP2Y1010Sensor::getDataName( uint8_t dataType )
{
    return "PM10";
}
