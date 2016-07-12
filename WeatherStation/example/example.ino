/***************************************************
  Example of Weather Station Kit from DFRobot
 <http://www.dfrobot.com/>
  
 ***************************************************
 This example  reads data from sensor and temperature and humidity from SHT1x Humidity and Temperature Sensor.

20:00 is the end of a day
Gather data every hour.
Show

 
  
 Created 2014-8
 By Leo Yan <leo.yan@dfrobot.com>
  
 GNU Lesser General Public License.
 See <http://www.gnu.org/licenses/> for details.
 All above must be included in any redistribution
 ****************************************************/
 
/***********Notice and Trouble shooting***************
 ****************************************************/


#include "public.h"

#include "dht.h"
#include <Wire.h>
#include <BMP085.h>
#include "DS3231.h"
#include <LiquidCrystal_I2C.h>

#include "WeatherStation.h"

/**Config sensors of weather station**/
DataConfig a_dataConfigList[]=
{
    /* Date Type     ,    Sensor Type       , Pin   */
    {DATA_TEMPERATURE, FUNCTION_SENSOR_DHT22, 9},           //config temperature sensor
    {DATA_HUMIDITY, FUNCTION_SENSOR_DHT22, 9},              //config Humidity sensor
    {DATA_AIR_PRESSURE, FUNCTION_SENSOR_BMP085, PIN_I2C},   //config barmeter sensor
    //{DATA_PM10, FUNCTION_SENSOR_GP2Y1010, A0, A1},            //config dust sensor. if you add dust sensor,  remove the comment '//'  at the start of the line
};


/**main program**/
void setup() 
{                
    DBG_BEGIN(9600);
    DBG_PRINTLN("init");
    g_station.init(a_dataConfigList, sizeof(a_dataConfigList)/sizeof(DataConfig)); 
}

void loop() 
{   
    g_station.run();
}




