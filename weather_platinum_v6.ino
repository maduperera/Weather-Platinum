#include <MyWireLib.h>
//#include <SFE_BMP180.h>

/********** constants *****************/
#define S0 0
#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5
#define PRELOAD 0xFF06 // = 65536 – 65286 = 250 decimal gives is 1000 interrupts per second
#define NUM_OF_SENSORS 3 // defines the number of sensors
#define ACCURACY_TOLERANCE 2 // as defined in the data sheet 
#define TEMP_MIN 0.0 // min temperature in ºC can be measured as defined in the data sheet 
#define TEMP_MAX 65 // max tempurature in ºC  can be measured as defined in the data sheet
#define PRESSURE_MIN 500.00 // min pressure in Pa can be measured as defined in the data sheet 
#define PRESSURE_MAX 1500.00 // max pressure in Pa can be measured as defined in the data sheet



/********** constants *****************/
const uint8_t oss = 3;  // Set oversampling setting
const uint8_t osd = 26; // with corresponding oversampling delay 


/********** variables *****************/
MyWireLib Sens[3];
int state1 = S0, state2 = S0, state3 = S0;

int16_t  ac1[3], ac2[3], ac3[3], b1[3], b2[3], mb[3], mc[3], md[3]; // Store sensor PROM values from BMP180
uint16_t ac4[3], ac5[3], ac6[3];  // Store sensor PROM values from BMP180

 
float temp; 
int32_t b5[3];
unsigned int count = 0, timeStamp = 0;  // counting interrupts

// for validation & measurements
float t_min = 0.0;
float t_max = 0.0;
float t_avg = 0.0;

float T[NUM_OF_SENSORS]; // measured temperature values
float P[NUM_OF_SENSORS]; // measured pressure values
int SensorHealthOf[NUM_OF_SENSORS]; // sensor health S[index] -> {0,1} where 0 represents a fault and 1 represents good 

unsigned long previous_time_for_sensor_1_temperature;
unsigned long previous_time_for_sensor_2_temperature;
unsigned long previous_time_for_sensor_3_temperature;
unsigned long previous_time_for_sensor_1_pressure;
unsigned long previous_time_for_sensor_2_pressure;
unsigned long previous_time_for_sensor_3_pressure;
float previous_temp_sensor_1 = 0.0;
float previous_temp_sensor_2 = 0.0;
float previous_temp_sensor_3 = 0.0; 
float previous_pressure_sensor_1 = 0.0;
float previous_pressure_sensor_2 = 0.0;
float previous_pressure_sensor_3 = 0.0; 

float dT[NUM_OF_SENSORS]; // array to hold sensor temperature rates
float dP[NUM_OF_SENSORS]; // array to hold sensor pressure rates



 
ISR(TIMER1_OVF_vect) {
  
    TCNT1=PRELOAD; 
    count++; 
    
    if(count>=1000) {
        count=0;

        
        Serial.println("=========================== Weather Platinum V6.0 =============================================");

        checkSensorHealthForTempurature(T);
        float averageTemperature = getAverageTemp(SensorHealthOf, T);
        float temperatureAccuracy = getTempAccuracy(SensorHealthOf, T, averageTemperature);

        checkSensorHealthForPressure(P);
        float averagePressure = getAveragePressure(SensorHealthOf, P);
        float pressureAccuracy = getPressureAccuracy(SensorHealthOf, P, averagePressure);

        
        Serial.print("\n\n");
        
        Serial.println("------------------------------------------------------------------");
        Serial.println("   T1   |   T2   |   T3   |     P1     |     P2     |     P3     |");
        Serial.println("        |        |        |            |            |            |");
        Serial.print(" ");
        Serial.print(T[0], 3);  
        Serial.print(" ");
        Serial.print("|");
        Serial.print(" ");
        //Serial.print(" ");
        Serial.print(T[1], 3);  
        Serial.print(" ");
        Serial.print("|");
        Serial.print(" ");
        //Serial.print(" ");
        Serial.print(T[2], 3);  
        Serial.print(" ");
        Serial.print("|");
        Serial.print(" ");
        Serial.print("  ");
        Serial.print(P[0], 3);  
        Serial.print("  ");
        Serial.print("|");
        Serial.print("  ");
        //Serial.print(" ");
        Serial.print(P[1], 3);  
        Serial.print("   ");
        Serial.print("|");
        Serial.print("   ");
        //Serial.print("   ");
        Serial.print(P[2], 3);  
        Serial.print("  ");
        Serial.print("|");
        Serial.println("   ");
        Serial.println("        |        |        |            |            |            |"); 
        Serial.println("------------------------------------------------------------------");
        


        
        Serial.print("Average Temperature = ");
        Serial.print(averageTemperature);
        Serial.println(" ºC \n");
            
        Serial.print("Temperature Accuracy = ");
        Serial.print(temperatureAccuracy);
        Serial.println(" % \n");


        Serial.print("Temperature rate of sensor 1 = ");
        Serial.print(dT[0],10);
        Serial.println(" ºC/s \n");

        Serial.print("Temperature rate of sensor 2 = ");
        Serial.print(dT[1],10);
        Serial.println(" ºC/s \n");

        Serial.print("Temperature rate of sensor 3 = ");
        Serial.print(dT[2],10);
        Serial.println(" ºC/s \n");


        Serial.println("********************************************************************");


        Serial.print("Average Pressure = ");
        Serial.print(averagePressure);
        Serial.println(" mbar \n");
            
        Serial.print("Pressure Accuracy = ");
        Serial.print(pressureAccuracy);
        Serial.println(" % \n");

        Serial.print("Pressure rate of sensor 1 = ");
        Serial.print(dP[0],10);
        Serial.println(" mbar/s \n");

        Serial.print("Pressure rate of sensor 2 = ");
        Serial.print(dP[1],10);
        Serial.println(" mbar/s \n");

        Serial.print("Pressure rate of sensor 3 = ");
        Serial.print(dP[2],10);
        Serial.println(" mbar/s \n");
        
        Serial.println("========================================== END ================================================\n\n\n\n\n");
        
    }
    
 }

void setup() 
{ 

//Serial.begin(9600);

      // (as we do not know the initial  values)
      cli();         // disable global interrupts
      TCCR1A = 0;    // set entire TCCR1A register to 0
      TCCR1B = 0;    // set entire TCCR1B register to 0 
                   
      // enable Timer1 overflow interrupt:
      TIMSK1 |= (1 << TOIE1); //Atmega8 has no TIMSK1 but a TIMSK register
      // Preload 
      TCNT1= PRELOAD;
      TCCR1B |= (1 << CS11); // Sets bit CS11 in TCCR1B
      TCCR1B |= (1 << CS10); // and CS10
      /*the clock source is divided by 64, i.e. one clock cycle every 64 / (16 * 10^6) = 4 * 10^(-6) = 0.000004s
      // This is achieved by shifting binary 1 (0b00000001)
      // to the left by CS11 or CS10 bits. This is then bitwise
      // OR-ed into the current value of TCCR1B, which effectively set
      // this one bit high.
      // enable global interrupts:
      */
      
      Serial.begin(9600);
      //declare pins
      Sens[0].SCLpinI=2;
      Sens[0].SDApinI=3;
      Sens[1].SCLpinI=4;
      Sens[1].SDApinI=5;
      Sens[2].SCLpinI=6;
      Sens[2].SDApinI=7;
      Sens[0].Soss=oss;
      Sens[1].Soss=oss;
      Sens[2].Soss=oss;
      
      //> for test
      //Serial.print("SCLPIN for sensor 0: ");Serial.println(Sens[0].SCLpinI);
      //Serial.print("SDAPIN for sensor 0: ");Serial.println(Sens[0].SDApinI);
      //Serial.print("SCLPIN for sensor 1: ");Serial.println(Sens[1].SCLpinI);
      //Serial.print("SDAPIN for sensor 1: ");Serial.println(Sens[1].SDApinI);
      //Serial.print("SCLPIN for sensor 2: ");Serial.println(Sens[2].SCLpinI);
      //Serial.print("SDAPIN for sensor 2: ");Serial.println(Sens[2].SDApinI);
      //< for test
       delay(500);
       
       Sens[0].InitWire();
       Sens[1].InitWire();
       Sens[2].InitWire();
       
       init_SENSOR(0); 
       init_SENSOR(1);
       init_SENSOR(2); 
       sei();
       
}

void loop() 
{
  
    switch(state1) {
  
        case S0:   
                  temperatureInit(0); 
                  timeStamp = count; 
                  state1 = S1; 
                  break;
        case S1:  
                  if(timeStamp + 5 <= count) {
                    state1 = S2;
                    break;
                  }
                  else {
                      state1 = S1;
                      break;
                  }
        case S2: 
                  T[0] = getTemperature(0); 
                  dT[0] = ( T[0] - previous_temp_sensor_1 ) / ( ( millis() - previous_time_for_sensor_1_temperature ) * 1000 );
                  previous_time_for_sensor_1_temperature = millis();
                  previous_temp_sensor_1 = T[0];
                  state1 = S3; 
                  break;
        case S3: 
                  pressureInit(0); 
                  timeStamp = count; 
                  state1 = S4; 
                  break;
        case S4:  
                  if(timeStamp + 26 <= count) {
                    state1 = S5;
                    break;
                  }
                  else {
                    state1 = S4;
                    break;
                  }   
        case S5: 
                  //Serial.print("P[0] : ");
                  //Serial.println(P[0]);

                  //Serial.print("previous_pressure_sensor_1 : ");
                  //Serial.println(previous_pressure_sensor_1);

                  //Serial.print("dt : ");
                  //Serial.println(millis() - previous_time_for_sensor_1_pressure );
                  
                  P[0] = getPressure(b5[0],0);
                  dP[0] = ( P[0] - previous_pressure_sensor_1 ) / ( ( millis() - previous_time_for_sensor_1_pressure ) * 1000 );
                  previous_time_for_sensor_1_pressure = millis();
                  previous_pressure_sensor_1 = P[0];
                  //P[0] = getPressure(b5[0],0); 
                  state1 = S0; 
                  break;
    }

    switch(state2) {
        
        case S0: 
                  temperatureInit(1); 
                  timeStamp = count; 
                  state2 = S1; 
                  break;
        case S1: 
                  if(timeStamp + 5 <= count) {
                      state2 = S2;
                      break;
                  }
                  else {
                      state2 = S1;
                      break;
                  }
        case S2: 
                  T[1] = getTemperature(1); 
                  dT[1] = ( T[1] - previous_temp_sensor_2 ) / ( ( millis() - previous_time_for_sensor_2_temperature ) * 1000 );
                  previous_time_for_sensor_2_temperature = millis();
                  previous_temp_sensor_2 = T[1];
                  //T[1] = getTemperature(1); 
                  state2 = S3; 
                  break;
        case S3: 
                  pressureInit(1); 
                  timeStamp = count; 
                  state2 = S4; 
                  break;
        case S4: 
                  if(timeStamp + 26 <= count) {
                      state2 = S5;
                      break;
                  }
                  else {
                      state2 = S4;
                      break;
                  }   
        case S5: 
                  //Serial.print("P[1] : ");
                  //Serial.println(P[1]);

                  //Serial.print("previous_pressure_sensor_2 : ");
                  //Serial.println(previous_pressure_sensor_2);

                  //Serial.print("dt : ");
                  //Serial.println(millis() - previous_time_for_sensor_2_pressure );
                  
                  P[1] = getPressure(b5[1],1);
                  dP[1] = ( P[1] - previous_pressure_sensor_2 ) / ( ( millis() - previous_time_for_sensor_2_pressure ) * 1000 );
                  previous_time_for_sensor_2_pressure = millis();
                  previous_pressure_sensor_2 = P[1];
                  //P[1] = getPressure(b5[1],1); 
                  state2 = S0; 
                  break;
    
    }

    switch(state3) {
    
        case S0: 
                  temperatureInit(2); 
                  timeStamp = count; 
                  state3 = S1; 
                  break;
        case S1: 
                if(timeStamp+5<=count) {
                    state3=S2;
                    break;
                }
                else {
                    state3=S1;
                    break;
                }
        case S2: 
                  T[2] = getTemperature(2); 
                  dT[2] = ( T[2] - previous_temp_sensor_3 ) / ( ( millis() - previous_time_for_sensor_3_temperature ) * 1000 );
                  previous_time_for_sensor_3_temperature = millis();
                  previous_temp_sensor_3 = T[2];
                  //T[2]= getTemperature(2); 
                  state3 = S3; 
                  break;
        case S3: 
                  pressureInit(2); 
                  timeStamp = count; 
                  state3 = S4; 
                  break;
        case S4: 
                  if(timeStamp+26<=count) {
                      state3=S5;
                      break;
                  }
                  else {
                      state3=S4;
                      break;
                  }   
        case S5: 
                  //Serial.print("P[2] : ");
                  //Serial.println(P[2]);

                  //Serial.print("previous_pressure_sensor_3 : ");
                  //Serial.println(previous_pressure_sensor_3);

                  //Serial.print("dt : ");
                  //Serial.println(millis() - previous_time_for_sensor_3_pressure );
                  
                  P[2] = getPressure(b5[2],2);
                  dP[2] = ( P[2] - previous_pressure_sensor_3 ) / ( ( millis() - previous_time_for_sensor_3_pressure ) * 1000 );
                  previous_time_for_sensor_3_pressure = millis();
                  previous_pressure_sensor_3 = P[2];
                  //P[2] = getPressure(b5[2],2); 
                  state3 = S0; 
                  break;
    }
    
}

void init_SENSOR(int sensnr)
{
    ac1[sensnr] = Sens[sensnr].Get16bitFromRegister(0xAA);
    ac2[sensnr] = Sens[sensnr].Get16bitFromRegister(0xAC);
    ac3[sensnr] = Sens[sensnr].Get16bitFromRegister(0xAE);
    ac4[sensnr] = Sens[sensnr].Get16bitFromRegister(0xB0);
    ac5[sensnr] = Sens[sensnr].Get16bitFromRegister(0xB2);
    ac6[sensnr] = Sens[sensnr].Get16bitFromRegister(0xB4);
    b1[sensnr]  = Sens[sensnr].Get16bitFromRegister(0xB6);
    b2[sensnr]  = Sens[sensnr].Get16bitFromRegister(0xB8);
    mb[sensnr]  = Sens[sensnr].Get16bitFromRegister(0xBA);
    mc[sensnr]  = Sens[sensnr].Get16bitFromRegister(0xBC);
    md[sensnr]  = Sens[sensnr].Get16bitFromRegister(0xBE);
    
    Serial.println("");
    Serial.print("Sensor ");  Serial.print(sensnr);  Serial.println(" calibration data:");
    Serial.print(F("AC1 = ")); Serial.println(ac1[sensnr]);
    Serial.print(F("AC2 = ")); Serial.println(ac2[sensnr]);
    Serial.print(F("AC3 = ")); Serial.println(ac3[sensnr]);
    Serial.print(F("AC4 = ")); Serial.println(ac4[sensnr]);
    Serial.print(F("AC5 = ")); Serial.println(ac5[sensnr]);
    Serial.print(F("AC6 = ")); Serial.println(ac6[sensnr]);
    Serial.print(F("B1 = "));  Serial.println(b1[sensnr]);
    Serial.print(F("B2 = "));  Serial.println(b2[sensnr]);
    Serial.print(F("MB = "));  Serial.println(mb[sensnr]);
    Serial.print(F("MC = "));  Serial.println(mc[sensnr]);
    Serial.print(F("MD = "));  Serial.println(md[sensnr]);
    Serial.println("");
}


/**********************************************
 
  Calcualte pressure readings

***********************************************/
float getPressure(int32_t b5, int sensnr)
{
    int32_t x1, x2, x3, b3, b6, p, UP;
    uint32_t b4, b7; 
    UP = Sens[sensnr].Get24bitFromRegister(0xf6);
    
    b6 = b5 - 4000;
    x1 = (b2[sensnr] * (b6 * b6 >> 12)) >> 11; 
    x2 = ac2[sensnr] * b6 >> 11;
    x3 = x1 + x2;
    b3 = (((ac1[sensnr] * 4 + x3) << oss) + 2) >> 2;
    x1 = ac3[sensnr] * b6 >> 13;
    x2 = (b1[sensnr] * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4[sensnr] * (uint32_t)(x3 + 32768)) >> 15;
    b7 = ((uint32_t)UP - b3) * (50000 >> oss);
    if(b7 < 0x80000000) {
        p = (b7 << 1) / b4; 
    } else {
        p = (b7 / b4) << 1; 
    } // or p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
      
    return (p + ((x1 + x2 + 3791) >> 4)) / 100.0f; // Return pressure in mbar
}


/**********************************************
  
  Read uncompensated temperature

************************************************/
void temperatureInit(int sensnr)
{
    byte tobesend[2] = {0xf4, 0x2e};
    Sens[sensnr].sendbytes(tobesend,2);
}



int32_t getTemperature(int sensnr)
{
    int32_t x1, x2, b5, UT;
    UT = Sens[sensnr].Get16bitFromRegister(0xf6);
    // Calculate true temperature
    x1 = (UT - (int32_t)ac6[sensnr]) * (int32_t)ac5[sensnr] >> 15;
    x2 = ((int32_t)mc[sensnr] << 11) / (x1 + (int32_t)md[sensnr]);
    
    //Serial.println("x1");
    //Serial.println(x1);
    //Serial.println("x2");
    //Serial.println(x2);
    
    b5 = x1 + x2;
    temp  = (b5 + 8) >> 4;
    temp = temp / 10.0; // Temperature in celsius 
  
    //Serial.println("temp");
    //Serial.println(temp);
    
    return temp;  
}



/**********************************************
  
  Read uncompensated pressure value
 
************************************************/
void pressureInit(int sensnr)
{
    byte tobesend[3] = {0xf4, (0x34 + (oss << 6))};
    Sens[sensnr].sendbytes(tobesend,3);
}



/********************************************************* Temperature Calculations *******************************************/


/**********************************************
  
  Calculate Average Temperature
  
************************************************/
float getAverageTemp(int SensorHealthOf[], float TemperatureOf[]){

    float t_sum = 0.0; 
    int s_sum = 0;

    for(int i = 0; i < NUM_OF_SENSORS; i++){
        t_sum += TemperatureOf[i] * SensorHealthOf[i];
        s_sum += SensorHealthOf[i];
      
    }

    return (t_sum / s_sum);

}


/**********************************************
 
  Calculate Temperature Accuracy

***********************************************/
float getTempAccuracy(int SensorHealthOf[], float TemperatureOf[], float average_temp){

    float accuracy = 0.0;

    for(int i = 0; i < NUM_OF_SENSORS; i++){

        accuracy += ( 1  - (abs(TemperatureOf[i] - average_temp)/average_temp) ) * ( 100 / NUM_OF_SENSORS ) * SensorHealthOf[i];
      
    }

    return accuracy;

}


/*********************************************************************************************************

  Validate Sensors with respect to Tempurature readings
  This will evaluate the sensor health. S[index] -> {0,1} where 0 represents a fault and 1 represents good 

 **********************************************************************************************************/
void checkSensorHealthForTempurature(float TemperatureOf[]){

    makeAllSensorsToDefaultState(); // clear and re-initialize the array of sensors to 1  
   
    float temp_median = 0.0;
    float _t = 0.0;
    int _cursor = 0;

    // sensors are assumed tobe working correctly by default. 
    // If theres a fault they will be assigned 0 at run time, since its time consuming otherwise. 
    
    for(int i = 0; i < NUM_OF_SENSORS; i++){

        float _t = 0.0; // clear at each iteration
        int _cursor = 0; // clear at each iteration
        
        // check for valid range. The valid range is 0ºC ~ 65ºC
        if(TemperatureOf[i] < TEMP_MIN || TemperatureOf[i] > TEMP_MAX){

            // print sensor i is faulty
            Serial.println("WARNING!");
            Serial.print(" Sensor");
            Serial.print(i+1); // array is starting from index 0
            Serial.println(" is faulty for temperature. Therefore the accuracy will derease");
            SensorHealthOf[i] = 0; // label the sensor as faulty
            
        }else{ // if the sensor value is within valid range

             if( dT[i] != 0.0){
                // print sensor i is faulty
                Serial.println("WARNING!");
                Serial.print(" Sensor");
                Serial.print(i+1); // array is starting from index 0
                Serial.println(" is faulty for temperature. Therefore the accuracy will derease");
                SensorHealthOf[i] = 0; // label the sensor as faulty
             }
          
        }      

    }

}

void makeAllSensorsToDefaultState(){
    for (int i = 0; i < NUM_OF_SENSORS; ++i){
        SensorHealthOf[i] = 1; // 1 means sensor is in good health
    }  
}


/********************************************************* Preassure Calculations *******************************************/




/**********************************************
  
  Calculate Average Pressure
  
************************************************/
float getAveragePressure(int SensorHealthOf[], float PressureOf[]){

    float p_sum = 0.0; 
    int s_sum = 0;

    for(int i = 0; i < NUM_OF_SENSORS; i++){
        p_sum += PressureOf[i] * SensorHealthOf[i];
        s_sum += SensorHealthOf[i];
      
    }

    return (p_sum / s_sum);

}


/**********************************************
 
  Calculate Temperature Accuracy

***********************************************/
float getPressureAccuracy(int SensorHealthOf[], float PressureOf[], float average_pressure){

    float accuracy = 0.0;

    for(int i = 0; i < NUM_OF_SENSORS; i++){

        accuracy += ( 1  - (abs(PressureOf[i] - average_pressure)/average_pressure) ) * ( 100 / NUM_OF_SENSORS ) * SensorHealthOf[i];
      
    }

    return accuracy;

}

/*********************************************************************************************************

  Validate Sensors with respect to Pressure readings
  This will evaluate the sensor health. S[index] -> {0,1} where 0 represents a fault and 1 represents good 

 **********************************************************************************************************/
void checkSensorHealthForPressure(float PressureOf[]){

    makeAllSensorsToDefaultState(); // clear and re-initialize the array of sensors to 1  

    // sensors are assumed tobe working correctly by default. 
    // If theres a fault they will be assigned 0 at run time, since its time consuming otherwise. 
    
    for(int i = 0; i < NUM_OF_SENSORS; i++){

        
        // check for valid range. The valid range is 500Pa ~ 1500Pa
        if(PressureOf[i] < PRESSURE_MIN || PressureOf[i] > PRESSURE_MAX){

            // print sensor i is faulty
            Serial.println("WARNING!");
            Serial.print(" Sensor");
            Serial.print(i+1); // array is starting from index 0
            Serial.println(" is faulty for pressure. Therefore the accuracy will derease");
            SensorHealthOf[i] = 0; // label the sensor as faulty
            
        }else{ // if the sensor value is within valid range

             if( abs(dP[i] * 10000 ) >= 1.0){ // experimentally found value dP/dt = -0.0002815676 Pa/s when changed from 935.280 Pa to 594.020 Pa 
                // print sensor i is faulty
                Serial.println("WARNING!");
                Serial.print(" Sensor");
                Serial.print(i+1); // array is starting from index 0
                Serial.println(" is faulty for pressure. Therefore the accuracy will derease");
                SensorHealthOf[i] = 0; // label the sensor as faulty
             }
          
        }      

    }

}



