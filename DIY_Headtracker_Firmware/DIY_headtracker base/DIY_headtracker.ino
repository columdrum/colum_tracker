//-----------------------------------------------------------------------------
// Original project by Dennis Frie - 2012 - Dennis.frie@gmail.com
// Discussion: http://www.rcgroups.com/forums/showthread.php?t=1677559
//
// Other contributors to this code:
//  Mark Mansur (Mangus on rcgroups)
//  
// Version history:
// - 0.01 - 0.08 - Dennis Frie - preliminary releases
// - 1.01 - April 2013 - Mark Mansur - code clean-up and refactoring, comments
//      added. Added pause functionality, added settings retrieval commands.
//      Minor optimizations.
//-----------------------------------------------------------------------------

#include <Wire.h>
#include <EEPROM.h>

#define HT_TILT_REVERSE_BIT     0x01
#define HT_ROLL_REVERSE_BIT     0x02
#define HT_PAN_REVERSE_BIT      0x04
#define FIRMWARE_VERSION_FLOAT  1.04    // 2 decimal places

#define  ARDUINO_LED 13
#define SERIAL_BAUD 57600

// Gyro
//
#define ITG3205_ADDR 0x68    // The address of ITG3205
#define ITG3205_X_ADDR 0x1D  // Start address for x-axis
#define SCALING_FACTOR 13     // Scaling factor - used when converting to angle

// Accelerometer
//
#define ADXL345_ADDR (0x53)  // The adress of ADXL345 
#define ADXL345_X_ADDR (0x32)// Start address for x-axis
#define ACC_SENS 256         // Sensitivity. 13 bit adc, +/- 16 g. Calculated as: (2^13)/(16*2)
#define ASSUME_1G_ACC 0      // Assuming the total gravitation is 1. True if only earth gravitation has influence.  


// Magnetometer
//
#define HMC_ADDR 0x1E        // The address of HMC5883
#define HMC_X_ADDR (0x03)    // Start address for x-axis. 


unsigned char ADXL345_ID = 0;
unsigned char ITG3205_ID = 0;
unsigned char HMC_ID = 0;

int magPosOff[3];
int magNegOff[3];
float magGain[3];


// Variables defined elsewhere
//
extern long channel_value[];

// Local variables
//
byte sensorBuffer[10];   

// Local file variables
//
int frameNumber = 0;		    // Frame count since last debug serial output

char serial_data[101];          // Array for serial-data 
unsigned char serial_index = 0; // How many bytes have been received?
char string_started = 0;        // Only saves data if string starts with right byte
unsigned char channel_mapping[13];

char outputMag = 0;             // Stream magnetometer data to host
char outputAcc = 0;             // Stream accelerometer data to host
char outputMagAcc = 0;          // Stream mag and accell data (for calibration on PC)
char outputTrack = 0;	        // Stream angle data to host

// Keep track of button press
char lastButtonState = 0;           // 0 is not pressed, 1 is pressed
unsigned long buttonDownTime = 0;   // the system time of the press
char pauseToggled = 0;              // Used to make sure we toggle pause only once per hold
char ht_paused = 0;

// External variables (defined in other files)
//
char read_sensors = 0;
extern char resetValues;   
extern char tiltInverse;
extern char rollInverse;
extern char panInverse;

// Settings (Defined in sensors.cpp)
//
extern float tiltRollBeta;
extern float panBeta;
extern float gyroWeightTiltRoll;
extern float GyroWeightPan;
extern int servoPanCenter;
extern int servoTiltCenter;
extern int servoRollCenter;
extern int panMaxPulse;
extern int panMinPulse;
extern int tiltMaxPulse;
extern int tiltMinPulse;
extern int rollMaxPulse;
extern int rollMinPulse;
extern float panFactor;
extern float tiltFactor;  
extern float rollFactor;
extern unsigned char servoReverseMask;
extern unsigned char htChannels[];
extern float gyroOff[];
extern float magOffset[];
extern int accOffset[]; 



/****************************************************/

char resetValues = 1;        // Used to reset headtracker/re-center. 

long accRaw[3];              // Raw readings from accelerometer
float accG[3];               // G-force in each direction
float accAngle[3];           // Measured angle from accelerometer
float R;                     // Unit vector - total G.

float gyroRaw[3];            // Raw readings from gyro
float angle[3];              // Angle from gyro 
float angleRaw[3];           // Temp for angle-calculation

float magRaw[3];             // Raw readings from magnetometer
float magAngle[3];           // Measured angles from magnetometer
float mx = 0;                // Calculated magnetometer value in x-direction with pan/tilt compensation
float my = 0;                // Calculated magnetometer value in y-direction with pan/tilt compensation


#define MAG0MAX 625
#define MAG1MAX 625
#define MAG2MAX 625
#define MAG0MIN -625
#define MAG1MIN -625
#define MAG2MIN -625


// Final angles for headtracker:
float tiltAngle = 90;       // Tilt angle
float tiltAngleLP = 90;     // Tilt angle with low pass FilterSensorData
float lastTiltAngle = 90;   // Used in low pass FilterSensorData.

float rollAngle = 0;        // Roll angle
float rollAngleLP = 90;     // Roll angle with low pass FilterSensorData
float lastRollAngle = 90;   // Used in low pass FilterSensorData

float panAngle = 90;        // Pan angle
float panAngleLP = 90;      // Pan angle with low pass FilterSensorData
float lastPanAngle = 90;    // Used in low pass FilterSensorData

// Start values - center position for head tracker
float tiltStart = 0;
float panStart = 0;
float rollStart = 0;

char TrackerStarted = 0;

// Servo reversing
char tiltInverse = -1;
char rollInverse = -1;
char panInverse = -1;


/************************************************************************/
void SensorInfoPrint()
{ 
   Serial.print("Mag cal:");
   Serial.print(magNegOff[0] - magPosOff[0]);
   Serial.print(",");   
   Serial.print(magNegOff[1] - magPosOff[1]);
   Serial.print(",");      
   Serial.print(magNegOff[2] - magPosOff[2]);
   
   Serial.print(",");      
   Serial.print(magGain[0]);
   Serial.print(",");   
   Serial.print(magGain[1]);
   Serial.print(",");      
   Serial.println(magGain[2]);   

   Serial.print("ADXL345 ID: ");
   Serial.println((int)ADXL345_ID); 
   Serial.print("ITG3205 ID: ");
   Serial.println((int)ITG3205_ID); 
   Serial.print("HMC ID: ");
   Serial.println((int)HMC_ID);   
}


// Function used to write to I2C:
void WriteToI2C(int device, byte address, byte val)
{
    Wire.beginTransmission(device);
    Wire.write(address);
    Wire.write(val);
    Wire.endTransmission();
}

// Function to read from I2C
void ReadFromI2C(int device, char address, char bytesToRead)
{
    Wire.beginTransmission(device);
    Wire.write(address);
    Wire.endTransmission();
  
    Wire.beginTransmission(device);
    Wire.requestFrom(device, bytesToRead);
   
    char i = 0;   
    while ( Wire.available() )
    {
        sensorBuffer[i++] = Wire.read();
    }   
    Wire.endTransmission();
}

void InitSensors()
{

    ReadFromI2C(ITG3205_ADDR, 0x00, 1);
    ITG3205_ID = sensorBuffer[0];
 
    
    Serial.print("ITG3205: ");
    Serial.print(sensorBuffer[0]);


    ReadFromI2C(ADXL345_ADDR, 0x00, 1);
    ADXL345_ID = sensorBuffer[0];
 
    
    Serial.print("    ADXL: ");
    Serial.print(sensorBuffer[0]); 


    // Accelerometer increase G-range (+/- 16G)
    WriteToI2C(ADXL345_ADDR, 0x31, 0b00001011);     
  /*
    ReadFromI2C(HMC_ADDR, 0x00, 1);
    HMC_ID = sensorBuffer[0];

    
    Serial.print("    HMC: ");
    Serial.println(sensorBuffer[0]); 

*/
    //WriteToI2C(ITG3205_ADDR, 22, 24);

    //  ADXL345 POWER_CTL
    WriteToI2C(ADXL345_ADDR, 0x2D, 0); 
    WriteToI2C(ADXL345_ADDR, 0x2D, 16);
    WriteToI2C(ADXL345_ADDR, 0x2D, 8);

    // HMC5883
    // Run in continuous mode
    WriteToI2C(HMC_ADDR, 0x02, 0x00);    


}


//--------------------------------------------------------------------------------------
// Func: SaveSettings
// Desc: Saves device settings to EEPROM for retrieval at boot-up.
//--------------------------------------------------------------------------------------
void SaveSettings()
{  
    EEPROM.write(1, (unsigned char)(tiltRollBeta * 100));
    EEPROM.write(2, (unsigned char)(panBeta * 100));
    EEPROM.write(3, (unsigned char)(gyroWeightTiltRoll * 100));
    EEPROM.write(4, (unsigned char)(GyroWeightPan * 100));
  
    EEPROM.write(5, (unsigned char)servoReverseMask);
    
    // 6 unused
  
    EEPROM.write(7, (unsigned char)servoPanCenter);
    EEPROM.write(8, (unsigned char)(servoPanCenter >> 8));  
  
    EEPROM.write(9, (unsigned char)(tiltFactor * 10));
    EEPROM.write(10, (int)((tiltFactor * 10)) >> 8);  

    EEPROM.write(11, (unsigned char) (panFactor * 10));
    EEPROM.write(12, (int)((panFactor * 10)) >> 8);  

    EEPROM.write(13, (unsigned char) (rollFactor * 10));
    EEPROM.write(14, (int)((rollFactor * 10)) >> 8);  

    // 15 unused

    EEPROM.write(16, (unsigned char)servoTiltCenter);
    EEPROM.write(17, (unsigned char)(servoTiltCenter >> 8));  

    EEPROM.write(18, (unsigned char)servoRollCenter);
    EEPROM.write(19, (unsigned char)(servoRollCenter >> 8));  


    EEPROM.write(20, (unsigned char)panMaxPulse);
    EEPROM.write(21, (unsigned char)(panMaxPulse >> 8));  
  
    EEPROM.write(22, (unsigned char)panMinPulse);
    EEPROM.write(23, (unsigned char)(panMinPulse >> 8));    

    EEPROM.write(24, (unsigned char)tiltMaxPulse);
    EEPROM.write(25, (unsigned char)(tiltMaxPulse >> 8));    

    EEPROM.write(26, (unsigned char)tiltMinPulse);
    EEPROM.write(27, (unsigned char)(tiltMinPulse >> 8));

    EEPROM.write(28, (unsigned char)rollMaxPulse);
    EEPROM.write(29, (unsigned char)(rollMaxPulse >> 8));    

    EEPROM.write(30, (unsigned char)rollMinPulse);
    EEPROM.write(31, (unsigned char)(rollMinPulse >> 8)); 
  
    EEPROM.write(32, (unsigned char)htChannels[0]);
    EEPROM.write(33, (unsigned char)htChannels[1]);
    EEPROM.write(34, (unsigned char)htChannels[2]);
  
    // Saving gyro calibration values
    int temp = (int)(gyroOff[0] + 500.5);
    EEPROM.write(35, (unsigned char)temp);
    EEPROM.write(36, (unsigned char)(temp >> 8));   
  
    temp = (int)(gyroOff[1] + 500.5);
    EEPROM.write(37, (unsigned char)temp);
    EEPROM.write(38, (unsigned char)(temp >> 8));     

    temp = (int)(gyroOff[2] + 500.5);
    EEPROM.write(39, (unsigned char)temp);
    EEPROM.write(40, (unsigned char)(temp >> 8));    
  
    // Mark the memory to indicate that it has been
    // written. Used to determine if board is newly flashed
    // or not.
    EEPROM.write(0,8); 

    Serial.println("Settings saved!");
}

//--------------------------------------------------------------------------------------
// Func: GetSettings
// Desc: Retrieves device settings from EEPROM.
//--------------------------------------------------------------------------------------
void GetSettings()
{  
    tiltRollBeta    = (float)EEPROM.read(1) / 100;
    panBeta         = (float)EEPROM.read(2) / 100;
    gyroWeightTiltRoll = (float)EEPROM.read(3) / 100;
    GyroWeightPan   = (float)EEPROM.read(4) / 100;  
  
    tiltInverse = 1;
    rollInverse = 1;
    panInverse = 1;

    unsigned char temp = EEPROM.read(5);
    if ( temp & HT_TILT_REVERSE_BIT )
    {
        tiltInverse = -1;
    }  
    if ( temp & HT_ROLL_REVERSE_BIT )
    {
        rollInverse = -1;
    }
    if ( temp & HT_PAN_REVERSE_BIT )
    {
        panInverse = -1;
    }

    // 6 unused

    servoPanCenter  = EEPROM.read(7) + (EEPROM.read(8) << 8);
    tiltFactor      = (float)(EEPROM.read(9) + (EEPROM.read(10) << 8)) / 10;
    panFactor       = (float)(EEPROM.read(11) + (EEPROM.read(12) << 8)) / 10;
    rollFactor       = (float)(EEPROM.read(13) + (EEPROM.read(14) << 8)) / 10;  

    // 15 unused

    servoTiltCenter = EEPROM.read(16) + (EEPROM.read(17) << 8);
    servoRollCenter = EEPROM.read(18) + (EEPROM.read(19) << 8);  
  
    panMaxPulse   = EEPROM.read(20) + (EEPROM.read(21) << 8);  
    panMinPulse   = EEPROM.read(22) + (EEPROM.read(23) << 8);    
  
    tiltMaxPulse  = EEPROM.read(24) + (EEPROM.read(25) << 8);  
    tiltMinPulse  = EEPROM.read(26) + (EEPROM.read(27) << 8);      
  
    rollMaxPulse  = EEPROM.read(28) + (EEPROM.read(29) << 8);  
    rollMinPulse  = EEPROM.read(30) + (EEPROM.read(31) << 8);        
  
    htChannels[0] = EEPROM.read(32);  
    htChannels[1] = EEPROM.read(33);  
    htChannels[2] = EEPROM.read(34);    
  
    gyroOff[0] = EEPROM.read(35) + (EEPROM.read(36) << 8) - 500; 
    gyroOff[1] = EEPROM.read(37) + (EEPROM.read(38) << 8) - 500; 
    gyroOff[2] = EEPROM.read(39) + (EEPROM.read(40) << 8) - 500;   
  
    magOffset[0] = EEPROM.read(200) + (EEPROM.read(201) << 8) - 2000;     
    magOffset[1] = EEPROM.read(202) + (EEPROM.read(203) << 8) - 2000;     
    magOffset[2] = EEPROM.read(204) + (EEPROM.read(205) << 8) - 2000;       
  
    accOffset[0] = EEPROM.read(206) + (EEPROM.read(207) << 8) - 2000;     
    accOffset[1] = EEPROM.read(208) + (EEPROM.read(209) << 8) - 2000;     
    accOffset[2] = EEPROM.read(210) + (EEPROM.read(211) << 8) - 2000;       
  
 

}

//--------------------------------------------------------------------------------------
// Func: SaveMagData
// Desc: Stores magnetometer calibration info to EEPROM.
//--------------------------------------------------------------------------------------
void SaveMagData()
{
    int temp = (int)(magOffset[0] + 2000);
    EEPROM.write(200, (unsigned char)temp);
    EEPROM.write(201, (unsigned char)(temp >> 8));   
  
    temp = (int)(magOffset[1] + 2000);
    EEPROM.write(202, (unsigned char)temp);
    EEPROM.write(203, (unsigned char)(temp >> 8));   
  
    temp = (int)(magOffset[2] + 2000);
    EEPROM.write(204, (unsigned char)temp);
    EEPROM.write(205, (unsigned char)(temp >> 8));   
  
    Serial.println("Mag offset saved!"); 
    Serial.print(magOffset[0]);
    Serial.print(", "); 
    Serial.print(magOffset[1]);
    Serial.print(", ");   
    Serial.println(magOffset[2]); 
}

//--------------------------------------------------------------------------------------
// Func: SaveAccelData
// Desc: Stores accelerometer calibration data to EEPOM.
//--------------------------------------------------------------------------------------
void SaveAccelData()
{
    int temp = (int)(accOffset[0] + 2000);
    EEPROM.write(206, (unsigned char)temp);
    EEPROM.write(207, (unsigned char)(temp >> 8));   
  
    temp = (int)(accOffset[1] + 2000);
    EEPROM.write(208, (unsigned char)temp);
    EEPROM.write(209, (unsigned char)(temp >> 8));   
  
    temp = (int)(accOffset[2] + 2000);
    EEPROM.write(210, (unsigned char)temp);
    EEPROM.write(211, (unsigned char)(temp >> 8));   
  
    Serial.println("Acc offset saved!"); 
    Serial.print(accOffset[0]);
    Serial.print(","); 
    Serial.print(accOffset[1]);
    Serial.print(",");   
    Serial.println(accOffset[2]);  
}

//--------------------------------------------------------------------------------------
// Func: DebugOutput
// Desc: Outputs useful device/debug information to the serial port.
//--------------------------------------------------------------------------------------
void DebugOutput()
{
    Serial.println();  
    Serial.println();
    Serial.println();
    Serial.println("------ Debug info------");

    Serial.print("FW Version: ");
    Serial.println(FIRMWARE_VERSION_FLOAT, 2);
    
    Serial.print("tiltRollBeta: ");
    Serial.println(tiltRollBeta); 

    Serial.print("panBeta: ");
    Serial.println(panBeta); 
 
    Serial.print("gyroWeightTiltRoll: ");
    Serial.println(gyroWeightTiltRoll); 

    Serial.print("GyroWeightPan: ");
    Serial.println(GyroWeightPan); 

    Serial.print("servoPanCenter: ");
    Serial.println(servoPanCenter); 
 
    Serial.print("servoTiltCenter: ");
    Serial.println(servoTiltCenter); 

    Serial.print("servoRollCenter: ");
    Serial.println(servoRollCenter); 

    Serial.print("tiltFactor: ");
    Serial.println(tiltFactor); 

    Serial.print("panFactor: ");
    Serial.println(panFactor);  
 
    Serial.print("Gyro offset stored ");
    Serial.print(gyroOff[0]);
    Serial.print(",");   
    Serial.print(gyroOff[1]);
    Serial.print(",");      
    Serial.println(gyroOff[2]);    
 
    Serial.print("Mag offset stored ");
    Serial.print(magOffset[0]);
    Serial.print(",");   
    Serial.print(magOffset[1]);
    Serial.print(",");      
    Serial.println(magOffset[2]);
 
    Serial.print("Acc offset stored ");
    Serial.print(accOffset[0]);
    Serial.print(",");   
    Serial.print(accOffset[1]);
    Serial.print(",");      
    Serial.println(accOffset[2]);
 
    SensorInfoPrint();    
}


void UpdateSensors()
{

    // Read x, y, z acceleration, pack the data.
    ReadFromI2C(ADXL345_ADDR, ADXL345_X_ADDR, 6);
    accRaw[0] = ((int)sensorBuffer[0] | ((int)sensorBuffer[1] << 8)) * -1;
    accRaw[1] = ((int)sensorBuffer[2] | ((int)sensorBuffer[3] << 8)) * -1;       
    accRaw[2] = (int)sensorBuffer[4] | ((int)sensorBuffer[5] << 8);       
   
   /*
    // Read x, y, z from gyro, pack the data
    ReadFromI2C(ITG3205_ADDR, ITG3205_X_ADDR, 6);
    gyroRaw[0] = (int)sensorBuffer[1] | ((int)sensorBuffer[0] << 8);
    gyroRaw[1] = ( (int)sensorBuffer[3] | ((int)sensorBuffer[2] << 8) ) * -1;
    gyroRaw[2] = ( (int)sensorBuffer[5] | ((int)sensorBuffer[4] << 8) ) * -1;
   */
   
    // Read x, y, z from magnetometer;
    ReadFromI2C(HMC_ADDR, HMC_X_ADDR, 6);
    for (unsigned char i =0; i < 3; i++)
    {
       magRaw[i] = (int)sensorBuffer[(i * 2) + 1] | ((int)sensorBuffer[i * 2] << 8);
    }    
}


void AccelCalc()
{
    accRaw[0] += accOffset[0];
    accRaw[1] += accOffset[1];
    accRaw[2] += accOffset[2];   
  
    for (unsigned char i = 0; i<3; i++)
    {
        accG[i] = (float)accRaw[i] / ACC_SENS;
    }
     
    // So, lets calculate R
    // R^2 = Rx^2+Ry^2+Rz^2    
    #if (ASSUME_1G_ACC == 0) 
        R = sqrt((accG[0] * accG[0]) + (accG[1] * accG[1]) + (accG[2] * accG[2]));
    #else // Otherwise, just assume total G = 1.
        R = 1;
    #endif
      
    // Calculate final angles:
    if (R < 1.3 && R > 0.7)
    { 
        for (unsigned char i = 0; i<3; i++)
        {
            accAngle[i] = acos(accG[i] / R) * 57.3;
        }  
    }
}

//--------------------------------------------------------------------------------------
// Func: MagCalc
// Desc: Calculates angle from magnetometer data.
//--------------------------------------------------------------------------------------
void MagCalc()
{
    // Invert 2 axis  
    magRaw[1] *= -1;
    magRaw[2] *= -1;
    
    // Set gain:
    magRaw[0] *= magGain[0];
    magRaw[1] *= magGain[1];
    magRaw[2] *= magGain[2];    
    
    magRaw[0] -= magOffset[0];
    magRaw[1] -= magOffset[1];
    magRaw[2] -= magOffset[2];    
  
    float testAngle = tiltAngle - 90;
    mx = magRaw[0] * cos((testAngle) / 57.3)
        + magRaw[1] * sin(testAngle / 57.3);

    my = magRaw[0] * sin((rollAngle - 90) / 57.3)
        * sin((tiltAngle - 90) / 57.3)
        + magRaw[2] * cos((rollAngle - 90) / 57.3)
        - magRaw[1] * sin((rollAngle - 90) / 57.3)
        * cos((tiltAngle - 90) / 57.3);
      
    // Calculate pan-angle from magnetometer. 
    magAngle[2] = (atan(mx / my) * 57.3 + 90);

    // Get full 0-360 degrees. 
    if (my < 0)
    {
        magAngle[2] += 180;
    }
    
    float tempAngle = panStart - magAngle[2];
      
    if (tempAngle > 180)
    {
        tempAngle -= 360; 
    }  
    else if (tempAngle < -180)
    {
        tempAngle += 360; 
    }
      
    magAngle[2] = tempAngle * -1;
}



// End settings   -------------------------------------------------------------

//--------------------------------------------------------------------------------------
// Func: setup
// Desc: Called by Arduino framework at initialization time. This sets up pins for I/O,
//       initializes sensors, etc.
//--------------------------------------------------------------------------------------
void setup()
{
    Serial.begin(SERIAL_BAUD);

  
    pinMode(9,OUTPUT);
    digitalWrite(2,HIGH);
    digitalWrite(3,HIGH);  
  
    // Set all other pins to input, for safety.
    pinMode(0,INPUT);
    pinMode(1,INPUT);
    pinMode(2,INPUT);
    pinMode(3,INPUT);
    pinMode(6,INPUT);
    pinMode(7,INPUT);  
    pinMode(8,INPUT);    

  
    digitalWrite(0,LOW); // pull-down resistor
    digitalWrite(1,LOW);
    digitalWrite(2,HIGH);
    digitalWrite(3,HIGH);  
  
    pinMode(ARDUINO_LED,OUTPUT);    // Arduino LED
    digitalWrite(ARDUINO_LED, HIGH);
    

    // Give it time to be noticed, then turn it off
    delay(200); // Note: only use delay here. This won't work when Timer0 is repurposed later.
    digitalWrite(ARDUINO_LED, LOW);

    Wire.begin();               // Start I2C

  
    if (EEPROM.read(0) != 8)
    {
     //   InitSensors();
        //SaveSettings();
        //SaveMagData();
        //SaveAccelData();
    }
 
   // GetSettings();                 // Get settings saved in EEPROM
    InitSensors();                // Initialize I2C sensors
  //  CalibrateMag();
   // ResetCenter();
}

//--------------------------------------------------------------------------------------
// Func: loop
// Desc: Called by the Arduino framework once per frame. Represents main program loop.
//--------------------------------------------------------------------------------------
void loop()
{  

    // All this is used for communication with GUI 
    //
    if (Serial.available())
    {
       
      Serial.read();
    }
      //UpdateSensors();
      //GyroCalc();
      //AccelCalc();
      //MagCalc();
      //FilterSensorData();
      
      if (string_started == 0) {
          string_started = 1;
          digitalWrite(ARDUINO_LED, HIGH);
      } else {
          string_started = 0;
          digitalWrite(ARDUINO_LED, LOW);
      }
      
      delay(500); 
}

