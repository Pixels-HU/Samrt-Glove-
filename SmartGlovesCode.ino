
#include <Wire.h>
#include "SD.h"
#define SD_ChipSelectPin 10
#include "TMRpcm.h"
#include "SPI.h"
#include "init.h"
#define Flex_Pin0 A3   
#define Flex_Pin1 A1 
// #define Flex_Pin2 A2   
TMRpcm tmrpcm;
#define Button 8
unsigned long long int t=0;
const int FlexBendTolarance =40 ;
const int FlexStraightTolarance = 20;
const int NumberOfCalibrations = 10;

int Fist[2] = {0, 0};
int Point[2] = {0, 0};

int ReadFlex[2] = {0, 0};
typedef union accel_t_gyro_union{
  struct
  {
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
    uint8_t t_h;
    uint8_t t_l;
    uint8_t x_gyro_h;
    uint8_t x_gyro_l;
    uint8_t y_gyro_h;
    uint8_t y_gyro_l;
    uint8_t z_gyro_h;
    uint8_t z_gyro_l;
  } reg;
  struct
  {
    int x_accel;
    int y_accel;
    int z_accel;
    int temperature;
    int x_gyro;
    int y_gyro;
    int z_gyro;
  } value;
};

// Use the following global variables and access functions to help store the overall
// rotation angle of the sensor
unsigned long last_read_time;
float         last_x_angle;  // These are the filtered angles
float         last_y_angle;
float         last_z_angle;
float         last_gyro_x_angle;  // Store the gyro angles to compare drift
float         last_gyro_y_angle;
float         last_gyro_z_angle;

void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
  last_gyro_x_angle = x_gyro;
  last_gyro_y_angle = y_gyro;
  last_gyro_z_angle = z_gyro;
}

inline unsigned long get_last_time() {
  return last_read_time;
}
inline float get_last_x_angle() {
  return last_x_angle;
}
inline float get_last_y_angle() {
  return last_y_angle;
}
inline float get_last_z_angle() {
  return last_z_angle;
}
inline float get_last_gyro_x_angle() {
  return last_gyro_x_angle;
}
inline float get_last_gyro_y_angle() {
  return last_gyro_y_angle;
}
inline float get_last_gyro_z_angle() {
  return last_gyro_z_angle;
}

//  Use the following global variables and access functions
//  to calibrate the acceleration sensor
float    base_x_accel;
float    base_y_accel;
float    base_z_accel;

float    base_x_gyro;
float    base_y_gyro;
float    base_z_gyro;


int read_gyro_accel_vals(uint8_t* accel_t_gyro_ptr) {
  // Read the raw values.
  // Read 14 bytes at once,
  // containing acceleration, temperature and gyro.
  // With the default settings of the MPU-6050,
  // there is no filter enabled, and the values
  // are not very stable.  Returns the error value

  accel_t_gyro_union* accel_t_gyro = (accel_t_gyro_union *) accel_t_gyro_ptr;

  int error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) accel_t_gyro, sizeof(*accel_t_gyro));

  // Swap all high and low bytes.
  // After this, the registers values are swapped,
  // so the structure name like x_accel_l does no
  // longer contain the lower byte.
  uint8_t swap;
#define SWAP(x,y) swap = x; x = y; y = swap

  SWAP ((*accel_t_gyro).reg.x_accel_h, (*accel_t_gyro).reg.x_accel_l);
  SWAP ((*accel_t_gyro).reg.y_accel_h, (*accel_t_gyro).reg.y_accel_l);
  SWAP ((*accel_t_gyro).reg.z_accel_h, (*accel_t_gyro).reg.z_accel_l);
  SWAP ((*accel_t_gyro).reg.t_h, (*accel_t_gyro).reg.t_l);
  SWAP ((*accel_t_gyro).reg.x_gyro_h, (*accel_t_gyro).reg.x_gyro_l);
  SWAP ((*accel_t_gyro).reg.y_gyro_h, (*accel_t_gyro).reg.y_gyro_l);
  SWAP ((*accel_t_gyro).reg.z_gyro_h, (*accel_t_gyro).reg.z_gyro_l);

  return error;
}

// The sensor should be motionless on a horizontal surface
//  while calibration is happening
void calibrate_sensors() {
  int                   num_readings = 10;
  float                 x_accel = 0;
  float                 y_accel = 0;
  float                 z_accel = 0;
  float                 x_gyro = 0;
  float                 y_gyro = 0;
  float                 z_gyro = 0;
  accel_t_gyro_union    accel_t_gyro;

  //Serial.println("Starting Calibration");

  // Discard the first set of values read from the IMU
  read_gyro_accel_vals((uint8_t *) &accel_t_gyro);

  // Read and average the raw values from the IMU
  for (int i = 0; i < num_readings; i++) {
    read_gyro_accel_vals((uint8_t *) &accel_t_gyro);
    x_accel += accel_t_gyro.value.x_accel;
    y_accel += accel_t_gyro.value.y_accel;
    z_accel += accel_t_gyro.value.z_accel;
    x_gyro += accel_t_gyro.value.x_gyro;
    y_gyro += accel_t_gyro.value.y_gyro;
    z_gyro += accel_t_gyro.value.z_gyro;
    delay(100);
  }
  x_accel /= num_readings;
  y_accel /= num_readings;
  z_accel /= num_readings;
  x_gyro /= num_readings;
  y_gyro /= num_readings;
  z_gyro /= num_readings;

  // Store the raw calibration values globally
  base_x_accel = x_accel;
  base_y_accel = y_accel;
  base_z_accel = z_accel;
  base_x_gyro = x_gyro;
  base_y_gyro = y_gyro;
  base_z_gyro = z_gyro;

  //Serial.println("Finishing Calibration");
}

int x = 0;
String First_move = "";
String Second_move="";

 float angle_x = 0;
 float angle_y = 0;
 float angle_z = 0;

 bool once = true;

 int error;
 uint8_t c;

bool toggle=0;
const int xtolr=5;
void setup(){   
   tmrpcm.speakerPin=9;
  
  if(!SD.begin(SD_ChipSelectPin))
{
  Serial.println("SD fail");
  return;
}
  tmrpcm.setVolume(6);
  pinMode(Button,INPUT_PULLUP);
  Serial.begin(19200);
  
  Serial.println("Make Fist");
  tmrpcm.play("1.wav");
  while(digitalRead(Button));
  delay(500);
  
  CalibrateFist();
  Serial.print("Fist");Serial.print(Fist[0]);Serial.print(" , ");Serial.print(Fist[1]);Serial.print(" , ");Serial.println(Fist[2]);

  Serial.println("Make Point");
  tmrpcm.play("2.wav");
  while(digitalRead(Button));
  delay(500);
  
  CalibratePoint();
   Serial.print("Point");Serial.print(Point[0]);Serial.print(" , ");Serial.print(Point[1]);Serial.print(" , ");Serial.println(Point[2]);
   while(digitalRead(Button));
   delay(500);

  mpu_intialize();
 
}


void loop(){
   get_mpu_data();
   if(digitalRead(Button)==LOW){
    toggle=!toggle;
    Serial.print(toggle);
    delay(500);
    }
   if(toggle){
   print_mpu_filter_data();
  //  Serial.print(CheckBend(0));Serial.print(CheckBend(1));Serial.println(CheckBend(2));
  // Serial.print("ADC : ");Serial.print(ReadFlex[0]);Serial.print(" , ");Serial.print(ReadFlex[1]);Serial.print(" , ");Serial.println(ReadFlex[2]);
   if (((angle_x>=15 && angle_x<=45) && (angle_y>=-75 && angle_y<=-35)&&(!CheckBend(0)) &&(CheckBend(1)) )||(First_move=="yadobak")){
    First_move="yadobak"; //make timer
    if(once){
      once=false;
      t=millis();
    }
      if(t+2500<millis() && !once)
      {
      First_move="";
      Second_move="yaobak";
      t=0;
      once=true;
      tmrpcm.play("ydobak.wav");
      Serial.println("yadouuuuuuuuubak");
      }
      
}
else{
  once=true;
}


  
   }
   
   delay(5);
   
   //======================================================================================= LOGIC ======================================================================================
  ReadFlex[0] = analogRead(Flex_Pin0);
  ReadFlex[1] = analogRead(Flex_Pin1);
  //ReadFlex[2] = analogRead(Flex_Pin2);

//Serial.print("ADC : ");Serial.print(ReadFlex[0]);Serial.print(" , ");Serial.print(ReadFlex[1]);Serial.print(" , ");Serial.println(ReadFlex[2]);
//Serial.print("Bool : ");Serial.print(CheckStraight(0));Serial.print(" , ");Serial.print(CheckStraight(1));Serial.print(" , ");Serial.println(CheckStraight(2));
Serial.print(CheckBend(0));Serial.print(CheckBend(1));
Serial.println("  ");


 if (((angle_x<(5-xtolr) && angle_y<=18 )&& !CheckBend(0) && !CheckBend(1))||First_move=="hello"){
    First_move="hello"; //make timer
    Serial.println("hellllllooooo ====================================================");
    if (angle_x>(5+xtolr) && angle_y<=18 ){
     First_move="";
     Serial.println("hello");
     tmrpcm.play("hello.wav");
     //delay(250);
    }
 }
 if (((angle_x<(5-xtolr) && angle_y<=18 )&& CheckBend(0) && !CheckBend(1))||First_move=="no"){
    First_move="no"; //make timer
    Serial.println("nooooooooooooooooooo ====================================================");
    if (angle_x>(5+xtolr) && angle_y<=18 ){
     First_move="";
     Serial.println("no");
     tmrpcm.play("no.wav");
     //delay(250);
    }
 }




  if (((angle_x>=40 && angle_x<=75) && (angle_y>=-20 && angle_y<=15)&&(CheckStraight(1)) &&(CheckBend(0)) )||(First_move=="weare")){
    First_move="weare"; //make timer
    if ((angle_x>=5 && angle_x<=15) && (angle_y>=-10 && angle_y<=10)){
      Second_move="weare";
      if (((angle_x>=10 && angle_x<=50) && (angle_y>=-35 && angle_y<=-10)&&(CheckStraight(1)) &&(CheckBend(0)))||(Second_move=="weare")){
        
      
     First_move="";
     Serial.println("weare");
    }
 }
 }

  if (((angle_x>=65 && angle_x<=85) && (angle_y>=-20 && angle_y<=-5)&&(!CheckBend(0)) &&(CheckBend(1)) )||(First_move=="help")){
    First_move="help"; //make timer
    if ((angle_x>=65 && angle_x<=85) && (angle_y>=0 && angle_y<=15)){
      Second_move="help";
      if (((angle_x>=65 && angle_x<=85) && (angle_y>=-20 && angle_y<=-5)&&(!CheckBend(0)) &&(CheckBend(1)) )||(Second_move=="help")){
        
      
     First_move="";
     Serial.println("help");
     tmrpcm.play("help.wav");
     
    }
  }
 }

 //(angle_x>=-9 && angle_x<=5) &&
 //(angle_x<=15 && angle_x>=0) &&
 if (( (angle_y>=-90 && angle_y<=-70)&&(CheckBend(0))&&(CheckBend(1)))||(First_move=="yes")){
  First_move="yes";
  if (  (angle_y>=-10 && angle_y<=20)&&(CheckBend(0))  ){
   First_move="";
   Serial.println("yes");
   tmrpcm.play("yes.wav");
  }
 }

/*if(  ((CheckStraight(0))&&(CheckStraight(1)))&&(First_move!="i love you")  )
{
   First_move="i love you";Serial.println("i love you " );
}*/
}  


int MPU6050_read(int start, uint8_t *buffer, int size){
  int i, n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while (Wire.available() && i < size)
  {
    buffer[i++] = Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}
int MPU6050_write(int start, const uint8_t *pData, int size){
  int n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}
int MPU6050_write_reg(int reg, uint8_t data){
  int error;
  error = MPU6050_write(reg, &data, 1);
  return (error);
}
void get_mpu_data(){
   int error;
  double dT;
  accel_t_gyro_union accel_t_gyro;

  /*
    Serial.println(F(""));
    Serial.println(F("MPU-6050"));
  */

  // Read the raw values.
  error = read_gyro_accel_vals((uint8_t*) &accel_t_gyro);

  // Get the time of reading for rotation computations
  unsigned long t_now = millis();


  float FS_SEL = 131;
  
  float gyro_x = (accel_t_gyro.value.x_gyro - base_x_gyro) / FS_SEL;
  float gyro_y = (accel_t_gyro.value.y_gyro - base_y_gyro) / FS_SEL;
  float gyro_z = (accel_t_gyro.value.z_gyro - base_z_gyro) / FS_SEL;


  // Get raw acceleration values
  //float G_CONVERT = 16384;
  float accel_x = accel_t_gyro.value.x_accel;
  float accel_y = accel_t_gyro.value.y_accel;
  float accel_z = accel_t_gyro.value.z_accel;

  // Get angle values from accelerometer
  float RADIANS_TO_DEGREES = 180 / 3.14159;
  //  float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
  float accel_angle_y = atan(-1 * accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * RADIANS_TO_DEGREES;
  float accel_angle_x = atan(accel_y / sqrt(pow(accel_x, 2) + pow(accel_z, 2))) * RADIANS_TO_DEGREES;

  float accel_angle_z = 0;

  // Compute the (filtered) gyro angles
  float dt = (t_now - get_last_time()) / 1000.0;
  float gyro_angle_x = gyro_x * dt + get_last_x_angle();
  float gyro_angle_y = gyro_y * dt + get_last_y_angle();
  float gyro_angle_z = gyro_z * dt + get_last_z_angle();

  // Compute the drifting gyro angles
  float unfiltered_gyro_angle_x = gyro_x * dt + get_last_gyro_x_angle();
  float unfiltered_gyro_angle_y = gyro_y * dt + get_last_gyro_y_angle();
  float unfiltered_gyro_angle_z = gyro_z * dt + get_last_gyro_z_angle();

  // Apply the complementary filter to figure out the change in angle - choice of alpha is
  // estimated now.  Alpha depends on the sampling rate...
  float alpha = 0.96;
  angle_x = alpha * gyro_angle_x + (1.0 - alpha) * accel_angle_x;  //define as global and fold into function
  angle_y = alpha * gyro_angle_y + (1.0 - alpha) * accel_angle_y;
  angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle

  // Update the saved data with the latest values
  set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);
  
  
  }
void print_mpu_filter_data(){
Serial.print(F("#FIL:"));             //Filtered angle
Serial.print(angle_x, 2);
Serial.print(F("/"));
Serial.print(angle_y, 2);
Serial.print(F("/"));
Serial.print(angle_z, 2);
Serial.print(F("               "));
}
void mpu_intialize(){
  Wire.begin();

  error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
  error = MPU6050_read (MPU6050_PWR_MGMT_2, &c, 1);
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
  calibrate_sensors();
  set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);
}


//======================================================================================= Flex logic ====================================================================================
bool CheckBend(int index){
  int x = ReadFlex[index];
  if(x<Fist[index]+FlexBendTolarance)
    return true;
  return false;
}
bool CheckStraight(int index){
  int x = ReadFlex[index];
  if(x>Point[index]-FlexStraightTolarance)
    return true;
  return false;
}
void CalibrateFist(){
  int Sum=0;
  for(int i=0 ; i<NumberOfCalibrations ; i++){
    Sum+= analogRead(Flex_Pin0);
  }
  Fist[0]=Sum/NumberOfCalibrations;
  Sum=0;
  
  for(int i=0 ; i<NumberOfCalibrations ; i++){
    Sum+= analogRead(Flex_Pin1);
  }
  Fist[1]=Sum/NumberOfCalibrations;
  Sum=0;
  /*
  for(int i=0 ; i<NumberOfCalibrations ; i++){
    Sum+= analogRead(Flex_Pin2);
  }
  Fist[2]=Sum/NumberOfCalibrations;*/
}

void CalibratePoint(){
  int Sum=0;
  for(int i=0 ; i<NumberOfCalibrations ; i++){
    Sum+= analogRead(Flex_Pin0);
  }
  Point[0]=Sum/NumberOfCalibrations;
  Sum=0;
  
  for(int i=0 ; i<NumberOfCalibrations ; i++){
    Sum+= analogRead(Flex_Pin1);
  }
  Point[1]=Sum/NumberOfCalibrations;
  Sum=0;
  
 /* for(int i=0 ; i<NumberOfCalibrations ; i++){
     Sum+= analogRead(Flex_Pin2);
  }
  Point[2]=Sum/NumberOfCalibrations;
  */
}
