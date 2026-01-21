
#include <Wire.h>
#include <MPU6050_light.h>

// Control values
const int TS = 200; //Milliseconds

//State feedback matrix K*(angle, w_housing, w_flying wheel)
double K_EDGE[3] = {-33.7027,-14.2271,-0.195372}; // Values from Matlab, dependent on TS

// Other constant values
MPU6050 mpu(Wire);
unsigned long timer = 0;
double angles[2];
double gyros[3];
const int MPU = 0x68;
float xa,ya,za,xg,yg,zg;
int t;

// Custom functions
template<typename T, int N>
double dot_product(T (&a1)[N], T (&a2)[N])
{
    double product = 0;
    for(int i=0;i<N;i++)
        product += a1[i] * a2[i];
    return product;
}

void getAngles(double (&angles)[2]) {
    // function to get the roll and the pitch of the MPU, applied in that order
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  //send starting register address, accelerometer high byte
    Wire.endTransmission(false); //restart for read
    Wire.requestFrom(MPU, 6, true); //get six bytes accelerometer data
    t = Wire.read();
    xa = (t << 8) | Wire.read();
    t = Wire.read();
    ya = (t << 8) | Wire.read();
    t = Wire.read();
    za = (t << 8) | Wire.read();

    angles[0] = atan2(ya , za) * 180.0 / PI; //roll
    angles[1] = atan2(-xa , sqrt(ya * ya + za * za)) * 180.0 / PI; //pitch
}

void getGyros(double (&gyros)[3]) {
    // function to get he raw gyro values from MPU
    Wire.beginTransmission(MPU);
    Wire.write(0x43);  //send starting register address, gyro high byte
    Wire.endTransmission(false); //restart for read
    Wire.requestFrom(MPU, 6, true); //get six bytes gyro data
    t = Wire.read();
    xg = (t << 8) | Wire.read();
    t = Wire.read();
    yg = (t << 8) | Wire.read();
    t = Wire.read();
    zg = (t << 8) | Wire.read();

    gyros[0] = xg;
    gyros[1] = yg;
    gyros[2] = zg;
}

void init_mpu()
{
  // Disable sleep and self test for MPU
  Wire.begin();
  Wire.beginTransmission(MPU); 
  Wire.write(MPU);                  
  Wire.write(0b00000000);            
  Wire.endTransmission();            
}

void verify_status_mpu(byte &status)
{
  // Verify status of mpu and stop program if not ok
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0)
  {
  } // Terminate program by staying in loop
}

void calc_offsets_mpu()
{
  // Calculate offsets of MPU and write to settings in class
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
}

void setup()
{
  // Begin transmitting with serial
  Serial.begin(115200);

  // Init MPU and check status
  init_mpu();

  // Configure mpu by begin, with the sensitivity
  byte status = mpu.begin(0, 0);
  
  // Check status mpu and stop program if not ok
  verify_status_mpu(status);

  // Calculate offsets
  calc_offsets_mpu();

  // Set filter value (high is more gyro, lower is more acce)
  mpu.setFilterGyroCoef(0.91);
}

void loop()
{
  mpu.update();
  if ((millis() - timer) > TS)
  { 

    getAngles(angles); // Calculate angles manually using acceleratometer only

    // Get all sensor values from MPU6050 & Actuator
    double phi_hat = angles[0]*PI/180; //Result is in degrees -> rad
    double wh = mpu.getGyroX()*PI/180; //This is in degrees/second --> rad/second
    double ww = 0.0; //TODO

    // Define state vector
    double cubli_ss[3] = {phi_hat, wh, ww};

    // Define new value for driver
    double u = dot_product<double, 3>(K_EDGE, cubli_ss);


    Serial.print("u : ");
    Serial.print(u);
    Serial.print("\tmpu Angle: ");
    Serial.print(mpu.getAccAngleX());
    Serial.print("\tangles[0] : ");
    Serial.print(angles[0]);
    Serial.print("\tGyroX : ");
    Serial.println(gyros[0]);
    timer = millis();
  }
}

