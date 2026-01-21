
#include <Wire.h>
#include <MPU6050_light.h>

// Control values
const int TS = 20; //Milliseconds

//State feedback matrix K*(angle, w_housing, w_flying wheel)
double K_EDGE[3] = {-33.7027,-14.2271,-0.195372}; // Values from Matlab, dependent on TS

// Other constant values
MPU6050 mpu(Wire);
unsigned long timer = 0;

// Custom functions
template<typename T, int N>
double dot_product(T (&a1)[N], T (&a2)[N])
{
    double product = 0;
    for(int i=0;i<N;i++)
        product += a1[i] * a2[i];
    return product;
}

void init_mpu()
{
  // Disable sleep and self test for MPU
  Wire.begin();
  Wire.beginTransmission(0b1101000); 
  Wire.write(0x6B);                  
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
}

void loop()
{
  mpu.update();
  if ((millis() - timer) > TS)
  { 
    // Get all sensor values from MPU6050 & Actuator
    mpu.getAccX(&ax, &ay, &az);
    mpu.getAccY
    mpu.

    // Convert the raw accelerometer data to "g" (assuming the sensor is in 2g mode)
    ax /= 16384.0; // Scale factor for 2g mode
    ay /= 16384.0;
    az /= 16384.0;

    // Calculate roll (in radians)
    roll = atan2(ay, az);

    // Calculate pitch (in radians)
    pitch = atan2(-ax, sqrt(ay * ay + az * az));

    // Optionally convert to degrees
    float roll_deg = roll * 180.0 / PI;
    float pitch_deg = pitch * 180.0 / PI;

    // Output the results in radians
    Serial.print("Roll (degrees): ");
    Serial.println(roll_deg);
    
    Serial.print("Pitch (degrees): ");
    Serial.println(pitch_deg);

    timer = millis();
  }
}

