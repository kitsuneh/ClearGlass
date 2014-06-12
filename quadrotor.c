a#include <Wire.h> // Used for I2C
#include <Adafruit_L3GD20.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345.h>
#include <HMC5883L.h>
#include <Servo.h>
#include <PID_v1.h>


#define pw_max 2300
#define pw_min 1220
#define pw_set 700

Adafruit_L3GD20 gyro;
Adafruit_ADXL345 accel = Adafruit_ADXL345(12345);
HMC5883L compass;
int iter = 1;
double angle_x, angle_y, angle_z;
double avg_x, avg_y;
unsigned long dt;
double looptime;
double anVolt, inches, cm;
double sum = 0;
double old_x, old_y;

Servo s1,s2,s3,s4;

int spin_speed_x, spin_speed_y, spin_speed_z;
double output_a, output_c, output_b, output_d, output_x, output_y;
int lower_limit = 800;
double throttle = 1000;

double input_a, input_b, input_z;
double setp = 0;
double output_spin;

//PID pa (&input_a, &output_a, &setp, 0.68, 0.3, 0.0001, DIRECT);
//PID pc (&input_a, &output_c, &setp, 0.68, 0.3, 0.0001, REVERSE);
//PID pb (&input_b, &output_b, &setp, 0.68, 0.3, 0.0001, REVERSE);
//PID pd (&input_b, &output_d, &setp, 0.68, 0.3, 0.0001, DIRECT);
PID X (&input_a, &output_x, &setp, 0.03, 0, 0.0 , DIRECT);
PID Y (&input_b, &output_y, &setp, 0.03, 0, 0.0 , REVERSE);
PID spin (&input_z, &output_spin, &setp, 5, 3, 0.01 , REVERSE);

void setup()
{
    Serial.begin(115200);

    s1.attach(22);
    s2.attach(23);
    s3.attach(24);
    s4.attach(33);
  
    s1.writeMicroseconds(700);
    s2.writeMicroseconds(700);
    s3.writeMicroseconds(700);
    s4.writeMicroseconds(700);
  
    pinMode(A1, INPUT);

    Serial.println ("Calibrating motors");
 
    delay(3500);
    Serial.println ("Calibrating motors done!");

    //Wire.begin(); //Join the bus as a master

    if (!gyro.begin(gyro.L3DS20_RANGE_500DPS))
    {
        Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
        while (1);
    }
    Serial.println ("System Initialization done!");

    if(!accel.begin())
    {
        /* There was a problem detecting the ADXL345 ... check your connections */
        Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
        while(1);
    }

    accel.setDataRate(ADXL345_DATARATE_400_HZ);

    compass = HMC5883L(); // Construct a new HMC5883 compass.
    compass.SetScale(1.3);
    compass.SetMeasurementMode(Measurement_Continuous);
    
    //pa.SetSampleTime(2);
    //pc.SetSampleTime(2);
    //pb.SetSampleTime(2);
    //pd.SetSampleTime(2);
    X.SetSampleTime(2);
    Y.SetSampleTime(2);
    X.SetMode (AUTOMATIC);
    Y.SetMode (AUTOMATIC);
    spin.SetMode (AUTOMATIC);
    spin.SetSampleTime(5);
    spin.SetOutputLimits(-120, 120);
    X.SetOutputLimits(-1,1);
    Y.SetOutputLimits(-1,1);
    Serial.println ("System Initialization done!");
}

void loop()
{
    dt = millis();
    
    char command;
    if (Serial.available())
    {
        command = Serial.read();
        if (command != '$')
            return;
        while (Serial.available())
        {
            command = Serial.read();
            if (command == 'f')
            {
                delay(1000);
            }
            if (command == 't')
            {
                //pa.SetOutputLimits(1950, 2300);
                //pc.SetOutputLimits(1950, 2300);
                //pb.SetOutputLimits(1950, 2300);
                //pd.SetOutputLimits(1950, 2300);
            }
            if (command == 's')
            {
                stopp();
                delay(1000000);
            }
        }
    }
    
    sensors_event_t event;
    accel.getEvent(&event);

    old_x = angle_x;
    old_y = angle_y;
 
    angle_x = atan2 (event.acceleration.x, sqrt (event.acceleration.z * event.acceleration.z + event.acceleration.y * event.acceleration.y)) / PI * 180;
    angle_y = atan2 (event.acceleration.y, sqrt (event.acceleration.z * event.acceleration.z + event.acceleration.x * event.acceleration.x)) / PI * 180;
    if (abs (angle_x) < 2.5)
      angle_x = 0;
    if (abs (angle_y) < 2.5)
      angle_x = 0;
    

    gyro.read();
    int spin_speed_x = gyro.data.x;
    int spin_speed_y = gyro.data.y;
    int spin_speed_z = gyro.data.z;
    
    spin_speed_z /= 2;
    input_z = spin_speed_z;
    spin.Compute();
    
    angle_x = angle_refine (old_x, angle_x, -spin_speed_y);
    angle_y = angle_refine (old_y, angle_y, -spin_speed_x);
    
    MagnetometerScaled scaled = compass.ReadScaledAxis();
    double heading = atan2(scaled.YAxis, scaled.XAxis);
    heading += 0.2132;

    if(heading < 0)
        heading += 2*PI;

    if(heading > 2*PI)
        heading -= 2*PI;

    heading = heading * 180/M_PI;

    anVolt = analogRead(1)/2;

    inches = anVolt;
    cm = inches * 2.54;

    input_a = angle_x / 5;
    //pa.Compute();
    
    //pc.Compute();
    
    input_b = angle_y / 5;
    
    //pb.Compute();
    
   // pd.Compute();
   X.Compute();
   Y.Compute();
    
    calculate_pw ();
    //update();
    
    looptime = (millis() - dt) / 1000.0;
    
    if (iter % 64 == 0)
    {
      Serial.print ("a : ");
      Serial.println (output_a);
      Serial.print ("b : ");
      Serial.println (output_b);
      Serial.print ("c : ");
      Serial.println (output_c);
      Serial.print ("d : ");
      Serial.println (output_d);
      Serial.print ("xx : ");
      Serial.println (output_x);
      Serial.print ("yy : ");
      Serial.println (output_y);
      Serial.print ("z : ");
      Serial.println (output_spin);
      Serial.print ("dt : ");
      Serial.println (looptime, 5);
      Serial.print ("x :");
      Serial.println (angle_x);
      Serial.print ("y :");
      Serial.println (angle_y);
      Serial.println();
      Serial.print("speedg:");
      Serial.println(spin_speed_y);
      if (lower_limit < 1700)
        lower_limit += 40;
      //pa.SetMode (AUTOMATIC);
      //pa.SetOutputLimits(lower_limit, 1800);
      //pc.SetMode (AUTOMATIC);
      //pc.SetOutputLimits(lower_limit, 1800);
      //pb.SetMode (AUTOMATIC);
      //pb.SetOutputLimits(lower_limit, 1800);
      //pd.SetMode (AUTOMATIC);
      //pd.SetOutputLimits(lower_limit, 1800);
      //if (lower_limit < 1780)
        //lower_limit += 40;
      //pa.SetOutputLimits(lower_limit, 1300);
      //pb.SetOutputLimits(lower_limit, 1300);
      //pc.SetOutputLimits(lower_limit, 1300);
      //pd.SetOutputLimits(lower_limit, 1300);
    }
    iter ++;
}


void stopp()
{
  s1.writeMicroseconds(700);
  s2.writeMicroseconds(700);
  s3.writeMicroseconds(700);
  s4.writeMicroseconds(700);
}


void inline update()
{  
  output_a = constrain (output_a, 900, 1300);
  output_b = constrain (output_b, 900, 1300);
  output_c = constrain (output_c, 900, 1300);
  output_d = constrain (output_d, 900, 1300);

  s1.writeMicroseconds(output_a);
  s2.writeMicroseconds(output_b);
  s3.writeMicroseconds(output_c);
  s4.writeMicroseconds(output_d);
}


void donotspin()
{
  output_a += output_spin;
  output_c += output_spin;
  output_b -= output_spin;
  output_d -= output_spin;
}

double inline angle_refine (double old_deg, double new_deg, double speed_g)
{
  return (0.96 * (old_deg + speed_g * looptime) + 0.04 * new_deg);
}

void calculate_pw ()
{
  output_a = (int) (throttle * (output_x + 1));
  output_c = (int) (throttle * 2  -  output_a);
  output_b = (int) (throttle * (output_y + 1));
  output_d = (int) (throttle * 2 - output_b);
}
  

