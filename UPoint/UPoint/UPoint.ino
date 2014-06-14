#include <Servo.h>

//Axemetre
// Reference the I2C Library
#include <Wire.h>
// Reference the HMC5883L Compass Library
#include <HMC5883L.h>

//GPS
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define SERIAL_SPEED 9600
#define NEEDLE_PIN 9
#define BASE_OFFSET 90


//Axemetre
// Store our compass as a variable.
HMC5883L compass;
// Record any errors that may occur in the compass.
int error = 0;
int init_angle = 0;

//GPS
TinyGPSPlus gps;
SoftwareSerial ss(2, 3);
const double EIFFEL_TOWER_LAT = 48.85826;
const double EIFFEL_TOWER_LNG = 2.294516;
//double courseTo;


//ServoMoteur needle
Servo needle;  // create NEEDLE object to control a NEEDLE
int needleAngle;

void setup()
{
  Serial.begin(SERIAL_SPEED);
  ss.begin(9600); // Use Soft Serial object to talk to GPS
  
  /*

  needleAngle = 0;
  needleStep = 10;
  needleVitesse = 2;
  needle.attach(NEEDLE_PIN);


  needle.write(0);

  Serial.println("Starting the I2C interface.");
  Wire.begin(); // Start the I2C interface.

  Serial.println("Constructing new HMC5883L");
  compass = HMC5883L(); // Construct a new HMC5883 compass.

  Serial.println("Setting scale to +/- 1.3 Ga");
  error = compass.SetScale(4.0); // Set the scale of the compass.
  if (error != 0) // If there is an error, print it out.
    Serial.print("GPS error: ");
    Serial.println(compass.GetErrorText(error));

  Serial.println("Setting measurement mode to continous.");
  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if (error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
  */
}

/*
// Our main program loop.
void loop()
{
  // Retrive the raw values from the compass (not scaled).
  MagnetometerRaw raw = compass.ReadRawAxis();
  // Retrived the scaled values from the compass (scaled to the configured scale).
  MagnetometerScaled scaled = compass.ReadScaledAxis();

  float heading = calculateHeading(raw, scaled);
  float headingDegrees = getHeadingDegrees(heading);

  // Output the data via the serial port.
  Output(raw, scaled, heading, headingDegrees);

  // Normally we would delay the application by 66ms to allow the loop
  // to run at 15Hz (default bandwidth for the HMC5883L).
  // However since we have a long serial out (104ms at 9600) we will let
  // it run at its natural speed.
  delay(1000);
}
*/

void loop()
{
  Serial.println("loop!");
  while (ss.available() > 0)
  {
    Serial.println(ss.read());
    
    gps.encode(ss.read());

    //if (gps.location.isUpdated())
    {
      Serial.println();
      Serial.println("------");
      Serial.print("Precision=");
      Serial.println(gps.hdop.value());
      Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
      Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
      Serial.print("ALT=");  Serial.println(gps.altitude.meters());
      // Serial.print("rawLAT=");  Serial.println(gps.location.rawLat().billionths);
      //Serial.print("rawLONG="); Serial.println(gps.location.rawLng().billionths);


      double distanceKm =
        gps.distanceBetween(
          gps.location.lat(),
          gps.location.lng(),
          EIFFEL_TOWER_LAT,
          EIFFEL_TOWER_LNG) / 1000.0;

      double courseTo =
        gps.courseTo(
          gps.location.lat(),
          gps.location.lng(),
          EIFFEL_TOWER_LAT,
          EIFFEL_TOWER_LNG);

      Serial.print("Distance Tour Eiffel (km) ");
      Serial.println(distanceKm);
      Serial.print("Angle pour aller à la tour Eiffel (Nord=0): ");
      Serial.println(courseTo);
      Serial.print("Directions: ");
      Serial.println(gps.cardinal(courseTo));

      //delay(1000);

    }
  }
  

  /*
  // Retrive the raw values from the compass (not scaled).
  MagnetometerRaw raw = compass.ReadRawAxis();
  // Retrived the scaled values from the compass (scaled to the configured scale).
  MagnetometerScaled scaled = compass.ReadScaledAxis();

  float heading = calculateHeading(raw, scaled);
  float headingDegrees = getHeadingDegrees(heading);

  // Output the data via the serial port.
  //DEBUG// Output(raw, scaled, heading, headingDegrees);
  // Normally we would delay the application by 66ms to allow the loop
  // to run at 15Hz (default bandwidth for the HMC5883L).
  // However since we have a long serial out (104ms at 9600) we will let
  // it run at its natural speed.
  //  delay(1000);




  // Calcul de l'angle
  needleAngle = (needleAngle + needleStep) % 180;
  //needleAngle=map(degreesToNorth(headingDegrees),0,359,0,179);
  //needleAngle=0;
  //needleAngle=courseTo;
  // Mouvement de le needle
  needle.write(needleAngle);
  // print Angle
  //Serial.print("Je suis a un needleAngle de ");
  //Serial.println(needleAngle);
  // print cycle
  //Serial.print("Je suis au needleCycle numero ");
  //Serial.println(cycle);
  
  */
  
  // Attente
  delay(500); // waits for the NEEDLE to get there
}

/*

float calculateHeading(MagnetometerRaw raw, MagnetometerScaled scaled) {

  // Values are accessed like so:
  int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)

  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(scaled.YAxis, scaled.XAxis);

  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: 2ÔøΩ 37' W, which is 2.617 Degrees, or (which we need) 0.0456752665 radians, I will use 0.0457
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.0457;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;

  return heading;


}

float getHeadingDegrees(float heading) {
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180 / M_PI;

  return headingDegrees;
}

float degreesToNorth(float headingDegrees) {
  float degree;
  if (headingDegrees > 180) {
    degree = -360 + headingDegrees;
  } else {
    degree = headingDegrees;
  }
  return degree;
}


// Output the data down the serial port.
void Output(MagnetometerRaw raw, MagnetometerScaled scaled, float heading, float headingDegrees)
{
  Serial.print("Raw:\t");
  Serial.print(raw.XAxis);
  Serial.print("   ");
  Serial.print(raw.YAxis);
  Serial.print("   ");
  Serial.print(raw.ZAxis);
  Serial.print("   \tScaled:\t");

  Serial.print(scaled.XAxis);
  Serial.print("   ");
  Serial.print(scaled.YAxis);
  Serial.print("   ");
  Serial.print(scaled.ZAxis);

  Serial.print("   \tHeading:\t");
  Serial.print(heading);
  Serial.print(" Radians   \t");
  Serial.print(headingDegrees);
  Serial.println(" Degrees   \t");
  Serial.print("DegreesToNorth \t");
  Serial.print(degreesToNorth(headingDegrees));
  Serial.println("\t");
}
*/
