// link between the computer and the SoftSerial Shield
//at 9600 bps 8-N-1
//Computer is connected to Hardware UART
//SoftSerial Shield is connected to the Software UART:D2&D3


#include <SoftwareSerial.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;
SoftwareSerial ss(2, 3);

const double EIFFEL_TOWER_LAT = 48.85826;
const double EIFFEL_TOWER_LNG = 2.294516;



void setup()
{
  Serial.begin(9600); // for debugging
  ss.begin(9600); // Use Soft Serial object to talk to GPS
}


void loop()
{
  while (ss.available() > 0)
  {
    Serial.print(".");
    gps.encode(ss.read());
    

    
    if (gps.location.isUpdated())
    {
    Serial.println();  
    Serial.println("------");
    Serial.print("Precision=");
    Serial.println(gps.hdop.value());
    Serial.print("LAT=");  Serial.println(gps.location.lat(),6);
    Serial.print("LONG="); Serial.println(gps.location.lng(),6);
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
    
    //delay(10);
    
    }
  }
}
