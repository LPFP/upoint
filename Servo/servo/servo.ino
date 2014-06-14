#include <Servo.h>

#define SERIAL_SPEED 9600
#define SERVO_PIN 9
#define BASE_OFFSET 90

Servo myservo;  // create servo object to control a servo

int angle;
int step;
int cycle;
int vitesse;
// variable to read the value from the analog pin

void setup()
{



    angle=0;
    step=10;
    cycle=0;
    vitesse=2;


    myservo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
    Serial.begin(SERIAL_SPEED);
    Serial.print(" Position initiale : ");
    Serial.println(angle);
    Serial.println("Je retourne a 0");
    myservo.write(0);
    Serial.println("Je vais commencer a tourner");
}

void loop()
{
    cycle=cycle+1;
    angle=(angle+step)%180;
    myservo.write(angle);
    Serial.print("Je suis a un angle de ");
    Serial.println(angle);
    Serial.print("Je suis au cycle numero ");
    Serial.println(cycle);
    delay(vitesse*1000/180*step);                           // waits for the servo to get there
}
