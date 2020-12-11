//Robert McQueens UR2 Final Project code
#include <Servo.h>
#include <math.h>

//Diagnosis setup
const int ledPin = 13; // the pin that the LED is attached to

//Magnet setup
const int MagPin = 12;//Enable magnet pin on or off

//Stepper setup
const int stepPin = 3; //Stepper motor steping pin using standard full step mode
const int directionPin = 4; //Direction pin forward or backwards pin for stepper
double RackExtension = 0; //For value of desired rack extension past zero location
const int StepsPerQInch = 23; //Steps per inch of rack travel
//Stepper will start at the same location everytime for each run will not need to send it to a start location

//Servo setups 
const int BaseServoPin = 8; //Base Servo Pin Control
const int MagServoPin = 7; // Magnet servo Pin control
Servo BaseServo; //name base servo using library
Servo MagServo; //name magnet servo using library
double BaseAngle = 0; //Angle of the base Servo to start
double MagAngle = 0; //Angle of the Manget Servo to start

//Setup for Serial communication and byte reading from C# code
const byte buffSize = 40;
unsigned int inputBuffer[buffSize]; 
const char startMarker = '<'; 
const char endMarker = '>';
byte bytesRecvd = 0; 
boolean readInProgress = false; 
boolean newDataFromPC = false; 
byte coordinates[3]; 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup()
{
  Serial.begin(115200);   
  pinMode(ledPin, OUTPUT); //Pin 13 led on and off output pin for diagnosis
  pinMode(MagPin, OUTPUT); //Manget pin 12 output pin on off style
  pinMode(stepPin,OUTPUT); //stepper motor pin output step pin 3 
  pinMode(directionPin,OUTPUT); //stepper motor pin output direction pin 4
  pinMode(BaseServoPin, OUTPUT); // baser servo pin output pin 8 
  pinMode(MagServoPin, OUTPUT); // magnet servo pin output pin 7
  BaseServo.attach(8); //attach base servo to pin 8
  MagServo.attach(7); // attach magnet servo to pin 7 
  //Send Servo to zero
  for (MagAngle; MagAngle >= 0; MagAngle -= 1) // goes from 0 degrees to 180 degrees with a step og 5 degree
  { 
    MagServo.write(MagAngle);// rotates the servo to rotate at specific angle
    delay(25);  // adding delay of 25 msec
  }
  //Send Servo to zero 
  for (BaseAngle; BaseAngle >= 0; BaseAngle -= 1) // goes from 0 degrees to 180 degrees with a step og 5 degree
  { 
    BaseServo.write(BaseAngle);// rotates the servo to rotate at specific angle
    delay(100);  // adding delay of 100 msec
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop() 
{
  //get coordinates from C# code
  getDataFromPC(); 
 
  if(newDataFromPC)
  {     
    sendSuspendCmd();
    digitalWrite(ledPin, HIGH);

    //Send coordinates from webcam for editing in reference to robot location
    RobotCoordinatesEdit();
    delay (10000);            
  } 
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void sendSuspendCmd()
{ 
  // send the suspend-false command 
  //Let C# know its still waiting on robot movement
  Serial.println("<S>"); 
} 


 
void sendEnableCmd()
{ 
  // send the suspend-true command 
  //Let C# know to send next shape coordinates
  Serial.println("<P>"); 
} 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


 
// alternative to the readBytes function: 
void getDataFromPC() 
{ 
  // receive data from PC and save it into inputBuffer 
  
  if(Serial.available() > 0) 
  { 
 
    char x = Serial.read(); 
 
    // the order of these IF clauses is significant 
    
    if (x == endMarker)
    {      
      readInProgress = false;       
      newDataFromPC = true;       
      inputBuffer[bytesRecvd] = 0;       
      coordinates[0] = inputBuffer[0];      
      coordinates[1] = inputBuffer[1]; 
      coordinates[2] = inputBuffer[2];
    } 
  
    if(readInProgress) 
    {       
      inputBuffer[bytesRecvd] = x;      
      bytesRecvd ++;      
      if (bytesRecvd == buffSize) 
      {        
        bytesRecvd = buffSize - 1; 
      } 
    } 
 
    if (x == startMarker) 
    {        
      bytesRecvd = 0;        
      readInProgress = true; 
    } 
  } 
  
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////
///////ALL COORDINATES BASED ON 1/4 INCH INCREMENTS////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

//Robot
void RobotCoordinatesEdit()
{
  //enter coordinates into a matrix
  //I will be using this to move from camera orgin to the robot orgin
  double x = coordinates[0];
  double y = coordinates[1];
    Serial.print("< ");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y); 
  double CameraCoordinates[4][1] = {{x},
                                   {y},  
                                   {0},
                                   {1}};  
    
  //Transformation matrix of robot from camera frame to the robot orgin                               
  double TranformationMatrix[4][4] ={{1, 0, 0, -13.5},//x offset
                                    {0, -1, 0, 55},//y offset
                                    {0, 0, -1, 0},
                                    {0, 0, 0, 1}};
   
  //Matrix for new coordinates from robots location                                 
  double CoordinatesfromRobot[4][1]= {{0},
                                     {1}, 
                                     {2},
                                     {4}};
   
  //For loop to multiply matricies and store them in new robot matrix                       
  for (int i = 0 ; i < 4 ; i++)
  {
    CoordinatesfromRobot[i][0] = CameraCoordinates[0][0]*TranformationMatrix[i][0]
                                 +CameraCoordinates[0][1]*TranformationMatrix[i][1]
                                 +CameraCoordinates[0][2]*TranformationMatrix[i][2]
                                 +CameraCoordinates[0][3]*TranformationMatrix[i][3];
  }
      
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //At this point I have my coordinates from the robots orgin and we can move to them to get the shapes
  //Need to convert this to polar form based on robot configuration
  //This will allow us to turn the base Servo to an angle and extend the rack to the desired distance to reach the shapes
  
  //extract coordinates from matrix
  double x0 = CoordinatesfromRobot[0][0];
  double y0 = CoordinatesfromRobot[0][1];
  
  //find base angle and rack extension values
  BaseAngle = atan(y0/x0);   
  Serial.print(BaseAngle);
  Serial.print(",");
  //Covert from rad to degrees
  BaseAngle = (BaseAngle * 180.0)/M_PI;  
  Serial.print(",");
  Serial.print(BaseAngle);
  
  //distance of hyp length
  RackExtension = sqrt ((x0*x0) + (y0*y0));
 
/////Code is correct to this line/////////////////////////////////////////////////////////////////////////////////////////

  // Send base to correct angle
  //if base angle is negitve have to adjust for that 
  if (x < 22)
  {
    if (BaseAngle < 0)
    {
    BaseAngle = 180+(BaseAngle);
    }
  }
  else if (x>22)
  {
    {
      BaseAngle = BaseAngle + 5;
    }
  }
  Serial.print(" ,");
  Serial.print(BaseAngle); 
  for (int ang1 = 0; ang1 <= BaseAngle; ang1 += 1) // goes from 0 degrees to 180 degrees with a step og 5 degree
  { 
    BaseServo.write(ang1);// rotates the servo to rotate at specific angle
    delay(50);  // adding delay of 100 msec
  }

  //Send rack to location
  //Adjust for starting extension
  double R = RackExtension - 13;                                  
  //Convert to steps
  double z = R*StepsPerQInch;
  Serial.print(" ,");
  Serial.print(R);
  Serial.println(">");
  for (int x=0; x < z; x++) //loop to continously step
  {
    digitalWrite(directionPin,HIGH); //Enable direction pin (Extend)
    digitalWrite (stepPin,HIGH); //Step forward 1
    delayMicroseconds(2000); // slows motor down the higher the number
    digitalWrite (stepPin,LOW); //Step forward 1 again
    delayMicroseconds(2000); //slows motor down the higher the number
  }
  
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Currently at the location of the shape
  //Lets turn magnet on first
  digitalWrite(MagPin,HIGH);
  
  //Swing Magnet across the shape slowly to grab it
  for ( MagAngle = 0; MagAngle <= 180; MagAngle += 1) // goes from 0 degrees to 180 degrees with a step og 5 degree
  { 
    MagServo.write(MagAngle);// rotates the servo to rotate at specific angle
    delay(25);  // adding delay of 25 msec
  }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Now we have the shape in the location of the shape
  //Retract the rack and move to the correct side of the base and drop the shape
  //Retract same distance as extended
  for (int x=0; x < z; x++) //loop to continously step
  {
    digitalWrite(directionPin,LOW); //Enable direction pin (Extend)
    digitalWrite (stepPin,HIGH); //Step forward 1
    delayMicroseconds(2000); // slows motor down the higher the number
    digitalWrite (stepPin,LOW); //Step forward 1 again
    delayMicroseconds(2000); //slows motor down the higher the number
  }

  //Shape byte from C# 
  int shapeSort = coordinates[2];
  if (shapeSort == 0)
  {
    //Square
    for (BaseAngle; BaseAngle >= 0; BaseAngle -= 1) // goes from 0 degrees to 180 degrees with a step og 5 degree
    { 
      BaseServo.write(BaseAngle);// rotates the servo to rotate at specific angle
      delay(50);  // adding delay of 100 msec
    }
    
    //magnet faces forward drops shape infront of line
    for (MagAngle; MagAngle >= 130; MagAngle -= 1) // goes from 0 degrees to 180 degrees with a step og 5 degree
    { 
        MagServo.write(MagAngle);// rotates the servo to rotate at specific angle
        delay(25);  // adding delay of 25 msec
    }
    
    //Drop Shape
    digitalWrite(MagPin,LOW);
    
    //Rotate MagServo to Stow Position
    for (MagAngle; MagAngle >= 0; MagAngle -= 1) // goes from 0 degrees to 180 degrees with a step og 5 degree
    { 
        MagServo.write(MagAngle);// rotates the servo to rotate at specific angle
        delay(25);  // adding delay of 25 msec
    }
  }
  
  else if (shapeSort == 1) 
  {
    //Triangle 
    for (BaseAngle; BaseAngle <= 180; BaseAngle += 1) // goes from 0 degrees to 180 degrees with a step og 5 degree
    { 
      BaseServo.write(BaseAngle);// rotates the servo to rotate at specific angle
      delay(50);  // adding delay of 100 msec
    }
    //Drop Shape
    digitalWrite(MagPin,LOW);
    
    //Rotate MagServo to Stow Position
    for (MagAngle; MagAngle >= 0; MagAngle -= 1) // goes from 0 degrees to 180 degrees with a step og 5 degree
    { 
        MagServo.write(MagAngle);// rotates the servo to rotate at specific angle
        delay(25);  // adding delay of 25 msec
    }
    
    //Rotate base to the right to stow position
    for (BaseAngle; BaseAngle >= 0; BaseAngle -= 1) // goes from 0 degrees to 180 degrees with a step og 5 degree
    { 
      BaseServo.write(BaseAngle);// rotates the servo to rotate at specific angle
      delay(50);  // adding delay of 100 msec
    }
  }
  else 
  {
    //Not identified shape drop it
    digitalWrite(MagPin,LOW);
    //Send error LED Code
    digitalWrite(ledPin,LOW);
    delay(1000);
    digitalWrite(ledPin,HIGH);
    delay(1000);
    digitalWrite(ledPin,LOW);
    delay(1000);
    digitalWrite(ledPin,HIGH);
    delay(1000);
    digitalWrite(ledPin,LOW);
    delay(1000);
    digitalWrite(ledPin,HIGH);
    delay(1000);
  }
  //Incase we didnt make it there rotate to home anyways
  
   //Rotate MagServo to Stow Position
    for (MagAngle; MagAngle >= 0; MagAngle -= 1) // goes from 0 degrees to 180 degrees with a step og 5 degree
    { 
        MagServo.write(MagAngle);// rotates the servo to rotate at specific angle
        delay(25);  // adding delay of 25 msec
    }
    
    //Rotate base to the right to stow position
    for (BaseAngle; BaseAngle >= 0; BaseAngle -= 1) // goes from 0 degrees to 180 degrees with a step og 5 degree
    { 
      BaseServo.write(BaseAngle);// rotates the servo to rotate at specific angle
      delay(50);  // adding delay of 100 msec
    }


  //Ask for next coordinate, return to waiting state
  sendEnableCmd();
  digitalWrite(ledPin, LOW); 
  newDataFromPC = false; 
}
