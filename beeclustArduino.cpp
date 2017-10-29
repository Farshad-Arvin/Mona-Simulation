/*
  BEECLUST FOR ONE ROBOT
  WRITTEN BY: Sarika Ramroop
  LAST MODIFIED: 24/08/2017
*/

#include <MONA.h>
#include <TimerOne.h>

MONA mona;    //mona object

//defines
#define cruiseSpeed 100
#define safeDist 1.0
#define intensity A5

//typedefs
typedef enum { FORWARD = 1, OBSTACLE, WAIT, TURN } state_t;

//function declarations
int getLight();
void stopProxMine();

//variables
state_t state;
unsigned int wait_time;
int16_t proxData[3];     //array to store the proximity sensor data
unsigned int stop_time;
unsigned int start_time;

bool verbose = true;    //print to Serial port

//INITIALIZATION
void setup(){
  Serial.begin(9600); //baud rate
  
  mona.initMotors();
  mona.initProxSensor();
  
  //init variables
  state = FORWARD; 
  if (verbose)
    Serial.println("State: Forward");
    
  wait_time = 0;
  stop_time = 0;
  start_time = 0;
  
}

void loop(){
  if (state == 1){
    //FORWARD
    mona.readAllSensors(&proxData[0]); //get sensor data
    
    //check for obstacles
    if (proxData[0]>800 && proxData[1]>800 && proxData[2]>800)
    {
		//mona.setRMotorPWM(cruiseSpeed,FW);
		//mona.setLMotorPWM(cruiseSpeed,FW);

                //PWM to motors
                
                digitalWrite(3, 1);    //right motor direction
                digitalWrite(4, 1);    //left motor direction
  
                analogWrite(5,cruiseSpeed); //RIGHT motor
                analogWrite(6,cruiseSpeed);

    }
    else {
		state = OBSTACLE;
                Serial.println(proxData[0]);
                Serial.println(proxData[1]);
                Serial.println(proxData[2]);
                
               /* int start_rev = millis();
                int end_rev = millis();
                while((end_rev - start_rev)<50){
                  mona.setRMotorPWM(150,BW);  //reverse robot
  		  mona.setLMotorPWM(150,BW);
                  end_rev = millis();
                } */
                
		mona.setRMotorPWM(0,FW);  //stop robot
		mona.setLMotorPWM(0,FW);
                if (verbose)
                  Serial.println("State: Obstacle");
    }
  }
  else if(state == 2){
    //OBSTACLE
	stopProxMine();    //take off proximity sensors
	delay(50);  //so any IR signals can disperse befor checking
        int now = millis();
	int end_time = millis();
	while ((end_time - now) < 200){	//wait for 0.5s
		proxData[0] = analogRead(A1);	//check if IR sensors are getting signals
		proxData[1] = analogRead(A2);
                proxData[2] = analogRead(A3);
                
		if ((proxData[0]<450 && proxData[0]>50) ||(proxData[1]<450 && proxData[1]>50) || (proxData[2]<450 && proxData[2]>50)){
			//if IR received, robot detected; wait
			start_time = millis();
			state = WAIT;
                        Serial.println(proxData[0]);
                        Serial.println(proxData[1]);
                        Serial.println(proxData[2]);
                        if (verbose)
                          Serial.println("State: Wait");
			mona.initProxSensor();					//re-enable IR sensors
			wait_time = getLight()*255000/1023/3; 	//wait time in ms
                        Serial.println(wait_time);
			break;
		} else if (end_time - now >=150){ 
			state = TURN; 	//if no IR, obstacle is not a robot; turn
                        mona.initProxSensor();	
                        if (verbose)
                            Serial.println("State: Turn");
			break;
		}	
		end_time = millis();
    }
  }
  else if(state == 3){
    //WAIT
	stop_time = millis();
	if ((stop_time - start_time)< wait_time){
		mona.setRMotorPWM(0,FW);  //stop robot
		mona.setLMotorPWM(0,FW);
	}
	else{
		state = TURN;
                if (verbose)
                    Serial.println("State: Turn");
	}
    
  }
  else if(state == 4){
    //TURN
    mona.readAllSensors(&proxData[0]); //get sensor data
	
    if(proxData[0]>800 && proxData[1]>800 && proxData[2]>800){
          state = FORWARD;
          if (verbose)
            Serial.println("State: Forward");
    }
  //if the front sensor detects something, avoid fast
    else if(proxData[1]<800){
      mona.setRMotorPWM(255,BW);
      mona.setLMotorPWM(255,FW);
    }
  //if the right sensor detects somethig, avoid, but no so fast
    else if(proxData[0]<950){
      mona.setRMotorPWM(200,FW);
      mona.setLMotorPWM(255,BW);
      delay(500);
    }
  //if the left sensor detects something, also avoid duh    
    else if(proxData[2]<950){
      mona.setRMotorPWM(255,BW);
      mona.setLMotorPWM(200,FW);
      delay(500);
    }	
  }
  
}

int getLight(){
	return analogRead(intensity);
}

void stopProxMine(){
  pinMode(2, 0); //Disabel IR emiters
  digitalWrite(2,0);
}