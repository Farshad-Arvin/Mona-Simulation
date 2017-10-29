/* **********************************************************
	THIRD DRAFT OF BEECLUST ALGORITHM FOR STAGE
	AUTHOR: Sarika Ramroop
	DATE LAST UPDATED: 12/08/2017

	CHANGES FROM v2: All of the function calls and updates 
	are moved into the Robot class.
	Modified to deal with the Monas scaled 1:1 i.e. with
	a size of 65mm x 65mm

************************************************************* */

#include "stage.hh"
#include <vector>
#include <time.h>

using namespace Stg;

const bool verbose = false;

//---------------SPEED CONTROL PARAMETERS-----------------//
const double safeDist = 0.025;					//arbitrary safe distance.
const double safeSpd = 0.04;					//speed when clear of obstacles
const double turnSpd = 0.25;					//turn speed when an obstacle is detected
//--------------------------END---------------------------//

// State typedef
typedef enum { FORWARD = 1, OBSTACLE, WAIT, TURN } state_t;

//Robot class 
class Robot{
private:
	ModelPosition *pos;
	ModelRanger *ranger;			//used for obstacle detection
	ModelBlobfinder *blobfinder;	//used to determine "light intensity"
	ModelFiducial *fid;				//used for ID-ing robots
	state_t state;					//state ID
	//Wait-related variables
	unsigned int now;				//initial time. 
	unsigned int wait_time;			//wait time
	unsigned int stop_time;			//the time at which "wait" state ends
	unsigned int wait_count;		//loop count during wait state	

public:
	Robot(ModelPosition *pos) //constructor
		:	pos(pos),
			ranger((ModelRanger *)pos->GetChild("ranger:0")),
			fid((ModelFiducial *)pos->GetUnusedModelOfType("fiducial")),
			blobfinder((ModelBlobfinder *)pos->GetUnusedModelOfType("blobfinder")),
			state(FORWARD),
			now(time(NULL)),
			wait_time(0),
			stop_time(time(NULL)),
			wait_count(0)
	{
		//Need at least these models to get anything done
		assert(ranger);
		assert(fid);
		assert(blobfinder);
		
		pos->AddCallback(Model::CB_UPDATE, (model_callback_t)PositionUpdate, this);
		pos->Subscribe();	/*Need to subscribe to a module for it to be update every interation 
							in the Stage window*/
		
		ranger->AddCallback(Model::CB_UPDATE, (model_callback_t)IrUpdate, this);
		ranger->Subscribe();
		
		blobfinder->AddCallback(Model::CB_UPDATE,(model_callback_t)BlobUpdate, this);
		blobfinder->Subscribe();
		
		fid->AddCallback(Model::CB_UPDATE,(model_callback_t)FidUpdate,this);
		fid->Subscribe();
	}
	
	//Concatenate 3 vectors into one
	const std::vector<meters_t> concat(const std::vector<meters_t> &a, const std::vector<meters_t> &b, const std::vector<meters_t> &c){
		std::vector<meters_t> ret = std::vector<meters_t>();
		copy(a.begin(), a.end(), back_inserter(ret));
		copy(b.begin(), b.end(), back_inserter(ret));
		copy(c.begin(), c.end(), back_inserter(ret));
		return ret;
	}
	
	//print state if verbose = true
	void printState(state_t state){
		switch(state){
			case FORWARD:
				printf ("State: Forward\n");
				break;
			case TURN:
				printf("State: Turn\n");
				break;
			case WAIT:
				printf("State: Wait\n");
				break;
			case OBSTACLE:
				printf("State: Obstacle\n");
				break;
		}
	}

	void IrWork(){
		//get data from each IR sensor
		const std::vector<meters_t> &scan0 = ranger->GetSensors()[0].ranges;	
		const std::vector<meters_t> &scan1 = ranger->GetSensors()[1].ranges;	
		const std::vector<meters_t> &scan2 = ranger->GetSensors()[2].ranges;	
		
		//merge sensor data into one vector
		const std::vector<meters_t> scan = concat(scan0,scan1,scan2);
		
		unsigned int sample_count = scan.size();
		
		double minrange = 1e6;
		unsigned int vectpos = 0;
		bool right = true;		//turn direction
		
		if ((state == 1) || (state == 4)) 	//so program doesn't need to execute the for loop in every state
		{		
			for (uint32_t i = 0; i < sample_count; i++)
			{		
				if (scan[i]<=minrange){
					minrange = scan[i]; //sets new min value
					vectpos = i;		//position in vector with min value
				}
			}
		} //gets the min distance to an obstacle

		//only look for obstacles in the "forward" state
		if (state == 1){		
			//printf("Min range: %.2f\n",minrange);
			if (minrange < safeDist) //obstacle detected
			{ //change from forward to obstacle
				state = OBSTACLE;
				pos->Stop();
				
				if(verbose)
					printState(state);
			}
		} else if (state == 4){
			//if in turn state
			if (minrange < safeDist){ //turn if obstacle		
				if (vectpos <= (sample_count/3)){
					pos->Stop();
					pos->SetTurnSpeed(-turnSpd);
					//printf("obstacle detected! vectpos is: %u\n",vectpos);
					right = false;	//left turn
				} else if (vectpos >= (2*sample_count/3)){
					pos->Stop();
					pos->SetTurnSpeed(-turnSpd);
					right = true;	//right turn
				} else {
					if (right){
						pos->Stop();
						pos->SetTurnSpeed(-turnSpd);
					}else {
						pos->Stop();
						pos->SetTurnSpeed(turnSpd);
					}
				}
			} else{
				state = FORWARD;	//state change when clear of obstacle			
				
				if(verbose)
					printState(state);
			}
		}
	}
	
	static int IrUpdate(ModelRanger *, Robot *robot)
	{		
		robot->IrWork();	
		return 0;
	}
	
	//Blobfinder for "light intensity" measurement
	void BlobWork(){
		// get data from blobfinder
		const std::vector<ModelBlobfinder::Blob> blobscan = blobfinder->GetBlobs();
		unsigned int color_count = blobscan.size();	//vector size 
		
		//Check if obstacle is is a robot or not. 
		if (state == 1){
			pos->SetSpeed( safeSpd, 0, 0 );
		}//end forward
		else if ((state==3)&&(wait_count==1)&&(color_count>0)){
			for (int i = 0; i<color_count; i++){			
				wait_time = (blobscan[i].color.r)*255/3; /*this only gets intensity
				as soon as the state change occurs and not repeatedly.*/
				printf("%s Wait time: %u\n",pos->Token(),wait_time);
			}
			wait_count++;
		} 
		else if ((state==3)&&(wait_count==1)&&(color_count==0)){
			state = TURN; //turn from wait
			wait_time = 0;	//reset wait time
			printf("%s Wait time: %u\n",pos->Token(),wait_time);
			if (verbose)
				printState(state);
		}//if wait but no colour detected, do not wait.
		else if (state == 3){
			stop_time = now + wait_time; //wait time based
			//printf("Wait time: %u\n",wait_time);
			if (time(NULL)<stop_time){
				pos->SetSpeed(0,0,0);	//waiting
				wait_count++;
			} else {
				state = TURN; //state changed from wait to turn 
				if (verbose)
					printState(state);
			}
		}
	}
	
	//Check obstacles, or get intensity
	static int BlobUpdate( ModelBlobfinder *, Robot *robot )
	{
		robot->BlobWork();
		
		return 0;
	}	 
	
	void FidWork(){
		//get fiducial data
		std::vector<ModelFiducial::Fiducial> &fids = fid->GetFiducials();
		unsigned int fid_count = fids.size();
		
		//check for robot. Robot has ID "1"
		if ((state == 2)&&(fid_count==0)){
			state = TURN;		//obstacle is not a robot
			
			if (verbose)
				printState(state);			
		}
		else if (state == 2){		
			for (int i=0; i < fid_count; i++){
				if (fids[i].id == 1){	//robot detected; go into wait state
					now = time(NULL); 	//current time	
					state = WAIT;		//state change from obstacle to wait
					wait_count = 1;
				
					if (verbose)
						printState(state);
					
					break; //exit loop if robot found
				}
				else if ((fids[i].id != 1)&&(i > (fid_count-1))){
					state = TURN;		//no robot
					
					if (verbose)
						printState(state);
				}
			}
		}
	}
	
	
	static int FidUpdate(ModelFiducial *mod, Robot *robot)
	{
		robot->FidWork();	
		return 0;
	}
	
	Pose retPose(){ Pose pose = pos->GetPose(); }
	
	//This outputs the position of the robot.  Not really using this right now
	static int PositionUpdate(Model *mod, Robot *robot)
	{	
		robot->retPose();
		
		// if (verbose)
			// printf("Pose: [%.2f %.2f %.2f %.2f]\n", pose.x, pose.y, pose.z, pose.a);
		
		return 0;
	}
	
}; //end class Robot

// Stage calls this when the model starts up
extern "C" int Init(Model *mod, CtrlArgs *)
{
  new Robot((ModelPosition *)mod);

  return 0; // ok
}