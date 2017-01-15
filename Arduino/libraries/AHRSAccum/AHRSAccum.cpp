#include "AHRSAccum.h"

bool AHRSAccum::setCurrentAngle(float angle){
	if(!((angle >= -180) && (angle <= 180))){
		return false;
	}

	this->last = this->now;

	if(angle >= -180 && angle < -90){
		this->now = 3;
	}else if(angle >= -90 && angle < 0){
		this->now = 1;
	}else if(angle >= 0 && angle < 90){
		this->now = 2;
	}else if(angle >= 90 && angle <= 180){
		this->now = 4;
	}
	this->angle = angle;
	calcAccum();
	return true;
}

void AHRSAccum::calcAccum(){
	//Check for any crossovers
	if(this->last == 1 && this->now == 2 && this->crossovers != 0){
		this->crossovers++;
	}else if(this->last == 2 && this->now == 1 && this->crossovers != 0){
		this->crossovers--;
	}else if(this->last == 4 && this->now == 3){
		crossovers++;
	}else if(this->last == 3 && this->now == 4){
		crossovers--;
	}
	
	//Calculate accumulated angle
	int crossoverSign = (this->crossovers > 0) - (this->crossovers < 0); //(true-false=1-0=1) (false-true=0-1=-1)
	int angleSign = (this->angle > 0) - (this->angle < 0);
		
	if(crossoverSign == angleSign){
		this->accumulated = (180 * this->crossovers) + this->angle;
	}else if(angleSign == 0 && crossoverSign == 1){
		this->accumulated = (180 * this->crossovers) + this->angle;
	}else if(angleSign == 0 && crossoverSign == -1){
		this->accumulated = (180 * this->crossovers) + (-180 - this->angle);
	}else if(crossoverSign == 1 && angleSign == -1){
		this->accumulated = (180 * this->crossovers) + fabs(-180 - this->angle);
	}else if(crossoverSign == -1 && angleSign == 1){
		this->accumulated = (180 * this->crossovers) - fabs(180 - this->angle);
	}else if(crossoverSign == 0){
		this->accumulated = angle;
	}
}

void AHRSAccum::reset(){
	this->crossovers = 0;
	this->last = 0;
	this->now = 0;
	this->angle = 0;
}

float AHRSAccum::getCurrentAngle(){
	return this->angle;
}

float AHRSAccum::getAccumulatedAngle(){
	return this->accumulated;
}