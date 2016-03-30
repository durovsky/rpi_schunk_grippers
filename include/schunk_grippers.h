/*********************************************************************************************//**
* @file schunk_grippers.h
* 
* Header file which provides declarations for schunk_grippers interface 
*
* Copyright (c)
* Frantisek Durovsky 
* Department of Robotics
* Technical University Kosice 
* March 2016
*   
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* *********************************************************************************************/
#ifndef SCHUNK_GRIPPERS_H
#define SCHUNK_GRIPPERS_H

#include <ros/ros.h>

#include <schunk_ezn64/get_position.h>
#include <schunk_ezn64/set_position.h>
#include <schunk_ezn64/reference.h>
#include <schunk_ezn64/acknowledge_error.h>
#include <schunk_ezn64/get_error.h>

#include <schunk_pg70/get_position.h>
#include <schunk_pg70/set_position.h>
#include <schunk_pg70/reference.h>
#include <schunk_pg70/acknowledge_error.h>
#include <schunk_pg70/get_error.h>

//OPEN & CLOSE position values adjusted to real cylinder size
static const float SCHUNK_EZN64_OPEN = 11.0;
static const float SCHUNK_EZN64_CLOSE = 0.0;
static const float SCHUNK_PG70_OPEN = 65.0;
static const float SCHUNK_PG70_CLOSE =29.0;

static const float SCHUNK_EZN64_OPEN_THRESH = 10.5;
static const float SCHUNK_PG70_OPEN_THRESH  = 64.0;	 

//Variable Declarations
static ros::ServiceClient ezn64_get_position_client;
static ros::ServiceClient ezn64_set_position_client;
static ros::ServiceClient ezn64_acknowledge_client;
static ros::ServiceClient ezn64_reference_client;

static ros::ServiceClient pg70_get_position_client;
static ros::ServiceClient pg70_set_position_client;
static ros::ServiceClient pg70_acknowledge_client;
static ros::ServiceClient pg70_reference_client;

//Schunk Grippers 
float ezn64GetPosition(void);
bool  ezn64SetPosition(float position);
bool  ezn64AcknowledgeError(void);
void  ezn64Reference(void);

float pg70GetPosition(void);
bool  pg70SetPosition(float position);
bool  pg70AcknowledgeError(void);
void  pg70Reference(void);

#endif //SCHUNK_GRIPPERS_H
