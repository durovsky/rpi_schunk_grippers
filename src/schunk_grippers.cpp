/*********************************************************************************************//**
* @file schunk_grippers.cpp
* 
* Sources to provides service calls for schunk_grippers 
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

#ifndef SCHUNK_GRIPPERS_CPP
#define SCHUNK_GRIPPERS_CPP

#include <schunk_grippers.h>

float ezn64GetPosition(void)
{
  schunk_ezn64::get_position ezn64_srv;
  
  float response;
  if(ezn64_get_position_client.call(ezn64_srv))
    response = ezn64_srv.response.actual_position;
  else
    ROS_ERROR("EZN64: Not able to get actual position");

  return(response);
}

bool  ezn64SetPosition(float position)
{
  schunk_ezn64::set_position ezn64_srv;
  ezn64_srv.request.goal_position = position;
  
  bool response;
  if(ezn64_set_position_client.call(ezn64_srv))
    response = ezn64_srv.response.goal_accepted;  
  else
    ROS_ERROR("EZN64: Not able to call set_position service");
  
  return(response);
}

bool  ezn64AcknowledgeError(void)
{
  schunk_ezn64::acknowledge_error ezn64_srv;
  
  bool response;
  if(ezn64_acknowledge_client.call(ezn64_srv))
    response = ezn64_srv.response.acknowledge_response;
  else
    ROS_ERROR("EZN64: Not able to acknowledge_error");

  return(response);
}

void  ezn64Reference(void)
{
  schunk_ezn64::reference ezn64_srv;
   
  if(ezn64_reference_client.call(ezn64_srv))
    ROS_INFO("EZN64: Referencing");
  else
    ROS_ERROR("EZN64: Not able to reference gripper");
}

float pg70GetPosition(void)
{
  schunk_pg70::get_position pg70_srv;
  
  float response;
  if(pg70_get_position_client.call(pg70_srv))
    response = pg70_srv.response.actual_position;
  else
    ROS_ERROR("PG70: Not able to get actual position");

  return(response);
}

bool pg70SetPosition(float position)
{
  schunk_pg70::set_position pg70_srv;
  pg70_srv.request.goal_position = position;
  pg70_srv.request.goal_velocity = 40;
  pg70_srv.request.goal_acceleration = 40;
  
  bool response;
  if(pg70_set_position_client.call(pg70_srv))
    response = pg70_srv.response.goal_accepted;  
  else
    ROS_ERROR("PG70: Not able to call set_position service");
  
  return(response); 
}

bool pg70AcknowledgeError(void)
{
  schunk_pg70::acknowledge_error pg70_srv;
  
  bool response;
  if(pg70_acknowledge_client.call(pg70_srv))
    response = pg70_srv.response.acknowledge_response;
  else
    ROS_ERROR("PG70: Not able to acknowledge_error");

  return(response);
}

void pg70Reference(void)
{
  schunk_pg70::reference pg70_srv;
   
  if(pg70_reference_client.call(pg70_srv))
    ROS_INFO("PG70: Referencing");
  else
    ROS_ERROR("PG70: Not able to reference gripper");
}

#endif //SCHUNK_GRIPPERS_CPP
