/*********************************************************************************************//**
* @file motoman_grippers.cpp
*
* Main node to convert digital inputs from motoman robot controller
* to USB signals for schunk grippers
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
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AREDISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
OR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* *********************************************************************************************/

#include <gpio.h>
#include <schunk_grippers.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "motoman_grippers");
  ros::NodeHandle nh;

  //Wait until everything boots
  ros::Duration(5).sleep();

  int gpio_init_retval = 0;

  //Export pins
  if(GPIOExport(RED_LED) == -1) gpio_init_retval++;
  if(GPIOExport(GREEN_LED) == -1) gpio_init_retval++;
  if(GPIOExport(RED_BUTTON) == -1) gpio_init_retval++;
  if(GPIOExport(GREEN_BUTTON) == -1) gpio_init_retval++; 
  if(GPIOExport(DIG_INPUT_1) == -1) gpio_init_retval++;
  if(GPIOExport(DIG_INPUT_2) == -1) gpio_init_retval++; 
  
  //Safety pause
  ros::Duration(0.1).sleep();

  //Set Pin directions
  if(GPIODirection(RED_LED,OUT) == -1) gpio_init_retval++;
  if(GPIODirection(GREEN_LED, OUT) == -1) gpio_init_retval++;
  if(GPIODirection(RED_BUTTON, IN) == -1) gpio_init_retval++;
  if(GPIODirection(GREEN_BUTTON, IN) == -1) gpio_init_retval++;
  if(GPIODirection(DIG_INPUT_1, IN) == -1) gpio_init_retval++;
  if(GPIODirection(DIG_INPUT_2, IN) == -1) gpio_init_retval++;

  //If GPIO initialization failed, retry one more time
  if(gpio_init_retval != 0)
  {
    ros::Duration(5).sleep();
    gpio_init_retval = 0;

    //Export pins
    if(GPIOExport(RED_LED) == -1) gpio_init_retval++;
    if(GPIOExport(GREEN_LED) == -1) gpio_init_retval++;
    if(GPIOExport(RED_BUTTON) == -1) gpio_init_retval++;
    if(GPIOExport(GREEN_BUTTON) == -1) gpio_init_retval++; 
    if(GPIOExport(DIG_INPUT_1) == -1) gpio_init_retval++;
    if(GPIOExport(DIG_INPUT_2) == -1) gpio_init_retval++; 
  
    //Safety pause
    ros::Duration(0.1).sleep();

    //Set Pin directions
    if(GPIODirection(RED_LED,OUT) == -1) gpio_init_retval++;
    if(GPIODirection(GREEN_LED, OUT) == -1) gpio_init_retval++;
    if(GPIODirection(RED_BUTTON, IN) == -1) gpio_init_retval++;
    if(GPIODirection(GREEN_BUTTON, IN) == -1) gpio_init_retval++;
    if(GPIODirection(DIG_INPUT_1, IN) == -1) gpio_init_retval++;
    if(GPIODirection(DIG_INPUT_2, IN) == -1) gpio_init_retval++;
  }
  
  // If second attempt failed, exit
  if(gpio_init_retval != 0)
  {
    ROS_ERROR("GPIO: Not able to initialize pins: %d", gpio_init_retval);
    exit(-1);
  }

  //Wait For Services 
  ROS_INFO( "Waiting for services....");
  ros::service::waitForService("schunk_ezn64/get_position");
  ros::service::waitForService("schunk_ezn64/set_position");
  ros::service::waitForService("schunk_ezn64/acknowledge_error");
  ros::service::waitForService("schunk_ezn64/reference");

  ros::service::waitForService("schunk_pg70/get_position");
  ros::service::waitForService("schunk_pg70/set_position");
  ros::service::waitForService("schunk_pg70/acknowledge_error");
  ros::service::waitForService("schunk_pg70/reference");

  //Create service clients
  ezn64_get_position_client = nh.serviceClient<schunk_ezn64::get_position>("schunk_ezn64/get_position");
  ezn64_set_position_client = nh.serviceClient<schunk_ezn64::set_position>("schunk_ezn64/set_position");
  ezn64_acknowledge_client  = nh.serviceClient<schunk_ezn64::acknowledge_error>("schunk_ezn64/acknowledge_error");
  ezn64_reference_client    = nh.serviceClient<schunk_ezn64::reference>("schunk_ezn64/reference"); 

  pg70_get_position_client  = nh.serviceClient<schunk_pg70::get_position>("schunk_pg70/get_position");
  pg70_set_position_client  = nh.serviceClient<schunk_pg70::set_position>("schunk_pg70/set_position");
  pg70_acknowledge_client   = nh.serviceClient<schunk_pg70::acknowledge_error>("schunk_pg70/acknowledge_error");
  pg70_reference_client     = nh.serviceClient<schunk_pg70::reference>("schunk_pg70/reference");

  //Acknowledge erros if any
  bool ezn64_error_occured = ezn64AcknowledgeError();
  bool pg70_error_occured  = pg70AcknowledgeError();

  //Reference gripeers
  ROS_INFO("Press Green Button to reference EZN64 gripper: ");
  while(GPIORead(GREEN_BUTTON) == 0)
  {
    LedOn(GREEN_LED);
    ros::Duration(0.1).sleep();
    LedOff(GREEN_LED);
    ros::Duration(0.1).sleep();
  }
  LedOn(GREEN_LED);
  ezn64Reference();
   
  ROS_INFO("Press Red Button to reference PG70 gripper: ");
  while(GPIORead(RED_BUTTON) == 0)
  {
    LedOn(RED_LED);
    ros::Duration(0.1).sleep();
    LedOff(RED_LED);
    ros::Duration(0.1).sleep();
  }
  LedOn(RED_LED);
  pg70Reference();

  //Open grippers
  bool ezn64_is_open = false;  
  ROS_INFO("EZN64: Press Green Button to open grippper");
  while(GPIORead(GREEN_BUTTON) == 0)
  {
    LedOn(GREEN_LED);
    ros::Duration(0.1).sleep();
    LedOff(GREEN_LED);
    ros::Duration(0.1).sleep();   
  } 
  ezn64SetPosition(SCHUNK_EZN64_OPEN);
  
  bool pg70_is_open = false;
  ROS_INFO("PG70: Press Red button to open gripper");
  while(GPIORead(RED_BUTTON) == 0)
  {
    LedOn(RED_LED);
    ros::Duration(0.1).sleep();
    LedOff(RED_LED);
    ros::Duration(0.1).sleep();
  }
  pg70SetPosition(SCHUNK_PG70_OPEN);
  
  //Wait for feedback
  float ezn64_actual_position = ezn64GetPosition();
  float pg70_actual_position  = pg70GetPosition();

  while((ezn64_actual_position < SCHUNK_EZN64_OPEN_THRESH)
      && (pg70_actual_position < SCHUNK_PG70_OPEN_THRESH))
  {
    if(ezn64_actual_position > SCHUNK_EZN64_OPEN_THRESH)
    {
      ezn64_is_open = true;
      LedOn(GREEN_LED);
    }

    if(pg70_actual_position > SCHUNK_PG70_OPEN_THRESH) 
    {
      pg70_is_open = true;
      LedOn(RED_LED);
    }
      
    ezn64_actual_position = ezn64GetPosition();
    pg70_actual_position  = pg70GetPosition();

    ros::Duration(1).sleep();
  }  
  

  //============================================================
  //Main Loop
  //============================================================

  while(1){
    if((GPIORead(GREEN_BUTTON) == 1) && (ezn64_is_open == false))
    {
      ezn64SetPosition(SCHUNK_EZN64_OPEN);
      LedOn(GREEN_LED);
      ezn64_is_open = true;
    } 
    
    if((GPIORead(GREEN_BUTTON) == 0) && (ezn64_is_open == true))
    {
      ezn64SetPosition(SCHUNK_EZN64_CLOSE);
      LedOff(GREEN_LED);
      ezn64_is_open = false;
    }
    
    if((GPIORead(RED_BUTTON) == 1) && (pg70_is_open == false))
    {
      pg70SetPosition(SCHUNK_PG70_OPEN);
      LedOn(RED_LED);
      pg70_is_open = true;
    }   

    if((GPIORead(RED_BUTTON) == 0) && (pg70_is_open == true))
    {
      pg70SetPosition(SCHUNK_PG70_CLOSE);
      LedOff(RED_LED);
      pg70_is_open = false;
    }

    ros::Duration(0.1).sleep();
  }

  return EXIT_SUCCESS;
}
