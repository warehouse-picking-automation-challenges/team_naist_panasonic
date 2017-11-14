/*
 * Version:  2017.07.31
 * Authors:  Members of the Team NAIST-Panasonic at the Amazon Robotics Challenge 2017:
 *           Gustavo A. Garcia R. <garcia-g at is.naist.jp> (Captain), 
 *           Lotfi El Hafi, Felix von Drigalski, Wataru Yamazaki, Viktor Hoerig, Arnaud Delmotte, 
 *           Akishige Yuguchi, Marcus Gall, Chika Shiogama, Kenta Toyoshima, Pedro Uriguen, 
 *           Rodrigo Elizalde, Masaki Yamamoto, Yasunao Okazaki, Kazuo Inoue, Katsuhiko Asai, 
 *           Ryutaro Futakuchi, Seigo Okada, Yusuke Kato, and Pin-Chu Yang
 *********************
 * Copyright 2017 Team NAIST-Panasonic 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at 
 *     http://www.apache.org/licenses/LICENSE-2.0 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************
 */

#include "tnp_end_effector.h"


EndEffectorNode::EndEffectorNode()
{
  //Topic you want to publish
  pub_arduino_write_ = n_.advertise<std_msgs::String>("tnp_arduino/write", 100);
  pub_item_is_suctioned_ = n_.advertise<std_msgs::Int16>("tnp_end_effector/itemIsSuctioned", 100);
  pub_item_is_gripped_ = n_.advertise<std_msgs::Int16>("tnp_end_effector/itemIsGripped", 100);
  pub_suction_is_contact_ = n_.advertise<std_msgs::Int16>("tnp_end_effector/cupIsTouchingSomething", 100);
  pub_linear_actuator_gripper_ = n_.advertise<std_msgs::Int16>("tnp_end_effector/linearActuatorGripper", 100);
  pub_linear_actuator_suction_ = n_.advertise<std_msgs::Int16>("tnp_end_effector/linearActuatorSuction", 100);
  pub_gripper_open_close_ = n_.advertise<std_msgs::Int16>("tnp_end_effector/gripperOpenClose", 100);
  pub_gripper_is_contact_ = n_.advertise<std_msgs::Int16>("tnp_end_effector/gripperIsTouchingSomething", 100);
  pub_drawer_status_ = n_.advertise<std_msgs::Int16>("tnp_end_effector/drawerStatus", 100);
  pub_suction_on_off_ = n_.advertise<std_msgs::Int16>("tnp_end_effector/suctionOnOff", 100);
  pub_blower_ = n_.advertise<std_msgs::Int16>("tnp_end_effector/blowerStatus", 100);

  //Topic you want to subscribe
  sub_arduino_read_ = n_.subscribe("tnp_arduino/read", 1, &EndEffectorNode::ArduinoReadCallback, this);

  // services you want to advertise
  SuctionService = n_.advertiseService("tnp_end_effector/suction", &EndEffectorNode::SuctionCallback, this);
  LinActuatorSuctionService = n_.advertiseService("tnp_end_effector/lin_actuator_suction",
                                                  &EndEffectorNode::LinActuatorSuctionCallback, this);
  GripperService = n_.advertiseService("tnp_end_effector/gripper", &EndEffectorNode::GripperCallback, this);
  LinActuatorGripperService = n_.advertiseService("tnp_end_effector/lin_actuator_gripper",
                                                  &EndEffectorNode::LinActuatorGripperCallback, this);
  BlowerService = n_.advertiseService("tnp_end_effector/blower", &EndEffectorNode::BlowerCallback, this);
  DrawerService = n_.advertiseService("tnp_end_effector/drawer", &EndEffectorNode::DrawerCallback, this);
  EEControlService = n_.advertiseService("tnp_end_effector/set_ee_status", &EndEffectorNode::EEControlCallback, this);
  SuctionStatusService = n_.advertiseService("tnp_end_effector/suction_status", &EndEffectorNode::SuctionStatusCallback, this);
}

//Helpers

/* The readme.md of the tnp_end_effector has the commands details*/
bool EndEffectorNode::SendCommandToArduino(const std::string& arduinoCommand)
{
  ROS_DEBUG("Sending command to Arduino. Wait until the connection is completed...");
  ros::Rate loop_rate(100);
  while (pub_arduino_write_.getNumSubscribers() == 0)
  {
    loop_rate.sleep();
    ros::spinOnce(); //testing if this make the wait until connection not to get stuck
  }

  std_msgs::String msg;
  msg.data = arduinoCommand;

  // once connection is ok, send the command
  if (ros::ok())
  {
    pub_arduino_write_.publish(msg);
    ros::spinOnce();
  }
  else
  {
    ROS_INFO("ros not ok. Tried sending: %s", msg.data.c_str());
    return false;
  }

  ROS_DEBUG("Sent: %s", msg.data.c_str());
  return true;
}

// Topics callbacks

void EndEffectorNode::ArduinoReadCallback(const std_msgs::StringConstPtr& input)
{

  ROS_DEBUG("ArduinoReadCallback called");
  std::string eestatus = input->data.c_str();//eestatus is the data of all from Arduino

  std::list<int> status_changes;
  std_msgs::Int16 msg;
  int flag_drawer = 0;
  const std::string state_Z("0");
  const std::string state_A("1");

  for(int i=8;i<=18;i++){
    if(eestatus[i] != old_eestatus[i]){
      status_changes.push_back(i);
    }
  }

  for(auto itr = status_changes.begin(); itr != status_changes.end(); ++itr)
  {
    int x = *itr;

    //gripper open close
    if(x == 8)
    {
      std::string eestatus_GripperOpenClose;
      eestatus_GripperOpenClose = eestatus[8];
      if (eestatus_GripperOpenClose.compare(state_Z) == 0)
      {
        msg.data = 0;
        pub_gripper_open_close_.publish(msg);
        ROS_INFO_STREAM("EndEffectorNode::ArduinoReadCallback: Gripper Open");
      }
      else if (eestatus_GripperOpenClose.compare(state_A) == 0)
      {
        msg.data = 1;
        pub_gripper_open_close_.publish(msg);
        ROS_INFO_STREAM("EndEffectorNode::ArduinoReadCallback: Gripper Close");
      }
      else
      {
        ROS_ERROR_STREAM("ArduinoReadCallback error: unexpected value: " << input);
      }
    }

    //suction on off
    else if(x == 9)
    {
      std::string eestatus_SuctionOnOff;
      eestatus_SuctionOnOff = eestatus[9];
      if (eestatus_SuctionOnOff.compare(state_Z) == 0)
      {
        msg.data = 0;
        pub_suction_on_off_.publish(msg);
        ROS_INFO_STREAM("EndEffectorNode::ArduinoReadCallback: Suction Off");
      }
      else if (eestatus_SuctionOnOff.compare(state_A) == 0)
      {
        msg.data = 1;
        pub_suction_on_off_.publish(msg);
        ROS_INFO_STREAM("EndEffectorNode::ArduinoReadCallback: Suction On");
      }
      else
      {
        ROS_ERROR_STREAM("ArduinoReadCallback error: unexpected value: " << input);
      }
    }

    //linear_actuator-gripper
    else if(x == 10)
    {
      std::string eestatus_LAGripper;
      eestatus_LAGripper = eestatus[10];
      if (eestatus_LAGripper.compare(state_Z) == 0)
      {
        msg.data = 0;
        pub_linear_actuator_gripper_.publish(msg);
        ROS_INFO_STREAM("EndEffectorNode::ArduinoReadCallback: Gripper is retracted");
      }
      else if (eestatus_LAGripper.compare(state_A) == 0)
      {
        msg.data = 1;
        pub_linear_actuator_gripper_.publish(msg);
        ROS_INFO_STREAM("EndEffectorNode::ArduinoReadCallback: Gripper is advanced");
      }
      else
      {
        ROS_ERROR_STREAM("ArduinoReadCallback error: unexpected value: " << input);
      }
    }

    //linear_actuator-suction
    else if(x == 11)
    {
      std::string eestatus_LASuction;
      eestatus_LASuction = eestatus[11];
      if (eestatus_LASuction.compare(state_Z) == 0)
      {
        msg.data = 0;
        pub_linear_actuator_suction_.publish(msg);
        ROS_INFO_STREAM("EndEffectorNode::ArduinoReadCallback: Suction cup is retracted");
      }
      else if (eestatus_LASuction.compare(state_A) == 0)
      {
        msg.data = 1;
        pub_linear_actuator_suction_.publish(msg);
        ROS_INFO_STREAM("EndEffectorNode::ArduinoReadCallback: Suction cup is advanced");
      }
      else
      {
        ROS_ERROR_STREAM("ArduinoReadCallback error: unexpected value: " << input);
      }
    }

    //drawer stuck and status
    else if(x == 12 || x == 18)
    {
      while(flag_drawer == 0){
        std::string eestatus_DrawerStuck;
        eestatus_DrawerStuck = eestatus[18];
        std::string eestatus_DrawerStatus;
        eestatus_DrawerStatus = eestatus[12];
        if (eestatus_DrawerStuck.compare(state_A) == 0)
        {
          msg.data = 2;
          pub_drawer_status_.publish(msg);
          ROS_ERROR_STREAM("EndEffectorNode::ArduinoReadCallback: Drawer Stuck!!");
          ROS_INFO("Shaking Drawer");
          ROS_DEBUG("[TNP_STATE L4] Executing tnp_end_effector/DrawerCallback Shaking the drawer");
          eecommand[9]='2';
          SendCommandToArduino(eecommand);
        }
        else if (eestatus_DrawerStatus.compare(state_Z) == 0)
        {
          msg.data = 0;
          pub_drawer_status_.publish(msg);
          ROS_INFO_STREAM("EndEffectorNode::ArduinoReadCallback: Drawer Close Complete");
        }
        else if (eestatus_DrawerStatus.compare(state_A) == 0)
        {
          msg.data = 1;
          pub_drawer_status_.publish(msg);
          ROS_INFO_STREAM("EndEffectorNode::ArduinoReadCallback: Drawer Open Complete");
        }
        else
        {
          ROS_ERROR_STREAM("ArduinoReadCallback error: unexpected value: " << input);
        }
        flag_drawer = 1;
      }
    }

    //blower
    else if(x == 13)
    {
      std::string eestatus_Blower;
      eestatus_Blower = eestatus[13];
      if (eestatus_Blower.compare(state_Z) == 0)
      {
        msg.data = 0;
        pub_blower_.publish(msg);
        ROS_INFO_STREAM("EndEffectorNode::ArduinoReadCallback: Blower Off");
      }
      else if (eestatus_Blower.compare(state_A) == 0)
      {
        msg.data = 1;
        pub_blower_.publish(msg);
        ROS_INFO_STREAM("EndEffectorNode::ArduinoReadCallback: Blower On");
      }
      else
      {
        ROS_ERROR_STREAM("ArduinoReadCallback error: unexpected value: " << input);
      }
    }

    //item_is_gripped
    else if(x == 14)
    {
      std::string eestatus_grip;
      eestatus_grip = eestatus[14];
      //ROS_DEBUG("I heard: [%s]", input->data.c_str());
      if (eestatus_grip.compare(state_Z) == 0)
      {
        // publish the item is not gripped
        msg.data = 0;
        pub_item_is_gripped_.publish(msg);
        ROS_INFO_STREAM("EndEffectorNode::ArduinoReadCallback: Item NOT gripped");
      }
      else if (eestatus_grip.compare(state_A) == 0)
      {
        // publish the item is gripped
        msg.data = 1;
        pub_item_is_gripped_.publish(msg);
        ROS_INFO_STREAM("EndEffectorNode::ArduinoReadCallback: Item GRIPPED!");
      }
      else
      {
        ROS_ERROR_STREAM("ArduinoReadCallback error: unexpected value: " << input);
      }
    }

    //item_is_suctioned
    else if(x == 15)
    {
      std::string eestatus_suction;
      eestatus_suction = eestatus[15];
      //ROS_DEBUG("I heard: [%s]", input->data.c_str());
      if (eestatus_suction.compare(state_Z) == 0)
      {
        // publish the item is not suctioned
        msg.data = 0;
        pub_item_is_suctioned_.publish(msg);
        suction_status = false;
        ROS_INFO_STREAM("EndEffectorNode::ArduinoReadCallback: Item NOT suctioned");
      }
      else if (eestatus_suction.compare(state_A) == 0)
      {
        // publish the item is suctioned
        msg.data = 1;
        pub_item_is_suctioned_.publish(msg);
        suction_status = true;
        ROS_INFO_STREAM("EndEffectorNode::ArduinoReadCallback: Item SUCTIONED!");
      }
      else
      {
        ROS_ERROR_STREAM("ArduinoReadCallback error: unexpected value: " << input);
      }
    }

    //gripper contactDetect
    else if(x == 16)
    {
      std::string eestatus_GripperContact;
      eestatus_GripperContact = eestatus[16];
      if (eestatus_GripperContact.compare(state_Z) == 0)
      {
        msg.data = 0;
        pub_gripper_is_contact_.publish(msg);
        ROS_INFO_STREAM("EndEffectorNode::ArduinoReadCallback: Gripper NOT Contact");
      }
      else if (eestatus_GripperContact.compare(state_A) == 0)
      {
        msg.data = 1;
        pub_gripper_is_contact_.publish(msg);
        ROS_INFO_STREAM("EndEffectorNode::ArduinoReadCallback: Gripper CONTACT!!");
      }
      else
      {
        ROS_ERROR_STREAM("ArduinoReadCallback error: unexpected value: " << input);
      }
    }

    //suction contactDetect
    else if(x == 17)
    {
      std::string eestatus_suction_contact;
      eestatus_suction_contact = eestatus[17];
      if (eestatus_suction_contact.compare(state_Z) == 0)
      {
        msg.data = 0;
        pub_suction_is_contact_.publish(msg);
        ROS_INFO_STREAM("EndEffectorNode::ArduinoReadCallback: Suction NOT Contact");
      }
      else if (eestatus_suction_contact.compare(state_A) == 0)
      {
        msg.data = 1;
        pub_suction_is_contact_.publish(msg);
        ROS_INFO_STREAM("EndEffectorNode::ArduinoReadCallback: Suction CONTACT!");
      }
      else
      {
        ROS_ERROR_STREAM("ArduinoReadCallback error: unexpected value: " << input);
      }
    }
    else
    {
      ROS_ERROR_STREAM("ArduinoReadCallback: Unknown value of x " << x);
    }


  }
  old_eestatus = eestatus;
}

// Service definitions

bool EndEffectorNode::SuctionCallback(tnp_end_effector::Suction::Request &req, tnp_end_effector::Suction::Response &res)
{
  ROS_DEBUG("SuctionCallback was called");

  ROS_DEBUG("setSuctionState (bool) value is %s", req.setSuctionState ? "true" : "false");
  if (req.setSuctionState == true)
  {
    ROS_INFO("Turning ON suction");
    ROS_DEBUG("[TNP_STATE L4] Executing tnp_end_effector/Suction Turning on the suction");
    std::ostringstream sout;
    sout << std::setfill('0') << std::setw(2) << req.suction_force.data;
    std::string c_para = sout.str();
    eecommand[17]=c_para[0];
    eecommand[18]=c_para[1];
    eecommand[6]='1';
    SendCommandToArduino(eecommand);
    res.succeeded = true;
  }
  else if (req.setSuctionState == false)
  {
    ROS_INFO("Turning OFF suction");
    ROS_DEBUG("[TNP_STATE L4] Executing tnp_end_effector/Suction Turning off the suction");
    eecommand[6]='0';
    SendCommandToArduino(eecommand);
    res.succeeded = true;
  }
  else
  {
    ROS_ERROR("SuctionCallback: uknown value"); //this is just for good coding practice
    ROS_DEBUG("[TNP_STATE L4] Error tnp_end_effector/Suction Error when trying to turn on/off the suction");
  }

  ROS_DEBUG("[TNP_STATE L4] Done tnp_end_effector/Suction Fuction called successfully");

  return true;
}

bool EndEffectorNode::LinActuatorSuctionCallback(tnp_end_effector::LinActuatorSuction::Request &req,
                                                 tnp_end_effector::LinActuatorSuction::Response &res)
{
  ROS_DEBUG("LinActuatorSuctionCallback was called");
  ROS_DEBUG("setLinActuatorState (bool) value is %s", req.setLinActuatorState ? "true" : "false");
  if (req.setLinActuatorState == true)
  {
    ROS_INFO("Advancing suction linear actuator");
    ROS_DEBUG("[TNP_STATE L4] Executing tnp_end_effector/LinActuatorSuction Advancing the linear actuator of suction");
    eecommand[8]='1';
    SendCommandToArduino(eecommand);
  }
  else if (req.setLinActuatorState == false)
  {
    ROS_INFO("Retracting suction linear actuator");
    ROS_DEBUG("[TNP_STATE L4] Executing tnp_end_effector/LinActuatorSuction Retracting the linear actuator of suction");
    eecommand[8]='0';
    SendCommandToArduino(eecommand);
  }
  else
  {
    ROS_ERROR("LinActuatorSuctionCallback: uknown value"); //this is just for good coding practice
    ROS_DEBUG(
        "[TNP_STATE L4] Error tnp_end_effector/LinActuatorSuction Error when trying to advance/retract linear actuator of suction");
  }

  ROS_DEBUG("LinActuatorSuctionCallback: done");
  ROS_DEBUG("[TNP_STATE L4] Done tnp_end_effector/LinActuatorSuction Function called successfully");

  return true;
}


bool EndEffectorNode::GripperCallback(tnp_end_effector::Gripper::Request &req, tnp_end_effector::Gripper::Response &res)
{
  ROS_DEBUG("GripperCallback was called");
  ROS_DEBUG("[TNP_STATE L4] Calling tnp_end_effector/Gripper Setting the gripper width/force function");

  ROS_DEBUG_STREAM("Control method is " << req.gripper_control_method.data);
  ROS_DEBUG_STREAM("Control parameter is " << req.gripper_control_parameter.data);

  const std::string force_control_str("force_control");
  const std::string position_control_str("position_control");

  if(position_control_str.compare(req.gripper_control_method.data) == 0 || req.gripper_control_method.data == "0")
  {
    ROS_ASSERT(req.gripper_control_parameter.data >= 0 && req.gripper_control_parameter.data <= 1500); //this should be millimeters
    ROS_INFO("Gripper open");
    //set parameter
    std::ostringstream sout;
    sout << std::setfill('0') << std::setw(4) << req.gripper_control_parameter.data;
    std::string c_para = sout.str();
    eecommand[12]=c_para[0];
    eecommand[13]=c_para[1];
    eecommand[14]=c_para[2];
    eecommand[15]=c_para[3];
    eecommand[5]='0';
    SendCommandToArduino(eecommand);
    res.succeeded = true;
  }
  else if(force_control_str.compare(req.gripper_control_method.data) == 0 || req.gripper_control_method.data == "1")
  {
    ROS_ASSERT(req.gripper_control_parameter.data >= 0 && req.gripper_control_parameter.data <= 1500); //this should be mN
    ROS_INFO("Gripper close");
    //set parameter
    std::ostringstream sout;
    sout << std::setfill('0') << std::setw(4) << req.gripper_control_parameter.data;
    std::string c_para = sout.str();
    eecommand[12]=c_para[0];
    eecommand[13]=c_para[1];
    eecommand[14]=c_para[2];
    eecommand[15]=c_para[3];
    eecommand[5]='1';
    SendCommandToArduino(eecommand);
    res.succeeded = true;
  }
  else
  {
    ROS_ERROR_STREAM("EndEffectorNode::GripperCallback: Unknown control method: " << req.gripper_control_method);
  }

  ROS_DEBUG("GripperCallback: done");
  ROS_DEBUG("[TNP_STATE L4] Doone tnp_end_effector/Gripper Function called successfully");

  return true;
}

bool EndEffectorNode::LinActuatorGripperCallback(tnp_end_effector::LinActuatorGripper::Request &req,
                                                 tnp_end_effector::LinActuatorGripper::Response &res)
{
  ROS_DEBUG("LinActuatorGripperCallback was called");
  ROS_DEBUG("setLinActuatorState (bool) value is %s", req.setLinActuatorState ? "true" : "false");
  if (req.setLinActuatorState == true)
  {
    ROS_INFO("Advancing Gripper");
    ROS_DEBUG(
        "[TNP_STATE L4] Executing tnp_end_effector/LinActuatorGripperCallback Advancing the linear actuator of the gripper");
    eecommand[7]='1';
    SendCommandToArduino(eecommand);
  }
  else if (req.setLinActuatorState == false)
  {
    ROS_INFO("Retracting Gripper");
    ROS_DEBUG(
        "[TNP_STATE L4] Executing tnp_end_effector/LinActuatorGripperCallback Retracting the linear actuator of the gripper");
    eecommand[7]='0';
    SendCommandToArduino(eecommand);
  }
  else
  {
    ROS_ERROR("LinActuatorGripperCallback: uknown value"); //this is just for good coding practice
    ROS_DEBUG(
        "[TNP_STATE L4] Error tnp_end_effector/LinActuatorGripperCallback Error when trying to advance/retract linear actuator of the gripper");
  }

  ROS_INFO("done");
  ROS_DEBUG("[TNP_STATE L4] Done tnp_end_effector/LinActuatorGripperCallback Function called successfully");
  return true;
}

bool EndEffectorNode::BlowerCallback(tnp_end_effector::Blower::Request &req, tnp_end_effector::Blower::Response &res)
{
  ROS_DEBUG("BlowerCallback was called");
  ROS_DEBUG("setBlowerState (bool) value is %s", req.setBlowerState ? "true" : "false");

  if (req.setBlowerState == true)
  {
    ROS_INFO("Turning ON Blower");
    ROS_DEBUG("[TNP_STATE L4] Executing tnp_end_effector/BlowerCallback Turning on the blower");
    eecommand[10]='1';
    SendCommandToArduino(eecommand);
  }
  else if (req.setBlowerState == false)
  {
    ROS_INFO("Turning OFF Blower");
    ROS_DEBUG("[TNP_STATE L4] Executing tnp_end_effector/BlowerCallback Turning off the blower");
    eecommand[10]='0';
    SendCommandToArduino(eecommand);
  }
  else
  {
    ROS_DEBUG("[TNP_STATE L4] Error tnp_end_effector/BlowerCallback Error when turning on/off the blower");
    ROS_ERROR("BlowerCallback: uknown value"); //this is just for good coding practice
  }

  ROS_DEBUG("BlowerCallback: done");
  ROS_DEBUG("[TNP_STATE L4] Done tnp_end_effector/BlowerCallback Function called successfully");
  return true;
}


bool EndEffectorNode::DrawerCallback(tnp_end_effector::Drawer::Request &req, tnp_end_effector::Drawer::Response &res)
{
  ROS_DEBUG("DrawerCallback was called");
  ROS_DEBUG("setDrawerAction value is %d", req.setDrawerAction.data);

  if (req.setDrawerAction.data == 0) // close
  {
    ROS_INFO("Closing Drawer");
    ROS_DEBUG("[TNP_STATE L4] Executing tnp_end_effector/DrawerCallback Closing the drawer");
    eecommand[9]='1';
    SendCommandToArduino(eecommand);
    res.succeeded = true;
  }
  else if (req.setDrawerAction.data == 1) // open
  {
    ROS_INFO("Opening Drawer");
    ROS_DEBUG("[TNP_STATE L4] Executing tnp_end_effector/DrawerCallback Opening the drawer");
    eecommand[9]='0';
    SendCommandToArduino(eecommand);
    res.succeeded = true;
  }
  else if (req.setDrawerAction.data == 2) // shake
  {
    ROS_INFO("Shaking Drawer");
    ROS_DEBUG("[TNP_STATE L4] Executing tnp_end_effector/DrawerCallback Shaking the drawer");
    eecommand[9]='2';
    SendCommandToArduino(eecommand);
    res.succeeded = true;
  }
  else
  {
    ROS_DEBUG("[TNP_STATE L4] Error tnp_end_effector/DrawerCallback Error when turning on/off the drawer");
    ROS_ERROR("DrawerCallback: uknown value"); //this is just for good coding practice
  }

  ROS_DEBUG("DrawerCallback: done");
  ROS_DEBUG("[TNP_STATE L4] Done tnp_end_effector/DrawerCallback Function called successfully");

  return true;
}


bool EndEffectorNode::EEControlCallback(tnp_end_effector::EEControl::Request &req, tnp_end_effector::EEControl::Response &res)
{
  ROS_DEBUG("EEControlCallback was called");
  ROS_DEBUG("[TNP_STATE L4] Calling tnp_end_effector/EEcontrol");

  //gripper
  ROS_DEBUG_STREAM("Hand control method is " << req.GripperClose);
  ROS_DEBUG_STREAM("Hand control parameter is " << req.gripper_control_parameter);

  if(req.GripperClose == false)
  {
    ROS_ASSERT(req.gripper_control_parameter.data >= 0 && req.gripper_control_parameter.data <= 1500); //this should be milimeters
    ROS_INFO("Gripper open");
    //set parameter
    std::ostringstream sout;
    sout << std::setfill('0') << std::setw(4) << req.gripper_control_parameter.data;
    std::string c_para = sout.str();
    eecommand[12]=c_para[0];
    eecommand[13]=c_para[1];
    eecommand[14]=c_para[2];
    eecommand[15]=c_para[3];
    eecommand[5]='0';
    res.succeeded = true;
  }
  else if(req.GripperClose == true)
  {
    ROS_ASSERT(req.gripper_control_parameter.data >= 0 && req.gripper_control_parameter.data <= 1500); //this should be percentage
    ROS_INFO("Gripper close");
    //set parameter
    std::ostringstream sout;
    sout << std::setfill('0') << std::setw(4) << req.gripper_control_parameter.data;
    std::string c_para = sout.str();
    eecommand[12]=c_para[0];
    eecommand[13]=c_para[1];
    eecommand[14]=c_para[2];
    eecommand[15]=c_para[3];
    eecommand[5]='1';
    res.succeeded = true;
  }
  else
  {
    ROS_ERROR_STREAM("EndEffectorNode::EEControlCallback: Unknown control method: " << req.GripperClose);
  }

  //suction
  ROS_DEBUG("SuctionOn (bool) value is %s", req.SuctionOn ? "true" : "false");
  if (req.SuctionOn == true)
  {
    ROS_INFO("Turning ON suction");
    ROS_DEBUG("[TNP_STATE L4] Executing tnp_end_effector/EEControl Turning on the suction");
    //set parameter
    std::ostringstream sout;
    sout << std::setfill('0') << std::setw(2) << req.suction_force.data;
    std::string c_para = sout.str();
    eecommand[17]=c_para[0];
    eecommand[18]=c_para[1];
    eecommand[6]='1';
    res.succeeded = true;
  }
  else if (req.SuctionOn == false)
  {
    ROS_INFO("Turning OFF suction");
    ROS_DEBUG("[TNP_STATE L4] Executing tnp_end_effector/EEControl Turning off the suction");
    eecommand[6]='0';
    res.succeeded = true;
  }
  else
  {
    ROS_ERROR("EEControlCallback: uknown value"); //this is just for good coding practice
    ROS_DEBUG("[TNP_STATE L4] Error tnp_end_effector/EEControl Error when trying to turn on/off the suction");
  }


  //lin_act_gripper_extended
  ROS_DEBUG("lin_act_gripper_extended (bool) value is %s", req.lin_act_gripper_extended ? "true" : "false");
  if (req.lin_act_gripper_extended == true)
  {
    ROS_INFO("Advancing gripper linear actuator");
    ROS_DEBUG("[TNP_STATE L4] Executing tnp_end_effector/EEControl Advancing the linear actuator of gripper");
    eecommand[7]='1';
  }
  else if (req.lin_act_gripper_extended == false)
  {
    ROS_INFO("Retracting gripper linear actuator");
    ROS_DEBUG("[TNP_STATE L4] Executing tnp_end_effector/EEControl Retracting the linear actuator of gripper");
    eecommand[7]='0';
  }
  else
  {
    ROS_ERROR("EEControlCallback: uknown value"); //this is just for good coding practice
    ROS_DEBUG(
        "[TNP_STATE L4] Error tnp_end_effector/EEControl Error when trying to advance/retract linear actuator of gripper");
  }


  //lin_act_suction_extended
  ROS_DEBUG("lin_act_suction_extended (bool) value is %s", req.lin_act_suction_extended ? "true" : "false");
  if (req.lin_act_suction_extended == true)
  {
    ROS_INFO("Advancing suction linear actuator");
    ROS_DEBUG("[TNP_STATE L4] Executing tnp_end_effector/EEControl Advancing the linear actuator of suction");
    eecommand[8]='1';
  }
  else if (req.lin_act_suction_extended == false)
  {
    ROS_INFO("Retracting suction linear actuator");
    ROS_DEBUG("[TNP_STATE L4] Executing tnp_end_effector/EEControl Retracting the linear actuator of suction");
    eecommand[8]='0';
  }
  else
  {
    ROS_ERROR("EEControlCallback: uknown value"); //this is just for good coding practice
    ROS_DEBUG(
        "[TNP_STATE L4] Error tnp_end_effector/EEControl Error when trying to advance/retract linear actuator of suction");
  }
  SendCommandToArduino(eecommand);
  ROS_DEBUG("EEControlCallback: done");
  ROS_DEBUG("[TNP_STATE L4] Doone tnp_end_effector/EEControl Function called successfully");

  ros::Duration(.5).sleep();
  SendCommandToArduino(eecommand);

  return true;
}

bool EndEffectorNode::SuctionStatusCallback(tnp_end_effector::ItemSuctioned::Request &req, tnp_end_effector::ItemSuctioned::Response &res)
{
  res.suctioned = suction_status;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tnp_end_effector");

  EndEffectorNode SAPObject;

  ROS_INFO("End effector node started");

  ros::spin();

  return 0;
}
