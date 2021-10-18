/*
  WTA assignment

  Turns an LED on based on message via subscription to /${namespace}/assignment.

  created 20 Sep 2021
  modified 22 Sep 2021
  by Anubhav Gupta

*/

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Time.h>

ros::NodeHandle nh;

std_msgs::Int64 assignment_msg;
ros::Time last_time;
ros::Time current_time;

void messageCb( const std_msgs::Int64 &msg) {
  assignment_msg = msg;
  last_time = nh.now();
}

ros::Subscriber<std_msgs::Int64> sub("assignment", &messageCb, 1 );

int weapon_id;
int msg_time_diff;
bool flag_agent;
bool flag_weapon_1;
bool flag_weapon_2;
bool flag_weapon_3;
bool flag_weapon_4;
bool flag_weapon_none;

// the setup function runs once when you press reset or power the board
void setup() {

  // initialize digital pins - LED_BUILTIN, 33, 36, 39 as outputs.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(36, OUTPUT);
  pinMode(39, OUTPUT);

  // initialize node and create subscriber
  nh.initNode();
  nh.subscribe(sub);

  // set flags and epoch time
  flag_agent = 0;
  flag_weapon_1 = 0;
  flag_weapon_2 = 0;
  flag_weapon_3 = 0;
  flag_weapon_4 = 0;
  flag_weapon_none = 0;
  last_time = nh.now();
  
  //  nh.loginfo("Initialized LEDs | Weapon Assignment");/
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(39, HIGH);
  digitalWrite(36, HIGH);
  digitalWrite(33, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(39, LOW);
  digitalWrite(36, LOW);
  digitalWrite(33, LOW);
}

// the loop function runs over and over again forever
void loop() {
  weapon_id = assignment_msg.data;

  // reset assignment if duration of message is more than 3 seconds
  current_time = nh.now();
  msg_time_diff = current_time.sec - last_time.sec;
  if (abs(msg_time_diff) > 3 && flag_agent == 0) {
    weapon_id = 5;
    nh.loginfo("No assignment communicated in last 3 seconds");
    //nh.logwarn((String(current_time.sec)).c_str());
    //nh.logwarn((String(last_time.sec)).c_str());
    //nh.logwarn((String(assignment_msg.data)).c_str());
  }


  if (weapon_id == 0) {
    // turn off all LEDs
    if (flag_agent == 0) {
      String a = "Agent ";
      //ros::param();
      //String d = ros::param::get("/assignment", weapon_id);
      //nh.getParam("/agent_0/serial_node/baud");
      String b = String(weapon_id);
      String c = " Attrited";
      nh.logwarn((a+c).c_str());
      flag_agent = 1;
      flag_weapon_1 = 0;
      flag_weapon_2 = 0;
      flag_weapon_3 = 0;
      flag_weapon_4 = 0;
      flag_weapon_none = 0;
    }
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(39, LOW);
    digitalWrite(36, LOW);
    digitalWrite(33, LOW);
  }

  else if (weapon_id == 1) {
    // turn on one LED and turn off the rest
    if (flag_weapon_1 == 0) {
      nh.loginfo("White LED | Weapon 1");
      flag_agent = 0;
      flag_weapon_1 = 1;
      flag_weapon_2 = 0;
      flag_weapon_3 = 0;
      flag_weapon_4 = 0;
      flag_weapon_none = 0;
    }
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(39, LOW);
    digitalWrite(36, LOW);
    digitalWrite(33, LOW);
  }

  else if (weapon_id == 2) {
    // turn on one LED and turn off the rest
    if (flag_weapon_2 == 0) {
      nh.loginfo("Red LED | Weapon 2");
      flag_agent = 0;
      flag_weapon_1 = 0;
      flag_weapon_2 = 1;
      flag_weapon_3 = 0;
      flag_weapon_4 = 0;
      flag_weapon_none = 0;
    }
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(39, HIGH);
    digitalWrite(36, LOW);
    digitalWrite(33, LOW);
  }

  else if (weapon_id == 3) {
    // turn on one LED and turn off the rest
    if (flag_weapon_3 == 0) {
      nh.loginfo("Blue LED | Weapon 3");
      flag_agent = 0;
      flag_weapon_1 = 0;
      flag_weapon_2 = 0;
      flag_weapon_3 = 1;
      flag_weapon_4 = 0;
      flag_weapon_none = 0;
    }
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(39, LOW);
    digitalWrite(36, HIGH);
    digitalWrite(33, LOW);
  }

  else if (weapon_id == 4) {
    // turn on one LED and turn off the rest
    if (flag_weapon_4 == 0) {
      nh.loginfo("Yellow LED | Weapon 4");
      flag_agent = 0;
      flag_weapon_1 = 0;
      flag_weapon_2 = 0;
      flag_weapon_3 = 0;
      flag_weapon_4 = 1;
      flag_weapon_none = 0;
    }
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(39, LOW);
    digitalWrite(36, LOW);
    digitalWrite(33, HIGH);
  }

  else {
    // turn on All LEDs
    if (flag_weapon_none == 0) {
      nh.loginfo("Blinking LEDs | Weapon Assignment");
      flag_agent = 0;
      flag_weapon_1 = 0;
      flag_weapon_2 = 0;
      flag_weapon_3 = 0;
      flag_weapon_4 = 0;
      flag_weapon_none = 1;
    }
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(39, HIGH);
    digitalWrite(36, HIGH);
    digitalWrite(33, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(39, LOW);
    digitalWrite(36, LOW);
    digitalWrite(33, LOW);
    delay(1000);
  }

  nh.spinOnce();
  //  delay(3);   // to compensate for wrong chksum
}
