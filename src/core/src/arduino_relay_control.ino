


#include <ros.h>
#include <std_msgs/String.h>

// Recorded Dec 27, 2017 by Robojax
#define relay1 7
#define relay2 6
#define relay3 5
#define relay4 4

ros::NodeHandle nh;
void messageCb(const std_msgs::String& msg){
  String data = msg.data;
  if(data.equals("ON")){
    digitalWrite(relay3,HIGH);// turn relay 3 ON
    Serial.println(msg.data);
  }else{
    digitalWrite(relay3,LOW); // turn relay 3 OFF
    Serial.println(msg.data);
  }
}

ros::Subscriber<std_msgs::String> sub("/vacuum",&messageCb);

void setup() {
  nh.initNode();
  nh.subscribe(sub);

  
  Serial.begin(9600);// setup Serial Monitor to display information
  pinMode(relay1, OUTPUT);// connected to Relay 1
  pinMode(relay2, OUTPUT);// connected to Relay 2
  pinMode(relay3, OUTPUT);// connected to Relay 3
  pinMode(relay4, OUTPUT);// connected to Relay 4  


}
void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(1);

}
