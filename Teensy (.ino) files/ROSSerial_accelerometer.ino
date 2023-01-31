/*
 * A2 - RED/ORANGE
 * A3 - YELLOW/GRAY
 * A4 - GREEN
 * 10 - INPUT
 */
/*#define USE_TEENSY_HW_SERIAL*/
#define window 100
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

int index1 = 1;

int buff[window];
float sum_window = 0.0;
float val = 0.0;
float final_val = 0.0;

float phase = 0.0;
float twopi = 3.14159 * 2;
elapsedMicros usec = 0;

ros::NodeHandle  nh;

std_msgs::Int16MultiArray sense_arr;
ros::Publisher chatter("chatter", &sense_arr);

void setup()
{
  Serial.begin(115200);
  // Initializing ROS node
  nh.initNode();
  nh.advertise(chatter);
  sense_arr.data = (int16_t*)malloc(sizeof(int16_t)*5);
  sense_arr.data_length = 5;
  analogWriteResolution(12);
  pinMode(A14,OUTPUT); // motor control
  pinMode(A2,INPUT); // input for motor control
  pinMode(A4,INPUT); //Z-axis
}

void loop()
{
  // Motor control
  sum_window = 0.0;
  val = analogRead(A2);
  for (int i = 0; i < window-1; i++){
      buff[i] = buff[i+1];
      sum_window = sum_window + buff[i];
    }
  buff[window-1] = val;
  sum_window = sum_window + val;
  final_val = sum_window/(float)window;

  float val_out = sin(phase) * 2000.0 + 2050.0;
  analogWrite(A14, (int)val_out);
  phase = phase + 0.001*final_val;
  if (phase >= twopi) phase = 0;
  while (usec < 500) ; // wait
  usec = usec - 500;

  // Accelerometer data acquisition
  if(index1 == 1){
    sense_arr.data[0] = analogRead(A4);
    index1 = 2;
  }else if(index1 == 2){
    sense_arr.data[1] = analogRead(A4);
    index1 = 3;
  }else if(index1 == 3){
    sense_arr.data[2] = analogRead(A4);
    index1 = 4;
  }else if(index1 == 4){
    sense_arr.data[3] = analogRead(A4);
    index1 = 5;
  }else if(index1 == 5){
    sense_arr.data[4] = analogRead(A4);
    chatter.publish( &sense_arr );
    index1 = 1;
  }
  nh.spinOnce();
}
