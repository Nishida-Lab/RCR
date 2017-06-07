#define ACC_X  7
#define ACC_Y  8
#define ACC_Z  9
#define OFFSET_X 1801.7
#define OFFSET_Y 1856

int readAnalog(int pin){
  switch(pin){
  case ACC_X:
    digitalWrite( 8, HIGH);
    digitalWrite( 9, HIGH);
    digitalWrite(10, HIGH);
    digitalWrite(11, LOW); break; //Acc_x
  case ACC_Y:
    digitalWrite( 8, LOW);
    digitalWrite( 9, LOW);
    digitalWrite(10, LOW);
    digitalWrite(11, HIGH); break; //Acc_y
  case ACC_Z: 
    digitalWrite( 8, HIGH);
    digitalWrite( 9, LOW);
    digitalWrite(10, LOW);
    digitalWrite(11, HIGH); break; //Acc_z
  }

  return analogRead(0);
}

struct ACCEL{
  unsigned long time;
  double data_x;
  double data_y;
  double data_z;
};

ACCEL acc_0,acc_1,acc_2;

void getAcc(int num){
  switch(num){
  case 0:
    acc_0.time = micros();
    //重力加速度[m/s^2]*(電圧[mV]-オフセット電圧[mV])/感度[mV/g] = 加速度[m/s^2]
    acc_0.data_x = 9.8*(readAnalog(7)*4.9-OFFSET_X)/660; 
    acc_0.data_y = 9.8*(readAnalog(8)*4.9-OFFSET_Y)/660; 
    acc_0.data_z = 9.8*(readAnalog(9)*4.9)/660; break;
 
  case 1:
    acc_1.time = micros();
    acc_1.data_x = 9.8*(readAnalog(7)*4.9-OFFSET_X)/660; 
    acc_1.data_y = 9.8*(readAnalog(8)*4.9-OFFSET_Y)/660; 
    acc_1.data_z = 9.8*(readAnalog(9)*4.9-1650)/660; break;

  case 2:
    acc_2.time = micros();
    acc_2.data_x = 9.8*(readAnalog(7)*4.9-OFFSET_X)/660;
    acc_2.data_y = 9.8*(readAnalog(8)*4.9-OFFSET_Y)/660; 
    acc_2.data_z = 9.8*(readAnalog(9)*4.9-1650)/660; break;
  }
}

int getPosition(unsigned long time_0, double acc_0, unsigned long time_1, double acc_1, unsigned long time_2, double acc_2){
  int position = 0;
  if(time_0 < time_1 && time_1 < time_2){
    position = (acc_0 + 4*acc_1 + acc_2)*(time_2 - time_0)/6;
  }
  else if(time_1 < time_2 && time_2 < time_0){
    position = (acc_1 + 4*acc_2 + acc_0)*(time_0 - time_1)/6;
  }
  else if(time_2 < time_0 && time_0 < time_1){
    position = (acc_2 + 4*acc_0 + acc_1)*(time_1 - time_2)/6;
  }

  return position;
}


double new_data = 0,last_answer = 0,answer = 0;
int tim = 0;
unsigned long new_time = 0, last_time = 0, dt = 0;
double vel,pos;

void setup(){
  pinMode( 8, OUTPUT);
  pinMode( 9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  Serial.begin(115200) ;
}


void loop(){  
  int num[3] = {0,1,2};
  double a = 0.85;

//  getAcc(num[tim]);
//  tim++; 
//  if(tim > 2) tim = 0;

  getAcc(0);
  new_data = acc_0.data_y;
  answer = a*last_answer + (1-a)*new_data;

// new_time = micros();
// dt = new_time - last_time;
// 
// vel = answer * dt;
  
//  Serial.print(acc_0.time); Serial.print(" "); Serial.print(acc_0.data_z); Serial.print(" ");
//  Serial.print(acc_1.time); Serial.print(" "); Serial.print(acc_1.data_z); Serial.print(" ");
//  Serial.print(acc_2.time); Serial.print(" "); Serial.print(acc_2.data_z);
  Serial.println(answer);// Serial.print(" "); Serial.println(new_data); 
  last_answer = answer; 

//  last_time = new_time;
}