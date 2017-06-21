#define ACC_X  7 //set Pin number
#define ACC_Y  8
#define ACC_Z  9
#define ACC_G 9.80665
#define OFFSET_X 1642 //sensor offset
#define OFFSET_Y 1642
#define OFFSET_Z 1230


int tim = 0;
double new_acc[3] = {0,0,0};
double acc[3] = {0,0,0};
double pos_x,pos_y,pos_z;
double new_vel[3] = {0,0,0};
double last_vel[3] = {0,0,0};
double vel[3] = {0,0,0};
 



int readAnalog(int pin){ //read kxr94 by Multiplexer
  switch(pin){
  case ACC_X:
    digitalWrite( 8, LOW);
    digitalWrite( 9, HIGH);
    digitalWrite(10, HIGH);
    digitalWrite(11, HIGH); break; //Acc_x
  case ACC_Y:
    digitalWrite( 8, HIGH);
    digitalWrite( 9, LOW);
    digitalWrite(10, LOW);
    digitalWrite(11, LOW); break; //Acc_y
  case ACC_Z: 
    digitalWrite( 8, HIGH);
    digitalWrite( 9, LOW);
    digitalWrite(10, LOW);
    digitalWrite(11, HIGH); break; //Acc_z
  }

  return analogRead(0);
}

struct DATA{
  unsigned long time;
  double data_x;
  double data_y;
  double data_z;
  double sum;
};

DATA acc_0,acc_1,acc_2,vel_0,vel_1,vel_2;

void getAcc(int num, double x, double y, double z){
  double sum, answer;
  switch(num){
  case 0:
    acc_0.time = micros();
    //重力加速度[m/s^2]*(電圧[mV]-オフセット電圧[mV])/感度[mV/g] = 加速度[m/s^2]
    acc_0.data_x = 9.8*(x * 4.9 - OFFSET_X)/1000; 
    acc_0.data_y = 9.8*(y * 4.9 - OFFSET_Y)/1000; 
    acc_0.data_z = 9.8*(z * 4.9 - OFFSET_Z)/1000;
    if(acc_0.data_x < 0 && acc_0.data_y < 0){
      sum = -acc_0.data_x*acc_0.data_x - acc_0.data_y*acc_0.data_y + acc_0.data_z*acc_0.data_z - ACC_G*ACC_G;
    }else if(acc_0.data_x >= 0 && acc_0.data_y < 0){
      sum =  acc_0.data_x*acc_0.data_x - acc_0.data_y*acc_0.data_y + acc_0.data_z*acc_0.data_z - ACC_G*ACC_G;
    }else if(acc_0.data_x < 0 && acc_0.data_y >= 0){
      sum = -acc_0.data_x*acc_0.data_x + acc_0.data_y*acc_0.data_y + acc_0.data_z*acc_0.data_z - ACC_G*ACC_G;
    }else{
      sum =  acc_0.data_x*acc_0.data_x + acc_0.data_y*acc_0.data_y + acc_0.data_z*acc_0.data_z - ACC_G*ACC_G;
    }
    
    if(sum < 0) acc_0.sum = -sqrt(-sum);
    else if(sum >= 0) acc_0.sum = sqrt(sum);
    break;
 
  case 1:
    acc_1.time = micros();
    acc_1.data_x = 9.8*(x * 4.9 - OFFSET_X)/1000; 
    acc_1.data_y = 9.8*(y * 4.9 - OFFSET_Y)/1000; 
    acc_1.data_z = 9.8*(z * 4.9 - OFFSET_Z)/1000; 
    sum = acc_1.data_x*acc_1.data_x + acc_1.data_y*acc_1.data_y;// + acc_1.data_z*acc_1.data_z - ACC_G*ACC_G;
    if(sum < 0) acc_1.sum = -sqrt(-sum);
    else if(sum >= 0) acc_1.sum = sqrt(sum);
    break;

  case 2:
    acc_2.time = micros();
    acc_2.data_x = 9.8*(x * 4.9 - OFFSET_X)/1000;
    acc_2.data_y = 9.8*(y * 4.9 - OFFSET_Y)/1000; 
    acc_2.data_z = 9.8*(z * 4.9 - OFFSET_Z)/1000; 
    sum = acc_2.data_x*acc_2.data_x + acc_2.data_y*acc_2.data_y;// + acc_2.data_z*acc_2.data_z - ACC_G*ACC_G;
    if(sum < 0) acc_2.sum = -sqrt(-sum);
    else if(sum >= 0) acc_2.sum = sqrt(sum);
    break;
  }
}

  

double getIntegral(unsigned long time_0, double data_0, unsigned long time_1, double data_1, unsigned long time_2, double data_2){
  double answer = 0;
  if(time_0 < time_1 && time_1 < time_2){
    answer = (data_0 + 4*data_1 + data_2)*(time_2 - time_0)/6;
  }
  else if(time_1 < time_2 && time_2 < time_0){
    answer = (data_1 + 4*data_2 + data_0)*(time_0 - time_1)/6;
  }
  else if(time_2 < time_0 && time_0 < time_1){
    answer = (data_2 + 4*data_0 + data_1)*(time_1 - time_2)/6;
  }

  return answer;
}


void getVelocity(int t){
  switch(t){
  case 0:
    vel_0.time = acc_1.time;    
    vel_0.sum = getIntegral(acc_0.time, acc_0.sum, acc_1.time, acc_1.sum, acc_2.time, acc_2.sum)/(1000*1000);
    break;
  case 1:
    vel_1.time = acc_2.time;
    vel_1.sum = getIntegral(acc_0.time, acc_0.sum, acc_1.time, acc_1.sum, acc_2.time, acc_2.sum)/(1000*1000);
    break;
  case 2:
    vel_2.time = acc_0.time;
    vel_2.sum = getIntegral(acc_0.time, acc_0.sum, acc_1.time, acc_1.sum, acc_2.time, acc_2.sum)/(1000*1000);
    break;
  }
}


/*
  double Gn  = 0;
  double hn[2] = {0,0};
  double theta_hat[2] = {0,0};
  double psi[2] = {0.0};
  int estimate_count = 0;

  //use ARMA model
  double estimate(double y_2, double y_1, double y_0){
  psi[0] = -y_1;
  psi[1] = -y_0;

  Gn += psi[0]*psi[0] + psi[1]*psi[1];
  hn[0] += y_2*psi[0];
  hn[1] += y_2*psi[1];

  Gn /= estimate_count+1;
  hn[0] /= estimate_count+1;
  hn[1] /= estimate_count+1;

  theta_hat[0] = hn[0] / Gn;
  theta_hat[1] = hn[1] / Gn;

  y_2 = theta_hat[0]*psi[0] + theta_hat[1]*psi[1];
  estimate_count++;

  Gn *= estimate_count+1;
  hn[0] *= estimate_count+1;  
  hn[1] *= estimate_count+1;
  return y_2;
  }
*/

void setup(){
  pinMode( 8, OUTPUT);
  pinMode( 9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  Serial.begin(115200) ;
}

int count = 0;
double last_poss = 0;
double value = 0;

void loop(){  
  int num[3] = {0,1,2};
  double param = 0.95; //RC filter parametor

  //get Accel(RC filter)
  new_acc[0] = readAnalog(ACC_X);
  acc[0] = (1-param) * acc[0] + param * new_acc[0];
    
  new_acc[1] = readAnalog(ACC_Y);
  acc[1] = (1-param) * acc[1] + param * new_acc[1];
  
  new_acc[2] = readAnalog(ACC_Z);
  acc[2] = (1-param) * acc[2] + param * new_acc[2];
 
  getAcc(num[tim],acc[0],acc[1],acc[2]); //get Accel[m/s^2]
  getVelocity(num[tim]); //get Velocity[m/s]
  
  //get Position
  pos_x = getIntegral(vel_0.time, vel_0.sum, vel_1.time, vel_1.sum, vel_2.time, vel_2.sum)/(1000*1000);  //get Position[m]
  pos_x = param * last_poss + (1-param) * pos_x;
  last_poss = pos_x; 

  value += pos_x;

//  Serial.print(acc[0]); Serial.print(" ");
//  Serial.print(acc[1]); Serial.print(" ");
//  Serial.println(acc[2]);

//  Serial.print(ACC_G); Serial.print(" ");
//  Serial.print(acc_0.data_x); Serial.print(" ");
//  Serial.print(acc_0.data_y); Serial.print(" ");
//  Serial.print(acc_0.data_z); Serial.print(" ");
//  Serial.println(acc_0.sum);

  Serial.print(acc_0.sum,10); Serial.print(" ");
  Serial.print(vel_0.sum,10); Serial.print(" "); 
  //  Serial.println(pos_x,10); //print Screen
  Serial.println(value,10);
  
  tim++;
  if(tim > 2) tim = 0;
}