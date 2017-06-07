#define ACC_X  7 //set Pin number
#define ACC_Y  8
#define ACC_Z  9
#define OFFSET_X 1801.7 //sensor offset
#define OFFSET_Y 1856

int tim = 0;
double new_data[3] = {0,0,0};
double last_answer[3] = {0,0,0};
double answer[3] = {0,0,0};
double pos_x,pos_y,pos_z;

int readAnalog(int pin){ //read kxr94 by Multiplexer
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

struct DATA{
  unsigned long time;
  double data_x;
  double data_y;
  double data_z;
};

DATA acc_0,acc_1,acc_2,vel_0,vel_1,vel_2;

void getAcc(int num, double x, double y, double z){
  switch(num){
    case 0:
    acc_0.time = micros() * 1000000;
    //重力加速度[m/s^2]*(電圧[mV]-オフセット電圧[mV])/感度[mV/g] = 加速度[m/s^2]
    acc_0.data_x = 9.8*(x * 4.9 - OFFSET_X)/660; 
    acc_0.data_y = 9.8*(y * 4.9 - OFFSET_Y)/660; 
    acc_0.data_z = 9.8*(z * 4.9)/660; break;
 
  case 1:
    acc_1.time = micros() * 1000000;
    acc_1.data_x = 9.8*(x * 4.9 - OFFSET_X)/660; 
    acc_1.data_y = 9.8*(y * 4.9 - OFFSET_Y)/660; 
    acc_1.data_z = 9.8*(z * 4.9 - 1650)/660; break;

  case 2:
    acc_2.time = micros() * 1000000;
    acc_2.data_x = 9.8*(x * 4.9-OFFSET_X)/660;
    acc_2.data_y = 9.8*(y * 4.9-OFFSET_Y)/660; 
    acc_2.data_z = 9.8*(z * 4.9-1650)/660; break;
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

  return y_2;
}

void setup(){
  pinMode( 8, OUTPUT);
  pinMode( 9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  Serial.begin(115200) ;
}


void loop(){  
  int num[3] = {0,1,2};
  double rc_param = 0.85; //RC filter parametor


  //get Accel
  new_data[0] = readAnalog(7);
  answer[0] = rc_param * last_answer[0] + (1-rc_param) * new_data[0]; //RC filter
  last_answer[0] = answer[0];

  new_data[1] = readAnalog(8);
  answer[1] = rc_param * last_answer[1] + (1-rc_param) * new_data[1]; 
  last_answer[1] = answer[1];

  new_data[2] = readAnalog(9);
  answer[2] = rc_param * last_answer[2] + (1-rc_param) * new_data[2]; 
  last_answer[2] = answer[2];

  getAcc(num[tim],answer[0],answer[1],answer[2]);

  if(estimate_count%3 == 0) acc_2.data_x = estimate(acc_2.data_x, acc_1.data_x, acc_0.data_x);
  if(estimate_count%3 == 1) acc_0.data_x = estimate(acc_0.data_x, acc_2.data_x, acc_1.data_x);
  if(estimate_count%3 == 2) acc_1.data_x = estimate(acc_1.data_x, acc_0.data_x, acc_2.data_x);


  tim++;
  if(tim > 2) tim = 0;


  //get Velocity
  vel_0.time = acc_0.time;
  vel_0.data_x = getIntegral(acc_0.time, acc_0.data_x, acc_1.time, acc_1.data_x, acc_2.time, acc_2.data_x);
  vel_0.data_y = getIntegral(acc_0.time, acc_0.data_y, acc_1.time, acc_1.data_y, acc_2.time, acc_2.data_y);
  vel_0.data_z = getIntegral(acc_0.time, acc_0.data_z, acc_1.time, acc_1.data_z, acc_2.time, acc_2.data_z);

  vel_1.time = acc_1.time;
  vel_1.data_x = getIntegral(acc_0.time, acc_0.data_x, acc_1.time, acc_1.data_x, acc_2.time, acc_2.data_x);
  vel_1.data_y = getIntegral(acc_0.time, acc_0.data_y, acc_1.time, acc_1.data_y, acc_2.time, acc_2.data_y);
  vel_1.data_z = getIntegral(acc_0.time, acc_0.data_z, acc_1.time, acc_1.data_z, acc_2.time, acc_2.data_z);

  vel_2.time = acc_2.time;
  vel_2.data_x = getIntegral(acc_2.time, acc_0.data_x, acc_1.time, acc_1.data_x, acc_2.time, acc_2.data_x);
  vel_2.data_y = getIntegral(acc_2.time, acc_0.data_y, acc_1.time, acc_1.data_y, acc_2.time, acc_2.data_y);
  vel_2.data_z = getIntegral(acc_2.time, acc_0.data_z, acc_1.time, acc_1.data_z, acc_2.time, acc_2.data_z);
 

  //get Position
  pos_x = getIntegral(vel_0.time, vel_0.data_x, vel_1.time, vel_1.data_x, vel_2.time, vel_2.data_x);
  pos_y = getIntegral(vel_0.time, vel_0.data_y, vel_1.time, vel_1.data_y, vel_2.time, vel_2.data_y);
  pos_z = getIntegral(vel_0.time, vel_0.data_z, vel_1.time, vel_1.data_z, vel_2.time, vel_2.data_z);


  Serial.print(acc_0.data_x); Serial.print(" "); 
  //Serial.print(vel_1.data_x); Serial.print(" "); 
  //Serial.println(pos_x); //print Screen
}