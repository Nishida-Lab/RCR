void setup(){
  pinMode( 8, OUTPUT);
  pinMode( 9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  Serial.begin(9600) ;
}


void loop(){
  long x , y , z;
  int i = 0;
  x = y = z = 0 ;

  
  digitalWrite( 8, HIGH);
  digitalWrite( 9, HIGH);
  digitalWrite(10, HIGH);
  digitalWrite(11, LOW);  //Acc_x
  x = analogRead(0); // Ｘ軸
  
  digitalWrite( 8, LOW);
  digitalWrite( 9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, HIGH);//Acc_
  y = analogRead(0); // Ｙ軸

  digitalWrite( 8, HIGH);
  digitalWrite( 9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, HIGH); //Acc_z
  z = analogRead(0); // Ｚ軸
  
  Serial.print("X:");
  Serial.print(x);
  Serial.print(" Y:");
  Serial.print(y);
  Serial.print(" Z:");
  Serial.println(z);
  delay(100);
}