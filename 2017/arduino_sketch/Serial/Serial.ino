void setup(){
  Serial.begin(9600);
}

void loop(){
  char serial;
    serial = Serial.read();
  
    switch(serial){
    case 'a': Serial.write("A"); break;
    case 'b': Serial.write("B"); break;
    case 'c': Serial.write("C"); break;
    case 'd': Serial.write("D"); break;
    case 'e': Serial.write("E"); break;
    case 'f': Serial.write("F"); break;
    case 'g': Serial.write("G"); break;
    case 'h': Serial.write("H"); break;
    case 'i': Serial.write("I"); break;
    case 'j': Serial.write("J"); break;
    case 'k': Serial.write("K"); break;
    case 'l': Serial.write("L"); break;
    case 'm': Serial.write("M"); break;
    case 'n': Serial.write("N"); break;
    case 'o': Serial.write("O"); break;
    case 'p': Serial.write("P"); break;
    case 'q': Serial.write("Q"); break;
    case 'r': Serial.write("R"); break;
    case 's': Serial.write("S"); break;
    case 't': Serial.write("T"); break;
    case 'u': Serial.write("U"); break;
    case 'w': Serial.write("W"); break;
    case 'x': Serial.write("X"); break;
    case 'y': Serial.write("Y"); break;
    case 'z': Serial.write("Z"); break;
    default: Serial.write(false);
    }
}
