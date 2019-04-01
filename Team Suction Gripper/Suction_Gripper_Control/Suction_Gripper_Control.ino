bool state = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(4, OUTPUT);

  
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    String string = Serial.readStringUntil("\n");
  
    if(string == "open"){
      digitalWrite(4, HIGH);
    }
    else if(string == "close"){
      digitalWrite(4, LOW);
    }
  }
}
