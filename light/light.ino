const int ledPin =13;

void setup() {
  Serial.begin(9600); 
  pinMode(ledPin, OUTPUT);
}


void loop() {
  if (Serial.available() > 0) {          // Check if data is available to read
    int signal = Serial.read();          // Read the incoming byte
	if(signal=='1'){
digitalWrite(ledPin,HIGH);	
}	
else{
	digitalWrite(ledPin, LOW);
}
    }
}

