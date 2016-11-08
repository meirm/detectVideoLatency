/*
  This example code is in the public domain.

*/

#define BUTTON 9
#define fps 30.0
#define frames 300
#define LED2 12
// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(BUTTON, INPUT);
  digitalWrite(BUTTON,HIGH);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  pinMode(LED2, OUTPUT);
  digitalWrite(LED2, LOW);    // turn the LED off by making the voltage LOW
 
  Serial.begin(115200);
  
}

// the loop function runs over and over again forever
void loop() {
  if(digitalRead(BUTTON)==LOW){
    
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.write('0');
    Serial.flush();
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(2000);
    Serial.write('1');
    Serial.flush();
    digitalWrite(LED_BUILTIN, HIGH); // B Y   // turn the LED on (HIGH is the voltage level)
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off
    
  }
}
