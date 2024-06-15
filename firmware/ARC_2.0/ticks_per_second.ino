#define encoder0PinA 18      // encoder 1
#define encoder0PinB 19

#define encoder1PinA 20      // encoder 2
#define encoder1PinB 21

unsigned long currentMillis;
unsigned long previousMillis;

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2

volatile long startPos0 = 0;      // starting position for encoder 1
volatile long startPos1 = 0;      // starting position for encoder 2

boolean counting = false;         // flag to indicate counting status

void setup() {
  Serial.begin(9600);
  pinMode(encoder0PinA, INPUT_PULLUP);    // encoder pins
  pinMode(encoder0PinB, INPUT_PULLUP);

  pinMode(encoder1PinA, INPUT_PULLUP); 
  pinMode(encoder1PinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoderB, CHANGE); 

  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoderC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoderD, CHANGE); 

  Serial.println("Type 'start' to begin counting ticks and 'stop' to end counting.");
}

void loop() {
  // Check for serial input
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove any trailing whitespace or newline characters

    if (input.equalsIgnoreCase("start")) {
      if (!counting) {
        // Start counting
        counting = true;
        startPos0 = encoder0Pos;
        startPos1 = encoder1Pos;
        Serial.println("Counting ticks... Rotate the wheel one full revolution.");
      }
    } else if (input.equalsIgnoreCase("stop")) {
      if (counting) {
        // Stop counting and print results
        counting = false;
        long ticksPerRevolution0 = encoder0Pos - startPos0;
        long ticksPerRevolution1 = encoder1Pos - startPos1;
        Serial.print("Encoder 1 ticks per revolution: ");
        Serial.println(ticksPerRevolution0);
        Serial.print("Encoder 2 ticks per revolution: ");
        Serial.println(ticksPerRevolution1);
      }
    }
  }

  // Optional: Print the current position for debugging
  currentMillis = millis();
  if (currentMillis - previousMillis >= 1000) {  // Print every second
    previousMillis = currentMillis;
    Serial.print("Encoder 1 Position: ");
    Serial.print(encoder0Pos);
    Serial.print(", Encoder 2 Position: ");
    Serial.println(encoder1Pos);
  }
}

// ************** encoders interrupts **************

// ************** encoder 1 *********************

void doEncoderA() {  
  if (digitalRead(encoder0PinA) == HIGH) { 
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos++;         // CW
    } else {
      encoder0Pos--;         // CCW
    }
  } else { 
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos++;          // CW
    } else {
      encoder0Pos--;          // CCW
    }
  }
}

void doEncoderB() {  
  if (digitalRead(encoder0PinB) == HIGH) {   
    if (digitalRead(encoder0PinA) == HIGH) {  
      encoder0Pos++;         // CW
    } else {
      encoder0Pos--;         // CCW
    }
  } else { 
    if (digitalRead(encoder0PinA) == LOW) {   
      encoder0Pos++;          // CW
    } else {
      encoder0Pos--;          // CCW
    }
  }
}

// ************** encoder 2 *********************

void doEncoderC() {  
  if (digitalRead(encoder1PinA) == HIGH) { 
    if (digitalRead(encoder1PinB) == LOW) {  
      encoder1Pos--;         // CW
    } else {
      encoder1Pos++;         // CCW
    }
  } else { 
    if (digitalRead(encoder1PinB) == HIGH) {   
      encoder1Pos--;          // CW
    } else {
      encoder1Pos++;          // CCW
    }
  }
}

void doEncoderD() {  
  if (digitalRead(encoder1PinB) == HIGH) {   
    if (digitalRead(encoder1PinA) == HIGH) {  
      encoder1Pos--;         // CW
    } else {
      encoder1Pos++;         // CCW
    }
  } else { 
    if (digitalRead(encoder1PinA) == LOW) {   
      encoder1Pos--;          // CW
    } else {
      encoder1Pos++;          // CCW
    }
  }
}
