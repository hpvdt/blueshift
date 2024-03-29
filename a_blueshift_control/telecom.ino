void setupRadio() {
    // Set up the radio
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);          // Low power, raise if a decoupling capacitor is added
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(1);                    // Ensure autoACK is disabled
  radio.enableAckPayload();               // Allow optional ack payloads
  radio.setRetries(5, 15);                // Smallest time between retries, max no. of retries
  radio.enableDynamicPayloads();
  radio.setChannel(100);

  if (recieverRole == true) {
    // Reciever
    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1, pipes[0]);
    radio.startListening();
  }
  else {
    // Sender
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1, pipes[1]);
    radio.stopListening();
  }
  radio.maskIRQ(1, 1, 0);                 // Only toggles interrupt (falling edge/LOW) when recieving data

  // Interrupts on recieving a message
  pinMode(radioInteruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(radioInteruptPin), checkRadio, FALLING); // Set flagging interrupt
}

void checkRadio () {
  recievedRadioData = true; // Set flag that something was recieved
}

void radioRecieved() {
  for (byte i = 0; i <32; i++) {
    if (radioMessage[i] == '\0') break; // Most messages are less than 10 charact5ers so this should reduce unnecessary steps
    radioMessage[i] = '\0';
  }
  // Start by getting where the message is from
  radio.available(&pipeNumber); // Get pipe the message we recieved on in the event a reply is needed

  // Read the message. Reads both messages and acknoledgements. 
  radio.read(&radioMessage, radio.getDynamicPayloadSize());

  if (debugMode) {
    Serial.print("Recieved on nRF24: ");
    Serial.println(radioMessage);
  }
}

void radioSend(String outputMessage) {
  int messLength = outputMessage.length();
  if (recieverRole == true) {
    // Reciever
    // Uses the pointer to the text as a character array
    radio.writeAckPayload(pipeNumber, outputMessage.c_str(), messLength);
  }
  else {
    // Sender
    radio.write((outputMessage.c_str()), messLength);
  }
  if (debugMode) {
    Serial.print("Sending to nRF24: ");
    Serial.println(outputMessage);
  }
}
