/*
Laser Diode Lock Servo Arduino microcontroller Code
Author: Brendan Saxberg
Date: March 28, 2016
Outline: (For detailed description see http://arxiv.org/pdf/1602.03504.pdf) 
Arduino scans fabry-perot waveform, stores peak values until switched on.  When on, checks if peak higher than threshold (if injection lock is present).  If yes, active follower runs.  If no, recovery program runs.
User should have arduino switched off (pin 30 low) and find the lock manually, then switch arduino on to track the lock.
 */
//initialize pin resolution and status.  Pin 30 used to switch between "don't track lock" and "track lock" states of Arduino
void setup() {
  Serial.begin(9600);
  analogReadResolution(12);
  analogWriteResolution(12);
  pinMode(A0, INPUT);
  pinMode(30, INPUT);
  pinMode(32, OUTPUT);
  digitalWrite(32, HIGH);
  pinMode(28, OUTPUT);
  digitalWrite(28, LOW);
  pinMode(DAC1, OUTPUT);
}

//Read the maximum fabry-perot signal over past (scantime) milliseconds
int maxreader(int scantime) {
  int val = 0;
  int temp = 0;
  unsigned long tstart = 0;
  int maxval = 0;
  unsigned long dt = 0;
  tstart = millis();
  dt = 0;
  maxval = analogRead(A0);
  while (dt < scantime) {
    temp = analogRead(A0);
    maxval = max(temp, maxval);
    dt = millis() - tstart;
  }
  return maxval;
}

//Read the size (max-min) of the fabry-perot signal over the past (scantime) milliseconds
int maxminreader(int scantime) {
  int val = 0;
  int temp = 0;
  unsigned long tstart = 0;
  int minval = 0;
  int maxval = 0;
  unsigned long dt = 0;
  tstart = millis();
  dt = 0;
  minval = analogRead(A0);
  maxval = minval;
  while (dt < scantime) {
    temp = analogRead(A0);
    minval = min(temp, minval);
    maxval = max(temp, maxval);
    dt = millis() - tstart;
  }
  val = maxval - minval;
  return val;
}

//Initialize useful variables for main loop
float maximum = 0;
float newmaximum = 0;
float noisesignal = 0;
float locksignal = 0;
int signalout = 1000;
int lock = 1000;
unsigned long timeoflock = 0;
int diff = 0;
int peak = 0;

//Check if injection lock present.  If not, recovery block runs.  If yes, active follower block runs
void loop() {
  maximum = (float)maxreader(500);

  //Passive mode of Arduino - determines what an injection lock should look like in peak voltage levels.
  if (digitalRead(30) == LOW) {
    signalout = 1000;
    lock = 1000;
    analogWrite(DAC1, signalout);
    diff = maxminreader(500);
    peak = maxreader(500);
    noisesignal = (float)peak - ( (float)diff / 5);
    locksignal = (float)peak - ( (float)diff / 50);
  }

  //Lock Recovery block:
  if ((maximum < noisesignal) && (digitalRead(30) == HIGH) ) {
    //move up
    if (lock + 1500 > 4095) {
      signalout = 4095;
    }
    else {
      signalout = lock + 700;
    }
    analogWrite(DAC1, signalout);
    while ((maximum < locksignal) && (digitalRead(30) == HIGH)) {
      if (maximum < noisesignal) {
        signalout = signalout - 50;
      }
      else {
        signalout = signalout - 5;
      }
      analogWrite(DAC1, signalout);
      maximum = (float)maxreader(200);
    }
    if ( maximum > locksignal ) {
      lock = signalout;
      timeoflock = millis();
    }
  }

  //Active follower block
  if ((maximum > locksignal) && (digitalRead(30) == HIGH) && (millis() - timeoflock > 5000)) {
    float lowmax = 0;
    float midmax = 0;
    float highmax = 0;
    int jump = 10;
    analogWrite(DAC1, signalout - jump);
    lowmax = (float)maxreader(500);
    analogWrite(DAC1, signalout);
    delay(300);
    midmax = (float)maxreader(500);
    delay(300);
    analogWrite(DAC1, signalout + jump);
    highmax = (float)maxreader(500);

    //If lower current give us a .5% boost, go up
    if (((lowmax - midmax) / lowmax) > .005) {
      signalout = signalout - jump;
    }
    //if higher current doesn't decrease strength by more than .25% & w/in locksig, go up
    else if ( (((highmax - midmax) / highmax) > -.0025) && (highmax > locksignal) ) {
      signalout = signalout + jump;
    }
    //if nothing else, don't move.
    else {
      signalout = signalout;
    }
    lock = signalout;
    analogWrite(DAC1, signalout);
    timeoflock = millis();
  }
}
