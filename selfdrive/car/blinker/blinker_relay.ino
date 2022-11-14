int relay_left = 6;
int relay_right =7;
int input_left = 8;
int input_right = 9;

void setup()
{
    pinMode(relay_left, OUTPUT);
    pinMode(relay_right, OUTPUT);
    pinMode(input_left, INPUT_PULLUP;
    pinMode(input_right, INPUT_PULLUP);
}

void loop()
{
  if(digitalRead(input_left) == LOW && digitalRead(input_right) == LOW)
  {
    digitalWrite(relay_left, LOW)
    digitalWrite(relay_right, LOW)
  }
  else if(digitalRead(input_left) == LOW && digitalRead(input_right) == HIGH)
  {
    digitalWrite(relay_left, HIGH)
    digitalWrite(relay_right, LOW)
  }
  else if(digitalRead(input_left) == HIGH && digitalRead(input_right) == LOW)
  {
    digitalWrite(relay_left, LOW)
    digitalWrite(relay_right, HIGH)
  }
}
8:50-> 10:00