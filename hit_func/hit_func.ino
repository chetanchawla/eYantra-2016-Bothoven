#include<Servo.h>
Servo myservo1;
Servo myservo2;
int servoLeft = 0, servoRight = 180, pinNo = A11, modelNo, DistanceDash; //rotServ=11, strikeServ=12;
float Distance;
int Curr_Pos, Prev_Pos, clockwise = 1;

/*
   Function Name      : hit
   Input              : The mnp number which we have to hit
   Output             : None
   Logic              : Sets the firebird in desired direction
   Example Call       : hit(5)
*/

float sharp_distance()
{
  float val = analogRead(pinNo);
  float dist_cm = 10.00 * ((1.00 / ((0.001240875 * (float)val) + 0.005)) - 0.42) / 100.00;
  dist_cm = (dist_cm * 100);
  return dist_cm;
}

void striking_left()
{
  //myservo1.attach(11);
  //myservo2.attach(12);
  myservo1.write(servoLeft);
  delay(1000);
  for (int i = servoLeft; i < 80; i++)
  {
    myservo1.write(i);
    Distance = sharp_distance();
    Serial2.println(analogRead(pinNo));
    Serial2.println(Distance);
    if (Distance < 25)
    {
      myservo1.write(i + 5);
      delay(500);
      myservo2.write(40);
      delay(500);
      myservo2.write(180);
      delay(500);

      myservo1.write(90);
      delay(500);
      //      myservo1.detach();
      //      myservo2.detach();
      break;
    }
    delay(100);

  }
}
void striking_right()
{
  //myservo1.attach(11);
  //myservo2.attach(12);
  myservo1.write(servoRight);
  delay(1000);
  for (int i = servoRight; i > 100; i--)
  {
    myservo1.write(i);
    Distance = sharp_distance();
    if (Distance < 25)
    {
      myservo1.write(i - 5);
      delay(500);
      myservo2.write(40);
      delay(500);
      myservo2.write(180);
      delay(500);
      myservo1.write(90);
      delay(500);
      //  myservo1.detach();
      //  myservo2.detach();
      break;
    }
    delay(50);
  }
}
void striking_middle()
{
  //myservo1.attach(11);
  //myservo2.attach(12);
  for (int i = 70; i < 110; i++)
  {
    myservo1.write(i);
    Distance = sharp_distance();
    if (Distance < 25)
    {
      myservo1.write(i + 5);
      delay(500);
      myservo2.write(40);
      delay(500);
      myservo2.write(180);
      delay(500);
      myservo1.write(90);
      delay(500);
      //myservo1.detach();
      //myservo2.detach();
      break;
    }
    delay(50);
  }
}

void hit(int number)
{
  if (Curr_Pos >= 97 && Curr_Pos <= 121 && number <= 24) //Outer Circle
  {
    if (clockwise == 1) //Clockwise Direction
    {
      striking_left();
    }
    else if (clockwise == 0) //Anti-Clockwise Direction
    {
      striking_right();
    }
  }
  else if ((Curr_Pos == 97 || Curr_Pos == 101 || Curr_Pos == 105 || Curr_Pos == 109 || Curr_Pos == 113 || Curr_Pos == 117) && number >= 24)
  {
    if (clockwise == 1) //Clockwise Direction
    {
      striking_right();
    }
    else if (clockwise == 0) //Anti-Clockwise Direction
    {
      striking_left();
    }
  }
  else if (Curr_Pos >= 65 && Curr_Pos <= 89 && (Prev_Pos == 99 || Prev_Pos == 103 || Prev_Pos == 107 || Prev_Pos == 111 || Prev_Pos == 115 || Prev_Pos == 119))
  {
    if (number == 25)
    {
      if ((Prev_Pos == 'w' && Curr_Pos == 'A') || (Prev_Pos == 'A' && Curr_Pos == 'C') || (Prev_Pos == 'C' && Curr_Pos == 'B'))
      {
        striking_left();
      }
      else if ((Prev_Pos == 'c' && Curr_Pos == 'B') || (Prev_Pos == 'B' && Curr_Pos == 'C') || (Prev_Pos == 'C' && Curr_Pos == 'A'))
      {
        striking_right();
      }
      else if ((Prev_Pos == 'G' && Curr_Pos == 'B') || (Prev_Pos == 'D' && Curr_Pos == 'C') || (Prev_Pos == 'U' && Curr_Pos == 'C'))
      {
        striking_middle();
      }
    }

    else if (number == 27)
    {
      if ((Prev_Pos == 'c' && Curr_Pos == 'G') || (Prev_Pos == 'G' && Curr_Pos == 'F') || (Prev_Pos == 'F' && Curr_Pos == 'H'))
      {
        striking_left();
      }
      else if ((Prev_Pos == 'g' && Curr_Pos == 'H') || (Prev_Pos == 'H' && Curr_Pos == 'F') || (Prev_Pos == 'F' && Curr_Pos == 'G') || (Prev_Pos == 'O' && Curr_Pos == 'F'))
      {
        striking_right();
      }
      else if ((Prev_Pos == 'B' && Curr_Pos == 'G') || (Prev_Pos == 'E' && Curr_Pos == 'F'))
      {
        striking_middle();
      }
    }

    else if (number == 28)
    {
      if ((Prev_Pos == 'g' && Curr_Pos == 'I') || (Prev_Pos == 'I' && Curr_Pos == 'O') || (Prev_Pos == 'O' && Curr_Pos == 'J') || (Prev_Pos == 'F' && Curr_Pos == 'O'))
      {
        striking_left();
      }
      else if ((Prev_Pos == 'k' && Curr_Pos == 'J') || (Prev_Pos == 'J' && Curr_Pos == 'O') || (Prev_Pos == 'O' && Curr_Pos == 'I'))
      {
        striking_right();
      }
      else if ((Prev_Pos == 'N' && Curr_Pos == 'O') || (Prev_Pos == 'K' && Curr_Pos == 'J'))
      {
        striking_middle();
      }
    }

    else if (number == 33)
    {
      if ((Prev_Pos == 's' && Curr_Pos == 'T') || (Prev_Pos == 'T' && Curr_Pos == 'U') || (Prev_Pos == 'U' && Curr_Pos == 'X'))
      {
        striking_left();
      }
      else if ((Prev_Pos == 'w' && Curr_Pos == 'X') || (Prev_Pos == 'X' && Curr_Pos == 'U') || (Prev_Pos == 'U' && Curr_Pos == 'T') || (Prev_Pos == 'C' && Curr_Pos == 'U'))
      {
        striking_right();
      }
      else if ((Prev_Pos == 'S' && Curr_Pos == 'T') || (Prev_Pos == 'V' && Curr_Pos == 'U'))
      {
        striking_middle();
      }
    }

    else if (number == 31)
    {
      if ((Prev_Pos == 'o' && Curr_Pos == 'Q') || (Prev_Pos == 'Q' && Curr_Pos == 'R') || (Prev_Pos == 'R' && Curr_Pos == 'S') || (Prev_Pos == 'L' && Curr_Pos == 'R'))
      {
        striking_left();
      }
      else if ((Prev_Pos == 's' && Curr_Pos == 'S') || (Prev_Pos == 'S' && Curr_Pos == 'R') || (Prev_Pos == 'R' && Curr_Pos == 'Q'))
      {
        striking_right();
      }
      else if ((Prev_Pos == 'T' && Curr_Pos == 'S') || (Prev_Pos == 'W' && Curr_Pos == 'R'))
      {
        striking_middle();
      }
    }
    else if (number  == 30)
    {
      if ((Prev_Pos == 'k' && Curr_Pos == 'K') || (Prev_Pos == 'K' && Curr_Pos == 'L') || (Prev_Pos == 'L' && Curr_Pos == 'P'))
      {
        striking_left();
      }
      else if ((Prev_Pos == 'o' && Curr_Pos == 'P') || (Prev_Pos == 'P' && Curr_Pos == 'L') || (Prev_Pos == 'L' && Curr_Pos == 'K') || (Prev_Pos == 'R' && Curr_Pos == 'L'))
      {
        striking_right();
      }
      else if ((Prev_Pos == 'J' && Curr_Pos == 'K') || (Prev_Pos == 'M' && Curr_Pos == 'L'))
      {
        striking_middle();
      }
    }

    else if (number  == 26)
    {
      if ((Prev_Pos == 'F' && Curr_Pos == 'G') || (Prev_Pos == 'O' && Curr_Pos == 'F') || (Prev_Pos == 'G' && Curr_Pos == 'B') || (Prev_Pos == 'c' && Curr_Pos == 'B') || (Prev_Pos == 'B' && Curr_Pos == 'C') || (Prev_Pos == 'C' && Curr_Pos == 'D'))
      {
        striking_left();
      }
      else if ((Prev_Pos == 'C' && Curr_Pos == 'B') || (Prev_Pos == 'U' && Curr_Pos == 'C') || (Prev_Pos == 'c' && Curr_Pos == 'G') || (Prev_Pos == 'B' && Curr_Pos == 'G') || (Prev_Pos == 'G' && Curr_Pos == 'F') || (Prev_Pos == 'F' && Curr_Pos == 'E'))
      {
        striking_right();
      }
      else if ((Prev_Pos == 'A' && Curr_Pos == 'C') || (Prev_Pos == 'H' && Curr_Pos == 'F'))
      {
        striking_middle();
      }
    }

    else if (number  == 29)
    {
      if ((Prev_Pos == 'K' && Curr_Pos == 'J') || (Prev_Pos == 'J' && Curr_Pos == 'O') || (Prev_Pos == 'k' && Curr_Pos == 'J') || (Prev_Pos == 'O' && Curr_Pos == 'N') || (Prev_Pos == 'M' && Curr_Pos == 'L') || (Prev_Pos == 'B' && Curr_Pos == 'C') || (Prev_Pos == 'C' && Curr_Pos == 'D'))
      {
        striking_left();
      }
      else if ((Prev_Pos == 'C' && Curr_Pos == 'B') || (Prev_Pos == 'U' && Curr_Pos == 'C') || (Prev_Pos == 'c' && Curr_Pos == 'G') || (Prev_Pos == 'B' && Curr_Pos == 'G') || (Prev_Pos == 'G' && Curr_Pos == 'F') || (Prev_Pos == 'F' && Curr_Pos == 'E'))
      {
        striking_right();
      }
      else if ((Prev_Pos == 'P' && Curr_Pos == 'L') || (Prev_Pos == 'I' && Curr_Pos == 'O'))
      {
        striking_middle();
      }
    }

    else if (number  == 32)
    {
      if ((Prev_Pos == 'R' && Curr_Pos == 'W') || (Prev_Pos == 'W' && Curr_Pos == 'V') || (Prev_Pos == 'V' && Curr_Pos == 'U') || (Prev_Pos == 'U' && Curr_Pos == 'T') || (Prev_Pos == 'T' && Curr_Pos == 'S') || (Prev_Pos == 'S' && Curr_Pos == 'R') || (Prev_Pos == 's' && Curr_Pos == 'S') || (Prev_Pos == 'C' && Curr_Pos == 'U'))
      {
        striking_left();
      }
      else if ((Prev_Pos == 'W' && Curr_Pos == 'R') || (Prev_Pos == 'V' && Curr_Pos == 'W') || (Prev_Pos == 'U' && Curr_Pos == 'V') || (Prev_Pos == 'T' && Curr_Pos == 'U') || (Prev_Pos == 'S' && Curr_Pos == 'T') || (Prev_Pos == 'R' && Curr_Pos == 'S') || (Prev_Pos == 's' && Curr_Pos == 'T') || (Prev_Pos == 'L' && Curr_Pos == 'R'))
      {
        striking_right();
      }
      else if ((Prev_Pos == 'X' && Curr_Pos == 'U') || (Prev_Pos == 'Q' && Curr_Pos == 'R'))
      {
        striking_middle();
      }
    }
  }
}









void setup()
{
  DDRA = 0x0F;
  DDRL = DDRL | 0x18;
  PORTL = PORTL | 0x18;
  DDRE  = DDRE & 0xEF;
  PORTE = PORTE | 0x10;
  DDRE  = DDRE & 0xDF;
  PORTE = PORTE | 0x20;
  //Servos initialise
  pinMode(pinNo, INPUT);
  UCSR0B = 0x00;
  UCSR0A = 0x00;
  UCSR0C = 0x06;
  UBRR0L = 0x5F;
  UBRR0H = 0x00;
  UCSR0B = 0x98;

  // Serial2.begin(9600);
  myservo1.attach(11);
  myservo2.attach(12);
  myservo2.write(180);
  delay(1000);
  /*
       for (int i = 0; i < 10; i++)
    {
      DistanceDash = sharp_distance();
      Serial.println(DistanceDash);
      delay(500);
    }
  */
  Curr_Pos = 'U';
  Prev_Pos = 'X';
  hit(32);
  myservo1.detach();
  myservo2.detach();

}
void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  /* DistanceDash = sharp_distance();
    Serial2.println(DistanceDash);
    delay(1000);
  */

}
