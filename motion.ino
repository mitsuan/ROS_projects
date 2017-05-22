


void pinMode_setup()
{
  for(int i=0;i<3;i++)
  {
    pinMode(s_p[i],OUTPUT);
    pinMode(d_p[i],OUTPUT);
  }
}


void inverseKinematics()
{
  float m[3][3]={{-0.33, 0.58, 0.33}, {-0.33, -0.58, 0.33}, {0.67, 0, 0.33}};

  s[0]=m[0][0]*x + m[0][1]*y + m[0][2]*w;
  s[1]=m[1][0]*x + m[1][1]*y + m[1][2]*w;
  s[2]=m[2][0]*x + m[2][1]*y + m[2][2]*w;

  for(int i=0;i<3;i++)
  {
    //Determining direction of motors
    if(s[i]>0)
    d[i]=1;
    else
    d[i]=0;

    //Taking absolute values of motor outputs
    s[i]=abs(s[i]);
  }
}

void motion()
{
  //pwm write to motors
  for(int i=0;i<3;i++)
  {
    analogWrite(s_p[i],s[i]);
  }

  //direction to motors
  for(int i=0;i<3;i++)
  {
    digitalWrite(d_p[i],d[i]);
  }
}

/*
void display_motion_values()
{
  Serial.println("-------Motor Values-------");
  for(int i=0;i<3;i++)
  {
    Serial.print("\t s");
    Serial.print(i+1);
    Serial.print(": ");
    Serial.print(s[i]);
    Serial.print(" , ");
    Serial.print(d[i]);
    
  }
  Serial.println();

    Serial.print("x: ");
    Serial.print(x);
    Serial.print("y: ");
    Serial.print(y);
    Serial.print("w: ");
    Serial.println(w);
}
*/
