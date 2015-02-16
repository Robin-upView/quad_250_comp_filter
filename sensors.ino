

void imu_Valget ()
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float pitchAcc, rollAcc;

  // Integrate the gyroscope data -> int(angularSpeed) = angle
  pitch += ((float)(gx-biasX)*2000.0f/32768.0f) * G_Dt; // Angle around the X-axis
  roll -= ((float)(gy-biasY)*2000.0f/32768.0f) * G_Dt; // Angle around the Y-axis

  // Compensate for drift with accelerometer data if no vibration
  // Weight for accelerometer info (<0.75G = 0.0, 1G = 1.0 , >1.25G = 0.0)
  int forceMagnitudeApprox = abs(ax) + abs(ay) + abs(az);


  if (forceMagnitudeApprox > 6144 && forceMagnitudeApprox < 10240) //was 6144 10240
  {
    // Turning around the X axis results in a vector on the Y-axis
    pitchAcc = atan2((float)ay, (float)az) * 180 / M_PI;
    pitch = pitch * 0.98 + pitchAcc * 0.02;

    // Turning around the Y axis results in a vector on the X-axis
    rollAcc = atan2((float)ax, (float)az) * 180 / M_PI;
    roll = roll * 0.98 + rollAcc * 0.02;
  }
 
}



void calib_gyro()
{
  for(int c=0; c<10; c++)
  { 
    digitalWrite(led, HIGH);
    delay(200);
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    digitalWrite(led, LOW); 
    delay(200);
  }

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  biasX = gx;
  biasY = gy;
  biasZ = gz;

  for(int i=0;i<100;i++)
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    biasX = biasX*0.80 + gx*0.20;
    biasY = biasY*0.80 + gy*0.20;
    biasZ = biasZ*0.80 + gz*0.20;
    delay(20);
  }
}


void IMU_print ()
{
   Serial.print(G_Dt,5);
  Serial.print(" ");
  Serial.print(rc[3]);
  Serial.print(" ");
  Serial.print(pid_altitude);
Serial.print(" ");
  Serial.print(altitude_demand);
  Serial.println(" ");

  
  while(Serial.available())
  {
    char c = Serial.read();
    
    if (c=='a')
   {
     kp+=0.5;
   } 
   else if (c=='z')
   {
    kp-=0.5; 
   }
    else if (c=='q')
   {
     ki+=0.5;
   }
    else if (c=='s')
   {
     ki-=0.5;
   }
       else if (c=='w')
   {
     kd+=0.5;
   }
    else if (c=='x')
   {
     kd-=0.5;
   }
    
  }

 
}

float getAltitude(float press, float temp) {
  //return (1.0f - pow(press/101325.0f, 0.190295f)) * 4433000.0f;
  return ((pow((sea_press / press), 1/5.257) - 1.0) * (temp + 273.15)) / 0.0065;
}

void pushAvg(float val) {
  movavg_buff[movavg_i] = val;
  movavg_i = (movavg_i + 1) % MOVAVG_SIZE;
}

float getAvg(float * buff, int size) {
  float sum = 0.0;
  for(int i=0; i<size; i++) {
    sum += buff[i];
  }
  return sum / size;
}


