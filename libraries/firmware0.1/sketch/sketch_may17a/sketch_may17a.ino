int sercmd;
float Offset;
void setup() {
Serial.begin(9600);
}

void loop()
{  
   
   while(Serial.available())
  { sercmd=Serial.read();
    Serial.flush();
    Serial.println(sercmd);
    if(sercmd==67)
    { 
      Serial.println("secondo");
      Serial.println("Offset : ");
      delay(1000);
      while(Serial.available() == 0)
      {//Serial.println("vuoto");
       delay(1);
      }
      while(Serial.available())
      {
       Offset = Serial.parseFloat();
       Serial.println(Offset);
    }
  }
}
}
