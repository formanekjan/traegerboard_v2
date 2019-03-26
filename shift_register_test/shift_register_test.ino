#define SERIN_595 21 
#define CLK_595 13 
#define RCLK_595 12 //the latch pin, to transfer from buffer to output

//achtung invertiert in meinem fall
//achtung max 50Khz


/*void enableMT3608() {
  
}*/

byte reverseByte(byte data_) {
  byte temp = 0x00;

  temp = temp | (data_ & 0B00000001);
  
  int n;
  for(n=0; n<7;n++) {
    temp = temp << 1;
    data_ = data_ >> 1;
    temp = temp | (data_ & 0B00000001);
    
  }

  return temp;
  
}

//testen ob man 16bit mit 2xigem Aufruf ausschiften kann
void shiftoutSlow(byte clkPin, byte  rclkPin, byte dataPin, byte data_) {
  pinMode(dataPin, OUTPUT);
  pinMode(clkPin, OUTPUT);
  pinMode(rclkPin, OUTPUT); 

  //idle clock = low
  digitalWrite(clkPin, HIGH);
  digitalWrite(rclkPin, HIGH); //latch, call at end

  int count = 0;
  for(count =0; count <8; count++) {
    if(data_ & B00000001) {
      digitalWrite(dataPin, HIGH);
    }
    else {
      digitalWrite(dataPin, LOW);
    }
    delay(1); //1ms delay = 1khz
    digitalWrite(clkPin, LOW); //übernehmen
    delay(1); //1ms delay = 1khz
    digitalWrite(clkPin, HIGH); //clk release
    data_ = data_ >> 1;
  }
  
  digitalWrite(rclkPin, LOW); //latchout data
  delay(1);
  digitalWrite(rclkPin, HIGH); //relase latchout
}

void latch() {
  digitalWrite(rclkPin, LOW); //latchout data
  delay(1);
  digitalWrite(rclkPin, HIGH); //relase latchout
}


void setup() {

  Serial.begin(115200);
  Serial.print("setup");
  byte testbyte = 0B00000001;
  byte testbyte2 = 0B00001000;
  byte shiftedbyte = reverseByte(testbyte2);
  //byte testbyte = 254;
  //byte reversedtestbyte;
  //reversedtestbyte =123;
  Serial.print("testbyte:"+String(testbyte2)); 
  Serial.print("shiftedbyte:"+String(shiftedbyte)); 
  
}

void loop() {
  //shiftoutSlow(CLK_595, RCLK_595, SERIN_595, 0x00);
  shiftoutSlow(CLK_595, RCLK_595, SERIN_595, B00000000);
  shiftoutSlow(CLK_595, RCLK_595, SERIN_595, B11111111);
 
  
  
  

}
