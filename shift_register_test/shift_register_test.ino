#define SERIN_595 21 
#define CLK_595 13 
#define RCLK_595 12 //the latch pin, to transfer from buffer to output

#define MUX_X1 17 
#define MUX_X2 23 

//achtung invertiert in meinem fall
//achtung max 50Khz

int portExpander = 0x00;

#define EXT1_OUT 0
#define LOAD100MA_EN 1
#define POWERBANK_EN 2
#define MT3608_EN 3
#define MUX4052B_S1 4
#define MUX4052B_S0 5
#define MUX4052B_EN 6
#define EXT2_OUT 7
#define A7_PWR_KEY 8
#define A7_MODULE_EN 9
#define MUX4052A_S1 10
#define MUX4052A_S0 11
#define MUX4052A_EN 12
#define A7_SLEEP 13
#define A7_RESET 14
#define SDS011_EN 15

void updatePorts() {
  shiftoutSlow(CLK_595, RCLK_595, SERIN_595, highByte(portExpander));
  shiftoutSlow(CLK_595, RCLK_595, SERIN_595, lowByte(portExpander));
  latch(RCLK_595);
}

void enableMT3608() {
  portExpander = portExpander & (0 << MT3608_EN); 
  updatePorts();
  
}

void disableMT3608() {
  portExpander = portExpander | (1 << MT3608_EN);
  updatePorts();
}

void setMuxA(byte channel) {
  switch(channel) {
    case 0:
      portExpander = portExpander & ~(1 << MUX4052A_S0);
      portExpander = portExpander & ~(1 << MUX4052A_S1);
      break;
   case 1:
      portExpander = portExpander | (1 << MUX4052A_S0);
      portExpander = portExpander & ~(1 << MUX4052A_S1);
      break;
   case 2:
     portExpander = portExpander & ~(1 << MUX4052A_S0);
     portExpander = portExpander | (1 << MUX4052A_S1);
     break;
   case 3:
    portExpander = portExpander | (1 << MUX4052A_S0);
     portExpander = portExpander | (1 << MUX4052A_S1);
     break;
  }

  updatePorts();
}

void resetPorts() {
  //portExpander = 0xFFFF;
  portExpander = 0x0000;
  updatePorts();
}



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
  //invert data, cause dataline is inverted
  data_ = ~data_;
  data_ = reverseByte(data_);
  pinMode(dataPin, OUTPUT);
  pinMode(clkPin, OUTPUT);
  pinMode(rclkPin, OUTPUT); 

  //idle clock = low
  digitalWrite(rclkPin, HIGH); //relase latchout
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
}

void latch(byte rclkPin) {
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
  resetPorts();
  
}

void loop() {
  //shiftoutSlow(CLK_595, RCLK_595, SERIN_595, 0x00);
  /*shiftoutSlow(CLK_595, RCLK_595, SERIN_595, B00000000);
  shiftoutSlow(CLK_595, RCLK_595, SERIN_595, B11111111);
  latch(RCLK_595);*/
  //enableMT3608();
  setMuxA(1);
  //portExpander = portExpander | (1 << MUX4052A_S0);
  //portExpander = portExpander & (0 << MUX4052A_S1);
  //portExpander = 0x8000;
  //portExpander = 0x4000;
  //portExpander = 0x2000;
  //updatePorts();
  //disableMT3608();
  /*shiftoutSlow(CLK_595, RCLK_595, SERIN_595, B00000010); //15-8
  shiftoutSlow(CLK_595, RCLK_595, SERIN_595, B00001000); //7-0
  latch(RCLK_595);*/
  
  
  

}
