#include <FastLED.h>
//#include <stdio.h>
//#include <ros.h>
//#include <std_msgs/Int8.h>

#define LED_PIN     6
#define NUM_LEDS    34
#define BRIGHTNESS  255
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];


//int mau_led = 0;
int a = 0;
uint8_t colorIndex = 0;
bool newData = false;
//String receivedData="";

//ros::NodeHandle nh;

#define UPDATES_PER_SECOND 50 // số lần cập nhật màu trên chuỗi led trên mỗi dây

CRGBPalette16 currentPalette; // biến lưu trừ palette(bảng màu) màu sắc cho chuỗi led.
TBlendType    currentBlending; // biến xác định kiểu kết hợp màu sắc của chuỗi led

//extern CRGBPalette16 myRedWhiteBluePalette;
//extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;

//void callback1(const std_msgs::Int8& ahihi)
//{
//  mau_led = ahihi.data;
//}

//ros::Subscriber<std_msgs::Int8> sub1("alo_ban", &callback1);

void setup() {
    Serial.begin(9600);
    delay( 3000 );
    
//    nh.getHardware() ->setBaud(9600);
//    nh.initNode();
//    nh.subscribe(sub1);
  
    // cấu hình fastled, nói cho fastled sử dụng các thông số đã đặt
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.setBrightness(BRIGHTNESS); // độ sáng ban đầu cho toàn bộ chuỗi led
  
    currentPalette = RainbowColors_p; // bảng màu ban đầu
    currentBlending = LINEARBLEND; // thay đổi màu theo kiểu trơn tru từ bảng màu này qua bảng màu khác (khác với NOBLEND)
}


void loop()
{
  if (newData) {
   switch (a) 
   {
    case 0:
      ddo();
      break;
    case 1:
      vang();
      break;
    case 2:
      xanh_la();
      break;
    case 3:
      xanh_duong();
      break;
    case 4:
      di_sac();
      break;
    case 5:
      sac_pin_moi();
      break;
    case 6:
      nhieu_mau();
      break;
    case 7:
      loi();
      break;
   }
  }
   nhan_serial();

//    nh.spinOnce();
}

void nhan_serial()
{
if (Serial.available() > 0) 
  {
    // Read an integer from the serial port
    a = Serial.parseInt(); 
    newData = true; 
     if (Serial.read() == '\n') {
//      newData = true;
        // The received data is a valid integer
        
        Serial.println(a);
     }
  }
}

//void nhan_serial()
//{
//  // Đọc dữ liệu từ cổng serial
//  while (Serial.available() > 0) {
//    char c = Serial.read();
//    receivedData += c;
//  }
//
//  // Loại bỏ dữ liệu cũ, chỉ giữ lại dữ liệu mới nhất
//  if (receivedData.length() > 0) {
//    receivedData = receivedData.substring(receivedData.length() - 1);
//  }
//
//  // Xử lý dữ liệu mới nhất ở đây
//  // Ví dụ: in ra màn hình Serial Monitor
//  Serial.println(receivedData);
//}

// hàm thiết lập màu cho led
void FillLEDsFromPaletteColors( uint8_t colorIndex)
{
//   Serial.println("456");
    uint8_t brightness = 255;
    
    for( int i = 0; i < NUM_LEDS; ++i) {
        leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
        colorIndex += 3;
    }
}

// hàm thay đổi bảng màu theo thời gian
void ChangePalettePeriodically()
{
    uint8_t secondHand = (millis() / 1000) % 60;
    static uint8_t lastSecond = 99;
    
//    Serial.println("111");
    
    if( lastSecond != secondHand) {
        lastSecond = secondHand;
        if( secondHand == 0)  { currentPalette = RainbowColors_p;         currentBlending = LINEARBLEND; }
        if( secondHand == 3)  { currentPalette = RainbowStripeColors_p;   currentBlending = LINEARBLEND;  }
        if( secondHand == 6)  { currentPalette = RainbowStripeColors_p;   currentBlending = LINEARBLEND; }
        if( secondHand == 9)  { SetupPurpleAndGreenPalette();             currentBlending = LINEARBLEND; }
        if( secondHand == 36)  { SetupTotallyRandomPalette();              currentBlending = LINEARBLEND; }
//        if( secondHand == 45)  { currentPalette = myRedWhiteBluePalette_p; currentBlending = LINEARBLEND;  }
//        if( secondHand == 54)  { currentPalette = myRedWhiteBluePalette_p; currentBlending = LINEARBLEND; }
    }
}

void SetupTotallyRandomPalette()
{
//   Serial.println("333");
   for( int i = 0; i < 16; ++i) 
   {
      currentPalette[i] = CHSV( random8(), 255, random8());
   }
}

void SetupPurpleAndGreenPalette()
{
//  Serial.println("444");
    CRGB purple = CHSV( HUE_PURPLE, 255, 255);
    CRGB green  = CHSV( HUE_GREEN, 255, 255);
    CRGB red  = CHSV( HUE_RED, 255, 255);
    CRGB black  = CRGB::Black;

//    Serial.println("222");
    
    currentPalette = CRGBPalette16(
                                   green,  green,  red,  black,
                                   purple, purple, red,  black,
                                   green,  green,  red,  black,
                                   purple, purple, red,  black );
}

//const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM =
//{
//    CRGB::Red,
//    CRGB::Gray,
//    CRGB::Blue,
//    CRGB::Black,
//    
//    CRGB::Red,
//    CRGB::Gray,
//    CRGB::Blue,
//    CRGB::Black,
//    
//    CRGB::Red,
//    CRGB::Red,
//    CRGB::Gray,
//    CRGB::Gray,
//    CRGB::Blue,
//    CRGB::Blue,
//    CRGB::Black,
//    CRGB::Black
//};

void ddo()
{
  for( int i=0 ; i < NUM_LEDS; i++ ) {
//    Serial.println("555");
    leds[i] = CRGB(0, 255, 0);
  }
  FastLED.show();
  FastLED.delay(1000 / UPDATES_PER_SECOND);
}

void vang()
{
//  Serial.println("2345");
  for( int i=0 ; i < NUM_LEDS; i++ ) {
//    Serial.println("2345");
    leds[i] = CRGB(150, 255, 0);
  }
   FastLED.show();
   FastLED.delay(1000 / UPDATES_PER_SECOND);
}

void xanh_la()
{
   for( int i=0 ; i < NUM_LEDS; i++ ) {
  leds[i] = CRGB(255, 0, 0);
  }
   FastLED.show();
   FastLED.delay(1000 / UPDATES_PER_SECOND);
}

void xanh_duong()
{
  for( int i=0 ; i < NUM_LEDS; i++ ) {
  leds[i] = CRGB(255, 0, 255);
  }
   FastLED.show();
   FastLED.delay(1000 / UPDATES_PER_SECOND);
}

void sac_pin()
{
//  Serial.println("3456");
  for(int i = 0; i < NUM_LEDS/2 +2; i++) {
    Serial.print("2345");
    leds[i] = CRGB(0, 0, 255);
    leds[i+1] = CRGB(0, 0, 255);
    leds[i+2] = CRGB(0, 0, 255);
    leds[NUM_LEDS-i] = CRGB(0, 0, 255);
    leds[NUM_LEDS-i-1] = CRGB(0, 0, 255);
    leds[NUM_LEDS-i-2] = CRGB(0, 0, 255);
    FastLED.show();
    delay(50);
    leds[i] = CRGB::White;
    leds[i+1] = CRGB::White;
    leds[i+2] = CRGB::White;
    leds[NUM_LEDS-i] = CRGB::White;
    leds[NUM_LEDS-i-1] = CRGB::White;
    leds[NUM_LEDS-i-2] = CRGB::White;
    FastLED.show();
  }
}
void di_sac()
{
  for (int i = 0; i < NUM_LEDS; i++) 
  {
    leds[i] = CRGB::Yellow;
  }
  FastLED.show();
  delay(50);
  for (int i = 0; i < NUM_LEDS; i++) 
  {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
  delay(50);
}
//void nhieu_mau()
//{
//    for (int j =0; j<100;j++){
//    Serial.println("123");
//    ChangePalettePeriodically();
//    
//    FillLEDsFromPaletteColors(colorIndex);
//    
//    FastLED.show();
//    FastLED.delay(1000 / UPDATES_PER_SECOND);
//    colorIndex++;
//    }
//}

void nhieu_mau()
{
//    Serial.println("123");
    ChangePalettePeriodically();
    
    FillLEDsFromPaletteColors(colorIndex);
    
    FastLED.show();
    FastLED.delay(1000 / UPDATES_PER_SECOND);
    colorIndex++;
}

void sac_pin_moi ()
{
  uint16_t posBeat  = beatsin16(30, 0, NUM_LEDS - 1, 0, 0);
  uint16_t posBeat2 = beatsin16(60, 0, NUM_LEDS - 1, 0, 0);

  uint16_t posBeat3 = beatsin16(30, 0, NUM_LEDS - 1, 0, 32767);
  uint16_t posBeat4 = beatsin16(60, 0, NUM_LEDS - 1, 0, 32767);

  leds[(posBeat + posBeat2) / 2]  = CRGB(255, 0, 255);
  leds[(posBeat3 + posBeat4) / 2]  = CRGB(255, 0, 255);

  fadeToBlackBy(leds, NUM_LEDS, 7);
  FastLED.show();
  
}

void loi()
{
  for (int i = 0; i < NUM_LEDS; i++) 
  {
    leds[i] = CRGB(0, 255, 0);
  }
  FastLED.show();
  FastLED.delay(1000 / UPDATES_PER_SECOND);
  delay(50);
  for (int i = 0; i < NUM_LEDS; i++) 
  {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
  FastLED.delay(1000 / UPDATES_PER_SECOND);
  delay(50);
}
