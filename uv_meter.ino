#include "HCDisplay.h"
#include<Wire.h>
#include "driver/periph_ctrl.h"

#include<ADS1115_WE.h> 


#define CORRECTION_FACTOR 0.7
#define TIMEOUT_SECS 60

#define I2C_ADDRESS 0x48
// Default pins
#define CE_PIN      7
#define DC_PIN      20
#define CLK_PIN     4
#define DIN_PIN     6
#define RST_PIN   21  // Optional reset pin


ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);  
HCDisplay HCDisplay;    //Creates an instance of the HCDisplay library
  
unsigned int MaxX, MaxY;
float uvMAX;
int animpos = 0;
boolean FGColour = 1;
  
void gotosleep() {
      //WiFi.disconnect();
      //display.hibernate();
      //SPI.end();
      Wire.end();
      pinMode(SS, INPUT_PULLUP );
      pinMode(6, INPUT_PULLUP );
      pinMode(4, INPUT_PULLUP );
      pinMode(8, INPUT_PULLUP );
      pinMode(9, INPUT_PULLUP );
      pinMode(10, INPUT_PULLUP );
      pinMode(20, INPUT_PULLUP );
      pinMode(21, INPUT_PULLUP );
      pinMode(1, INPUT_PULLUP );
      pinMode(2, INPUT_PULLUP );
      pinMode(3, INPUT_PULLUP );
      pinMode(5, INPUT_PULLUP);

      //delay(10000);
      //rtc_gpio_isolate(gpio_num_t(SDA));
      //rtc_gpio_isolate(gpio_num_t(SCL));
      //periph_module_disable(PERIPH_I2C0_MODULE);  
      //digitalWrite(SDA, 0);
      //digitalWrite(SCL, 0);
      esp_deep_sleep_enable_gpio_wakeup(1 << 5, ESP_GPIO_WAKEUP_GPIO_LOW);
      esp_sleep_enable_timer_wakeup(100 * 1000000ULL);
      delay(1);
      esp_deep_sleep_start();
      //esp_light_sleep_start();
      delay(1000);
}  


float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();
  while(adc.isBusy()){}
  voltage = adc.getResult_mV(); // alternative: getResult_mV for Millivolt
  return voltage;
}
  
void setup()
{
  Wire.begin();
  adc.init();
  adc.setVoltageRange_mV(ADS1115_RANGE_4096);
  adc.setConvRate(ADS1115_860_SPS);
  HCDisplay.Init(CE_PIN, DC_PIN,  RST_PIN);
  HCDisplay.Sleep(false);
  HCDisplay.Contrast(16);
  HCDisplay.SetFont(MedProp_12ptFont);
  HCDisplay.Flip(SCREEN_R180);
  HCDisplay.AutoRefresh(false);

}
     
     
void loop()
{

  int adc0 = readChannel(ADS1115_COMP_0_GND);
  float volts = readChannel(ADS1115_COMP_3_GND) / 500.0;
  //if (adc0 < 20) {adc0 = 0;}
  float UVindex = adc0/100.0;
  UVindex = UVindex * CORRECTION_FACTOR;
  if (UVindex < 0.04) {UVindex = 0;}
  if (UVindex > uvMAX) {uvMAX = UVindex;}
  

  HCDisplay.Clear();
  HCDisplay.SetFont(SystemFont);
  HCDisplay.ScaleXY(2,2);
  HCDisplay.Pos(0,0);
  HCDisplay.Print("UV ");
  HCDisplay.Print(UVindex, 2);
  HCDisplay.Pos(0,14+14);
  HCDisplay.Print("MAX ");
  HCDisplay.Print(uvMAX, 2);
  HCDisplay.Pos(0,14+14+14+6);
  HCDisplay.SetFont(SystemFont);
  HCDisplay.ScaleXY(1,1);
  HCDisplay.Print("vBat ");
  HCDisplay.Print(volts, 3);
  HCDisplay.Print("v");
  HCDisplay.Pos(0,14+14+14+14);
  HCDisplay.Print(adc0);
  HCDisplay.Print("mV");
  HCDisplay.Rect(animpos, 18, animpos+4, 22, FILLED); //draw our little status indicator
  animpos += 1; //make it fly
  if (animpos > 128) {animpos = 0;} //make it wrap around
  HCDisplay.Refresh();

  

if (millis() > (TIMEOUT_SECS * 1000))  {gotosleep(); }
}