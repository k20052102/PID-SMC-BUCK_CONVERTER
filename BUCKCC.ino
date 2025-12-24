//-----------------------------------------------------------------------------------------------------
//--- CODE BY KHANH 
//  THƯ VIỆN 
//----------------------------------------------------------------------------------------
#include <EEPROM.h>                   // Lưu cấu hình vào bộ nhớ flash
#include <Wire.h>                     // Thư viện I2C
#include <SPI.h>                      // Thư viện SPI
#include <WiFi.h>                     // Thư viện WiFi cho ESP32
#include <WiFiClient.h>               // Client WiFi
#include <BlynkSimpleEsp32.h>         // Thư viện Blynk IOT điều khiển qua app điện thoại 
#include <LiquidCrystal_I2C.h>        // Thư viện LCD I2C
#include <Adafruit_ADS1X15.h>         // Thư viện ADC ADS1115 
LiquidCrystal_I2C lcd(0x27,16,2);     // Khởi tạo LCD I2C địa chỉ 0x27, 16x2
TaskHandle_t Core2;                   // Chạy trên core 2 ESP32        
Adafruit_ADS1115 ads;                 // Khởi tạo ADC ADS1115

//  KHAI BÁO CHÂN GPIO
//---------------------------------------------------------------------------------------
#define backflow_MOSFET 27          // MOSFET chống dòng ngược
#define buck_IN         33          // Chân PWM điều khiển MOSFET buck
#define buck_EN         32          // Chân enable driver buck (Cho phép IC hoạt động)
#define LED             2           // LED báo trạng thái
#define FAN             16          // Quạt làm tản nhiệt
#define ADC_ALERT       34          // Chân ALERT từ ADC
#define TempSensor      35          // Cảm biến nhiệt độ
#define buttonLeft      18          // Nút trái
#define buttonRight     17          // Nút phải
#define buttonBack      19          // Nút quay lại
#define buttonSelect    23          // Nút chọn

//   THÔNG TIN WIFI
//----------------------------------------------------------------------------------------
char 
auth[] = "Jr4a7DcxKYP19F6DlXCTtsx05c9NDkvb",       //     Token 
ssid[] = "KCX An Sinh",                            //     WiFi SSID
pass[] = "88889999";                               //     WiFi Password

//   THAM SỐ
//----------------------------------------------------------------------------------------
bool                                  
MPPT_Mode               = 1,           //   1 = dùng MPPT, 0 = CC-CV
output_Mode             = 1,           //   0 = nguồn DC, 1 = sạc pin
disableFlashAutoLoad    = 0,           //   1 = không lưu cấu hình EEPROM 
enablePPWM              = 1,           //   Predictive PWM – tăng tốc độ đáp ứng
enableWiFi              = 1,           //   Bật WiFi = 1, tắt = 0 ;
enableFan               = 1,           //   Bật Fan = 1, tắt bằng = 0;
enableBluetooth         = 1,           //   Bật Bluetooth = 1, tắt bằng = 0;
enableLCD               = 1,           //   Bật LCD
enableLCDBacklight      = 1,           //   Bật đèn nền LCD
overrideFan             = 0,           //   1 = quạt luôn chạy
enableDynamicCooling    = 0;           //   Điều khiển quạt theo PWM = 1
int
serialTelemMode         = 1,           //   Chế độ in serial
pwmResolution           = 11,          //   PWM 11 bit (0–2047)
pwmFrequency            = 39000,       //   PWM 39kHz (Tần số cho buck)
temperatureFan          = 60,          //   Nhiệt độ bật quạt
temperatureMax          = 90,          //   Quá nhiệt → shutdown
telemCounterReset       = 0,           //   Reset bộ đếm telemetry
errorTimeLimit          = 1000,        //   Thời gian lỗi cho phép
errorCountLimit         = 5,           //   Số lần lỗi tối đa
millisRoutineInterval   = 250,         //   Xử lý hệ thống
millisSerialInterval    = 1,           //   Serial
millisLCDInterval       = 1000,        //   LCD
millisWiFiInterval      = 2000,        //   WiFi
millisLCDBackLInterval  = 2000,        //   Kiểm tra backlight
backlightSleepMode      = 0,           //   Chế độ ngủ LCD
baudRate                = 500000;      //   Serial baudrate
float 
voltageBatteryMax       = 27.3000,     //   Điện áp sạc tối đa
voltageBatteryMin       = 22.4000,     //   Điện áp pin thấp
currentCharging         = 30.0000,     //   Dòng sạc tối đa
electricalPrice         = 3.460;       //   Giá điện VNĐ


//   THAM SỐ HIỆU CHỈNH
//-----------------------------------------------------------------------------------

bool
ADS1015_Mode            = 0;          //   1 = ADS1015, 0 = ADS1115
int
ADC_GainSelect          = 2,          //   ADC Gain Selection (0→±6.144V 3mV/bit, 1→±4.096V 2mV/bit, 2→±2.048V 1mV/bit)
avgCountVS              = 3,          //   Hệ số chia áp đầu vào ( Nên để 3 )
avgCountCS              = 4,          //   Hệ số chia áp đầu ra  ( Nên để 4 )
avgCountTS              = 500;        //   Số mẫu trung bình cảm biến nhiệt
float
inVoltageDivRatio       = 40.2156,    //  Hệ số chia áp đo điện áp đầu vào
outVoltageDivRatio      = 24.5000,    //  Hệ số chia áp đo điện áp đầu ra
vOutSystemMax           = 50.0000,    //  Điện áp đầu ra tối đa hệ thống cho phép
cOutSystemMax           = 50.0000,    //  Dòng đầu ra tối đa hệ thống cho phép
ntcResistance           = 100000.00,  //  Điện trở NTC 
voltageDropout          = 1.0000,     //  Sụt áp buck do giới hạn duty cycle
voltageBatteryThresh    = 1.5000,     //  Ngưỡng cắt khi điện áp quá thấp
currentInAbsolute       = 31.0000,    //  Dòng đầu vào tối đa cho phép
currentOutAbsolute      = 50.0000,    //  Dòng đầu ra tối đa cho phép
PPWM_margin             = 99.5000,    //  Giới hạn duty cycle cho Predictive PWM
PWM_MaxDC               = 97.0000,    //  Duty cycle tối đa (%)
efficiencyRate          = 1.0000,     //  Hiệu suất giả định của buck
currentMidPoint         = 2.5250,     //  Điện áp offset cảm biến dòng
currentSens             = 0.0000,     //  Độ nhạy dòng (tính toán)
currentSensV            = 0.0660,     //  Độ nhạy cảm biến dòng (V/A)
vInSystemMin            = 10.000;     //  Điện áp đầu vào tối thiểu cho phép

//    BIẾN HỆ THỐNG 
//   

bool
buckEnable            = 0,           // Trạng thái kích hoạt buck
fanStatus             = 0,           // Trạng thái quạt (1 = bật)
bypassEnable          = 0,           // Bypass hệ thống
chargingPause         = 0,           // Tạm dừng sạc
lowPowerMode          = 0,           // Chế độ tiết kiệm điện
buttonRightStatus     = 0,           // Trạng thái nút phải
buttonLeftStatus      = 0,           // Trạng thái nút trái
buttonBackStatus      = 0,           // Trạng thái nút quay lại
buttonSelectStatus    = 0,           // Trạng thái nút chọn
buttonRightCommand    = 0,           // Lệnh nút phải
buttonLeftCommand     = 0,           // Lệnh nút trái
buttonBackCommand     = 0,           // Lệnh nút back
buttonSelectCommand   = 0,           // Lệnh nút select
settingMode           = 0,           // Đang trong chế độ cài đặt
setMenuPage           = 0,           // Trang menu cài đặt
boolTemp              = 0,           // Biến tạm
flashMemLoad          = 0,           // Trạng thái load EEPROM
confirmationMenu      = 0,           // Menu xác nhận
WIFI                  = 0,           // Trạng thái WiFi
BNC                   = 0,           // Lỗi pin không kết nối
REC                   = 0,           // Trạng thái ghi dữ liệu
FLV                   = 0,           // Lỗi điện áp thấp
IUV                   = 0,           // Lỗi điện áp vào thấp
IOV                   = 0,           // Lỗi điện áp vào cao
IOC                   = 0,           // Lỗi dòng vào cao
OUV                   = 0,           // Lỗi điện áp ra thấp
OOV                   = 0,           // Lỗi điện áp ra cao
OOC                   = 0,           // Lỗi dòng ra cao
OTE                   = 0;           // Lỗi quá nhiệt
int
inputSource           = 0,           // 0 = không có nguồn, 1 = solar, 2 = pin
avgStoreTS            = 0,           // Biến tích lũy nhiệt độ
temperature           = 0,           // Nhiệt độ hiện tại
sampleStoreTS         = 0,           // Đếm mẫu nhiệt
pwmMax                = 0,           // Giá trị PWM tối đa
pwmMaxLimited         = 0,           // PWM giới hạn an toàn
PWM                   = 0,           // Giá trị PWM hiện tại
PPWM                  = 0,           // PWM dự đoán
pwmChannel            = 0,           // Kênh PWM
batteryPercent        = 0,           // % pin
errorCount            = 0,           // Bộ đếm lỗi
menuPage              = 0,           // Trang menu chính
subMenuPage           = 0,           // Trang menu phụ
ERR                   = 0,           // Mã lỗi
conv1                 = 0,           // Biến chuyển đổi 1
conv2                 = 0,           // Biến chuyển đổi 2
intTemp               = 0;           // Biến nhiệt tạm
float
VSI                   = 0.0000,      // Điện áp ADC đầu vào
VSO                   = 0.0000,      // Điện áp ADC đầu ra
CSI                   = 0.0000,      // Điện áp ADC cảm biến dòng
CSI_converted         = 0.0000,      // Dòng điện sau khi chuyển đổi
TS                    = 0.0000,      // Giá trị ADC nhiệt độ
powerInput            = 0.0000,      // Công suất đầu vào (W)
powerInputPrev        = 0.0000,      // Công suất đầu vào trước đó
powerOutput           = 0.0000,      // Công suất đầu ra
energySavings         = 0.0000,      // Điện năng tiết kiệm
voltageInput          = 0.0000,      // Điện áp đầu vào
voltageInputPrev      = 0.0000,      // Điện áp đầu vào trước đó
voltageOutput         = 0.0000,      // Điện áp đầu ra
currentInput          = 0.0000,      // Dòng đầu vào
currentOutput         = 0.0000,      // Dòng đầu ra
TSlog                 = 0.0000,      // Giá trị log nhiệt
ADC_BitReso           = 0.0000,      // Độ phân giải ADC
daysRunning           = 0.0000,      // Số ngày chạy
Wh                    = 0.0000,      // Watt-giờ
kWh                   = 0.0000,      // Kiliowatt-Hours
MWh                   = 0.0000,      // MWh
loopTime              = 0.0000,      // Thời gian 1 vòng loop
outputDeviation       = 0.0000,      // Sai lệch điện áp ra
buckEfficiency        = 0.0000,      // Hiệu suất buck
floatTemp             = 0.0000,      // Biến float tạm
vOutSystemMin         = 0.0000;      // Điện áp ra nhỏ nhất

unsigned long 
currentErrorMillis    = 0,           
currentButtonMillis   = 0,           
currentSerialMillis   = 0,           
currentRoutineMillis  = 0,          
currentLCDMillis      = 0,      
currentLCDBackLMillis = 0,          
currentMenuSetMillis  = 0,           
prevButtonMillis      = 0,        
prevSerialMillis      = 0,         
prevRoutineMillis     = 0,      
prevErrorMillis       = 0,       
prevWiFiMillis        = 0,     
prevLCDMillis         = 0,           
prevLCDBackLMillis    = 0,         
timeOn                = 0,          
loopTimeStart         = 0,          
loopTimeEnd           = 0,          
secondsElapsed        = 0;           

//   CHƯƠNG TRÌNH CHÍNH  
//   !!!!  Do chương trình dài nên được chia tab để dễ quản lý 
//   Các đoạn code bên dưới chứa toàn bộ tiến trình
//   Firmware chạy song song trên 2 nhân của ESP32.
//   Hàm xTaskCreatePinnedToCore() cho phép sử dụng nhân còn lại để chạy đa nhiệm

//DUAL CORE MODE || CORE 1 (NHÂN CHÍNH)
void coreTwo(void * pvParameters){
 setupWiFi();                                      //TAB#7 
// CORE0: LOOP (DUAL CORE MODE) 
  while(1){
    Wireless_Telemetry();                          //TAB SỐ #7 - (WiFi & Bluetooth)
    
}}
// CORE 1: SETUP DUAL CORE MODE 
void setup() { 
  
  // KHỞI TẠO SERIAL       
  Serial.begin(baudRate);                                   // Thiết lập tốc độ baud cho Serial
  Serial.println("> Serial Initialized");                   // Thông báo khởi động Serial
  
  // GPIO PIN 
  pinMode(backflow_MOSFET,OUTPUT);              // MOSFET chống dòng ngược        
  pinMode(buck_EN,OUTPUT);                      // Chân enable mạch buck
  pinMode(LED,OUTPUT);                          // LED báo trạng thái
  pinMode(FAN,OUTPUT);                          // Quạt làm mát  
  pinMode(TS,INPUT);                            // Cảm biến nhiệt độ
  pinMode(ADC_ALERT,INPUT);                     // Chân cảnh báo ADC
  pinMode(buttonLeft,INPUT);                    // Nút trái
  pinMode(buttonRight,INPUT);                   // Nút phải
  pinMode(buttonBack,INPUT);                    // Nút quay lại
  pinMode(buttonSelect,INPUT);                  // Nút chọn
  
  // KHỞI TẠO PWM 
  ledcSetup(pwmChannel,pwmFrequency,pwmResolution);          // Cấu hình PWM (kênh, tần số, độ phân giải)
  ledcAttachPin(buck_IN, pwmChannel);                        // Gán chân PWM cho mạch buck
  ledcWrite(pwmChannel,PWM);                                 // Ghi PWM ban đầu (0%)
  pwmMax = pow(2,pwmResolution)-1;                           // Giá trị PWM tối đa theo độ phân giải
  pwmMaxLimited = (PWM_MaxDC*pwmMax)/100.000;                // Giới hạn duty cycle an toàn
  
  // KHỞI TẠO ADC 
  ADC_SetGain();                                             // Cấu hình gain & dải đo ADC
  ads.begin();                                               // Khởi động ADC ADS1115 

  // TẮT BUCK BAN ĐẦU                        
  buck_Disable();                                            // Đảm bảo buck tắt khi khởi động


  // KÍCH HOẠT ĐA NHÂN
  xTaskCreatePinnedToCore(coreTwo,"coreTwo",10000,NULL,0,&Core2,0);
  
  // KHỞI TẠO EEPROM 
  EEPROM.begin(512);
  Serial.println("> FLASH MEMORY: STORAGE INITIALIZED");  //Startup message 
  initializeFlashAutoload();                              //Load stored settings from flash memory       
  Serial.println("> FLASH MEMORY: SAVED DATA LOADED");    //Startup message 

  // KHỞI TẠO LCD
  if(enableLCD==1){
    lcd.begin();
    lcd.setBacklight(HIGH);
    lcd.setCursor(0,0);
    lcd.print("MPPT INITIALIZED");
    lcd.setCursor(0,1);
    lcd.print("FIRMWARE ");
    lcd.print(firmwareInfo);    
    delay(1500);
    lcd.clear();
  }

  //SETUP FINISHED
  Serial.println("> MPPT HAS INITIALIZED");                //Startup message

}
//================== CORE1: LOOP (DUAL CORE MODE) ======================//
void loop() {
  Read_Sensors();         //TAB#2 - Sensor data measurement and computation
  Device_Protection();    //TAB#3 - Fault detection algorithm  
  System_Processes();     //TAB#4 - Routine system processes 
  Charging_Algorithm();   //TAB#5 - Battery Charging Algorithm                    
  Onboard_Telemetry();    //TAB#6 - Onboard telemetry (USB & Serial Telemetry)
  LCD_Menu();             //TAB#8 - Low Power Algorithm
}
