#include <Wire.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#define SEND_GREE
#include <IRsend.h>

#define SSD1306_I2C_ADDRESS 0x3C
#define SSD1306_64_48

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


#define ENABLE_LOGGER
#include <SerialOutput.h>

#include <SerialInput.h>
#include <Button.h>
#include <Constants.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "BLYNK_AUTH";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "WIFINAME";
char pass[] = "PASS";

bool focus1 = true;
bool focus2 = false;
short counter = 0;
bool g_fresh_data = false;

#define IR_LED D5  // ESP8266 GPIO pin to use. Recommended: 4 (D2).

volatile float g_temperature = 0;
volatile float g_humidity = 0;
volatile bool  g_isCool = true;

bool LOCK = false;
volatile bool connection_sts = false;

IRsend irsend(IR_LED);

WidgetLED ack_led(V3);

BlynkTimer timer;

#define OLED_RESET 0  // GPIO0
Adafruit_SSD1306 display(OLED_RESET);
Adafruit_BME280 bme; // I2C
#define SCREEN_WIDTH 7 

enum COMMAND
{
    C_ON,
    C_OFF,
    C_COOL_21,
    C_COOL_23,
    C_HOT_25,
    C_HOT_27,
    C_LAST
};


uint16_t AC_COMMANDS[C_LAST][263] =
{
    /*C_ON*/{},
    /*C_OFFFD5D35DF*/    {8024,4238,1844,524,674,1730,642,1726,668,1732,642,1730,640,1728,644,1728,642,1724,1818,524,674,1728,668,1730,642,1730,642,1730,642,1730,1814,524,674,1756,614,1730,1816,550,1822,550,1820,554,670,1704,1840,524,674,1730,642,1756,642,1700,670,1730,642,1730,642,1728,642,1730,642,1730,640,1756,644,1728,642,1728,642,1728,642,1700,670,1700,670,1730,642,1728,670,1730,642,1728,644,1728,642,1728,642,1702,670,1728,642,1730,668,1728,1788,552,1818,552,674,1728,642,1730,642,1728,644,1728,668,1702,670,1698,672,1730,642,1730,642,1728,1814,526,672,1702,694,1730,642,1730,1786,554,672,1730,642,1730,1680,31406,8028,4258,1818,552,674,1728,642,1730,642,1728,642,1730,642,1728,660,1738,642,1702,1816,554,672,1730,642,1730,642,1728,642,1730,666,1732,1786,554,672,1730,642,1730,1806,534,1844,526,1846,526,696,1732,1788,552,672,1700,672,1730,642,1730,642,1700,670,1732,662,1710,668,1700,672,1730,640,1732,640,1730,642,1730,640,1730,660,1682,696,1730,642,1730,640,1702,670,1730,640,1728,644,1730,660,1740,640,1730,642,1730,640,1730,642,1730,1814,528,1844,554,670,1704,668,1730,640,1730,642,1730,640,1730,642,1730,640,1730,666,1732,642,1730,1786,554,672,1730,642,1700,670,1698,1844,528,692,1682,694,1730,1680},
    /*C_COOL_212704FD4F*/{7970,4258,1818,552,672,1732,638,1732,640,1730,640,1700,672,1732,664,1734,640,1730,1788,552,672,1704,668,1730,640,1730,640,1730,662,1738,1786,552,672,1732,640,1702,1842,526,672,1730,1812,528,696,1732,1786,556,670,1732,640,1732,638,1734,640,1732,640,1758,642,1730,640,1730,642,1702,670,1730,640,1758,612,1734,640,1732,664,1734,640,1704,668,1732,640,1730,640,1732,638,1760,608,1736,660,1736,640,1730,640,1732,1786,554,1844,528,670,1702,668,1760,640,1732,638,1732,638,1732,638,1732,640,1732,640,1732,638,1734,662,1736,1784,554,670,1734,1784,556,670,1704,1840,528,672,1758,640,1732,638,1732,1676,30814,7994,4262,1814,554,670,1732,640,1730,638,1732,618,1782,638,1732,638,1734,638,1732,1800,540,668,1706,668,1732,660,1736,638,1734,636,1734,1782,558,668,1734,638,1732,1810,530,692,1736,1784,554,668,1736,1782,556,670,1734,638,1760,606,1738,622,1750,662,1736,640,1732,638,1734,638,1702,694,1708,636,1762,626,1718,662,1732,642,1732,640,1732,636,1736,636,1734,640,1732,618,1780,636,1736,638,1734,662,1678,670,1734,1808,532,1840,530,694,1736,640,1732,636,1706,690,1708,638,1734,636,1734,638,1734,658,1710,666,1734,638,1734,1782,558,670,1732,1808,532,668,1706,1838,534,690,1736,638,1704,668,1734,1676},
    /*C_COOL_2325C9CB97*/{7938,4324,1764,602,622,1780,616,1756,616,1756,592,1780,592,1754,634,1764,614,1784,1736,602,624,1780,592,1780,592,1752,618,1780,608,1762,1788,580,622,1780,592,1752,1784,582,1796,576,1822,550,650,1780,1762,578,622,1780,594,1778,592,1780,592,1780,592,1780,614,1784,592,1780,592,1778,592,1780,592,1778,616,1728,622,1778,610,1762,616,1782,642,1698,622,1754,618,1780,592,1806,562,1782,610,1762,616,1782,592,1780,1736,602,1790,582,622,1780,592,1806,584,1788,592,1778,594,1780,592,1780,592,1780,592,1780,592,1778,616,1782,1738,602,622,1782,614,1728,1784,586,1796,576,648,1756,612,1786,592,1780,1654,30834,7948,4310,1768,602,622,1782,590,1782,590,1782,606,1764,616,1782,592,1780,616,1756,1758,582,624,1780,592,1778,606,1792,592,1780,590,1780,1736,602,622,1782,590,1780,1762,576,1820,578,1770,602,622,1752,1790,578,622,1780,592,1808,562,1782,612,1760,618,1752,618,1780,592,1780,616,1756,592,1780,592,1806,586,1758,618,1782,614,1756,614,1758,616,1756,592,1780,614,1756,608,1790,592,1780,592,1780,590,1750,620,1780,1764,576,1796,574,640,1790,592,1780,592,1780,592,1780,592,1780,616,1756,590,1780,614,1784,592,1750,622,1780,1736,602,622,1782,616,1754,1764,576,1796,602,622,1752,622,1750,620,1780,1630},
    /*C_HOT_25*92B58E67*/{7936,4374,1736,578,672,1758,590,1780,590,1780,616,1756,590,1782,614,1756,592,1806,588,1752,1766,602,622,1782,592,1780,592,1778,592,1780,1762,576,674,1756,590,1780,1736,604,620,1782,590,1782,1762,576,1796,576,648,1780,590,1780,590,1780,590,1780,616,1754,614,1756,592,1778,640,1758,616,1754,616,1756,590,1750,620,1750,628,1772,592,1780,616,1784,616,1754,592,1752,646,1754,592,1778,592,1780,590,1750,642,1784,1760,578,1764,606,624,1780,592,1780,616,1754,614,1758,614,1784,590,1780,592,1780,590,1750,622,1780,616,1782,1736,576,1820,576,1792,578,1768,602,648,1756,616,1754,618,1724,648,1754,1654,31580,7970,4314,1736,602,646,1758,590,1780,616,1728,640,1728,668,1758,614,1756,616,1724,648,1754,1786,552,646,1756,616,1754,616,1782,616,1756,1736,602,622,1780,590,1748,1794,576,648,1756,616,1782,1760,578,1770,602,650,1726,644,1782,562,1782,616,1728,642,1784,616,1724,644,1758,616,1756,590,1780,596,1774,590,1782,614,1782,616,1756,592,1778,592,1782,590,1780,614,1726,644,1784,592,1780,614,1726,620,1780,590,1782,1762,576,1794,576,646,1758,638,1760,616,1756,616,1756,592,1780,590,1780,616,1756,616,1754,616,1754,642,1756,1762,576,1768,604,1794,578,1820,550,646,1758,614,1784,616,1756,588,1782,1654},
    /*C_HOT_27*DCB40247*/{8052,4292,1784,554,672,1732,640,1732,638,1702,670,1732,662,1736,638,1702,670,1704,666,1732,1812,528,670,1734,638,1732,658,1710,668,1704,1812,556,670,1702,670,1704,1840,530,1842,526,692,1738,1784,554,1816,556,668,1734,638,1702,668,1700,670,1732,664,1734,638,1732,638,1704,666,1732,640,1732,638,1758,612,1732,664,1732,640,1732,638,1702,668,1734,640,1702,668,1702,668,1728,670,1734,638,1706,666,1700,670,1732,1810,528,1842,528,670,1732,660,1736,640,1700,670,1702,668,1732,638,1732,638,1732,640,1704,684,1740,638,1704,1812,556,1814,556,670,1704,668,1700,1842,528,696,1734,638,1702,668,1732,1678,30780,8026,4292,1782,556,670,1704,668,1732,640,1732,638,1732,640,1730,664,1734,640,1704,666,1732,1782,556,670,1732,638,1732,640,1702,692,1704,1814,556,672,1732,640,1704,1838,528,1842,528,672,1730,1810,556,1814,556,670,1734,638,1704,666,1734,636,1734,638,1760,638,1732,636,1734,638,1704,666,1734,636,1760,608,1736,638,1732,662,1736,636,1734,638,1732,638,1706,666,1702,692,1736,608,1706,694,1704,666,1704,668,1704,1812,556,1840,530,668,1734,638,1728,670,1732,640,1732,636,1704,668,1704,666,1734,636,1734,638,1734,662,1704,1816,554,1840,530,670,1704,666,1732,1810,530,670,1730,668,1734,636,1706,1702}

};

short printTemperature(float i_data, bool i_focuse);
short printHumidity(float i_data, bool i_focuse, char i_sighn = '%');

void timerEvent()
{
    LOG_MSG("About to read sensors data");

    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    
    float pressure = bme.readPressure();
    float pressureAt = (float)( pressure / 100 ) * 0.000986F;
    auto pressureHpa = (float)( pressure / 100 );

    if ( temperature != NAN && g_temperature != temperature )
    {
        g_temperature = temperature;
        g_fresh_data = true;
    }

    if ( humidity != NAN && g_humidity != humidity )
    {
        g_humidity = humidity;
        g_fresh_data = true;
    }

    LOG_MSG("Reading sensors data : T=" << g_temperature << "C H=" << g_humidity << "% P" << pressureHpa << "hPa");
}

void sendIRSignal(unsigned long i_data)
{
    LOG_MSG("Sending IR signal " << i_data << " " << (const void*)i_data);
    irsend.sendGree(i_data);
}

void sendIRSignal(COMMAND i_command = C_COOL_23)
{
    //uint16_t rawData[263] = { 7938, 4324, 1764, 602, 622, 1780, 616, 1756, 616, 1756, 592, 1780, 592, 1754, 634, 1764, 614, 1784, 1736, 602, 624, 1780, 592, 1780, 592, 1752, 618, 1780, 608, 1762, 1788, 580, 622, 1780, 592, 1752, 1784, 582, 1796, 576, 1822, 550, 650, 1780, 1762, 578, 622, 1780, 594, 1778, 592, 1780, 592, 1780, 592, 1780, 614, 1784, 592, 1780, 592, 1778, 592, 1780, 592, 1778, 616, 1728, 622, 1778, 610, 1762, 616, 1782, 642, 1698, 622, 1754, 618, 1780, 592, 1806, 562, 1782, 610, 1762, 616, 1782, 592, 1780, 1736, 602, 1790, 582, 622, 1780, 592, 1806, 584, 1788, 592, 1778, 594, 1780, 592, 1780, 592, 1780, 592, 1780, 592, 1778, 616, 1782, 1738, 602, 622, 1782, 614, 1728, 1784, 586, 1796, 576, 648, 1756, 612, 1786, 592, 1780, 1654, 30834, 7948, 4310, 1768, 602, 622, 1782, 590, 1782, 590, 1782, 606, 1764, 616, 1782, 592, 1780, 616, 1756, 1758, 582, 624, 1780, 592, 1778, 606, 1792, 592, 1780, 590, 1780, 1736, 602, 622, 1782, 590, 1780, 1762, 576, 1820, 578, 1770, 602, 622, 1752, 1790, 578, 622, 1780, 592, 1808, 562, 1782, 612, 1760, 618, 1752, 618, 1780, 592, 1780, 616, 1756, 592, 1780, 592, 1806, 586, 1758, 618, 1782, 614, 1756, 614, 1758, 616, 1756, 592, 1780, 614, 1756, 608, 1790, 592, 1780, 592, 1780, 590, 1750, 620, 1780, 1764, 576, 1796, 574, 640, 1790, 592, 1780, 592, 1780, 592, 1780, 592, 1780, 616, 1756, 590, 1780, 614, 1784, 592, 1750, 622, 1780, 1736, 602, 622, 1782, 616, 1754, 1764, 576, 1796, 602, 622, 1752, 622, 1750, 620, 1780, 1630 };  // UNKNOWN 25C9CB97    
    digitalWrite(BUILTIN_LED, LED_ON);
    LOG_MSG("Sending IR signal " << i_command << ", size " << (short)(sizeof(AC_COMMANDS[i_command]) / sizeof(AC_COMMANDS[i_command][0])));
    irsend.sendRaw(AC_COMMANDS[i_command], (uint16_t)(sizeof(AC_COMMANDS[i_command]) / sizeof(AC_COMMANDS[i_command][0])), 38);  // Send a raw data capture at 38kHz. 
    digitalWrite(BUILTIN_LED, LED_OFF);
}
// This function will be called every time Slider Widget
// in Blynk app writes values to the Virtual Pin V1
BLYNK_WRITE(V0)
{
    int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

    LOG_MSG("About to send COOL 21 IR signal");

    sendIRSignal( (g_isCool) ? C_COOL_21 : C_HOT_27 );

    ack_led.on();//sending to Blynk
}

BLYNK_READ(V1) //Blynk app has something on V5
{
    LOG_MSG("Writing to Blynk service");

    digitalWrite(BUILTIN_LED, LED_ON);
    if (g_temperature > 0)
    {
        Blynk.virtualWrite(V1, g_temperature); //sending to Blynk
    }

    if (g_humidity > 0)
    {
        Blynk.virtualWrite(V2, g_humidity); //sending to Blynk
    }

    digitalWrite(BUILTIN_LED, LED_OFF);
}

BLYNK_WRITE(V5)
{
    int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

    LOG_MSG("About to send COOL 23 IR signal");

    sendIRSignal( (g_isCool ) ? C_COOL_23 : C_HOT_25 );

    ack_led.on();//sending to Blynk
}

BLYNK_WRITE(V4)
{
    int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

    LOG_MSG("About to send OFF IR signal");

    if ( 0 != ack_led.getValue() ) //turned on
    {
        sendIRSignal(C_OFF);
    }

    ack_led.off(); //sending to Blynk
}

BLYNK_WRITE(V6)
{     
    g_isCool = ( 0 == param.asInt() ); // assigning incoming value from pin V1 to a variable

    LOG_MSG("New mode is " << ( (g_isCool) ? "cool" : "hot" ));
}

void setup()
{
    // Debug console
    Serial.begin(19200);

    irsend.begin();

    Blynk.begin( auth, ssid, pass );
    // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)

    display.begin( SSD1306_SWITCHCAPVCC, 0x3C );  // initialize with the I2C addr 0x3C (for the 64x48)
    display.setTextWrap( false );
   
    bme.begin( 0x76 );
    
    ack_led.off();
    timer.setInterval(1000L, timerEvent);

    //Button
    pinMode( D8, INPUT );

    pinMode(BUILTIN_LED, OUTPUT);
    digitalWrite( BUILTIN_LED, LED_OFF);
    LOG_MSG("setup completed");
}

InputReader_t input;

struct TextSize
{
    short m_X;
    short m_Y;
    TextSize(short i_x, short i_y) : m_X(i_x), m_Y(i_y){}
};

TextSize textSize[] = { TextSize(0, 0), TextSize(5, 7), TextSize(10, 14), TextSize(15, 21), TextSize(20, 28) };
/*
Button(uint8_t i_pin,
SingleClickHandler i_sc_handler = &Button::onSingleClick,
DoubleClickHandler i_dc_handler = &Button::onDoubleClick
LongClickHandler   i_lc_handler = &Button::onLongClick,
) :
m_pin(i_pin),(
*/
Button_t button( D8, []() { sendIRSignal(C_COOL_21); ack_led.on(); },
    [](){ focus2 = !focus2; focus1 = !focus1; ( 0 == ++counter % 3 ) ? display.dim(true) : display.dim(false); g_fresh_data = true; },
    [](){ if (ack_led.getValue() == 0) { sendIRSignal(); ack_led.on(); } else { sendIRSignal(C_OFF); ack_led.off(); } }
    );

short printTemperature(float i_data, bool i_focuse)
{
    short digitsSize = 1, sighnSize = 1;
    if (i_focuse)
    {
        digitsSize = 3;
        sighnSize = 2;
    }

    display.setTextSize(digitsSize);
    if (i_focuse)
    {
        display.print((short)i_data);
    }
    else
    {
        display.printf("%.2f", i_data);
    }
    display.setTextSize(sighnSize);
    display.setCursor(display.getCursorX() + 3, display.getCursorY() + textSize[digitsSize].m_Y - textSize[sighnSize].m_Y);
    display.print("C");

    if (i_focuse)
    {
        display.setTextSize(1);
        display.setCursor( display.getCursorX() - textSize[sighnSize].m_X - 5, display.getCursorY() - textSize[1].m_Y - 1 );

        display.print("o");
    }
    else
    {
        display.drawPixel(display.getCursorX() - textSize[1].m_X - 4, display.getCursorY(), WHITE);
    }

    display.setTextSize(1);
    display.setCursor(display.width() - textSize[1].m_X, 1);

    if ( g_isCool )
    {
        display.drawRect(display.width() - textSize[1].m_X - 2, 0, textSize[1].m_X + 2, textSize[1].m_Y + 2, WHITE); 
    }
    else
    {
        display.fillRect(display.width() - textSize[1].m_X - 2, 0, textSize[1].m_X + 2, textSize[1].m_Y + 2, WHITE);
    }
    
    return textSize[digitsSize].m_Y;
}

short printHumidity( float i_data, bool i_focuse, char i_sighn )
{
    short digitsSize = 1, sighnSize = 1;
    
    if (i_focuse)
    {
        digitsSize = 2;
        sighnSize = 1;
    }

    display.setTextSize(digitsSize);
    display.printf("%.1f", i_data);

    display.setTextSize(sighnSize);
    display.setCursor(display.getCursorX(), display.getCursorY() + textSize[digitsSize].m_Y - textSize[sighnSize].m_Y);
    display.print(i_sighn);

    return textSize[digitsSize].m_Y;
}

void loop()
{
    Blynk.run();
    timer.run();
    button.run();

    if (!connection_sts && (WiFi.status() == WL_CONNECTED && Blynk.connected()))
    {
        connection_sts = true;
        g_fresh_data = true;
        String connected = "Paired ";
        connected += WiFi.localIP().toString();
        LOG_MSG("Connected to remote service IP: " << connected );
    }
    else if (connection_sts && !(WiFi.status() == WL_CONNECTED && Blynk.connected()))
    {
        connection_sts = false;
        g_fresh_data = true;
        LOG_MSG("No connection! WiFi.status() " << WiFi.status() << " Blynk.connected() " << Blynk.connected() );
    }

    if ( g_fresh_data )
    {
        g_fresh_data = false;
        display.setTextColor(WHITE);
        display.setCursor(0, 0);
        display.clearDisplay();

        short nextLine = printTemperature(g_temperature, focus1 );
        display.setCursor( 0, ++nextLine );  
        if ( nextLine + textSize[1].m_Y > display.height() )
            goto NEXT;

        nextLine += printHumidity(g_humidity, focus2 );
        display.setCursor( 0, ++nextLine );
        if (nextLine + textSize[1].m_Y > display.height())
            goto NEXT;
        
        display.setTextSize(1);

        if ( connection_sts )
        {
           display.println();
           display.println("Connected");
        }
        else
        {
          display.println();
          display.println("Not Connected");
        }


      

       NEXT:
         display.display();
    }

    arduino::utils::Result<unsigned long> result = input.read<unsigned long>();

    if (result.m_ok)
    {
        if (result.m_result == 0)
        {
            sendIRSignal();
        }
        else
        {
            sendIRSignal(result.m_result);
        }
    }
}
