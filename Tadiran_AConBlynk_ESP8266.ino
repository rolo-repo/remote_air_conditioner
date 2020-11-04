#include "buildinfo.h"
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

#include <Wire.h>
#include "EEPROM_Adapter.h"
#include <Adafruit_BME280.h>
//#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <BlynkSimpleEsp8266.h>
#include <ESP8266SSDP.h>   
#define DECODE_AC
#define DECODE_GREE

#include <IRrecv.h>
#include <IRutils.h>
#include <IRsend.h>

#define ENABLE_LOGGER
#include "MulticastOutput.h"
#include "Constants.h"
#include "Pair.h"
#include "resource.h"
#include "Led.h"
#include <Button.h>
#include <TimeManager.h>
#include "PlaceHolder.h"

#include "OTA.h"
#include <TimeLib.h>  


#include "iostream"
#include "Pair.h"
#include "SetupBlynkAndWifi.h"

using namespace arduino::utils;
ESP8266WebServer  server(80);

const char* const HTML_open = "<!DOCTYPE html><html><head></head>";
const char* const HTML_body_open = "<body>";
const char* const HTML_body_close = "</body>";
const char* const HTML_new_line = "<p>";
const char* const HTML_close = "</html>";

String ssid;
String ssid_password;
String blynk_auth;
String blynk_domain;
uint16 blynk_port;

uint16 data_offset = 0;
uint16 counter_indx = 0;
volatile bool g_isCool = true;

#define CAPTURE_BUFFER_SIZE 1024
#define TIMEOUT 50U 

#define SCL_PIN D1
#define SDA_PIN D2
#define IR_LED D3
#define STATUS_LED D4 //BULDIN
#define BLUE_LED D5
#define GREEN_LED D0
#define RED_LED D6
#define RECV_PIN D7
#define BUTTON D8


#ifdef OLD_VERSION
#define IR_LED D5
#define STATUS_LED D3
#define BLUE_LED D4
#endif

constexpr char* hostname = "acRemote_";

WidgetLED ackLed(V3);

Led stsRedLed(STATUS_LED);
RGBLed rgbLed(GREEN_LED, BLUE_LED, RED_LED, Led::Brightness::_20, true);

arduino::utils::Timer idleTimer("[IDLE]");
arduino::utils::Timer actionTimer("[action]");

IRrecv irrecv(RECV_PIN, CAPTURE_BUFFER_SIZE, TIMEOUT, true);
IRsend irsend(IR_LED);

enum IR_CODES { IR_COLD_LOW = 0, IR_COLD_HIGH, IR_HOT_LOW, IR_HOT_HIGH, IR_OFF, IR_LAST };

const char * const IR_CODES_NAMES[IR_LAST] = { "COLD_LOW","COLD_HIGH","HOT_LOW","HOT_HIGH","OFF" };

BufferAndSize_t< uint16, uint16 > ir_coded[IR_LAST];

uint16_t AC_COMMANDS[IR_LAST][263] =
{
	//C_ON{},
	//C_COOL_2325C9CB97
	{7938,4324,1764,602,622,1780,616,1756,616,1756,592,1780,592,1754,634,1764,614,1784,1736,602,624,1780,592,1780,592,1752,618,1780,608,1762,1788,580,622,1780,592,1752,1784,582,1796,576,1822,550,650,1780,1762,578,622,1780,594,1778,592,1780,592,1780,592,1780,614,1784,592,1780,592,1778,592,1780,592,1778,616,1728,622,1778,610,1762,616,1782,642,1698,622,1754,618,1780,592,1806,562,1782,610,1762,616,1782,592,1780,1736,602,1790,582,622,1780,592,1806,584,1788,592,1778,594,1780,592,1780,592,1780,592,1780,592,1778,616,1782,1738,602,622,1782,614,1728,1784,586,1796,576,648,1756,612,1786,592,1780,1654,30834,7948,4310,1768,602,622,1782,590,1782,590,1782,606,1764,616,1782,592,1780,616,1756,1758,582,624,1780,592,1778,606,1792,592,1780,590,1780,1736,602,622,1782,590,1780,1762,576,1820,578,1770,602,622,1752,1790,578,622,1780,592,1808,562,1782,612,1760,618,1752,618,1780,592,1780,616,1756,592,1780,592,1806,586,1758,618,1782,614,1756,614,1758,616,1756,592,1780,614,1756,608,1790,592,1780,592,1780,590,1750,620,1780,1764,576,1796,574,640,1790,592,1780,592,1780,592,1780,592,1780,616,1756,590,1780,614,1784,592,1750,622,1780,1736,602,622,1782,616,1754,1764,576,1796,602,622,1752,622,1750,620,1780,1630},
	//C_COOL_212704FD4F
	{7970,4258,1818,552,672,1732,638,1732,640,1730,640,1700,672,1732,664,1734,640,1730,1788,552,672,1704,668,1730,640,1730,640,1730,662,1738,1786,552,672,1732,640,1702,1842,526,672,1730,1812,528,696,1732,1786,556,670,1732,640,1732,638,1734,640,1732,640,1758,642,1730,640,1730,642,1702,670,1730,640,1758,612,1734,640,1732,664,1734,640,1704,668,1732,640,1730,640,1732,638,1760,608,1736,660,1736,640,1730,640,1732,1786,554,1844,528,670,1702,668,1760,640,1732,638,1732,638,1732,638,1732,640,1732,640,1732,638,1734,662,1736,1784,554,670,1734,1784,556,670,1704,1840,528,672,1758,640,1732,638,1732,1676,30814,7994,4262,1814,554,670,1732,640,1730,638,1732,618,1782,638,1732,638,1734,638,1732,1800,540,668,1706,668,1732,660,1736,638,1734,636,1734,1782,558,668,1734,638,1732,1810,530,692,1736,1784,554,668,1736,1782,556,670,1734,638,1760,606,1738,622,1750,662,1736,640,1732,638,1734,638,1702,694,1708,636,1762,626,1718,662,1732,642,1732,640,1732,636,1736,636,1734,640,1732,618,1780,636,1736,638,1734,662,1678,670,1734,1808,532,1840,530,694,1736,640,1732,636,1706,690,1708,638,1734,636,1734,638,1734,658,1710,666,1734,638,1734,1782,558,670,1732,1808,532,668,1706,1838,534,690,1736,638,1704,668,1734,1676},
	//C_HOT_25*92B58E67
	{7936,4374,1736,578,672,1758,590,1780,590,1780,616,1756,590,1782,614,1756,592,1806,588,1752,1766,602,622,1782,592,1780,592,1778,592,1780,1762,576,674,1756,590,1780,1736,604,620,1782,590,1782,1762,576,1796,576,648,1780,590,1780,590,1780,590,1780,616,1754,614,1756,592,1778,640,1758,616,1754,616,1756,590,1750,620,1750,628,1772,592,1780,616,1784,616,1754,592,1752,646,1754,592,1778,592,1780,590,1750,642,1784,1760,578,1764,606,624,1780,592,1780,616,1754,614,1758,614,1784,590,1780,592,1780,590,1750,622,1780,616,1782,1736,576,1820,576,1792,578,1768,602,648,1756,616,1754,618,1724,648,1754,1654,31580,7970,4314,1736,602,646,1758,590,1780,616,1728,640,1728,668,1758,614,1756,616,1724,648,1754,1786,552,646,1756,616,1754,616,1782,616,1756,1736,602,622,1780,590,1748,1794,576,648,1756,616,1782,1760,578,1770,602,650,1726,644,1782,562,1782,616,1728,642,1784,616,1724,644,1758,616,1756,590,1780,596,1774,590,1782,614,1782,616,1756,592,1778,592,1782,590,1780,614,1726,644,1784,592,1780,614,1726,620,1780,590,1782,1762,576,1794,576,646,1758,638,1760,616,1756,616,1756,592,1780,590,1780,616,1756,616,1754,616,1754,642,1756,1762,576,1768,604,1794,578,1820,550,646,1758,614,1784,616,1756,588,1782,1654},
	//C_HOT_27*DCB40247
	{8052,4292,1784,554,672,1732,640,1732,638,1702,670,1732,662,1736,638,1702,670,1704,666,1732,1812,528,670,1734,638,1732,658,1710,668,1704,1812,556,670,1702,670,1704,1840,530,1842,526,692,1738,1784,554,1816,556,668,1734,638,1702,668,1700,670,1732,664,1734,638,1732,638,1704,666,1732,640,1732,638,1758,612,1732,664,1732,640,1732,638,1702,668,1734,640,1702,668,1702,668,1728,670,1734,638,1706,666,1700,670,1732,1810,528,1842,528,670,1732,660,1736,640,1700,670,1702,668,1732,638,1732,638,1732,640,1704,684,1740,638,1704,1812,556,1814,556,670,1704,668,1700,1842,528,696,1734,638,1702,668,1732,1678,30780,8026,4292,1782,556,670,1704,668,1732,640,1732,638,1732,640,1730,664,1734,640,1704,666,1732,1782,556,670,1732,638,1732,640,1702,692,1704,1814,556,672,1732,640,1704,1838,528,1842,528,672,1730,1810,556,1814,556,670,1734,638,1704,666,1734,636,1734,638,1760,638,1732,636,1734,638,1704,666,1734,636,1760,608,1736,638,1732,662,1736,636,1734,638,1732,638,1706,666,1702,692,1736,608,1706,694,1704,666,1704,668,1704,1812,556,1840,530,668,1734,638,1728,670,1732,640,1732,636,1704,668,1704,666,1734,636,1734,638,1734,662,1704,1816,554,1840,530,670,1704,666,1732,1810,530,670,1730,668,1734,636,1706,1702},
	//C_OFFFD5D35DF
	{8024,4238,1844,524,674,1730,642,1726,668,1732,642,1730,640,1728,644,1728,642,1724,1818,524,674,1728,668,1730,642,1730,642,1730,642,1730,1814,524,674,1756,614,1730,1816,550,1822,550,1820,554,670,1704,1840,524,674,1730,642,1756,642,1700,670,1730,642,1730,642,1728,642,1730,642,1730,640,1756,644,1728,642,1728,642,1728,642,1700,670,1700,670,1730,642,1728,670,1730,642,1728,644,1728,642,1728,642,1702,670,1728,642,1730,668,1728,1788,552,1818,552,674,1728,642,1730,642,1728,644,1728,668,1702,670,1698,672,1730,642,1730,642,1728,1814,526,672,1702,694,1730,642,1730,1786,554,672,1730,642,1730,1680,31406,8028,4258,1818,552,674,1728,642,1730,642,1728,642,1730,642,1728,660,1738,642,1702,1816,554,672,1730,642,1730,642,1728,642,1730,666,1732,1786,554,672,1730,642,1730,1806,534,1844,526,1846,526,696,1732,1788,552,672,1700,672,1730,642,1730,642,1700,670,1732,662,1710,668,1700,672,1730,640,1732,640,1730,642,1730,640,1730,660,1682,696,1730,642,1730,640,1702,670,1730,640,1728,644,1730,660,1740,640,1730,642,1730,640,1730,642,1730,1814,528,1844,554,670,1704,668,1730,640,1730,642,1730,640,1730,642,1730,640,1730,666,1732,642,1730,1786,554,672,1730,642,1700,670,1698,1844,528,692,1682,694,1730,1680}
};

WiFiEventHandler onGotIPhandler;
WiFiEventHandler onDisconnect;
struct
{
	bool enabled;
	Pair<float, float> data[12];
	Adafruit_BME280 bme;
	Adafruit_BME280& operator*() { return bme; }
} bme;

#define SCREEN_WIDTH 7 
struct
{
	bool enabled;
	Adafruit_SSD1306 display;
	Adafruit_SSD1306& operator*() { return display; }
} display;

auto bmeReadTemperature()
{
	LOG_MSG("About to read sensors data - temperature");

	float temperature = (bme.enabled) ? (*bme).readTemperature() : NAN;

	//if ( temperature > 25 ) 
		temperature -= 5 ;

	return   ((temperature != NAN) ? temperature : 0.0F);
}

auto bmeReadHumidity()
{
	LOG_MSG("About to read sensors data - humidity");

	float humidity = (bme.enabled) ? (*bme).readHumidity() : NAN;

	return   ((humidity != NAN) ? humidity : 0.0F);
}

Pair< float, float >  bmeReadPressure()
{
	LOG_MSG("About to read sensors data - preasure");
	float pressure = (bme.enabled) ? (*bme).readPressure() : NAN;
	float pressureAt = (float)(pressure / 100) * 0.000986F;
	float pressureHpa = (float)(pressure / 100);

	return make_Pair(pressureAt, pressureHpa);
}

void irRecieveAndSave(BufferAndSize_t<uint16, uint16 > &buff)
{
	using namespace arduino::utils;

	bool recv_sts = false;
	decode_results results;

	LOG_MSG("Waiting for IR signal");
	irrecv.enableIRIn();
	yield();
	unsigned long ctime = millis();
	while (!irrecv.decode(&results) || results.rawlen < 50) {
		yield();
		if (millis() - ctime > 30000)
			return;
	}
	irrecv.disableIRIn();
	rgbLed.rapid_blynk(1000);

	LOG_MSG("Got IR signal Size " << getCorrectedRawLength(&results));

	//save the buffer
	buff = BufferAndSize_t<uint16, uint16 >(new uint16[results.rawlen - 1], results.rawlen - 1);

	//update the buffer
	for (uint16 i = 1; i < results.rawlen; i++) {
		buff.m_pData[i - 1] = results.rawbuf[i] * RAWTICK;
		ESP.wdtFeed();
		LOG_MSG(buff.m_pData[i - 1]);
	}
}

void irSend(const BufferAndSize_t<uint16, uint16 > &i_buff)
{
	LOG_MSG("Sending IR");
	rgbLed.turn_off();
	rgbLed.turn_on((g_isCool) ? RGBLed::LedType::Blue : RGBLed::LedType::Red);
	irsend.sendRaw(i_buff.m_pData, i_buff.m_size, 38);
	rgbLed.turn_off();
	//rgbLed.rapid_blynk(300);
}

class IRConfiguration_RequestHander : public RequestHandler
{
public:

	IRConfiguration_RequestHander(const char* i_action) : m_uri(i_action) {
	}

	bool canHandle(HTTPMethod method, String uri) {
		LOG_MSG("Request : " << uri << " , can handle : " << m_uri);

		if (method == HTTP_GET && uri.startsWith('/' + m_uri))
			return true;
		else
			return false;
	}

	bool handle(ESP8266WebServer& server, HTTPMethod requestMethod, String requestUri) {
		LOG_MSG("Request : " << requestUri << " " << requestMethod);
		//Read from IR
		//Store to EEPROM , last index
		using namespace arduino::utils;
		if (server.args() == 0)
		{
			PlaceHolder a1("cold_high_color", (ir_coded[IR_COLD_HIGH].m_size > 0) ? "green" : "red");
			PlaceHolder a2("cold_low_color", (ir_coded[IR_COLD_LOW].m_size > 0) ? "green" : "red");
			PlaceHolder a3("hot_high_color", (ir_coded[IR_HOT_HIGH].m_size > 0) ? "green" : "red");
			PlaceHolder a4("hot_low_color", (ir_coded[IR_HOT_LOW].m_size > 0) ? "green" : "red");
			PlaceHolder a5("cold_high_disabled", "");
			PlaceHolder a6("hot_high_disabled", "");
			PlaceHolder a7("cold_low_disabled", "");
			PlaceHolder a8("hot_low_disabled", "");
			PlaceHolder a9("off_disabled", "");
			PlaceHolder a10("action", m_uri);

			server.send(200, "text/html",
				PlaceHolder::tr(PlaceHolder::makeList(VA_LIST(&a1, &a2, &a3, &a4, &a5, &a6, &a7, &a8, &a9, &a10)), FPSTR(configure_ir_html)));
		}
		else
		{
			LOG_MSG("Going to read IR sensor " << server.argName(0));

			rgbLed.turn_on(RGBLed::LedType::Blue);

			char id = 0;

			if (server.argName(0).equals("cold_low")) { id = IR_COLD_LOW; }
			if (server.argName(0).equals("cold_high")) { id = IR_COLD_HIGH; }
			if (server.argName(0).equals("hot_low")) { id = IR_HOT_LOW; }
			if (server.argName(0).equals("hot_high")) { id = IR_HOT_HIGH; }
			if (server.argName(0).equals("off")) { id = IR_OFF; }

			LOG_MSG("Going to listen for ir data");
			//	String resp(response_html);

			PlaceHolder p("data", String("send a command to for ") + server.argName(0));
			//	resp.replace("${data}", String("send a command to for ") + server.argName(0));
			server.send(200, "text/html", PlaceHolder::tr(&p, FPSTR(response_html)));

			irRecieveAndSave(ir_coded[id]);

			rgbLed.rapid_blynk(300);

			LOG_MSG("Store " << id << " size " << ir_coded[id].m_size);

			uint16 idx = data_offset;
			for (uint16 i = 0; i < IR_LAST; i++)
			{
				idx = EEPROM_Adapter_t::save(ir_coded[i], idx);
			}

			server.send(200, "text/html", success_html);
		}

		return true;
	}

private:
	String m_uri;
};

class IR_RequestHander : public RequestHandler
{
public:

	IR_RequestHander(const char* i_action) : m_uri(i_action) {
	}

	bool canHandle(HTTPMethod method, String uri) {
		LOG_MSG("Request : " << uri << " , can handle : " << m_uri);

		if (method == HTTP_GET && uri.startsWith('/' + m_uri))
			return true;
		else
			return false;
	}

	bool handle(ESP8266WebServer& server, HTTPMethod requestMethod, String requestUri) {
		LOG_MSG("Request : " << requestUri << " " << requestMethod);
		using namespace arduino::utils;
		if (server.args() == 0)
		{
			PlaceHolder a1("cold_high_color", (ir_coded[IR_COLD_HIGH].m_size > 0) ? "green" : "red");
			PlaceHolder a2("cold_low_color", (ir_coded[IR_COLD_LOW].m_size > 0) ? "green" : "red");
			PlaceHolder a3("hot_high_color", (ir_coded[IR_HOT_HIGH].m_size > 0) ? "green" : "red");
			PlaceHolder a4("hot_low_color", (ir_coded[IR_HOT_LOW].m_size > 0) ? "green" : "red");
			PlaceHolder a5("cold_high_disabled", (ir_coded[IR_COLD_HIGH].m_size > 0) ? "" : "disabled");
			PlaceHolder a6("hot_high_disabled", (ir_coded[IR_HOT_HIGH].m_size > 0) ? "" : "disabled");
			PlaceHolder a7("cold_low_disabled", (ir_coded[IR_COLD_LOW].m_size > 0) ? "" : "disabled");
			PlaceHolder a8("hot_low_disabled", (ir_coded[IR_HOT_LOW].m_size > 0) ? "" : "disabled");
			PlaceHolder a9("off_disabled", (ir_coded[IR_OFF].m_size > 0) ? "" : "disabled");
			PlaceHolder a10("action", m_uri);

			server.send(200, "text/html", PlaceHolder::tr(PlaceHolder::makeList(VA_LIST(&a1, &a2, &a3, &a4, &a5, &a6, &a7, &a8, &a9, &a10)), FPSTR(configure_ir_html)));
		}
		else
		{
			LOG_MSG("Going to send IR sensor " << server.argName(0));
			stsRedLed.blynk();

			if (server.argName(0).equals("cold_low")) { irSend(ir_coded[IR_COLD_LOW]); }
			if (server.argName(0).equals("cold_high")) { irSend(ir_coded[IR_COLD_HIGH]); }
			if (server.argName(0).equals("hot_low")) { irSend(ir_coded[IR_HOT_LOW]); }
			if (server.argName(0).equals("hot_high")) { irSend(ir_coded[IR_HOT_HIGH]); }
			if (server.argName(0).equals("off")) { irSend(ir_coded[IR_OFF]); }

			server.send(200, "text/html", success_html);
			stsRedLed.blynk();
		}

		return true;
	}

private:
	String m_uri;
};

class LandingPage_RequestHandler : public RequestHandler
{
private:
	struct TimerTask
	{
		uint8 id = 0;
		time_t date_time = 0;
		uint8 code = 0;
		bool recuring = false;
		bool enb = false;

		void schedule()
		{
			if (this->enb)
			{
				if (!recuring)
					actionTimer.addTask(date_time, [this](long&) { irSend(ir_coded[this->code]); this->enb = false; });
				else
					actionTimer.addRecuringTask(date_time, arduino::utils::Timer::DAY, [this](long&) { irSend(ir_coded[this->code]); this->date_time += arduino::utils::Timer::DAY; });
			}
		}

		String toJson()
		{
			String data;
			if (this->enb) {
				data += '{';
				data += "\"id\":";
				data += id;
				data += ',';
				data += "\"date\":\"";
				data += TIME.getDateStrYYYY_MM_DD(date_time);
				data += "\",";
				data += "\"time\":\"";
				data += TIME.getTimeStr(date_time);
				data += "\",";
				data += "\"code\":";
				data += code;
				data += ',';
				data += "\"recuring\": \"";
				data += (recuring) ? "true" : "false";
				data += "\"}";
			}

			return data;
		}
	} tt[10];
public:
	bool canHandle(HTTPMethod method, String uri) override {
		LOG_MSG("Request : " << uri << " method " << method << " can handle : " << "dash or fput");

		if (method == HTTP_GET && (uri.startsWith("/dash") || uri.startsWith("/fput") || uri.startsWith("/schedule")))
			return true;
		else
			return false;
	}

	bool handle(ESP8266WebServer& server, HTTPMethod requestMethod, String requestUri) override {

		using namespace arduino::utils;
		LOG_MSG("Request : " << requestUri << " " << requestMethod);

		if (requestUri.startsWith("/dash"))
		{
			PlaceHolder p("t", "");
			PlaceHolder v("version", TIME.getTimeDateStr(BUILD_ID));

			server.setContentLength(CONTENT_LENGTH_UNKNOWN);
			server.send(200, "text/html", " ");

			BufferAndSize_t<char, long> buff(new char[1024], 1024, true);

			for (uint16 i = 0; i < sizeof(dashboard_html); i += 1023) {

				short size = ((sizeof(dashboard_html) - i > 1023) ? 1023 : sizeof(dashboard_html) - i);
				memcpy_P(*buff, dashboard_html + i, size);
				buff[size] = 0x0;//null terminator
				//the mechanism of translate will not work in case the string will be slitted exactly by placeholder
				server.sendContent(PlaceHolder::tr(PlaceHolder::makeList(VA_LIST(&p, &v)), *buff));
			}

			return true;
		}

		if (requestUri.startsWith("/schedule"))
		{
			String jsonResp;
			jsonResp += '[';

			for (uint8 i = 0; i < SIZE_OF_ARR(tt); i++) {
				if (tt[i].enb) {
					jsonResp += tt[i].toJson();
					jsonResp += ',';
				}
			}
			jsonResp.remove(jsonResp.lastIndexOf(','));
			jsonResp += ']';

			server.send(200, "text/html", jsonResp);
			return true;
		}

		String date, temp, action, time;

		static const char* date_p = "d_";
		static const char* action_p = "a_";
		static const char* time_p = "t_";
		static const char* highlow_p = "temp_";

		actionTimer = arduino::utils::Timer("[action]");

		for (uint8 i = 0; i < SIZE_OF_ARR(tt); i++) {
			tt[i].enb = false;
		}

		uint8 tt_index = 0;

		for (uint8 i = 0; i < server.args(); i++)
		{
			if (server.argName(i).startsWith(date_p)) {
				date = server.arg(i); continue;
			}
			if (server.argName(i).startsWith(time_p)) {
				time = server.arg(i); continue;
			}
			if (server.argName(i).startsWith(highlow_p)) {
				temp = server.arg(i); continue;
			}
			if (server.argName(i).startsWith(action_p))
			{
				if (time.isEmpty() || temp.isEmpty())
				{
					arduino::utils::PlaceHolder p("data", "error wrong format of input , expected date,time,temperature,action ");
					server.send(200, "text/html", arduino::utils::PlaceHolder::tr(&p, FPSTR(response_html)));

					LOG_MSG("Error time is empty " << time.isEmpty() << " temp is empty " << temp.isEmpty());
					return true;
				}

				action = server.arg(i);
				int32 hour, min, yyyy, mm, dd;

				sscanf(time.c_str(), "%d:%d", &hour, &min);

				time_t exeTime = 0;

				if (!date.isEmpty())
				{
					sscanf(date.c_str(), "%d-%d-%d", &yyyy, &mm, &dd);

					tmElements_t tm;

					tm.Hour = hour;
					tm.Minute = min;
					tm.Second = 0;
					tm.Year = yyyy - 1970;
					tm.Day = dd;
					tm.Month = mm;

					exeTime = ::makeTime(tm);

					if (exeTime < TIME.getEpochTime()) {
						LOG_MSG("scheduled time " << TIME.getShortTimeDateStr(exeTime) << " is less that current time " << TIME.getShortTimeDateStr());
						continue;
					}
					tt[tt_index].recuring = false;
				}
				else//recurring action
				{
					exeTime = TIME.getEpochDate() + hour * arduino::utils::Timer::HOUR + min * arduino::utils::Timer::MINUTE;

					if (exeTime < TIME.getEpochTime())
						exeTime += arduino::utils::Timer::DAY;

					tt[tt_index].recuring = true;
				}

				tt[tt_index].code = (action.equals("on")) ? temp.toInt() : IR_OFF;
				tt[tt_index].date_time = exeTime;
				tt[tt_index].enb = true;
				tt[tt_index].id = tt_index++;

				action.clear(); temp.clear(); time.clear(); date.clear();
			}

			ESP.wdtFeed();
		}

		for (uint8 i = 0; i < SIZE_OF_ARR(tt); i++) {
			if (tt[i].enb) {
				LOG_MSG(tt[i].toJson());
				tt[i].schedule();
			}
		}

		PlaceHolder p("data", "schedule updated with " + String(tt_index) + " entities");
		server.send(200, "text/html", PlaceHolder::tr(&p, FPSTR(response_html)));
		return true;
	}
};

void notFound() {
	String message = "Not Found\n\n";
	message += "URI: ";
	message += server.uri();
	message += "\nMethod: ";
	message += (server.method() == HTTP_GET) ? "GET" : "POST";
	message += "\nArguments: ";
	message += server.args();
	message += "\n";
	for (uint8 i = 0; i < server.args(); i++) {
		message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
	}
	server.send(404, "text/plain", message);
}

void setup() {

	LOG_MSG_BEGIN(115200);
	Serial.println("Starting");
	rgbLed.turn_off();
	rgbLed.turn_on(RGBLed::LedType::Blue);

	stsRedLed.turn_off();
	WiFi.persistent(false);
	WiFi.disconnect();

	Wire.begin(SDA_PIN, SCL_PIN);
	Wire.beginTransmission(0x76);

	if (0 == Wire.endTransmission())
	{
		LOG_MSG("BME device found on channel " << 0x76);
		bme.enabled = (*bme).begin(0x76);
	}

	Wire.beginTransmission(0x3C);

	if (0 == Wire.endTransmission())
	{
		LOG_MSG("Display device found on channel " << 0x3C);
		display.enabled = true;
		(*display).begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 64x48)
		(*display).setTextWrap(false);
	}

	server.begin();

	EEPROM_Adapter_t::begin();
	TIME.begin(); //no need the time starts from 0
	irsend.begin();

	bool sts = true;
	uint16 index = 0;
	sts &= EEPROM_Adapter_t::load(ssid, index);
	sts &= EEPROM_Adapter_t::load(ssid_password, index);
	sts &= EEPROM_Adapter_t::load(blynk_auth, index);
	sts &= EEPROM_Adapter_t::load(blynk_domain, index);
	sts &= EEPROM_Adapter_t::load(blynk_port, index);
	if (!sts)
	{
		ssid = "";
		ssid_password = "";
		blynk_auth = "";

		LOG_MSG("Setting soft-AP ... ");

		String _SSID("WebIR_");
		_SSID += arduino::utils::Constants::ID();

		IPAddress local_IP(192, 168, 2, 1);
		IPAddress gateway(192, 168, 2, 1);
		IPAddress subnet(255, 255, 255, 0);

		if (WiFi.softAPConfig(local_IP, gateway, subnet) && WiFi.softAP(_SSID.c_str()))
		{
			LOG_MSG("Soft-AP IP address = " << WiFi.softAPIP().toString() << " (" << _SSID << ")");
		}
		else
		{
			LOG_MSG("Critical error!!! , not able to start server");
			RESET();
		}

		// Send web page with input fields to client
		//server.on("/", []() { server.send(200, "text/html", setupWiFiHTML); });

		server.addHandler( new SetUpWiFi_RequestHandler( ssid, ssid_password, blynk_auth, blynk_domain, blynk_port, data_offset , _SSID ,
			[](){
					LOG_MSG("Saving data ssid " << ssid << " pass " << ssid_password << " blynk auth" << blynk_auth);
					uint16 index = 0;

					index = EEPROM_Adapter_t::save(ssid);
					index = EEPROM_Adapter_t::save(ssid_password, index);
					index = EEPROM_Adapter_t::save(blynk_auth, index);
					index = EEPROM_Adapter_t::save(blynk_domain, index);
					index = EEPROM_Adapter_t::save(blynk_port, index);
					/*
					for (uint16 i = 0; i < IR_LAST; i++)
					{
						index = EEPROM_Adapter_t::save(  BufferAndSize_t< uint16,uint16 >( AC_COMMANDS[i], SIZE_OF_ARR( AC_COMMANDS[i] ) ) , index );
					}
					*/
					//restart in 5 sec
					idleTimer.addTask(TIME.getEpochTime() + 5, [](long&) { LOG_MSG("Going to restart");  RESET(); });

					return true;
				   }
			) );

		idleTimer.addRecuringTask(TIME.getEpochTime(), 3, [](long&) { rgbLed.blynk(RGBLed::LedType::Blue); });
	}
	else
	{

		LOG_MSG("Going to connect to " << ssid);

		WiFi.hostname(String(hostname) + arduino::utils::Constants::ID());
		WiFi.begin(ssid.c_str(), ssid_password.c_str());
		WiFi.setAutoReconnect(true);

		Blynk.config(blynk_auth.c_str(), blynk_domain.c_str(), blynk_port);

		onGotIPhandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP& event) { LOG_MSG("Local IP " << event.ip.toString());  Blynk.connect(1000); rgbLed.turn_on(RGBLed::LedType::Green); });
		onDisconnect = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected& event) { LOG_MSG("Disconnected"); rgbLed.turn_on(RGBLed::LedType::Red); Blynk.disconnect(); });

		idleTimer.addRecuringTask(TIME.getEpochTime(), 3, [](long& io_time) { (WiFi.isConnected()) ? rgbLed.turn_on(RGBLed::LedType::Green) : rgbLed.turn_on(RGBLed::LedType::Red); });

		idleTimer.addRecuringTask(TIME.getEpochTime(), 3, [](long& io_time) { (Blynk.connected()) ? stsRedLed.turn_off() : stsRedLed.blynk(); });

		if (display.enabled)
			idleTimer.addRecuringTask(TIME.getEpochTime(), 10, [](long& io_time) {  updateDisplay(bmeReadTemperature(), bmeReadHumidity(), (Blynk.connected() && WiFi.isConnected())); });

		idleTimer.addRecuringTask(TIME.getEpochTime(), 60, [](long& io_time) { if (WiFi.isConnected()) { LOG_MSG("MY IP " << WiFi.localIP().toString()); } });

		if (bme.enabled)
			idleTimer.addRecuringTask(TIME.getEpochTime(), arduino::utils::Timer::DAY / SIZE_OF_ARR(bme.data), [](long& io_time) { static uint8 i = 0; bme.data[i++ % SIZE_OF_ARR(bme.data)] = make_Pair(bmeReadTemperature(), bmeReadHumidity()); });

		// Send web page with input fields to client
		server.on("/", []() {
			String html;
			html += HTML_open;
			html += HTML_body_open;

			html += "Temperature :";
			html += bmeReadTemperature();
			html += HTML_new_line;
			html += "Humidity :";
			html += bmeReadHumidity();

			html += HTML_body_close;
			html += HTML_close;

			server.sendHeader("Access-Control-Allow-Origin", "*");
			server.send(200, "text/html", html.c_str());
		});


		server.on("/bme", []() {
			/*
			[
			{
				t: value,
				h: value
			}
			]
			*/
			char objOpen = '{';
			char objClose = '}';
			String jsonResp;
			jsonResp += '[';
			for (int8 i = 0; i < SIZE_OF_ARR(bme.data); i++)
			{
				jsonResp += objOpen;
				jsonResp += "\"t\":";
				jsonResp += String(bme.data[i].first);
				jsonResp += ',';
				jsonResp += "\"h\":";
				jsonResp += String(bme.data[i].second);
				jsonResp += objClose;
				if (i != SIZE_OF_ARR(bme.data) - 1)
					jsonResp += ',';
			}
			jsonResp += ']';
			server.sendHeader("Access-Control-Allow-Origin", "*");
			server.send(200, "text/html", jsonResp.c_str());
		});

		/*Setup SSDP*/

		server.on("/description.xml", HTTP_GET, []() {
			SSDP.schema(server.client());
		});

		SSDP.setDeviceType("upnp:rootdevice");
		SSDP.setSchemaURL("description.xml");
		SSDP.setHTTPPort(80);
		String _SSID(hostname);
		_SSID += arduino::utils::Constants::ID();
		SSDP.setName(_SSID);
		SSDP.setSerialNumber(arduino::utils::Constants::ID());
		SSDP.setURL("/");
		SSDP.setModelName(_SSID);
		SSDP.setModelNumber(arduino::utils::Constants::ID());
		SSDP.setModelURL("/");
		SSDP.setManufacturer("");
		SSDP.setManufacturerURL("/");
		SSDP.begin();
	}

	arduino::utils::OTA_Init(BUILD_ID);

	//data offset for array of codes
	data_offset = index;
	for (uint16 i = 0; i < IR_LAST; i++)
	{
		if ( EEPROM_Adapter_t::load( ir_coded[i], index ) )
		{
			LOG_MSG("Succesfully loaded " << index << " size " << ir_coded[i].m_size);
		}
		else
		{
			ir_coded[i].m_pData = AC_COMMANDS[i];
			ir_coded[i].m_size = SIZE_OF_ARR(AC_COMMANDS[i]);

			LOG_MSG("Command at index " << i << " not found  , using default, size " << ir_coded[i].m_size);
		}
	}

	server.addHandler(new IRConfiguration_RequestHander("setup"));
	server.addHandler(new IR_RequestHander("run"));
	server.addHandler(new LandingPage_RequestHandler());
	server.on("/test", []() { Led tst(IR_LED); tst.rapid_blynk(2000); server.send(200, "text/html", success_html);  });
	server.on("/clean", []() { 
		LOG_MSG("Cleaning eeprom");
		EEPROM_Adapter_t::clean();
		PlaceHolder p("delay", "5");
		server.send(200, "text/html", PlaceHolder::tr(&p, FPSTR(restart_in_process_html)));
		//restart in 5 sec
		idleTimer.addTask(TIME.getEpochTime() + 5, [](long&) { LOG_MSG("Going to restart");  RESET(); });
	});

	server.on("/restart", []() {
		PlaceHolder p("delay", "10");
		server.send(200, "text/html", PlaceHolder::tr(&p, FPSTR(restart_in_process_html)));
		//restart in 5 sec
		idleTimer.addTask(TIME.getEpochTime() + 10, [](long&) { LOG_MSG("Going to restart");  RESET(); });
	});

	server.onNotFound(notFound);

	stsRedLed.turn_on();
	
	Led(IR_LED).rapid_blynk(100);

	LOG_MSG("Application started");
}


struct TextSize
{
	short m_X;
	short m_Y;
	TextSize(short i_x, short i_y) : m_X(i_x), m_Y(i_y) {}
};

TextSize textSize[] = { TextSize(0, 0), TextSize(5, 7), TextSize(10, 14), TextSize(15, 21), TextSize(20, 28) };

auto printTemperature(float i_data)
{
	short digitsSize = 3, sighnSize = 2;

	(*display).setTextSize(digitsSize);

	(*display).print((short)i_data);

	(*display).setTextSize(sighnSize);
	(*display).setCursor((*display).getCursorX() + 3, (*display).getCursorY() + textSize[digitsSize].m_Y - textSize[sighnSize].m_Y);
	(*display).print("C");

	(*display).setTextSize(1);
	(*display).setCursor((*display).getCursorX() - textSize[sighnSize].m_X - 5, (*display).getCursorY() - textSize[1].m_Y - 1);

	(*display).print("o");

	(*display).setTextSize(1);
	(*display).setCursor((*display).width() - textSize[1].m_X, 1);

	if (g_isCool)
	{
		(*display).drawRect((*display).width() - textSize[1].m_X - 2, 0, textSize[1].m_X + 2, textSize[1].m_Y + 2, WHITE);
	}
	else
	{
		(*display).fillRect((*display).width() - textSize[1].m_X - 2, 0, textSize[1].m_X + 2, textSize[1].m_Y + 2, WHITE);
	}

	return textSize[digitsSize].m_Y;
}

auto printHumidity(float i_data ,  char i_sighn = '%')
{
	short digitsSize = 1, sighnSize = 1;

	(*display).setTextSize(digitsSize);
	(*display).printf("%.1f", i_data);

	(*display).setTextSize(sighnSize);
	(*display).setCursor((*display).getCursorX(), (*display).getCursorY() + textSize[digitsSize].m_Y - textSize[sighnSize].m_Y);
	(*display).print(i_sighn);

	return textSize[digitsSize].m_Y;
}

void updateDisplay(float i_temperature, float i_humidity, bool i_isconnected)
{
	if (!display.enabled)
		return;

	(*display).setTextColor(WHITE);
	(*display).setCursor(0, 0);
	(*display).clearDisplay();

	short nextLine = printTemperature(i_temperature);
	(*display).setCursor(0, ++nextLine);
	nextLine += printHumidity(i_humidity);
	(*display).setCursor(0, ++nextLine);

	(*display).setTextSize(1);

	if (i_isconnected)
	{
		(*display).println();
		(*display).println("Connected");
	}
	else
	{
		(*display).println();
		(*display).println("Not Connected");
	}

	(*display).display();
}

BLYNK_WRITE(V0)
{
	int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

	LOG_MSG("About to send COOL 21 or HOT 27 IR signal");

	irSend((g_isCool) ? ir_coded[IR_COLD_HIGH] : ir_coded[IR_HOT_HIGH]);

	ackLed.on();//sending to Blynk

	rgbLed.rapid_blynk(300);
}

BLYNK_READ(V1) //Blynk app has something on V5
{
	LOG_MSG("Writing to Blynk service");

	Blynk.virtualWrite(V1, bmeReadTemperature()); //sending to Blynk

	Blynk.virtualWrite(V2, bmeReadHumidity()); //sending to Blynk

	//rgbLed.rapid_blynk(300);
}

BLYNK_WRITE(V4)
{
	int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

	LOG_MSG("About to send OFF IR signal");

	if (0 != ackLed.getValue()) //turned on
	{
		irSend(ir_coded[IR_OFF]);
	}

	rgbLed.rapid_blynk(300);

	ackLed.off(); //sending to Blynk
}

BLYNK_WRITE(V5)
{
	int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

	LOG_MSG("About to send COOL 23 IR or HOT 25 signal");

	irSend((g_isCool) ? ir_coded[IR_COLD_LOW] : ir_coded[IR_HOT_LOW]);

	stsRedLed.rapid_blynk(300);

	ackLed.on();//sending to Blynk
}

BLYNK_WRITE(V6)
{
	g_isCool = (0 == param.asInt()); // assigning incoming value from pin V1 to a variable

	rgbLed.rapid_blynk(300);

	if (g_isCool)
		rgbLed.turn_on(RGBLed::LedType::Blue);
	else
		rgbLed.turn_on(RGBLed::LedType::Red);

	LOG_MSG("New mode is " << ((g_isCool) ? "cool" : "hot"));
}

void loop() {
	using namespace arduino::utils;
	OTA_CHECK_AND_UPDATE;

	static Button button(BUTTON, []() {},
		[]() {},
		[]() {
		LOG_MSG("Cleaning eeprom");
		EEPROM_Adapter_t::clean();
		stsRedLed.rapid_blynk(500);
	});


	button.run();
	server.handleClient();
	idleTimer.run();
	Blynk.run();
	TIME.run();
	actionTimer.run();
}