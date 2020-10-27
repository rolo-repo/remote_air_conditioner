// SetupBlynkAndWifi.h

#ifndef _SETUPBLYNKANDWIFI_h
#define _SETUPBLYNKANDWIFI_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#include "SerialOutput.h"
#include "PlaceHolder.h"
#include "EEPROM_Adapter.h"
#include "Constants.h"

#include "ESP8266WebServer.h"
namespace arduino {
	namespace utils {

		const char setupWiFiHTML[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>${title}</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script>
       function myFunction() {
  			var x = document.getElementById("password");
  			if (x.type === "password") {
  			  x.type = "text";
  			} else {
  			  x.type = "password";
  			}
			}
    </script>
  </head>
<body>
  <div style="background-color:lightblue" > 
  <form action="get">
  	<fieldset>
  		<legend>WiFi setting</legend>
  		<dl>
  		<dd><p style="text-align:left;">SSID: <br/><input type="text" size="30" name="ssid" required/><br /></p></dd>
  		<dd><p style="text-align:left;">Password:<br/> <input type="password" size="30" id="password" name="ssidp"/><input type="checkbox" onclick="myFunction()">Reveal<br /></p></dd>
		<dd><p style="text-align:left;">BLYNK AUTH <br/><input type="text" size="40" name="blynk_auth" required/><br /></p></dd> 
		<dd><p style="text-align:left;">BLYNK HOST <br/><input type="text" size="30" name="blynk_domain" value="blynk-cloud.com"/><br /></p></dd>  	
		<dd><p style="text-align:left;">BLYNK PORT <br/><input type="number" size="10" name="blynk_port" value="80"/><br /></p></dd>  		 	 		
		<dd><p><input type="submit" value="Save"></p></dd>
  	</dl>
  	</fieldset>
  </form>
  </div>
</body>
</html>
)rawliteral";

		class SetUpWiFi_RequestHandler : public RequestHandler
		{
		private:
			String &ssid;
			String &ssid_password;
			String &blynk_auth;
			String &blynk_domain;
			uint16 &blynk_port;
			uint16 &data_offset;

			String name;
			bool(*callback)();

			void mainPage(ESP8266WebServer& server)
			{
				PlaceHolder a1("title", name);
				server.send(200, "text/html", PlaceHolder::tr(PlaceHolder::makeList(VA_LIST(&a1)), FPSTR(setupWiFiHTML)));
			}

		public:

			SetUpWiFi_RequestHandler(String &o_ssid,
				String& o_ssid_password,
				String& o_blynk_auth,
				String& o_blynk_domain,
				uint16& o_blynk_port,
				uint16& o_data_offset,
				String i_name = "no name", bool(*i_callback)() = []() { return true; }) :
				ssid(o_ssid), ssid_password(o_ssid_password),
				blynk_auth(o_blynk_auth), blynk_domain(o_blynk_domain),
				blynk_port(o_blynk_port), data_offset(o_data_offset), name(i_name), callback(i_callback) {}


			bool canHandle(HTTPMethod method, String uri) {
				//	if (uri.startsWith("/get") && method == HTTP_GET)
				return true;
				//	else
					//	return false;
			}



			bool handle(ESP8266WebServer& server, HTTPMethod requestMethod, String requestUri) {

				if (server.args() == 0)
				{
					mainPage(server);

					return true;
				}

				const char* INPUT_1 = "ssid";
				const char* INPUT_2 = "ssidp";

				const char* INPUT_3 = "blynk_auth";
				const char* INPUT_4 = "blynk_domain";
				const char* INPUT_5 = "blynk_port";

				// GET input1 value on <ESP_IP>/get?input1=<inputMessage>
				if (server.hasArg(INPUT_1)) {
					ssid = server.arg(INPUT_1);
					ssid.trim();
				}

				if (server.hasArg(INPUT_2) && server.arg(INPUT_2).length() > 0) {
					ssid_password = server.arg(INPUT_2);
					ssid_password.trim();
				}


				if (server.hasArg(INPUT_3) && server.arg(INPUT_3).length() > 0) {
					blynk_auth = server.arg(INPUT_3);
					blynk_auth.trim();
				}


				if (server.hasArg(INPUT_4) && server.arg(INPUT_4).length() > 0) {
					blynk_domain = server.arg(INPUT_4);
					blynk_domain.trim();
				}

				if (server.hasArg(INPUT_5) && server.arg(INPUT_5).length() > 0) {
					blynk_port = (uint16)server.arg(INPUT_5).toInt();
				}

				//All is done we can call callback
				if (callback())
					server.send(200, "text/html", "Thank you");
				else
					mainPage(server);

			}
		};
	}
}
#endif

