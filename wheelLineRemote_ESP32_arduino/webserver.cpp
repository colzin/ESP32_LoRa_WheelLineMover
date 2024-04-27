#include "webserver.h"

#include "defines.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/


#if WEBSERVER
WebServer server(80);
String htmlS = "<html>\
  <head>\
    <title>Wireless_Bridge</title>\
      <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\
  </head>\
  <style type=\"text/css\">\
    .header{display: block;margin-top:10px;text-align:center;width:100%;font-size:10px}\
    .input{display: block;text-align:center;width:100%}\
    .input input{height: 20px;width: 300px;text-align:center;border-radius:15px;}\
    .input select{height: 26px;width: 305px;text-align:center;border-radius:15px;}\
    .btn,button{width: 305px;height: 40px;border-radius:20px; background-color: #000000; border:0px; color:#ffffff; margin-top:20px;}\
  </style>\
  <script type=\"text/javascript\">\
    function myrefresh()\
    {\
      window.location.reload();\
    }\
window.onload=function(){\
      setTimeout('myrefresh()',4500);\
      }   \
  </script>\
  <body>\
    <div style=\"width:100%;text-align:center;font-size:25px;font-weight:bold;margin-bottom:20px\">Wireless_Bridge</div>\
      <div style=\"width:100%;text-align:center;\">\
        <div class=\"header\"><span>(Note 1: The default refresh time of this page is 10s. If you need to modify the refresh time, you can modify it in the 'setTimeout' function.)</span></div>\
        <div class=\"header\"><span>(Note 2: The refresh time needs to be modified according to the data sending frequency.)</span></div>\
        <div class=\"header\"><span>Data: ";
String htmlF = "</span></div>\
      </form>\
    </div>\
  </body>\
</html>";

String htmlW = "<html>\
  <head>\
    <title>Wireless_Bridge</title>\
      <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\
  </head>\
  <style type=\"text/css\">\
    .header{display: block;margin-top:10px;text-align:center;width:100%;font-size:10px}\
    .input{display: block;text-align:center;width:100%}\
    .input input{height: 20px;width: 300px;text-align:center;border-radius:15px;}\
    .input select{height: 26px;width: 305px;text-align:center;border-radius:15px;}\
    .btn,button{width: 305px;height: 40px;border-radius:20px; background-color: #000000; border:0px; color:#ffffff; margin-top:20px;}\
  </style>\
  <script type=\"text/javascript\">\
window.onload=function(){\
      document.getElementsByName(\"server\")[0].value = \"\";\
      }   \
  </script>\
  <body>\
    <div style=\"width:100%;text-align:center;font-size:25px;font-weight:bold;margin-bottom:20px\">Wireless_Bridge</div>\
      <div style=\"width:100%;text-align:center;\">\
        <div class=\"header\"><span>(Note 1: The data sent from this webpage to LoRa needs to be decoded to be able to be viewed normally.)</span></div>\
        <div class=\"header\"><span>(Note 2: The default size of data sent by LoRa is 4 bytes, which can be modified in the program.)</span></div>\
        <form method=\"POST\" action=\"\" onsubmit=\"\">\
          <div class=\"header\"><span>DATA</span></div>\
          <div class=\"input\"><input type=\"text\"  name=\"server\" value=\"\"></div>\
        <div class=\"header\"><input class=\"btn\" type=\"submit\" name=\"submit\" value=\"Submit\"></div>\
      </form>\
    </div>\
  </body>\
</html>";

String Page_data = "";
String symbol = ":";
static uint32_t g_lastNumRxSent = 0;
void ROOT_HTML()
{
  if (g_lastNumRxSent != g_numRx)
  {
    Page_data = Page_data + (String)g_numRx + symbol + LoRa_data + "<br>";
    String html = htmlS + Page_data + htmlF;
    server.send(200, "text/html", html);
    g_lastNumRxSent = g_numRx;
  }
}

bool WiFiDownLink = false;
uint32_t WiFidonwlinkTime;
String Write_data = "";
void ROOT_HTMLW()
{
  if (server.hasArg("server"))
  {
    Serial.println(server.arg("server"));
    Write_data = server.arg("server");
    WiFiDownLink = true;
    WiFidonwlinkTime = millis();
    // unsigned char* puc;
    // puc = (unsigned char*)(&Write_data);
    // appDataSize = 4;  //AppDataSize max value is 64
    // appData[0] = puc[0];
    // appData[1] = puc[1];
    // appData[2] = puc[2];
    // appData[3] = puc[3];
    // LoRaWAN.send();
    Serial.println("TODO send loRa");
  }
  server.send(200, "text/html", htmlW);
}

#endif // #if WEBSERVER