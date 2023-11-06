//----------------------------------------------------------------------------
// TOOLS>BOARD SHOULD BE SET TO EITHER "ESP32 Dev Module" OR "ESP32-WROOM-DA Module"
// Troubleshooting: If the board reboots in a loop with SW_RESET, make sure:
//  - 'Flash Mode' is set to 'DIO'
//  - 'Flash Speed' is set to '40MHz'

//#define FASTLED_ALL_PINS_HARDWARE_SPI
//#define FASTLED_ALLOW_INTERRUPTS 0
#include <FastLED.h>
#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <uptime_formatter.h>

#include "HWPinConfig.h"
#include "Helpers.h"

//#define ENABLE_DEBUG_ID
#define HARWARE_NAME  "Cowking"

//----------------------------------------------------------------------------
// LED count

#define NUM_LEDS  43  // TODO: it would be much better if I could fit 44 on the ring.

//----------------------------------------------------------------------------

CRGB        leds[NUM_LEDS];

WebServer   server(80);
LoopTimer   loopTimer;
uint32_t    chipId = 0;
uint32_t    MsSinceConnected = 0;
IPAddress   wifi_AP_local_ip(192,168,1,1);
IPAddress   wifi_AP_gateway(192,168,1,1);
IPAddress   wifi_AP_subnet(255,255,255,0);

//----------------------------------------------------------------------------
// Read these from the flash

bool        has_wifi_credentials = false;
bool        has_dirty_credentials = false;
String      wifi_ST_SSID;
String      wifi_ST_Pass;
String		  wifi_AP_SSID;
String		  wifi_AP_Pass;
int32_t     state_mode = 0;
int32_t     state_debug_id = 0;
int32_t     state_brightness = 255;   // [0, 255]
int32_t     state_wave_speed = 50;    // [0, 100]
int32_t     state_wave_min = 25;      // [0, 100]
int32_t     state_wave_contrast = 1;  // [0, 3]
int32_t     state_wave_tiling = 50;   // [0, 100]
int32_t     state_base_speed = 50;    // [0, 100]
int32_t     state_base_min = 80;      // [0, 100]
int32_t     state_hue_shift = 0;      // [0, 255]
int32_t     state_sat_shift = 0;      // [-255, 255]
int32_t     state_hue_speed = 50;      // [0, 100]
CRGB        state_color_top = CRGB(255, 255, 255);
CRGB        state_color_bot = CRGB(255, 255, 255);
CRGB        state_color_left = CRGB(255, 255, 255);
CRGB        state_color_right = CRGB(255, 255, 255);

// Element sizes & addresses in the flash storage:
const int   kEEPROM_ST_ssid_size = 20;  // 20 chars max
const int   kEEPROM_ST_pass_size = 60;  // 60 chars max
const int   kEEPROM_AP_ssid_size = 20;  // 20 chars max
const int   kEEPROM_AP_pass_size = 60;  // 60 chars max
const int   kEEPROM_int32_t_size = 4;
const int   kEEPROM_col_rgb_size = 3;

const int   kEEPROM_ST_ssid_addr = 1;
const int   kEEPROM_ST_pass_addr = kEEPROM_ST_ssid_addr + kEEPROM_ST_ssid_size;
const int   kEEPROM_AP_ssid_addr = kEEPROM_ST_pass_addr + kEEPROM_ST_pass_size;
const int   kEEPROM_AP_pass_addr = kEEPROM_AP_ssid_addr + kEEPROM_AP_ssid_size;
const int   kEEPROM_mode_addr = kEEPROM_AP_pass_addr + kEEPROM_AP_pass_size;
const int   kEEPROM_debug_id_addr = kEEPROM_mode_addr + kEEPROM_int32_t_size;
const int   kEEPROM_bright_addr = kEEPROM_debug_id_addr + kEEPROM_int32_t_size;
const int   kEEPROM_wave_speed_addr = kEEPROM_bright_addr + kEEPROM_int32_t_size;
const int   kEEPROM_wave_min_addr = kEEPROM_wave_speed_addr + kEEPROM_int32_t_size;
const int   kEEPROM_wave_contrast_addr = kEEPROM_wave_min_addr + kEEPROM_int32_t_size;
const int   kEEPROM_wave_tiling_addr = kEEPROM_wave_contrast_addr + kEEPROM_int32_t_size;
const int   kEEPROM_base_speed_addr = kEEPROM_wave_tiling_addr + kEEPROM_int32_t_size;
const int   kEEPROM_base_min_addr = kEEPROM_base_speed_addr + kEEPROM_int32_t_size;
const int   kEEPROM_hue_shift_addr = kEEPROM_base_min_addr + kEEPROM_int32_t_size;
const int   kEEPROM_sat_shift_addr = kEEPROM_hue_shift_addr + kEEPROM_int32_t_size;
const int   kEEPROM_hue_speed_addr = kEEPROM_sat_shift_addr + kEEPROM_int32_t_size;
const int   kEEPROM_col_top_addr = kEEPROM_hue_speed_addr + kEEPROM_int32_t_size;
const int   kEEPROM_col_bot_addr = kEEPROM_col_top_addr + kEEPROM_col_rgb_size;
const int   kEEPROM_col_left_addr = kEEPROM_col_bot_addr + kEEPROM_col_rgb_size;
const int   kEEPROM_col_right_addr = kEEPROM_col_left_addr + kEEPROM_col_rgb_size;
const int   kEEPROM_col_FIRST_addr = kEEPROM_col_top_addr;

const int   kEEPROM_total_size = 1 + // first byte is a key == 0
                                 kEEPROM_ST_ssid_size + kEEPROM_ST_pass_size + kEEPROM_AP_ssid_size + kEEPROM_AP_pass_size +
                                 kEEPROM_int32_t_size * 12 +
                                 kEEPROM_col_rgb_size * 4;

static_assert(kEEPROM_total_size == kEEPROM_col_right_addr + kEEPROM_col_rgb_size);

//----------------------------------------------------------------------------

void setup()
{
//  setCpuFrequencyMhz(160);

  pinMode(PIN_OUT_WS2812, OUTPUT);

  for (int i = 0; i < NUM_LEDS; i++)
    leds[i].setRGB(0, 0, 0);

  // Setup LED strip
  FastLED.addLeds<WS2812, PIN_OUT_WS2812, GRB>(leds, NUM_LEDS);  // The WS2812 strip I got from aliexpress has red and green swapped, so GRB instead of RGB
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 2000);  // 5V, 2A
  FastLED.setBrightness(255);
  FastLED.clear();
  FastLED.show();

  Serial.begin(115200);
  Serial.println();

//  delay(15000); // DEBUG

  // Init baselines
  chipId = 0;
  for(int i=0; i<17; i=i+8)
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  Serial.printf("ESP32 Chip model = %s Rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("This chip has %d cores\n", ESP.getChipCores());
  Serial.printf("Chip ID: %08X\n", chipId);

  // Dump MAC address to serial. Useful for dev during initial network setup.
  // Just send the mac address to @vksiezak to pin this module to a specific IP address
  {
    uint8_t macAddr[6];
    WiFi.macAddress(macAddr);
    Serial.printf("Mac address: %02x:%02x:%02x:%02x:%02x:%02x\n", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
  }

  // Init default AP name before reading from flash
  wifi_AP_SSID = HARWARE_NAME "_" + String(chipId, HEX);
  wifi_AP_Pass = "0123456789";

  // Read the AP name & pass + server's IP & port we recorded from the previous runs to instantly connect to
  // what will likely be the correct one if we reboot.
  ESP32Flash_Init();
  ESP32Flash_ReadServerInfo();

  // Setup wifi in both access-point & station mode:
  SetupServer();

  // Try to connect to the wifi network
  ConnectToWiFi();

  WaitForWiFi(2000);  // Wait at most 2 seconds for wifi
}

//----------------------------------------------------------------------------

static CRGB ApplyPropagationBrightness(float cursor, float et, float p0, const CRGB &base)
{
  const float p1 = 4.0f * (state_wave_tiling / 100.0f);  // period of cursor offset, increase this to make the noise "tile" more across space
  const float bMin = state_wave_min / 100.0f;
  const float bMax = 1.0f;
  const float noise01 = Noise((et + cursor * p1) * p0) * 0.5f + 0.5f;           // remap from [-1, 1] to [0, 1]
  const float intensity_01 = clamp(noise01 * (bMax - bMin) + bMin, 0.0f, 1.0f); // remap from [0, 1] to [bMin, bMax]
#if 1
  const float intensity = powf(intensity_01, state_wave_contrast + 1.0f);
#else
  float       intensity = intensity_01;
  // Avoid using expensive powf()
  switch (state_wave_contrast)
  {
    case  0:
      break;
    case  1:
      intensity = intensity * intensity;
      break;
    case  2:
      intensity = intensity * intensity * intensity;
      break;
    case  3:
      intensity = intensity * intensity;
      intensity = intensity * intensity;
      break;
  }
#endif
  return CRGB(int32_t(base.r * intensity),
              int32_t(base.g * intensity),
              int32_t(base.b * intensity));
}

//----------------------------------------------------------------------------

void loop()
{
  loopTimer.tick();

  // Update wifi & web server
  WifiServerLoop();

  const float   dt = loopTimer.dt();
  const float   et = loopTimer.elapsedTime();

  // What to configure:
  // - Mode: 0=normal, 1=hue cycle, 2=debug zones, 3=debug ID, 4=debug ID cycle, 5=debug RGB cycle
  // - overall hue shift
  // - 5 inner colors
  // - 2 eye colors
  // - animation speed (main)
  // - animation speed (eyes)

  FastLED.setBrightness(state_brightness);

  if (state_mode == 0 || state_mode == 1)
  {
    const float   hue_speed = (state_mode == 1) ? 0.5f * state_hue_speed / 100.0f : 0.0f;
    const int32_t hue_shift_anim = int32_t(fmodf(hue_speed * et, 1.0f) * 255.0f);
    const int32_t hue_shift = (state_hue_shift + hue_shift_anim) % 256;

    const CRGB  colors[4] =
    {
      ShiftHS(state_color_top, hue_shift, state_sat_shift),
      ShiftHS(state_color_right, hue_shift, state_sat_shift),
      ShiftHS(state_color_bot, hue_shift, state_sat_shift),
      ShiftHS(state_color_left, hue_shift, state_sat_shift),
    };

    // Compute global brightness level
    const float p0 = 2.0f * state_base_speed / 100.0f;
    const float p1 = 2.0f * state_wave_speed / 100.0f;
    const float bMin = state_base_min / 100.0f;
    const float bMax = 1.0f;
    const float noise01 = Noise(et * p0) * 0.5f + 0.5f;                           // remap from [-1, 1] to [0, 1]
    const float intensity_01 = clamp(noise01 * (bMax - bMin) + bMin, 0.0f, 1.0f); // remap from [0, 1] to [bMin, bMax]
    FastLED.setBrightness(int32_t(state_brightness * intensity_01));

    const float di = 4.0f / NUM_LEDS;
    float       fi = 0.0f;
    for (int i = 0; i < NUM_LEDS; i++)
    {
      const int   i0 = int(fi);
      const int   i1 = (i0 + 1) % 4;
      const CRGB  &c0 = colors[i0];
      const CRGB  &c1 = colors[i1];
      const CRGB  c = c0.lerp8(c1, uint8_t(fi * 256.0f));
      const float cursor = 1.0f - fabsf((fi / 2.0f) - 1.0f);
      leds[i] = ApplyPropagationBrightness(cursor, et, p1, c);
      fi += di;
    }
  }
  else if (state_mode == 2) // Coloration per-area
  {
    int i = 0;
    for (; i < (1*NUM_LEDS)/4; i++)
      leds[i] = CRGB(255, 40, 10);
    for (; i < (2*NUM_LEDS)/4; i++)
      leds[i] = CRGB(10, 150, 255);
    for (; i < (3*NUM_LEDS)/4; i++)
      leds[i] = CRGB(255, 150, 10);
    for (; i < (4*NUM_LEDS)/4; i++)
      leds[i] = CRGB(80, 255, 10);
  }
  else
  {
    // Increase 'x' counter every 0.2 seconds to animate the debug modes:
    static int    x = 0;
    {
      static float  t = 0;
      const float   p = 0.2f;
      t += dt;
      if (t > p)
      {
        t -= p;
        x++;
      }
    }

    if (state_mode == 3 || state_mode == 4)  // Highlight specific LED
    {
      const unsigned int  led_to_highlight = (state_mode == 3) ? state_debug_id : (x % NUM_LEDS);
      for (int i = 0; i < NUM_LEDS; i++)
        leds[i].setRGB(255, 50, 10);
      if (led_to_highlight < NUM_LEDS)
          leds[led_to_highlight].setRGB(255, 255, 255);
    }
    else  // RGB "christmas tree" + brightness variation
    {
      const float b = sinf(et) * 0.5f + 0.5f;
      const int   c0 = clamp((int)(b * 255) + 1, 1, 255);
      const int   c1 = clamp((int)(b * 50), 0, 255);
      for (int i = 0; i < NUM_LEDS; i++)
      {
        int key = (i + x) % NUM_LEDS;
        leds[i].setRGB((key % 3) == 0 ? c0 : c1, ((key + 1) % 3) == 0 ? c0 : c1, ((key + 2) % 3) == 0 ? c0 : c1);
      }
    }
  }

  FastLED.show();

  delay(20);
}

//----------------------------------------------------------------------------
//
//  WiFi helpers
//
//----------------------------------------------------------------------------

void  ConnectToWiFi()
{
  if (!has_wifi_credentials)  // Nothing to connect to if we don't have any credentials
    return;

  Serial.println("Connecting to WiFi: " + wifi_ST_SSID);

  WiFi.begin(wifi_ST_SSID, wifi_ST_Pass);
}

//----------------------------------------------------------------------------

void  WaitForWiFi(int maxMs)
{
  if (!has_wifi_credentials)  // Nothing to wait on if we don't have any credentials
    return;

  const int kMsPerRetry = 500;
  int retries = max(maxMs / kMsPerRetry, 1);
  Serial.print("Waiting for Wifi connection");
  while (WiFi.status() != WL_CONNECTED && retries-- > 0)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("WiFi connected to " + wifi_ST_SSID);
    Serial.println("IP address: " + WiFi.localIP());
  }
}

//----------------------------------------------------------------------------

void  EnsureWiFiConnected()
{
  static long int prevMS = millis() - 10;
  const long int  curMS = millis();
  const int       dtMS = (curMS > prevMS) ? curMS - prevMS : 10;  // by default 10 ms when wrapping around
  prevMS = curMS;

  // The wifi network might have gone down, then up again.
  // We don't want to require rebooting the IOT there, so if the connection was lost, redo a connection round
  if (WiFi.status() != WL_CONNECTED)
  {
    MsSinceConnected += dtMS;
    if (MsSinceConnected > 10000) // retry every 10s
    {
      MsSinceConnected = 0;
      ConnectToWiFi();
    }
  }
  else if (MsSinceConnected != 0) // was previously not connected
  {
    MsSinceConnected = 0;
    Serial.println("WiFi connected to " + wifi_ST_SSID);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

//----------------------------------------------------------------------------

void  DisconnectFromWiFi()
{
  WiFi.disconnect();
}

//----------------------------------------------------------------------------

void  WifiServerLoop()
{
  server.handleClient();

  if (has_dirty_credentials)
  {
    Serial.println("Has dirty credentials");
    DisconnectFromWiFi();
    ConnectToWiFi();
    MsSinceConnected = 0;
    has_dirty_credentials = false;
  }

  if (has_wifi_credentials)
    EnsureWiFiConnected();
}

//----------------------------------------------------------------------------
//
//  Web server
//
//----------------------------------------------------------------------------

#define FONT_OTAG_CODE  "<font class=\"code\">"

static String  _BuildStandardResponsePage(const String &contents)
{
  static uint32_t reqId = 0;
  String reply;
  reply += "<!DOCTYPE HTML>\r\n"
           "<html><head><style>\r\n"
           "body { font-size: 9pt; font-family: Roboto, Tahoma, Verdana; }\r\n"
           "table { border-collapse: collapse; font-size: 9pt; }\r\n"
           "td { padding: 2px; padding-left: 10px; padding-right: 10px; }\r\n"
           "td.code { font-family: Consolas, Courier; }\r\n"
           "td.header { color: #AAA; background-color: #505050; }\r\n"
           ".code { font-family: Consolas, Courier; background:#EEE; }\r\n"
           "</style></head><body>\r\n"
           "<h1>" + wifi_AP_SSID + "</h1>\r\n"
           "<hr/>\r\n";
  reply += contents;
  reply += "<hr/>\r\n"
           "Uptime: ";
  reply += uptime_formatter::getUptime();
  reply += "<br/>\r\n"
           "Requests since startup: ";
  reply += ++reqId;
  reply += "<br/>\r\n"
           "Chip ID: " FONT_OTAG_CODE;
  reply += String(chipId, HEX);
  reply += "</font><br/>\r\n";
  reply += "MAC Address: " FONT_OTAG_CODE + WiFi.macAddress() + "</font><br/>\r\n";
  reply += "CPU Frequency: " FONT_OTAG_CODE + String(getCpuFrequencyMhz()) + " MHz</font><br/>\r\n";
  reply += "APB Frequency: " FONT_OTAG_CODE + String(int32_t(getApbFrequency() / 1.0e+6f)) + " MHz</font><br/>\r\n";
  reply += "XTAL Frequency: " FONT_OTAG_CODE + String(getXtalFrequencyMhz()) + " MHz</font>";

  if (WiFi.status() == WL_CONNECTED)
  {
    reply += "<br/>\r\n"
             "IP Address: " FONT_OTAG_CODE;
    reply += WiFi.localIP().toString();
    reply += "</font><br/>\r\n"
             "Signal strength: ";
    reply += WiFi.RSSI();
    reply += " dB<br/>\r\n";
  }

  reply += "</body></html>\r\n";
  return reply;
}

//----------------------------------------------------------------------------
// HTML form helpers

#define FORM_INPUTBOX(__name, __title, __value) \
    "  <tr>\r\n" \
    "    <td><label for=\"" __name "\">" __title ":</label></td>\r\n" \
    "    <td><input type=\"text\" id=\"" __name "\" name=\"" __name "\" value=\"" + __value + "\"/></td>\r\n" \
    "  </tr>"
#define FORM_PASSWORD(__name, __title, __value) \
    "  <tr>\r\n" \
    "    <td><label for=\"" __name "\">" __title ":</label></td>\r\n" \
    "    <td><input type=\"password\" id=\"" __name "\" name=\"" __name "\" value=\"" + __value + "\"/><input type=\"checkbox\" onclick=\"togglePassword_" __name "()\"/>Show\r\n" \
    "      <script>function togglePassword_" __name "() { var x = document.getElementById(\"" __name "\"); if (x.type == \"password\") x.type = \"text\"; else x.type = \"password\"; }</script>" \
    "    </td>" \
    "  </tr>"
#define FORM_SLIDER(__name, __title, __min, __max, __value) \
    "  <tr>\r\n" \
    "    <td><label for=\"" __name "\">" __title ":</label></td>\r\n" \
    "    <td><input type=\"range\" min=\"" __min "\" max=\"" __max "\" id=\"" __name "\" name=\"" __name "\" value=\"" + __value + "\"/></td>\r\n" \
    "  </tr>"
#define FORM_SLIDER_RST(__name, __title, __min, __max, __value, __default) \
    "  <tr>\r\n" \
    "    <td><label for=\"" __name "\">" __title ":</label></td>\r\n" \
    "    <td><input type=\"range\" min=\"" __min "\" max=\"" __max "\" id=\"" __name "\" name=\"" __name "\" value=\"" + __value + "\"/>&nbsp;\r\n" \
    "      <button type=\"button\" style=\"height:20px;\" onclick=\"reset_" __name "()\">Reset</button>\r\n" \
    "      <script>function reset_" __name "() { var x = document.getElementById(\"" __name "\"); x.value = \""  __default  "\"; }</script>" \
    "    </td>\r\n" \
    "  </tr>"
#define FORM_RGB(__name, __title, __value_r, __value_g, __value_b) \
    "  <tr>\r\n" \
    "    <td><label for=\"" __name "_r\">" __title ":</label></td>\r\n" \
    "    <td><input type=\"text\" id=\"" __name "_r\" name=\"" __name "_r\" value=\"" + __value_r + "\" style=\"width:50px\"/>\r\n" \
    "        <input type=\"text\" id=\"" __name "_g\" name=\"" __name "_g\" value=\"" + __value_g + "\" style=\"width:50px\"/>\r\n" \
    "        <input type=\"text\" id=\"" __name "_b\" name=\"" __name "_b\" value=\"" + __value_b + "\" style=\"width:50px\"/></td>\r\n" \
    "  </tr><tr>\r\n"

//----------------------------------------------------------------------------

static void _HandleRoot()
{
  String  reply;
  reply += "<h2>Configure lighting:</h2>\r\n"
           "<form action=\"/configure\" id=\"configForm\">\r\n"
           "  <table>\r\n"
           "  <tr>\r\n"
           "    <td><label for=\"mode\">Mode:</label></td>\r\n"
           "    <td>\r\n"
           "      <select id=\"mode\" name=\"mode\" onChange=\"on_mode_changed()\">\r\n"
           "        <option value=\"0\">Normal</option>\r\n"
           "        <option value=\"1\"" + String(state_mode == 1 ? " selected" : "") + ">Hue cycle</option>\r\n"
           "        <option value=\"2\"" + String(state_mode == 2 ? " selected" : "") + ">Debug regions</option>\r\n"
#if defined(ENABLE_DEBUG_ID)
           "        <option value=\"3\"" + String(state_mode == 3 ? " selected" : "") + ">Debug ID</option>\r\n"
#endif
           "        <option value=\"4\"" + String(state_mode == 4 ? " selected" : "") + ">Debug ID Cycle</option>\r\n"
           "        <option value=\"5\"" + String(state_mode == 5 ? " selected" : "") + ">Debug RGB Cycle</option>\r\n"
           "      </select>&nbsp;\r\n"
#if defined(ENABLE_DEBUG_ID)
           "      <input type=\"text\" id=\"did\" name=\"did\" value=\"" + String(state_debug_id) + "\" style=\"width: 40px; visibility: " + String(state_mode == 3 ? "visible" : "hidden") + ";\">\r\n"
           "      <script>function on_mode_changed() {\r\n"
           "          var m = document.getElementById(\"mode\");\r\n"
           "          var id = document.getElementById(\"did\");\r\n"
           "          id.style.visibility = (m.value == 3) ? 'visible' : 'hidden';\r\n"
           "        }</script>\r\n"
#else
           "      <script>function on_mode_changed() {}</script>\r\n"
#endif
           "    </td>\r\n"
           "  </tr>\r\n"
           "  <tr>\r\n"
           "    <td colspan=2 height=\"25px\"><big><b>Color control:</b></big></td>\r\n"
           "  </tr>\r\n"
           FORM_SLIDER_RST("bright", "Brightness", "0", "255", String(state_brightness), "255")
           FORM_SLIDER_RST("sshift", "Saturation", "-255", "255", String(state_sat_shift), "0")
           FORM_SLIDER_RST("hshift", "Hue shift", "0", "255", String(state_hue_shift), "0")
           FORM_RGB("col_top", "Top color", String(state_color_top.r), String(state_color_top.g), String(state_color_top.b))
           FORM_RGB("col_bot", "Bottom color", String(state_color_bot.r), String(state_color_bot.g), String(state_color_bot.b))
           FORM_RGB("col_left", "Left color", String(state_color_left.r), String(state_color_left.g), String(state_color_left.b))
           FORM_RGB("col_right", "Right color", String(state_color_right.r), String(state_color_right.g), String(state_color_right.b))
           "  <tr>\r\n"
           "    <td colspan=2 height=\"25px\"><big><b>Animation:</b></big></td>\r\n"
           "  </tr>\r\n"
           FORM_SLIDER("wspeed", "Wave anim speed", "0", "100", String(state_wave_speed))
           FORM_SLIDER("wmin", "Wave min intensity", "0", "100", String(state_wave_min))
           FORM_SLIDER("wcontrast", "Wave contrast", "0", "3", String(state_wave_contrast))
           FORM_SLIDER("wtiling", "Wave tiling", "0", "100", String(state_wave_tiling))
           FORM_SLIDER("bspeed", "Base anim speed", "0", "100", String(state_base_speed))
           FORM_SLIDER("bmin", "Base min intensity", "0", "100", String(state_base_min))
           FORM_SLIDER("hspeed", "Hue shift speed", "0", "100", String(state_hue_speed))
           "  <tr>\r\n"
           "    <td colspan=2><input type=\"submit\" value=\"Submit\" id=\"submit\"/>&nbsp;<span class=\"statusCtrl\"></span></td>\r\n"
           "  </tr>\r\n"
           "  </table>\r\n"
           "</form>\r\n";

  reply += "<hr/>\r\n"
           "<h2>Set WiFi network credentials:</h2>\r\n"
           "This will allow the device to connect to your local WiFi, and access this page through your WiFi network without the need to connect to the access-point.<br/><br/>\r\n"
           "<form action=\"/set_credentials\" id=\"wifiForm\">\r\n"
           "  <table>\r\n"
           FORM_INPUTBOX("ssid", "WiFi SSID", wifi_ST_SSID)
           FORM_PASSWORD("passwd", "WiFi Password", wifi_ST_Pass)
           "  <tr>\r\n"
           "    <td colspan=2><input type=\"submit\" value=\"Submit\"/>&nbsp;<span class=\"statusCtrl\"></span></td>\r\n"
           "  </tr>\r\n"
           "  </table>\r\n"
           "</form>\r\n"
    		   "<hr/>\r\n"
    		   "<a href=\"/set_credentials_ap\">Change access-point settings</a><br/>\r\n"
#if !defined(ENABLE_DEBUG_ID)
           "<script>\r\n"
           "function postFormNoRefresh(e)\r\n"
           "{\r\n"
           "  const form = e.target;\r\n"
           "  var x = form.getElementsByClassName(\"statusCtrl\")[0];\r\n"
           "  x.innerHTML = \"...\";\r\n"
           "  e.preventDefault();\r\n"
           "  fetch(form.action, {\r\n"
           "    method: \"POST\",\r\n"
           "    body: new FormData(form),\r\n"
           "  })\r\n"
           "  .then((res) => x.innerHTML = \"OK\")\r\n"
           "  .catch((err) => {\r\n"
           "    console.log(err.message)\r\n"
           "    x.innerHTML = \"Query Failed\";\r\n"
           "  });\r\n"
           "}\r\n"
           "document.addEventListener(\"DOMContentLoaded\", () => {\r\n"
           "  document.getElementById(\"configForm\").addEventListener(\"submit\", postFormNoRefresh);\r\n"
           "  document.getElementById(\"wifiForm\").addEventListener(\"submit\", postFormNoRefresh);\r\n"
           "});\r\n"
#endif
           "</script>\r\n";

  server.send(200, "text/html", _BuildStandardResponsePage(reply));
}

//----------------------------------------------------------------------------

static void _HandleNotFound()
{
  String reply;
  reply += "Unhandled Request\n\n";
  reply += "URI: ";
  reply += server.uri();
  reply += "\nMethod: ";
  reply += (server.method() == HTTP_GET) ? "GET" : "POST";
  reply += "\nArguments: ";
  reply += server.args();
  reply += "\n";
  for (uint8_t i = 0; i < server.args(); i++)
    reply += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  server.send(404, "text/plain", reply);
}

//----------------------------------------------------------------------------

static void _HandleSetCredentials()
{
  // Expected format:
  // /set_credentials?ssid=WifiName&passwd=WifiPassword

  String    ssid;
  String    pass;
  for (uint8_t i = 0; i < server.args(); i++)
  {
    if (server.argName(i) == "ssid")
       ssid = server.arg(i);
    else if (server.argName(i) == "passwd")
       pass = server.arg(i);
  }

  if (ssid != wifi_ST_SSID || pass != wifi_ST_Pass)
  {
    wifi_ST_SSID = ssid;
    wifi_ST_Pass = pass;
    ESP32Flash_WriteServerInfo();
    has_wifi_credentials = wifi_ST_SSID.length() > 0;
    has_dirty_credentials = true;  // Reconnect to wifi on next loop
    Serial.println("Got new Wifi network credentials");
  }

  // Auto-redirect immediately to the root
  server.send(200, "text/html", "<html><head><meta http-equiv=\"refresh\" content=\"3; URL=/\" /></head><body></body></html>");
}

//----------------------------------------------------------------------------

static void _HandleSetCredentialsAP()
{
  // Expected format:
  // /set_credentials_ap?ssid=WifiName&passwd=WifiPassword

  String    ssid;
  String    pass;
  for (uint8_t i = 0; i < server.args(); i++)
  {
    if (server.argName(i) == "ssid")
       ssid = server.arg(i);
    else if (server.argName(i) == "passwd")
       pass = server.arg(i);
  }
  
  String	reply;
  
  const bool	hasSSID = ssid.length() != 0;
  const bool	hasPass = pass.length() != 0;
  if (!hasSSID && !hasPass)
  {
    // No args to the page: Display regular page with the form to change it
    reply += "<h2>Set access-point credentials</h2>\r\n"
             "This allows to configure the access-point settings of the device:<br/>\r\n"
             "It will change how the device appears in the list of wireless networks, and the password needed to connect to it.<br/>\r\n"
             "<br/>\r\n"
             "These will only be taken into account after a reboot of the device.<br/>\r\n"
             "To reboot the device, simply power it off, then on again.<br/>\r\n"
             "<hr/>\r\n"
    			   "<font color=red><b>!!!! DANGER ZONE !!!</b><br/>\r\n"
    			   "If you do not remember the access-point password, and you did not setup a connection to your local WiFi network, there will be no way to recover this.\r\n"
    			   "The device will be \"bricked\" as you won't be able to connect to it from anywhere, and only re-flashing the firmware from the USB connection will fix this.</font><br/>\r\n"
    			   "<br/>\r\n"
    			   "<form action=\"/set_credentials_ap\">\r\n"
             "  <table>\r\n"
             FORM_INPUTBOX("ssid", "Access-point SSID", wifi_AP_SSID)
             FORM_PASSWORD("passwd", "Access-point Password", wifi_AP_Pass)
             "  <tr>\r\n"
             "    <td colspan=2><input type=\"submit\" value=\"Submit\"/></td>\r\n"
             "  </tr>\r\n"
             "  </table>\r\n"
		         "</form>\r\n";

	  reply = _BuildStandardResponsePage(reply);
  }
  else if (hasSSID != hasPass)
  {
	  // One of them is empty but not the other: Don't allow.
	  // We must have a non-empty password and a non-empty access-point name.
    reply += "<h3>Set access-point credentials</h3>\r\n"
             "Invalid credentials: You must specify both an SSID (the name the device as it will appear in the WiFi networks list), as well as a password (the password you will need to enter to connect to the device).<br/>\r\n"
             "<br/>\r\n"
		         "</form>\r\n";

	  reply = _BuildStandardResponsePage(reply);
  }
  else
  {
	  // Both are non-empty: OK
	  if (ssid != wifi_ST_SSID || pass != wifi_ST_Pass)
	  {
  		wifi_AP_SSID = ssid;
  		wifi_AP_Pass = pass;
  		ESP32Flash_WriteServerInfo();
  		// New AP name will be taken into account on next device boot: That's OK
  		
  		// If we want to do it without reboot, set a flag here, and in 'loop()', if the flag is set, call 'SetupServer()'.
  		// However it will disconnect the current connection, and might be balls-breaking. IE: maybe you made a typo in the password,
  		// and you have no way to re-check it once you clicked 'Submit', and.. you'll be fucked ! you'll have to re-upload a new firmware
  		// if you had not connected to the local wifi before. So this is dangerous.
	  }
	  // Auto-redirect immediately back to root
	  reply = "<html><head><meta http-equiv=\"refresh\" content=\"3; URL=/\" /></head><body></body></html>";
  }

  // Redirect immediately to the root
  server.send(200, "text/html", reply);
}

//----------------------------------------------------------------------------

static void _HandleConfigure()
{
  for (uint8_t i = 0; i < server.args(); i++)
  {
    const String  argName = server.argName(i);
    const int32_t argInt = server.arg(i).toInt();
    if (argName == "mode")
       state_mode = argInt;
    else if (argName == "did")
       state_debug_id = argInt;
    else if (argName == "bright")
       state_brightness = argInt;
    else if (argName == "wspeed")
       state_wave_speed = argInt;
    else if (argName == "wmin")
       state_wave_min = argInt;
    else if (argName == "wcontrast")
       state_wave_contrast = argInt;
    else if (argName == "wtiling")
       state_wave_tiling = argInt;
    else if (argName == "bspeed")
       state_base_speed = argInt;
    else if (argName == "bmin")
       state_base_min = argInt;
    else if (argName == "sshift")
       state_sat_shift = argInt;
    else if (argName == "hshift")
       state_hue_shift = argInt;
    else if (argName == "hspeed")
       state_hue_speed = argInt;
    else if (argName == "col_top_r")  // Top
       state_color_top.r = argInt;
    else if (argName == "col_top_g")
       state_color_top.g = argInt;
    else if (argName == "col_top_b")
       state_color_top.b = argInt;
    else if (argName == "col_bot_r")  // Bottom
       state_color_bot.r = argInt;
    else if (argName == "col_bot_g")
       state_color_bot.g = argInt;
    else if (argName == "col_bot_b")
       state_color_bot.b = argInt;
    else if (argName == "col_left_r")   // Left
       state_color_left.r = argInt;
    else if (argName == "col_left_g")
       state_color_left.g = argInt;
    else if (argName == "col_left_b")
       state_color_left.b = argInt;
    else if (argName == "col_right_r")   // Right
       state_color_right.r = argInt;
    else if (argName == "col_right_g")
       state_color_right.g = argInt;
    else if (argName == "col_right_b")
       state_color_right.b = argInt;
  }

  ESP32Flash_WriteServerInfo();

  // Auto-redirect immediately to the root
  server.send(200, "text/html", "<html><head><meta http-equiv=\"refresh\" content=\"3; URL=/\" /></head><body></body></html>");
}

//----------------------------------------------------------------------------

void  SetupServer()
{
  server.stop();
  server.close(); // just in case we're reconnecting

  // Log SSID & Password of local access-point in case it gets set and forgotten, and the local wifi network access hasn't been setup
  // In that case, there's no way to recover it, except by wathing the serial output through the USB connection, and seeing this print:
  Serial.println("Setting up access-point: \"" + wifi_AP_SSID + "\", Password: \"" + wifi_AP_Pass + "\"");

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(wifi_AP_SSID, wifi_AP_Pass);
  WiFi.softAPConfig(wifi_AP_local_ip, wifi_AP_gateway, wifi_AP_subnet);

  // Setup server callbacks/handlers
  server.on("/", _HandleRoot);
  server.on("/set_credentials", _HandleSetCredentials);
  server.on("/set_credentials_ap", _HandleSetCredentialsAP);
  server.on("/configure", _HandleConfigure);
  server.onNotFound(_HandleNotFound);

  // Start the server
  server.begin();
  Serial.println("Server started");
}

//----------------------------------------------------------------------------
//
//  EEPROM helper for ESP32
//  Will not work on regular arduino, EEPROM interface is slightly different.
//  EEPROM emulates through flash memory on ESP32
//
//----------------------------------------------------------------------------

void  ESP32Flash_Init()
{
  EEPROM.begin(kEEPROM_total_size);

  // factory-initialized to 0xFF. Our first byte should always be exactly 0.
  // If it's not, wipe the entire memory.
  if (EEPROM.read(0) != 0)
  {
    // Init all the beginning before the first colors to 0
    for (int i = 0, stop = kEEPROM_col_FIRST_addr; i < stop; i++)
      EEPROM.write(i, 0);

    // Init all colors to 0xFF (assumes colors appear last)
    for (int i = kEEPROM_col_FIRST_addr, stop = EEPROM.length(); i < stop; i++)
      EEPROM.write(i, 0xFF);

    // Hand-patch a few things that must be initialized to zero:
    EEPROM.put(kEEPROM_bright_addr, int32_t(0xFF));
    EEPROM.put(kEEPROM_wave_speed_addr, int32_t(50));
    EEPROM.put(kEEPROM_wave_min_addr, int32_t(25));
    EEPROM.put(kEEPROM_wave_contrast_addr, int32_t(1));
    EEPROM.put(kEEPROM_wave_tiling_addr, int32_t(50));
    EEPROM.put(kEEPROM_base_speed_addr, int32_t(50));
    EEPROM.put(kEEPROM_base_min_addr, int32_t(80));
    EEPROM.put(kEEPROM_hue_speed_addr, int32_t(50));

    EEPROM.commit();
  }
}

//----------------------------------------------------------------------------

void  ESP32Flash_WriteServerInfo()
{
  char      eeprom_ST_ssid[kEEPROM_ST_ssid_size] = {};
  char      eeprom_ST_pass[kEEPROM_ST_pass_size] = {};
  char      eeprom_AP_ssid[kEEPROM_AP_ssid_size] = {};
  char      eeprom_AP_pass[kEEPROM_AP_pass_size] = {};
  static_assert(sizeof(eeprom_ST_ssid) == kEEPROM_ST_ssid_size);
  static_assert(sizeof(eeprom_ST_pass) == kEEPROM_ST_pass_size);
  static_assert(sizeof(eeprom_AP_ssid) == kEEPROM_AP_ssid_size);
  static_assert(sizeof(eeprom_AP_pass) == kEEPROM_AP_pass_size);

  for (int i = 0; i < wifi_ST_SSID.length() && i < sizeof(eeprom_ST_ssid)-1 - 1; i++)
    eeprom_ST_ssid[i] = wifi_ST_SSID[i];
  eeprom_ST_ssid[sizeof(eeprom_ST_ssid)-1] = '\0';

  for (int i = 0; i < wifi_ST_Pass.length() && i < sizeof(eeprom_ST_pass)-1 - 1; i++)
    eeprom_ST_pass[i] = wifi_ST_Pass[i];
  eeprom_ST_pass[sizeof(eeprom_ST_pass)-1] = '\0';

  for (int i = 0; i < wifi_AP_SSID.length() && i < sizeof(eeprom_AP_ssid)-1 - 1; i++)
    eeprom_AP_ssid[i] = wifi_AP_SSID[i];
  eeprom_AP_ssid[sizeof(eeprom_AP_ssid)-1] = '\0';

  for (int i = 0; i < wifi_AP_Pass.length() && i < sizeof(eeprom_AP_pass)-1 - 1; i++)
    eeprom_AP_pass[i] = wifi_AP_Pass[i];
  eeprom_AP_pass[sizeof(eeprom_AP_pass)-1] = '\0';

  EEPROM.write(0, 0);
  EEPROM.put(kEEPROM_ST_ssid_addr, eeprom_ST_ssid);
  EEPROM.put(kEEPROM_ST_pass_addr, eeprom_ST_pass);
  EEPROM.put(kEEPROM_AP_ssid_addr, eeprom_AP_ssid);
  EEPROM.put(kEEPROM_AP_pass_addr, eeprom_AP_pass);

  EEPROM.put(kEEPROM_mode_addr, state_mode);
  EEPROM.put(kEEPROM_debug_id_addr, state_debug_id);
  EEPROM.put(kEEPROM_bright_addr, state_brightness);

  EEPROM.put(kEEPROM_wave_speed_addr, state_wave_speed);
  EEPROM.put(kEEPROM_wave_min_addr, state_wave_min);
  EEPROM.put(kEEPROM_wave_contrast_addr, state_wave_contrast);
  EEPROM.put(kEEPROM_wave_tiling_addr, state_wave_tiling);
  EEPROM.put(kEEPROM_base_speed_addr, state_base_speed);
  EEPROM.put(kEEPROM_base_min_addr, state_base_min);
  EEPROM.put(kEEPROM_hue_shift_addr, state_hue_shift);
  EEPROM.put(kEEPROM_sat_shift_addr, state_sat_shift);
  EEPROM.put(kEEPROM_hue_speed_addr, state_hue_speed);

  static_assert(sizeof(state_color_top.r) == 1);
  EEPROM.put(kEEPROM_col_top_addr + 0, state_color_top.r);
  EEPROM.put(kEEPROM_col_top_addr + 1, state_color_top.g);
  EEPROM.put(kEEPROM_col_top_addr + 2, state_color_top.b);
  EEPROM.put(kEEPROM_col_bot_addr + 0, state_color_bot.r);
  EEPROM.put(kEEPROM_col_bot_addr + 1, state_color_bot.g);
  EEPROM.put(kEEPROM_col_bot_addr + 2, state_color_bot.b);
  EEPROM.put(kEEPROM_col_left_addr + 0, state_color_left.r);
  EEPROM.put(kEEPROM_col_left_addr + 1, state_color_left.g);
  EEPROM.put(kEEPROM_col_left_addr + 2, state_color_left.b);
  EEPROM.put(kEEPROM_col_right_addr + 0, state_color_right.r);
  EEPROM.put(kEEPROM_col_right_addr + 1, state_color_right.g);
  EEPROM.put(kEEPROM_col_right_addr + 2, state_color_right.b);
  EEPROM.commit();
}

//----------------------------------------------------------------------------

void  ESP32Flash_ReadServerInfo()
{
  char      eeprom_ST_ssid[kEEPROM_ST_ssid_size] = {};
  char      eeprom_ST_pass[kEEPROM_ST_pass_size] = {};
  char      eeprom_AP_ssid[kEEPROM_AP_ssid_size] = {};
  char      eeprom_AP_pass[kEEPROM_AP_pass_size] = {};
  static_assert(sizeof(eeprom_ST_ssid) == kEEPROM_ST_ssid_size);
  static_assert(sizeof(eeprom_ST_pass) == kEEPROM_ST_pass_size);
  static_assert(sizeof(eeprom_AP_ssid) == kEEPROM_AP_ssid_size);
  static_assert(sizeof(eeprom_AP_pass) == kEEPROM_AP_pass_size);

  EEPROM.get(kEEPROM_ST_ssid_addr, eeprom_ST_ssid);
  EEPROM.get(kEEPROM_ST_pass_addr, eeprom_ST_pass);
  EEPROM.get(kEEPROM_AP_ssid_addr, eeprom_AP_ssid);
  EEPROM.get(kEEPROM_AP_pass_addr, eeprom_AP_pass);
  eeprom_ST_ssid[sizeof(eeprom_ST_ssid)-1] = '\0';
  eeprom_ST_pass[sizeof(eeprom_ST_pass)-1] = '\0';
  eeprom_AP_ssid[sizeof(eeprom_AP_ssid)-1] = '\0';
  eeprom_AP_pass[sizeof(eeprom_AP_pass)-1] = '\0';

  wifi_ST_SSID = eeprom_ST_ssid;
  wifi_ST_Pass = eeprom_ST_pass;
  if (eeprom_ST_ssid[0] != '\0') // don't check password: allow empty password
    has_wifi_credentials = true;
  if (eeprom_AP_ssid[0] != '\0')
  {
    wifi_AP_SSID = eeprom_AP_ssid;
    wifi_AP_Pass = eeprom_AP_pass;
  }

  // Read state
  EEPROM.get(kEEPROM_mode_addr, state_mode);
  EEPROM.get(kEEPROM_debug_id_addr, state_debug_id);
  EEPROM.get(kEEPROM_bright_addr, state_brightness);
  EEPROM.get(kEEPROM_wave_speed_addr, state_wave_speed);
  EEPROM.get(kEEPROM_wave_min_addr, state_wave_min);
  EEPROM.get(kEEPROM_wave_contrast_addr, state_wave_contrast);
  EEPROM.get(kEEPROM_wave_tiling_addr, state_wave_tiling);
  EEPROM.get(kEEPROM_base_speed_addr, state_base_speed);
  EEPROM.get(kEEPROM_base_min_addr, state_base_min);
  EEPROM.get(kEEPROM_hue_shift_addr, state_hue_shift);
  EEPROM.get(kEEPROM_sat_shift_addr, state_sat_shift);
  EEPROM.get(kEEPROM_hue_speed_addr, state_hue_speed);
  static_assert(sizeof(state_color_top.r) == 1);
  EEPROM.get(kEEPROM_col_top_addr + 0, state_color_top.r);
  EEPROM.get(kEEPROM_col_top_addr + 1, state_color_top.g);
  EEPROM.get(kEEPROM_col_top_addr + 2, state_color_top.b);
  EEPROM.get(kEEPROM_col_bot_addr + 0, state_color_bot.r);
  EEPROM.get(kEEPROM_col_bot_addr + 1, state_color_bot.g);
  EEPROM.get(kEEPROM_col_bot_addr + 2, state_color_bot.b);
  EEPROM.get(kEEPROM_col_left_addr + 0, state_color_left.r);
  EEPROM.get(kEEPROM_col_left_addr + 1, state_color_left.g);
  EEPROM.get(kEEPROM_col_left_addr + 2, state_color_left.b);
  EEPROM.get(kEEPROM_col_right_addr + 0, state_color_right.r);
  EEPROM.get(kEEPROM_col_right_addr + 1, state_color_right.g);
  EEPROM.get(kEEPROM_col_right_addr + 2, state_color_right.b);
}