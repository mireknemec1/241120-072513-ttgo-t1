
#include <WiFi.h>
#include "ThingSpeak.h" // always include thingspeak header file after other header files and custom macros
#include "thermistor.h" // Měření teplotních čidel
#include "HardwareSerial.h"
#include <TFT_eSPI.h> // ESP32 Display
#include <SPI.h>
#include <Arduino.h> // Pro odesílání dat přes remote infra diodu
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <TimeLib.h>
#include <math.h>

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600; // Nastavení časového pásma
const int daylightOffset_sec = 3600;

const uint16_t kIrLed = 26; // Pin IRLED, červený 3,3V, černý GDN
#define Pin_Voda_Vystup 36  // Pin propojit na GDN rezistorem 10k, druhý pin Thermistoru zapojit na 3,3V   žlutý drát
#define Pin_Voda_Vstup 37   // Pin propojit na GDN rezistorem 10k, druhý pin Thermistoru zapojit na 3,3V   modrý drát
#define Pin_Teplota_Doma 38 // Pin propojit na GDN rezistorem 10k, druhý pin Thermistoru zapojit na 3,3V   bílý drát
#define Pin_Prutokomer 33   // Žlutý Hall senzor, Černý__GDN, Červený__ 5V
#define Pin_Preruseni 33
#define Pin_Privod_elektriny 32
#define Pin_Prutokovy_Spinac 27
#define Pin_TFT_Led 4 // Pomocné makro s definicí pinu, kterým zapínám/vypínám podsvícení LCD

IRsend irsend(kIrLed); // Set the GPIO to be used to sending the message

uint16_t rawData_Sinclair_0W[35] = {6004, 3012, 630, 576, 628, 1676, 630, 574, 630, 1676, 630, 1676, 630, 574, 632, 576, 602, 610, 594, 1700, 632, 572, 630, 1654, 652, 576, 628, 576, 632, 1672, 630, 576, 630, 1674, 632};                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   // UNKNOWN CDA2418B
uint16_t rawData_Sinclair_400W[35] = {5982, 3018, 628, 574, 632, 1652, 654, 1676, 602, 606, 600, 1706, 602, 602, 602, 600, 632, 576, 628, 1682, 600, 602, 628, 1680, 628, 576, 630, 580, 602, 1704, 632, 574, 632, 1678, 632};                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 // UNKNOWN F6BD48CF
uint16_t rawData_Sinclair_800W[35] = {6008, 3012, 632, 1680, 610, 592, 632, 1680, 600, 604, 604, 1702, 628, 578, 632, 574, 630, 576, 630, 1676, 632, 576, 604, 1702, 630, 578, 628, 574, 634, 1678, 626, 576, 630, 1676, 632};                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 // UNKNOWN 85963258
uint16_t rawData_Sinclair_1500W[35] = {6036, 3026, 592, 1786, 520, 1682, 628, 1680, 626, 1716, 592, 608, 598, 612, 592, 582, 624, 578, 654, 1692, 590, 580, 630, 1678, 626, 618, 590, 684, 522, 1682, 630, 578, 626, 1764, 544};                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               // UNKNOWN 615C3544
uint16_t rawData_Sinclair_Zapnuto23[279] = {9000, 4478, 638, 566, 640, 578, 628, 1672, 634, 1674, 634, 1674, 636, 568, 662, 542, 638, 568, 638, 1672, 634, 1670, 638, 1674, 634, 578, 628, 570, 638, 574, 632, 570, 636, 574, 632, 570, 634, 570, 636, 572, 634, 568, 640, 568, 638, 1672, 636, 1668, 642, 570, 634, 568, 638, 568, 638, 568, 636, 570, 636, 1672, 634, 568, 636, 1672, 638, 568, 638, 578, 628, 1672, 636, 568, 638, 20012, 636, 572, 634, 576, 628, 572, 634, 570, 636, 568, 638, 568, 638, 570, 636, 570, 636, 582, 624, 572, 634, 1670, 640, 568, 638, 568, 638, 568, 638, 1668, 638, 1668, 640, 568, 638, 572, 634, 570, 638, 568, 662, 544, 636, 574, 632, 572, 634, 572, 634, 572, 634, 570, 636, 568, 636, 572, 636, 1682, 624, 574, 632, 570, 636, 1680, 630, 40086, 8948, 4480, 638, 568, 638, 580, 626, 574, 634, 574, 632, 570, 638, 568, 636, 656, 548, 574, 632, 572, 634, 574, 632, 574, 634, 572, 636, 568, 638, 568, 638, 568, 636, 570, 636, 572, 634, 574, 630, 572, 634, 570, 638, 576, 630, 568, 638, 656, 548, 572, 632, 572, 634, 570, 636, 656, 548, 576, 630, 570, 636, 1676, 632, 572, 636, 1674, 630, 572, 636, 1672, 638, 572, 634, 20010, 638, 568, 636, 572, 636, 578, 628, 578, 628, 572, 636, 578, 628, 572, 634, 570, 636, 576, 630, 572, 658, 542, 640, 576, 630, 580, 624, 584, 624, 572, 634, 576, 630, 570, 638, 572, 634, 572, 634, 626, 580, 572, 634, 570, 634, 570, 634, 578, 628, 578, 630, 568, 638, 570, 636, 570, 638, 568, 636, 1674, 634, 568, 636, 1674, 634}; // GREE
uint16_t rawData_Sinclair_Zapnuto22[279] = {9022, 4474, 640, 582, 624, 572, 634, 1668, 638, 1668, 638, 1670, 634, 570, 636, 574, 630, 568, 638, 574, 630, 1676, 630, 1670, 638, 566, 638, 576, 628, 572, 634, 568, 636, 546, 660, 574, 632, 566, 638, 568, 634, 570, 636, 570, 634, 1678, 626, 1672, 636, 568, 638, 572, 634, 570, 634, 572, 634, 574, 632, 1670, 638, 570, 632, 1668, 636, 572, 632, 568, 636, 1676, 630, 568, 638, 19994, 636, 576, 632, 568, 636, 572, 636, 570, 636, 574, 628, 572, 634, 574, 632, 570, 634, 570, 636, 566, 638, 1672, 634, 574, 632, 574, 628, 574, 632, 1672, 632, 1674, 632, 568, 636, 568, 638, 568, 636, 570, 634, 568, 638, 568, 638, 566, 640, 624, 580, 570, 632, 568, 636, 568, 638, 570, 636, 566, 638, 568, 638, 566, 640, 1670, 634, 40006, 8990, 4476, 636, 572, 632, 568, 638, 576, 628, 570, 638, 568, 636, 568, 636, 572, 632, 570, 636, 570, 634, 570, 636, 570, 636, 570, 636, 568, 636, 576, 628, 580, 626, 658, 546, 574, 634, 566, 640, 572, 632, 568, 634, 576, 630, 572, 634, 572, 632, 570, 634, 572, 632, 548, 660, 566, 640, 570, 634, 570, 634, 1678, 630, 566, 636, 1670, 636, 568, 636, 1674, 632, 576, 628, 19994, 636, 570, 636, 572, 634, 566, 638, 572, 634, 570, 634, 570, 634, 572, 634, 568, 636, 570, 634, 568, 640, 568, 634, 570, 634, 570, 636, 568, 634, 574, 632, 570, 636, 624, 580, 570, 638, 568, 634, 568, 636, 572, 632, 570, 636, 570, 634, 572, 634, 578, 626, 572, 634, 572, 634, 572, 632, 568, 638, 1668, 636, 568, 636, 1672, 634};   // UNKNOWN FC5A8347
uint16_t rawData_Sinclair_Vypnuto[279] = {8996, 4504, 630, 570, 636, 568, 636, 1648, 656, 572, 634, 1670, 636, 572, 634, 570, 634, 578, 628, 1672, 632, 1678, 628, 1670, 636, 570, 638, 568, 634, 576, 628, 572, 634, 570, 636, 656, 546, 570, 636, 570, 636, 654, 550, 572, 632, 1678, 630, 570, 636, 566, 638, 574, 630, 568, 638, 566, 636, 570, 634, 1676, 630, 570, 636, 1670, 636, 570, 638, 566, 638, 1670, 632, 572, 634, 19992, 636, 570, 634, 568, 636, 568, 638, 568, 638, 568, 634, 568, 636, 570, 636, 570, 634, 568, 638, 570, 634, 570, 636, 572, 634, 578, 628, 570, 636, 1724, 580, 1672, 632, 578, 626, 570, 636, 568, 638, 574, 628, 572, 634, 566, 640, 568, 638, 566, 636, 568, 636, 570, 636, 568, 636, 572, 634, 1674, 630, 570, 634, 568, 638, 568, 636, 39996, 8972, 4500, 638, 572, 632, 572, 634, 570, 636, 570, 634, 570, 634, 626, 580, 570, 634, 576, 628, 574, 634, 548, 656, 568, 636, 572, 634, 576, 628, 570, 634, 570, 636, 570, 636, 570, 632, 574, 630, 574, 632, 568, 638, 568, 636, 566, 638, 572, 658, 570, 638, 568, 636, 578, 628, 570, 636, 564, 642, 566, 638, 1672, 634, 570, 634, 1672, 634, 568, 634, 1682, 626, 568, 636, 20000, 628, 574, 630, 570, 636, 568, 636, 572, 634, 570, 634, 570, 636, 572, 632, 568, 636, 582, 624, 570, 634, 568, 636, 568, 636, 570, 636, 578, 628, 568, 638, 568, 634, 570, 634, 576, 630, 570, 636, 570, 634, 568, 638, 570, 636, 658, 546, 570, 636, 572, 634, 572, 632, 570, 636, 566, 638, 570, 636, 1670, 636, 566, 638, 1670, 636};       // UNKNOWN 1171ABD0

THERMISTOR thermistor(Pin_Voda_Vstup, // Analog pin
                      11500,          // Nominal resistance at 25 ºC
                      3950,           // Thermistor's beta coefficient
                      10000);         // Funkce načte teploty na daných pinech
                                      // Funkce načte teploty na daných pinech
THERMISTOR thermistor1(Pin_Voda_Vystup,
                       11600,
                       3950,
                       10000);

THERMISTOR thermistor2(Pin_Teplota_Doma,
                       11600,
                       3950,
                       10000);

TFT_eSPI tft = TFT_eSPI();

uint16_t temp;

const char *ssid = "Xiaomi"; // WiFi
const char *password = "1234q6789";

WiFiClient client;

unsigned long myChannelNumber = 294614;
const char *myWriteAPIKey = "9TAEJGXH9MGJ0CO4";
const char *api_keyRead = "9ZQ2KOCYLPUK4LL7";

struct tm currentTime; // Globální proměnná pro aktuální datum a čas
float Prutok_Litru_Minuta = 0;

const float zakladni_teplota = 22.0; // Základní požadovaná teplota
float Pozadovana_Teplota_Doma = 0.0; // Požadovaná teplota doma ve stupních C po desetinách stupně¨

int Prikon_Zasuvka = 0;
int odesilany_vykon_IR = 0;
int Vykon_W = 0;
float Pomer_Vykonu = 0.0;
float Voda_Vstup = 0;
float Voda_Vystup = 0;
float Teplota_Doma = 0;
float rozdil_400W_800W = 2.0;
float rozdil_Vypnuto_Zapnuto = 1.2;

//============================================================================================================

// Funkce pro připojení k WiFi síti
void pripojeniWifi()
{

  Serial.println("");
  Serial.println("Start funkce pripojeniWifi___________ ");
  Serial.println("");

  Serial.println(ssid);

  WiFi.begin(ssid, password);

  // Čekání na připojení
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi připojeno.");
  Serial.print("IP adresa: ");

  tft.drawString("PRIPOJENO", tft.width() / 2, 10);               // Přidal jsem Y souřadnici pro správné umístění
  tft.drawString(WiFi.localIP().toString(), tft.width() / 2, 50); // IP adresa převedena na String pomocí toString()

  // Nastavení NTP a načtení aktuálního času
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  Serial.println("Čekání na získání času z NTP...");
  while (!getLocalTime(&currentTime))
  {
    Serial.println("Čekám na čas...");
    tft.drawString("Cekam na cas", tft.width() / 2, 100);
    delay(1000);
  }

  Serial.println("Aktuální čas načten:");
}

//===================================================================================================

// Funkce pro čtení a průměrování hodnot ze senzorů
void cteniDatSenzoru()
{

  Serial.println("");
  Serial.println("Start funkce cteniDatSenzoru___________ ");
  Serial.println("");

  static int cyklus = 0; // sleduje počet opakování programu od spuštění
  cyklus++;

  // ===== Průměrování Voda_Vstup 3x =====
  static float Voda_Vstup_Sum = 0.0;
  Voda_Vstup_Sum += (thermistor.read() / 10.0);
  if (cyklus > 3)
  {
    Voda_Vstup_Sum -= Voda_Vstup;
  }
  Voda_Vstup = (Voda_Vstup_Sum / min(cyklus, 3));
  Serial.println("Voda vstup: " + String(Voda_Vstup));
  delay(100);

  // ===== Průměrování Voda_Vystup 3x =====
  static float Voda_Vystup_Sum = 0.0;
  Voda_Vystup_Sum += (thermistor1.read() / 10.0);
  if (cyklus > 3)
  {
    Voda_Vystup_Sum -= Voda_Vystup;
  }
  Voda_Vystup = (Voda_Vystup_Sum / min(cyklus, 3));
  Serial.println("Voda vystup: " + String(Voda_Vystup));
  delay(100);

  // ===== Průměrování Teplota_Doma  =====
  static float Teplota_Doma_Sum = 0.0;
  Teplota_Doma_Sum += (thermistor2.read() / 10.0);
  if (cyklus > 80)
  {
    Teplota_Doma_Sum -= Teplota_Doma;
  }
  Teplota_Doma = (Teplota_Doma_Sum / min(cyklus, 80));
  Serial.println("Teplota doma: " + String(Teplota_Doma));
  delay(100);

  // Kontrola stavu pinu před měřením průtoku
  if (digitalRead(Pin_Privod_elektriny) == HIGH)
  {
    // Měření průtoku s kontrolními měřeními
    int pokusy = 0;
    Serial.println("Startuji měření průtoku: ");
    do
    {
      unsigned long startTime = millis();
      int flowCount = 0;
      while (millis() - startTime < 1000)
      { // měření po dobu 1 sekundy
        if (digitalRead(Pin_Prutokomer) == HIGH)
        {
          flowCount++;
          while (digitalRead(Pin_Prutokomer) == HIGH)
            ; // Čeká na LOW, aby nedošlo k opakovanému počítání
        }
      }
      Prutok_Litru_Minuta = flowCount / 8.0;
      pokusy++;
    } while (Prutok_Litru_Minuta < 5 && pokusy < 5);

    Serial.print("Průtok v litrech za minutu: ");
    Serial.println(Prutok_Litru_Minuta, 1); // Výpis s jednou desetinnou čárkou
  }
  else
  {
    Prutok_Litru_Minuta = 0;
    Serial.println("Prutokomer je LOW, měření průtoku přeskočeno.");
  }

  Vykon_W = (((Voda_Vystup - Voda_Vstup) * Prutok_Litru_Minuta) / 13.2) * 1000;
  Serial.println("Vykon: " + String(Vykon_W));

  // ===== Aktualizace displeje =====
  tft.fillScreen(TFT_RED);
  tft.drawString("VYSTUP " + String(Voda_Vystup) + "C ", tft.width() / 2, tft.height() / 2);
  tft.drawString("DOMA " + String(Teplota_Doma) + "C ", tft.width() / 2, tft.height() / 4);
  tft.drawString(String(Vykon_W) + " W  " + String(Prutok_Litru_Minuta, 1), tft.width() / 2, 100);
}

//==============================================================================================

void regulaceTeploty()
{
  Serial.println("");
  Serial.println("Start funkce regulaceTeploty___________ ");
  Serial.println("");

  // Resetování Pozadovana_Teplota_Doma na základní hodnotu před úpravami
  Pozadovana_Teplota_Doma = zakladni_teplota;

  // Deklarace statických proměnných na úrovni funkce
  static float rozdil_pocatecni = 0.0;
  static bool rozdil_pocatecni_nastaven = false;
  static bool first_run = true; // Přidaná statická proměnná pro detekci prvního spuštění

  // Získání aktuálního času z NTP serveru
  struct tm timeinfo;
  if (getLocalTime(&timeinfo))
  {                              // Pokud je získání času úspěšné
    int hour = timeinfo.tm_hour; // Extrahování aktuální hodiny

    // Nastavení požadované teploty doma na základě aktuálního času
    if (hour >= 19 && hour <= 23)
    {
      // Od 19:00 do 23:00 snižujeme teplotu o 0,2°C každou hodinu
      float adjustment = -0.2 * (hour - 18); // Celkem snížení až o 1,0°C
      Pozadovana_Teplota_Doma += adjustment;
    }
    else if (hour >= 0 && hour < 8)
    {
      // Od 0:00 do 8:00 udržujeme teplotu sníženou o 1,0°C
      Pozadovana_Teplota_Doma -= 1.0;
    }
    else if (hour >= 9 && hour <= 12)
    {
      // Od 9:00 do 12:00 zvyšujeme teplotu o 0,15°C každou hodinu
      float adjustment = 0.15 * (hour - 8); // Zvyšujeme o 0,15°C každou hodinu
      Pozadovana_Teplota_Doma += adjustment;
    }
    // V ostatních hodinách zůstává požadovaná teplota základní

    // Úprava regulace mezi 13:00 a 18:00
    if (hour >= 13 && hour < 18)
    {
      if (!rozdil_pocatecni_nastaven)
      {
        rozdil_pocatecni = Pozadovana_Teplota_Doma - Teplota_Doma;
        rozdil_pocatecni_nastaven = true;
        Serial.println("");
        Serial.print("Uložený počáteční rozdíl v čase 13-18.00 hod: ");
        Serial.println(rozdil_pocatecni);
        Serial.println("");
      }
      // Nastavení požadované teploty tak, aby rozdíl zůstal konstantní
      Pozadovana_Teplota_Doma = Teplota_Doma + rozdil_pocatecni;
    }
    else
    {
      // Mimo časový úsek 13:00 - 18:00 resetujeme příznak
      rozdil_pocatecni_nastaven = false;
    }
  }

  Serial.println("________________________________");
  Serial.print("Teplota_Doma: ");
  Serial.println(Teplota_Doma);
  Serial.print("Pozadovana_Teplota_Doma: ");
  Serial.println(Pozadovana_Teplota_Doma);

  // Normální regulace teploty
  if (Teplota_Doma >= Pozadovana_Teplota_Doma)
  {
    vypnutiZarizeni();
  }
  else if (Teplota_Doma <= (Pozadovana_Teplota_Doma - (first_run ? 0 : rozdil_Vypnuto_Zapnuto)))
  {
    // Teplota klesla o více než "rozdil_Vypnuto_Zapnuto" pod požadovanou hodnotu
    if (digitalRead(Pin_Privod_elektriny) == LOW)
    {
      Serial.println("Funkce regulaceTeploty: Teplota_Doma <= (Pozadovana_Teplota_Doma - rozdil_Vypnuto_Zapnuto), voláme inicializaceZapnuti");
      inicializaceZapnuti();
    }
    int vykon = ((Pozadovana_Teplota_Doma - Teplota_Doma) <= rozdil_400W_800W) ? 400 : 800;
    if ((Pozadovana_Teplota_Doma - Teplota_Doma) <= rozdil_400W_800W)
    {
      irsend.sendRaw(rawData_Sinclair_400W, 35, 38);
      odesilany_vykon_IR = 400;
      Serial.println("Přepínám na 400W");
    }
    else
    {
      irsend.sendRaw(rawData_Sinclair_800W, 35, 38);
      odesilany_vykon_IR = 800;
      Serial.println("Přepínám na 800W");
    }
  }
  else if (digitalRead(Pin_Privod_elektriny) == HIGH)
  {
    // Pokud je zařízení zapnuté, upravujeme výkon podle rozdílu teplot
    int vykon = ((Pozadovana_Teplota_Doma - Teplota_Doma) <= rozdil_400W_800W) ? 400 : 800;
    if ((Pozadovana_Teplota_Doma - Teplota_Doma) <= rozdil_400W_800W)
    {
      irsend.sendRaw(rawData_Sinclair_400W, 35, 38);
      odesilany_vykon_IR = 400;
      Serial.println("Přepínám na 400W");
    }
    else
    {
      irsend.sendRaw(rawData_Sinclair_800W, 35, 38);
      odesilany_vykon_IR = 800;
      Serial.println("Přepínám na 800W");
    }
  }

  if (first_run)
  {
    first_run = false;
  }

  Serial.println("________________________________");
  Serial.print("Teplota_Doma: ");
  Serial.println(Teplota_Doma);
  Serial.print("Pozadovana_Teplota_Doma: ");
  Serial.println(Pozadovana_Teplota_Doma);
  Serial.print("rozdil_400W_800W: ");
  Serial.println(rozdil_400W_800W);
  Serial.print("rozdil_Vypnuto_Zapnuto: ");
  Serial.println(rozdil_Vypnuto_Zapnuto);
}

//====================================================================================================

void vypnutiZarizeni()
{
  // Vypneme zařízení a nastavíme příznak vypnuto
  if (digitalRead(Pin_Privod_elektriny) == HIGH)
  {
    irsend.sendRaw(rawData_Sinclair_0W, 35, 38);
    odesilany_vykon_IR = 0;
    Serial.println("Odesláno z funkce vypnutiZarizeni rawData_Sinclair_0W ");
    delay(2000);
    irsend.sendRaw(rawData_Sinclair_0W, 35, 38);
    delay(20000);
    irsend.sendRaw(rawData_Sinclair_0W, 35, 38);
    delay(90000);
    digitalWrite(Pin_Privod_elektriny, LOW);
    Serial.println("Odesláno z funkce vypnutiZarizeni Pin_Privod_elektriny, LOW");
  }
}

//============================================================================================

void sendToThingSpeak()
{
  // Nastavení hodnot do jednotlivých polí (fields)
  ThingSpeak.setField(1, Vykon_W);                 // Field 1: Vykon_W
  ThingSpeak.setField(2, Prutok_Litru_Minuta);     // Field 2: Prutok_Litru_Minuta
  ThingSpeak.setField(3, Voda_Vstup);              // Field 3: Voda_Vstup
  ThingSpeak.setField(4, Voda_Vystup);             // Field 4: Voda_Vystup
  ThingSpeak.setField(5, Teplota_Doma);            // Field 5: Teplota_Doma
  ThingSpeak.setField(6, Pozadovana_Teplota_Doma); // Field 6: Pozadovana_Teplota_Doma
  ThingSpeak.setField(7, odesilany_vykon_IR);      // Field 7: Prikon_Zasuvka
  if (Pomer_Vykonu != 0)
  {                                       // Pokud je Pomer_Vykonu nenulový
    ThingSpeak.setField(8, Pomer_Vykonu); // Field 8: Pomer_Vykonu
  }

  // Odeslání dat na ThingSpeak
  int responseCode = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

  // Kontrola odpovědi
  if (responseCode == 200)
  {
    Serial.println("Channel write successful.");
  }
  else
  {
    Serial.println("Problem writing to channel. HTTP error code " + String(responseCode));
  }
}

//============================================================================================================

void kontrolaBezpecnosti()
{

  Serial.println("");
  Serial.println("Start funkce kontrolaBezpecnosti___________ ");
  Serial.println("");

  if (Voda_Vystup >= 52 || Voda_Vstup <= 10 || (digitalRead(Pin_Privod_elektriny) == HIGH && Prutok_Litru_Minuta < 5))
  {
    tft.fillScreen(TFT_BLACK);
    if (Voda_Vystup >= 52)
    {
      Serial.println("Voda_Vystup >= 52");
      tft.drawString("VODA>52 C", tft.width() / 2, 100);
    }
    if (Voda_Vstup <= 10)
    {
      Serial.println("Voda_Vstup <= 10 C");
      tft.drawString("Voda<10 C", tft.width() / 2, 100);
    }
    if (digitalRead(Pin_Privod_elektriny) == HIGH && Prutok_Litru_Minuta < 5)
    {
      Serial.println("Prutok < 5");
      Serial.print("Průtok v litrech za minutu: ");
      Serial.println(Prutok_Litru_Minuta, 1);
      tft.drawString("PRUTOK<5", tft.width() / 2, 100);
    }
    delay(1000);
    odesilany_vykon_IR = 0;

    for (int i = 0; i < 5; i++)
    {
      irsend.sendRaw(rawData_Sinclair_Vypnuto, 279, 38);
      delay(2000);
    }

    Serial.println("Odesláno z funkce kontrolaBezpecnosti rawData_Sinclair_Vypnuto a čekám 110s");
    delay(110000);

    digitalWrite(Pin_Privod_elektriny, LOW);
    Serial.println("Odesláno z funkce kontrolaBezpecnosti Pin_Privod_elektriny, LOW");
  }
}

//===============================================================================

void inicializaceZapnuti()
{

  Serial.println("");
  Serial.println("Start funkce inicializaceZapnuti___________ ");
  Serial.println("");

  static int pokus = 0; // Přidáno pro sledování počtu pokusů
  digitalWrite(Pin_Privod_elektriny, HIGH);
  delay(4000);
  irsend.sendRaw(rawData_Sinclair_Zapnuto22, 279, 38);
  delay(2000);
  irsend.sendRaw(rawData_Sinclair_Zapnuto22, 279, 38);
  delay(2000);
  Serial.println("________________________________");
  Serial.println("Odesláno z funkce inicializaceZapnuti Pin_Privod_elektriny, HIGH");
  for (int i = 0; i < 40; i++)
  {
    irsend.sendRaw(rawData_Sinclair_1500W, 35, 38);
    odesilany_vykon_IR = 1500;
    Serial.println("________________________________");
    Serial.println("Odesláno z funkce inicializaceZapnuti rawData_Sinclair_1500W");
    Serial.println(i);
    cteniDatSenzoru();
    kontrolaBezpecnosti();
    sendToThingSpeak();
    delay(15000);
  }

  // Kontrola výkonu a zda počet pokusů o znovu inicializaci není větší než 3
  if (Vykon_W < 1300)
  {
    tft.drawString("Vykon<1300", tft.width() / 2, 100);
    odesilany_vykon_IR = 0;

    Serial.println("Výkon po inicializaceZapnuti je menší než 1300W Odesláno rawData_Sinclair_Vypnuto");
    irsend.sendRaw(rawData_Sinclair_Vypnuto, 279, 38);
    delay(2000);
    irsend.sendRaw(rawData_Sinclair_Vypnuto, 279, 38);
    delay(2000);

    delay(120000);
    digitalWrite(Pin_Privod_elektriny, LOW);
    delay(20000);

    pokus++;
    if (pokus < 3)
    { // Kontrola, zda počet pokusů není větší než 3
      inicializaceZapnuti();
    }
    else
    {
      Serial.println("Maximální počet 3 pokusů dosažen. Nedostatečný výkon. Funkce inicializaceZapnuti se ukončuje.");
    }
  }
}

//===================================================================================================
//===================================================================================================

void setup(void)
{

  Serial.println("");
  Serial.println("START funkce Setup ");
  Serial.println("");

  pinMode(Pin_Privod_elektriny, OUTPUT);
  pinMode(Pin_Prutokomer, INPUT);
  pinMode(Pin_TFT_Led, OUTPUT);

  digitalWrite(Pin_Privod_elektriny, HIGH); // Spustí proud do vodního i tepelného čerpadla

#if ESP8266
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY); // Nutné pro IR odesílání RAW dat
#else                                               // ESP8266
  Serial.begin(115200, SERIAL_8N1); // Ostatní desky
#endif

  delay(3000);

  irsend.begin();

  cteniDatSenzoru();
  kontrolaBezpecnosti();

  digitalWrite(Pin_Privod_elektriny, LOW);

  tft.init();
  tft.setRotation(1); // Orientace na šířku
  tft.setTextSize(3); // Nastavení velikosti a geometrie písma
  tft.setTextDatum(MC_DATUM);
  digitalWrite(Pin_TFT_Led, HIGH); // Zapnutí podsvícení displeje
  tft.setTextColor(TFT_WHITE);     // Barva písma
  tft.fillScreen(TFT_RED);         // Barva pozadí

  WiFi.mode(WIFI_STA); // WIFI_AP_STA: Kombinace režimů STA a AP, což umožňuje zařízení být současně klientem jiné sítě a zároveň vytvářet vlastní síť

  pripojeniWifi();

  ThingSpeak.begin(client); // Initialize ThingSpeak

  Serial.println("");
  Serial.print("KONEC funkce Setup ");
  Serial.println("");
}

//=================================================================================================
//=================================================================================================

void loop()
{

  Serial.println("");
  Serial.println("START funkce loop ");
  Serial.println("");

  cteniDatSenzoru();
  kontrolaBezpecnosti();
  regulaceTeploty();
  sendToThingSpeak();

  delay(15000);
}
