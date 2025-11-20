#include <Wire.h>
#include <AccelStepper.h>  
#include <SPI.h>  
#include <MFRC522.h>  
#include <LiquidCrystal.h>  
#define EN_PIN_X    5  
#define DIR_PIN_X   6  
#define STEP_PIN_X  7  
#define SERIAL_TX_X 16  
#define SERIAL_RX_X 17  
#define EN_PIN_Y    2  
#define DIR_PIN_Y   3  
#define STEP_PIN_Y  4  
#define SERIAL_TX_Y 18  
#define SERIAL_RX_Y 19  
#define EN_PIN_Z    11  
#define DIR_PIN_Z   12  
#define STEP_PIN_Z  13  
#define SERIAL_TX_Z 14  
#define SERIAL_RX_Z 15  
// Parametry sterowników  
#define DRIVER_ADDRESS_X 0b00  
#define DRIVER_ADDRESS_Y 0b01  
#define DRIVER_ADDRESS_Z 0b10  
#define R_SENSE 0.11  
//Definicja pinu krańcówek  
#define K1_PIN 30  // KRANCOWKA X LEWA  
#define K3_PIN 32  // KRANCOWKA Y TYL  
#define K4_PIN 33  // KRANCOWKA Y PRZOD  
#define K5_PIN 34  // KRANCOWKA Z DOL  
// Definicja RC522
#define RST_PIN 8 // Podłącz RST do D5  
#define SS_PIN 53 // Podłącz SDA (SS) do D53  
#define BUZZ_PIN 22 // Buzzer  
// Piny dla TTP229 
#define SCL 35 
#define SDO 36 
// Piny dla dodatkowych wyjść OUT1 do OUT5 
#define OUT1 37 
#define OUT2 38 
#define OUT3 39 
#define OUT4 40 
#define OUT5 41 
LiquidCrystal lcd(23, 24, 25, 26, 27, 28);  
int selectedShelf = 0; 
MFRC522 rfid(SS_PIN, RST_PIN); 
unsigned long previousRFIDMillis = 0;  
const long rfidInterval = 2000;          
bool paleta1wykryta = false;  
bool paleta2wykryta = false;  
bool paleta3wykryta = false;  
bool paleta4wykryta = false;  
bool paleta5wykryta = false;  
bool kartaOdbioru = false;          
unsigned long lastDebounceTime = 0; 
const unsigned long debounceDelay = 100; 
AccelStepper stepperX(AccelStepper::DRIVER, STEP_PIN_X, DIR_PIN_X);  
AccelStepper stepperY(AccelStepper::DRIVER, STEP_PIN_Y, DIR_PIN_Y);  
AccelStepper stepperZ(AccelStepper::DRIVER, STEP_PIN_Z, DIR_PIN_Z);  
bool movementInProgress = false;  
bool rfidDetected = false; 
long currentPosition_X = stepperX.currentPosition();  
long currentPosition_Y = stepperY.currentPosition();  
long currentPosition_Z = stepperZ.currentPosition();  
unsigned long currentMillis = 0;  
int sequenceStep = 0; 
int Odk_step = 0;  
bool Odk = false;  
bool odkladanieZakonczone = false;  
bool ODB1 = false, ODB2 = false, ODB3 = false, ODB4 = false; 
bool ODB5 = false, ODB6 = false, ODB7 = false, ODB8 = false; 
bool move_end = false;  
bool homeFinished = false;  
int processState = 0;  
bool readStop = false; 
bool messageDisplayed = false;
bool ignoreFirstKeyPress = true;
bool pobranieZakonczone = false; 

void homeAxis(AccelStepper &stepper, int limitSwitchPin, int direction) 
{ 
  stepper.setMaxSpeed(1200); 
  stepper.setAcceleration(800);  
  stepper.moveTo(direction * 26000);    
    while (digitalRead(limitSwitchPin) == HIGH) 
    {   
        stepper.run();   
    }  
  stepper.stop();   
  stepper.setCurrentPosition(0);   
}  
  
byte targetUID[] = { 0x56, 0xCA, 0x77, 0x3D}; // biala karta UID  
byte paleta1UID[] = {0xD2, 0x68, 0x15, 0x10}; // Paleta 1  
byte paleta2UID[] = {0xE2, 0x90, 0x09, 0x10}; // Paleta 2  
byte paleta3UID[] = {0x96, 0x1E, 0xFB, 0xFF}; // Paleta 3  
byte paleta4UID[] = {0x02, 0x96, 0x12, 0x10}; // Paleta 4  
byte paleta5UID[] = {0x6E, 0x78, 0x07, 0x90}; // Paleta 5  
  
bool rfidMatchesUID(byte *readUID, byte *targetUID) 
{  
    for (byte i = 0; i < 4; i++) 
    {  
        if (readUID[i] != targetUID[i]) 
        {  
            return false;  
        }  
    }  
    return true;  
}  
 
void displayMessage(const char* line1, const char* line2 = "") 
{
  lcd.clear();            
  lcd.begin(16, 2);       
  delay(50);              
  lcd.setCursor(0, 0);  
  lcd.print(line1);     
  lcd.setCursor(0, 1);  
  lcd.print(line2);    
}

 
void resetFlagsP() 
{ 
  ODB1 = false; 
  ODB2 = false; 
  ODB3 = false; 
  ODB4 = false; 
  ODB5 = false; 
  ODB6 = false; 
  ODB7 = false; 
} 
 
void buzzer() 
{ 
  tone(BUZZ_PIN, 1800);  // Włącza dźwięk na buzzerze o częstotliwości 1000 Hz 
  delay(50);            // Dźwięk będzie trwał 100 ms 
  noTone(BUZZ_PIN); 
  delay(300);  
  tone(BUZZ_PIN, 1800);  // Włącza dźwięk na buzzerze o częstotliwości 1000 Hz 
  delay(100);            // Dźwięk będzie trwał 100 ms 
  noTone(BUZZ_PIN);    // Wyłącza dźwięk 
} 
void buzzer2() 
{ 
  tone(BUZZ_PIN, 1800);  // Włącza dźwięk na buzzerze o częstotliwości 1000 Hz 
  delay(150);            // Dźwięk będzie trwał 100 ms 
  noTone(BUZZ_PIN);  
  delay(300); 
  tone(BUZZ_PIN, 1800);  // Włącza dźwięk na buzzerze o częstotliwości 1000 Hz 
  delay(100);            // Dźwięk będzie trwał 100 ms 
  noTone(BUZZ_PIN);   // Wyłącza dźwięk 
} 

void buzzer3() 
{ 
  tone(BUZZ_PIN, 2000);  // Włącza dźwięk na buzzerze o częstotliwości 1000 Hz 
  delay(50);            // Dźwięk będzie trwał 100 ms 
  noTone(BUZZ_PIN);  
  delay(100); 
  tone(BUZZ_PIN, 2000);  // Włącza dźwięk na buzzerze o częstotliwości 1000 Hz 
  delay(50);            // Dźwięk będzie trwał 100 ms 
  noTone(BUZZ_PIN);   // Wyłącza dźwięk 
  delay(100); 
  tone(BUZZ_PIN, 2000);  // Włącza dźwięk na buzzerze o częstotliwości 1000 Hz 
  delay(50);            // Dźwięk będzie trwał 100 ms 
  noTone(BUZZ_PIN);   // Wyłącza dźwięk 
} 
 
void readRFID() 
{ 
  currentMillis = millis(); 
  if (!rfidDetected && !movementInProgress) { 
      if (currentMillis - previousRFIDMillis >= rfidInterval) { 
          previousRFIDMillis = currentMillis; 
          if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) 
          { 
            if (rfidMatchesUID(rfid.uid.uidByte, targetUID)) 
            { 
              kartaOdbioru = true; 
              readStop = true; 
              buzzer(); 
              displayMessage("TRYB ODBIORU:","ON"); 
              delay(1500);
              displayMessage("POZYCJA HOME","TRYB ODBIERANIA");
            } 
            else if (rfidMatchesUID(rfid.uid.uidByte, paleta1UID)) 
            { 
              rfidDetected = true; 
              paleta1wykryta = true; 
              displayMessage("PALETA 1", "REGAL 5"); 
            } 
            else if (rfidMatchesUID(rfid.uid.uidByte, paleta2UID)) 
            { 
              rfidDetected = true; 
              paleta2wykryta = true; 
              displayMessage("PALETA 2 ", "REGAL 6"); 
            } 
            else if (rfidMatchesUID(rfid.uid.uidByte, paleta3UID)) 
            { 
              rfidDetected = true; 
              paleta3wykryta = true; 
              displayMessage("PALETA 3", "REGAL 7");
            } 
            else if (rfidMatchesUID(rfid.uid.uidByte, paleta4UID)) 
            { 
              rfidDetected = true; 
              paleta4wykryta = true; 
              displayMessage("PALETA 4", "REGAL 8"); 
            } 
            else if (rfidMatchesUID(rfid.uid.uidByte, paleta5UID)) 
            { 
              rfidDetected = true; 
              paleta5wykryta = true; 
              displayMessage("PALETA 5", "REGAL 9");
            } 
            rfid.PICC_HaltA();    // Zatrzymaj odczyt karty 
            rfid.PCD_StopCrypto1(); // Zatrzymaj komunikację 
          } 
        } 
    } 
} 
 
int readTTP229() {
  static bool ignoreFirstKeyPress = true; // Flaga do ignorowania pierwszego przycisku
  for (int i = 1; i <= 8; i++) 
  {
    digitalWrite(SCL, LOW);
    delayMicroseconds(5);

    if (digitalRead(SDO) == LOW) 
    {
      digitalWrite(SCL, HIGH);
      
      if (ignoreFirstKeyPress) 
      { 
        ignoreFirstKeyPress = false; 
        return 7; 
      }
      return i;  // Zwróć numer przycisku
    }
    digitalWrite(SCL, HIGH);
    delayMicroseconds(5);
  }
  return 0;  // Brak wciśniętego przycisku
}

void setup() 
{  
  Serial.begin(9600);  
  SPI.begin();           
  rfid.PCD_Init();       
  pinMode(BUZZ_PIN, OUTPUT); 
  pinMode(K1_PIN, INPUT_PULLUP);  
  pinMode(EN_PIN_X, OUTPUT);  
  digitalWrite(EN_PIN_X, LOW); // Aktywacja sterowników (LOW = aktywny dla TMC2209)  
  pinMode(EN_PIN_Y, OUTPUT);  
  digitalWrite(EN_PIN_Y, LOW); // Aktywacja sterowników (LOW = aktywny dla TMC2209)  
  pinMode(EN_PIN_Z, OUTPUT);  
  digitalWrite(EN_PIN_Z, LOW); // Aktywacja sterowników (LOW = aktywny dla TMC2209)  

   // Ustawienie pinów dla TTP229 
  pinMode(SCL, OUTPUT); 
  pinMode(SDO, INPUT); 
  lcd.begin(16, 2);
  Wire.setClock(10000);  
  lcd.print("Inicjalizacja...");  
  delay(2000);  
  lcd.setCursor(0, 0);
  lcd.print("                ");  
  lcd.setCursor(0, 0);
  lcd.print("Pozycjonowanie...");

  homeAxis(stepperY, K3_PIN, 1); 
  homeAxis(stepperX, K1_PIN, 1); // Oś Y do krańcówki K3  
  homeAxis(stepperZ, K5_PIN, -1); // Oś Z do krańcówki K5  
  buzzer3();
  homeFinished = true;  

  displayMessage("Pozycjonowanie Zakonczone", "Prosze czekac..");
  delay(1000);
  displayMessage("POZYCJA HOME","TRYB ODKLADANIA");
}   
 
void Pobranie_RUCH1()
{  
  if (!pobranieZakonczone) 
    { 
    switch (sequenceStep) 
      {  
      case 0: 
          stepperY.setMaxSpeed(10000);  
          stepperY.setAcceleration(8000);  
          stepperZ.setMaxSpeed(10000);  
          stepperZ.setAcceleration(8000);  
          stepperY.moveTo(-4300);  
          stepperZ.moveTo(stepperZ.currentPosition() - 40);  
          sequenceStep = 1;  
          break;  

      case 1: 
          if (stepperY.distanceToGo() == 0) 
          { 
            stepperZ.moveTo(stepperZ.currentPosition() + 400); 
            sequenceStep = 2; 
          }  
          break;  

      case 2: 
          if (stepperZ.distanceToGo() == 0) 
          {  
            stepperY.moveTo(-2500);
            sequenceStep = 3; 
          }  
          break;  

      case 3: 
          if (stepperY.distanceToGo() == 0) 
          {   
            stepperZ.moveTo(stepperZ.currentPosition() - 400);  
            sequenceStep = 4; 
          }  
          break;  

      case 4: 
          if (stepperZ.distanceToGo() == 0) 
          {   
            stepperY.moveTo(-4300); 
            sequenceStep = 5; 
          }  
          break;  

      case 5:  
          if (stepperY.distanceToGo() == 0) 
          {   
            stepperZ.moveTo(stepperZ.currentPosition() + 400); 
            sequenceStep = 6; 
          }  
          break;  

      case 6: 
          if (stepperZ.distanceToGo() == 0) 
          {   
            stepperY.moveTo(0);  
            sequenceStep = 7; 
          }  
          break;  

      case 7: 
          if (stepperY.distanceToGo() == 0) 
          {   
            pobranieZakonczone = true; 
            sequenceStep = 0;  
          }  
          break;  
    }  
  }  
}  
  
void Pobranie_RUCH2(){  
     if (!pobranieZakonczone) 
     { 
        switch (sequenceStep) 
        {  

        case 0: // Ruch osi Y na pozycję -4300  
            stepperY.setMaxSpeed(10000);  
            stepperY.setAcceleration(8000);  
            stepperY.moveTo(-4300);  
            stepperZ.setMaxSpeed(10000);  
            stepperZ.setAcceleration(8000);  
            stepperZ.moveTo(stepperZ.currentPosition() - 380);  
            sequenceStep = 1; 
            break;  

        case 1: // Sprawdzanie, czy oś Y dotarła na miejsce  
            if (stepperY.distanceToGo() == 0) 
            { 
              stepperZ.moveTo(stepperZ.currentPosition() + 700); 
              sequenceStep = 2;  
            }  
            break;  

        case 2: // Sprawdzanie ruchu osi Z  
            if (stepperZ.distanceToGo() == 0) 
            { 
              stepperY.moveTo(-2200); 
              sequenceStep = 3; 
            }  
            break;  

        case 3: // Sprawdzanie pozycji osi Y  
            if (stepperY.distanceToGo() == 0) 
            {   
              stepperZ.moveTo(stepperZ.currentPosition() - 700); 
              sequenceStep = 4; 
            }  
            break;  

        case 4: // Sprawdzanie pozycji osi Z  
            if (stepperZ.distanceToGo() == 0) 
            {   
              stepperY.moveTo(-4300); 
              sequenceStep = 5;  
            }  
            break;  

        case 5: // Powrót osi Z na pozycję początkową  
            if (stepperY.distanceToGo() == 0) 
            {   
              stepperZ.moveTo(stepperZ.currentPosition() + 700); 
              sequenceStep = 6; 
            }  
            break;  

        case 6: // Powrót osi Y na pozycję początkową  
            if (stepperZ.distanceToGo() == 0) 
            {   
              stepperY.moveTo(0); 
              sequenceStep = 7; 
            }  
            break;  

        case 7: // Sekwencja zakończona  
            if (stepperY.distanceToGo() == 0) 
            {   
              pobranieZakonczone = true; // Ustawienie flagi zakończenia pobierania  
              move_end = false;  
              sequenceStep = 0;  
            }  
            break;  
        }  
    }  
}  
  
void Odlozenie_RUCH1()
{  
  if (!odkladanieZakonczone) 
  { 
    switch (Odk_step) 
    {  
    case 0: // Ruch osi Y na pozycję -4300  
        stepperY.setMaxSpeed(10000);  
        stepperY.setAcceleration(8000);  
        stepperY.moveTo(-4300);  
        Odk_step = 1; 
        break;  

    case 1: // Sprawdzanie, czy oś Y dotarła na miejsce  
        if (stepperY.distanceToGo() == 0) 
        { 
          stepperZ.setMaxSpeed(10000);  
          stepperZ.setAcceleration(8000);  
          stepperZ.moveTo(stepperZ.currentPosition() - 600); 
          Odk_step = 2; 
        }  
        break;  

    case 2: // Sprawdzanie ruchu osi Z  
        if (stepperZ.distanceToGo() == 0) 
        { 
          stepperY.moveTo(-3000); //   
          Odk_step = 3; 
        }  
        break;  

    case 3: // Sprawdzanie pozycji osi Y  
        if (stepperY.distanceToGo() == 0) 
        {   
          stepperZ.moveTo(stepperZ.currentPosition() + 600);  
          Odk_step = 4;  
        }  
        break;  

    case 4: // Sprawdzanie pozycji osi Z  
        if (stepperZ.distanceToGo() == 0) 
        {   
          stepperY.moveTo(-4300); 
          Odk_step = 5; 
        }  
        break;  

    case 5: // Powrót osi Z na pozycję początkową  
        if (stepperY.distanceToGo() == 0) 
        {   
          stepperZ.moveTo(stepperZ.currentPosition() - 600); 
          Odk_step = 6; 
        }  
        break;  

    case 6: // Powrót osi Y na pozycję początkową  
        if (stepperZ.distanceToGo() == 0) 
        {   
          stepperY.moveTo(0); 
          Odk_step = 7; 
        }  
        break;  

    case 7: // Sekwencja zakończona  
        if (stepperY.distanceToGo() == 0) 
        {   
          odkladanieZakonczone = true; 
          Odk_step = 0;  
        }  
        break;  
    }  
  }  
}    
  
void Odlozenie_RUCH2() {      
    if(move_end){  
    if (!odkladanieZakonczone) {   
        switch (Odk_step) {  
            case 0:  
                stepperY.setMaxSpeed(10000);  
                stepperY.setAcceleration(8000);  
                stepperY.moveTo(-4300); 
                stepperZ.setMaxSpeed(10000);  
                stepperZ.setAcceleration(8000);  
                stepperZ.moveTo(stepperZ.currentPosition() + 240); 
                Odk_step = 1;  
                break;  
  
            case 1:   
                if (stepperY.distanceToGo() == 0) { 
                    stepperZ.moveTo(stepperZ.currentPosition() - 260); 
                    Odk_step = 2;  
                }  
                break;  
  
            case 2:  
                if (stepperZ.distanceToGo() == 0) { 
                    stepperY.moveTo(-2500); 
                    Odk_step = 3; 
                }  
                break;  
  
            case 3:  
                if (stepperY.distanceToGo() == 0) {  
                    stepperZ.moveTo(stepperZ.currentPosition() + 260);   
                    Odk_step = 4;   
                }  
                break;  
  
            case 4:  
                if (stepperZ.distanceToGo() == 0) { 
                    stepperY.moveTo(-4300);   
                    Odk_step = 5; 
                }  
                break;  
  
            case 5:  
                if (stepperY.distanceToGo() == 0) { 
                    stepperZ.moveTo(stepperZ.currentPosition() - 260); 
                    Odk_step = 6;  
                }  
                break;  
  
            case 6:  
                if (stepperZ.distanceToGo() == 0) { 
                    stepperY.moveTo(0);  
                    Odk_step = 7;   
                }  
                break;  
  
            case 7:  
                if (stepperY.distanceToGo() == 0) {   
                    odkladanieZakonczone = true;   
                    move_end = false;   
                    Odk_step = 0;  
                }  
                break;  
        }  
    }  
  }  
}  
  
void HOME(AccelStepper &stepper, int limitSwitchPin, int direction) 
{ 
  stepper.setMaxSpeed(2000); 
  stepper.setAcceleration(1800); 
  stepper.moveTo(direction * 26000); 
    
  while (digitalRead(limitSwitchPin) == HIGH) 
  { 
    stepper.run();  
  }  
    
  stepper.stop(); 
  stepper.setCurrentPosition(0); 
}  
  
void Powrot()  
{  
  HOME(stepperY, K3_PIN, 1); // Oś X do krańcówki K1  
  HOME(stepperX, K1_PIN, 1); // Oś Y do krańcówki K3  
  HOME(stepperZ, K5_PIN, -1); // Oś Z do krańcówki K5  
  homeFinished = true;  
  paleta1wykryta = false;  
  paleta2wykryta = false;  
  paleta3wykryta = false;  
  paleta4wykryta = false;  
  paleta5wykryta = false;  
}  

const int regalCount = 12; // Liczba regałów  
const int regalCoords[regalCount][2] = {  
  {-9000, 100},   // REGAL_1  
  {-13250, 100},  // REGAL_2  
  {-17500, 100},  // REGAL_3  
  {-9000, 6400},  // REGAL_4  
  {-13250, 6400}, // REGAL_5  
  {-17500, 6400}, // REGAL_6  
  {-9000, 12150}, // REGAL_7  
  {-13250, 12150},// REGAL_8  
  {-17500, 12150},// REGAL_9  
  {-9000, 18250}, // REGAL_10  
  {-13250, 18250},// REGAL_11  
  {-17500, 18250} // REGAL_12  
};  
  
void RUCH1(int regalNumber) 
{  
  stepperX.run();  
  stepperY.run();  
  stepperZ.run();  
  rfidDetected = true; 
  static int step = 0; 

  switch (processState) 
  {  
    case 0: // Start procesu - pobranie palety  
    movementInProgress = true;
        delay(3000); 
        Pobranie_RUCH1();  
        if (pobranieZakonczone) 
        {  
          processState = 1; 
          pobranieZakonczone = false;  
        }  
        break;  

    case 1: // Przemieszczanie do regału  
        if (step == 0) 
        {  
          if (regalNumber < 1 || regalNumber > regalCount) 
          {  
            return;  
          }  
          // Pobierz współrzędne regału  
          int targetX = regalCoords[regalNumber - 1][0];  
          int targetZ = regalCoords[regalNumber - 1][1];  

          stepperX.moveTo(targetX);  
          stepperZ.moveTo(targetZ);  

          stepperX.setMaxSpeed(8000);  
          stepperX.setAcceleration(6000);  
          stepperZ.setMaxSpeed(8000);  
          stepperZ.setAcceleration(6000);  

          step = 1;  
        }  

        if (step == 1 && stepperX.distanceToGo() == 0 && stepperZ.distanceToGo() == 0) 
        {  
          step = 0;          
          move_end = true;  
          processState = 2;   
        }  
        break;  

    case 2: // Odkładanie palety  
        Odlozenie_RUCH1();  
        if (odkladanieZakonczone) 
        {  
          processState = 3; 
          odkladanieZakonczone = false;  
        }  
        break;  

    case 3: // Powrót do pozycji początkowej  
        Powrot();  
        if (homeFinished) 
        {  
          homeFinished = false;  
          Odk = false;  
          move_end = false;  
          paleta1wykryta = 0;  
          paleta2wykryta = 0; 
          paleta3wykryta = 0;
          paleta4wykryta = 0;
          paleta5wykryta = 0; 
          rfidDetected = false;  
          movementInProgress = false; 
          displayMessage("KONIEC","ODKLADANIA");
          delay(1000); 
          displayMessage("POZYCJA HOME","TRYB ODKLADANIA"); 
          buzzer3();
          processState = 0;  
        }  
        break;  
    }  
}  
  
void RUCH2(int regalNumber)   
{  
  stepperX.run();  
  stepperY.run();  
  stepperZ.run();  
  movementInProgress = true;  
  rfidDetected = true; 
  static int step = 0; 
  switch (processState) 
  {  
    case 0: // Przemieszczanie do regału  
      if (step == 0) 
      {  
        if (regalNumber < 1 || regalNumber > regalCount) 
        {  
          return;  
        }  
        // Pobierz współrzędne regału  
        int targetX = regalCoords[regalNumber - 1][0];  
        int targetZ = regalCoords[regalNumber - 1][1]; 

        delay(1500); 

        stepperX.moveTo(targetX);  
        stepperZ.moveTo(targetZ);  

        stepperX.setMaxSpeed(8000);  
        stepperX.setAcceleration(6000);  
        stepperZ.setMaxSpeed(8000);  
        stepperZ.setAcceleration(6000);  

        step = 1; // Przejdź do kolejnego etapu  
      }  
  
      if (step == 1 && stepperX.distanceToGo() == 0 && stepperZ.distanceToGo() == 0)
      {  
        step = 0;          
        move_end = true;    
        processState = 1;  
      }  
      break;  
     
    case 1:  
        Pobranie_RUCH2();  
        if (pobranieZakonczone) 
        {  
          processState = 2; 
          pobranieZakonczone = false;  
        }  
        break;  
  
    case 2:  
        Powrot();  
        if (homeFinished) 
        {  
          processState = 3; 
          move_end = true;   
        }  
        break;  
  
          case 3: // Odkładanie palety  
        Odlozenie_RUCH2();  
        if (odkladanieZakonczone) 
        {  
          odkladanieZakonczone = false;  
          homeFinished = false;   
          Odk = false;  
          move_end = false;   
          paleta2wykryta = false;  
          paleta1wykryta = false;  
          rfidDetected = false;  
          movementInProgress = false; 
          messageDisplayed = false;   
          resetFlagsP(); 
          displayMessage("KONIEC ODBIERANIA","");
          delay(1000);
          displayMessage("POZYCJA HOME","TRYB ODBIERANIA");
          buzzer3();
          processState = 0; 
        }  
      break;  
    }  
}  
 
void loop()
{ 
  if(!readStop)
  { 
  readRFID(); 
  } 

  if (kartaOdbioru) 
  { 
    unsigned long currentMillis = millis(); 
    if (currentMillis - lastDebounceTime > debounceDelay && !movementInProgress) 
    { 
      int pressedKey = readTTP229();  
    if (pressedKey > 0 && pressedKey <= 8) {  
        lastDebounceTime = currentMillis; 

        switch (pressedKey) { 
          case 1: ODB1 = true; break; 
          case 2: ODB2 = true; break; 
          case 3: ODB3 = true; break; 
          case 4: ODB4 = true; break; 
          case 5: ODB5 = true; break; 
          case 6: ODB6 = true; break; 
          case 7: ODB7 = true; break; 
          case 8: ODB8 = true; break; 
        }  
        if (!messageDisplayed) {
          displayMessage("Trwa odbieranie", "Prosze czekac...");
          messageDisplayed = true;
        }
      } 
    } 
  } 
    if (ODB1) { RUCH2(5); } 
    if (ODB2) { RUCH2(6); } 
    if (ODB3) { RUCH2(7); } 
    if (ODB4) { RUCH2(8); } 
    if (ODB5) { RUCH2(9); } 
    if (ODB8) { 
      kartaOdbioru = false; 
      readStop = false; 
      rfidDetected = false;
      messageDisplayed = false;
      buzzer2();
      displayMessage("TRYB ODBIERANIA", "OFF"); 
      delay(1500); 
      displayMessage("POZYCJA HOME", "TRYB ODKLADANIA");
      ODB8 = false;
    } 
   else if (!kartaOdbioru)
   { 
    if (paleta1wykryta) { RUCH1(5); } 
    if (paleta2wykryta) { RUCH1(6); } 
    if (paleta3wykryta) { RUCH1(7); } 
    if (paleta4wykryta) { RUCH1(8); } 
    if (paleta5wykryta) { RUCH1(9); } 
  } 
  stepperX.run(); 
  stepperY.run(); 
  stepperZ.run(); 
} 
