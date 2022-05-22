/*
  Limiti: 63 Giorni Incubazione con frequenza di rotazione di 6 ore
 */


//PIN D2: DHT / OUTPUT
//PIN D3: BUTTON 2 / INPUT_PULLUP
//PIN D4: BUTTON 1 / INPUT_PULLUP
//PIN D5: BUTTON 3 / INPUT_PULLUP
//PIN D6: IN2 / OUTPUT
//PIN D7: 
//PIN D8: IN4 / OUTPUT
//PIN D9: IN3 / OUTPUT
//PIN D10: RESISTOR / OUTPUT PWM 490Hz
//PIN D11: IN1 / OUTPUT
//PIN D12:
//PIN D13:
//PIN A0:
//PIN A1:
//PIN A2:
//PIN A3:
//PIN A4: SDA: LCD 20x4; RTC
//PIN A5: SCL: LCD 20x4; RTC
//PIN A6:
//PIN A7:



//Librerie:
#include "RTClib.h"
#include <Stepper.h>
#include "DHT.h"
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>



//Costanti e Variabili RTC:
RTC_DS1307 rtc;
unsigned long unix_data = 0;

unsigned int secondi_restanti = 0;
unsigned int minuti_restanti = 0;
unsigned int ore_restanti = 0;
unsigned int giorni_restanti = 0;
unsigned long t_restanti = 0;


//Costanti e Variabili Display LCD:
LiquidCrystal_I2C lcd(0x27, 20, 4);


//Costanti e Variabili Sensore di Temperatura DHT:
#define DHTPIN 2 
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
float temperature, humidity = 0;              //0 = humidity, 1 = temeperature celsius, 2 = temperature fahrenheit.


//Costanti e Variabili Stepper Motor:
#define IN1  11
#define IN2  6
#define IN3  9
#define IN4  8

#define StepMotore  200                 //modificare il valore in base agli step del motore in possesso
#define StepDaPercorrere  33

Stepper mioMotore(StepMotore, IN1, IN2, IN3, IN4);

unsigned long delay_modifica_statostepper = 0;

int girare = 0;
bool EnableMotor = 1;


//Costanti e Variabili Output: 
#define Resistenza 10


//Costanti e Variabili di Input:
#define b1 4
#define b2 3
#define b3 5


//Costanti e Variabili EEPROM: 
int indirizzo_dati_fondamentali = 0;
unsigned long timer_reset_eeprom = 0;


//Costanti e Variabili PID:
float PID_p, PID_i, PID_d = 0;
float kp = 0.3;   float ki = 0.1;   float kd = 0.15;
float PID_error = 0;
float previous_error = 0;
unsigned long elapsedTime, Time, timePrev;
float PID_value = 0;


//Costanti e Varibili per Algoritmi:
unsigned int giorni_incubazione = 0;
float temperatura_incubazione = 37.00;
unsigned int umidita_incubazione = 40;

unsigned long delay_modifica_dati = 0;
unsigned long delay_attesa = 0;

bool a = 0;

bool oneprint = 0;



void setup() {
  //Setup Comunicazione Seriale:
  Serial.begin(115200);
  delay(250);


  //Setup RTC:
  if(!rtc.begin())errore(2);
  Serial.println();
  if (!rtc.isrunning())rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));


  //Setup Display LCD:
  lcd.begin();
  lcd.backlight();
  lcd.clear();


  //Setup Motore Stepper:
  mioMotore.setSpeed(5);    


  //Setup Sensore Temperatura:
  dht.begin();


  //Setup Input:
  pinMode(b1, INPUT_PULLUP);
  pinMode(b2, INPUT_PULLUP);
  pinMode(b3, INPUT_PULLUP);


  //Setup Output:
  pinMode(Resistenza, OUTPUT);
  digitalWrite(Resistenza, LOW);


  //Setup Per Registrazione Dati:
  cancellazione_eeprom();

  if(!EEPROM.read(200)) EEPROM.write(200,1);                    //Variabile per rotazione motore. Se non è presente in EEPROM scrivo 1 di default.
  if(EEPROM.read(555) != 0) girare = EEPROM.read(555);          //Variabile di stato rotazione motore. Se è presente in EEPROM la salvo nella variabile "girare".

  //Registrazione Dati Fondamentali (Giorni incubazione, temperatura e umidità ideali, data di start in formato unix) se non sono in EEPROM.
  if(!EEPROM.get(indirizzo_dati_fondamentali + 6, giorni_incubazione)){         
    registrazione_dati();
    registrazione_orario_start();
  }
  else{                                                                         //Se sono presenti in EEPROM li recupero e li salvo nelle rispettive variabili.
    temperatura_incubazione = EEPROM.get(indirizzo_dati_fondamentali, temperatura_incubazione);
    indirizzo_dati_fondamentali += sizeof(float);
    umidita_incubazione = EEPROM.get(indirizzo_dati_fondamentali, umidita_incubazione);
    indirizzo_dati_fondamentali += sizeof(int);
    giorni_incubazione = EEPROM.get(indirizzo_dati_fondamentali, giorni_incubazione);
    EEPROM.get(800, unix_data);
    
    indirizzo_dati_fondamentali = 0;


    Serial.print("Giorni di Incubazione: ");
    Serial.print(giorni_incubazione);
    Serial.print(" Temperatura di Incubazione: ");
    Serial.print(temperatura_incubazione);
    Serial.print("°C");
    Serial.print(" Umidità di Incubazione: ");
    Serial.print(umidita_incubazione);
    Serial.print("%");
    Serial.print(" Data di start in formato UNIX: ");
    Serial.print(unix_data);
    Serial.println("s");
    Serial.println("");
    Serial.println("");
  }
  
  Time = millis();                                              //Variabile per calcolo PID.
}

void loop() {
  DateTime now = rtc.now();

  if(unlock_modifica_dati()){                                   //Statement per la modifica dei dati fondamentali durante l'incubazione.
    lcd.clear();
    while(digitalRead(!b3)){
      lcd.setCursor(0,0);
      lcd.print("Rilascia il Pulsante");
      if(!oneprint){
        Serial.println("Rilascia il Pulsante"); 
        oneprint = 1;
      }
      if(digitalRead(b3))break;
    }
    lcd.clear();
    Serial.println("");
    Serial.println("Modifica Parametri");
    Serial.println("");
    Serial.println("");
    registrazione_dati();
  }

  if(unlock_modifica_statostepper()){                           //Statement per la modifica dei dati fondamentali durante l'incubazione.
    lcd.clear();
    while(digitalRead(!b1)){
      lcd.setCursor(0,0);
      lcd.print("Rilascia il Pulsante");
      if(!oneprint){
        Serial.println("Rilascia il Pulsante"); 
        oneprint = 1;
      }
      if(digitalRead(b1))break;
    }
    lcd.clear();
    
    EnableMotor = !EnableMotor;

    if(EnableMotor){
      Serial.println("");
      Serial.println("Stepper Attivato");
      Serial.println("");
      Serial.println("");
    }
    else{
      while(girare != 0) rotazione();
      Serial.println("Stepper Disattivato");
      lcd.setCursor(0,0);
      lcd.print("Stepper Disattivato");
    }
  }

  if((millis() - delay_attesa) >= 2000){                          //Statement Principale, per la lettura del sensore e il controllo dei PID.
    lettura_sensore();
    pid_control();
    lcd_print();
    delay_attesa = millis();
  }
  
  if((now.unixtime() - unix_data) >= (21600 * EEPROM.read(200)) && t_restanti > 0){        //Rotazione Stepper Motor  8 ORE: 28800      6 ORE: 21600
    Serial.print("Rotazione ");
    Serial.print((now.unixtime() - unix_data)/(EEPROM.read(200)));
    Serial.print(" ");
    if(EnableMotor) rotazione();
    Serial.println(girare);
    EEPROM.write(200,(EEPROM.read(200)+1));
  }
}



void pid_control(){
  PID_error = temperatura_incubazione - temperature;

  PID_p = kp * PID_error;
  
  if(PID_error > -3 && PID_error < 3) PID_i += (ki * PID_error);
  else PID_i = 0;
  
  if(PID_i >= 100) PID_i = 100;
  else if(PID_i <= -100) PID_i = -100;
  
  timePrev = Time;                         
  Time = millis();                          
  elapsedTime = (Time - timePrev) / 1000; 
  PID_d = kd*((PID_error - previous_error)/elapsedTime);

  PID_value = PID_p + PID_i + PID_d;  

  if(PID_value < -40) PID_value = -40;   
  if(PID_value > (250-40)) PID_value = (250-40); 

  //Serial.println((int)PID_value);
  
  analogWrite(Resistenza,(40 + (int)PID_value));      //diminuire 50
  
  previous_error = PID_error;  
}



void lettura_sensore(){
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  if (isnan(temperature) || isnan(humidity)){
    Serial.println(F("Errore di lettura del sensore!"));
  } 
}



void lcd_print(){
  if(!a){
    lcd.clear();
    
    DateTime now = rtc.now();
    
    Serial.print("Orario Attuale: ");
    Serial.print(now.day(),DEC);
    Serial.print("/");
    Serial.print(now.month(),DEC);
    Serial.print("/");
    Serial.print(now.year(),DEC);
    Serial.print("  ");
    Serial.print(now.hour());
    Serial.print(':');
    Serial.print(now.minute());
    Serial.print(':');
    Serial.print(now.second());
    Serial.print("  ");
    
    lcd.setCursor(0,0);
    lcd.print("Now: ");
    lcd.print(now.day());
    lcd.print('/');
    lcd.print(now.month());
    lcd.print('/');
    lcd.print(now.year());
    
    lcd.setCursor(0,1);
    lcd.print(now.hour());
    lcd.print(':');
    lcd.print(now.minute());
    lcd.print(':');
    lcd.print(now.second());

    lcd.setCursor(0,2);
    lcd.print("Tempo Mancante: ");
    
    t_restanti = ((unix_data+(86400*giorni_incubazione)) - now.unixtime());

    if(t_restanti > 0){
      secondi_restanti = t_restanti % 60;
      t_restanti /= 60;
      minuti_restanti = t_restanti % 60;
      t_restanti /= 60;
      ore_restanti = t_restanti % 24;
      giorni_restanti = t_restanti / 24;
    
      Serial.print("Tempo Mancante: ");
      Serial.print(giorni_restanti);
      Serial.print("gg ");
      Serial.print(ore_restanti);
      Serial.print("hh ");
      Serial.print(minuti_restanti);
      Serial.println("mm ");
      Serial.println("");
      
      lcd.setCursor(0,3);
      lcd.print(giorni_restanti);
      lcd.print("gg ");
      lcd.print(ore_restanti);
      lcd.print("hh ");
      lcd.print(minuti_restanti);
      lcd.print("mm");
    }
    else{
      Serial.println("Incubazione Terminata: ");
      Serial.println("");

      while(girare != 0) rotazione();
      
      lcd.setCursor(0,3);
      lcd.print("Incubazione Terminata");
    }
    
    a = !a;
  }
  else{
    lcd.clear();

    Serial.print("Temperatura Reale: ");
    Serial.print(temperature);
    Serial.print("°C");
    Serial.print(" Temperatura Ideale: ");
    Serial.print(temperatura_incubazione);
    Serial.println("°C");
    
    Serial.print("Umidità Reale: ");
    Serial.print(humidity);
    Serial.print("%");
    Serial.print(" Umidità Ideale: ");
    Serial.print(umidita_incubazione);                           
    Serial.println("%");
    Serial.println("");
    
    lcd.setCursor(0,0);
    lcd.print("Temp. Reale: ");
    lcd.print(temperature);
    lcd.print("C");
    lcd.setCursor(0,1);
    lcd.print("Temp. Ideale: ");
    lcd.print(temperatura_incubazione);
    lcd.print("C");

    lcd.setCursor(0,2);
    lcd.print("Humidity Re: ");
    lcd.print(humidity);
    lcd.print("%");
    lcd.setCursor(0,3);
    lcd.print("Humidity Id: ");
    lcd.print(umidita_incubazione);             
    lcd.print("%");

    a = !a;
  }
}





//EEPROM Function:
void cancellazione_eeprom(void){
  while(girare != 0) rotazione();
  while(!digitalRead(b3)){          //Cancello EEPROM
    lcd.setCursor(0,0);
    lcd.print("Tieni Premuto Per"); 
    lcd.setCursor(0,1);
    lcd.print("Cancellare la EEPROM");
    if(!oneprint){
      Serial.println("Tieni Premuto per cancellare la EEPROM"); 
      oneprint = 1;
    }
    if(millis() > 10000){
      lcd.clear();
      while(!digitalRead(b3)){
        lcd.setCursor(0,0);
        lcd.print("Rilascia il dito");
        if(oneprint){
          Serial.println("Rilascia il dito");
          oneprint = 0; 
        }
      }
      lcd.clear();
      for(int i = 0; i < EEPROM.length(); i++){
        EEPROM.update(i, 0);
      }
      lcd.setCursor(0,0);
      lcd.print("EEPROM cancellata"); 
      Serial.println("EEPROM Cancellata");
      Serial.println("");
      delay(5000);  
    }
  }
  lcd.clear();
}



void rotazione(void){
  if(girare == 0){
    girare = 1; 
  }
  else if(girare == 2){
    girare = 3;
  }
  else if(girare == 4){
    girare = 5;
  }
  else if(girare == 6){
    girare = 7;
  }
  
  if(girare != 0){
    if(girare == 1){
      mioMotore.step(StepDaPercorrere);
      girare = 2;
    }
    else if(girare == 3){
      mioMotore.step(-StepDaPercorrere);
      girare = 4;
    }
    else if(girare == 5){
      mioMotore.step(-StepDaPercorrere);
      girare = 6;
    }
    else if(girare == 7){
      mioMotore.step(StepDaPercorrere);
      girare = 0;
    }
  }

  EEPROM.write(555, girare);
}





//Other:

void errore(int e){
  switch(e){
    case(2):                              //Errore Comunicazione con RTC in fase di Setup (15tab)
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Errore RTC");
      Serial.println("Errore RTC");
      while(1); 
    default:
      break;
  }
}





bool unlock_modifica_dati(void){
  if(digitalRead(b3))if(!delay_modifica_dati)delay_modifica_dati = millis();   
  else delay_modifica_dati = millis();

  if((millis() - delay_modifica_dati) > 5000){
    oneprint = 0;
    return 1;
  }
  else return 0;
}


bool unlock_modifica_statostepper(void){
  if(digitalRead(b1))if(!delay_modifica_statostepper)delay_modifica_statostepper = millis();   
  else delay_modifica_statostepper = millis();

  if((millis() - delay_modifica_statostepper) > 5000){
    oneprint = 0;
    return 1;
  }
  else return 0;
}





void registrazione_dati(void){
  while(1){
    lcd.setCursor(0, 0);
    lcd.print("Quanti giorni");
    lcd.setCursor(0, 1);
    lcd.print("deve durare");
    lcd.setCursor(0, 2);
    lcd.print("l'incubazione?");
    
    lcd.setCursor(0, 3);
    lcd.print("Giorni: ");
    lcd.print(giorni_incubazione);
    
    if(!digitalRead(b1)){
      giorni_incubazione++;
      delay(300);
    }
    else if(!digitalRead(b2) && giorni_incubazione > 0 ){
      if(giorni_incubazione == 10)lcd.clear();
      giorni_incubazione--;
      delay(300);
    }
    
    if(!digitalRead(b3)){
      lcd.clear();
      delay(400);
      break;
    }
  }

  while(1){
    lcd.setCursor(0, 0);
    lcd.print("Quale deve essere la");
    lcd.setCursor(0, 1);
    lcd.print("Temperatura?");
    
    lcd.setCursor(0, 3);
    lcd.print("Temperatura: ");
    lcd.print(temperatura_incubazione);
    
    if(!digitalRead(b1)){
      temperatura_incubazione += 0.1;
      delay(300);
    }
    else if(!digitalRead(b2) && temperatura_incubazione > 0){
      temperatura_incubazione -= 0.1;
      delay(300);
    }
    
    if(!digitalRead(b3)){
      lcd.clear();
      delay(400);
      break;
    }
  }

  while(1){
    lcd.setCursor(0, 0);
    lcd.print("Quale deve essere");
    lcd.setCursor(0, 1);
    lcd.print("l'umidita'?");
    
    lcd.setCursor(0, 3);
    lcd.print("Umidita': ");
    lcd.print(umidita_incubazione);
    lcd.print("%");
    
    if(!digitalRead(b1)){
      umidita_incubazione += 1;
      delay(300);
    }
    else if(!digitalRead(b2) && umidita_incubazione > 0){
      umidita_incubazione -= 1;
      delay(300);
    }
    
    if(!digitalRead(b3)){
      lcd.clear();
      delay(400);
      break;
    }
  }

  EEPROM.put(indirizzo_dati_fondamentali, temperatura_incubazione);
  indirizzo_dati_fondamentali += sizeof(float); 
  EEPROM.put(indirizzo_dati_fondamentali, umidita_incubazione); 
  indirizzo_dati_fondamentali += sizeof(int); 
  EEPROM.put(indirizzo_dati_fondamentali, giorni_incubazione);
  indirizzo_dati_fondamentali = 0;

  Serial.print("Giorni di Incubazione: ");
  Serial.print(giorni_incubazione);
  Serial.print(" Temperatura di Incubazione: ");
  Serial.print(temperatura_incubazione);
  Serial.print("°C");
  Serial.print(" Umidità di Incubazione: ");
  Serial.print(umidita_incubazione);
  Serial.println("%");
  Serial.println("");
  Serial.println("");
}



void registrazione_orario_start(){
  DateTime now = rtc.now();
  
  unix_data = now.unixtime();
  Serial.print("Data di start registrata in formato UNIX: ");
  Serial.println(unix_data);
  Serial.println("");
  Serial.println("");

  EEPROM.put(800, unix_data);
}
