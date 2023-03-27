#if !( ARDUINO_ARCH_NRF52840 && TARGET_NAME == ARDUINO_NANO33BLE )
  #error This code is designed to run on nRF52-based Nano-33-BLE boards using mbed-RTOS platform! Please check your Tools->Board setting.
#endif

#include "mbed.h"
#include "FanManager.h"
#include "Potentiometer.h"
#include "HCSR04.h"

#define MONITOR 




//*********************************************************
//***************MAIN**************************************
//*********************************************************
#define MainFanEnablPin D2
#define MainFanPWMPin D3
#define MainFanHallPin D7
#define MainFanCurrentPin A2

#define SecondaryFanEnablPin D6
#define SecondaryFanPWMPin D5
#define SecondaryFanHallPin D9
#define SecondaryFanCurrentPin A3

#define PotentiometerPin A0

#define HS_TrigPin D4
#define HS_EchoPin D8

int i=0;
bool start = false;
bool ramp = false;
int setpointRPM;
int realSetpoint;
String command;


FanManager mainFan(MainFanPWMPin,MainFanHallPin,MainFanEnablPin,MainFanCurrentPin);

FanManager secondaryFan(SecondaryFanPWMPin,SecondaryFanHallPin,SecondaryFanEnablPin,SecondaryFanCurrentPin);

Potentiometer consigneExterne(PotentiometerPin);

HCSR04 heightSensor(HS_TrigPin, HS_EchoPin);

void mainFan_incRpmCounter()
{
  mainFan.incRpmCounter();
}

void SecondaryFan_incRpmCounter()
{
  secondaryFan.incRpmCounter();
}

void setupFanInterrupts()
{
  interrupts(); 

  pinMode(mainFan.getHallPinNumber(), INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(mainFan.getHallPinNumber()), mainFan_incRpmCounter, RISING);

  pinMode(secondaryFan.getHallPinNumber(), INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(secondaryFan.getHallPinNumber()), SecondaryFan_incRpmCounter, RISING);
}





/*
Le programme est séparé en plusieurs tâches.
Afin d'avoir un minimum de synchronisation dans l'ordre des opérations, 
on utilisera un timer hardware principal, qui sera chargé d'éxécuter
les différentes autres tâches avec un système de diviseurs.
Seule la tâche de monitoring est séparée du reste et appelée dans la
loop() arduino. Elle est rythmée par une pause, ce qui ne la rend pas temps réel.

Vous pouvez librement adapter la porposition ci-dessous, mais n'oubliez pas de
le justifier dans la rapport, ce qui me permettra de comprendre plus façilement
ce que vous avez fait.

Attention, l'utilisation de la communication serielle n'est pas anodine en terme
de consommation des ressources. Veillez à limiter au maximum les print dans les
tâches synchronisées et laissez dans la mesure du possible cette tâche à la
tâche de monitoring.

Chacune de ces tâches sera organisée en terme de séquencement.
On commence par récupérer les valeurs (Input), on calcul ensuite les sorties (Compute)
puis on affecte finnalement les sorties physiques ou de la tâche suivante (Ouput).

Le rôle de chacune des tâches est décrit ci-dessous:

speed_Ctrl_Task:
  - Récupère la valeur de la vitesse des ventilateur
  - Récupère la dernière consigne de vitesse
  - Calcule la nouvelle sortie/correction
  - Affecte la sortie sur le pwm

position_Ctrl_Task:
  - Récupère la valeur de position du capteur
  - Récupère la dernière consigne de position
  - Calcule la nouvelle sortie/correction
  - Affecte la sortie pour la boucle de vitesse

user_Ctrl_Task:
  - Récupère la dernière consigne selon le mode
  - Contrôle si une action utilisateur est demandée
  - Exécute l'action et calcule la nouvelle consigne le cas échéant
  - Affecte la sortie pour la boucle de postion

monitoring_Task
  - fait une copie des dernières valeurs à remonter
  - Met en forme pour l'affichage
  - Envoie la tramme sur le sortie serielle

NB: D'autre tâches peuvent apparaître au cours du projet, il s'agit ici de la base de l'application.

*/

void speed_Ctrl_Task()
{
  mainFan.computeSpeedRPM();
  secondaryFan.computeSpeedRPM();
  //TODO
}

void position_Ctrl_Task()
{
  heightSensor.measureDistance();
  //TODO
}

void user_Ctrl_Task()
{
  consigneExterne.getValuePercent();
  //TODO
}

void setup() {

  setupFanInterrupts();

  Serial.begin(115200);
}



void loop() //MainTask
{
   
//Récupération de la commande
  if(Serial.available())
  {
    command = Serial.readStringUntil('\n');
         
    if(command.equals("start"))
    {
        start = true;
    }
    else if(command.equals("inc"))
    {
        i++;
    }
    else if(command.equals("dec"))
    {
        i--;
    }
    else if(command.equals("ramp"))
    {
        ramp = !ramp;
    }
    else if(command.equals("stop"))
    {
        start = false;
    }
    else if(command.equals("reset"))
    {
        NVIC_SystemReset();                         // Reset the microcontroller
    }
    else{
        Serial.println("Invalid command");
    }
  }

//Application des actions demandées
  mainFan.enableRotation(start);
  secondaryFan.enableRotation(start);

  if(start)
  {
    if(ramp)
    {
      i++;
      setpointRPM =i*1000;
      i = (setpointRPM> 16000) ? 0 : i;
    }
    else
    {
      setpointRPM = 3500+((consigneExterne.getValuePercent()/100)*(14500-3500));
    }
  }
  else
  {
    setpointRPM = 0;
  }
  realSetpoint = mainFan.setSpeed(setpointRPM);
  secondaryFan.setSpeed(setpointRPM);
   
  //Wait 1 second
  delay(200);
  //refreshInputTask();

 #ifdef MONITOR
 
   //Prints the number calculated above
  Serial.print ("cons_ext.:");
  Serial.print (consigneExterne.getValuePercent());
  Serial.print (",MainFanRpm:");
  Serial.print (mainFan.computeSpeedRPM(), DEC);
  Serial.print (",SecondaryFanRpm:");
  Serial.print (secondaryFan.computeSpeedRPM(), DEC);
  Serial.print (",setpointRPM:");
  Serial.print (setpointRPM, DEC);
  Serial.print (",realSetpoint:");
  Serial.print (realSetpoint, DEC);
  Serial.print (",HS_value:");
  Serial.print (heightSensor.measureDistance(),DEC);
  Serial.print ("\r\n");
  
#endif
  //pwm->write(consigneExterne.getValuePercent()/100);
  //analogWrite(D3, consigneExterne.getValuePercent()*2.55);
  //Serial.print(counter);
  
}