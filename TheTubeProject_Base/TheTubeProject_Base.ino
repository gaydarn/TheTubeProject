#include "mbed.h"
#include "FanManager.h"
#include "Potentiometer.h"
#include "HCSR04.h"
#include "SimpleTimer_TheTube.h"


/*********************************************************
           INSTANCIATION DES PERIPHERIQUES
**********************************************************/
int i=0;
bool start = false;
bool ramp = false;
bool contest = false;
int setpointRPM;
int realSetpoint;
String command;

float _mainFanSpeed;
float _secondaryFanSpeed;
float _plotHeight;
float _externalSetpoint;
float _fan2Setpoint=0.5;
float _lastTrajSetpoint=0;
float _Quiet=HIGH;

long int tExecSpeedTask;
long int tExecPosTask;
long int tExecUserTask;
long int tExecMonTask;
long int tTemp;

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

FanManager mainFan(MainFanPWMPin,MainFanHallPin,MainFanEnablPin,MainFanCurrentPin);

FanManager secondaryFan(SecondaryFanPWMPin,SecondaryFanHallPin,SecondaryFanEnablPin,SecondaryFanCurrentPin);

Potentiometer consigneExterne(PotentiometerPin);

HCSR04 heightSensor(HS_TrigPin, HS_EchoPin);
/*********************************************************
          GESTION DES INTERRUPTIONS (HARDWARE)
**********************************************************/
/*
Afin de pouvoir évaluer la vitesse des ventilateurs, il est
nécéssaire de compter les impulsions des capteurs Hall des
ventilateurs.
La fonction de callback ne peut pas être une méthode de classe,
a moins qu'elle soit statique. Comme on a deux ventilateur,
On a choisi de créer des fonction de callback dans le code 
principal, qui vont elle-même apeler en cascade les méthode
des deux instances de ventilateurs.
*/

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
  pinMode(mainFan.getHallPinNumber(), INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(mainFan.getHallPinNumber()), mainFan_incRpmCounter, CHANGE);

  pinMode(secondaryFan.getHallPinNumber(), INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(secondaryFan.getHallPinNumber()), SecondaryFan_incRpmCounter, CHANGE);
}

/*********************************************************
               GESTION DES TACHES ET TIMERS
**********************************************************/
/*
Le programme est séparé en plusieurs tâches.
Afin d'avoir un minimum de synchronisation dans l'ordre des opérations, 
on utilisera un premier timer software pour les actions les plus critiques 
de régulation, qui sera chargé d'éxécuter les différentes tâches 
avec un système de diviseurs.
Un deuxipme timer software sera chargé de la tâche user_Ctrl, moins 
critique au niveau du timing, insi que le tâche monitoring qui
ne doit pas être exécutée plus souvent que nécéssaire afin de ne pas
surcharger le système.

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

Le rôle de chacune des tâches est proposé ci-dessous:

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
  - Envoie la tramme sur la sortie serielle

NB: D'autre tâches peuvent apparaître au cours du projet, il s'agit ici de la base de l'application.
*/

//#define MONITOR 
//#define PLOT_TIMINGS


void speed_Ctrl_Task()
{
  tTemp = micros();
  //Inputs
  _mainFanSpeed = mainFan.computeSpeedRPM();
  _secondaryFanSpeed = secondaryFan.computeSpeedRPM();
  //Compute
  if(!start)
  {
    mainFan.setSpeedProp(0);
    secondaryFan.setSpeedProp(0);
  }
  if(contest)
  {
    mainFan.setSpeedProp(float(_lastTrajSetpoint/100));
    secondaryFan.enableRotation(false);
  }
  else
  {
    mainFan.setSpeedProp(float(_externalSetpoint/100));
    secondaryFan.setSpeedProp(_fan2Setpoint);
  }

  //Output
  mainFan.enableRotation(!_Quiet);
  secondaryFan.enableRotation(!_Quiet);
  tExecSpeedTask = micros() - tTemp;
}

void position_Ctrl_Task()
{
  tTemp = micros();
  //Inputs
  _plotHeight = heightSensor.measureDistance();
 
  //Compute

  //Ouput

  tExecPosTask = micros() - tTemp;
}

void user_Ctrl_Task()
{
  tTemp = micros();
  _externalSetpoint = consigneExterne.getValuePercent();

  //Récupération de la commande
  if(Serial.available())
  {
    command = Serial.readStringUntil('\n');
         
    if(command.equals("start"))
    {
      start = true;
      contest = false;
      _Quiet = LOW;
    }
    else if(command.equals("inc"))
    {
      i++;
    }
    else if(command.equals("dec"))
    {
      i--;
    }
    else if(command.equals("stop"))
    {
      start = false;
    }
    else if(command.equals("reset"))
    {
      NVIC_SystemReset();                         // Reset the microcontroller
    }
    else if(command.indexOf("kp") == 0) //The command format must be Kp=xx.xx
    {
      //Kp=command.substring(3).toDouble();
    }
    else if(command.indexOf("ki") == 0) //The command format must be Ki=xx.xx
    {
      //Ki=command.substring(3).toDouble();
    }
    else if(command.indexOf("kd") == 0) //The command format must be Kd=xx.xx
    {
      //Kd=command.substring(3).toDouble();
    }
    else if(command.indexOf("fan2") == 0) //The command format must be Fan2=xx.xx
    {
      _fan2Setpoint=command.substring(5).toDouble();
    }
     else if(command.indexOf("quiet") != -1) //Si on détecte un quiet
    {
      _Quiet=!_Quiet;
    }
    else if(command.equals("contest"))
    {
      contest = true;
      start = false;
      _Quiet = LOW;
      _lastTrajSetpoint = 0;
    }
    else if(command.indexOf("traj") == 0) //The command format must be traj=<time>;<setpoint>
    {
      _lastTrajSetpoint = command.substring(21).toDouble(); //Interprétation de la commande et récupération de la valeur de consigne
      Serial.println(command.substring(5,command.length()-1) + ";" + String(float(_mainFanSpeed)/145.0));
      //Serial.println(command.substring(5,command.length()-1) + ";" + String(_plotHeight)); //Réponse avec les informations reçue + la position actuelle 
    }
    else
    {
      Serial.println("Invalid command");
    }
  }

  //TODO
  tExecUserTask = micros() - tTemp;
}

void monitoring_Task()
{
  tTemp = micros();
    
#ifdef MONITOR
  Serial.print ("cons_ext.:");
  Serial.print (_externalSetpoint, 1);
  Serial.print (",MainFanRpm:");
  Serial.print (_mainFanSpeed, 0);
  Serial.print (",SecondaryFanRpm:");
  Serial.print (_secondaryFanSpeed, 0);
  Serial.print (",setpointRPM:");
  Serial.print (setpointRPM, DEC);
  Serial.print (",realSetpoint:");
  Serial.print (realSetpoint, DEC);
  Serial.print (",HS_value:");
  Serial.print (_plotHeight,1);
  Serial.print ("\r\n");
#endif

#ifdef PLOT_TIMINGS
  Serial.println ("Application timings: ");
  Serial.println ("Speed Task: "+String(float(tExecSpeedTask)/1000)+" ms");
  Serial.println ("Pos Task: "+String(float(tExecPosTask)/1000)+" ms");
  Serial.println ("User Task: "+String(float(tExecUserTask)/1000)+" ms");
  Serial.println ("Monitoring Task: "+String(float(tExecMonTask)/1000)+" ms");
#endif

    tExecMonTask = micros() - tTemp;
}

#define SPEED_TASK_PERIOD_MS 60
#define POS_TASK_MUL  2 //Give position task time = POS_TASK_MUL*SPEED_TASK_PERIOD
#define USER_TASK_MUL  2 //Give user task time = USER_TASK_MUL*SPEED_TASK_PERIOD
#define MON_TASK_MUL  4 //Give monitoring task time = MON_TASK_MUL*SPEED_TASK_PERIOD

#define TIMER_INTERVAL_US        SPEED_TASK_PERIOD_MS*1000      // 1s = 1 000 000us

// the soft timer object
SimpleTimer timerSoft;
SimpleTimer timerHard;
int counterHard=0;
int counterSoft=0;

void HandlerTickTaskHard()
{
  // Call the different Tasks here inside ISR
  // No Serial.print() can be used 
  if(counterHard % POS_TASK_MUL)
  {
    position_Ctrl_Task();
  }  
  counterHard++;

  speed_Ctrl_Task();
}

void HandlerTickTaskSoft() {
  
  user_Ctrl_Task();

  if(counterSoft % (MON_TASK_MUL/USER_TASK_MUL))
  {
    monitoring_Task();
  }  
  counterSoft++;
}

/*********************************************************
                      APPLICATION
**********************************************************/

#define NBR_DIG 2

void setup()
{
  interrupts(); 

  setupFanInterrupts();

  timerHard.setInterval(SPEED_TASK_PERIOD_MS, HandlerTickTaskHard);

  timerSoft.setInterval(SPEED_TASK_PERIOD_MS*USER_TASK_MUL, HandlerTickTaskSoft);
  
  Serial.begin(115200);

}

void loop() //monitoring_Task
{
  timerSoft.run();
  timerHard.run();
}