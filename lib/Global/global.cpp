#include <global.h>


/*

Anciennes notes non mises à jour :


notes : attention, bibliothéque buffered serial modiiée : 
ssize_t BufferedSerial::read(void *buer, size_t length)
{
    size_t data_read = 0;

    char *ptr = static_cast<char *>(buer);

    i (length == 0) {
        return 0;
    }

    api_lock();
    int i =0;
    while (_rxbu.empty()) {
        i (!_blocking) {
            api_unlock();
            return -EAGAIN;
        }
        api_unlock();
        // Do we need a proper delay?
        thread_sleep_or(1);
        api_lock();
        i++;
        i (i>100000){return 0;} // ain qu'il ne reste pas bloqué et bloque tous les herkulex
    }

*/
/*

Protocol CAN message : 
ID pour controler les Herkulex : 0x50
Longueur : 6 octet
Contenue DATA : [ID du Herkulex] ; [commande] ; [position] ; [position] ; [playtime] ; [setLed] ; 

position < 1023
position de -180° à +180° avec 0° = 512

playtime < 255

setLed < 255 -> Bug trouvé : si à 0x02 active la rotation ininie du herkulex 
0x04 vert
0x08 bleu
0x10 rouge

------------------------------------------------------------------------------------------------------------------------------

ID qu'utilise les Herkulex pour répondre: 0x51
Longueur : 4 octet
Contenue DATA : [ID du Herkulex] ; [etat] ; [position] ; [position] ;

------------------------------------------------------------------------------------------------------------------------------

ID pour activer/desactiver le Torque : 0x52
Longueur : 2 octet
Contenue DATA : [ID du Herkulex] ; [ON/O]

------------------------------------------------------------------------------------------------------------------------------

ID pour clear  : 0x55
Longueur : 1 octet
Contenue DATA : [ID du Herkulex]

------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------

ID pour controler le moteur pas à pas : 0x60
Longueur : 4 octet
Contenue DATA : [position] ; [position] ; [position] ; [position] 
position en degrées en int

------------------------------------------------------------------------------------------------------------------------------

ID pour changer le mode : 0x61
longueur : 3 octet
Contenue DATA : [m0] ; [m1] ; [m2] 
m0, m1 et m2 sont des bool

------------------------------------------------------------------------------------------------------------------------------

ID pour changer bouger la pince : 0x70
longueur : 1 octet
Contenue DATA : [etat] 
Etat 0 : serrer et monter la pince
Etat 1 : lacher et descendre

------------------------------------------------------------------------------------------------------------------------------

ID pour changer l'ID du herkulex pour bouger la pince : 0x71
longueur : 1 octet
Contenue DATA : [ID] 

*/


#define DEGREPARSTEP 1.8

int IdHerkulexPince = 0x08, IdHerkulexPinceArriere1 = 4, IdHerkulexPinceArriere2 = 5;

Herkulex servo1(PB_6, PB_7, 115200);
DigitalIn contact_gat_rose(A4);
PwmOut servoCerise(A6);



// set pin numbers:
DigitalInOut EN (PA_3);
/*EN.input();
EN.mode(PullUp);*/
/*EN.output();*/
DigitalOut M0 (PB_5);
DigitalOut M1 (PA_8);
DigitalOut M2 (PF_1);
DigitalOut STDBY (PF_0);
DigitalOut STEP (PB_1);
DigitalOut DIRE (PB_0);

//mode
int m0 = 0;
int m1 = 0;
int m2 = 0;
int dir= 0;
// Global variable for the current position of the motor (in degrees)
float currentPosDegree = 0.0;

char counter = 0;
uint8_t status;
uint16_t position1[3] = {100, 512, 100};
uint16_t positionPinceArriere1[3] = {50, 160, 512};
uint16_t positionPinceArriere2[3] = {880, 755, 390};//Bonne valeurs



void printCANMsg(CAN *can, CANMessage &msg) {
    printf("  ID      = 0x%.3x\r\n", msg.id);
    printf("  Type    = %d\r\n", msg.type);
    printf("  format  = %d\r\n", msg.format);
    printf("  Length  = %d\r\n", msg.len);
    printf("  Data    =");            
    for(int i = 0; i < msg.len; i++){printf(" 0x%.2X", msg.data[i]);}
    printf("\r\n");
 }



void controleHerkulex(CAN *can, CANMessage &msg){
  ack(can, ACKNOWLEDGE_HERKULEX, IDCAN_HERKULEX);

  /*                     0              1                2           3            4            5           6*/
  /*Contenue DATA : [ID du Herkulex] ; [commande] ; [position] ; [position] ; [playtime] ; [setLed] ; */
    uint8_t IDHerkulex1 = msg.data[0];
    uint8_t commande = msg.data[1];
    uint16_t position1 = (msg.data[3]<<8) + (msg.data[2]);

    if(commande == 0){
      uint8_t playtime = msg.data[4];
      uint8_t setLed = msg.data[5];
      // printf("IDHerkulex : %d\ncommande : %d\position1 : %d\nplaytime : %d\nsetLed : %d\n", IDHerkulex1, commande, position, playtime ,setLed);
      servo1.positionControl(IDHerkulex1, position1 , playtime , setLed);
      // status = servo1.getPos(IDHerkulex1);
      // printf("Status = %04X\n", status);
    }
    else{
       uint8_t IDHerkulex2 = msg.data[4];
       uint16_t position2 = (msg.data[6]<<8) + (msg.data[5]);
       uint8_t playtime = msg.data[7];

       servo1.positionControl_Mul_ensemble(IDHerkulex1, position1, playtime, 0x04, IDHerkulex2, position2, 0x10);
    }
    
  ack(can, INSTRUCTION_END_HERKULEX, IDCAN_HERKULEX);
}

void controleTorque(CAN *can, CANMessage &msg){
  ack(can, ACKNOWLEDGE_HERKULEX, IDCAN_HERKULEX_Torque);
  /*                     0              1                2  */
  /*Contenue DATA : [ID du Herkulex] ; [ON/O]*/
  int IDHerkulex = msg.data[0];
  int etat = msg.data[1];
  servo1.setTorque(IDHerkulex, etat);
  status = servo1.getPos(IDHerkulex);
  printf("Status = %04X\n", status);
}

void clearHerkulex(CAN *can, CANMessage &msg){

  ack(can, ACKNOWLEDGE_HERKULEX, IDCAN_HERKULEX_Clear);
  /*                     0              1        */
  /*Contenue DATA : [ID du Herkulex]*/
  int IDHerkulex = msg.data[0];
  servo1.clear(IDHerkulex);

  status = servo1.getPos(IDHerkulex);
  printf("Status = %04X\n", status);
  printf("Herkulex clear 0\n");
}

void vitesseHerkulex(CAN *can, CANMessage &msg){
  ack(can, ACKNOWLEDGE_HERKULEX, ID_HERKULEX_VITESSE);

  uint8_t IDHerkulex1 = msg.data[0];
  uint16_t position1 = (msg.data[2]<<8) + (msg.data[1]);
  uint8_t setLed = msg.data[3];

  servo1.velocityControl(IDHerkulex1, position1, setLed);

  ack(can, INSTRUCTION_END_HERKULEX, ID_HERKULEX_VITESSE);
}

void initDriver(){
  STDBY.write(HIGH);
  EN.write(LOW); // output stage disabled
  M0.write(HIGH); M1.write(LOW); M2.write(LOW); //initial mode coniguration 
  delay (1000);
  printf("initDriver()\n"); 
}

void stepMotorPos(CAN *can, CANMessage &msg){
  ack(can, ACKNOWLEDGE_ACTIONNEURS, IDCAN_STEP_MOT_POS);
  
  int Pos = (msg.data[3]<<24) + (msg.data[2]<<16) + (msg.data[1]<<8) + (msg.data[0]); //en degrées
  printf("Control step motors position; position : %d°", Pos);
  position((int)Pos);

  ack(can, INSTRUCTION_END_ACTIONNEURS, IDCAN_STEP_MOT_POS);
}

void stepMotorMode(CAN *can, CANMessage &msg){
  ack(can, ACKNOWLEDGE_ACTIONNEURS, IDCAN_STEP_MOT_MODE);

  m0 = (msg.data[0] & 0x01);
  m1 = (msg.data[1] & 0x01);
  m2 = (msg.data[2] & 0x01); 
}

// function stepper for EVALSP820-XS
void stepper (int swpulse, int m0, int m1, int m2, int dir){
  if (m0==0) {M0.write(LOW);} else {M0.write(HIGH);}
  if (m1==0) {M1.write(LOW);} else {M1.write(HIGH);}
  if (m2==0) {M2.write(LOW);} else {M2.write(HIGH);}
  if (dir==0) {DIRE.write(LOW);} else {DIRE.write(HIGH);}
  EN.input();
  EN.mode(PullUp); // output stage  on
  delay(10);
  if (EN.read()==LOW)  {printf(" VM UVLO condition: check the VM power supply \n"); }
  while (EN.read()==LOW) {;}
  // STEP GENERATOR
  for ( int i = 0; i < swpulse; i++) {
      STEP = 1;
      STEP = 0;
      delay(1); 
  if (EN.read()==LOW)  {printf("fault  \n"); break; }
  }

  EN.output();  
  EN.write(LOW); // output stage off
  return;
}

// Function to move the motor to a specific position (in degrees)
void position(int degree) {//3600° c'est 10 tours
  if(degree == currentPosDegree){return;}
  float degreeParStep = DEGREPARSTEP;
  //      m0 == 0 && m1 == 0 && m2 == 0
  if     (m0 == 1 && m1 == 0 && m2 == 0){degreeParStep /= 2.0;}
  else if(m0 == 0 && m1 == 1 && m2 == 0){degreeParStep /= 4.0;}// 0.25 degrees per step at 1/4 microstep resolution
  else if(m0 == 1 && m1 == 1 && m2 == 0){degreeParStep /= 8.0;}
  else if(m0 == 0 && m1 == 0 && m2 == 1){degreeParStep /= 16.0;}
  else if(m0 == 1 && m1 == 0 && m2 == 1){degreeParStep /= 32.0;}
  else if(m0 == 0 && m1 == 1 && m2 == 1){degreeParStep /= 64.0;}
  else if(m0 == 1 && m1 == 1 && m2 == 1){degreeParStep /= 128.0;}

  int degreeAparcourir = degree - currentPosDegree;
  if(degreeAparcourir<0){dir = 1; degreeAparcourir *= -1;}
  else{dir = 0;}

  // Calculate the number of steps needed to reach the target position
  float steps = degreeAparcourir / degreeParStep *1.0; 
  // Update the current position of the motor
  currentPosDegree = degree;
  // Call the "stepper" function to move the motor
  stepper((int)steps, m0, m1, m2, dir); // 1/4 microstep resolution (M0 = 0, M1 = 1, M2 = 0)
}


void controlePince(CAN *can, CANMessage &msg){
  ack(can, ACKNOWLEDGE_ACTIONNEURS, IDCAN_PINCE);

  uint8_t Etage = msg.data[0];
  uint8_t etatHerkulex =((msg.data[1] == 1) ? 1 : 0); // 1-> serrer, 0 -> lacher
  signed char sens = msg.data[2];// Si sens >=0 :  Moteur pas à pas puis herkulex.  Sinon : herkulex puis Moteur pas à pas

  int Hauteur = Etage * 80.0 / 4;//0 mm, 20 mm, 40 mm, 80 mm...
  uint8_t playtime = 0;
  servo1.setTorque(IdHerkulexPince, 0x60);
  delay(2);

  if(sens>=0){
    position(Hauteur*3600.0/80.0);//Un doute ici sur la conversion en degrée
    delay(50);
    servo1.positionControl(IdHerkulexPince, position1[etatHerkulex], playtime, 0x04);
    delay(250);
  }
  else{
    servo1.positionControl(IdHerkulexPince, position1[etatHerkulex], playtime, 0x04);
    delay(300);
    position(Hauteur*3600.0/80.0);//Un doute ici sur la conversion en degrée; 8 mm -> 1 tours -> 360 °... Nan c'est bon enfaite
  }

  if(etatHerkulex != 1){servo1.setTorque(IdHerkulexPince, 0x00);}

  ack(can, INSTRUCTION_END_PINCE, IDCAN_PINCE);
}

void controlePinceArriere(CAN *can, CANMessage &msg){
  ack(can, ACKNOWLEDGE_ACTIONNEURS, IDCAN_PINCE_ARRIERE);

  uint8_t etat_pince = msg.data[0];// 0 -> fermé, 1 -> position gateau, 2 -> ouvert
  bool poseCerise =msg.data[1]; // 1-> poser, 0 -> non
  bool presenceGatoInAccount = msg.data[2];
  uint8_t playtime = 20;

  servo1.setTorque(IdHerkulexPinceArriere1, 0x60);  delay(5); servo1.setTorque(IdHerkulexPinceArriere2, 0x60); delay(5);
  
  servo1.positionControl_Mul_ensemble(IdHerkulexPinceArriere1, positionPinceArriere1[etat_pince], playtime, 0x10, IdHerkulexPinceArriere2, positionPinceArriere2[etat_pince], 0x04);
  delay(500);

  if(etat_pince == 1 && poseCerise){
    if(contact_gat_rose.read() || !presenceGatoInAccount){
      //servo
      poserCerise();
    }
  }

  delay(300);

  servo1.setTorque(IdHerkulexPinceArriere1, 0x00);  delay(5); servo1.setTorque(IdHerkulexPinceArriere2, 0x00); 

  ack(can, INSTRUCTION_END_PINCE, IDCAN_PINCE_ARRIERE);
}



void controleServo(PwmOut &servo, int pos){//in degree
  int position = (pos * 5.5 / 1) + 1000;
  servo.pulsewidth_us(position);
}

void poserCerise(){
  controleServo(servoCerise, 110);
  delay(80);
  controleServo(servoCerise, 190);
}


void ack(CAN *can, uint16_t id, uint16_t ack){
  CANMessage txMsg;
  remplirStruct(txMsg, id, 2, ack&0xFF, (ack>>8)&0xFF,0,0,0,0,0,0);
  can->write(txMsg);
}

void remplirStruct(CANMessage &msg, int idf, char lenf, char dt0f, char dt1f, char dt2f, char dt3f, char dt4f, char dt5f, char dt6f, char dt7f){
  msg.type = CANData;
  if(idf>0x7FF){msg.format = CANExtended;}
  else{msg.format = CANStandard;}
  msg.id = idf;
  msg.len = lenf;
  msg.data[0] = dt0f;
  msg.data[1] = dt1f;
  msg.data[2] = dt2f;
  msg.data[3] = dt3f;
  msg.data[4] = dt4f;
  msg.data[5] = dt5f;
  msg.data[6] = dt6f;
  msg.data[7] = dt7f;
}

void delay(float ms){
  wait_us((int)(ms*1000.0));
}