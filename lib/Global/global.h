#include <mbed.h>
#include <herkulex.h>
#include <identCrac.h>

#define LOW 0
#define HIGH 1


void controleHerkulex(CAN *can, CANMessage &msg);
void controleTorque(CAN *can, CANMessage &msg);
void clearHerkulex(CAN *can, CANMessage &msg);
void vitesseHerkulex(CAN *can, CANMessage &msg);
void initDriver();
void stepMotorPos(CAN *can, CANMessage &msg);
void stepMotorMode(CAN *can, CANMessage &msg);
void stepper (int swpulse, int m0, int m1, int m2, int dir);
void position(int degree);
void controlePince(CAN *can, CANMessage &msg);
void controlePinceArriere(CAN *can, CANMessage &msg);

void controleServo(PwmOut &servo, int pos);
void poserCerise();

void printCANMsg(CANMessage& msg);
void remplirStruct(CANMessage &msg, int idf, char lenf, char dt0f, char dt1f, char dt2f, char dt3f, char dt4f, char dt5f, char dt6f, char dt7f);
void ack(CAN *can, uint16_t id, uint16_t ack);
void delay(float ms);