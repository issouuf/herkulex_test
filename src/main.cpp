#include <mbed.h>
#include <herkulex.h>

#define ALL_HERKULEX 0xFE


#define HERKULEX1 0x01
#define HERKULEX2 0x02

#define STEP_TORQUE_ON 0x01
#define STEP_TORQUE_OFF 0x00
#define TEST_STEPPER 0x02
#define MOVE_STEPPER 0x03
#define OPEN_GRIPPER 0x04
#define CLOSE_GRIPPER 0x05


#define HERKULEX_INIT 0x21

DigitalOut STBY(D7);
DigitalOut STEP(D6);
DigitalOut DIr(D3);
DigitalOut EN(A2);
DigitalOut M0(D5);
DigitalOut M1(D9);
DigitalOut M2(D8);






Herkulex servo(PB_6, PB_7, 115200); 

CAN can(PA_11, PA_12,1000000);

int stepper(int swpulse, int m0, int m1, int m2, int dir, int dur);
void clockwise(void);
void counterclockwise(void);
void blockStepper(void);
void initStepper(void);


void initialisation_herkulex(void);

void movePosition(uint16_t nbrPas, uint8_t direction);




int main() {

    // servo.clear(ALL_HERKULEX);
    // servo.setTorque(ALL_HERKULEX, 0X60);
    //initialisation_herkulex();
    //servo.setTorque(HERKULEX1, TORQUE_ON);
    //servo.setTorque(HERKULEX2, TORQUE_ON);
    
    // STBY = 1;
    // EN = 1;
    // M0 = 0;
    // M1 = 0;
    // M2 = 0;
    // initStepper();
    // stepper(1000, 0, 0, 0, 1, 1);
    // blockStepper();

        servo.setTorque(0xFD,0x00);  
        servo.clear(0xFD);


    while (1) {
        // ThisThread::sleep_for(2000ms);
        // servo.positionControl(HERKULEX1, 1023, 100,BLED_ON);
        // servo.positionControl(HERKULEX2, 21, 80,BLED_ON);

        // //servo.positionControl(HERKULEX2, 1000, 100,GLED_ON);
        // //clockwise();
        // ThisThread::sleep_for(2000ms);
        // servo.positionControl(HERKULEX1, 720, 100,GLED_ON);
        // servo.positionControl(HERKULEX2, 300, 80,GLED_ON);

        //servo.positionControl(HERKULEX2, 100, 100,RLED_ON);
        //counterclockwise();
        //int position = servo.getPos(0xFD);
        //printf("Position: %d\n", position);
        int status = servo.getStatus(0xFD);
        printf("Status: %d\n", status); 
        ThisThread::sleep_for(2000ms);

            
        CANMessage msg;
        if (can.read(msg)){

            printf("Message reçu avec l'ID: %d \n", msg.id);
            switch (msg.id)
            {
                case STEP_TORQUE_ON:
                    blockStepper();
                    break;
                case STEP_TORQUE_OFF:
                    initStepper();
                    break;
                case TEST_STEPPER:
                    clockwise();
                    break;  
                case MOVE_STEPPER:
                if (msg.len == 3){
                    
                    movePosition((msg.data[0] << 8) | msg.data[1], msg.data[2]);
                    blockStepper();
                }else {
                    printf("Erreur dans la taille du message\n");
                }
                    break;
                case HERKULEX_INIT:
                    servo.positionControl(ALL_HERKULEX, 1000, 100,BLED_ON);
                    break;
                case OPEN_GRIPPER:
                    servo.positionControl(HERKULEX1, 1023, 100,BLED_ON);
                    servo.positionControl(HERKULEX2, 21, 80,BLED_ON);
                    break;
                case CLOSE_GRIPPER:
                    servo.positionControl(HERKULEX1, 720, 100,GLED_ON);
                    servo.positionControl(HERKULEX2, 300, 80,GLED_ON);
                    break;
        }
     }
    }
}


void initialisation_herkulex(void){
    servo.clear(ALL_HERKULEX);
    servo.setTorque(ALL_HERKULEX, TORQUE_ON);
    servo.positionControl(HERKULEX1, 720, 100,BLED_ON);
    servo.positionControl(HERKULEX2, 300, 100,RLED_ON);
}








int stepper(int swpulse, int m0, int m1, int m2, int dir, int dur)
{
    M0 = m0;
    M1 = m1;
    M2 = m2;
    DIr = dir;
    EN = 1;
    // step generator
    for (int i = 0; i < swpulse; i++)
    {
        STEP = 1;
        ThisThread::sleep_for(1ms);
        STEP = 0;
        ThisThread::sleep_for(1ms);
    }
    EN = 0;
    return 0;
}

void movePosition(uint16_t nbrPas, uint8_t direction){ //uint8_t vitesse
    stepper(nbrPas, 0, 0, 0, direction, 1);
}

void clockwise(void)
{
    stepper(1000, 0, 0, 0, 1, 1);
}

void counterclockwise(void)
{
    stepper(1000, 0, 0, 0, 0, 1);
}
void blockStepper(void){
    STBY = 1;
    EN =1;
    M0 = 1;
    M1 = 1;
    M2 = 1;

}
void initStepper(void){
    STBY = 1;
    EN = 0;
    M0 = 0;
    M1 = 0;
    M2 = 0;
}