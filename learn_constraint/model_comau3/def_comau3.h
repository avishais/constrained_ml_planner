#define RR 2600 
#define RD 400 
#define PI 3.1416

#define ROBOT1_X -RR
#define ROBOT1_Y 0
#define ROBOT1_Z 0
#define ROBOT1_ROT 0

#define ROBOT2_ROT (-PI*2/3)
#define ROBOT2_X round(RR*sin(ROBOT2_ROT-PI/2))
#define ROBOT2_Y round(-RR*cos(ROBOT2_ROT-PI/2))
#define ROBOT2_Z 0

#define ROBOT3_ROT (PI*2/3)
#define ROBOT3_X round(RR*sin(ROBOT3_ROT-PI/2))
#define ROBOT3_Y round(-RR*cos(ROBOT3_ROT-PI/2))
#define ROBOT3_Z 0