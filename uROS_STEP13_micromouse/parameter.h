#define TIRE_DIAMETER (48.1)  //47.55
#define TREAD_WIDTH (66.0)    //67.1
#define TREAD_CIRCUIT (TREAD_WIDTH * PI / 4.0)
#define PULSE (TIRE_DIAMETER * PI / 400.0)
#define MIN_HZ 80.0

#define WAITLOOP_SLED 10

#define REF_SEN_R 500
#define REF_SEN_L 660

#define TH_SEN_R 150
#define TH_SEN_L 350
#define TH_SEN_FR 100  //103
#define TH_SEN_FL 150

#define CONTROL_TH_SEN_R TH_SEN_R
#define CONTROL_TH_SEN_L TH_SEN_L

#define CON_WALL_KP 0.08
#define SEARCH_ACCEL 1.5
#define TURN_ACCEL 0.3

#define SEARCH_SPEED 350
#define MAX_SPEED 1000
#define MIN_SPEED (MIN_HZ * PULSE)

#define GOAL_X 2
#define GOAL_Y 6

#define INC_FREQ 3000
#define DEC_FREQ 2000

#define BATT_MAX 12000
#define BATT_MIN 10000

#define HALF_SECTION 90
#define SECTION 180
