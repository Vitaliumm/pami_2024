# ifndef TEAM_SELECT
# define TEAM_SELECT


# define time_evitement_depart 1/portTICK_PERIOD_MS

#ifdef PAMI_1

float x_goal = 2450; // 1=2500 2=500 3=2400  4=500
float y_goal = 800; // 1=1400 2=1600 3=1000 4=600
volatile bool go_to_1 = true ;
float x_goal_2 = 2650;// 1=3000 2=200 3=2800 4=0
float y_goal_2 = 860; // 1=1400 2=1800 3=1000 4=600
int avancement_depart = 1100 ; //1=1100 2=400 3=700 4=400
int time_start = 2000; //1 = 2000 2 = 1000 3 = 0 4 = 3000
int depart = 0 ;

# endif

#ifdef PAMI_2

float x_goal = 2500; // 1=2500 2=500 3=2400  4=500
float y_goal = 1400; // 1=1400 2=1600 3=1000 4=600
volatile bool go_to_1 = true ;
float x_goal_2 = 2970;// 1=3000 2=200 3=2800 4=0
float y_goal_2 = 1400; // 1=1400 2=1800 3=1000 4=600
int avancement_depart = 400 ; //1=1100 2=200 3=600 4=100
int time_start = 1000; //1 = 90000s 2 = 90500s 3 = 90000s 4 = 90500s
int depart = 0 ;

# endif

#ifdef PAMI_3

float x_goal = 500; // 1=2500 2=500 3=2400  4=500
float y_goal = 1600; // 1=1400 2=1600 3=1000 4=600
volatile bool go_to_1 = true ;
float x_goal_2 = 250;// 1=3000 2=200 3=2800 4=0
float y_goal_2 = 1800; // 1=1400 2=1800 3=1000 4=600
int avancement_depart = 700; //1=1100 2=200 3=600 4=100
int time_start = 0; //1 = 90000s 2 = 90500s 3 = 90000s 4 = 90500s
int depart = 0 ;

# endif

#ifdef PAMI_4

float x_goal = 500; // 1=2500 2=500 3=2400  4=500
float y_goal = 200; // 1=1400 2=1600 3=1000 4=600
volatile bool go_to_1 = true ;
float x_goal_2 = 200;// 1=3000 2=200 3=2800 4=0
float y_goal_2 = 100; // 1=1400 2=1800 3=1000 4=600
int avancement_depart = 400 ; //1=1100 2=200 3=600 4=100
int time_start = 3000; //1 = 90000s 2 = 90500s 3 = 90000s 4 = 90500s
int depart = 0 ;

# endif

# endif