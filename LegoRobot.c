#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "brick.h"
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
#include "ev3_dc.h"

#define Sleep(ms) usleep((ms)*1000)

#define MOTOR_LEFT      	OUTA
#define MOTOR_RIGHT    	    OUTB
#define MOTOR_SERVO    	    OUTC
#define MOTOR_D    		    OUTD
#define SENSOR_GYRO         IN1
#define SENSOR_2		    IN2
#define SENSOR_3		    IN3
#define SENSOR_US		    IN4

#define MOTOR_BOTH (MOTOR_LEFT | MOTOR_RIGHT)



POOL_T ultrasonicSensor;
POOL_T gyroSensor;

int max_speed_servo;
int max_speed;



void travel_backward()
{
    tacho_set_speed_sp(MOTOR_BOTH, max_speed * 0.4);
    tacho_run_forever(MOTOR_BOTH);
    Sleep(1000);
    tacho_stop(MOTOR_BOTH);
}


void CallibrateGyro()
{
    sensor_set_mode(SENSOR_GYRO, LEGO_EV3_GYRO_GYRO_RATE);
    sensor_set_mode(SENSOR_GYRO, LEGO_EV3_GYRO_GYRO_CAL);
    sensor_set_mode(SENSOR_GYRO, LEGO_EV3_GYRO_GYRO_ANG);
}

void TravelForward()
{
    tacho_set_speed_sp(MOTOR_BOTH, max_speed * 0.4);
    tacho_set_position_sp(MOTOR_BOTH, -5115);
    tacho_run_to_rel_pos(MOTOR_BOTH);
    Sleep(10000);
}
//Lämnar paketet
void Throw_package()
{
    tacho_set_speed_sp(MOTOR_SERVO, max_speed_servo * -1.0);
    tacho_run_forever(MOTOR_SERVO);
    Sleep(800);
    tacho_stop(MOTOR_SERVO);
}

// Svänger till det specifika graden
void TurnTowards(int deg)
{

    int sense_deg = sensor_get_value(0, gyroSensor, 0);

    while (abs(sensor_get_value(0, gyroSensor, 0) - (sense_deg + deg)) > 2)     //Medan den uppfyllda graden (+- 2 graders marginal) inte uppfyllts kommer roboten att fortsätta snurra.
    {
        if (sensor_get_value(0, gyroSensor, 0) > (sense_deg + deg))   //Kommer vända sig till sidan med kortast avstånd till den sökande graden.
        {
            tacho_set_speed_sp(MOTOR_LEFT, max_speed * -0.3);
            tacho_set_speed_sp(MOTOR_RIGHT, max_speed * 0.3);
            tacho_run_forever(MOTOR_BOTH);
        }
        else if (sensor_get_value(0, gyroSensor, 0) < (sense_deg + deg))
        {
            tacho_set_speed_sp(MOTOR_LEFT, max_speed * 0.3);
            tacho_set_speed_sp(MOTOR_RIGHT, max_speed * -0.3);
            tacho_run_forever(MOTOR_BOTH);
        }
        Sleep(80);     // Väntar med 50 millisekunder innan sensorn registrerar nytt värde.
    }
    tacho_stop(MOTOR_BOTH);
    Sleep(1000);
}

void TravelToWall()
{ 
    // Om distansen är mindre än 30cm ger den ett istället ett värde på 250cm och åker bakåt tills 30cm är uppmätt från väggen
    if (sensor_get_value0(ultrasonicSensor, 0) < 290)
    {
        tacho_set_speed_sp(MOTOR_BOTH, max_speed * 0.3);
        tacho_run_forever(MOTOR_BOTH);
        while (sensor_get_value0(ultrasonicSensor, 0) < 290 || sensor_get_value0(ultrasonicSensor, 0) == 2500)
        {
            Sleep(200);
        }       
        tacho_stop(MOTOR_BOTH);
    }        
    // Om distansen är större än 31cm ger den ett istället ett värde på 250cm och åker frammåt tills 30cm är uppmätt från väggen
    else if (sensor_get_value0(ultrasonicSensor, 0) > 310)
    {
            tacho_set_speed_sp(MOTOR_BOTH, max_speed * -0.3);
            tacho_run_forever(MOTOR_BOTH);
            while (sensor_get_value0(ultrasonicSensor, 0)  > 310 || sensor_get_value0(ultrasonicSensor, 0) == 2500)
            {
                Sleep(200);
            }
            tacho_stop(MOTOR_BOTH);
    }
}

// Skannings funktion som registrerar den kortaste distanssen och grad till väggen.
void Scan()
{    
    int wall_min_deg;
    int initial_Degree = sensor_get_value(0, gyroSensor, 0);    // Sätter start värde på gyro sensor
    float min_dist = sensor_get_value0(ultrasonicSensor, 0);    // Sätter ett värde på minsta avståd

    tacho_set_speed_sp(MOTOR_LEFT, max_speed * 0.3);
    tacho_set_speed_sp(MOTOR_RIGHT, max_speed * -0.3);

    tacho_run_forever(MOTOR_LEFT);
    tacho_run_forever(MOTOR_RIGHT); 


    while (initial_Degree + 360 > sensor_get_value(0, gyroSensor, 0))   // Läser av gyro sensorns värde medan roboten har gjort ett helt varv. 
    {
        if (sensor_get_value0(ultrasonicSensor, 0) <= min_dist)  // Om ultraljuds sensorns värde är mindre än start värdet sätter ultra sensor ett nytt startvärde.
        {
            min_dist = sensor_get_value0(ultrasonicSensor, 0);  // Sätter ett nytt startvärde för distans
            wall_min_deg = sensor_get_value(0, gyroSensor, 0);  // Ger ett nytt gradvärde för väggen           
        }
        Sleep(200);   // Väntar 200 millisekunder innan sensorn registrerar på nytt.
    }

    tacho_stop(MOTOR_BOTH);
    Sleep(80);

    TurnTowards(wall_min_deg - sensor_get_value(0, gyroSensor, 0));     // Skickar graden för kortast distans grad minus gyro sensorns nuvarande värde för att kunna svänga till just den grade. 
    TravelToWall();           
}

//Nedan är programkodenerna för respektive leveransadress
void Route_1()
{
    CallibrateGyro();
    Scan();
    Sleep(300);
    TurnTowards(90);
    Sleep(300);
    TravelForward();
    Sleep(300);
    TurnTowards(-90);
    Sleep(300);
    TravelToWall();
    Sleep(300);
    Throw_package();
    Sleep(300);
    
    travel_backward();
    TurnTowards(-90);
    TravelForward();
    Scan();
    TurnTowards(-180);
    
}

void Route_2()
{
    CallibrateGyro();
    Scan();
    Sleep(300);
    TurnTowards(-90);
    Sleep(300);
    TravelForward();
    Sleep(300);
    TurnTowards(90);
    Sleep(300);
    TravelToWall();
    Sleep(300);
    Throw_package();
    Sleep(300);

    travel_backward();
    TurnTowards(90);
    TravelForward();
    Scan();
    TurnTowards(-180);
}

void Route_3()
{
    CallibrateGyro();
    Scan();
    Sleep(300);
    TurnTowards(-180);
    Sleep(300);
    TravelToWall();
    Sleep(300);
    TurnTowards(-90);
    Sleep(300);
    TravelForward();
    Sleep(300);
    TurnTowards(90);
    Sleep(300);
    TravelToWall();
    Sleep(300);
    Throw_package();
    Sleep(300);

    travel_backward();
    TurnTowards(-180);
    TravelForward();
    TurnTowards(-90);
    TravelForward();
    Scan();
    TurnTowards(-180);
}

void Route_4()
{
    CallibrateGyro();
    Scan();
    Sleep(300);
    TurnTowards(-180);
    Sleep(300);
    TravelToWall();
    Sleep(300);
    TurnTowards(90);
    Sleep(300);
    TravelForward();
    Sleep(300);
    TurnTowards(-90);
    Sleep(300);
    TravelToWall();
    Sleep(300);
    Throw_package();
    Sleep(300);

    travel_backward();
    TurnTowards(-180);
    TravelForward();
    TurnTowards(90);
    TravelForward();
    Scan();
    TurnTowards(-180);
}

// Funktion spm innehåller en switch funktion där användaren får välja rutt.
void Choose_Route() 
{
    char route; 
    printf("Which package route would you like to make (A, B, C or D)?  \n");
    scanf("%c", &route); 

    switch (route)
    {
    case 'A':
        Route_1();
        break;

    case 'B':
        Route_2();
        break;

    case 'C':
        Route_3();
        break;

    case 'D':
        Route_4();
        break;

    default:
        printf("No route chosen. \n");
        break;
    }
} 


int main(void)
{     
    //Startar roboten

    if (!brick_init()) return (1);  printf("*** ( EV3 ) Hello! ***\n");  Sleep(2000);        
    
    ultrasonicSensor = sensor_search(LEGO_EV3_US);
    us_set_mode_us_dist_cm(ultrasonicSensor);
    gyroSensor = sensor_search(LEGO_EV3_GYRO);
    gyro_set_mode_gyro_ang(gyroSensor);    
    
    if (tacho_is_plugged(MOTOR_BOTH, TACHO_TYPE__NONE_))
    {
        max_speed = tacho_get_max_speed(MOTOR_LEFT, 0);	    // Kollar maxhastigheten som motorn kan ha 
        tacho_reset(MOTOR_BOTH);
        max_speed_servo = tacho_get_max_speed(MOTOR_SERVO, 0);
    }
    else
    {
        printf("Anslut vänster motor i port A,\n" "Anslut höger motor i port B.\n");
        brick_uninit();
        return (0);  // Stänger av sig om motorer ej är inkopplade 
    }    

    Choose_Route(); // Startar programmet om att välja rutt.

    brick_uninit();
    printf("*** ( EV3 ) Bye! ***\n");
    return (0);
}
