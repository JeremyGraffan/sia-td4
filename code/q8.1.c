#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define SENSOR_NUMBER 9
#define SENSOR_PER_SIDE 3

#define LEFT 0
#define RIGHT 1

#define BOUND(x, a, b) (((x) < (a)) ? (a) : ((x) > (b)) ? (b) : (x))

static WbDeviceTag sensors[SENSOR_NUMBER], left_motor, right_motor;

static const double max_speed = 19.1;

static const int num_sensors = SENSOR_NUMBER;
static int time_step = 0;

void enable_sensors() {
  char sensors_name[5];
  sprintf(sensors_name, "%s", "ds0");
  int i;

  for (i = 0; i < num_sensors; i++) {
    sensors[i] = wb_robot_get_device(sensors_name);
    wb_distance_sensor_enable(sensors[i], time_step);
    sensors_name[2]++;
  }
}

static void initialize() {
  wb_robot_init();

  time_step = wb_robot_get_basic_time_step();

  enable_sensors();

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  const char *robot_name = wb_robot_get_name();
  printf("The %s robot is initialized, it uses %d distance sensors\n",
         robot_name, num_sensors);
}

#define WANDER_DURATION 100
#define WANDER_COOLDOWN 500

void go_forward(double *speeds) {
  speeds[LEFT] = 12.0;
  speeds[RIGHT] = 12.0;
}

void wander(double *speeds, int *wander_remaining, int *wander_direction, int *wander_cooldown) {
  if(*wander_remaining > 0) {
    speeds[LEFT] = *wander_direction ? 0 : 10;
    speeds[RIGHT] = *wander_direction ? 10 : 0;
    *wander_remaining -= 1;
  } else if(*wander_cooldown > 0) {
    *wander_cooldown -= 1;
  } else {
    *wander_direction = rand() & 1;
    *wander_remaining = WANDER_DURATION;
    *wander_cooldown = WANDER_COOLDOWN;
  }
}

void obstacle_back(double *sensors_value, double *speeds, double *proposed_speeds) {
  double CRITICAL_DISTANCE = 50;
  double max_right =  fmax(sensors_value[4], fmax(sensors_value[5], sensors_value[6]));
  double max_left = fmax(sensors_value[1], fmax(sensors_value[2], sensors_value[3]));

  speeds[LEFT] = proposed_speeds[LEFT];
  speeds[RIGHT] = proposed_speeds[RIGHT];

  if (max_left > CRITICAL_DISTANCE) {
    speeds[LEFT] = -3;
  } 
  if (max_right > CRITICAL_DISTANCE) {
    speeds[RIGHT] = -3;
  }
}

void avoid_right(double *sensors_value, double *speeds) {
  double AVOID_DISTANCE = 30;
  double max_right =  fmax(sensors_value[4], fmax(sensors_value[5], sensors_value[6]));
  double max_left = fmax(sensors_value[1], fmax(sensors_value[2], sensors_value[3]));

  if (max_left > AVOID_DISTANCE || max_right > AVOID_DISTANCE) {
    speeds[LEFT] = -10.0;
    speeds[RIGHT] = 10.0;
  } 
}

void subsumption_architecture() {
  double speed[2] = {0, 0};
  double proposed_speeds[2] = {0, 0};
  int wander_cooldown = WANDER_COOLDOWN;
  int wander_remaining = 0;
  int wander_direction = 0;

  while (wb_robot_step(time_step) != -1) {
    double sensors_value[SENSOR_NUMBER];
    for (int i = 0; i < SENSOR_NUMBER; i++) {
      sensors_value[i] = wb_distance_sensor_get_value(sensors[i]);
    }

    go_forward(proposed_speeds);
    obstacle_back(sensors_value, speed, proposed_speeds);

    wander(proposed_speeds, &wander_remaining, &wander_direction, &wander_cooldown);
    obstacle_back(sensors_value, speed, proposed_speeds);

    avoid_right(sensors_value, proposed_speeds);
    obstacle_back(sensors_value, speed, proposed_speeds);

    speed[LEFT] = BOUND(speed[LEFT], -max_speed, max_speed);
    speed[RIGHT] = BOUND(speed[RIGHT], -max_speed, max_speed);

    wb_motor_set_velocity(left_motor, speed[LEFT]);
    wb_motor_set_velocity(right_motor, speed[RIGHT]);
  }
}

int main() {
  initialize();
  subsumption_architecture();

  return 0;
}
