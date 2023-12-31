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

void go_forward(double *speeds) {
  speeds[LEFT] = 7.0;
  speeds[RIGHT] = 7.0;
}

void obstacle_stop(double *sensors_value, double *veto_below, double *veto_above) {
  double CRITICAL_DISTANCE = 40;
  double max_right =  fmax(sensors_value[4], fmax(sensors_value[5], sensors_value[6]));
  double max_left = fmax(sensors_value[1], fmax(sensors_value[2], sensors_value[3]));
  if (max_left > CRITICAL_DISTANCE) {
    veto_above[LEFT] = 0;
  } 
  if (max_right > CRITICAL_DISTANCE) {
    veto_above[RIGHT] = 0;
  }
}


void apply_vetos(double *speeds, double *veto_below, double *veto_above) {
  if(speeds[LEFT] > veto_above[LEFT]) {
    speeds[LEFT] = veto_above[LEFT];
  }
  if(speeds[RIGHT] > veto_above[RIGHT]) {
    speeds[RIGHT] = veto_above[RIGHT];
  }
  if(speeds[LEFT] < veto_below[LEFT]) {
    speeds[LEFT] = veto_below[LEFT];
  }
  if(speeds[RIGHT] < veto_below[RIGHT]) {
    speeds[RIGHT] = veto_below[RIGHT];
  }
}

void avoid_right(double *sensors_value, double *speeds) {
  double AVOID_DISTANCE = 25;
  double max_right =  fmax(sensors_value[4], fmax(sensors_value[5], sensors_value[6]));
  double max_left = fmax(sensors_value[1], fmax(sensors_value[2], sensors_value[3]));

  if (max_left > AVOID_DISTANCE || max_right > AVOID_DISTANCE) {
    speeds[LEFT] = -10.0;
    speeds[RIGHT] = 10.0;
  } 
}

void vote_architecture() {
  double speed[2];
  double proposed_speeds[2];

  while (wb_robot_step(time_step) != -1) {
    double sensors_value[SENSOR_NUMBER];
    double veto_above[2] = {INFINITY, INFINITY};
    double veto_below[2] = {-INFINITY, -INFINITY};
    for (int i = 0; i < SENSOR_NUMBER; i++) {
      sensors_value[i] = wb_distance_sensor_get_value(sensors[i]);
    }

    go_forward(proposed_speeds);
    speed[LEFT] = proposed_speeds[LEFT];
    speed[RIGHT] = proposed_speeds[RIGHT];

    avoid_right(sensors_value, proposed_speeds);
    speed[LEFT] += proposed_speeds[LEFT];
    speed[RIGHT] += proposed_speeds[RIGHT];

    obstacle_stop(sensors_value, veto_below, veto_above);
    apply_vetos(speed, veto_below, veto_above);

    speed[LEFT] = BOUND(speed[LEFT], -max_speed, max_speed);
    speed[RIGHT] = BOUND(speed[RIGHT], -max_speed, max_speed);

    wb_motor_set_velocity(left_motor, speed[LEFT]);
    wb_motor_set_velocity(right_motor, speed[RIGHT]);
  }
}

int main() {
  initialize();
  vote_architecture();

  return 0;
}
