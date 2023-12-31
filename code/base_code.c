#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define SENSOR_NUMBER 9
#define RANGE 2000
#define BOUND(x, a, b) (((x) < (a)) ? (a) : ((x) > (b)) ? (b) : (x))

static WbDeviceTag sensors[SENSOR_NUMBER], left_motor, right_motor;

static const double matrix[9][2] = {
    {-2.67, -2.67},  {-10.86, 21.37}, {-16.03, 26.71},
    {-37.4, 37.4},   {37.4, -32.06},  {26.71, -21.37},
    {21.37, -10.86}, {-2.67, -2.67},  {-5.34, -5.34}};

static const int num_sensors = SENSOR_NUMBER;
static const double range = RANGE;
static int time_step = 0;
static const double max_speed = 19.1;

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

void braitenberg() {
  while (wb_robot_step(time_step) != -1) { // Run simulation
    int i, j;
    double speed[2];
    double sensors_value[num_sensors];

    for (i = 0; i < num_sensors; i++)
      sensors_value[i] = wb_distance_sensor_get_value(sensors[i]);

    for (i = 0; i < 2; i++) {
      speed[i] = 0.0;

      for (j = 0; j < num_sensors; j++) {
        speed[i] += matrix[j][i] * (1.0 - (sensors_value[j] / range));
      }

      speed[i] = BOUND(speed[i], -max_speed, max_speed);
    }

    wb_motor_set_velocity(left_motor, speed[0]);
    wb_motor_set_velocity(right_motor, speed[1]);
  }
}

int main() {
  initialize();
  braitenberg();

  return 0;
}
