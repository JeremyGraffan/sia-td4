#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define RANGE 512
#define BOUND(x, a, b) (((x) < (a)) ? (a) : ((x) > (b)) ? (b) : (x))

static WbDeviceTag sensors[9];
static WbDeviceTag left_motor, right_motor;

static const double weights[9][2] = {
                                      {-2.67, -2.67},  {-10.86, 21.37}, {-16.03, 26.71}, 
                                      {-37.4, 37.4},   {37.4, -32.06},  {26.71, -21.37}, 
                                      {21.37, -10.86}, {-2.67, -2.67},  {-5.34, -5.34}
                                    };


static const int num_sensors = 9;
static const double range = 2000.0;
static int time_step = 0;
static const double max_speed = 20;

static void initialize() {
  wb_robot_init();

  time_step = wb_robot_get_basic_time_step();

  char sensors_name[4];
  for (int i = 0; i < num_sensors; i++) {
    sprintf(sensors_name, "ds%d", i);
    sensors[i] = wb_robot_get_device(sensors_name);
    wb_distance_sensor_enable(sensors[i], time_step);
  }

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  const char *robot_name = wb_robot_get_name();
  printf("The %s robot is initialized, it uses %d distance sensors\n", robot_name, num_sensors);
}

void process() {
  while (wb_robot_step(time_step) != -1) {
    int i, j;
    double speed[2];
    double sensors_value[num_sensors];

    for (i = 0; i < num_sensors; i++) {
      sensors_value[i] = wb_distance_sensor_get_value(sensors[i]);
      printf("Sensor %d value: %f\n", i, sensors_value[i]);
    }

    for (i = 0; i < 2; i++) {
      speed[i] = 0.0;

      for (j = 0; j < num_sensors; j++) {
        speed[i] += weights[j][i] * (1.0 - (sensors_value[j] / range));
      }

      speed[i] = BOUND(speed[i], -max_speed, max_speed);
    }

    wb_motor_set_velocity(left_motor, speed[0]);
    wb_motor_set_velocity(right_motor, speed[1]);
  }
}


int main() {
  initialize();
  process();
  wb_robot_cleanup();
  return 0;
}
