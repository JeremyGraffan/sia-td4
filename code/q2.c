#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define SENSOR_NUMBER 9

#define FRONT_SENSOR_COUNT 4
#define RANGE 2000
#define BOUND(x, a, b) (((x) < (a)) ? (a) : ((x) > (b)) ? (b) : (x))

static WbDeviceTag sensors[SENSOR_NUMBER], left_motor, right_motor;

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

    if ((i + 1) >= 10) {
      sensors_name[2] = '1';
      sensors_name[3]++;

      if ((i + 1) == 10) {
        sensors_name[3] = '0';
        sensors_name[4] = '\0';
      }
    } else {
      sensors_name[2]++;
    }
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

void question_2() {
  static const int front_sensor_indexes[FRONT_SENSOR_COUNT] = { 2, 3, 4, 5 };
  static const double speed = 10;
  
  while (wb_robot_step(time_step) != -1) {
    double sensors_value[FRONT_SENSOR_COUNT];
    double current_speed = speed;

    for (int i = 0; i < FRONT_SENSOR_COUNT; i++) {
      sensors_value[i] = wb_distance_sensor_get_value(sensors[front_sensor_indexes[i]]);
      if (sensors_value[i] > 100.0 )
      {
        current_speed = 0;
        break;
      }
    }

    wb_motor_set_velocity(left_motor, current_speed);
    wb_motor_set_velocity(right_motor, current_speed);
  }
}

int main() {
  initialize();
  question_2();
  return 0;
}
