#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define SENSOR_COUNT 4

static WbDeviceTag front_sensors[SENSOR_COUNT];
static WbDeviceTag left_motor, right_motor;

static int time_step = 0;
static const double speed = 10;

static void initialize() {
  wb_robot_init();

  time_step = wb_robot_get_basic_time_step();

  char sensors_name[4];
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sprintf(sensors_name, "ds%d", i+2);
    front_sensors[i] = wb_robot_get_device(sensors_name);
    wb_distance_sensor_enable(front_sensors[i], time_step);
  }

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
}

void process() {
  while (wb_robot_step(time_step) != -1) {
    double sensors_value[SENSOR_COUNT];
    double current_speed = speed;

    for (int i = 0; i < SENSOR_COUNT; i++) {
      sensors_value[i] = wb_distance_sensor_get_value(front_sensors[i]);
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
  process();
  wb_robot_cleanup();
  return 0;
}
