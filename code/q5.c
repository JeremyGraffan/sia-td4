#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define SENSOR_NUMBER 9

#define FRONT_SENSOR_COUNT 4
#define ALPHABOT_SENSOR_COUNT 2

#define BOUND(x, a, b) (((x) < (a)) ? (a) : ((x) > (b)) ? (b) : (x))

static WbDeviceTag sensors[SENSOR_NUMBER], left_motor, right_motor;


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

double binarize_sensor_value(double sensor_value) {
  return sensor_value > 40 ? 1 : 0;
}

enum State {
  FORWARD,
  ROTATE_RIGHT,
  ROTATE_LEFT
};

void process() {
  const int alphabot_sensor_indexes[ALPHABOT_SENSOR_COUNT] = {3, 4};
  double speed[2] = {0, 0};
  enum State state = FORWARD;

  while (wb_robot_step(time_step) != -1) {
    double sensors_value[ALPHABOT_SENSOR_COUNT];

    for (int i = 0; i < ALPHABOT_SENSOR_COUNT; i++) {
      double initial_sensor_value =
          wb_distance_sensor_get_value(sensors[alphabot_sensor_indexes[i]]);
      sensors_value[i] = binarize_sensor_value(initial_sensor_value);
    }

    if(sensors_value[0] == 0 && sensors_value[1] == 0) {
      state = FORWARD;
    } else if (sensors_value[0] == 0 && sensors_value[1] == 1) {
      state = ROTATE_LEFT;
    } else if (sensors_value[0] == 1) {
      state = ROTATE_RIGHT;
    } 

    switch (state) {
      case FORWARD:
        speed[0] = 19;  
        speed[1] = 19;  
        break;
      case ROTATE_LEFT:
        speed[0] = 0;  
        speed[1] = 19;  
        break;
      case ROTATE_RIGHT:
        speed[0] = 19;  
        speed[1] = 0;  
        break;
    }

    wb_motor_set_velocity(left_motor, speed[0]);
    wb_motor_set_velocity(right_motor, speed[1]);
  }
}

int main() {
  initialize();
  process();

  return 0;
}
