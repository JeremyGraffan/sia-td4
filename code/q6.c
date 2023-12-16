#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define SENSOR_NUMBER 9
#define SENSOR_PER_SIDE 3

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

#define STICKYNESS 20

enum State {
  AVOIDING,
  FORWARD
};

void process() {
  enum State state = FORWARD;

  double CRITICAL_DISTANCE = 60;
  int obstacle_avoidance_counter = 0;

  while (wb_robot_step(time_step) != -1) {
    double speed[2] = {0, 0};
    double sensors_value[SENSOR_NUMBER];
    for (int i = 0; i < SENSOR_NUMBER; i++) {
      sensors_value[i] = wb_distance_sensor_get_value(sensors[i]);
    }

    double max_right = fmax(sensors_value[4], fmax(sensors_value[5], sensors_value[6]));
    double max_left = fmax(sensors_value[1], fmax(sensors_value[2], sensors_value[3]));

    if (max_right > CRITICAL_DISTANCE || max_left > CRITICAL_DISTANCE) {
      state = AVOIDING;
      obstacle_avoidance_counter = STICKYNESS;
    } else if(obstacle_avoidance_counter <= 0){
        state = FORWARD;
    }

    if (state == AVOIDING) {
      if (max_left > CRITICAL_DISTANCE) {
          speed[1] = 10.0;
          speed[0] = -10.0;
      } else if (max_right > CRITICAL_DISTANCE) {
          speed[1] = 10.0;
          speed[0] = -10.0;
      } else {
        speed[0] = 12.0;
        speed[1] = 3.0;
      }
      obstacle_avoidance_counter--;
    } 

    if(state == FORWARD) {
        speed[0] = 10.0;
        speed[1] = 10.0;
    }

    speed[0] = BOUND(speed[0], -max_speed, max_speed);
    speed[1] = BOUND(speed[1], -max_speed, max_speed);

    wb_motor_set_velocity(left_motor, speed[0]);
    wb_motor_set_velocity(right_motor, speed[1]);
  }
}

int main() {
  initialize();
  process();

  return 0;
}
