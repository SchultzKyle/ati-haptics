#include <stdio.h>
#include <math.h>
#include "dhdc.h"
#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>

class AtiHaptics {
  public:
    
    // haptics device parameters
    double origin[3] = {0.0, 0.0, 0.0}; // origin
    double K[3] = {170.0, 170.0, 170.0}; // haptic device spring constants to origin
    double B[3] = {9.0, 9.0, 9.0}; // haptic device damping constants to origin
    double sat_force = 18.0; // saturation force
    double force_sensor_weight_vertical = 5.0; // multiplier for spring force, 12 N applied by person / 0.07 m on haptic device = 170 * multiplier
    double force_sensor_weight_horizontal = 5.0; // multiplier for spring force, 12 N applied by person / 0.07 m on haptic device = 170 * multiplier

    // haptics device feedback:
    double p[3]; // position
    double o[3]; // orientation
    double v[3]; // velocity
    double force[3]={0.0, 0.0, 0.0};
    double gripper;

    void callback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
      double ati_x_force;
      double ati_y_force;
      double ati_z_force;

      // read ati sensor:
      ati_x_force = msg->data[0];
      ati_y_force = msg->data[1];
      ati_z_force = msg->data[2];

      //read haptic device states:
      dhdGetPosition(&(p[0]), &(p[1]), &(p[2]));
      dhdGetOrientationRad(&(o[0]), &(o[1]), &(o[2]));
      dhdGetLinearVelocity (&(v[0]), &(v[1]), &(v[2]));
      dhdGetGripperGap(&gripper);
      // printf("x: %lf, y: %lf, z: %lf\n",p[0], p[1], p[2]);

      // set haptic device spring-damper force:
      for(int i=0; i<3; i++) {
        force[i] = K[i]*(origin[i]-p[i]) - B[i]*v[i];
      }

      // set haptic device force from ati measurement: 
      force[0] -= ati_x_force * force_sensor_weight_horizontal;
      force[1] -= ati_y_force * force_sensor_weight_horizontal;
      force[2] -= ati_z_force * force_sensor_weight_vertical;

      // saturate haptic device force:
      for(int i=0; i<3; i++) {
        force[i] = std::min(std::max(force[i], -sat_force), sat_force); // saturation
      }

      // send force to haptic device:
      dhdSetForceAndTorqueAndGripperForce(force[0],force[1],force[2],0.0,0.0,0.0,0.0);
    }
};

int main (int argc,char **argv) {

  // initialize haptics device:
  if (dhdOpen () < 0) {
    printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
    dhdSleep (2.0);
    return -1;
  }
  printf ("%s device detected\n\n", dhdGetSystemName());
  dhdEnableForce (DHD_ON);

  // ROS node
  ros::init(argc, argv, "ati_haptics");
  ros::NodeHandle n;
  AtiHaptics ati_haptics;
  ros::Subscriber sub = n.subscribe("/ati/forces", 1, &AtiHaptics::callback, &ati_haptics);
  ros::spin();
  return 0;

}
