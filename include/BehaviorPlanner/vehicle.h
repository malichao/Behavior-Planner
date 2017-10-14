#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

class Vehicle {
 public:
  struct collider {
    bool collision;  // is there a collision?
    int time;        // time collision happens
  };

  struct Pose {
    int lane;
    int s;
    int v;
    int a;
  };

  typedef vector<Pose> Trajectory;

  int L = 1;

  int preferred_buffer = 6;  // impacts "keep lane" behavior.

  int lane;

  int s;

  int v;

  int a;

  int target_speed;

  int lanes_available;

  int max_acceleration;

  int goal_lane;

  int goal_s;

  string state;

  /**
  * Constructor
  */
  Vehicle(int lane, int s, int v, int a);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  const Pose GetPose(int at_time = 0) const;

  void SetPose(Pose pose);

  void update_state(map<int, vector<Pose> > predictions);

  void configure(vector<int> road_data);

  string display();

  void increment(int dt);

  vector<int> state_at(int t) const;

  bool collides_with(Pose other_pose, int at_time);

  collider will_collide_with(Pose other, int timesteps);

  void realize_state(map<int, vector<Pose> > predictions);

  void realize_constant_speed();

  int _max_accel_for_lane(map<int, vector<Pose> > predictions, int lane, int s);

  void realize_keep_lane(map<int, vector<Pose> > predictions);

  void realize_lane_change(map<int, vector<Pose> > predictions,
                           string direction);

  void realize_prep_lane_change(map<int, vector<Pose> > predictions,
                                string direction);

  vector<Pose> generate_predictions(int horizon);

  vector<Pose> FilterPrediction(const map<int, vector<Pose> > &predictions);

  Trajectory GenerateTrajectory(map<int, vector<Pose> > predictions,
                                string state, Pose pose, int horizon = 9) const;
};

#endif
