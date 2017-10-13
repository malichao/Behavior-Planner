#include <BehaviorPlanner/road.h>
#include <BehaviorPlanner/vehicle.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <highgui.h>
#include <BehaviorPlanner/gif.h>

void DrawImage(vector<vector<string>>& str_img, int step) {
  // OpenCV coordinate system:
  // 0 ---- x
  // |
  // |
  // y

  // Road coordinate system:
  // 0 ----lane(0,1,2,3...)
  // |
  // |
  // distance

  cv::Mat image = cv::Mat::zeros(640, 320, CV_8UC1);
  image.setTo(cv::Scalar(255));
  static double lane_ratio = 60;
  static double col_ratio = 15;
  // Draw Lane lines
  cv::Point offset(50, 10);
  for (auto lane : {0, 1, 2, 3, 4}) {
    cv::Point start(lane * lane_ratio, 20);
    cv::Point end(lane * lane_ratio, 600);
    cv::line(image, start + offset, end + offset, cv::Scalar(0));
  }

  // Draw car IDs
  for (int col = 0; col < str_img.size(); col++) {
    for (int lane = 0; lane < str_img[col].size(); lane++) {
      cv::Point pos(lane * lane_ratio + lane_ratio / 5, col * col_ratio + 16);
      cv::putText(image, str_img[col][lane], pos + offset,  // Coordinates
                  cv::FONT_HERSHEY_DUPLEX,                  // Font
                  0.5,            // Scale. 2.0 = 2x bigger
                  cv::Scalar(0),  // Color
                  1);             // Thickness
    }
  }
  cv::imwrite("images/step_" + to_string(step) + ".jpg", image);
}

using namespace std;

// impacts default behavior for most states
int SPEED_LIMIT = 10;

// all traffic in lane (besides ego) follow these speeds
vector<int> LANE_SPEEDS = {6, 7, 8, 9};

// Number of available "cells" which should have traffic
double TRAFFIC_DENSITY = 0.15;

// At each timestep, ego can set acceleration to value between
// -MAX_ACCEL and MAX_ACCEL
int MAX_ACCEL = 2;

// s value and lane number of goal.
vector<int> GOAL = {300, 0};

// These affect the visualization
int FRAMES_PER_SECOND = 4;
int AMOUNT_OF_ROAD_VISIBLE = 40;

int main() {
  Road road = Road(SPEED_LIMIT, TRAFFIC_DENSITY, LANE_SPEEDS);

  road.update_width = AMOUNT_OF_ROAD_VISIBLE;

  road.populate_traffic();

  int goal_s = 300;
  int goal_lane = 0;

  // configuration data: speed limit, num_lanes, goal_s, goal_lane,
  // max_acceleration

  int num_lanes = LANE_SPEEDS.size();
  vector<int> ego_config = {SPEED_LIMIT, num_lanes, goal_s, goal_lane,
                            MAX_ACCEL};

  road.add_ego(2, 0, ego_config);
  int timestep = 0;

  // Create a folder to save test images
  system("mkdir images");

  while (road.get_ego().s <= GOAL[0]) {
    timestep++;
    if (timestep > 35) {
      break;
    }
    road.advance();
    auto str_img = road.display(timestep);
    DrawImage(str_img, timestep);
    // time.sleep(float(1.0) / FRAMES_PER_SECOND);
  }
  Vehicle ego = road.get_ego();
  if (ego.lane == GOAL[1]) {
    cout << "You got to the goal in " << timestep << " seconds!" << endl;
    if (timestep > 35) {
      cout << "But it took too long to reach the goal. Go faster!" << endl;
    }
  } else {
    cout << "You missed the goal. You are in lane " << ego.lane
         << " instead of " << GOAL[1] << "." << endl;
  }
  return 0;
}
