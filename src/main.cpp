#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "utils.h"

using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;


  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  // starting conditions
  double ref_vel = 0.0; // starting / reference velocity mph
  double lane = 1; // starting in lane 1
  double speed_limit = 50.0; // max speed mph

  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &speed_limit]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];


          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();

          if (prev_size > 0) {
            car_s = end_path_s;
          }

          double dist_max = 20; // distance from cars allowed to other cars
          double velocity_to_hit_ahead = speed_limit - 0.5;
          double closest_car_ahead_s = 30;
          double velocity_to_hit_left = speed_limit - 0.5;
          double closest_car_left_s = 30;
          double velocity_to_hit_right = speed_limit - 0.5;
          double closest_car_right_s = 30;
          bool too_close = false;
          bool left_open = true;
          bool right_open = true;
          bool prepare_left = false;
          bool prepare_right = false;
          // find ref_v to use
          for (int i = 0; i < sensor_fusion.size(); ++i) {
            //car is in my lane
            float d = sensor_fusion[i][6];
            if (d < (2 + 4*lane + 2) && d > (2 + 4*lane - 2)) {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];
              double dist_to_car = check_car_s - car_s;
              check_car_s += ((double)prev_size*0.02*check_speed); // if using prev points can project s value out
              if (dist_to_car > 0) {
                if (dist_to_car < closest_car_ahead_s) {
                  // find speed of closest car
                  closest_car_ahead_s = dist_to_car;
                  velocity_to_hit_ahead = std::min(check_speed*2.24, speed_limit - 0.5);
                  if (dist_to_car < dist_max) {
                    too_close = true;
                  }
                }
              }
            }
            // check if left lane is open
            if ((lane != 0) && d < (2 + 4*(lane-1) + 2) && d > (2 + 4*(lane-1) - 2)) {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];

              double dist_to_car = check_car_s - car_s;
              check_car_s += ((double)prev_size*0.02*check_speed); // if using prev points can project s value out

              // find speed of closest car on left
              if (dist_to_car > 0) {
                if (dist_to_car < closest_car_left_s) {
                  closest_car_left_s = dist_to_car;
                  velocity_to_hit_left = std::min(check_speed*2.24, speed_limit - 0.5);
                }
              }
              // if too close, lane is closed
              if (abs(dist_to_car) < dist_max) {
                left_open = false;
              }
            }
            // check if right lane is open
            if ((lane != 3) && d < (2 + 4*(lane+1) + 2) && d > (2 + 4*(lane+1) - 2)) {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];

              double dist_to_car = check_car_s - car_s;
              check_car_s += ((double)prev_size*0.02*check_speed); // if using prev points can project s value out

              // find speed of closest car on right
              if (dist_to_car > 0) {
                if (dist_to_car < closest_car_right_s) {
                  closest_car_right_s = dist_to_car;
                  velocity_to_hit_right = std::min(check_speed*2.24, speed_limit - 0.5);
                }
              }
              // if too close, lane is closed
              if (abs(dist_to_car) < dist_max) {
                right_open = false;
              }
            }
          }
          if (too_close) {
            ref_vel -= .224;
          } else if (ref_vel < velocity_to_hit_ahead) {
            ref_vel += .224;
          }
          // if lane is open and faster, prepare lane change
          if (left_open && (lane > 0) && (velocity_to_hit_ahead < velocity_to_hit_left)) {
            prepare_left = true;
          } else if (right_open && (lane < 2) && (velocity_to_hit_ahead < velocity_to_hit_right)) {
            prepare_right = true;
          }
           // Create a list of widely spaced (x,y) points, evenly sapced at 30m
           // later interpolate these waypoints with a spline
           vector<double> anchor_ptsx;
           vector<double> anchor_ptsy;

           // reference x,y, yaw states
           // either we will reference the starting point as where the car is or at the previous paths end point
           double ref_x = car_x;
           double ref_y = car_y;
           double ref_yaw = deg2rad(car_yaw);

           if (prev_size < 2) {
             // use two points that make the path tangent to the car
             double prev_car_x = car_x - cos(car_yaw);
             double prev_car_y = car_y - sin(car_yaw);

             anchor_ptsx.push_back(prev_car_x);
             anchor_ptsx.push_back(car_x);

             anchor_ptsy.push_back(prev_car_y);
             anchor_ptsy.push_back(car_y);
           } else {
             // use previous path's end point as starting reference

             // redefine ref state as previous path end point
             ref_x = previous_path_x[prev_size-1];
             ref_y = previous_path_y[prev_size-1];

             double ref_x_prev = previous_path_x[prev_size-2];
             double ref_y_prev = previous_path_y[prev_size-2];
             ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

             // use two points that make the path tangent to prev path's end point
             anchor_ptsx.push_back(ref_x_prev);
             anchor_ptsx.push_back(ref_x);

             anchor_ptsy.push_back(ref_y_prev);
             anchor_ptsy.push_back(ref_y);
           }
           // In Frenet add evenly 30m spaced points ahead of starting ref
           if (prepare_left) {
             lane--;
           } else if (prepare_right) {
             lane++;
           }
           vector<double> next_wp0 = getXY(car_s + 30, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
           vector<double> next_wp1 = getXY(car_s + 60, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
           vector<double> next_wp2 = getXY(car_s + 90, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

           anchor_ptsx.push_back(next_wp0[0]);
           anchor_ptsx.push_back(next_wp1[0]);
           anchor_ptsx.push_back(next_wp2[0]);

           anchor_ptsy.push_back(next_wp0[1]);
           anchor_ptsy.push_back(next_wp1[1]);
           anchor_ptsy.push_back(next_wp2[1]);

           for (int i = 0; i < anchor_ptsx.size(); ++i) {
             // shift car ref angle to 0 degrees
             double shift_x = anchor_ptsx[i] - ref_x;
             double shift_y = anchor_ptsy[i] - ref_y;

             anchor_ptsx[i] = (shift_x*cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw));
             anchor_ptsy[i] = (shift_x*sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw));
           }

           // create spline
           tk::spline s;

           // set (x,y) points to the spline
           s.set_points(anchor_ptsx, anchor_ptsy);

           // define actual (x,y) points we will use for Planner
           vector<double> next_x_vals;
           vector<double> next_y_vals;

           // start with all prev path points
           for (int i = 0; i < previous_path_x.size(); ++i) {
             next_x_vals.push_back(previous_path_x[i]);
             next_y_vals.push_back(previous_path_y[i]);
           }

           // calculate how to break up spline points so we travel at desired ref velocity
           double target_x = 30.0;
           double target_y =  s(target_x);
           double target_dist = sqrt((target_x*target_x) + (target_y*target_y));

           double x_add_on = 0;

           // fill up rest of path planner after filling with prev points; here will always output 50 points
           for (int i = 1; i <= 50 - previous_path_x.size(); ++i) {
             double N = (target_dist/(0.02*ref_vel/2.24));
             double x_point = x_add_on + (target_x)/N;
             double y_point = s(x_point);

             x_add_on = x_point;

             double x_ref = x_point;
             double y_ref = y_point;

             // rotate back to normal after rotating earlier
             x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
             y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

             x_point += ref_x;
             y_point += ref_y;

             next_x_vals.push_back(x_point);
             next_y_vals.push_back(y_point);
           }

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
