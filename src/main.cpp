#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// ===== START helper structs and enums =====

// Car{id, x, y, vx, vy, s, d}
struct Car {
    int id;
    // coordinates on global map
    double x, y;
    // velocity in reference to global map coordinates
    double vx, vy;
    // Frenet coordinates
    double s, d;
};

// States
enum STATES {
    KEEP_LANE,
    PLCR,       // prepare lane change right
    PLCL,       // prepare lane change left
    CLR,        // change lane right
    CLL,        // change lane left
};

// TODO introduce a function that enforces correctness of state changes!

// ===== END =====

#define NUM_WAYPOINTS 75   // number of waypoints to be calculated during path planning

int lane = 1;
double last_vel = 0;    // saving the latest calculated velocity, since telemetry data (like car_speed) is jumping unreliably sometimes
double ref_vel = 49.5;  // try to reach 49.5 mph asap
const double overall_max_speed = 49.6;  // for some reason the simulator returns speed in mph while expecting it in m/s
STATES current_state = STATES::KEEP_LANE;   // default state is KEEP_LANE

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

    h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
                &map_waypoints_dx,&map_waypoints_dy]
                (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);
//std::cout << " ---- next event ----" << std::endl;
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

                    json msgJson;

                    /**
                     * TODO: define a path made up of (x,y) points that the car will visit
                     *   sequentially every .02 seconds
                     */

                    int prev_size = previous_path_x.size();

                    if (prev_size > 0) {
                        // TODO check this statement!
                        car_s = end_path_s;
                    }
                    // ====== START check for other cars ======

                    if (current_state == STATES::KEEP_LANE) {
                        // try to maintain current speed
                        // check for other cars within my lane
                        // reduce speed to maintain safe distance (if necessary)
                        // change state to prepare lane change if possible
                        //  maybe the last 2 steps are both handeled by the PLCx state?!?!??

                        bool too_close = false;
                        double current_max_speed = overall_max_speed;

                        for ( int i = 0; i < sensor_fusion.size(); i++) {
                            Car oCar = {sensor_fusion[i][0], sensor_fusion[i][1], sensor_fusion[i][2], sensor_fusion[i][3], sensor_fusion[i][4], sensor_fusion[i][5], sensor_fusion[i][6]};
                            // any car in my lane?
                            if (oCar.d < (2 + 4*lane + 2) && oCar.d > (2 + 4*lane -2)) {
                                double check_speed = sqrt(oCar.vx*oCar.vx + oCar.vy*oCar.vy);
                                double check_car_s = oCar.s;

                                check_car_s += ((double)prev_size * .02 * check_speed);
                                // car is in front of me
                                if (check_car_s > car_s) {
                                    // distance is smaller than 30m
                                    if ((check_car_s - car_s) < 30) {
                                        // TODO start lane change preparation
                                        // switch lanes if a slower car is ahead of us
                                        if ( lane > 0) {
                                            lane = 0;
                                        } else if (lane == 0) {
                                            lane = 1;
                                        }
                                        current_max_speed = check_speed;
//                                    }
                                    // distance is smaller than 15m
//                                    if ((check_car_s - car_s) < 15) {
                                        too_close = true;
                                        current_max_speed = check_speed;
                                    }
                                }
                            }
                        }
                        // TODO try to move speed inc/dec into path point calculation later on to make it more effective
                        if (too_close) {
                            ref_vel = current_max_speed - 0.224;
                        } else if (ref_vel < current_max_speed) {
                            ref_vel = current_max_speed;
                        }
                    }

                    // ====== END check for other cars ======
                    // ====== START smoothen the path using splines ======

                    // temporary points (will be in a car related coordinate system after shifting)
                    vector<double> ptsx;
                    vector<double> ptsy;

                    double ref_x = car_x;
                    double ref_y = car_y;
                    double ref_yaw = deg2rad(car_yaw);

                    // If previous size is almost empty, use the car as starting reference
                    if (prev_size < 2) {
                        // Calculate the cars previous x and y position to create a path tangent to the car
                        double prev_car_x = car_x - cos(car_yaw);
                        double prev_car_y = car_y - sin(car_yaw);

                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(car_x);

                        ptsy.push_back(prev_car_y);
                        ptsy.push_back(car_y);
                    } else { // use the previous path's end point as starting reference
                        // Redefine reference state as previous path end point
                        ref_x = previous_path_x[prev_size - 1];
                        ref_y = previous_path_y[prev_size - 1];

                        double ref_x_prev = previous_path_x[prev_size - 2];
                        double ref_y_prev = previous_path_y[prev_size - 2];
                        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                        // Use two points that make the path tangent to the previous path's end point
                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_x);

                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back(ref_y);
                    }

                    // in frenet add evenly 30m spaced points ahead of the starting reference
                    vector<double> next_wp0 = getXY(car_s + 30, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    vector<double> next_wp1 = getXY(car_s + 60, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    vector<double> next_wp2 = getXY(car_s + 90, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

                    ptsx.push_back(next_wp0[0]);
                    ptsx.push_back(next_wp1[0]);
                    ptsx.push_back(next_wp2[0]);

                    ptsy.push_back(next_wp0[1]);
                    ptsy.push_back(next_wp1[1]);
                    ptsy.push_back(next_wp2[1]);

                    // shift coordinates to a car related coordinate system
                    for (int i = 0; i< ptsx.size(); i++) {
                        double shift_x = ptsx[i] - ref_x;
                        double shift_y = ptsy[i] - ref_y;
                        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
                        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
                    }

                    // create a spline and set (x, y) points accordingly
                    tk::spline s;
                    s.set_points(ptsx, ptsy);

                    // define actual (x, y) points we'll use for the planner
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    // start with all the points from the previous path
                    for (int i = 0; i < previous_path_x.size(); i++) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    // calculate how to break up spline points so that we travel at our desired reference velocity
                    double target_x = 30.0;
                    double target_y = s(target_x);
                    double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));

                    double x_add_on = 0;
                    double next_vel = last_vel;

                    // fill up the rest of our path planner after filling it with previous points, so we'll allways have NUM_WAYPOINTS points
                    for(int i = 1; i <= NUM_WAYPOINTS - previous_path_x.size(); i++) {
                        // slightly adapt vleocity till we reach final ref_vel
                        if (next_vel < ref_vel) {
                            next_vel += 0.224;
                            if (next_vel > ref_vel) {
                                next_vel = ref_vel;
                            }
                        } else if (next_vel > ref_vel) {
                            next_vel -= 0.224;
                            if (next_vel < 0) {
                                next_vel = 0;
                            }
                        }

                        // deviding ref_vel (reference velocity) by 2.24 to convert it to m/s from mph
                        // factor .02 is here, because the simulator expects a waypoint for each 0.02 seconds
                        double N = (target_dist / (.02 * next_vel / 2.24));
                        double x_point = x_add_on + (target_x) / N;
                        double y_point = s(x_point);

                        x_add_on = x_point;

                        // shift and rotate back to normal (global) coordinate systems after shifting it to car related earlier
                        double x_ref = x_point;
                        double y_ref = y_point;

                        x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
                        y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

                        x_point += ref_x;
                        y_point += ref_y;

                        next_x_vals.push_back(x_point);
                        next_y_vals.push_back(y_point);
                    }
                    last_vel = next_vel;    // update last_vel for the next update

                    // ====== END smoothen the path using splines ======

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
