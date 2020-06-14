#include <cmath>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

using nlohmann::json;
using std::string;

// for convenience
void reset(uWS::WebSocket<uWS::SERVER> ws);
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }

double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(const string &s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of('[');
  auto b2 = s.find_last_of(']');
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */

  bool twiddle = true;
  int n = 0;
  int reset_n = 230;
  int start_buffer = 5;

  double constants[] = {0.0505, 0.0001, 0.25};
  double dp[] = {0.01, 0.00005, 0.25};
  double total_error = 0;
  double best_error;
  double tolerance = 0.01;
  int constant_index = 0;
  bool first = true;
  bool repeat = false;

  pid.Init(0.0505, 0.0001, 0.25);

  h.onMessage([&first, &start_buffer, &pid, &twiddle, &n, &reset_n, &constants, &dp, &total_error, &repeat, &best_error, &tolerance, &constant_index](
      uWS::WebSocket<uWS::SERVER> ws,
      char *data,
      size_t length,
      uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (!s.empty()) {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value = 0;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          if (twiddle) {
            if ((dp[0] + dp[1] + dp[2]) < tolerance) {
              std::cout << "Final constants: Kp: " << constants[0] << ", Ki: " << constants[1] << ", Kd: "
                        << constants[2] << std::endl;

              return 0;
            }

            if (n == 0) {
              if(first){
                reset(ws);
              }

              std::cout << "New Initialization" << std::endl;

              if (!first && !repeat) {
                constants[constant_index] += dp[constant_index];
              }

              pid.Init(constants[0], constants[1], constants[2]);
              total_error = 0;
            }

            bool force_reset = false;

            if (n > start_buffer && (speed < 1 || abs(cte) > 6)) {
              force_reset = true;
            }

            if (n < reset_n && !force_reset) {
              if (n > start_buffer) {
                total_error += pow(cte, 2);
              }

              ++n;
            } else {
              total_error /= n;

              if (first) {
                first = false;
                best_error = total_error;
              } else if (repeat) {
                repeat = false;
                if (total_error < best_error) {
                  best_error = total_error;
                  dp[constant_index] *= 1.1;
                } else {
                  constants[constant_index] += dp[constant_index];
                  dp[constant_index] *= 0.9;
                }

                constant_index = (constant_index + 1) % 3;
              } else {
                if (total_error < best_error) {
                  best_error = total_error;
                  dp[constant_index] *= 1.1;

                  constant_index = (constant_index + 1) % 3;
                } else {
                  constants[constant_index] -= 2 * dp[constant_index];
                  repeat = true;
                }
              }

              n = 0;
              total_error = 0;
              reset(ws);
            }
            pid.UpdateError(cte);
            steer_value = pid.TotalError();

            std::cout << "N: " << n
                      << " Total Error: " << total_error
                      << " Current Average: " << total_error / (n - start_buffer)
                      << " Current Best: " << best_error
                      << " Kp: " << constants[0]
                      << ", Ki: " << constants[1]
                      << ", Kd: " << constants[2]
                      << std::endl;

          } else {

            pid.UpdateError(cte);

            steer_value = pid.TotalError();
          }
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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
void reset(uWS::WebSocket<uWS::SERVER> ws) {
  string msg = "42[\"reset\",{}]";
  ws.send(msg.data());
}
