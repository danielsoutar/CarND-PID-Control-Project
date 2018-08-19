#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <thread>
#include "json.hpp"
#include "PID.h"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Wanted to explore the parameter space.
  // Would have systematically searched it by iterating over different
  // combinations of Kp and Kd. Assume Ki fixed, at 0.001.

  // Starting from 0, 0, iterate over the combinations
  // in multiples of 0.1 per parameter.

  // int time_step = 0;
  // double Kp_i = 0;
  // double Kd_i = 0;
  // const double Kp_step = 0.1;
  // const double Kd_step = 0.1;
  // const int MAX_KP = 10;
  // const int MAX_KD = 10;
  // const double MAX_CTE_THRESHOLD = 3.0;

  // PID pid_only_proportional(0.3, 0.0, 0.0);
  // PID pid_only_integral(0.0, 0.0, 0.001);
  // PID pid_only_derivative(0.0, 5.0, 0.0);

  PID pid_steer(0.3, 5, 0.001);
  // pid_steer.RecordTotalError();
  PID pid_throttle(0.45, 0.5, 0.0);

  bool is_initialised = false;

  h.onMessage([&pid_steer, &pid_throttle, &is_initialised](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double steer_value;
          double throttle;
          double reference_velocity = 0.6;

          if(!is_initialised) {
            pid_steer.Init(cte);
            steer_value = pid_steer.Respond(cte);
            pid_throttle.Init(steer_value);
            throttle = pid_throttle.RespondThrottle(fabs(steer_value), reference_velocity);
            is_initialised = true;
          }
          else {
            steer_value = pid_steer.Respond(cte);
            throttle = pid_throttle.RespondThrottle(fabs(steer_value), reference_velocity);
          }

          // The below doesn't work - the simulator's latency between sending a reset message and said message
          // actually feeding back appears to be the reason. I would have appended the results to a log file.
          // I was actually just wanting to explore the parameter space. Alas, this simulator makes that
          // needlessly difficult. Hopefully this project is changed so that it is easier to test out in Python
          // before running the FINAL controller on the simulator. That would make much more sense.

          // time_step += 1;
          // if((fabs(cte) > MAX_CTE_THRESHOLD || time_step >= 24000)) {
          //   // std::cout << ((fabs(cte) > MAX_CTE_THRESHOLD) == true) << ", " << ((time_step % 24000 == 0) == true) << std::endl;

          //   if(fabs(cte) > MAX_CTE_THRESHOLD) {
          //     my_logfile << pid_steer.K_coeffs[0] << ", " << pid_steer.K_coeffs[1] << ", " << 1e19 << std::endl;
          //   }
          //   else
          //     my_logfile << pid_steer.K_coeffs[0] << ", " << pid_steer.K_coeffs[1] << ", " << pid_steer.GetTotalError() << std::endl;

          //   // Need to set this so that total error is reset 
          //   is_initialised = false;
          //   pid_steer.SetParams(0, (Kp_i++ * Kp_step));
          //   if(((Kp_i) * Kp_step) == MAX_KP) {
          //     pid_steer.SetParams(0, 0.0);
          //     pid_steer.SetParams(1, (Kd_i++ * Kd_step));
          //   }

          //   Restart(ws);
          // }

          // 0.6 is about as fast as it can go before things get too... wonky. 0.5 is safer and calmer though.
          
          // DEBUG
          // my_logfile << cte << ", " << steer_value << std::endl;
          // std::cout << "CTE (Throttle): " << cte << " Throttle Value: " << throttle << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } 
      else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    }
    else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  }
  else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
