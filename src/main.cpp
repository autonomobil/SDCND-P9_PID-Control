#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <string>
#include <chrono>

// for convenience
using json = nlohmann::json;

// data stream output
std::ofstream out_data;
std::fstream parameter_data;

// Kp , Ki , Kd, Throttle_max
double parameter[4] = {0.15, 0.003, 5.0, 0.6};

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws){

    // reset
    std::string msg("42[\"reset\", {}]");
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

}

inline bool exists_test(const std::string& name) {
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}

void read_parameter(){

    if(exists_test("parameter.dat")){
      string parameter_str[4];

      parameter_data.open("parameter.dat", ios::in);

      parameter_data >> parameter_str[0];
      parameter_data >> parameter_str[1];
      parameter_data >> parameter_str[2];
      parameter_data >> parameter_str[3];
      parameter[0] = std::stod(parameter_str[0]);
      parameter[1] = std::stod(parameter_str[1]);
      parameter[2] = std::stod(parameter_str[2]);
      parameter[3] = std::stod(parameter_str[3]);

      cout << "Parameter loaded"<< endl;
      // cout << "Kp: " << parameter_str[0] << '\t';
      // cout << "Ki: " << parameter_str[1] << '\t';
      // cout << "Kd: " << parameter_str[2] << '\t';
      // cout << "max Throttle: " << parameter_str[3] << endl;

      parameter_data.close();

    }
    // if file not exist, create it
    else{
      parameter_data.open("parameter.dat", ios::out);
      parameter_data << parameter[0] << ' ';
      parameter_data << parameter[1] << ' ';
      parameter_data << parameter[2] << ' ';
      parameter_data << parameter[3] << ' ' << std::endl;;

      parameter_data.close();

    }
}

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

int main()
{

  uWS::Hub h;

  PID pid;

  read_parameter();

  // Initialize the pid variable.
  double Kp = parameter[0];
  double Ki = parameter[1];
  double Kd = parameter[2];

  cout << "Kp: " << Kp << '\t';
  cout << "Ki: " << Ki << '\t';
  cout << "Kd: " << Kd << '\t';
  cout << "Max Throttle: " << parameter[3]  << endl;

  pid.Init(parameter[0], parameter[1], parameter[2]);

  std::string data_out_name = "dataOut-" + std::to_string(Kp) + "_" + std::to_string(Ki) + "_"  + std::to_string(Kd) + ".dat";

  out_data.open(data_out_name);
  // Initialize with zeros for matlab
  out_data << 000 << ',' << 000 << ',' << 000 << ',' << 000 << ',' << 000 << endl;
  out_data.close();

  out_data.open(data_out_name, ios::app);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle;
          // count timesteps
          long int timestep = pid.Time_stepper();

          if (cte > 5 || cte < -5) {
            out_data.close();
            reset_simulator(ws);

            std::cout << "----------------> PID controller terminated because car left track, please restart with ./pid" << std::endl;
            std::terminate();

          }
          else{
            // Update error values with cte
            pid.UpdateError(cte);

            // Calculate steering value
            steer_value = pid.TotalError(speed);

            throttle = parameter[3] - fabs(steer_value);
            //throttle = throttle * 1.0 / ( 1.0 + fabs(steer_value));
          }

          // out_data("data_out.txt", ios::app);
          out_data << timestep << ',';
          out_data << cte << ',';
          out_data << speed << ',';
          out_data << steer_value << ',';
          out_data << angle << endl;

          // DEBUG
          std::cout << "Timestep: " << timestep  << '\t';
          std::cout << " CTE: " << cte << '\t';
          std::cout << " Throttle: " << throttle << '\t';
          std::cout << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
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
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
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
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
