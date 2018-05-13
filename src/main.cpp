#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <limits>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
inline static double deg2rad(double x) { return x * pi() / 180; }
inline static double rad2deg(double x) { return x * 180 / pi(); }
inline static double mph2mps(double x) { return x * 0.44704; }
inline static double mps2mph(double x) { return x * 2.23693629; }
inline static double get_lane_center(int lane) { return 2 + 4 * lane; }

static const double MAX_REFERENCE_SPEED = 49.5; // mph
static const int STEPS = 50;
static const double TIME_INCR = 0.02; // secs
static const double TARGET_HORIZONT = 30.0; // m
static const double ACC_MAX = 8.5; // m/s^2
static const double SPEED_TOLERANCE = 0.01; // m/s
static const float MIN_SCORE_DIFF = 1.05; // %5

static double ref_speed = 0.0; // start at zero
static double ref_speed_delta = 0.0;
static int ref_speed_delta_i = 1;
static int lane_no = 1;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
static string HasData(const string& s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");

  if (found_null != string::npos)
    return "";
  else if (b1 != string::npos && b2 != string::npos)
    return s.substr(b1, b2 - b1 + 2);

  return "";
}

static inline double Distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

static int ClosestWaypoint(double x, double y, const vector<double>& maps_x, const vector<double>& maps_y)
{
	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < maps_x.size(); ++i)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = Distance(x, y, map_x, map_y);
		if (dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;

}

static int NextWaypoint(double x, double y, double theta, const vector<double>& maps_x, const vector<double>& maps_y)
{

	int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y - y), (map_x - x));

	double angle = fabs(theta - heading);
  angle = min(2*pi() - angle, angle);

  if (angle > pi()/4)
  {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size())
      closestWaypoint = 0;
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
static vector<double> GetFrenet(double x, double y, double theta, const vector<double>& maps_x, const vector<double>& maps_y)
{
	int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);
	int prev_wp = next_wp - 1;

	if (next_wp == 0)
		prev_wp = maps_x.size() - 1;

	double n_x = maps_x[next_wp] - maps_x[prev_wp];
	double n_y = maps_y[next_wp] - maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
	double proj_x = proj_norm * n_x;
	double proj_y = proj_norm * n_y;

	double frenet_d = Distance(x_x, x_y, proj_x, proj_y);

	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = Distance(center_x, center_y, x_x, x_y);
	double centerToRef = Distance(center_x, center_y, proj_x, proj_y);

	// see if d value is positive or negative by comparing it to a center point
	if(centerToPos <= centerToRef)
		frenet_d *= -1;

	// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; ++i)
		frenet_s += Distance(maps_x[i], maps_y[i], maps_x[i+1], maps_y[i+1]);

	frenet_s += Distance(0, 0, proj_x, proj_y);

	return { frenet_s, frenet_d };
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double>& maps_s, const vector<double>& maps_x, const vector<double>& maps_y)
{
	int prev_wp = -1;

	while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size() - 1)))
		++prev_wp;

	int wp2 = (prev_wp + 1) % maps_x.size();

	double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s - maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
	double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

	double perp_heading = heading-pi() / 2;

	double x = seg_x + d * cos(perp_heading);
	double y = seg_y + d * sin(perp_heading);

	return { x, y };
}

static int GetLaneScore(const vector<vector<float>>& sensor_fusion, const int check_lane_no, const int curr_lane_no, const float car_s, const double car_v, const int prev_sz, bool& too_close)
{
  static const int REJECT = -1E6;
  float check_car_s_front = std::numeric_limits<float>::max();
  float car_s_inline = 0;
  float check_car_s_behind = std::numeric_limits<float>::min();
  float check_speed_front;
  float check_speed_behind;
  int i_front = -1;
  int i_inline = -1;
  int i_behind = -1;
  bool collision_warning = false;

  for (int i = 0; i < sensor_fusion.size(); ++i)
  {
    const float d = sensor_fusion[i][6];
    if (d > get_lane_center(check_lane_no) + 2 || d < get_lane_center(check_lane_no) - 2)
    {
      continue; // given car is not in the examined lane
    }

    const double check_speed = sqrt(sensor_fusion[i][3]*sensor_fusion[i][3] + sensor_fusion[i][4]*sensor_fusion[i][4]);
    // s projected to last trajectory's endpoint
    const double check_car_s = sensor_fusion[i][5] + double(prev_sz) * TIME_INCR * check_speed;

    if (fabs(sensor_fusion[i][5] - car_s) < TARGET_HORIZONT * 0.2)
    { // car is in line with us
      car_s_inline = car_s;
      i_inline = i;
    }

    if (check_car_s >= car_s)
    { // car is in front of us
      if (check_car_s_front > check_car_s)
      {
        check_car_s_front = check_car_s;
        check_speed_front = check_speed;
        i_front = i;
      }
    }
    else // if (check_car_s < car_s)
    { // car is behind
      if (check_car_s_behind < check_car_s)
      {
        check_car_s_behind = check_car_s;
        check_speed_behind = check_speed;
        i_behind = i;
      }
    }

  }

  int score_s = 0;
  int score_v = 0;
  int score_t = 0;
  int sum = 0;

  if (i_inline >= 0)
  {
    score_s = REJECT * 2;
    collision_warning = true;
    //printf("GetLaneScore(%d|%03d):  {I} *TOO_CLOSE*\n", check_lane_no, (int)sensor_fusion[i_inline][0]);
  }
  else
  {
    //printf("GetLaneScore(%d|N/A):  {I}\n", check_lane_no);
  }

  if (i_front >= 0)
  {
    if (check_car_s_front - car_s < TARGET_HORIZONT)
    {
      score_s = REJECT;
      collision_warning = true;
    }
    else
    {
      score_s = int(check_car_s_front - car_s);
      if (check_lane_no == curr_lane_no && score_s > TARGET_HORIZONT * 3)
        score_s = 10000; // prefer the actual lane
    }

    score_v = int(10 * check_speed_front);

    if (car_v > 1)
      score_t = int((check_car_s_front - car_s) / car_v * 10);
    else
      score_t = 1000;
    //printf("GetLaneScore(%d|%03d):  {F}  s=%8d  |  v=%8d  |  t=%8d\n", check_lane_no, (int)sensor_fusion[i_front][0], score_s, score_v, score_t);
    sum += score_s + score_v + score_t;
  }
  else
  {
    //printf("GetLaneScore(%d|N/A):  {F}  10000\n", check_lane_no);
    sum += 2000;
  }

  if (i_behind >= 0)
  {
    if (car_s - check_car_s_behind < TARGET_HORIZONT * 0.5)
      score_s = REJECT;
    else
      score_s = int((car_s - check_car_s_behind) / 10); // less important that being front of us

    score_t = 0;

    score_v = int(MAX_REFERENCE_SPEED + car_v - check_speed_behind);
    //printf("GetLaneScore(%d|%03d):  {B}  s=%8d  |  v=%8d  |  t=%8d\n", check_lane_no, (int)sensor_fusion[i_behind][0], score_s, score_v, score_t);
    sum += score_s + score_v + score_t;
  }
  else
  {
    //printf("GetLaneScore(%d|N/A):  {B}  10000\n", check_lane_no);
    sum += 500;
  }

  // check feasibility
  if (curr_lane_no + 1 < check_lane_no || curr_lane_no - 1 > check_lane_no)
    sum = -2000000;

  if (collision_warning && check_lane_no == curr_lane_no)
    too_close = true; // it is in our lane, we must react

  //printf("GetLaneScore(%d|SUM): {%8d} %s\n", check_lane_no, sum, collision_warning ? "*COLL*" : "");
  return sum;
}

int main()
{
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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
  	istringstream iss(line);
  	double x, y;
  	float s, d_x, d_y;
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

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = HasData(data);

      if (s != "")
      {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry")
        {
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

            const int prev_sz = previous_path_x.size();
            bool too_close = false;

            if (prev_sz > 0)
              car_s = end_path_s;

            bool not_in_lane = car_d > get_lane_center(lane_no) + 0.4 || car_d < get_lane_center(lane_no) - 0.4;

            vector<int> lane_scores(3, 0);
            for (int i = 0; i < 3; ++i)
              lane_scores[i] = GetLaneScore(sensor_fusion, /* check_lane_no */ i, /* curr_lane_no */ lane_no, car_s, mph2mps(car_speed), prev_sz, too_close);

            if (!not_in_lane)
            { // may proceed with lane change when applicable
              vector<int>::iterator it_best_score = max_element(begin(lane_scores), end(lane_scores));
              // new score must be greater than zero and must be at least a 5% improvement over the current one
              int new_lane = *it_best_score > 0 && *it_best_score / (float)lane_scores[lane_no] > MIN_SCORE_DIFF ? distance(begin(lane_scores), it_best_score) : lane_no;
              if (lane_no != new_lane)
              { // lane change
                printf("lane change: %d(%d) -> %d(%d)\n", lane_no, lane_scores[lane_no], new_lane, *it_best_score);
                lane_no = new_lane;
              }
            }

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

          	vector<double> ptsx;
          	vector<double> ptsy;
            if (prev_sz < 2)
            { // fallback case when there is no previous info available
              double prev_ref_x = ref_x - cos(ref_yaw);
              double prev_ref_y = ref_y - sin(ref_yaw);

              ptsx.push_back(prev_ref_x);
              ptsx.push_back(ref_x);
              ptsy.push_back(prev_ref_y);
              ptsy.push_back(ref_y);
            }
            else
            {
              // add prev path's last point as starting point for spline fitting
              ref_x = previous_path_x[prev_sz - 1];
              ref_y = previous_path_y[prev_sz - 1];
              double prev_ref_x = previous_path_x[prev_sz - 2];
              double prev_ref_y = previous_path_y[prev_sz - 2];
              ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

              ptsx.push_back(prev_ref_x);
              ptsy.push_back(prev_ref_y);
              ptsx.push_back(ref_x);
              ptsy.push_back(ref_y);
            }

            { // local variable scoping
              vector<double> next_wp0 = getXY(car_s + 1 * TARGET_HORIZONT, get_lane_center(lane_no), map_waypoints_s, map_waypoints_x, map_waypoints_y);
              vector<double> next_wp1 = getXY(car_s + 2 * TARGET_HORIZONT, get_lane_center(lane_no), map_waypoints_s, map_waypoints_x, map_waypoints_y);
              vector<double> next_wp2 = getXY(car_s + 3 * TARGET_HORIZONT, get_lane_center(lane_no), map_waypoints_s, map_waypoints_x, map_waypoints_y);

              ptsx.push_back(next_wp0[0]); 
              ptsx.push_back(next_wp1[0]); 
              ptsx.push_back(next_wp2[0]); 
              ptsy.push_back(next_wp0[1]); 
              ptsy.push_back(next_wp1[1]); 
              ptsy.push_back(next_wp2[1]); 
            }

            // transforming coordinates into the car's local coordinate system
            for (int i = 0; i < ptsx.size(); ++i)
            {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
              ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);

              if (i > 0 && ptsx[i] <= ptsx[i-1])
              {
                printf("Warning: ptsx[%d]=%.2f is not greater than ptsx[%d]=%.2f\n", i, ptsx[i], i-1, ptsx[i-1]);
                ptsx.erase(ptsx.begin() + i);
                ptsy.erase(ptsy.begin() + i);
                --i;
              }
            }

            tk::spline s;
            s.set_points(ptsx, ptsy);

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            for (int i = 0; i < prev_sz; ++i)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            const double target_x = TARGET_HORIZONT; // target horizon in meters
            const double target_dist_ratio = Distance(0., 0., target_x, s(target_x)) / target_x;
            double ref_speed_delta_new = 0.;
            double x_point = 0.;
            double y_point = 0.;
            int i_adjusted = 1;
            for (int i = 1; i <= STEPS - prev_sz; ++i)
            {
              int action = 0;
              if (ref_speed >= SPEED_TOLERANCE && too_close)
                action = -1;
              else if (ref_speed <= MAX_REFERENCE_SPEED - SPEED_TOLERANCE)
                action = 1;

              i_adjusted = i;
              if (action)
              {
                if (action > 0 && ref_speed_delta > 0. || action < 0 && ref_speed_delta < 0.)
                  i_adjusted = ref_speed_delta_i;

                ref_speed_delta_new = (i_adjusted * TIME_INCR) * (i_adjusted * TIME_INCR) * 0.5 * mps2mph(ACC_MAX) / target_dist_ratio;
                if (ref_speed_delta_new > mps2mph(ACC_MAX) * TIME_INCR) ref_speed_delta_new = mps2mph(ACC_MAX) * TIME_INCR;

                if (action < 0)
                {
                  if (ref_speed_delta > 0.) // previous action was acceleration
                  {
                    ref_speed_delta_new = 0.;
                    ref_speed_delta_i = 1;
                  }
                  else if (ref_speed_delta_new + mps2mph(SPEED_TOLERANCE) < -ref_speed_delta) // large decceleration difference from last state
                  {
                    ref_speed_delta_new = ref_speed_delta;
                  }
                  else
                  {
                    ref_speed_delta_new = -ref_speed_delta_new;
                  }
                  ref_speed += ref_speed_delta_new;
                  if (ref_speed_delta < 0. && -ref_speed_delta < mps2mph(ACC_MAX) * TIME_INCR) ++ref_speed_delta_i;
                  if (ref_speed < 0.)
                  {
                    ref_speed = 0.;
                    ref_speed_delta = 0.;
                  }
                  else
                    ref_speed_delta = ref_speed_delta_new;
                }
                else // if (action > 0)
                {
                  if (ref_speed_delta < 0.) // previous action was decceleration
                  {
                    ref_speed_delta_new = 0.;
                    ref_speed_delta_i = 1;
                  }
                  else if (ref_speed_delta_new + mps2mph(SPEED_TOLERANCE) < ref_speed_delta) // large acceleration difference from last state
                  {
                    ref_speed_delta_new = ref_speed_delta;
                  }
                  else
                  {
                  }
                  ref_speed += ref_speed_delta_new;
                  if (ref_speed_delta > 0. && ref_speed_delta < mps2mph(ACC_MAX) * TIME_INCR) ++ref_speed_delta_i;
                  if (ref_speed >= MAX_REFERENCE_SPEED)
                  {
                    ref_speed = MAX_REFERENCE_SPEED;
                    ref_speed_delta = 0.;
                  }
                  else
                    ref_speed_delta = ref_speed_delta_new;
                }
              }

              x_point += mph2mps(ref_speed) * TIME_INCR;
              y_point = s(x_point);

              next_x_vals.push_back(x_point * cos(ref_yaw) - y_point * sin(ref_yaw) + ref_x);
              next_y_vals.push_back(x_point * sin(ref_yaw) + y_point * cos(ref_yaw) + ref_y);

              if (x_point >= target_x) break;
            }

          	// define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  }
  );

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t)
  {
    const std::string s = "<h1>Hello world!</h1>";

    if (req.getUrl().valueLength == 1)
      res->end(s.data(), s.length());
    else // i guess this should be done more gracefully?
      res->end(nullptr, 0);
  }
  );

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
    std::cout << "Connected!!!" << std::endl;
    ref_speed = 0.0; // start at zero
    ref_speed_delta = 0.0;
    ref_speed_delta_i = 1;
    lane_no = 1;
  }
  );

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char* message, size_t length)
  {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  }
  );

  int port = 4567;
  if (h.listen(port))
    std::cout << "Listening to port " << port << std::endl;
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
