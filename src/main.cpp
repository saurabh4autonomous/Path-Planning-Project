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
#include <bitset>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

#define   WEIGHT_FRONT_L2 	   	     	3
#define   WEIGHT_FRONT_L1     	 	 	32
#define   WEIGHT_REAR_L1      		 	30
#define   WEIGHT_HIGHEST_COST           30
#define   WEIGHT_MEDIUM_COST       	 	15
#define   WEIGHT_LOWEST_COST             5
   
#define   DIST_FRONT_L1 		 	 	32
#define   DIST_FRONT_L2			 		40
#define   DIST_REAR_L1			 		30	                   

#define   SPEED_LIMIT   				49.5

#define   MIN_SAFE_DISTANCE_FRONT       18
#define   MIN_SAFE_DISTANCE_REAR        15

/* This Variable is updated with intended lane, when lane change is initiated*/
static int final_lane = 1;
/*This variable waits for stablization at Init and after every Lane Change*/
/* a Non- Zero Value triggers the Timer and Prohibits any Lane Change*/
static unsigned int stablization_timer = 1;

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

  int lane =1; 
  double ref_vel =0;
 
  h.onMessage([&max_s,&ref_vel,&lane,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
          * define a path made up of (x,y) points that the car will visit
          *   sequentially every .02 seconds
          */
     	  vector<double> ptsx;
  	      vector<double> ptsy;
   
	      int prev_size = previous_path_x.size();
	      double ref_x = car_x;
	      double ref_y = car_y;
	      double ref_yaw = deg2rad(car_yaw);
	      double dist_inc = 0.3;
	               
	      bool too_close = false;

	      int state_decision =0;

	      /* Lane Specific records*/
	      vector<double> s_nearest_ahead = {max_s,max_s,max_s};
	      vector<double> s_nearest_behind = {max_s,max_s,max_s};
	      vector<double> lane_target_speed = {SPEED_LIMIT,SPEED_LIMIT,SPEED_LIMIT};
	      vector<unsigned short int> lane_traffic_density = {0,0,0};
	      vector<double> lane_cost = {0,0,0};
	      
	      /* Behaviour Specific Flags*/ 
	      bool LC_allowed = true;
	      bool LCL_safe = false;
	      bool LCR_safe = false;
          
          /*Our Car's current lane*/
    	  int my_lane = ((int)(car_d))/4;
		  if((int)(car_d) <0)  /*Safeguard for Invalid Negative Values*/
			{
				my_lane = 1;
			}
			else
			{
				my_lane = ((int)(car_d))/4;
			}
		  
		  /* Storing left out path Points from previous path planned*/  	 
		  if(prev_size>0)
		   {
			   	car_s = end_path_s;	
		   }
		   
		   /* Check for Lane Changing Phase, Further Lane Changing Command is prohibited*/	  
		  if(my_lane != final_lane) LC_allowed = false;
		  
		  /* If timer's wait for 4 Seconds is over , Disable the Timer*/	    
		  if(stablization_timer >= 200)    /*4 seconds = 0.02 * 200 */
		     {
		     	stablization_timer = 0;	
		     }
	  
          /* Collecting Lane related data from Sensor Fusion Data*/
          for(int i = 0; i< sensor_fusion.size(); i++)
          {
          		
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx +vy*vy);
                double check_car_s = sensor_fusion[i][5];
            	int d = (int)(sensor_fusion[i][6]);
            	int other_car_lane = d/4;
            	double s_diff=0;
            	
			     bool car_front_L1 = false;
				 bool car_front_L2 = false; 
				 bool car_same_L0 = false; 
				 bool car_rear_L1 = false; 
				 bool car_rear_L2 = false;             	
                          	
                //if using previous points can project s value out
                check_car_s += (double)(prev_size*0.02*check_speed); 

                /* Difference of other car and our car*/				
				s_diff = (check_car_s -car_s);
				
				if((check_car_s > car_s) && ( s_diff < DIST_FRONT_L1) )     /*Ahead Car check*/
                {
                    car_front_L1 = true;	
                }
                
                if((check_car_s <= car_s) && ( -s_diff < DIST_REAR_L1) )     /* Behind Car Check*/
                {
                   car_rear_L1 = true;
                }

                /* Check Distance, Speed and Lane of the car*/
                if(car_front_L1)
                {
                    /* Concluding the Nearest Vehicle, Lane Speed and Traffic Density from Vehicles in the Front */
                	if(s_diff < s_nearest_ahead[other_car_lane])
                	{
                		s_nearest_ahead[other_car_lane] = s_diff;
                		if(check_speed <= SPEED_LIMIT)
                		{
                			lane_target_speed[other_car_lane] = check_speed;
                		}
                		else
                		{
                			lane_target_speed[other_car_lane] = SPEED_LIMIT;                			
                		}

                		lane_traffic_density[other_car_lane]++;
                				
                	}
                }
                else if(car_rear_L1)
                {
                	/* Concluding the Nearest Vehicle and Traffic Density from Vehicles in the Rear */
                	 if(-s_diff < s_nearest_behind[other_car_lane])
                	{
                		s_nearest_behind[other_car_lane] = -s_diff;
                		lane_traffic_density[other_car_lane]++;                						
                	}
                }
					
		 
            } /* End of Sensor Fusion Loop*/
          
			/* Safety Check, check for if the LCL/LCR is safe or not from each lane'e perspective*/
	        switch(my_lane)
	        {
	        	case 0: LCL_safe = false;
	        			if((s_nearest_ahead[lane +1] > MIN_SAFE_DISTANCE_FRONT) && (s_nearest_behind[lane+1] > MIN_SAFE_DISTANCE_REAR))
	        			{
	        				LCR_safe = true;	
	        			} 
	        			break;
	        	case 1: 
	        			if((s_nearest_ahead[lane -1] > MIN_SAFE_DISTANCE_FRONT) && (s_nearest_behind[lane - 1] > MIN_SAFE_DISTANCE_REAR))
	        			{
	        				LCL_safe = true;	
	        			} 
	        			if((s_nearest_ahead[lane +1] > MIN_SAFE_DISTANCE_FRONT) && (s_nearest_behind[lane+1] > MIN_SAFE_DISTANCE_REAR))
	        			{
	        				LCR_safe = true;	
	        			}                 			
	        			break;
	         	case 2: LCR_safe = false;
	        			if((s_nearest_ahead[lane -1] > MIN_SAFE_DISTANCE_FRONT) && (s_nearest_behind[lane - 1] > MIN_SAFE_DISTANCE_REAR))
	        			{
	        				LCL_safe = true;	
	        			} 
	        			break;
			  default : LCL_safe = false;
					    LCR_safe = false;               			                			
	        }          
	      
	        /* Cost Calculation for each Lane*/
			for(int l = 0; l< s_nearest_ahead.size(); l++)
			{   
			    /* Cost is inversely proportional to the Distance from our nearest Vehicle*/
			    /* 0 cost means no vehicle ahead, 1 cost means the Vehicle is right front of vehicle with 1 distance or less */
				if(s_nearest_ahead[l] == max_s)
				{
					lane_cost[l] += 0.0;	
				}
				else
				{
					lane_cost[l] += 1/s_nearest_ahead[l];
				}
			    
			    /* Cost is inversely proportional to the Lane Speed Allowed in the lane*/
			    /* 0 cost means no vehicle ahead, 1 cost means the Vehicle is right front of vehicle and hence varies the lane speed */				
				if(lane_target_speed[l] == SPEED_LIMIT)
				{
					lane_cost[l] += 0.0;	
				}
				else
				{
					lane_cost[l] += 1/lane_target_speed[l];
				}			
				
			    /* Cost is directly proportional to the Traffic Density in the lane*/
			    /* 0 cost means no vehicle in the lane, 1 cost means the All Vehicles are present in this lane */				
				lane_cost[l] += lane_traffic_density[l]/sensor_fusion.size();			
				
			}
			
			/*Decision Section */
			
			/*If we are travelling with Speeds more than Lane Speed, we must reduce the speed*/
			if(lane_target_speed[my_lane]< car_speed)
			{
				too_close= true;   
				/*This Flag makes the system reduce the speed in command Section*/
			}
			
			/* Decision for Lane change/ Lane Keep based on Various Parameters wrt our lane*/
			double min_cost = *min_element(lane_cost.begin(), lane_cost.end()); 
			switch(my_lane)
			{
			    /* Checking for Lane Costs, Lane Change Safety before deciding*/
				case 0 : if((lane_cost[0]>lane_cost[1]) && (LCR_safe == true))
					 {
					    /* If our Lane is 0, Lane 1 cost is less than Lane 0 Cost and LCR is safe*/
						state_decision = 1;
					 }
					 else
					 {
					 	/* Keep Lane*/
						state_decision = 0;
					 }
					 break;

			    	
				case 1 : 
					
		          	if((lane_cost[0] == min_cost) && (LCL_safe ==true))
			   		{
			   		    /* If our Lane is 1, Lane 0 cost is minimum among all lanes and LCL is safe*/
		               	state_decision = -1;
			   		}
			   		else if((lane_cost[2] == min_cost) && (LCR_safe ==true))
			   		{
			   		    /* If our Lane is 1, Lane 2 cost is minimum among all lanes and LCR is safe*/
						state_decision = 1;
			   		}
			   		else  
			   		{   
			   		    /* Keep Lane*/
		                state_decision = 0;
			   		}
					break;

				case 2 :
					if((lane_cost[2]>lane_cost[1]) && (LCL_safe ==true))
					{
						/* If our Lane is 2, Lane 1 cost is less than Lane 2 and LCL is safe*/
						state_decision = -1;	
					}
					else
					{
						/* Keep Lane*/
						state_decision = 0;
					}
					break;
				default : state_decision = 0;	

			}	
	
		    /* Final Output Decision Command Section [ref_vel , lane]*/
		    
		    /* Command for 'ref_vel' variable as per the State Decision*/
		    if(too_close == true)
		    {
				ref_vel -=0.224;
		    }
		    else
		    {
		        if(ref_vel<SPEED_LIMIT)
		        {
		            ref_vel += 0.224;
		        }
		    }

			/* Command for 'lane' variable as per the State Decision*/
			/* LC_allowed shall be false when Lane Change is happening*/
			/* Stablization Timer shall be Non-Zero for 4 Seconds staring from Lane Change Command*/
			if((LC_allowed == true) &&(stablization_timer == 0))
			{    
				switch(state_decision)
				{
					case -1 : lane--;
					         //std::cout<<"LCL"<<std::endl;
							 break;
					case 1 : lane++;
							 //std::cout<<"LCR"<<std::endl;
					         break;
					case 0 : 
							
					default : lane =my_lane; 

				}		
				/*Stores the Intended final Lane after this command*/
				final_lane = lane;
			}
			else
			{
			    /* This Timer starts at Lane Change Command and counts till 4 seconds, which includes Lane change + stablization Time*/
				stablization_timer ++;	
			}
			


		    //std::cout<<lane<<"|"<<my_lane<<"|"<<state_decision<<std::endl; 
		 	//std::cout << "hits = " << std::bitset<8>(temp)  << std::endl;       
			//std::cout<<LCL_safe<<"|"<<LCR_safe<<"|"<<LC_allowed<<"|"<<state_decision<<std::endl;    
 			//std::cout<<lane_cost[0]<<"|"<<lane_cost[1]<<"|"<<lane_cost[2]<<"|"<<min_cost<<std::endl;   
		
          /* Trajectory Generation Code */
          if(prev_size<2)
          {
            double prev_car_x= car_x - cos(car_yaw);
            double prev_car_y= car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);

          }
          else
          {
          	ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }


 			vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);


          for (int i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x* cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            ptsy[i] = (shift_x* sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }

          tk::spline s;

          s.set_points(ptsx,ptsy);

          for (int i = 0; i < previous_path_x.size(); i++) {
			next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30.0;
          double target_y = s(target_y);
          double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

          double x_add_on = 0;

          for(int i =0; i <= 50-previous_path_x.size() ; i++)
          {
			  double N = target_dist/(0.02*ref_vel/2.24);
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

			  //Rotate back to normal after rotating it earlier
              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);

           }

		  /*Returning the final path planned*/
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
