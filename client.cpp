#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Rand.hh>
#include <vector>
#include "build/custom_messages.pb.h"

#include <gazebo/gazebo_client.hh>
#define verbose false //If true, then print out all debug msgs
#define ApplyEBrake false   //If true, apply emergency brake to strictly follow right hand rule
using namespace std; 
/**
 * Minimal client application to connect to the simulator, receive the
 * current world state and send the next ego vehicle velocity.
 * You need to extend the OnWorldStateReceived function, the rest
 * is the boilerplate code for communication.
 */

typedef const boost::shared_ptr<
        const custom_messages::WorldState>
        WorldStateRequestPtr;

typedef const boost::shared_ptr<
        const custom_messages::Statistics>
        StatisticsRequestPtr;

class Controller {

public:
    /* Constructor
    *  Initialize the vectors priorcar_predicted_pos and egocar_predicted_pos with size of K
    *  Initialize priorcar_pos, priorcar_vel, priorcar_acc, acc_cmd and da_cmd
    */
    Controller() 
    {
        this->priorcar_predicted_pos.resize(this->K);
        this->egocar_predicted_pos.resize(this->K);
        this->priorcar_pos = 50.0;
        this->priorcar_vel = 0.0;
        this->priorcar_acc = 0.0;
        this->acc_cmd = 0.0;
        this->da_cmd = 0.0;
        this->episode = 0;
        this->success = 0;
        this->collision = 0;
        this->points_in_region = 0;
    }

    void Init()
    {
        // Create our node for communication
        node = gazebo::transport::NodePtr(new gazebo::transport::Node());
        node->Init();

        // Subscribe to the topic, and register a callback
        this->world_sub = node->Subscribe(worldTopicName, &Controller::OnWorldStateReceived, this);

        this->statistics_sub = node->Subscribe(statisticsTopicName, &Controller::OnStatisticsReceived, this);

        // Publish to the velodyne topic
        this->pub = node->Advertise<custom_messages::Command>(commandTopicName);

        // Wait for a subscriber to connect to this publisher
        this->pub->WaitForConnection();
    }

    // Called when the simulator resets the world after reaching the goal, running out of time or crashing
    void ResetWorld()
    {
        std::cout << "New simulation round started, resetting world." << std::endl;
        this->episode += 1;
    }

    void PrintWorldStateMessage(WorldStateRequestPtr& msg) const
    {
        if(verbose)
        {
        std::cout << "Simulation round: " << msg->simulation_round() << "; "
                  << "time: " << msg->time().sec() << "." << msg->time().nsec() << std::endl;
        std::cout << "  ego_car p: (" << msg->ego_vehicle().position().x() << ", " << msg->ego_vehicle().position().y()
                  << ") v: (" << msg->ego_vehicle().velocity().x() << ", " << msg->ego_vehicle().velocity().y() << "); " << std::endl;
        for (const auto& vehicle_msg : msg->vehicles())
        {
            if(vehicle_msg.lane_id() == 1)
            {
            std::cout << "  car id " << vehicle_msg.vehicle_id()
                      << " lane id: " << vehicle_msg.lane_id()
                      << " p: (" << vehicle_msg.position().x() << ", " << vehicle_msg.position().y()
                      << ") v: (" << vehicle_msg.velocity().x() << ", " << vehicle_msg.velocity().y() << "); "
                      << std::endl;
            }
        }

        std::cout << std::endl;
        }
    }

    void PrintStatisticsMessage(StatisticsRequestPtr& msg) const
    {
        std::cout << "Statistics from previous round: "<<endl
                  << "Success: " << msg->success() <<endl
                  << "collision: " << msg->collision_detected() <<endl
                  << "Time steps: "
                  << msg->simulation_time_steps_taken()<<endl
                  << "total acceleration: " << msg->total_acceleration() <<endl
                  << "acceleration/jerk limits respected: " << msg->limits_respected()<<endl
                  << "Success rate is "<<this->success + 1<<"/"<<this->episode<<endl;
    }

    // Called every time a new update is received from the simulator.
    // This is the main function where you should implement your code
    void OnWorldStateReceived(WorldStateRequestPtr& msg)
    {
        PrintWorldStateMessage(msg);

        // If the simulation round is different then this is a whole new setting, reinitialize the world
        if (msg->simulation_round() != simulation_round)
        {
            ResetWorld();
            simulation_round = msg->simulation_round();
            this->priorcar_pos = 50.0;
        }

        // Calculate the next velocity for the ego car and send the response.
        // Implement your code here
        SetPriorCar(msg);
        // Print the state and commands if needed.
        if(verbose)
        {
            cout<< "Prior car is at "<< this->priorcar_pos<<" with speed "
            <<this->priorcar_vel<<", acceleration is "<<this->priorcar_acc<<endl;  
            cout<<"Ego car is at "<<msg->ego_vehicle().position().x()<<" with speed "<<msg->ego_vehicle().velocity().x()<<endl;
        }
        // Make action decision according to ego car position and prior car states, either yield or surpass the prior car.
        MakeDecision(msg);
        // Predict ego car's future acceleration list.
        PredictEgocarAcc(msg);
        // Calculate the cost according to the acceleration list
        CalculateCost(msg);
        // Set velocity according to the cost list
        SetVel(msg);
        custom_messages::Command response_msg;
        response_msg.set_ego_car_speed(this->vel_cmd);
        response_msg.set_simulation_round(msg->simulation_round());
        this->pub->Publish(response_msg);
    }

    void OnStatisticsReceived(StatisticsRequestPtr& msg)
    {
        PrintStatisticsMessage(msg);
        if(msg->success() == 1){this->success += 1;}
        if(msg->collision_detected() == 1){this->collision += 1;}
    }

private:

    // Reset all the lists of prior cars
    void ResetPriorCarList()
    {
        this->priorcar_yield_pos_list.clear();
        this->priorcar_yield_vel_list.clear();
        this->priorcar_surpass_pos_list.clear();
        this->priorcar_surpass_vel_list.clear();
    }

    /* Set the prior car position and velocity according all the received car states
    *  First check if there are cars on the right of the ego car, if so then categorize them 
    *  in priorcar_yield_pos_list and priorcar_surpass_pos_list according to yield_line.
    *  The first car in the priorcar_surpass_pos_list is the prior car but can be surpassed.
    *  The last car in the priorcar_yield_pos_list is the prior car but need to be yielded.
    *  If both priorcar_yield_pos_list and priorcar_surpass_pos_list have cars, then check
    *  the distance between the last car of priorcar_yield_pos_list and the first car of
    *  priorcar_surpass_pos_list, if distance is less than yield_line, then make the car in
    *  priorcar_surpass_pos_list the prior car.
    *  \param[in/out]: priorcar_pos
    *                  priorcar_vel
    *                  priorcar_acc
    *  \param[in]: WorldStateRequestPtr The states of all other cars
    *  \param[in]: yield_line The threshold for categorizing prior cars
     */ 
    void SetPriorCar(WorldStateRequestPtr& msg)
    {
        // Reset all the lists
        ResetPriorCarList();    
        // Read states of all cars
        for (const auto& vehicle_msg : msg->vehicles())
        {  
            // If cars show up on lane 1 and on the right,
            if (vehicle_msg.lane_id() == 1 && vehicle_msg.position().y() < 0)
            {
                // then catagorize them according to yield_line.
                if (vehicle_msg.position().y() > this->yield_line)
                {
                    this->priorcar_yield_pos_list.push_back(vehicle_msg.position().y());
                    this->priorcar_yield_vel_list.push_back(vehicle_msg.velocity().y());
                }
                else
                {
                    this->priorcar_surpass_pos_list.push_back(vehicle_msg.position().y());
                    this->priorcar_surpass_vel_list.push_back(vehicle_msg.velocity().y());
                }
            }  
        }
        // If there is any prior car on the right side
        if (this->priorcar_yield_pos_list.size() > 0 || this->priorcar_surpass_pos_list.size() > 0)
        {
            //If both priorcar_yield_pos_list and priorcar_surpass_pos_list have cars
            if(this->priorcar_yield_pos_list.size() > 0 && this->priorcar_surpass_pos_list.size() > 0)
            {   
            /*Check the distance between the last car of priorcar_yield_pos_list and the first car of
            *  priorcar_surpass_pos_list, if distance is less than yield_line, then make the car in
            *  priorcar_surpass_pos_list the prior car.
            */
                int priorCarYieldIndex = min_element(this->priorcar_yield_pos_list.begin(),this->priorcar_yield_pos_list.end()) - this->priorcar_yield_pos_list.begin();
                int priorCarSurpassIndex = max_element(this->priorcar_surpass_pos_list.begin(),this->priorcar_surpass_pos_list.end()) - this->priorcar_surpass_pos_list.begin();
                if(this->priorcar_surpass_pos_list[priorCarSurpassIndex] - this->priorcar_yield_pos_list[priorCarYieldIndex] > this->yield_line)
                {
                    this->priorcar_pos = this->priorcar_surpass_pos_list[priorCarSurpassIndex];
                    this->priorcar_vel = this->priorcar_surpass_vel_list[priorCarSurpassIndex];
                }
                else
                {
                    this->priorcar_pos = this->priorcar_yield_pos_list[priorCarYieldIndex];
                    this->priorcar_vel = this->priorcar_yield_vel_list[priorCarYieldIndex];
                }
            }
            //If only priorcar_yield_pos_listst has cars, then the last car is the prior car.
            else if(this->priorcar_yield_pos_list.size() > 0 && this->priorcar_surpass_pos_list.size() == 0)
            {
                int priorCarYieldIndex = min_element(this->priorcar_yield_pos_list.begin(),this->priorcar_yield_pos_list.end()) - this->priorcar_yield_pos_list.begin();
                this->priorcar_pos = this->priorcar_yield_pos_list[priorCarYieldIndex];
                this->priorcar_vel = this->priorcar_yield_vel_list[priorCarYieldIndex];
            }
            //If only priorcar_surpass_pos_list has cars, then the first car is the prior car.
            else
            {
                int priorCarSurpassIndex = max_element(this->priorcar_surpass_pos_list.begin(),this->priorcar_surpass_pos_list.end()) - this->priorcar_surpass_pos_list.begin();
                this->priorcar_pos = this->priorcar_surpass_pos_list[priorCarSurpassIndex];
                this->priorcar_vel = this->priorcar_surpass_vel_list[priorCarSurpassIndex];
            }
        }
        //If no cars on the right, then reinitialize priorcar_pos, priorcar_vel and priorcar_acc
        else
        {
            this->priorcar_pos = 50.0;
            this->priorcar_vel = this->max_v;
            this->priorcar_acc = 0.0;
        }
    }

    /* Make action decision according to ego car position and prior car states, either yield or surpass the prior car.
    *  \param[in/out]: YIELD
    *  \param[in]: WorldStateRequestPtr The position of the ego car
    *  \param[in]: yield_line
    *  \param[in]: priorcar_pos
     */
    void MakeDecision(WorldStateRequestPtr& msg)
    {
        this->YIELD  = false;
        //If the prior car crosses the yield_line, then apply yield action.
        if(this->priorcar_pos > this->yield_line && msg->ego_vehicle().position().x() < 0){this->YIELD  = true; if(verbose){cout<<"Yield!"<<endl;}}
        //Backup line if(this->priorcar_pos > this->yield_line && msg->ego_vehicle().position().x() < margin){this->YIELD  = true; if(verbose({cout<<"Yield!"<<endl;}}
    }
 
    /* Predict a car's future positions with constant acceleration in K steps.
    *  \param[in/out]: vector predicted_pos
    *  \param[in]: current_pos - The current position of the car.
    *  \param[in]: current_vel - The current velocity of the car.
    *  \param[in]: current_acc - The current acceleration of the car.
     */
    void PredictPosWithConstAcc(vector<double>& predicted_pos, const double current_pos, const double current_vel, const double current_acc)
    {
        for (int k = 0; k < this->K; k++)
        {
            // Formula : Xt+dt = Xt + Vt * dt + 1/2 * At * dt^2
            predicted_pos[k] = current_pos + current_vel*k*this->dt + 0.5*current_acc*pow(k*this->dt,2);
        }
    }

    /* Predict ego car's future acceleration list according to the ego car's current and predicted positions,
    *  prior car predicted positions and YIELD signal. 
    *  \param[in/out]: vector acc_list
    *  \param[in]: ego_vehicle().position().x() - The current position of ego car.
    *  \param[in]: egocar_predicted_pos - The predicted positions of the ego car.
    *  \param[in]: priorcar_predicted_pos - The predicted positions of the prior car.
    *  \param[in]: acc_cmd - acceleration command from previous time step.
    */
    void PredictEgocarAcc(WorldStateRequestPtr& msg)
    {
        // Predict the future positions of the prior car
        PredictPosWithConstAcc(this->priorcar_predicted_pos, this->priorcar_pos, this->priorcar_vel, this->priorcar_acc);
        // Initialize acc_list 
        this->acc_list.clear();
        this->point_in_region_list.clear();
        // Add each element of da_list to previous acc_cmd
        for(vector<float>::const_iterator it = this->da_list.begin(); it != this->da_list.end(); ++it)
        {   
            this->points_in_region = 0;
            // Limit acc_cmd with max_a and min_a
            if (this->acc_cmd + *it <= this->max_a && this->acc_cmd + *it >= this->min_a)
            {
                // Only take consideration of YIELD signal when the ego car hasnt passed the intersection
                // Calculate the next velocity
                double ego_vel_temp = msg->ego_vehicle().velocity().x() + (*it) * this->dt;

                // Predict the future positions of the ego car with this acc_cmd
                PredictPosWithConstAcc(this->egocar_predicted_pos, msg->ego_vehicle().position().x(), ego_vel_temp, *it);
                for (int k = this->K - 1; k >= 0; k--)
                {
                    //If the position at k point is in the obstacle region, penalize it.
                    //If ego car needs to yield, it should wait for the prior car to pass.
                    if(this->YIELD)
                    {                            
                        if(this->egocar_predicted_pos[k] > margin && this->priorcar_predicted_pos[k] < 0 && this->priorcar_predicted_pos[k] > this->yield_line)
                        {this->points_in_region += 1;}
                    }
                    //If ego car can surpass, it should pass as soon as possbile before prior car crosses yield line.
                    else 
                    {
                        if(this->egocar_predicted_pos[k] < 0 && this->priorcar_predicted_pos[k] < 0 && this->priorcar_predicted_pos[k] > this->yield_line)
                        {this->points_in_region += 1;}
                    }
                    //If there is collision risk, penalize it with 10 points.
                    if(abs(this->egocar_predicted_pos[k]) < 5 && abs(this->priorcar_predicted_pos[k]) < 5)
                    {this->points_in_region += 10;}
                }  
                this->acc_list.push_back(*it + this->acc_cmd);
                this->point_in_region_list.push_back(this->points_in_region);
            }
        }
    }

    /* Calculate the costs and store the calculated cost according to valid acc_cmds.
    *  Formula : Cv(vel_target - vel_next)^2 + Ca(acc^2) + (number of points that in the obstacle region)^2
    *  \param[in/out]: vector cost_list
    *  \param[in]: vel_target - The target velocity of ego car.
    *  \param[in]: vel_next - The next possible velocity of ego car.
    *  \param[in]: acc_list - The valid accelerations.
     */
    void CalculateCost(WorldStateRequestPtr& msg)
    {
        // Initialize cost_list
        this->cost_list.clear();
        // Loop over each valid acceleration in K steps and store the calculated cost in cost_list
        for(int i = 0; i < this->acc_list.size(); i++)
        {
            // Initialize a temp cost value
            double cost_temp = 0;
            for(int k = 0; k < this->K; k++)
            {
                double vel_temp = msg->ego_vehicle().velocity().x() + this->acc_list[i] * this->dt * k;
                cost_temp += this->Cv*pow(this->vel_target - vel_temp, 2) + Ca*pow(this->acc_list[i], 2) + pow(this->point_in_region_list[i], 2);
            }
            this->cost_list.push_back(cost_temp);
            
        }
    }

    /* Set the velocity according to the minimum cost while keep acc_cmd and vel_cmd under constraints.
    *  \param[in/out]: acc_cmd
    *  \param[in/out]: vel_cmd
    *  \param[in]: vector cost_list
    *  \param[in]: vector acc_list
     */
    void SetVel(WorldStateRequestPtr& msg)
    {
        //Set the acceleration that has the minimum cost and keep it in constraints

        int minCostIndex = min_element(this->cost_list.begin(),this->cost_list.end()) - this->cost_list.begin();
        this->da_cmd = this->acc_list[minCostIndex] - this->acc_cmd;
        this->acc_cmd = this->acc_list[minCostIndex];
        if (this->acc_cmd > this->max_a){this->acc_cmd = this->max_a;}
        if (this->acc_cmd < this->min_a){this->acc_cmd = this->min_a;}

        //Set the velocity according to the selected acceleration and keep it in constraints
        this->vel_cmd = msg->ego_vehicle().velocity().x() + this->acc_cmd * this->dt;
        if (this->vel_cmd > this->max_v){this->vel_cmd = this->max_v; this->acc_cmd = 0.0;}
        if (this->vel_cmd < this->min_v){this->vel_cmd = this->min_v; this->acc_cmd = 0.0;}  
        if (ApplyEBrake && msg->ego_vehicle().position().x() > this->margin && this->priorcar_pos < 0 && msg->ego_vehicle().position().x() < 0 && this->priorcar_pos > this->margin)
        {this->vel_cmd = 0.0; this->acc_cmd = 0.0; if(verbose){cout<<"In Margin!"<<endl;}}     
        if(verbose){cout<<"Velocity command is "<<this->vel_cmd<<" with acceleration of "<<this->acc_cmd<<" ,jerk is "<<this->da_cmd<<endl;}
    }

private:
    gazebo::transport::NodePtr node;
    gazebo::transport::SubscriberPtr world_sub;
    gazebo::transport::SubscriberPtr statistics_sub;
    gazebo::transport::PublisherPtr pub;

    std::string worldTopicName = "~/world_state";
    std::string statisticsTopicName = "~/statistics";
    std::string commandTopicName = "~/client_command";

    int32_t simulation_round = 0;
    double priorcar_pos;                            // Define the position of the prior car
    double priorcar_vel;                            // Define the velocity of the prior car
    double priorcar_acc;                            // Define the acceleration of the prior car, here is always 0
    double vel_cmd;                                 // The velocity command sent to the ego car
    double acc_cmd;                                 // The acceleration command for deciding next vel_cmd
    double da_cmd;                                  // Jerk of every acceleration command
    const int K = 50;                               // Number of steps for prediction horizon
    const double Cv = 1.0;                          // Factor for velocity term in cost function
    const double Ca = 2.0;                          // Factor for acceleration term in cost function
    const double margin = -10.0;                    // Margin as the safety distance before the intersection
    const double vel_target = 20.0;                 // Take max velocity as the target velocity
    const double max_a = 1.99, min_a = -1.99, max_v = 20.0, min_v = 0.0; // Acceleration and velocity constraints setup
    const vector<float> da_list = { -0.19, -0.1, 0.0, 0.1, 0.19 };   // Jerk constraints setup
    const float dt = 0.1;                           // Time step in seconds
    const double yield_line = -20;                  // In the yield line ego car should defer to the prior car 
    bool YIELD;                                     // A signal indicating if the ego car should yield
    vector<double> priorcar_predicted_pos;          // A vector for storing predicted positions of the prior car in K steps
    vector<double> egocar_predicted_pos;            // A vector for storing predicted positions of the ego car in K steps
    vector<double> acc_list;                        // Acceleration candidates for calculating cost function
    vector<int> point_in_region_list;               // Point list that stored how many points are in the obstacle region
    vector<double> cost_list;                       // Calculated costs according to acceleration candidates
    vector<double> priorcar_surpass_pos_list;       // The list of positions of prior cars that can be surpassed
    vector<double> priorcar_yield_pos_list;         // The list of positions of prior cars that need to be yielded
    vector<double> priorcar_surpass_vel_list;       // The list of velocities of prior cars that can be surpassed
    vector<double> priorcar_yield_vel_list;         // The list of velocities of prior cars that need to be yielded
    int episode;                                    // Counting the simulation episode
    int success;                                    // Counting the number of success
    int collision;                                  // Counting the number of collision
    int points_in_region;                           // Number of points that are in the obstacle region for each acceleration
};

int main(int _argc, char **_argv)
{
    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    Controller controller;
    controller.Init();

    // for (int i = 0; i < 100; ++i)
    while (true)
        gazebo::common::Time::MSleep(100);

    // Make sure to shut everything down.
    gazebo::client::shutdown();
}