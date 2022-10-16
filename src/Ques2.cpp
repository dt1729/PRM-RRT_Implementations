#include <iostream>
#include <deque>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>
#include <unordered_map>
#include <queue>
#include <set>
#include <random>
#include <fstream>
#include <chrono>
#include "matplotlibcpp.h"

#define _USE_MATH_DEFINES

namespace plt = matplotlibcpp;

struct point{
    float x = 0.0;
    float y = 0.0;
};

class node{
    public:
    int val;
    float g = 0.0;
    float cost = INFINITY;
    node* parent;
    std::vector<node*> child;
    point coordinate;
    bool operator==(node rhs) const {return this->val == rhs.val;}
};

bool equal(node* lhs, node* rhs){
    return lhs -> val == rhs -> val?true:false;
}

struct compare{
    bool operator()(node* a, node* b){
        return a->cost > b->cost;
    }
};

struct pathInfo{
    std::vector<node*> path;
    float length;
};

class graph{
    public:
        std::unordered_map<node*, std::vector<std::pair<node*, float>>> adjList;
        std::unordered_map<node*, node*> backpointer;
        graph(){};
        void add_edge(node* a, node* b, float c);
};

class obstacle{
    public:
        // The obstacle class is supposed to represent one obstacle 
        // Assumed that points given in acw direction as required by obstacle definitions
        std::vector<std::pair<float,float>> obs_points;

        obstacle(std::vector<std::pair<float,float>> points){
            for(std::pair<float,float> p : points){
                obs_points.push_back(p);
            }
        }
        void plot();
        bool CheckIntersectionWObs(point pos);
};

class RRT{
    public:
        std::vector<obstacle>   Union_obstacle;
        std::vector<node*>      sampledNodes;
        float                   goalBias;
        float                   deltaT;
        float                   goalRegion;
        point                   min_vals;
        point                   max_vals;
        node                    start;
        node                    goal;
        int                     maxIters;

        RRT(float min_x, float min_y, float max_x, float max_y, std::vector<obstacle> uObs, point start, point goal, float goalBias, float deltaT, float goalRegion, int maxIters){
            this->min_vals.x = min_x; this->min_vals.y = min_y;
            this->max_vals.x = max_x; this->max_vals.y = max_y;
            this->goalBias   = goalBias; this->goalRegion = goalRegion;
            this->deltaT     = deltaT; 
            this->Union_obstacle = uObs;
            this->start.coordinate = start; this->start.val = 0; this->sampledNodes.push_back(&this->start);
            this->goal.coordinate = goal; this->goal.val = 1;
            this->maxIters = maxIters;
        }   

        void ExtendRRT();
        pathInfo RRTpath();
        void plotGraph();
        bool checkEdge(point p1, point p2);
        bool checkInObstacle(point p);
};


float angle_wrap(float angle);
void part_a();
void part_a_hundred();
void part_b();
void part_b_hundred();
void part_c();
void part_c_hundred();
int main(){
    part_a();
    part_a_hundred();
    part_b();
    part_b_hundred();
    part_c();
    part_c_hundred();
    return 0;
}

void part_a(){
    std::vector<std::pair<float,float>> p0  = {std::pair<float,float>{3.5,0.5},std::pair<float,float>{4.5,0.5},std::pair<float,float>{4.5,1.5},std::pair<float,float>{3.5,1.5},std::pair<float,float>{3.5,0.5}};
    std::vector<std::pair<float,float>> p1  = {std::pair<float,float>{6.5,-1.5},std::pair<float,float>{7.5,-1.5},std::pair<float,float>{7.5,-0.5},std::pair<float,float>{6.5,-0.5},std::pair<float,float>{6.5,-1.5}};

    obstacle obs(p0);
    obstacle obs1(p1);

    std::vector<obstacle> Union_obstacle;

    Union_obstacle.push_back(obs);
    Union_obstacle.push_back(obs1);

    point start; start.x = 0; start.y = 0;
    point goal ; goal.x  = 10; goal.y  = 0;

    RRT rrt(-1, -3, 11, 3, Union_obstacle, start, goal, 0.05, 0.5, 0.25, 5000); //float goalBias, float deltaT, float goalRegion, int maxIters
    rrt.ExtendRRT();
    pathInfo path = rrt.RRTpath();
    // if(smooth){
    //     path = prm.pathSmoothner(path);
    // }
    std::vector<float> px, py;
    for(node* p: path.path){
        px.push_back(p->coordinate.x); py.push_back(p->coordinate.y);
    }
    for(obstacle o:Union_obstacle){
        o.plot();
    }
    rrt.plotGraph();
    plt::plot(px, py, "-*r");
    std::cout << std::string("Total Length of Path: ")<< path.length << std::endl;
    plt::xlabel("X axes(m)");
    plt::ylabel("Y axes(m)");
    plt::show();
}

void part_a_hundred(){
    std::vector<std::pair<float,float>> p0  = {std::pair<float,float>{3.5,0.5},std::pair<float,float>{4.5,0.5},std::pair<float,float>{4.5,1.5},std::pair<float,float>{3.5,1.5},std::pair<float,float>{3.5,0.5}};
    std::vector<std::pair<float,float>> p1  = {std::pair<float,float>{6.5,-1.5},std::pair<float,float>{7.5,-1.5},std::pair<float,float>{7.5,-0.5},std::pair<float,float>{6.5,-0.5},std::pair<float,float>{6.5,-1.5}};

    obstacle obs(p0);
    obstacle obs1(p1);

    std::vector<obstacle> Union_obstacle;

    Union_obstacle.push_back(obs);
    Union_obstacle.push_back(obs1);

    point start; start.x = 0; start.y = 0;
    point goal ; goal.x  = 10; goal.y  = 0;

    std::ofstream length; length.open("length_rrt.csv");
    std::ofstream time; time.open("time_rrt.csv");
    std::ofstream valid; valid.open("valid_rrt.csv");

    for(int i = 0; i < 100; i++){
        std::cout << i << std::endl;
        RRT rrt(-1, -3, 11, 3, Union_obstacle, start, goal, 0.05, 0.5, 0.25, 5000); //float goalBias, float deltaT, float goalRegion, int maxIters
        auto start = std::chrono::high_resolution_clock::now();
        rrt.ExtendRRT();
        pathInfo path = rrt.RRTpath();
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        length << path.length << ",";
        time << duration.count()<< ",";
        valid << (path.path.size() > 0)<< ",";
        length << "\n";
        time << "\n";
        valid << "\n";
    }
    length.close();
    time.close();
    valid.close();
}

void part_b(){
    std::vector<std::pair<float,float>> p0  = {std::pair<float,float>{1,1},std::pair<float,float>{2,1},std::pair<float,float>{2,5},std::pair<float,float>{1,5},std::pair<float,float>{1,1}};
    std::vector<std::pair<float,float>> p1  = {std::pair<float,float>{3,3},std::pair<float,float>{4,3},std::pair<float,float>{4,12},std::pair<float,float>{3,12},std::pair<float,float>{3,3}};
    std::vector<std::pair<float,float>> p2  = {std::pair<float,float>{3,12},std::pair<float,float>{12,12},std::pair<float,float>{12,13},std::pair<float,float>{3,13},std::pair<float,float>{3,12}};
    std::vector<std::pair<float,float>> p3  = {std::pair<float,float>{12,5},std::pair<float,float>{13,5},std::pair<float,float>{13,13},std::pair<float,float>{12,13},std::pair<float,float>{12,5}};
    std::vector<std::pair<float,float>> p4  = {std::pair<float,float>{6,5},std::pair<float,float>{12,5},std::pair<float,float>{12,6},std::pair<float,float>{6,6},std::pair<float,float>{6,5}};

    obstacle obs(p0);
    obstacle obs1(p1);
    obstacle obs2(p2);
    obstacle obs3(p3);
    obstacle obs4(p4);

    std::vector<obstacle> Union_obstacle;

    Union_obstacle.push_back(obs);
    Union_obstacle.push_back(obs1);
    Union_obstacle.push_back(obs2);
    Union_obstacle.push_back(obs3);
    Union_obstacle.push_back(obs4);

    point goal, start; goal.x = 10.0; goal.y = 10.0; start.x = 0.0; start.y = 0.0; 

    RRT rrt(-1, -1, 15, 15, Union_obstacle, start, goal, 0.05, 0.5, 0.25, 5000); //float goalBias, float deltaT, float goalRegion, int maxIters
    rrt.ExtendRRT();
    pathInfo path = rrt.RRTpath();
    // if(smooth){
    //     path = prm.pathSmoothner(path);
    // }
    std::vector<float> px, py;
    for(node* p: path.path){
        px.push_back(p->coordinate.x); py.push_back(p->coordinate.y);
    }
    for(obstacle o:Union_obstacle){
        o.plot();
    }
    rrt.plotGraph();
    plt::plot(px, py, "-*r");
    std::cout << std::string("Total Length of Path: ")<< path.length << std::endl;
    plt::xlabel("X axes(m)");
    plt::ylabel("Y axes(m)");
    plt::show();

}
void part_b_hundred(){
    std::vector<std::pair<float,float>> p0  = {std::pair<float,float>{1,1},std::pair<float,float>{2,1},std::pair<float,float>{2,5},std::pair<float,float>{1,5},std::pair<float,float>{1,1}};
    std::vector<std::pair<float,float>> p1  = {std::pair<float,float>{3,3},std::pair<float,float>{4,3},std::pair<float,float>{4,12},std::pair<float,float>{3,12},std::pair<float,float>{3,3}};
    std::vector<std::pair<float,float>> p2  = {std::pair<float,float>{3,12},std::pair<float,float>{12,12},std::pair<float,float>{12,13},std::pair<float,float>{3,13},std::pair<float,float>{3,12}};
    std::vector<std::pair<float,float>> p3  = {std::pair<float,float>{12,5},std::pair<float,float>{13,5},std::pair<float,float>{13,13},std::pair<float,float>{12,13},std::pair<float,float>{12,5}};
    std::vector<std::pair<float,float>> p4  = {std::pair<float,float>{6,5},std::pair<float,float>{12,5},std::pair<float,float>{12,6},std::pair<float,float>{6,6},std::pair<float,float>{6,5}};

    obstacle obs(p0);
    obstacle obs1(p1);
    obstacle obs2(p2);
    obstacle obs3(p3);
    obstacle obs4(p4);

    std::vector<obstacle> Union_obstacle;

    Union_obstacle.push_back(obs);
    Union_obstacle.push_back(obs1);
    Union_obstacle.push_back(obs2);
    Union_obstacle.push_back(obs3);
    Union_obstacle.push_back(obs4);

    point goal, start; goal.x = 10.0; goal.y = 10.0; start.x = 0.0; start.y = 0.0; 
    std::ofstream length; length.open("length_rrt_w1.csv");
    std::ofstream time; time.open("time_rrt_w1.csv");
    std::ofstream valid; valid.open("valid_rrt_w1.csv");

    for(int i = 0; i < 100; i++){
        std::cout << i << std::endl;
        RRT rrt(-1, -1, 15, 15, Union_obstacle, start, goal, 0.05, 0.5, 0.25, 5000); //float goalBias, float deltaT, float goalRegion, int maxIters
        auto start = std::chrono::high_resolution_clock::now();
        rrt.ExtendRRT();
        pathInfo path = rrt.RRTpath();
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        length << path.length << ",";
        time << duration.count()<< ",";
        valid << (path.path.size() > 0)<< ",";
        length << "\n";
        time << "\n";
        valid << "\n";
    }
    length.close();
    time.close();
    valid.close();


}
void part_c(){
    std::vector<std::pair<float,float>> p0  = {std::pair<float,float>{-6,-6},std::pair<float,float>{25,-6},std::pair<float,float>{25,-5},std::pair<float,float>{-6,-5},std::pair<float,float>{-6,-6}};
    std::vector<std::pair<float,float>> p1  = {std::pair<float,float>{-6,5},std::pair<float,float>{30,5},std::pair<float,float>{30,6},std::pair<float,float>{-6,6},std::pair<float,float>{-6,5}};
    std::vector<std::pair<float,float>> p2  = {std::pair<float,float>{-6,-5},std::pair<float,float>{-5,-5},std::pair<float,float>{-5,5},std::pair<float,float>{-6,5},std::pair<float,float>{-6,-5}};
    std::vector<std::pair<float,float>> p3  = {std::pair<float,float>{4,-5},std::pair<float,float>{5,-5},std::pair<float,float>{5,1},std::pair<float,float>{4,1},std::pair<float,float>{4,-5}};
    std::vector<std::pair<float,float>> p4  = {std::pair<float,float>{9,0},std::pair<float,float>{10,0},std::pair<float,float>{10,5},std::pair<float,float>{9,5},std::pair<float,float>{9,0}};
    std::vector<std::pair<float,float>> p5  = {std::pair<float,float>{14,-5},std::pair<float,float>{15,-5},std::pair<float,float>{15,1},std::pair<float,float>{14,1},std::pair<float,float>{14,-5}};
    std::vector<std::pair<float,float>> p6  = {std::pair<float,float>{19,0},std::pair<float,float>{20,0},std::pair<float,float>{20,5},std::pair<float,float>{19,5},std::pair<float,float>{19,0}};
    std::vector<std::pair<float,float>> p7  = {std::pair<float,float>{24,-5},std::pair<float,float>{25,-5},std::pair<float,float>{25,1},std::pair<float,float>{24,1},std::pair<float,float>{24,-5}};
    std::vector<std::pair<float,float>> p8  = {std::pair<float,float>{29,0},std::pair<float,float>{30,0},std::pair<float,float>{30,5},std::pair<float,float>{29,5},std::pair<float,float>{29,0}};
    obstacle obs(p0);
    obstacle obs1(p1);
    obstacle obs2(p2);
    obstacle obs3(p3);
    obstacle obs4(p4);
    obstacle obs5(p5);
    obstacle obs6(p6);
    obstacle obs7(p7);
    obstacle obs8(p8);

    std::vector<obstacle> Union_obstacle;

    Union_obstacle.push_back(obs);
    Union_obstacle.push_back(obs1);
    Union_obstacle.push_back(obs2);
    Union_obstacle.push_back(obs3);
    Union_obstacle.push_back(obs4);
    Union_obstacle.push_back(obs5);
    Union_obstacle.push_back(obs6);
    Union_obstacle.push_back(obs7);
    Union_obstacle.push_back(obs8);

    point start; start.x = 0; start.y = 0;
    point goal ; goal.x  = 35; goal.y  = 0;

    RRT rrt(-7, -7, 40, 7, Union_obstacle, start, goal, 0.05, 0.5, 0.25, 5000); //float goalBias, float deltaT, float goalRegion, int maxIters
    rrt.ExtendRRT();
    pathInfo path = rrt.RRTpath();
    // if(smooth){
    //     path = prm.pathSmoothner(path);
    // }
    std::vector<float> px, py;
    for(node* p: path.path){
        px.push_back(p->coordinate.x); py.push_back(p->coordinate.y);
    }
    for(obstacle o:Union_obstacle){
        o.plot();
    }
    rrt.plotGraph();
    plt::plot(px, py, "-*r");
    std::cout << std::string("Total Length of Path: ")<< path.length << std::endl;
    plt::xlabel("X axes(m)");
    plt::ylabel("Y axes(m)");
    plt::show();
}

void part_c_hundred(){
    std::vector<std::pair<float,float>> p0  = {std::pair<float,float>{-6,-6},std::pair<float,float>{25,-6},std::pair<float,float>{25,-5},std::pair<float,float>{-6,-5},std::pair<float,float>{-6,-6}};
    std::vector<std::pair<float,float>> p1  = {std::pair<float,float>{-6,5},std::pair<float,float>{30,5},std::pair<float,float>{30,6},std::pair<float,float>{-6,6},std::pair<float,float>{-6,5}};
    std::vector<std::pair<float,float>> p2  = {std::pair<float,float>{-6,-5},std::pair<float,float>{-5,-5},std::pair<float,float>{-5,5},std::pair<float,float>{-6,5},std::pair<float,float>{-6,-5}};
    std::vector<std::pair<float,float>> p3  = {std::pair<float,float>{4,-5},std::pair<float,float>{5,-5},std::pair<float,float>{5,1},std::pair<float,float>{4,1},std::pair<float,float>{4,-5}};
    std::vector<std::pair<float,float>> p4  = {std::pair<float,float>{9,0},std::pair<float,float>{10,0},std::pair<float,float>{10,5},std::pair<float,float>{9,5},std::pair<float,float>{9,0}};
    std::vector<std::pair<float,float>> p5  = {std::pair<float,float>{14,-5},std::pair<float,float>{15,-5},std::pair<float,float>{15,1},std::pair<float,float>{14,1},std::pair<float,float>{14,-5}};
    std::vector<std::pair<float,float>> p6  = {std::pair<float,float>{19,0},std::pair<float,float>{20,0},std::pair<float,float>{20,5},std::pair<float,float>{19,5},std::pair<float,float>{19,0}};
    std::vector<std::pair<float,float>> p7  = {std::pair<float,float>{24,-5},std::pair<float,float>{25,-5},std::pair<float,float>{25,1},std::pair<float,float>{24,1},std::pair<float,float>{24,-5}};
    std::vector<std::pair<float,float>> p8  = {std::pair<float,float>{29,0},std::pair<float,float>{30,0},std::pair<float,float>{30,5},std::pair<float,float>{29,5},std::pair<float,float>{29,0}};
    obstacle obs(p0);
    obstacle obs1(p1);
    obstacle obs2(p2);
    obstacle obs3(p3);
    obstacle obs4(p4);
    obstacle obs5(p5);
    obstacle obs6(p6);
    obstacle obs7(p7);
    obstacle obs8(p8);

    std::vector<obstacle> Union_obstacle;

    Union_obstacle.push_back(obs);
    Union_obstacle.push_back(obs1);
    Union_obstacle.push_back(obs2);
    Union_obstacle.push_back(obs3);
    Union_obstacle.push_back(obs4);
    Union_obstacle.push_back(obs5);
    Union_obstacle.push_back(obs6);
    Union_obstacle.push_back(obs7);
    Union_obstacle.push_back(obs8);

    point start; start.x = 0; start.y = 0;
    point goal ; goal.x  = 35; goal.y  = 0;

    std::ofstream length; length.open("length_rrt_w2.csv");
    std::ofstream time; time.open("time_rrt_w2.csv");
    std::ofstream valid; valid.open("valid_rrt_w2.csv");

    for(int i = 0; i < 100; i++){
        std::cout << i << std::endl;
        RRT rrt(-7, -7, 40, 7, Union_obstacle, start, goal, 0.05, 0.5, 0.25, 5000); //float goalBias, float deltaT, float goalRegion, int maxIters
        auto start = std::chrono::high_resolution_clock::now();
        rrt.ExtendRRT();
        pathInfo path = rrt.RRTpath();
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        length << path.length << ",";
        time << duration.count()<< ",";
        valid << (path.path.size() > 0)<< ",";
        length << "\n";
        time << "\n";
        valid << "\n";
    }
    length.close();
    time.close();
    valid.close();
}

void graph::add_edge(node* a, node* b, float weight){
    this->adjList[a].push_back(std::pair<node*,float>{b,weight});
    this->adjList[b].push_back(std::pair<node*,float>{a,weight});
}

void obstacle::plot(){
    std::vector<float> x_vals, y_vals;
    for(int i = 0; i < this->obs_points.size(); i++){
        x_vals.push_back(this->obs_points[i].first);
        y_vals.push_back(this->obs_points[i].second);
    }
    plt::named_plot("obstacle ",x_vals,y_vals,"*-");
}

bool obstacle::CheckIntersectionWObs(point pos){
    // Sum of angles of the point with each vertex point sums to 360 degrees if inside the obstacle
    int n                       = this->obs_points.size();
    float my_sum                = 0;
    bool intersection           = false;
    float prev_min              = INFINITY;
    float dist_from_line        = 0;
    int line_cnt                = 0;

    for(int i = 0; i < this->obs_points.size()-1; i++){
        // my_sum = sum of all interior angles; interior angles = angle of pos.point vec  - angle of pos.next point vec
        float ang           = std::atan2(this->obs_points[(i+1)%n].second - pos.y, this->obs_points[(i+1)%n].first - pos.x) 
                            - std::atan2(this->obs_points[i].second - pos.y, this->obs_points[i].first - pos.x);
        ang                 = angle_wrap(ang);
        my_sum              += ang;
    }
    if (std::abs(my_sum)    >= M_PI){
        intersection        = true;
    } 
    return intersection;
}

float dist(point p1, point p2){
    return std::sqrt(std::pow(p2.y - p1.y,2) + std::pow(p2.x - p1.x,2));
}

void RRT::ExtendRRT(){
    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()

    std::uniform_real_distribution<float> disx(this->min_vals.x, this->max_vals.x);
    std::uniform_real_distribution<float> disy(this->min_vals.y, this->max_vals.y);
    std::uniform_real_distribution<float> disGoal(0, 1);

    int count = 0; 
    while(count < this->maxIters){
        // Generate a node (with bias to deltaT)
        // Find nearest node in the sampledNodes
        // Find point at deltaT distance from nearest node 
        // Add node to tree(Update parent and child pointer in nodes) if edge and node is free else skip entirely
        
        float goalSelect = disGoal(gen);
        node* temp;
        node* temp1;
        float minDist = INFINITY;
        if(goalSelect < this->goalBias){
            temp          = &goal;
        }
        else{
            node* nNew = new node; nNew->coordinate.x = disx(gen); nNew->coordinate.y = disy(gen); 
            temp = nNew;
        }
        
        for(node* n: this->sampledNodes){
            float d = dist(n->coordinate, temp->coordinate);
            if(d < minDist){
                minDist = d;
                temp1 = n;
            }
        }

        float angle = std::atan2(temp->coordinate.y - temp1->coordinate.y, temp->coordinate.x - temp1->coordinate.x); 
        node* nn = new node;
        nn->coordinate.x = this->deltaT*std::cos(angle) + temp1->coordinate.x;
        nn->coordinate.y = this->deltaT*std::sin(angle) + temp1->coordinate.y;
        nn->val          = count + 2;

        if(this->checkEdge(temp->coordinate, nn->coordinate) || this->checkInObstacle(nn->coordinate)){
            continue;
        }
        else{
            nn->parent = temp1;
            temp1->child.push_back(nn);
            this->sampledNodes.push_back(nn);
        }

        if(dist(nn->coordinate, this->goal.coordinate) < this->goalRegion){
            break;
        }
    }
}

pathInfo RRT::RRTpath(){
    node* goalr; // Node in goal region
    node* temp;
    float len = 0.0;
    std::vector<node*> ansVec;

    for(node* n: this->sampledNodes){
        float d = dist(n->coordinate, this->goal.coordinate);
        if(d < this->goalRegion){
            goalr = n;
            break;
        }
    }
    temp = goalr;
    while(temp->val != 0){
        temp = temp->parent;
        ansVec.push_back(temp);
        len += dist(temp->parent->coordinate, temp->coordinate);
    }

    pathInfo path; path.path = ansVec; path.length = len;
    return path;
}


bool RRT::checkEdge(point p1, point p2){
    float t = 0.0; 
    float delta_t = 0.01; 
    // Check line parametrically
    while(t <= 1.0){
        point p; 
        p.x = (1-t)*(p1.x) + t*(p2.x); 
        p.y = (1-t)*p1.y + t*p2.y;

        for(obstacle o: this->Union_obstacle){
            if(o.CheckIntersectionWObs(p)){
                return true;
            }
        }

        t += delta_t;
    }
    return false;
}

bool RRT::checkInObstacle(point p){
    for(obstacle o: this->Union_obstacle){
        if(o.CheckIntersectionWObs(p)){
            return true;
        }
    }
    return false;
}

float angle_wrap(float angle){
    if(angle > M_PI){
        return -1*(2*M_PI-angle);
    }
    else if(angle < -M_PI){
        return -1*(-2*M_PI-angle);
    }
    return angle;
}

void RRT::plotGraph(){
        for(node* p:this->sampledNodes){
            for(node* pc: p->child){
                plt::plot(std::vector<float>{pc->coordinate.x, p->coordinate.x}, std::vector<float>{pc->coordinate.y, p->coordinate.y},"o-k");
            }
        }
}