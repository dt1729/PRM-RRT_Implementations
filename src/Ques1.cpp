#include <iostream>
#include <deque>
#include <cmath>
#include <vector>
#include <algorithm>
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

class PRM{
    public:
        std::vector<obstacle>   Union_obstacle;
        std::vector<node*>      sampledNodes;
        graph                   g;
        point                   min_vals;
        point                   max_vals;
        node                    start;
        node                    goal;

        PRM(float min_x, float min_y, float max_x, float max_y, std::vector<obstacle> uObs, point start, point goal){
            this->min_vals.x = min_x; this->min_vals.y = min_y;
            this->max_vals.x = max_x; this->max_vals.y = max_y;
            this->Union_obstacle = uObs;
            this->start.coordinate = start; this->start.val = 0; this->sampledNodes.push_back(&this->start);
            this->goal.coordinate = goal; this->goal.val = 1; this->sampledNodes.push_back(&this->goal);
        }

        void generateRoadMap(int N, float r);
        bool checkEdge(point p1, point p2, obstacle o);
        pathInfo findPath();
        void plotGraph();
};

float angle_wrap(float angle);
pathInfo Djikstra(graph g, node* start, node* goal);
float dist(point p1, point p2);
pathInfo find_path(graph g, node* node_n, node* start);
void part_a();
void part_b();

int main(){ 
    // part_a();
    part_b();
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

    PRM prm(-1, -3, 11, 3, Union_obstacle, start, goal);
    prm.generateRoadMap(200, 1);
    pathInfo path = prm.findPath();
    std::vector<float> px, py;
    for(node* p: path.path){
        px.push_back(p->coordinate.x); py.push_back(p->coordinate.y);
        std::cout << p->coordinate.x << " " << p->coordinate.y << std::endl;
    }
    for(obstacle o:Union_obstacle){
        o.plot();
    }
    plt::plot(px, py, "-*");
    // prm.plotGraph();
    // plt::legend();
    plt::xlabel("X axes(m)");
    plt::ylabel("Y axes(m)");
    plt::show();
}

void part_b(){
    std::vector<std::pair<float,float>> p0  = {std::pair<float,float>{3.5,0.5},std::pair<float,float>{4.5,0.5},std::pair<float,float>{4.5,1.5},std::pair<float,float>{3.5,1.5},std::pair<float,float>{3.5,0.5}};
    std::vector<std::pair<float,float>> p1  = {std::pair<float,float>{6.5,-1.5},std::pair<float,float>{7.5,-1.5},std::pair<float,float>{7.5,-0.5},std::pair<float,float>{6.5,-0.5},std::pair<float,float>{6.5,-1.5}};

    obstacle obs(p0);
    obstacle obs1(p1);

    std::vector<obstacle> Union_obstacle;

    Union_obstacle.push_back(obs);
    Union_obstacle.push_back(obs1);

    point start; start.x = 0; start.y = 0;
    point goal ; goal.x  = 10; goal.y  = 0;

    std::vector<std::pair<float,float>> Nr{std::pair<float,float>{200,0.5}, std::pair<float,float>{200,1},std::pair<float,float>{200,1.5},std::pair<float,float>{200,2},std::pair<float,float>{500,0.5},std::pair<float,float>{500,1},std::pair<float,float>{500,1.5},std::pair<float,float>{500,2}};

    std::ofstream length; length.open("length.csv");
    std::ofstream time; time.open("time.csv");
    std::ofstream valid; valid.open("valid.csv");

    for(int i = 0; i < 100; i++){
        for(std::pair<float,float> nr: Nr){
            std::cout << nr.first << " " << nr.second << std::endl;
            PRM prm(-1, -3, 11, 3, Union_obstacle, start, goal);
            auto start = std::chrono::high_resolution_clock::now();
            prm.generateRoadMap(nr.first, nr.second);
            pathInfo path = prm.findPath();
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
            length << path.length << ",";
            time << duration.count()<< ",";
            valid << (path.path.size() > 0)<< ",";
        }
        length << "\n";
        time << "\n";
        valid << "\n";
    }
    length.close();
    time.close();
    valid.close();
}

void PRM::generateRoadMap(int N, float r){
    /*
    *   Formulate N random points in free space and store in sampledNodes
    *   for each point in sampledNodes
            * For points in radius r, 
                * check if there is a connection between point
                    * if not add an edge
                * else continue checking      
    */

    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<float> disx(this->min_vals.x, this->max_vals.x);
    std::uniform_real_distribution<float> disy(this->min_vals.y, this->max_vals.y);

    int count = 2;

    while(count < N){
        bool check = true;
        point pt; pt.x = disx(gen); pt.y = disy(gen);
        for(obstacle o:this->Union_obstacle){
            if(o.CheckIntersectionWObs(pt)){
                check &= false;
                break; 
            }
        }
        if(!check){
            continue;
        }
        else{
            node* n = new node;
            n->val = count; n->coordinate = pt;
            this->sampledNodes.push_back(n);
            count++;
        }
    }

    for(node* p: this->sampledNodes){
        for(node* pi: this->sampledNodes){
            if(pi == p){ // Two points are not same condition
                continue;
            }
            else if(dist(p->coordinate,pi->coordinate) < r){ // distance between points less than r get added to the graph
                bool check = false;
                for(std::pair<node*,float> n: this->g.adjList[p]){
                    if(n.first->val == pi->val){
                        check = true;
                        break;
                    }
                }
                if(!check){
                    // Add check for edge intersection with obstacle
                    bool edgeIntersection = false;
                    for(obstacle o:this->Union_obstacle){
                        if(this->checkEdge(p->coordinate, pi->coordinate,o)){
                            edgeIntersection = true;
                        } 
                    }
                    if(!edgeIntersection){
                        this->g.add_edge(p,pi,dist(p->coordinate, pi->coordinate));
                    }
                    else{
                        continue;
                    }
                }
                else{
                    continue;
                }
            }
        }
    }
}

bool PRM::checkEdge(point p1, point p2, obstacle o){
    float t = 0.0; 
    float delta_t = 0.05; 
    // Check line parametrically
    while(t <= 1.0){
        point p; 
        p.x = (1-t)*(p1.x) + t*(p2.x); 
        p.y = (1-t)*p1.y + t*p2.y;
        if(o.CheckIntersectionWObs(p1)){
            return true;
        }
        t += delta_t;
    }
    return false;
}

void PRM::plotGraph(){
    for(auto const& a:this->g.adjList){
        for(std::pair<node*,float> p:a.second){
            std::cout << a.first->coordinate.x << " " << p.first->coordinate.x << std::endl;
            plt::plot(std::vector<float>{a.first->coordinate.x, p.first->coordinate.x}, std::vector<float>{a.first->coordinate.x, p.first->coordinate.x});
        }  
    }
}

pathInfo PRM::findPath(){
    return Djikstra(this->g, &this->start, &this->goal);
}


pathInfo Djikstra(graph g, node* start, node* goal){

    // While you reach the final point repeat the following steps
    /*
    * Make priority queue to show set
    * pick top of priority queue = n_best and add to a set C
    * if n_best = goal
    *   exit
    * Search all the neighbors of the best node that are not in C
        * if x is not in O 
        *   add x to O
        * else if g(nbest) + c(nbest,x) < g(x) then
        *   update nbest as x
    */

    std::priority_queue<node*, std::vector<node*>, compare> O;
    std::set<node*> C;

    int count = 0;

    O.push(start); // Push start node 
    start->cost = 0.0;

    while(!O.empty()){
        node* n_best = O.top(); O.pop();
        C.insert(n_best);
        count++;

        if(n_best == goal){
            std::cout << "Total Djikstra iterations: " << count << std::endl;
            return find_path(g, n_best, start);
        } 

        for(std::pair<node*, float> nn: g.adjList[n_best]){
            node* x = nn.first;
            float cost_tot = n_best->g + nn.second; // g_func(cost up until now) + c(weight of the graph)
            if(C.find(x) == C.end()){
                std::priority_queue<node*, std::vector<node*>, compare> temp = O;
                bool check = false;
                while(!temp.empty()){
                    if(temp.top()->val == x->val){
                        check = true;
                        goto t;
                    }
                    temp.pop();
                }
                t:;
                if(!check){
                    x->cost = cost_tot;
                    x->g = cost_tot;
                    O.push(x);
                    g.backpointer[x] = n_best;
                }
                else if(cost_tot < x->g){
                        x->cost = cost_tot;
                        x->g = cost_tot;
                        g.backpointer[x] = n_best;
                        O.push(x);
                }
            }
        }
   }
   pathInfo p; p.path = std::vector<node*>{}; p.length = 0.0;
   return p;
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

pathInfo find_path(graph g, node* node_n, node* start){
    pathInfo ret;
    std::vector<node*> ans;
    node* n = node_n;
    float len = 0;
    while(n != start){
        ans.push_back(n);
        for(std::pair<node*,float> a:g.adjList[g.backpointer[n]]){
            if(equal(a.first,n)) len+=a.second;
        }
        n = g.backpointer[n]; 
    }
    ans.push_back(start);
    std::reverse(ans.begin(),ans.end());
    std::cout << "Path length: " << len << std::endl << std::endl; 
    ret.path = ans; ret.length = len;
    return ret;
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