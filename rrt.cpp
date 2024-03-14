#include <chrono>
#include <cmath>
#include <thread>
#include <opencv2/opencv.hpp> 
#include <random>
#include <ctime>


#define START_X 50
#define START_Y 50
#define GOAL_X 1400
#define GOAL_Y 820
#define WIDTH 1500  // With of Image
#define HEIGHT 900  // Height of Image

using namespace std::chrono_literals;

//GraphNode class to store parent info with each node
class GraphNode
{
    public:
    int parent_idx;
    cv::Point2i parent;//Parent Node location
    cv::Point2i location;//Self location

    
    GraphNode()
    {
        parent_idx=0;
        this->parent=cv::Point2i(0,0);
        this->location=cv::Point2i(0,0);
    }

    GraphNode(cv::Point2i parent,cv::Point2i location, int parent_idx)
    {
        this->parent_idx=parent_idx;
        this->parent=parent;
        this->location=location;
    }
};

//Implementation of RRT
class RRT
{
    public:
    int MAX_X=0;                    // Maximum width dimension of image
    int MAX_Y=0;                    // Maximum height dimension of image
    int MAX_D=50;                   // Maximum step distace
    int MAX_ITERATIONS=1000;        // Maximum iterations alowed for sampling
    int DIST_THRESHOLD=50;          // Threashold to reach to goal
    cv::Point2i start_;             // Start Point
    cv::Point2i goal_;              // Goal Point
    cv::Mat map_;                   // Empty image
    std::vector<GraphNode> nodes_;  // Collection of GraphNodes-


    RRT(int MAX_X,int MAX_Y, cv::Point2i start, cv::Point2i goal, cv::Mat map_)
    {
        this->MAX_X=MAX_X;
        this->MAX_Y=MAX_Y;
        this->start_=start_;
        this->goal_=goal;
        this->map_=map_;

        GraphNode g(start_,start_,0); //First GraphNode
        this->nodes_.push_back(g);
        for(int i=0;i<this->MAX_ITERATIONS;i++)
        {
            int nearest_idx=0;
            cv::Point2i new_pt;
            while(1)  // Sampling new point untill a point without obstacle found
            {
                new_pt =this->random_pt();
                nearest_idx=this->nearest_to(new_pt);
                if(!this->obstacle_bw(new_pt,this->nodes_[nearest_idx].location))
                {
                    break;
                }
            }

            // If the sampled point is far away take an intercept within max step length
            if (this->distance(new_pt ,this->nodes_[nearest_idx].location)>this->MAX_D){
                float dx = new_pt.x - this->nodes_[nearest_idx].location.x;
                float dy = new_pt.y - this->nodes_[nearest_idx].location.y;

                // Calculate the length of the line segment
                float length = distance(this->nodes_[nearest_idx].location, new_pt);

                // Calculate the unit vector
                double ux = dx / length;
                double uy = dy / length;

                // Calculate the coordinates of the new point
                int newX = this->nodes_[nearest_idx].location.x + this->MAX_D * ux;
                int newY = this->nodes_[nearest_idx].location.y + this->MAX_D * uy;
                new_pt=cv::Point2i(newX,newY);
            }
            
            //Add the sampled point to the vector of GraphNodes
            GraphNode new_node(this->nodes_[nearest_idx].location,new_pt,nearest_idx);
            nodes_.push_back(new_node);

            // If the sampled point is close enough to the goal
            if (this->distance(goal_,new_pt)<=this->DIST_THRESHOLD){
                std::cout<<"Goal Found after "<<i<<" iterations"<<std::endl;
                break;  //Stop  search
            }

            // If maximum number of iterations done print info
            if (i==MAX_ITERATIONS-1){
                std::cout<<"Goal Not Found"<<std::endl;
            }
        }
    }




    // Function to check if path between any two points is blocked
    bool obstacle_bw(cv::Point2i pt1, cv::Point2i pt2)
    {
        // Line Equation y= mx+c
        float denom=(pt2.x-pt1.x);
        float m;
        if(denom!=0){
            m= (pt2.y-pt1.y)/denom;
            float c= pt1.y-m*pt1.x;

            //For each x b/w pt2 & pt1 find y and check if its blocked
            int increment=0;
            if(pt2.x > pt1.x){
                increment=1;
            }
            else{
                increment=-1;
            }
            int x_=pt1.x;

            while(x_!=pt2.x){
                int y=m*x_+c;
                if(y>1){
                    if (this->map_.at<uchar>(y,x_)!=255 || this->map_.at<uchar>(y+1,x_)!=255 || this->map_.at<uchar>(y-1,x_)!=255)
                    {
                        return true;
                    }
                }
                x_+=increment;
            }
        }
        else //Verticle Line
        {
            int y=pt1.y;
            int increment=0;
            if(pt2.y > pt1.y){
                increment=1;
            }
            else{
                increment=-1;
            }
            while(y !=pt2.y){
                if(y>1){
                    if (this->map_.at<uchar>(y,pt1.x)!=255)
                    {
                        return true;
                    }
                }
                y+=increment;
            }

        }
        return false;
    }

    //Function to sample a random Point
    cv::Point2i random_pt()
    {
        // Seed the random number generator
        std::random_device rd;
        std::mt19937 rng(rd());
        
        // Define the distribution
        std::uniform_int_distribution<int> distributionx(0, this->MAX_X);
        std::uniform_int_distribution<int> distributiony(0, this->MAX_Y);

        int x = distributionx(rng);
        int y = distributiony(rng);

        return cv::Point2i(x,y);

    }

    //Find index of the nearest node to a sampled point
    int nearest_to(cv::Point2i pt)
    {
        float nearest_dist=INT_MAX;
        int nearest_node_idx=0;
        for(int i=0;i<this->nodes_.size();i++)
        {
            if (distance(pt, nodes_[i].location) < nearest_dist)
            {
                nearest_dist=distance(pt, nodes_[i].location);
                nearest_node_idx=i;
            }
        }
        return nearest_node_idx;
    }

    //Function to check distance b/w two points
    float distance(cv::Point2i pt1, cv::Point2i pt2)
    {
        return sqrt(pow(pt1.x-pt2.x,2)+pow(pt1.y-pt2.y,2));
    }


};


int main()
{
    // Create a maze using opencv
    // Read image from file
    cv::Mat map_=cv::imread("map.jpg",cv::IMREAD_GRAYSCALE);
    
    //Take Start & Goal point
    cv::Point2i start_(START_X,START_Y); 
    cv::Point2i goal_(GOAL_X,GOAL_Y); 

    RRT rrt_(WIDTH,HEIGHT,start_,goal_,map_);

    cv::Mat map_color;
    cv::cvtColor(map_,map_color,cv::COLOR_GRAY2BGR);
    cv::circle(map_color, start_, 5,cv::Scalar(0,0,255), 2);
    cv::circle(map_color, goal_, 5,cv::Scalar(0,255,0), 2);
    cv::circle(map_color, goal_, rrt_.DIST_THRESHOLD,cv::Scalar(0,255,0), 1);

    // Draw points & lines to show RRT Algorithm 
    for(int i=0;i<rrt_.nodes_.size();i++)
    {
        cv::circle(map_color, rrt_.nodes_.at(i).location, 5,cv::Scalar(255,0,0), 2);
        cv::line(map_color,rrt_.nodes_.at(i).location,rrt_.nodes_.at(i).parent,cv::Scalar(0,255,0),1);
    }
    cv::line(map_color,rrt_.nodes_.back().location,goal_,cv::Scalar(0,255,0),1);

    // Back Trace the Path
    GraphNode node=rrt_.nodes_.back();
    while(node.parent_idx!=0)
    {
        cv::line(map_color,node.location,node.parent,cv::Scalar(0,0,255),3);
        node=rrt_.nodes_[node.parent_idx];
    }
    cv::line(map_color,node.location,start_,cv::Scalar(0,0,255),3);

    //Save the generated image
    cv::imwrite("output.jpg", map_color);
    cv::imshow("Map ",map_color);
    cv::waitKey(0);
    return 0;
}

