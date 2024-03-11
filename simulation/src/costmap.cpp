#include <iostream>
#include <cmath>

#include <vector>
#include <queue>
#include <tuple>

#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/TFMessage.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pathpub;
nav_msgs::Path PATH;

class graph{
    public:
        std::vector<float> map;
        int X, Y;
        float resolution;
        int originx, originy;

        graph(){}
        bool ready = false;
        std::vector<int> neighbours(int n, int depth){
            std::vector<int> ne;
            int x = n%X;
            int y = n/X;

            for (int dy = -depth; dy <= depth; dy++){
                for (int dx = -depth; dx <= depth; dx++){
                    if (dx == dy && dy == 0) continue;
                    int i = dx + x, j = dy + y;
                    if (valid(i, j)){
                        int N = i + j * X;
                        if (map[N] == 100) continue;
                        ne.push_back(N);
                    }
                }
            }return ne;
        }

        std::vector<int> outline(int n, int depth){
            std::vector<int> ne;
            int x = n%X;
            int y = n/X;

            for (int dy = -depth; dy <= depth; dy += depth*2){
                for (int dx = -depth; dx <= depth; dx++){
                    int i = dx + x, j = dy + y;
                    if (valid(i, j)){
                        int N = i + j * X;
                        if (map[N] == 100) continue;
                        ne.push_back(N);
                    }
                }
            }for (int dy = -depth+1; dy <= depth-1; dy++){
                for (int dx = -depth; dx <= depth; dx += depth*2){
                    int i = dx + x, j = dy + y;
                    if (valid(i, j)){
                        int N = i + j * X;
                        if (map[N] == 100) continue;
                        ne.push_back(N);
                    }
                }
            }return ne;
        }

        float heuristic(int n, int end){
            //return digna(n, end);
            return distance(n, end);
        }

        float gcost(int start, int n){
            return digna(start, n);
        }

        std::vector<int> aStar(int start, int end){
            std::vector<int> path;
            std::vector<int> parent(X*Y, -1);
            std::vector<int> explored(X*Y, 0);
            for (int t = 0; t < X*Y; t++){
                parent[t] = t;
            }
            std::vector<float> gc(X*Y, INFINITY);
            std::vector<float> fc(X*Y, INFINITY);

            float g = gcost(start, start);

            gc[start] = g;
            fc[start] = heuristic(start, end) + g;

            std::priority_queue<std::tuple<float, float, int>> open; // -fcost, gcost, n
            open.push(std::make_tuple(-fc[start], gc[start], start));

            while (!open.empty()){
                std::tuple<float, float, int> curr = open.top();
                if (std::get<2>(curr) == end){
                    PATH.header.stamp = ros::Time(0);
                    int th = end;
                    int t = 0;
                    while (parent[th] != th){
                        geometry_msgs::PoseStamped poseStamped;

                        poseStamped.header.frame_id = "odom";
                        poseStamped.header.stamp = ros::Time(0);
    
                        poseStamped.pose.position.x = th%X;
                        poseStamped.pose.position.y = th/X;
                        
                        PATH.poses.push_back(poseStamped);
                        
                        path.push_back((float)th);
                        th = parent[th];
                    }path.push_back(start);
                    pathpub.publish(PATH);
                    system("sleep 0.1");
                    pathpub.publish(PATH);
                    system("sleep 0.1");
                    pathpub.publish(PATH);
                    return path;
                }open.pop();
                explored[std::get<2>(curr)] = 1;
                std::vector<int> neigh = neighbours(std::get<2>(curr), 1);
                for (int N = 0; N < neigh.size(); N++){
                    int no = neigh[N];
                    float gexp = std::get<1>(curr) + gcost(std::get<2>(curr), no);
                    if (gexp < gc[no]){
                        gc[no] = gexp;
                        fc[no] = gexp + heuristic(no, end);
                        parent[no] = std::get<2>(curr);
                        if (explored[no]) continue;
                        open.push(std::make_tuple(-fc[no], gc[no], no));
                    }
                }
            }return path;
        }

    private:
        float digna(int n1, int n2){
            if (n1 == n2) return 0.0;

            int dx = abs(n1%X - n2%X);
            int dy = abs(n1/X - n2/X);

            return std::min(dx, dy) * sqrt(2) + abs(dx - dy);
        }

        float distance(int n1, int n2){
            if (n1 == n2) return 0.0;

            int dx = abs(n1%X - n2%X);
            int dy = abs(n1/X - n2/X);
        
            return dx*dx + dy*dy;
        }
    
        bool valid(int i, int j){
            bool boolx = (0 <= i && i < X);
            bool booly = (0 <= j && j < Y);
            return boolx && booly;
        }
};

ros::Publisher pub;
geometry_msgs::Pose bot;
int botX = -1, botY = -1;

geometry_msgs::Pose goal;
int goalX = -1, goalY = -1;

std::vector<int> path;
graph G;

geometry_msgs::Twist zeroV;
geometry_msgs::Pose zeroP;

geometry_msgs::Twist vel;

class Controller{
    public:
        std::vector<int> wayPoints;

        Controller(){}
        Controller(std::vector<int> points){
            wayPoints = points;
        }

        int getBestNodeIndex(geometry_msgs::Pose me){
            float minD = wayPoints[0];
            int i = 0;
            for (int t = 1; t < wayPoints.size(); t++){
                int n = wayPoints[t];
                float dx = (n%G.X * G.resolution) + G.originx - me.position.x;
                float dy = (n/G.X * G.resolution) + G.originy - me.position.y;
                float distance = dx*dx + dy*dy;
                if (distance < minD){
                    minD = distance;
                    i = t;
                }
            }return i;
        }	

        void run(geometry_msgs::Pose me, int lookAD){
            int bestI = getBestNodeIndex(me);
            int targetNode = wayPoints[std::max(bestI - lookAD, 0)];

            if (G.map[targetNode] == 100){
                //std::cout << "re-planning path..." << '\n';
                path = G.aStar(botX + botY*G.X, goalX + goalY*G.X);
                return ;
            }

            float dx = targetNode%G.X * G.resolution + G.originx - me.position.x;
            float dy = targetNode/G.X * G.resolution + G.originy - me.position.y;

            transform(dx, dy, me);
            float steerAngularV = getSteering(dx, dy) * vel.linear.x;

            vel.angular.z = steerAngularV;
            pub.publish(vel);
        }

        void transform(float &dx, float &dy, geometry_msgs::Pose pose){
            float theta = M_PI/2 - getPoseAngle(pose);
            float dx_ = dx * cos(theta) - dy * sin(theta);
            float dy_ = dx * sin(theta) + dy * cos(theta);
            dx = dx_;
            dy = dy_;
        }

        float getSteering(float dx, float dy){
            float theta = atan2(-dx, dy);
            float l = sqrt(dx * dx + dy * dy);
            return 2 * sin(theta/2) / l;
        }

        static float getPoseAngle(geometry_msgs::Pose pose){
            float z = pose.orientation.z;
            float w = pose.orientation.w;
            float theta = 2*atan2(z, w);
            if (theta < 0) theta += 2 * M_PI;
            return theta;	
        }
        
        static float getRotation(float targetAngle, float myAngle){
            float err = targetAngle - myAngle;
            if (err > 0){
                if (err < M_PI) return 1;
                return -1;
            }if (-err < M_PI) return -1;
            return 1;
        }
};

void turn(geometry_msgs::Pose pose){
    float max_err_t = 0.09; //radians

    float theta = Controller::getPoseAngle(pose);
    float myAngle = Controller::getPoseAngle(bot);
    
    geometry_msgs::Twist v;
    
    v.linear.x = 0;
    v.linear.y = 0;
    v.linear.z = 0;

    v.angular.x = 0;
    v.angular.y = 0;
    v.angular.z = Controller::getRotation(theta, myAngle)*0.2;

    pub.publish(v);
    while (ros::ok() && abs(Controller::getPoseAngle(bot) - theta) > max_err_t){

    }pub.publish(zeroV);
}

void turn(float targetAngle){
    float max_err_t = 0.09;
    float myAngle = Controller::getPoseAngle(bot);

    if (abs(myAngle - targetAngle) < max_err_t){
        return ;
    }

    geometry_msgs::Twist v;

    v.linear.x = 0;
    v.linear.y = 0;
    v.linear.z = 0;

    v.angular.x = 0;
    v.angular.y = 0;
    v.angular.z = Controller::getRotation(targetAngle, myAngle)*0.2;

    pub.publish(v);
    while (ros::ok() && abs(Controller::getPoseAngle(bot)-targetAngle) > max_err_t){
        //std::cout << Controller::getPoseAngle(bot) << ',' << targetAngle << '\n';
    }pub.publish(zeroV);
}

bool checkDistance(geometry_msgs::Pose me, geometry_msgs::Pose target){
    float max_err_d = 0.07;
    float dx = abs(target.position.x - me.position.x);
    float dy = abs(target.position.y - me.position.y);
    return (dx < max_err_d && dy < max_err_d);
}


Controller PP;
bool fready = false;

//callbacks
void getGoal(const geometry_msgs::PoseStamped::ConstPtr &fin){
    goal = fin->pose;
    goalX = (goal.position.x - G.originx)/G.resolution;
    goalY = (goal.position.y - G.originy)/G.resolution;

    path = G.aStar(botX + botY * G.X, goalX + goalY * G.X);

    int lookAD = 14;

    if (path.size() == 0){
        std::cout << "no path found" << '\n';
        return ;
    }

    int loc = path.size()-1-lookAD;
    
    float dx = (path[loc]%G.X - botX)*G.resolution;
    float dy = (path[loc]/G.X - botY)*G.resolution;

    float targetAngle = atan2(dy, dx);
    if (targetAngle < 0) targetAngle += 2*M_PI;

    if (abs(targetAngle) > M_PI/4){
        turn(targetAngle);
    }

    PP.wayPoints = path; 
    while (ros::ok() && !checkDistance(bot, goal)){
        PP.run(bot, lookAD);
    }pub.publish(zeroV);

    turn(goal);
}

void getMap(const nav_msgs::OccupancyGrid::ConstPtr& M){
    G.resolution = M->info.resolution;
    G.X = M->info.width;
    G.Y = M->info.height;
    G.originx = M->info.origin.position.x;
    G.originy = M->info.origin.position.y;
    std::vector<float> map;
    for (int t = 0; t < M->data.size(); t++){
        map.push_back(M->data[t]);
    }

    G.map = std::vector<float>(map.size(), 0.0);
    for (int t = 0; t < map.size(); t++){
        if (map[t] == 100){
            G.map[t] = 100;
            std::vector<int> no = G.outline(t, 2);
            for (int r = 0; r < no.size(); r++){
                G.map[no[r]] = 100;
            }
        }
    }G.ready = true;
}

void getBot(const tf2_msgs::TFMessage::ConstPtr &T){
    if (T->transforms[0].header.frame_id == "odom"){
        geometry_msgs::TransformStamped wrtOdom = T->transforms[0];
        tf2::doTransform(zeroP, bot, wrtOdom);
        if (G.ready){
            botX = (bot.position.x - G.originx)/G.resolution;
            botY = (bot.position.y - G.originy)/G.resolution;
        }fready = true;
    }
}



int main(int c, char** v){

    ros::init(c, v, "lychee");
    ros::NodeHandle node;

    PATH.header.frame_id = "odom";
    PATH.header.seq = 0;

    zeroV.linear.x = 0;
    zeroV.linear.y = 0;
    zeroV.linear.z = 0;

    zeroV.angular.x = 0;
    zeroV.angular.y = 0;
    zeroV.angular.z = 0;

    zeroP.position.x = 0;
    zeroP.position.y = 0;
    zeroP.position.z = 0;

    zeroP.orientation.w = 1;
    zeroP.orientation.x = 0;
    zeroP.orientation.y = 0;
    zeroP.orientation.z = 0;

    vel.linear.x = 0.2;
    vel.linear.y = 0;
    vel.linear.z = 0;

    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;

    ros::Subscriber mapSub = node.subscribe("map", 100, getMap);
    ros::Subscriber botSub = node.subscribe("tf", 100, getBot);
    ros::Subscriber goalSub = node.subscribe("move_base_simple/goal", 100, getGoal);

    pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    pathpub = node.advertise<nav_msgs::Path>("/path", 100);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    while (ros::ok()){
        if (!fready){
            continue;
        }
    }

    return 0;
}
