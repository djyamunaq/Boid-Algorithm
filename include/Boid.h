#include <eigen3/Eigen/Dense>
#include <cmath>
#include <vector>
#include <ctime>
#include <chrono>
#include <sys/time.h>
#include <iostream>

class Boid {
    private:
        Eigen::Vector2d pos{Eigen::Vector2d::Zero()};
        Eigen::Vector2d vel{Eigen::Vector2d::Zero()};
        Eigen::Vector2d acc{Eigen::Vector2d::Zero()};
        double minVel{1e-1};
    public:
        Boid();
        Boid(Eigen::Vector2d pos, Eigen::Vector2d vel);
        ~Boid();
        void setPos(Eigen::Vector2d pos);
        Eigen::Vector2d getPos();
        void setVel(Eigen::Vector2d vel);
        Eigen::Vector2d getVel() ;
        void setAcc(Eigen::Vector2d acc);
        Eigen::Vector2d getAcc();
        void update();
};