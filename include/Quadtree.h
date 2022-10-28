#include <eigen3/Eigen/Dense>
#include <cmath>
#include <vector>
#include <ctime>
#include <chrono>
#include <sys/time.h>
#include <iostream>
#include "Boid.h"

class Quadtree {
    private:
        Eigen::Vector2d pos{Eigen::Vector2d::Zero()};
        std::vector<Boid> boids;
        Quadtree *nw; 
        Quadtree *ne; 
        Quadtree *sw; 
        Quadtree *se; 
        double width{0};
        double height{0};
        int maxEl{0};
        bool divided{false};
    public:
        Quadtree();
        Quadtree(Eigen::Vector2d pos, double width, double height, int maxEl);
        ~Quadtree();
        std::vector<Boid> retrieve(Eigen::Vector2d pos);
        void insert(Boid boid);
        void clear();
        bool contains(Eigen::Vector2d pos);
        void divide();
};