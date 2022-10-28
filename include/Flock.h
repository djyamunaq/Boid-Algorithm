#include <eigen3/Eigen/Dense>
#include <cmath>
#include <vector>
#include <ctime>
#include <chrono>
#include <sys/time.h>
#include <iostream>
#include "Quadtree.h"

class Flock {
    private:
        int nBoids{0};
        std::vector<Boid*> boids;
        int allignRadius{100};
        double steerFactor{1e-2};
        double sepFactor{1e3};
        Quadtree qtree;
    public:
        Flock();
        Flock(int nBoids);
        ~Flock();
        void initializeQtree(Eigen::Vector2d pos, double width, double height, int maxEl);
        void run();
        std::vector<Boid*> getBoids();
};