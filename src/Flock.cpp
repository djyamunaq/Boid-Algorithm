#include "Flock.h"

Flock::Flock() {}

Flock::Flock(int nBoids) {
    this->nBoids = nBoids;

    for(int i=0; i<nBoids; i++) {
        double kP = 1e3;
        double kV = 1e1;
        
        Eigen::Vector2d pos = {kP*((((double)rand())/RAND_MAX) - 0.5), kP*((((double)rand())/RAND_MAX) - 0.5)};
        Eigen::Vector2d vel = {kV*((((double)rand())/RAND_MAX) - 0.5), kV*((((double)rand())/RAND_MAX) - 0.5)};
        this->boids.push_back(new Boid(pos, vel));
    }
}

Flock::~Flock() {}

void Flock::initializeQtree(Eigen::Vector2d pos, double width, double height, int maxEl) {
    this->qtree = Quadtree(pos, width, height, maxEl);
}

void Flock::run() {
    std::vector<Boid*>::iterator it = this->boids.begin();
    std::vector<Boid*>::iterator it2;

    this->qtree.clear();
    for(; it!=this->boids.end(); it++) {
        this->qtree.insert(*(*it));
    }

    for(it=this->boids.begin(); it!=this->boids.end(); it++) {
        Boid &boid = *(*it);
        Eigen::Vector2d avgDir = Eigen::Vector2d::Zero();
        Eigen::Vector2d avgPos = Eigen::Vector2d::Zero();
        Eigen::Vector2d avgSepVel = Eigen::Vector2d::Zero();
        int total = 0;

        std::vector<Boid> neighbors = this->qtree.retrieve(boid.getPos());
        std::vector<Boid>::iterator it2 = neighbors.begin();

        for(it2=neighbors.begin(); it2!=neighbors.end(); it2++) {
            Boid other = (*it2);

            double d = (boid.getPos() - other.getPos()).norm();
            if(d < this->allignRadius) {
                Eigen::Vector2d v = boid.getPos() - other.getPos();
                
                if(d != 0) {
                    avgSepVel += (this->sepFactor*v/pow(d, 2)); 
                }
                avgDir += other.getVel();
                avgPos += other.getPos();
                total++;
            }
        }
        /* Disconsider itself */
        total--;
        if(total > 0) {
            /* Separation */
            avgSepVel /= total;
            /* Coherence */
            avgPos -= boid.getPos();
            avgPos /= total;
            /* Allignment */
            avgDir -= boid.getVel();
            avgDir /= total;
            avgDir = avgDir/avgDir.norm();

            Eigen::Vector2d dVel = avgPos - boid.getPos();  
            Eigen::Vector2d steer0 = dVel  - boid.getVel();  
            Eigen::Vector2d steer1 = 10*avgDir - boid.getVel();  
            Eigen::Vector2d steer2 = avgSepVel - boid.getVel();

            Eigen::Vector2d steer = steer0 + steer1 + steer2;
            steer *= steerFactor;

            boid.setAcc(steer);
        }

    }
    for(it=this->boids.begin(); it!=this->boids.end(); it++) {
        (*it)->update();
    }
}

std::vector<Boid*> Flock::getBoids() {
    return this->boids;
}