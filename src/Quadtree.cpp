#include "Quadtree.h"

Quadtree::Quadtree() : nw(nullptr), ne(nullptr), sw(nullptr), se(nullptr) {}

Quadtree::Quadtree(Eigen::Vector2d pos, double width, double height, int maxEl) : nw(nullptr), ne(nullptr), sw(nullptr), se(nullptr) {
    this->pos = pos;
    this->width = width;
    this->height = height;
    this->maxEl = maxEl;
    this->boids = std::vector<Boid>();
}

Quadtree::~Quadtree() {
    if(this->divided) {
        nw->~Quadtree();
        ne->~Quadtree();
        sw->~Quadtree();
        se->~Quadtree();
    }
}

std::vector<Boid> Quadtree::retrieve(Eigen::Vector2d pos) {
    if(!this->divided && this->contains(pos)) return this->boids;
    if(!this->divided) return std::vector<Boid>();

    std::vector<Boid> v = this->nw->retrieve(pos);
    if(!v.empty()) return v;

    v = this->ne->retrieve(pos);
    if(!v.empty()) return v;

    v = this->sw->retrieve(pos);
    if(!v.empty()) return v;

    v = this->se->retrieve(pos);
    if(!v.empty()) return v;

    return std::vector<Boid>();
}

void Quadtree::insert(Boid boid) {
    if(!this->divided && this->boids.size() <= this->maxEl) {
        if(this->contains(boid.getPos())) {
            this->boids.push_back(boid);
        }
    } else {
        if(!this->divided) {
            divide();
        }
        this->nw->insert(boid);
        this->ne->insert(boid);
        this->sw->insert(boid);
        this->se->insert(boid);
    }
}

void Quadtree::clear() {
    this->boids = std::vector<Boid>();
    this->divided = false;

    if(this->divided) {
        this->nw->~Quadtree();
        this->ne->~Quadtree();
        this->sw->~Quadtree();
        this->se->~Quadtree();
    }
}

bool Quadtree::contains(Eigen::Vector2d pos) {
    return (pos(0) < (this->pos(0) + this->width/2) &&  pos(0) > (this->pos(0) - this->width/2) && pos(1) < (this->pos(1) + this->height/2) && pos(1) > (this->pos(1) - this->height/2));
}

void Quadtree::divide() {
    Eigen::Vector2d delta = Eigen::Vector2d(-this->width/4, this->height/4);
    nw = new Quadtree(this->pos+delta, this->width/2, this->height/2, this->maxEl);

    delta = Eigen::Vector2d(this->width/4, this->height/4);
    ne = new Quadtree(this->pos+delta, this->width/2, this->height/2, this->maxEl);

    delta = Eigen::Vector2d(-this->width/4, -this->height/4);
    sw = new Quadtree(this->pos+delta, this->width/2, this->height/2, this->maxEl);

    delta = Eigen::Vector2d(this->width/4, -this->height/4);
    se = new Quadtree(this->pos+delta, this->width/2, this->height/2, this->maxEl);

    for(std::vector<Boid>::iterator it=this->boids.begin(); it!=this->boids.end(); it++) {
        this->nw->insert(*it);
        this->ne->insert(*it);
        this->sw->insert(*it);
        this->se->insert(*it);
    }
}