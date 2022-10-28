#include "Boid.h"

Boid::Boid() {}
Boid::Boid(Eigen::Vector2d pos, Eigen::Vector2d vel) {
    this->pos = pos;
    this->vel = vel;
}
Boid::~Boid() {}

void Boid::setPos(Eigen::Vector2d pos) {
    this->pos = pos;
}

Eigen::Vector2d Boid::getPos() {
    return this->pos;
}

void Boid::setVel(Eigen::Vector2d vel) {
    this->vel = vel;
}

Eigen::Vector2d Boid::getVel() {
    return this->vel;
}

void Boid::setAcc(Eigen::Vector2d acc) {
    this->acc = acc;
}

Eigen::Vector2d Boid::getAcc() {
    return this->acc;
}

void Boid::update() {
    double k0 = 1e0;
    this->vel += k0*this->acc;
    this->pos += k0*this->vel;

    // if(abs(this->vel(0)) < this->minVel) this->vel(0) = this->minVel; 
    // if(abs(this->vel(1)) < this->minVel) this->vel(1) = this->minVel; 
}