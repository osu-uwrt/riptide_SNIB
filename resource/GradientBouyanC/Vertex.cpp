#include "Vertex.h"
#include <stdlib.h>
#include <iostream>
#include <vector>

using namespace std;

using Eigen::Vector3d;
using Eigen::Matrix3d;

Vertex::Vertex(Vector3d  l, Vector3d  c1, Vector3d  c2, Vector3d  c3){
    this->vector_from_center = l;
    this->c1_relative = c1;
    this->c2_relative = c2;
    this->c3_relative = c3;
}

Vertex::~Vertex(){

}

void Vertex::test(){
    cout << this->c2_absolute;
}

void Vertex::reflectAcrossWaterPlane(){
    // make the z coordinate into is opposite
    this->location[2] = -this->location[2];

    //all the relative vectors will now be have opposite z components so adjust for that too
    this->c1_relative[2] = -this->c1_relative[2];
    this->c2_relative[2] = -this->c2_relative[2];
    this->c3_relative[2] = -this->c3_relative[2];
}

void Vertex::calculateAbsoluteVectors(Matrix3d rotation_matrix){
        //apply transform to connecting vectors

        //rotate
        this->c1_absolute = rotation_matrix * this->c1_relative;
        this->c2_absolute = rotation_matrix * this->c2_relative;
        this->c3_absolute = rotation_matrix * this->c3_relative;

        this->absolute_vectors_calculated = true;
}

vector<Vector3d> Vertex::returnAdjacentIntersections(){

    vector<Vector3d> intersections;

    if(this->absolute_vectors_calculated == false){
        return intersections;
    }

    //find the points of intersection with the water plane adjacent to this vertex

    //if starting location is underwater
    if (this->location[2] < 0){
        
        //if vector will interest with water plane 
        if (this->c1_absolute[2] + this->location[2] > 0){

            //porportion of vector needed to reach water plane
            double l = -this->location[2] / this->c1_absolute[2];

            // calculate the point of intersection
            intersections.push_back(this->c1_absolute * l + this->location);
        }

        //if vector will interests with water plane 
        if((this->c2_absolute[2] + this->location[2]) > 0){

            //porportion of vector needed to reach water plane
            double l = -this->location[2] / this->c2_absolute[2];

            // calculate the point of intersection
            intersections.push_back(this->c2_absolute * l + this->location);
        }

        //if vector will interests with water plane 
        if ((this->c3_absolute[2] + this->location[2]) > 0){

            //porportion of vector needed to reach water plane
            double l = -this->location[2] / this->c3_absolute[2];

            // calculate the point of intersection
            intersections.push_back(this->c3_absolute * l + this->location);
        }
    }
    else{

        //if vector will interest with water plane 
        if((this->c1_absolute[2] + this->location[2]) < 0){

            //porportion of vector needed to reach water plane
            double l = -this->location[2] / this->c1_absolute[2];

            // calculate the point of intersection
            intersections.push_back(this->c1_absolute * l + this->location);
        }

        //if vector will interests with water plane 
        if((this->c2_absolute[2] + this->location[2]) < 0){

            //porportion of vector needed to reach water plane
            double l = -this->location[2] / this->c2_absolute[2];

            // calculate the point of intersection
            intersections.push_back(this->c2_absolute * l + this->location);
        }

        //if vector will interests with water plane 
        if((this->c3_absolute[2] + this->location[2]) < 0){

            //porportion of vector needed to reach water plane
            double l = -this->location[2] / this->c3_absolute[2];

            // calculate the point of intersection
            intersections.push_back(this->c3_absolute * l + this->location); 
        }     
    }

    this->intersections = intersections;

    return intersections;
}


