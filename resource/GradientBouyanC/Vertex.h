#ifndef __VERTEX__H
#define __VERTEX__H

#include <vector>
#include <Eigen/Dense>

using Eigen::Vector3d;
using Eigen::Matrix3d;

class Vertex{

    private:
        //if the absolute vectors have been calculated -- allows intersection calculation to run
        bool absolute_vectors_calculated = false;

    public:

        Vector3d location; // the location of the vertex
        Vector3d vector_from_center;
        Vector3d rotated_vector_from_center; 

        //connections to other vertices
        Vector3d c1_relative;
        Vector3d c2_relative;
        Vector3d c3_relative;

        Vector3d c1_absolute;
        Vector3d c2_absolute;
        Vector3d c3_absolute;

        //connections to intersections
        std::vector<Vector3d> intersections;

        //Constructor and Destructor
        Vertex(Vector3d l, Vector3d c1, Vector3d c2, Vector3d c3);
        ~Vertex();

        //for testing things
        void test();

        //reflect vertex across the water plane
        void reflectAcrossWaterPlane();

        //calculate the absolute vectors between adjoining vertices
        void calculateAbsoluteVectors(Matrix3d rotation_matrix);

        //return the intersection points that are in range of the connectin segments
        std::vector<Vector3d> returnAdjacentIntersections();
};


#endif