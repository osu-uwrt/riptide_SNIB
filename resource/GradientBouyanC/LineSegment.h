#ifndef __LINE__SEGEMENT__H__
#define __LINE__SEGEMENT__H__

#include <Eigen/Dense>
#include "Vertex.h"

using Eigen::Vector3d;
using Eigen::Matrix3d;

class LineSegment{
    private:

    public:
        // the end points of a line segment as [x,y,z]
        Vector3d end_point_one;
        Vector3d end_point_two;

        bool includes_vertices = false;

        //the end points of the segment as vertices
        Vertex vertex_one;
        Vertex vertex_two;

        LineSegment(Vector3d p1, Vector3d p2);
        LineSegment(Vertex p1, Vertex p2);

        bool checkForDuplicate(LineSegment other);
        bool isInXBound(double xValue);
        bool returnInterpolatedPoint(double xValue, Vector3d *interpolated_point);


        //get the magnitude of the vector between the two end points of the segment
        double getLength();
        double getProjectedLength();
        double returnXMax();

 





};




#endif
