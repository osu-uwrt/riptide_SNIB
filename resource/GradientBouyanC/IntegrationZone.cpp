#include "IntegrationZone.h"
#include "LineSegment.h"

#include <iostream>
#include <vector>
#include <bits/stdc++.h>

using namespace std;

using Eigen::Vector3d;
using Eigen::Vector2d;

IntegrationZone::IntegrationZone(vector<Vector3d> upper, vector<Vector3d> lower, vector<double>plane, double bouyantDensity){
    this->plane_coefficents = plane;
    this->bouyant_density = bouyantDensity;
    
    //the additional point that are needed to create vertical chops
    vector<Vector3d> lower_additional;
    vector<Vector3d> upper_additional;

    
    for(int counter = 0; counter < size(upper); counter++){
        //add a corrosponding point in lower for every point in upper
        Vector3d point = upper.at(counter);
        
        // find where to add and add ensure its not a point in lower
        int inner_counter = 0 ;
        while(inner_counter < size(lower) - 1){
            LineSegment segment = LineSegment(lower.at(inner_counter), lower.at(inner_counter + 1));
            inner_counter += 1;

            if(segment.isInXBound(point[0])){
                Vector3d additional_point;
                segment.returnInterpolatedPoint(point[0], &additional_point);

                lower_additional.push_back(additional_point);
            }
        }
    }
    
    for (int counter = 0; counter < size(lower); counter++){
        //add a corrosponding point in lower for every point in upper
        Vector3d point = lower.at(counter);

        // find where to add and add ensure its not a point in lower
        int inner_counter = 0;
        while (inner_counter < size(upper) - 1){
            LineSegment segment = LineSegment(upper[inner_counter], upper[inner_counter + 1]);
            inner_counter += 1;

            if(segment.isInXBound(point[0])){
                Vector3d additional_point;
                segment.returnInterpolatedPoint(point[0], &additional_point);

                upper_additional.push_back(additional_point);
            }
        }
    }



    this->upper_bound.insert(this->upper_bound.end(), upper.begin(), upper.end());
    this->upper_bound.insert(this->upper_bound.end(), upper_additional.begin(), upper_additional.end());
    this->lower_bound.insert(this->lower_bound.end(), lower.begin(), lower.end());
    this->lower_bound.insert(this->lower_bound.end(), lower_additional.begin(), lower_additional.end());

    auto lambda = [](Vector3d v1, Vector3d v2) -> bool
    {
        //if this returns true v1 will be put in front of v2
        //if this returns false v2 will be put in front of v1
        
        return (v1[0] < v2[0]);
    };

    sort(this->upper_bound.begin(), this->upper_bound.end(), lambda);
    sort(this->lower_bound.begin(), this->lower_bound.end(), lambda);

    this->integrate();
    
}

double IntegrationZone::integrate_eq(double Mymax, double Mymin, double Bymax, double Bymin, double Xmax, double Xmin, double k, double Cx, double Cy, double Cz){
    //plane defined by : k = Cx * X + Cy * Y + Cz * Z
    //Xmin is the lower X bound of integration
    //Xmax is the upper X bound of integration
    //Mymin is the slope of the lower Y bound of integration
    //Mymax is the slope of the uppwer Y bound of integration
    //Bymin is the intercept of the lower Y bound of integration
    //Bymax is the intercept of the upper Y bound of integration

    double deltaYMax = ((Mymax - Mymin) * Xmax * Xmax / 2 + (Bymax - Bymin) * Xmax);
    double deltaYMin = ((Mymax - Mymin) * Xmin * Xmin / 2 + (Bymax - Bymin) * Xmin);
    double deltaYSquaredMax = ((pow(Mymax, 2) - pow(Mymin,2)) * pow(Xmax, 3) / 3 + ((Mymax * Bymax) - (Mymin * Bymin)) * pow(Xmax, 2) + (pow(Bymax,2) - pow(Bymin,2)) * Xmax);
    double deltaYSquaredMin = ((pow(Mymax, 2) - pow(Mymin,2)) * pow(Xmin, 3) / 3 + ((Mymax * Bymax) - (Mymin * Bymin)) * pow(Xmin, 2) + (pow(Bymax,2) - pow(Bymin,2)) * Xmin);
        
    double p1 = (deltaYSquaredMax * Cy / -2) - (deltaYSquaredMin * Cy / -2);
    double p2 = (deltaYMax * k) - (deltaYMin * k);
    double p3 = (-Cx * ((Mymax - Mymin) * pow(Xmax,3) / 3 + (Bymax - Bymin) * pow(Xmax, 2) / 2)) - (-Cx * ((Mymax - Mymin) * pow(Xmin,3) / 3+ (Bymax - Bymin) * pow(Xmin, 2) / 2));

    return ((p1 + p2 + p3) / Cz);
}

double IntegrationZone::calculateYCentroid(double c, double h, double d, double j, double a, double b, double k, double l, double m, double o, double volume){
    //Mymax = c
    //Mymin = h
    //Bymax = d
    //Bymin = j
    //Xmax = a
    //Xmin = b
    //k = k
    //Cx = l
    //Cy = m
    //Cz = o
    // everything was assigned differently in mathmatica 

    // if c or h is zero -- very possible -- there is a divide by zero error
    if(c == 0){
        double p1 = pow(a, 4) * pow(h, 2) * (3 * l + 2 * h * m);
        double p2 = 4 * pow(a, 3) * h * (-h * k + 2 * j * l + 2 * h * j * m);
        double p3 = 4 * a * (-3 * pow(d, 2) * k + 2 * pow(d, 3) * m + pow(j, 2) * (3 * k - 2 * j * m));
        double p4 = 6 * pow(a, 2) * ((pow(j, 2) - pow(d, 2)) * l + 2 * h * j * (j * m - k));
        double p6 = 6 * pow(d, 2) * (2 * k - b * l);
        double p7 = 8 * pow(d, 3) * m;
        double p8 = pow(b, 3) * pow(h, 2) * (3 * l + 2 * h * m);
        double p9 = 4 * pow(j, 2) * (-3 * k + 2 * j * m);
        double p10 = 6 * b * j * (j * l - 2 * h * k + 2 * h * j * m);
        double p11 = 4 * pow(b, 2) * h * (2 * j * l - h * k + 2 * h * j * m);

        return (p1 + p2 - p3  + p4 - b * (p6 - p7 + p8 + p9 + p10 + p11)) / (24 * o * volume);
    } else if (h == 0){
        double p1 = (4 * (pow((a * c + d), 3) - pow((b * c + d), 3)) * k) / c;
        double p2 = 12 * b * pow(j, 2) * k - 12 * a * pow(j, 2) * k - 3 * pow(a, 4) * pow(c, 2) * l + 3 * pow(b, 4) * pow(c, 2) * l;
        double p3 = 8 * pow(b, 3 )* c * d * l - 8 * pow(a, 3) * c * d * l - 6 * pow(a, 2) * pow(d, 2) * l + 6 * pow(b, 2) * pow(d, 2) * l;
        double p4 = 6 * pow(a, 2) * pow(j, 2) * l - 6 * pow(b, 2) * pow(j, 2) * l + 8 * a * pow(j, 3) * m - 8 * b * pow(j, 3) * m;
        double p5 = 2 * m * (pow((a * c + d), 4) - pow((b * c + d), 4)) / c;

        return (p1 + p2 + p3 + p4 - p5) / (24 * o * volume);
    }

    double p1 = (4 * (pow((a * c + d), 3) - pow((b * c + d), 3)) * k) / c;
    double p2 = 4 * k * (pow((a * h + j), 3) - pow((b * h +j), 3)) / h;
    double p3 = -3 * pow(a, 4) * pow(c, 2) * l + 3 * pow(b, 4) * pow(c, 2) * l - 8 * pow(a, 3) * c * d * l + 8 * pow(b, 3) * c * d * l;
    double p4 = -6 * pow(a, 2) * pow(d, 2) * l + 6 * pow(b, 2) * pow(d, 2) * l + 3 * pow(a, 4) * pow(h, 2) * l - 3 * pow(b, 4) * pow(h, 2) * l;
    double p5 = 8 * pow(a, 3) * h * j * l - 8 * pow(b, 3) * h * j * l + 6 * pow(a, 2) * pow(j, 2) * l - 6 * pow(b, 2) * pow(j, 2) * l;
    double p6 = 2 * m * (pow((a * c + d), 4) - pow((b * c + d), 4)) / c;
    double p7 = 2 * m * (pow((a * h + j), 4) - pow((b * h + j), 4)) / h;
    
    return (p1 - p2 + p3 + p4 + p5 - p6 + p7) / (24 * o * volume);
}

double IntegrationZone::calculateXCentroid(double c, double h, double d, double j, double a, double b, double k, double l, double m, double o, double volume){
    // Mymax = c
    // Mymin = h
    // Bymax = d
    // Bymin = j
    // Xmax = a
    // Xmin = b
    // k = k
    // Cx = l
    // Cy = m
    // Cz = o
    // everything was assigned differently in mathmatica 

    double p1 = 12 * j * k - 12 * d * k + 6 * pow(d, 2) * m - 6 * pow(j, 2) * m;
    double p2 = 8 * a * d * (l + c * m) - 8 * a * j * (l + h * m);
    double p3 = a * (c - h) * (6 * a * l - 8 * k + 3 * a * m * (c + h));
    double p5 = 8 * b * d * (l + c * m) - 8 * b * j * (l + h * m);
    double p6 =  b * (c - h) * (6 * b * l - 8 * k + 3 * b * m * (c + h));

    return (pow(a, 2) *(p1 + p2 + p3) - pow(b, 2) * (p1 + p5 + p6)) / (-24 * o * volume);
}

Vector3d IntegrationZone::calculateCentroid(double Mymax, double Mymin, double Bymax, double Bymin, double Xmax, double Xmin, double k, double Cx, double Cy, double Cz, double volume){
    double x = calculateXCentroid(Mymax, Mymin, Bymax, Bymin, Xmax, Xmin, k, Cx, Cy, Cz, volume);
    double y = calculateYCentroid(Mymax, Mymin, Bymax, Bymin, Xmax, Xmin, k, Cx, Cy, Cz, volume);
    double z = 0.0;
    //z doesn't matter in this case -- to save work it isn't actually calculated
    //if, for some reason, it is needed, look in the python version

    return Vector3d(x,y,z);
}

Vector2d IntegrationZone::pointsToMxPlusB(Vector3d p1, Vector3d p2){
    //[m,b] - m is slope, b is y intercept
    double m = (p1[1] - p2[1]) / (p1[0] - p2[0]);
    double b = p1[1] - (p1[0] * m);

    return Vector2d(m, b);
}

void IntegrationZone::invertForceDirection(){
    this->total_force = -this->total_force;
}


void IntegrationZone::integrate(){
    // integrate the based on upper and lower bounds and plane

    // there should be one less zone than points in upper bound
    // lower bound should have the same number of points as upper bound
    int counter = 0;
    //need to skip over vertical segments
    int lower_vertical = 0;
    int upper_vertical = 0;
    while((counter + upper_vertical < (size(this->upper_bound) - 1)) && (counter + lower_vertical < (size(this->lower_bound) - 1))){
        //if the upper segment is vertical; skip
        while(abs(this->upper_bound[upper_vertical + counter][0] - this->upper_bound[upper_vertical + 1 + counter][0]) < 2.7e-10){
            //if the width of the zone is small that that of a water molecule, its essentially vertical (this is done for a reason)
            upper_vertical += 1;
            
            // in case all segments are vertical
            if(counter + upper_vertical >= (size(this->upper_bound) - 1)){
                break;
            }
        }
        //if the lower segment is vertical; skip
        while(abs(this->lower_bound[lower_vertical + counter][0] - this->lower_bound[lower_vertical + counter + 1][0]) < 2.7e-10){
            //if the width of the zone is small that that of a water molecule, its essentially vertical (this is done for a reason)
            lower_vertical += 1;
            
            // in case all segments are vertical
            if(counter + lower_vertical >= (size(this->lower_bound) - 1)){
                break;
            }
        }

        // in case all segments are vertical
        if(counter + upper_vertical >= (size(this->upper_bound) - 1)){
            break;
        }

        if(counter + lower_vertical >= (size(this->lower_bound) - 1)){
            break;
        }

        Vector2d high_line = this->pointsToMxPlusB(this->upper_bound[counter + upper_vertical], this->upper_bound[counter + upper_vertical + 1]);
        Vector2d low_line = this->pointsToMxPlusB(this->lower_bound[counter + lower_vertical], this->lower_bound[counter + lower_vertical + 1]);

        double volume = this->integrate_eq(high_line[0], low_line[0], high_line[1], low_line[1], 
            this->upper_bound[counter + upper_vertical + 1][0], this->upper_bound[counter + upper_vertical][0], 
            this->plane_coefficents[3], this->plane_coefficents[0], this->plane_coefficents[1], this->plane_coefficents[2]
            );
        this->total_volume += volume;
        Vector3d centroid = this->calculateCentroid(high_line[0], low_line[0], high_line[1], low_line[1], 
            this->upper_bound[counter + upper_vertical + 1][0], this->upper_bound[counter + upper_vertical][0], 
            this->plane_coefficents[3], this->plane_coefficents[0], this->plane_coefficents[1], this->plane_coefficents[2],
            volume);

        double bouyant_force = volume * bouyant_density;

        vector<Vector3d> centroids_to_sum = {this->bouyant_force_acting_location, centroid};
        vector<double> forces_to_sum = {this->total_force, bouyant_force};
        this->sumBouyantForces(forces_to_sum, centroids_to_sum, &this->total_force, &this->bouyant_force_acting_location);

        counter++;
    }

}

void IntegrationZone::sumBouyantForces(vector<double> forces, vector<Vector3d> centroids, double *sum_forces, Vector3d *overall_centroid){
    //  z position of bouyant forces dont matter
    //  Returns a scalar value for a for in the z direction and a acting location in the form of [x, y, z]
    //  forces: [] or scalar bouyant forces
    //  positions: [] of positions where forces act - assumed to be in same order as forces and in [x, y, z] format

    // find the total bouyant force
    double total_force = 0;
    for(int counter = 0; counter < size(forces); counter++){
        total_force += forces[counter];
    }

    //this shouldn't ever happen but just in case
    if(total_force == 0){
        *sum_forces = 0;
        *overall_centroid = Vector3d(0,0,0);

        return;
    }

   // find the total torque around the x-axis
    double x_torque = 0;
    for(int counter = 0; counter < size(centroids); counter++){
        x_torque += centroids[counter][1] * forces[counter];
    }

    //find the total torque around the y-axis
    double y_torque = 0;
    for(int counter = 0; counter < size(centroids); counter++){
        y_torque += centroids[counter][0] * forces[counter]; 
    }

    *sum_forces = total_force;

    Vector3d finalSum(y_torque / total_force, x_torque / total_force, 0);

    *overall_centroid = Vector3d(y_torque / total_force, x_torque / total_force, 0);

    return;
}





