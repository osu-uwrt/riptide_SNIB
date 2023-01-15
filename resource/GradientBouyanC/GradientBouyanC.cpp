#include "Vertex.h"
#include "LineSegment.h"
#include "IntegrationZone.h"

#include <bits/stdc++.h>
#include <vector>
#include <iostream>

using namespace std;

using Eigen::Vector3d;
using Eigen::Matrix3d;

void calculateBoundingPaths(vector<Vector3d> points, vector<Vector3d> *upper_path, vector<Vector3d> *lower_path);
void sumBouyantForces(vector<double> forces, vector<Vector3d> centroids, double *sum_forces, Vector3d *overall_centroid);
void calculateBouyancy(vector<Vertex> raw_vertices, Vector3d euler_rotation, Vector3d position, double bouyant_density, double bouyant_force, double *calculated_force, Vector3d *acting_loccation);

bool isCoplanar(Vector3d point, vector<double> plane);
bool determineIfAdditiveArea(Vector3d izv1, Vector3d izv2, Vector3d izi1, Vector3d izi2, Vector3d izo1, Vector3d izo2);
bool determineAdditiveArea(vector<double> plane, Vector3d other_surfaced_vertice);

Matrix3d rotationFromEuler(double x, double y, double z);

vector<double> pointsToPlane(Vector3d p1, Vector3d p2, Vector3d p3);

vector<Vertex> findOtherVertices(vector<Vector3d> surfaced_vertices, vector<Vector3d>vertices);
vector<Vertex> generateRawVertices(vector<double> bounding_box);

int main(){
    //Actually the start of the calculate bouyancy process
    //definition of size and location(relative to orgin) of bounding box
    vector<double> bounding_box = {.25, .26, .30, .35, .2, .1};
    vector<Vertex> raw_vertices = generateRawVertices(bounding_box);

    double bouyant_force = 100;
    double bouyant_density = bouyant_force / (bounding_box[0] + bounding_box[1]) / (bounding_box[2] + bounding_box[3]) / (bounding_box[4] + bounding_box[5]);

    Vector3d position = Vector3d(1,0,0);
    Vector3d orentation = Vector3d(3.14,0,0);

    double force;
    Vector3d location;
    calculateBouyancy(raw_vertices, orentation, position, bouyant_density, bouyant_force, &force, &location);

    cout << "Final Force: " << force << endl;
    cout << location[0] << "  " << location[1] << "  " << location[2] << endl;
}


void calculateBoundingPaths(vector<Vector3d> points, vector<Vector3d> *upper_bound, vector<Vector3d> *lower_bound){
    if(size(points) < 3){
        // thats not supposed to happen
        return;
    }

    auto lambda = [](Vector3d v1, Vector3d v2) -> bool
    {
        //if this returns true v1 will be put in front of v2
        //if this returns false v2 will be put in front of v1
        
        return (v1[0] < v2[0]);
    };

    // sort the points by x coordinate
    sort(points.begin(), points.end(), lambda);

    //initial point for both bounds is the same - so is the final
    lower_bound->push_back(points.at(0));
    upper_bound->push_back(points.at(0)); 

    //once the number of points in the two bounds is equal to the total number of points, add the final point 
    //initial point is double counted but the final point isn't
    int counter = 1;
    double midline_slope;
    if ((points.at(size(points) - 1)[0] - points.at(0)[0]) == 0){
        //if vertical
        midline_slope = 999999999;
    }
    else{
        midline_slope = (points.at(size(points) - 1)[1] - points.at(0)[1]) / (points.at(size(points) - 1)[0] - points.at(0)[0]);
    }

    while ((size(*lower_bound) + size(*upper_bound)) != size(points)){
        //points above midline go in upper bound below go in
        if(points.at(counter)[1] > points.at(0)[1] + midline_slope * (points.at(counter)[0] - points.at(0)[0])){
            upper_bound->push_back(points.at(counter));
        }
        else{ 
            lower_bound->push_back(points.at(counter));
        }

        counter ++;
    }

    // finish out bound path
    upper_bound->push_back(points.at(size(points) - 1));
    lower_bound->push_back(points.at(size(points) - 1));

    return;
}

vector<double> pointsToPlane(Vector3d p1, Vector3d p2, Vector3d p3){
    // calcuates the coefficents for a plane equation from three points
    // Cy * Y + Cx * X + Cz * Z = k
    // inputs should be in [x, y, z]
    // returns [Cx, Cy, Cz, k]

    Vector3d v1 = p1 - p2;
    Vector3d v2 = p1 - p3;

    Vector3d cp = v1.cross(v2);
    double k = cp.dot(p1);

    vector<double> plane={cp[0], cp[1], cp[2], k};
    return plane;
}

bool isCoplanar(Vector3d point, vector<double> plane){
    //  plane should be in [Cx, Cy, Cz, k] form
    //  point should be in [x, y, z] form
    //  checks to see if a point is on a plane

    if(plane.at(3) == (plane.at(0) * point[0] + plane.at(1) * point[1] + plane.at(2) * point[2])){
        return true;
    }

    return false;
}

bool determineAdditiveArea(vector<double> plane, Vector3d other_surfaced_vertice){
    //determine the z coordinate on the plane for the x and y vertice of the other sufraced vertex
    double planarZ = (plane[3] - plane[0] * other_surfaced_vertice[0] - plane[1] * other_surfaced_vertice[1]) / plane[2];

    if(planarZ > other_surfaced_vertice[2]){
        return true;
    } else {
        return false;
    }
}

bool determineIfAdditiveArea(Vector3d izv1, Vector3d izv2, Vector3d izi1, Vector3d izi2, Vector3d izo1, Vector3d izo2){
    //Used to Determine if an integration zone is additive in the case where four vertices are being evaluated
    //  izv# represents a vertex that is bounding the zone
    //  izi# represents an intersection that is bound the zone
    //  izo# represents a vertext that is not bounding the zone

    //  returning true is additive area, returning false is negative area

    //line created by the instersection of the water plane and a vertical plane created with izv1 and izv2
    double slope = 999999999;

    if ((izv2[0] - izv1[0]) != 0){
       slope = (izv2[1] - izv1[1]) / (izv2[0] - izv1[0]);
    }
    else if (((izi1[0] - izi2[0]) == 0) and (izv1[0] == izi1[0])){
        // if the line is vertical
        //if the second line is vertical too and shares the same x coordinate, the volume of the iz will be zero 
        return true;
    }

    double y_intercept = izv1[1] - (slope * izv1[0]);

    //determine if the intersections are above the line
    if((izi1[1] > (izi1[0] * slope + y_intercept)) && (izi2[1] > (izi2[0] * slope + y_intercept))){
        //the intersection are above the line
        // if the vertices that are not bounding the box are above the line as well, the integration zone represents negative area - if not additive area
        if ((izo1[1] > (izo1[0] * slope + y_intercept)) && (izo2[1] > (izo2[0] * slope + y_intercept))){
            //negative area
            return false;
        }else if((izo1[1] < (izo1[0] * slope + y_intercept)) && (izo2[1] < (izo2[0] * slope + y_intercept))){
            //additive area
            return true;
        }else{
            cout << "Determine additive area: This shouldn't happen... 1" << endl;
        }
    }else if((izi1[1] < (izi1[0] * slope + y_intercept)) && (izi2[1] < (izi2[0] * slope + y_intercept))){
        // the intersection are below the line
        // if the vertices that are not bounding the box are below the line as well, the integration zone represents negative area - if not additive area

        if ((izo1[1] > (izo1[0] * slope + y_intercept)) && (izo2[1] > (izo2[0] * slope + y_intercept))){
            // additive area
            return true;
        }else if((izo1[1] < (izo1[0] * slope + y_intercept)) && (izo2[1] < (izo2[0] * slope + y_intercept))){
            //negative area
            return false;
        }else{
            cout << "Determine additive area: This shouldn't happen... 2" << endl;
        }
    }else if ((izi1[1] < (izi1[0] * slope + y_intercept)) && (izi2[1] < (izi2[0] * slope + y_intercept))){
        //if both points are on the line, it doesn't matter, the area will be zero
        return true;
    }else{
        cout << "Determine additive area: This shouldn't happen... 4" << endl;
    }

    //casually return false - this should never happen
    return false;
}

vector<Vertex> findOtherVertices(vector<Vertex> surfaced_vertices, vector<Vertex>vertices){
    //used in conjunction with determine if additive area
    //finds the vertices not bounding the plane
    vector<Vector3d> surfaced_vertices_locations;
    vector<Vector3d> vertices_locations;

    for(int counter = 0; counter < size(surfaced_vertices); counter++){
        surfaced_vertices_locations.push_back(surfaced_vertices[counter].location);
    }

    for(int counter = 0; counter < size(vertices); counter++){
        vertices_locations.push_back(vertices[counter].location);
    }

    vector<Vertex> other_vertices;
    for(int counter = 0; counter < size(surfaced_vertices_locations); counter++){
        Vector3d vertex = surfaced_vertices_locations.at(counter);
        
        bool vertex_found = false;
        for(int inner_counter = 0; inner_counter < size(vertices_locations); inner_counter++){
            if(vertex == vertices_locations.at(inner_counter)){
                vertex_found = true;
            }
        }

        // if the vertex is not in the list add it to the other list
        if(vertex_found == false){
            other_vertices.push_back(surfaced_vertices[counter]);
        }
    }
    
    return other_vertices;
}

void sumBouyantForces(vector<double> forces, vector<Vector3d> centroids, double *sum_forces, Vector3d *overall_centroid){
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

void calculateBouyancy(vector<Vertex> raw_vertices, Vector3d euler_rotation, Vector3d position, double bouyant_density, double bouyant_force, double *calculated_force, Vector3d *acting_location){
    //calculate roation matrix
    Matrix3d rotation_matrix = rotationFromEuler(euler_rotation[0], euler_rotation[1], euler_rotation[2]);

    //calculate the box cov - different from positon
    //  the relative vector from the position passed to the cov of the box
    Vector3d relative_postion_cob_vector(
        (raw_vertices[0].vector_from_center[0] + raw_vertices[7].vector_from_center[0]) / 2,
        (raw_vertices[0].vector_from_center[1] + raw_vertices[7].vector_from_center[1]) / 2,
        (raw_vertices[0].vector_from_center[2] + raw_vertices[7].vector_from_center[2]) / 2);

    Vector3d absolute_postion_cob_vector = rotation_matrix * relative_postion_cob_vector;
    Vector3d cov = absolute_postion_cob_vector + position;

    //calculate rotated vertices and adjust vertex to account for position
    for(int counter = 0; counter < size(raw_vertices); counter++){
        raw_vertices.at(counter).rotated_vector_from_center = rotation_matrix * raw_vertices.at(counter).vector_from_center;
        cout << rotation_matrix(0,0) <<  " " << rotation_matrix(0,1) << "" <<  rotation_matrix(0,2) << endl;

        raw_vertices.at(counter).location = raw_vertices.at(counter).rotated_vector_from_center + position;
    }

    for(int counter = 0; counter < size(raw_vertices); counter++){
        cout << "vertex x: " << raw_vertices[counter].location[0] << " vertex y: " << raw_vertices[counter].location[1] << " vertex z: " << raw_vertices[counter].location[2] << endl;
        cout << "vertex x: " << raw_vertices[counter].vector_from_center[0] << " vertex y: " << raw_vertices[counter].vector_from_center[1] << " vertex z: " << raw_vertices[counter].vector_from_center[2] << endl;
    }

    //if midpoint is above water plane, robot is not submerged; if below, robot is submerged
    //   evaluate space where midpoint (cov not position) is not; if below, evaluate above; vice versa
    bool submerged = true;
    if(cov[2] > 0){
        submerged = false;
    }

    if (submerged == false){
        // we need to evaluate volume above water and the simplest shapes possible
        // if the simpler shape is below the water, reflect the box across the water plane and then take that into account when calculating the final bouyancy and acting location
        for(int counter = 0; counter < size(raw_vertices); counter++){
            raw_vertices[counter].reflectAcrossWaterPlane();
        }
    }

    vector<Vertex> surfaced_vertices;
    for (int counter = 0; counter < size(raw_vertices); counter ++){
        Vertex vertex = raw_vertices.at(counter);

        // find all submerged vertices
        if(vertex.location[2] > 0){
            surfaced_vertices.push_back(vertex);
        }
    }

    if (size(surfaced_vertices) == 0){
        if(submerged){
            *calculated_force = bouyant_force;
            *acting_location = cov;
        }else{
            *calculated_force = 0;
            *acting_location = Vector3d(0,0,0);
        }

        // if no vertices are above water, bouyant force is trivial
        return;
    }

    //calculate where the edges of the bow intersect with the water plane
    vector<Vector3d> surface_intersections;
    for (int counter = 0; counter < size(surfaced_vertices); counter++){
        surfaced_vertices[counter].calculateAbsoluteVectors(rotation_matrix);
        
        vector<Vector3d> intersections = surfaced_vertices[counter].returnAdjacentIntersections();
        for (int inner_counter = 0; inner_counter < size(intersections); inner_counter++){
            surface_intersections.push_back(intersections.at(inner_counter));
        }
    }

    if( size(surface_intersections) < 3){
        *calculated_force = 0;
        *acting_location = Vector3d(0,0,0);

        return;
    }
    
    vector<Vector3d> lower_surface_path;
    vector<Vector3d> upper_surface_path;
    
    calculateBoundingPaths(surface_intersections, &upper_surface_path, &lower_surface_path);

    vector<LineSegment> surfaced_segments;
    if(size(surfaced_vertices) == 4){
        // there is a plane the is entirely above the surface of the water -- only possiblity is 4 in theory

        //determine all six possible line segments that are completely on top of the water
        vector<LineSegment> segments = {
            LineSegment(surfaced_vertices.at(0), surfaced_vertices.at(1)),
            LineSegment(surfaced_vertices.at(0), surfaced_vertices.at(2)),
            LineSegment(surfaced_vertices.at(0), surfaced_vertices.at(3)),
            LineSegment(surfaced_vertices.at(1), surfaced_vertices.at(2)),
            LineSegment(surfaced_vertices.at(1), surfaced_vertices.at(3)),
            LineSegment(surfaced_vertices.at(2), surfaced_vertices.at(3)),
        };

        auto evaluateLength = [](LineSegment s1, LineSegment s2) -> bool
        {
            return (s1.getLength() < s2.getLength());
        };



        for(int counter = 0; counter < size(segments); counter++){
            cout << segments[counter].getLength() << endl;
        }     
        
        for(int counter = 0; counter < size(surfaced_vertices); counter++){
            cout << " x: " << surfaced_vertices[counter].location[0] << " y: " << surfaced_vertices[counter].location[1] << " z: " << surfaced_vertices[counter].location[2] << endl;
        }        

        //eliminate longest two to leave outer bounds of rectangle only
        sort(segments.begin(), segments.end(), evaluateLength);
        surfaced_segments = {segments.at(0),segments.at(1),segments.at(2),segments.at(3)};

        // there will be five planes each bounded by four sides
        // one plane will be completely above the water, the rest will intersect it

        //above water plane
        vector<Vector3d> points = {
            Vector3d(surfaced_vertices.at(0).location[0], surfaced_vertices.at(0).location[1], 0),
            Vector3d(surfaced_vertices.at(1).location[0], surfaced_vertices.at(1).location[1], 0),
            Vector3d(surfaced_vertices.at(2).location[0], surfaced_vertices.at(2).location[1], 0),
            Vector3d(surfaced_vertices.at(3).location[0], surfaced_vertices.at(3).location[1], 0)
            };
        vector<Vector3d> u1, l1;

        calculateBoundingPaths(points, &u1, &l1);
        vector<double> p1 = pointsToPlane(surfaced_vertices.at(0).location, surfaced_vertices.at(1).location, surfaced_vertices.at(2).location);
        IntegrationZone iz1 = IntegrationZone(u1, l1, p1, bouyant_density);

        //if the intersections and surfaced vertices are vertical iz1 is the only non zero integration zone
        if(submerged && (surfaced_vertices.at(0).location[0] == surfaced_vertices.at(0).intersections[0][0]) and (surfaced_vertices.at(0).location[1] == surfaced_vertices.at(0).intersections[0][1])){
            // for a box, if one is vertical, they all are vertical
            vector<Vector3d> acting_locations = {cov, iz1.bouyant_force_acting_location};
            vector<double> acting_forces = {bouyant_force, -iz1.total_force};
            sumBouyantForces(acting_forces, acting_locations, calculated_force, acting_location);
                        cout << bouyant_force << endl;

            return;
        }
        else if((surfaced_vertices.at(0).location[0] == surfaced_vertices.at(0).intersections[0][0]) && (surfaced_vertices.at(0).location[1] == surfaced_vertices.at(0).intersections[0][1])){            
            *calculated_force = iz1.total_force;
            *acting_location = iz1.bouyant_force_acting_location;
            return;
        }
        
        // some of the zones in this case represent negative area above water - all but two - it is important to figure out which
        // this can be done by seeing of the intersections and the surfaced vertices not forming the bound of the integration zone are on the same side of a perfectly vertical plane formed by the two surfaced vertices bounding the zone

        vector<Vector3d> u2, l2, u3, l3, u4, l4, u5, l5;

        // side plane one
        vector<Vector3d> points2 = {
            Vector3d(surfaced_segments[0].vertex_one.location[0], surfaced_segments[0].vertex_one.location[1], 0), 
            Vector3d(surfaced_segments[0].vertex_two.location[0], surfaced_segments[0].vertex_two.location[1], 0),
            surfaced_segments[0].vertex_one.intersections[0],
            surfaced_segments[0].vertex_two.intersections[0]
        };
        calculateBoundingPaths(points2, &u2, &l2);
        vector<double> p2 = pointsToPlane(surfaced_segments[0].vertex_one.location, surfaced_segments[0].vertex_one.intersections[0], surfaced_segments[0].vertex_two.intersections[0]);
        IntegrationZone iz2 = IntegrationZone(u2, l2, p2, bouyant_density);

        vector<Vertex> surfaced_segment_vertices = {surfaced_segments[0].vertex_one, surfaced_segments[0].vertex_two};
        vector<Vertex> other_vertices = findOtherVertices(surfaced_vertices, surfaced_segment_vertices);
        if(!determineAdditiveArea(p2, other_vertices[0].location)){
            iz2.invertForceDirection();
        }


        // side plane two
        vector<Vector3d> points3 = {
            Vector3d(surfaced_segments[1].vertex_one.location[0], surfaced_segments[1].vertex_one.location[1], 0), 
            Vector3d(surfaced_segments[1].vertex_two.location[0], surfaced_segments[1].vertex_two.location[1], 0),
            surfaced_segments[1].vertex_one.intersections[0],
            surfaced_segments[1].vertex_two.intersections[0]     
        };
        calculateBoundingPaths(points3, &u3, &l3);
        vector<double> p3 = pointsToPlane(surfaced_segments[1].vertex_one.location, surfaced_segments[1].vertex_one.intersections[0], surfaced_segments[1].vertex_two.intersections[0]);
        IntegrationZone iz3 = IntegrationZone(u3, l3, p3, bouyant_density);

        surfaced_segment_vertices = {surfaced_segments[1].vertex_one, surfaced_segments[1].vertex_two};
        other_vertices = findOtherVertices(surfaced_vertices, surfaced_segment_vertices);
        if(!determineAdditiveArea(p3, other_vertices[0].location)){
            iz3.invertForceDirection();
        }

        // side plane three
        vector<Vector3d> points4 = {
            Vector3d(surfaced_segments[2].vertex_one.location[0], surfaced_segments[2].vertex_one.location[1], 0), 
            Vector3d(surfaced_segments[2].vertex_two.location[0], surfaced_segments[2].vertex_two.location[1], 0),
            surfaced_segments[2].vertex_one.intersections[0],
            surfaced_segments[2].vertex_two.intersections[0]                
        };
        calculateBoundingPaths(points4, &u4, &l4);
        vector<double> p4 = pointsToPlane(surfaced_segments[2].vertex_one.location, surfaced_segments[2].vertex_one.intersections[0], surfaced_segments[2].vertex_two.intersections[0]);
        IntegrationZone iz4 = IntegrationZone(u4, l4, p4, bouyant_density);

        surfaced_segment_vertices = {surfaced_segments[2].vertex_one, surfaced_segments[2].vertex_two};   
        other_vertices = findOtherVertices(surfaced_vertices, surfaced_segment_vertices);
        if(!determineAdditiveArea(p4, other_vertices[0].location)){
            iz4.invertForceDirection();
        }

        // side plane four
        vector<Vector3d> points5 = {
            Vector3d(surfaced_segments[3].vertex_one.location[0], surfaced_segments[3].vertex_one.location[1], 0), 
            Vector3d(surfaced_segments[3].vertex_two.location[0], surfaced_segments[3].vertex_two.location[1], 0),
            surfaced_segments[3].vertex_one.intersections[0],
            surfaced_segments[3].vertex_two.intersections[0]      
        };
        calculateBoundingPaths(points5, &u5, &l5);
        vector<double> p5 = pointsToPlane(surfaced_segments[3].vertex_one.location, surfaced_segments[3].vertex_one.intersections[0], surfaced_segments[3].vertex_two.intersections[0]);
        IntegrationZone iz5 = IntegrationZone(u5, l5, p5, bouyant_density);

        surfaced_segment_vertices = {surfaced_segments[3].vertex_one, surfaced_segments[3].vertex_two};
        other_vertices = findOtherVertices(surfaced_vertices, surfaced_segment_vertices);
        if(!determineAdditiveArea(p5, other_vertices[0].location)){
            iz5.invertForceDirection();
        }


        double iz_total_force;
        Vector3d iz_overall_acting_location;
        vector<double> iz_forces = {iz1.total_force, iz2.total_force, iz3.total_force, iz4.total_force, iz5.total_force};
        vector<Vector3d> iz_acting_locations = {iz1.bouyant_force_acting_location, iz2.bouyant_force_acting_location, iz3.bouyant_force_acting_location, iz4.bouyant_force_acting_location, iz5.bouyant_force_acting_location};
        sumBouyantForces(iz_forces, iz_acting_locations, &iz_total_force, &iz_overall_acting_location);

        if(submerged){
            // flip the delta bouyant force because it represents a lack of bouyancy in this senario
            iz_total_force = -iz_total_force;
            //calcuate the bouyant force of the entire box - bouyancy if entire box is submerged - delta
            vector<double> overall_forces = {bouyant_force, iz_total_force};
            vector<Vector3d> overall_acting_locations = {cov, iz_overall_acting_location};
            return sumBouyantForces(overall_forces, overall_acting_locations, calculated_force, acting_location);
        }else{
            *calculated_force = iz_total_force;
            *acting_location = iz_overall_acting_location;
            return;
        }

    }else if(size(surfaced_vertices) == 3){
        //This one is pretty simple .. 5 planes total -- one pentagon that is bounded by the surfaced vertices and two intersection points(that are coplanar with the surfaced vertices)
        // 2 quadralaterals and two triangles
        // the triangular planes are formed by the double_intersection vertices and their two intersections
        // the quadralateral planes are formed by the singular intersection vertex, its intersection, a double intersection vertex and one of it's intersections
        //The three quadralaterals a can be found by combining 2 surfaced vertice and intersection pairs together in all possible ways

        Vertex single_intersection_vertex = Vertex(Vector3d(0,0,0),Vector3d(0,0,0),Vector3d(0,0,0),Vector3d(0,0,0));
        vector<Vertex> double_intersection_vertices;
        for(int counter = 0; counter < size(surfaced_vertices); counter++){
            Vertex vertex = surfaced_vertices.at(counter);
            if(size(vertex.intersections) == 1){
                single_intersection_vertex = vertex;
            }else{
                double_intersection_vertices.push_back(vertex);
            }
        }
        //find vertices form double_intersection_vertices that are coplanar with the three surfaced vertices
        vector<double> surfaced_plane = pointsToPlane(surfaced_vertices.at(0).location, surfaced_vertices.at(1).location, surfaced_vertices.at(2).location);

        // the indexes in these group corrospond the indexes in double_intersection vertices
        vector<Vector3d> coplanar_intersections;
        vector<Vector3d> nonplanar_intersections;
        for(int counter = 0; counter < size(double_intersection_vertices); counter++){
            Vertex vertex = double_intersection_vertices.at(counter);

            for(int inner_counter = 0; inner_counter < size(vertex.intersections); inner_counter++){
                Vector3d intersection = vertex.intersections.at(inner_counter);

                if(isCoplanar(intersection, surfaced_plane)){
                    coplanar_intersections.push_back(intersection);
                }
                else{
                    nonplanar_intersections.push_back(intersection);
                }
            }
        }

        vector<Vector3d> u1, l1, u2, l2, u3, l3, u4, l4, u5, l5;

        // pentagon
        vector<Vector3d> points = {
            coplanar_intersections[0], 
            coplanar_intersections[1], 
            Vector3d(surfaced_vertices.at(0).location[0], surfaced_vertices.at(0).location[1], 0),
            Vector3d(surfaced_vertices.at(1).location[0], surfaced_vertices.at(1).location[1], 0),
            Vector3d(surfaced_vertices.at(2).location[0], surfaced_vertices.at(2).location[1], 0)
        };
        calculateBoundingPaths(points, &u1, &l1);
        IntegrationZone iz1 = IntegrationZone(u1, l1, surfaced_plane, bouyant_density);

        //rectangle 1
        points = {
            Vector3d(double_intersection_vertices[0].location[0], double_intersection_vertices[0].location[1], 0), 
            Vector3d(single_intersection_vertex.location[0], single_intersection_vertex.location[1], 0), 
            single_intersection_vertex.intersections[0],
            nonplanar_intersections[0]
        };
        calculateBoundingPaths(points, &u2, &l2);
        vector<double> p2 = pointsToPlane(double_intersection_vertices[0].location, single_intersection_vertex.location, single_intersection_vertex.intersections[0]);
        IntegrationZone iz2 = IntegrationZone(u2, l2, p2, bouyant_density);

        //rectangle 2
        points = {
            Vector3d(double_intersection_vertices[1].location[0], double_intersection_vertices[1].location[1], 0), 
            Vector3d(single_intersection_vertex.location[0], single_intersection_vertex.location[1], 0), 
            single_intersection_vertex.intersections[0],
            nonplanar_intersections[1]
        };
        calculateBoundingPaths(points, &u3, &l3);
        vector<double> p3 = pointsToPlane(double_intersection_vertices[1].location, single_intersection_vertex.location, single_intersection_vertex.intersections[0]);
        IntegrationZone iz3 = IntegrationZone(u3, l3, p3, bouyant_density);

        //triangle 1
        points = {
            Vector3d(double_intersection_vertices[0].location[0], double_intersection_vertices[0].location[1], 0), 
            double_intersection_vertices[0].intersections[0], 
            double_intersection_vertices[0].intersections[1]
        };
        calculateBoundingPaths(points, &u4, &l4);
        vector<double> p4 = pointsToPlane(double_intersection_vertices[0].location, double_intersection_vertices[0].intersections[0], double_intersection_vertices[0].intersections[1]);
        IntegrationZone iz4 = IntegrationZone(u4, l4, p4, bouyant_density);

        //triangle 2
        points = {
            Vector3d(double_intersection_vertices[1].location[0], double_intersection_vertices[1].location[1], 0), 
            double_intersection_vertices[1].intersections[0], 
            double_intersection_vertices[1].intersections[1]
        };
        calculateBoundingPaths(points, &u5, &l5);
        vector<double> p5 = pointsToPlane(double_intersection_vertices[1].location, double_intersection_vertices[1].intersections[0], double_intersection_vertices[1].intersections[1]);
        IntegrationZone iz5 = IntegrationZone(u5, l5, p5, bouyant_density);

        double total_iz_forces;
        Vector3d iz_overall_acting_location;
        vector<double> iz_forces = {iz1.total_force, iz2.total_force, iz3.total_force, iz4.total_force, iz5.total_force};
        vector<Vector3d> iz_acting_locations = {iz1.bouyant_force_acting_location, iz2.bouyant_force_acting_location, iz3.bouyant_force_acting_location, iz4.bouyant_force_acting_location, iz5.bouyant_force_acting_location};
        sumBouyantForces(iz_forces, iz_acting_locations, &total_iz_forces, &iz_overall_acting_location);
        
        if(submerged){
            // flip the delta bouyant force because it represents a lack of bouyancy in this senario
            total_iz_forces = -total_iz_forces;
            //calcuate the bouyant force of the entire box - bouyancy if entire box is submerged - delta
            vector<double> overall_forces = {bouyant_force, total_iz_forces};
            vector<Vector3d> overall_acting_locations = {cov, iz_overall_acting_location};
            sumBouyantForces(overall_forces, overall_acting_locations, calculated_force, acting_location);
            
            return;
        }else{
            *calculated_force = total_iz_forces;
            *acting_location = iz_overall_acting_location;

            return;
        }
    }else if(size(surfaced_vertices) >= 2){
        //in this case there should be four intersections and four integration zones.
        // two zones will be rectangular and incorporate both surfaced vertices
        // two will be triangular and only incorporate 1 surfaced vertice
        Vector3d location_zero = surfaced_vertices.at(0).location;
        Vector3d location_one = surfaced_vertices.at(1).location;

        // make a list of all of the segments that are on the water plane
        vector<LineSegment> unused_surface_segements;

        int counter = 0;
        while(counter < (size(upper_surface_path) - 1)){
            unused_surface_segements.push_back(LineSegment(upper_surface_path[counter], upper_surface_path[counter + 1]));
            counter += 1;
        }

        counter = 0;
        while(counter < (size(lower_surface_path) - 1)){
            unused_surface_segements.push_back(LineSegment(lower_surface_path[counter], lower_surface_path[counter + 1]));
            counter += 1;
        }

        // find the two zones that's bounding plane includes only one surfaced vertex
        LineSegment l0_segment = LineSegment(surfaced_vertices.at(0).intersections[0], surfaced_vertices.at(0).intersections[1]);
        for(int counter2; counter2 < size(unused_surface_segements); counter2++){
            LineSegment segment = unused_surface_segements.at(counter2);
            // this should only remove one
            if(l0_segment.checkForDuplicate(segment)){
                unused_surface_segements.erase(unused_surface_segements.begin() + counter2);
                break;
            }
        }

        LineSegment l1_segment = LineSegment(surfaced_vertices.at(1).intersections[0], surfaced_vertices.at(1).intersections[1]);
        for(int counter2; counter2 < size(unused_surface_segements); counter2++){
            LineSegment segment = unused_surface_segements.at(counter2);

            // this should only remove one
            if(l1_segment.checkForDuplicate(segment)){
                unused_surface_segements.erase(unused_surface_segements.begin() + counter2);
                break;
            }
        }

        // define paths and planes
        vector<Vector3d> u1, l1, u2, l2, u3, l3, u4, l4;

        //u_ - upper bound, l_ - lower bound, p_ - plane
        vector<Vector3d> points = {
            Vector3d(location_zero[0], location_zero[1], 0), 
            surfaced_vertices.at(0).intersections[0], 
            surfaced_vertices.at(0).intersections[1]
        };
        calculateBoundingPaths(points, &u1, &l1);
        vector<double> p1 = pointsToPlane(location_zero,  surfaced_vertices.at(0).intersections[0], surfaced_vertices.at(0).intersections[1]);

        points = {
            Vector3d(location_one[0], location_one[1], 0), 
            surfaced_vertices.at(1).intersections[0], 
            surfaced_vertices.at(1).intersections[1]
        };
        calculateBoundingPaths(points, &u2, &l2);
        vector<double> p2 = pointsToPlane(location_one,  surfaced_vertices.at(1).intersections[0], surfaced_vertices.at(1).intersections[1]);

        points = {
            Vector3d(location_zero[0], location_zero[1], 0), 
            Vector3d(location_one[0], location_one[1], 0), 
            unused_surface_segements[0].end_point_one, 
            unused_surface_segements[0].end_point_two
        };
        calculateBoundingPaths(points, &u3, &l3);
        vector<double> p3 = pointsToPlane(location_zero, location_one, unused_surface_segements[0].end_point_one);

        points = {
            Vector3d(location_zero[0], location_zero[1], 0), 
            Vector3d(location_one[0], location_one[1], 0), 
            unused_surface_segements[1].end_point_one,
            unused_surface_segements[1].end_point_two
        };
        calculateBoundingPaths(points, &u4, &l4);
        vector<double> p4 = pointsToPlane(location_zero, location_one, unused_surface_segements[1].end_point_one);

        // define integration zones
        IntegrationZone iz1 = IntegrationZone(u1, l1, p1, bouyant_density);
        IntegrationZone iz2 = IntegrationZone(u2, l2, p2, bouyant_density);
        IntegrationZone iz3 = IntegrationZone(u3, l3, p3, bouyant_density);
        IntegrationZone iz4 = IntegrationZone(u4, l4, p4, bouyant_density);

        double total_iz_force;
        Vector3d total_iz_force_acting_location;
        vector<double> iz_forces = {iz1.total_force, iz2.total_force, iz3.total_force, iz4.total_force};
        vector<Vector3d> iz_forces_acting_locations = {iz1.bouyant_force_acting_location, iz2.bouyant_force_acting_location, iz3.bouyant_force_acting_location, iz4.bouyant_force_acting_location};

        sumBouyantForces(iz_forces, iz_forces_acting_locations, &total_iz_force, &total_iz_force_acting_location);

        if(submerged){
            // flip the delta bouyant force because it represents a lack of bouyancy in this senario
            total_iz_force = -total_iz_force;
            //calcuate the bouyant force of the entire box - bouyancy if entire box is submerged - delta
            vector<double> overall_forces = {bouyant_force, total_iz_force};
            vector<Vector3d> overall_forces_acting_location = {cov, total_iz_force_acting_location};
            sumBouyantForces(overall_forces, overall_forces_acting_location, calculated_force, acting_location);

            return; 
        }else{
            *calculated_force = total_iz_force;
            *acting_location = total_iz_force_acting_location;
            return;
        }

    }else{
        //only one vertxex above the water, this means there are three planes - each formed by two intersections and the surfaced vertex
        //four involved points
        Vector3d surfaced_point_1 = surfaced_vertices.at(0).intersections.at(0);
        Vector3d surfaced_point_2 = surfaced_vertices.at(0).intersections.at(1);
        Vector3d surfaced_point_3 = surfaced_vertices.at(0).intersections.at(2);
        Vector3d center_vertex = surfaced_vertices.at(0).location;

        //calculate the upper bounding path, lower bounding path and plane for each of the three planes
        //u_ - upper bound, l_ - lower bound, p_ - plane
        vector<Vector3d> l1, u1, l2, u2, l3, u3;

        vector<Vector3d> points1 = {Vector3d(center_vertex[0], center_vertex[1], 0), surfaced_point_1, surfaced_point_2};
        calculateBoundingPaths(points1, &u1, &l1);
        vector<double> p1 = pointsToPlane(center_vertex, surfaced_point_1, surfaced_point_2);
        
        vector<Vector3d> points2 = {Vector3d(center_vertex[0], center_vertex[1], 0), surfaced_point_1, surfaced_point_3};
        calculateBoundingPaths(points2, &u2, &l2);
        vector<double> p2 = pointsToPlane(center_vertex, surfaced_point_1, surfaced_point_3);

        vector<Vector3d> points3 = {Vector3d(center_vertex[0], center_vertex[1], 0), surfaced_point_3, surfaced_point_2};
        calculateBoundingPaths(points3, &u3, &l3);
        vector<double> p3 = pointsToPlane(center_vertex, surfaced_point_3, surfaced_point_2);
    
        // define and integrate all zones
        IntegrationZone iz1 = IntegrationZone(u1, l1, p1, bouyant_density);
        IntegrationZone iz2 = IntegrationZone(u2, l2, p2, bouyant_density);
        IntegrationZone iz3 = IntegrationZone(u3, l3, p3, bouyant_density);

        double iz_total_force;
        Vector3d iz_overall_acting_location;
        vector<double> iz_forces = {iz1.total_force, iz2.total_force, iz3.total_force};
        vector<Vector3d> iz_acting_locations = {iz1.bouyant_force_acting_location, iz2.bouyant_force_acting_location, iz3.bouyant_force_acting_location};
        sumBouyantForces(iz_forces, iz_acting_locations, &iz_total_force, &iz_overall_acting_location);

        if(submerged){
            // flip the delta bouyant force because it represents a lack of bouyancy in this senario
            iz_total_force = -iz_total_force;
            //calcuate the bouyant force of the entire box - bouyancy if entire box is submerged - delta
            vector<double> overall_forces = {bouyant_force, iz_total_force};
            vector<Vector3d> overall_forces_acting_locations = {cov, iz_overall_acting_location};
            sumBouyantForces(overall_forces, overall_forces_acting_locations, calculated_force, acting_location);
            return;
        }else{
            *calculated_force = iz_total_force;
            *acting_location = iz_overall_acting_location;

            return;
        }

    }

}

Matrix3d rotationFromEuler(double x, double y, double z){
    double a = cos(y) * cos(z);
    double b = sin(x) * sin(y) * cos(z) - cos(x) * sin(z);
    double c = cos(x) * sin(y) * cos(z) + sin(x) * sin(z);
    double d = cos(y) * sin(z);
    double e = sin(x) * sin(y) * sin(z) + cos(x) * cos(z);
    double f = cos(x) * sin(y) * sin(z) - sin(x) * cos(z);
    double g = -sin(y);
    double h = sin(x) * cos(y);
    double i = cos(x) * cos(y);

    Matrix3d matrix;
    matrix << a,b,c,d,e,f,g,h,i;

    return matrix;
}

vector<Vertex> generateRawVertices(vector<double> bB){
    // the raw vertices of the tempest bounding box
    vector<Vertex> vertices;
    
    vertices.push_back(Vertex(Vector3d(-bB[0], -bB[2], -bB[4]), 
        Vector3d((bB[0] + bB[1]), 0, 0),
        Vector3d(0, (bB[2] + bB[3]), 0),
        Vector3d(0, 0, (bB[4] + bB[5]))
    ));

    vertices.push_back(Vertex(Vector3d(-bB[0], -bB[2], bB[5]),
        Vector3d((bB[0] + bB[1]), 0, 0),
        Vector3d(0, (bB[2] + bB[3]), 0),
        Vector3d(0, 0, -(bB[4] + bB[5]))
    ));

    vertices.push_back(Vertex(Vector3d(-bB[0], bB[3], -bB[4]), 
        Vector3d((bB[0] + bB[1]), 0, 0),
        Vector3d(0, -(bB[2] + bB[3]), 0),
        Vector3d(0, 0, (bB[4] + bB[5]))
    ));

    vertices.push_back(Vertex(Vector3d(-bB[0], bB[3], bB[5]), 
        Vector3d((bB[0] + bB[1]), 0, 0),
        Vector3d(0, -(bB[2] + bB[3]), 0),
        Vector3d(0, 0, -(bB[4] + bB[5]))
    ));

    vertices.push_back(Vertex(Vector3d(bB[1], -bB[2], -bB[4]), 
        Vector3d(-(bB[0] + bB[1]), 0, 0),
        Vector3d(0, (bB[2] + bB[3]), 0),
        Vector3d(0, 0, (bB[4] + bB[5]))
    ));

    vertices.push_back(Vertex(Vector3d(bB[1], -bB[2], bB[5]),
        Vector3d(-(bB[0] + bB[1]), 0, 0),
        Vector3d(0, (bB[2] + bB[3]), 0),
        Vector3d(0, 0, -(bB[4] + bB[5]))
    ));

    vertices.push_back(Vertex(Vector3d(bB[1], bB[3], -bB[4]), 
        Vector3d(-(bB[0] + bB[1]), 0, 0),
        Vector3d(0, -(bB[2] + bB[3]), 0),
        Vector3d(0, 0, (bB[4] + bB[5]))
    ));

    vertices.push_back(Vertex(Vector3d(bB[1], bB[3], bB[5]), 
        Vector3d(-(bB[0] + bB[1]), 0, 0),
        Vector3d(0, -(bB[2] + bB[3]), 0),
        Vector3d(0, 0, -(bB[4] + bB[5]))
    ));

    return vertices;
}


