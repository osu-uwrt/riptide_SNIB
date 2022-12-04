from scipy.spatial.transform import Rotation
import numpy as np
import sympy.geometry as sy

print("Runnning Gradient Bouancy 2!")

#water plane is z = 0

class Vertex:
    #postion of vertex
    location = []
    vector_from_center = []
    rotated_vector_from_center = []

    #connections to other vertices
    c1_relative = []
    c2_relative = []
    c3_relative = []

    c1_absolute = []
    c2_absolute = []
    c2_absolute = []

    #connections to intersections
    intersections = []    

    def __init__(self, l, c1, c2, c3):
        self.vector_from_center = l
        self.c1_relative = c1
        self.c2_relative = c2
        self.c3_relative = c3

    def calculateAbsoluteVectors(self, rotation_matrix, translation):
        #apply transform to connecting vectors

        #rotate
        self.c1_absolute = np.dot(rotation_matrix, self.c1_relative)
        self.c2_absolute = np.dot(rotation_matrix, self.c2_relative)
        self.c3_absolute = np.dot(rotation_matrix, self.c3_relative)

        #translate
        self.c1_absolute += translation
        self.c2_absolute += translation
        self.c3_absolute += translation

    def returnAdjacentIntersections(self):
        if len(self.c1_absolute) != 3:
            print("Please ensure the absolute vectors have been calculated!!!!")
            return []

        intersections = []

        #find the points of intersection with the water plane adjacent to this vertex

        #if starting location is underwater
        if self.location[2] < 0:

            #if vector will interests with water plane 
            if (self.c1_absolute[2] + self.location[2]) > 0:

                #porportion of vector needed to reach water plane
                l = -self.location[2] / self.c1_absolute[2]

                # calculate the point of intersection
                intersections.append(self.c1_absolute * l + self.location)

            #if vector will interests with water plane 
            if (self.c2_absolute[2] + self.location[2]) > 0:

                #porportion of vector needed to reach water plane
                l = -self.location[2] / self.c2_absolute[2]

                # calculate the point of intersection
                intersections.append(self.c2_absolute * l + self.location)

            #if vector will interests with water plane 
            if (self.c3_absolute[2] + self.location[2]) > 0:

                #porportion of vector needed to reach water plane
                l = -self.location[2] / self.c3_absolute[2]

                # calculate the point of intersection
                intersections.append(self.c3_absolute * l + self.location)

        else:
            #if vector will interests with water plane 
            if (self.c1_absolute[2] + self.location[2]) < 0:

                #porportion of vector needed to reach water plane
                l = -self.location[2] / self.c1_absolute[2]

                # calculate the point of intersection
                intersections.append(self.c1_absolute * l + self.location)

            #if vector will interests with water plane 
            if (self.c2_absolute[2] + self.location[2]) < 0:

                #porportion of vector needed to reach water plane
                l = -self.location[2] / self.c2_absolute[2]

                # calculate the point of intersection
                intersections.append(self.c2_absolute * l + self.location)

            #if vector will interests with water plane 
            if (self.c3_absolute[2] + self.location[2]) < 0:

                #porportion of vector needed to reach water plane
                l = -self.location[2] / self.c3_absolute[2]

                # calculate the point of intersection
                intersections.append(self.c3_absolute * l + self.location)
                
        self.intersections = intersections

        return intersections

class LineSegment:
    end_point_one = []
    end_point_two = []

    def __init__(self, p1, p2):
        self.end_point_one = p1
        self.end_point_two = p2

    def checkForDuplicate(self, other):
        # check to see if this line segment is the same as the passed
        # false: other and self are different
        # true: other and self are the same

        if self.end_point_one == other.end_point_one:
            if self.end_point_two == other.end_point_two:
                return True
        elif self.end_point_one == other.end_point_two:
            if self.end_point_two == other.end_point_one:
                return True
        
        return False

    def getLength(self):
        return pow(pow((self.end_point_one[0]- self.end_point_two[0]),2) + pow((self.end_point_one[1]- self.end_point_two[1]),2) + pow((self.end_point_one[2]- self.end_point_two[2]),2),0.5)

    def returnXMax(self):
        if self.end_point_one[0] > self.end_point_two[0]:
            return self.end_point_one[0]

        return self.end_point_two[0]

    def isInXBound(self, xValue):
        #If the xValue is in the range of the line segment
        if self.end_point_one[0] > self.end_point_two[0]:
            if (xValue < self.end_point_one[0]) and (xValue > self.end_point_two[0]):
                return True
        else:
            if (xValue > self.end_point_one[0]) and (xValue < self.end_point_two[0]):
                return True

        return False

    def returnInterpolatedPoint(self, xValue):
        #returns [-1] if the point is not on the segment
        if not self.isInXBound(xValue):
            return [-1]

        if self.end_point_two[0]  == self.end_point_one[0]:
            #prevent divide by zero
            return self.end_point_one

        porportion_of_segment = (xValue - self.end_point_one[0]) / (self.end_point_two[0] - self.end_point_one[0])
        vector_from_one = porportion_of_segment * (np.array(self.end_point_two) - np.array(self.end_point_one))

        return (self.end_point_one + vector_from_one)

class IntegrationZone:
    #these bounds are on surface plane
    upper_bound = []
    lower_bound = []

    total_volume = 0

    #plane cofficients [cx, cy, cz, k] : cx*x + cy*y + cz*z = k
    plane_coefficents =[]

    def __init__(self, upper, lower, plane):
        self.plane_coefficents = plane

        #the additional point that are needed to create vertical chops
        lower_additional = []
        upper_additional = []

        for point in upper:
            #add a corrosponding point in lower for every point in upper
            
            # find where to add and add ensure its not a point in lower
            counter = 0 
            while counter < len(lower) - 1:
                segment = LineSegment(lower[counter], lower[counter + 1])
                counter += 1

                if segment.isInXBound(point[0]):
                    lower_additional.append(segment.returnInterpolatedPoint(point[0]))

        for point in lower:
            #add a corrosponding point in lower for every point in upper
            
            # find where to add and add ensure its not a point in lower
            counter = 0 
            while counter < len(upper) - 1:
                segment = LineSegment(upper[counter], upper[counter + 1])
                counter += 1

                if segment.isInXBound(point[0]):
                    upper_additional.append(segment.returnInterpolatedPoint(point[0]))

        self.upper_bound = upper + upper_additional
        self.lower_bound = lower + lower_additional

        def return_first(val):
            return val[0]

        self.upper_bound.sort(key=return_first)
        self.lower_bound.sort(key=return_first)

        print(f"upperbound {self.upper_bound} lowerbound {self.lower_bound}")

        self.integrate_eq(0,-1,1,-1,2,0,2,0,0,1)
        #self.integate()

    def integrate_eq(self, Mymax, Mymin, Bymax, Bymin, Xmax, Xmin, k, Cx, Cy, Cz):
        #plane defined by : k = Cx * X + Cy * Y + Cz * Z
        #Xmin is the lower X bound of integration
        #Xmax is the upper X bound of integration
        #Mymin is the slope of the lower Y bound of integration
        #Mymax is the slope of the uppwer Y bound of integration
        #Bymin is the intercept of the lower Y bound of integration
        #Bymax is the intercept of the upper Y bound of integration
        
        def deltaY(Mymax, Mymin, Bymax, Bymin, x):
            return ((Mymax - Mymin) * x * x  / 2 + (Bymax - Bymin) * x)

        def deltaYSquared(Mymax, Mymin, Bymax, Bymin, x):
            return (pow(Mymax - Mymin, 2) * pow(x, 3) / 3 + (Mymax - Mymin) * (Bymax - Bymin) * pow(x, 2) + pow(Bymax - Bymin, 2) * x)
        
        p1 = (deltaYSquared(Mymax, Mymin, Bymax, Bymin, Xmax) * Cy / -2) - (deltaYSquared(Mymax, Mymin, Bymax, Bymin, Xmin) * Cy / -2)
        p2 = (deltaY(Mymax, Mymin, Bymax, Bymin, Xmax) * k) - (deltaY(Mymax, Mymin, Bymax, Bymin, Xmin) * k)
        p3 = (-Cx * ((Mymax - Mymin) * pow(Xmax,3) / 3 + (Bymax - Bymin) * pow(Xmax, 2) / 2)) - (-Cx * ((Mymax - Mymin) * pow(Xmin,3) / 3+ (Bymax - Bymin) * pow(Xmin, 2) / 2))

        print("ps")
        print(p1)
        print(p2)
        print(p3)
        
        return ((p1 + p2 + p3) / Cz)

    def pointsToMxPlusB(self, p1, p2):
        #takes a line segment class instance
        
        m = (p1[1] - p2[1]) / (p1[0] - p2[0])
        b = p1[1] - (p1[0] * m)

        return m, b


    def integate(self):
        #integrate the based on upper and lower bounds and plane

        #there should be one less zone than points in upper bound
        #lower bound should have the same number of points as upper bound
        counter = 0
        #need to skip over vertical segments
        lower_vertical = 0
        upper_vertical = 0
        while(counter + upper_vertical < len(self.upper_bound) - 1):
            
            #if the upper segment is vertical; skip
            while(self.upper_bound[upper_vertical + counter][0] == self.upper_bound[upper_vertical + 1 + counter][0]):
                upper_vertical += 1

            #if the lower segment is vertical; skip
            while(self.lower_bound[lower_vertical + counter][0] == self.lower_bound[lower_vertical + counter + 1][0]):
                lower_vertical += 1

            mHigh, bHigh = self.pointsToMxPlusB(self.upper_bound[counter + upper_vertical], self.upper_bound[counter + upper_vertical + 1])
            mLow, bLow = self.pointsToMxPlusB(self.lower_bound[counter + lower_vertical], self.lower_bound[counter + lower_vertical + 1])

            self.total_volume += self.integrate_eq(mHigh, mLow, bHigh, bLow, 
                self.upper_bound[counter + upper_vertical + 1][0], self.upper_bound[counter + upper_vertical][0], 
                self.plane_coefficents[3], self.plane_coefficents[0], self.plane_coefficents[1], self.plane_coefficents[2],
                )

            counter += 1
            
            



#this only needs to be done once -------------------------
#   Measurements from COB in meters

#[-x, +x, -y, +y, -z, +z]
# all values should be positive!!!!!
bB = [.25, .26, .30, .35, .2, .1] #The bouancy box of tempest; TODO -- get actual

#determine vertices as vectors from center of box
#   nnn corrosponds to the veritice at [-x, -y, -z]; pnp would be [+x, -y, +z]
nnn = Vertex(np.array([-bB[0], -bB[2], -bB[4]]), #0
    np.array([(bB[0] + bB[1]), 0, 0]),
    np.array([0, (bB[2] + bB[3]), 0]),
    np.array([0, 0, (bB[4] + bB[5])])
)

nnp = Vertex(np.array([-bB[0], -bB[2], bB[5]]), #1
    np.array([(bB[0] + bB[1]), 0, 0]),
    np.array([0, (bB[2] + bB[3]), 0]),
    np.array([0, 0, -(bB[4] + bB[5])])
)

npn = Vertex(np.array([-bB[0], bB[3], -bB[4]]), #2
    np.array([(bB[0] + bB[1]), 0, 0]),
    np.array([0, -(bB[2] + bB[3]), 0]),
    np.array([0, 0, (bB[4] + bB[5])])
)

npp = Vertex(np.array([-bB[0], bB[3], bB[5]]), #3
    np.array([(bB[0] + bB[1]), 0, 0]),
    np.array([0, -(bB[2] + bB[3]), 0]),
    np.array([0, 0, -(bB[4] + bB[5])])
)

pnn = Vertex(np.array([bB[1], -bB[2], -bB[4]]), #4
    np.array([-(bB[0] + bB[1]), 0, 0]),
    np.array([0, (bB[2] + bB[3]), 0]),
    np.array([0, 0, (bB[4] + bB[5])])
)

pnp = Vertex(np.array([bB[1], -bB[2], bB[5]]),#5
    np.array([-(bB[0] + bB[1]), 0, 0]),
    np.array([0, (bB[2] + bB[3]), 0]),
    np.array([0, 0, -(bB[4] + bB[5])])
)

ppn = Vertex(np.array([bB[1], bB[3], -bB[4]]), #6
    np.array([-(bB[0] + bB[1]), 0, 0]),
    np.array([0, -(bB[2] + bB[3]), 0]),
    np.array([0, 0, (bB[4] + bB[5])])
)

ppp = Vertex(np.array([bB[1], bB[3], bB[5]]), #7
    np.array([-(bB[0] + bB[1]), 0, 0]),
    np.array([0, -(bB[2] + bB[3]), 0]),
    np.array([0, 0, -(bB[4] + bB[5])])
)

raw_vertices = [nnn, nnp, npn, npp, pnn, pnp, ppn, ppp]

bouyant_force = 100 # the bouyant force when the robot is completely underwater
bouyant_density = bouyant_force / (bB[0] + bB[1]) * (bB[2] + bB[3]) * (bB[4] + bB[5]) # N/m^3s

#this needs to be done at every cycle --------------------------------
#rotate each vertice vector to corrospond to robot orientation
euler_rotation = [45, 42, 0] #in degrees for time being
position = [0,0,-.26] #x,y,z COB

def calculateBoundingPaths(points):
    if(len(points) < 3):
        # thats not supposed to happen
        return None, None

    def return_first(val):
        return val[0]

    # sort the points by x coordinate
    points.sort(key=return_first)

    #initial point for both bounds is the same - so is the final
    lower_bound = [points[0]]
    upper_bound = lower_bound.copy()

    #once the number of points in the two bounds is equal to the total number of points, add the final point 
    #   initial point is double counted but the final point isn't
    counter = 1
    midline_slope = (points[len(points) - 1][1] - points[0][1]) / (points[len(points) - 1][0] - points[0][0])

    while (len(lower_bound) + len(upper_bound)) != len(points):
        #points above midline go in upper bound below go in
        if(points[counter][1] > points[0][1] + midline_slope *(points[counter][0] - points[0][0])):
            upper_bound.append(points[counter])
        else: 
            lower_bound.append(points[counter])

        counter += 1

    # finish out bound path
    upper_bound.append(points[len(points) - 1])
    lower_bound.append(points[len(points) - 1])

    return lower_bound, upper_bound

def pointsToPlane(p1, p2, p3):
    #calcuates the coefficents for a plane equation from three points
    #Cy * Y + Cx * X + Cz * Z = k
    # inputs should be in [x, y, z]
    #returns [Cx, Cy, Cz, k]

    cp = np.cross(np.array(p1) - np.array(p2), np.array(p1) - np.array(p3))
    k = np.dot(cp, np.array(p1))

    print(f"p1{p1}, p2{p2}, p3{p3}")

    return[cp[0], cp[1], cp[2], k]
    
def calculateBouyancy(raw_vertices, euler_rotation, position, bouyant_density, bouyant_force):
    #calculate roation matrix
    rotation_matrix = np.array(Rotation.from_euler("xyz", euler_rotation, degrees=True).as_matrix())

    #calculate rotated vertices
    for vertex in raw_vertices:
        vertex.rotated_vector_from_center = np.dot(rotation_matrix, vertex.vector_from_center)

    #adjust vertice for position of robot
    for vertex in raw_vertices:
        vertex.location = (vertex.rotated_vector_from_center + position)

    #if midpoint is above water plane, robot is not submerged; if below, robot is submerged
    #   evaluate space where midpoint is not; if below, evaluate above; vice versa
    submerged = True
    if position[2] > 0:
        submerged = False

    if submerged:
        # evaluate volume above water

        surfaced_vertices = []
        for vertex in raw_vertices:
            # find all submerged vertices
            if vertex.location[2] > 0:
                surfaced_vertices.append(vertex)
        
        if len(surfaced_vertices) == 0:
            # if no vertices are above water, bouyant force is trivial
            return bouyant_force 

        #calculate where the edges of the bow intersect with the water plane
        surface_intersections = []
        for vertex in surfaced_vertices:
            vertex.calculateAbsoluteVectors(rotation_matrix, position)
            
            intersections = vertex.returnAdjacentIntersections()
            for intersection in intersections:
                surface_intersections.append([intersection])

        if len(surface_intersections) < 3:
            print(f"This should not happen, it takes three vertices to define a plane but there are only {len(surface_intersections)}!")
            return 0

        #calcuate the two bounds of the n-gon created by the surface_intersection points
        ordered_intersections = []
        for point in surface_intersections:
            ordered_intersections.append(point[0])

        lowerSurfacePaths, upperSurfacePath = calculateBoundingPaths(ordered_intersections)

        surfaced_segments = []
        if(len(surfaced_vertices) == 4):
            # there is a plane the is entirely above the surface of the water -- only possiblity is 4 in theory
            l0 = surfaced_vertices[0].location
            l1 = surfaced_vertices[1].location            
            l2 = surfaced_vertices[2].location
            l3 = surfaced_vertices[3].location

            #determine all six possible line segments
            segments = [
                LineSegment(l0, l1),
                LineSegment(l0, l2),
                LineSegment(l0, l3),
                LineSegment(l1, l2),
                LineSegment(l1, l3),
                LineSegment(l2, l3),
            ]

            #eliminate longest two to leave outer bounds of rectangle only
            def returnLength(val):
                return val.getLength()

            segments.sort(key=returnLength)
            surfaced_segments = segments[:4]
        elif(len(surfaced_vertices) == 3):
            l0 = surfaced_vertices[0].location
            l1 = surfaced_vertices[1].location            
            l2 = surfaced_vertices[2].location  

            surfaced_segments = [
                LineSegment(l0, l1),
                LineSegment(l0, l2),
                LineSegment(l1, l2)                
            ]  
        elif(len(surfaced_vertices) >= 2):
            l0 = surfaced_vertices[0].location
            l1 = surfaced_vertices[1].location

            surfaced_segments = [LineSegment(l0, l1)]  
        else:
            #only one vertxex above the water, this means there are three planes - each formed by two intersections and the surfaced vertex

            #four involved points
            surfaced_point_1, surfaced_point_2, surfaced_point_3 = surfaced_vertices[0].intersections
            center_vertex = surfaced_vertices[0].location

            #calculate the upper bounding path, lower bounding path and plane for each of the three planes
            #u_ - upper bound, l_ - lower bound, p_ - plane
            l1, u1 = calculateBoundingPaths([[center_vertex[0], center_vertex[1], 0], surfaced_point_1, surfaced_point_2])
            p1 = pointsToPlane(center_vertex, surfaced_point_1, surfaced_point_2)
            # l2, u2 = calculateBoundingPaths([[center_vertex[0], center_vertex[1], 0], surfaced_point_1, surfaced_point_3])
            # p2 = pointsToPlane(center_vertex, surfaced_point_1, surfaced_point_3)
            # l3, u3 = calculateBoundingPaths([[center_vertex[0], center_vertex[1], 0], surfaced_point_3, surfaced_point_2])
            # p3 = pointsToPlane(center_vertex, surfaced_point_3, surfaced_point_2)
        
            # define and integrate all zones
            iz1 = IntegrationZone(u1, l1, p1)
            # iz2 = IntegrationZone(u2, l2, p2)
            # iz3 = IntegrationZone(u3, l3, p3)

            print(iz1.total_volume)

            # total_volume = iz1.total_volume + iz2.total_volume + iz3.total_volume
            # print(total_volume)






        


        


b = calculateBouyancy(raw_vertices, euler_rotation, position, bouyant_density, bouyant_force)
print(f"Bouyancy is {b}!")


