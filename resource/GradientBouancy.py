from scipy.spatial.transform import Rotation
import numpy as np
import sympy.geometry as sy
import time

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
    c3_absolute = []

    #connections to intersections
    intersections = []    

    def __init__(self, l, c1, c2, c3):
        self.vector_from_center = l
        self.c1_relative = c1
        self.c2_relative = c2
        self.c3_relative = c3

    def reflectAcrossWaterPlane(self):
        # make the z coordinate into is opposite
        self.location[2] = - self.location[2]

        # all the relative vectors will now be have opposite z components so adjust for that too
        self.c1_relative[2] = -self.c1_relative[2]
        self.c2_relative[2] = -self.c2_relative[2]
        self.c3_relative[2] = -self.c3_relative[2]

    def calculateAbsoluteVectors(self, rotation_matrix, translation):
        #apply transform to connecting vectors

        #rotate
        self.c1_absolute = np.dot(rotation_matrix, self.c1_relative)
        self.c2_absolute = np.dot(rotation_matrix, self.c2_relative)
        self.c3_absolute = np.dot(rotation_matrix, self.c3_relative)

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

    vertex_one = None
    vertex_two = None

    includes_vertices = False

    def __init__(self, p1, p2):
        if(isinstance(p1, Vertex) and isinstance(p2, Vertex)):
            # if one is a vertex, both must be vertexs
            self.includes_vertices = True

            self.end_point_one = p1.location
            self.end_point_two = p2.location

            self.vertex_one = p1
            self.vertex_two = p2

        else:
            self.end_point_one = p1
            self.end_point_two = p2

    def checkForDuplicate(self, other):
        # check to see if this line segment is the same as the passed
        # false: other and self are different
        # true: other and self are the same

        if not any(np.array(self.end_point_one) - np.array(other.end_point_one)):
            if not any(np.array(self.end_point_two) - np.array(other.end_point_two)):
                return True
        elif not any(np.array(self.end_point_one) - np.array(other.end_point_two)):
            if not any(np.array(self.end_point_two) - np.array(other.end_point_one)):
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
    total_force = 0
    bouyant_force_acting_loaction = [0, 0, 0]

    #plane cofficients [cx, cy, cz, k] : cx*x + cy*y + cz*z = k
    plane_coefficents = []
    bouyant_density = 0

    def __init__(self, upper, lower, plane, bouyant_density):
        self.plane_coefficents = plane
        self.bouyant_density = bouyant_density

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

        #self.integrate_eq(0,-1,1,-1,2,0,2,0,0,1)
        self.integrate()

    def integrate_eq(self, Mymax, Mymin, Bymax, Bymin, Xmax, Xmin, k, Cx, Cy, Cz):
        #plane defined by : k = Cx * X + Cy * Y + Cz * Z
        #Xmin is the lower X bound of integration
        #Xmax is the upper X bound of integration
        #Mymin is the slope of the lower Y bound of integration
        #Mymax is the slope of the uppwer Y bound of integration
        #Bymin is the intercept of the lower Y bound of integration
        #Bymax is the intercept of the upper Y bound of integration
        
        def deltaY(Mymax, Mymin, Bymax, Bymin, x):
            return ((Mymax - Mymin) * x * x / 2 + (Bymax - Bymin) * x)

        def deltaYSquared(Mymax, Mymin, Bymax, Bymin, x):
            return ((Mymax**2 - Mymin**2) * pow(x, 3) / 3 + ((Mymax * Bymax) - (Mymin * Bymin)) * pow(x, 2) + (Bymax**2 - Bymin**2) * x)
        
        p1 = (deltaYSquared(Mymax, Mymin, Bymax, Bymin, Xmax) * Cy / -2) - (deltaYSquared(Mymax, Mymin, Bymax, Bymin, Xmin) * Cy / -2)
        p2 = (deltaY(Mymax, Mymin, Bymax, Bymin, Xmax) * k) - (deltaY(Mymax, Mymin, Bymax, Bymin, Xmin) * k)
        p3 = (-Cx * ((Mymax - Mymin) * pow(Xmax,3) / 3 + (Bymax - Bymin) * pow(Xmax, 2) / 2)) - (-Cx * ((Mymax - Mymin) * pow(Xmin,3) / 3+ (Bymax - Bymin) * pow(Xmin, 2) / 2))

        return ((p1 + p2 + p3) / Cz)

    def calculateCentroid(self, Mymax, Mymin, Bymax, Bymin, Xmax, Xmin, k, Cx, Cy, Cz, volume):
        # calculate the centroid of a sub zone  and return its location

        x = self.calculateXCentroid(Mymax, Mymin, Bymax, Bymin, Xmax, Xmin, k, Cx, Cy, Cz, volume)
        y = self.calculateYCentroid(Mymax, Mymin, Bymax, Bymin, Xmax, Xmin, k, Cx, Cy, Cz, volume)
        z = self.calculateZCentroid(Mymax, Mymin, Bymax, Bymin, Xmax, Xmin, k, Cx, Cy, Cz, volume)

        return [x, y ,z]    

    def calculateZCentroid(self, Mymax, Mymin, Bymax, Bymin, Xmax, Xmin, k, Cx, Cy, Cz, volume):
        # calculate the z centroid
        #same param format as integrating for area
        # volume is the previously calculated volume

        #lets stop some divide by zero errors ;)
        # TODO -  change out with alternative formulas
        if Cx == 0:
            Cx = .0000001
        
        if Cy == 0:
            Cy = .0000001

        if Cy * Mymin == -1 * Cx:
            Cx = Cx + .0000001

        if Cy * Mymax == -1 * Cx:
            Cx = Cx + .0000001

        if Cz == 0:
            print("Error: Upper bounding plane is perpendicular the z-axis!")

        p1 = pow((Xmax * Cx - k + Xmax * Mymax * Cy + Bymax * Cy),4) - pow((Bymax * Cy - k + Xmin *(Cx + Mymax * Cy)), 4)
        p2 = pow((Bymin * Cy - k + Xmin * (Cx + Mymin * Cy)), 4) - pow((Bymin * Cy - k + Xmax * (Cx + Mymin * Cy)),4)

        return (p1 / (Cx + Mymax * Cy) + p2 / (Cx + Mymin * Cy)) / (24 * Cy * (Cz ** 2) * volume)

    def calculateYCentroid(self, c, h, d, j, a, b, k, l, m, o, volume):
        #Mymax = c
        #Mymin = h
        #Bymax = d
        #Bymin = j
        #Xmax = a
        #Xmin = b
        #k = k
        #Cx = l
        #Cy = m
        #Cz = o
        # everything was assigned differently in mathmatica 

        # if c or h is zero -- very possible -- there is a divide by zero error
        if c == 0:
            p1 = a ** 4 * h ** 2 * (3 * l + 2 * h * m)
            p2 = 4 * a ** 3 * h * (-h * k + 2 * j * l + 2 * h * j * m)
            p3 = 4 * a * (-3 * d ** 2 * k + 2 * d ** 3 * m + j ** 2 * (3 * k - 2 * j * m))
            p4 = 6 * a ** 2 * ((j ** 2 - d ** 2) * l + 2 * h * j * (j * m - k))
            p6 = 6 * d ** 2 * (2 * k - b * l)
            p7 = 8 * d ** 3 * m
            p8 = b ** 3 * h ** 2 * (3 * l + 2 * h * m)
            p9 = 4 * j ** 2 * (-3 * k + 2 * j * m)
            p10 = 6 * b * j * (j * l - 2 * h * k + 2 * h * j * m)
            p11 = 4 * b ** 2 * h * (2 * j * l - h * k + 2 * h * j * m)

            return (p1 + p2 - p3  + p4 - b * (p6 - p7 + p8 + p9 + p10 + p11)) / (24 * o * volume)  
        elif h == 0:
            p1 = (4 * ((a * c + d) ** 3 - (b * c + d) ** 3) * k) / c
            p2 = 12 * b * j ** 2 * k - 12 * a * j ** 2 * k - 3 * a ** 4 * c ** 2 * l + 3 * b ** 4 * c ** 2 * l
            p3 = 8 * b ** 3 * c * d * l - 8 * a ** 3 * c * d * l - 6 * a ** 2 * d ** 2 * l + 6 * b ** 2 * d ** 2 * l 
            p4 = 6 * a ** 2 * j ** 2 * l - 6 * b ** 2 * j ** 2 * l + 8 * a * j ** 3 * m - 8 * b * j ** 3 * m
            p5 = 2 * m * ((a * c + d) ** 4 - (b * c + d) ** 4) / c

            return (p1 + p2 + p3 + p4 - p5) / (24 * o * volume)

        p1 = (4 * ((a * c + d) ** 3 - (b * c + d) ** 3) * k) / c
        p2 = 4 * k * ((a * h + j) ** 3 - (b * h +j) ** 3) / h
        p3 = -3 * a ** 4 * c ** 2 * l + 3 * b ** 4 * c ** 2 * l - 8 * a ** 3 * c * d * l + 8 * b ** 3 * c * d * l
        p4 = -6 * a ** 2 * d ** 2 * l + 6 * b ** 2 * d ** 2 * l + 3 * a ** 4 * h ** 2 * l - 3 * b ** 4 * h ** 2 * l
        p5 = 8 * a ** 3 * h * j * l - 8 * b ** 3 * h * j * l + 6 * a ** 2 * j ** 2 * l - 6 * b ** 2 * j ** 2 * l
        p6 = 2 * m * ((a * c + d) ** 4 - (b * c + d) ** 4) / c
        p7 = 2 * m * ((a * h + j) ** 4 - (b * h + j) ** 4) / h
        
        return (p1 - p2 + p3 + p4 + p5 - p6 + p7) / (24 * o * volume)

    def calculateXCentroid(self, c, h, d, j, a, b, k, l, m, o, volume):
        #Mymax = c
        #Mymin = h
        #Bymax = d
        #Bymin = j
        #Xmax = a
        #Xmin = b
        #k = k
        #Cx = l
        #Cy = m
        #Cz = o
        # everything was assigned differently in mathmatica 

        p1 = 12 * j * k - 12 * d * k + 6 * d ** 2 * m - 6 * j ** 2 * m
        p2 = 8 * a * d * (l + c * m) - 8 * a * j * (l + h * m)
        p3 = a * (c - h) * (6 * a * l - 8 * k + 3 * a * m * (c + h))
        p5 = 8 * b * d * (l + c * m) - 8 * b * j * (l + h * m)
        p6 =  b * (c - h) * (6 * b * l - 8 * k + 3 * b * m * (c + h))

        return (a ** 2 *(p1 + p2 + p3) - b ** 2 * (p1 + p5 + p6)) / (-24 * o * volume)

    def pointsToMxPlusB(self, p1, p2):
        #takes a line segment class instance
        
        m = (p1[1] - p2[1]) / (p1[0] - p2[0])
        b = p1[1] - (p1[0] * m)

        return m, b

    def integrate(self):
        #integrate the based on upper and lower bounds and plane

        #there should be one less zone than points in upper bound
        #lower bound should have the same number of points as upper bound
        counter = 0
        #need to skip over vertical segments
        lower_vertical = 0
        upper_vertical = 0
        while(counter + upper_vertical < (len(self.upper_bound) - 1)) and (counter + lower_vertical < (len(self.lower_bound) - 1)):
            #if the upper segment is vertical; skip
            while(abs(self.upper_bound[upper_vertical + counter][0] - self.upper_bound[upper_vertical + 1 + counter][0]) < 2.7e-10):
                #if the width of the zone is small that that of a water molecule, its essentially vertical (this is done for a reason)
                upper_vertical += 1
                
                # in case all segments are vertical
                if(counter + upper_vertical >= (len(self.upper_bound) - 1)):
                    break

            #if the lower segment is vertical; skip
            while(abs(self.lower_bound[lower_vertical + counter][0] - self.lower_bound[lower_vertical + counter + 1][0]) < 2.7e-10):
                #if the width of the zone is small that that of a water molecule, its essentially vertical (this is done for a reason)
                lower_vertical += 1
                
                # in case all segments are vertical
                if(counter + lower_vertical >= (len(self.lower_bound) - 1)):
                    break

            # in case all segments are vertical
            if(counter + upper_vertical >= (len(self.upper_bound) - 1)):
                break

            if(counter + lower_vertical >= (len(self.lower_bound) - 1)):
                break


            mHigh, bHigh = self.pointsToMxPlusB(self.upper_bound[counter + upper_vertical], self.upper_bound[counter + upper_vertical + 1])
            mLow, bLow = self.pointsToMxPlusB(self.lower_bound[counter + lower_vertical], self.lower_bound[counter + lower_vertical + 1])

            volume = self.integrate_eq(mHigh, mLow, bHigh, bLow, 
                self.upper_bound[counter + upper_vertical + 1][0], self.upper_bound[counter + upper_vertical][0], 
                self.plane_coefficents[3], self.plane_coefficents[0], self.plane_coefficents[1], self.plane_coefficents[2],
                )
            self.total_volume +=  volume
            centroid = self.calculateCentroid(mHigh, mLow, bHigh, bLow, 
                self.upper_bound[counter + upper_vertical + 1][0], self.upper_bound[counter + upper_vertical][0], 
                self.plane_coefficents[3], self.plane_coefficents[0], self.plane_coefficents[1], self.plane_coefficents[2],
                volume)

            bouyant_force = volume * bouyant_density
            self.total_force, self.bouyant_force_acting_loaction = sumBouyantForces([self.total_force, bouyant_force], [self.bouyant_force_acting_loaction, centroid])

            counter += 1

        if (len(self.upper_bound) - counter - upper_vertical != 1) and (len(self.lower_bound) - counter - lower_vertical):
            print("Failing to integrate all sub zones due to mismatched upper and lower bounds!")
    
    def invertForceDirection(self):
        # flips the direction of the bouyant for given by the zone
        self.total_force = -self.total_force
            
#this only needs to be done once -------------------------
#   Measurements from COB in meters

#[-x, +x, -y, +y, -z, +z]
# all values should be positive!!!!!
bB = [.25, .26, .30, .35, .2, .1] #The bouancy box of tempest; TODO -- get actual
print(time.time())

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

# these need to be in this order!
raw_vertices = [nnn, nnp, npn, npp, pnn, pnp, ppn, ppp]

bouyant_force = 100 # the bouyant force when the robot is completely underwater
bouyant_density = bouyant_force / (bB[0] + bB[1]) / (bB[2] + bB[3]) / (bB[4] + bB[5]) # N/m^3s

#this needs to be done at every cycle --------------------------------
#rotate each vertice vector to corrospond to robot orientation
euler_rotation = [1, 0, 0] #in degrees for time being
position = [1,0, 0] #x,y,z COB

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


    if (points[len(points) - 1][0] - points[0][0]) == 0:
        midline_slope = 999999999
    else:
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

    return[cp[0], cp[1], cp[2], k]

def isCoplanar(plane, point):
    # plane should be in [Cx, Cy, Cz, k] form
    # point should be in [x, y, z] form
    # checks to see if a point is on a plane

    if plane[3] == (plane[0] * point[0] + plane[1] * point[1] + plane[2] * point[2]):
        return True
    
    return False

def determineIfAdditiveArea(izv1, izv2, izi1, izi2, izo1, izo2):
    #Used to Determine if an integration zone is additive in the case where four vertices are being evaluated
    # izv# represents a vertex that is bounding the zone
    # izi# represents an intersection that is bound the zone
    # izo# represents a vertext that is not bounding the zone

    # returning true is additive area, returning false is negative area

    #line created by the instersection of the water plane and a vertical plane created with izv1 and izv2
    slope = 999999999

    if (izv2[0] - izv1[0]) != 0:
       slope = (izv2[1] - izv1[1]) / (izv2[0] - izv1[0])
    elif ((izi1[0] - izi2[0]) == 0) and (izv1[0] == izi1[0]):
        # if the line is vertical
        #if the second line is vertical too and shares the same x coordinate, the volume of the iz will be zero 
        return True

    y_intercept = izv1[1] - (slope * izv1[0])

    # determine if the intersections are above the line
    if (izi1[1] > (izi1[0] * slope + y_intercept)) and (izi2[1] > (izi2[0] * slope + y_intercept)):
        # the intersection are above the line
        # if the vertices that are not bounding the box are above the line as well, the integration zone represents negative area - if not additive area
        if (izo1[1] > (izo1[0] * slope + y_intercept)) and (izo2[1] > (izo2[0] * slope + y_intercept)):
            # negative area
            return False
        elif (izo1[1] < (izo1[0] * slope + y_intercept)) and (izo2[1] < (izo2[0] * slope + y_intercept)):
            #additive area
            return True
        else:
            print("This really shouldn't happen")

    elif (izi1[1] < (izi1[0] * slope + y_intercept)) and (izi2[1] < (izi2[0] * slope + y_intercept)):
        # the intersection are below the line
        # if the vertices that are not bounding the box are below the line as well, the integration zone represents negative area - if not additive area

        if (izo1[1] > (izo1[0] * slope + y_intercept)) and (izo2[1] > (izo2[0] * slope + y_intercept)):
            # additive area
            return True
        elif (izo1[1] < (izo1[0] * slope + y_intercept)) and (izo2[1] < (izo2[0] * slope + y_intercept)):
            #negative area
            return False
        else:
            print("This really shouldn't happen")

    elif (izi1[1] < (izi1[0] * slope + y_intercept)) and (izi2[1] < (izi2[0] * slope + y_intercept)):
        #if both points are on the line, it doesn't matter, the area will be zero
        return True
    else:
        print("This shouldn't happen... ")

    #casually return false - this should never happen
    return False

def findOtherVertices(surfaced_vertices, vertices):
    #used in conjunction with determine if additive area
    #finds the vertices not bounding the plane
    other_vertices = []
    for vertex in surfaced_vertices:
        if not (vertex in vertices):
            other_vertices.append(vertex)
    
    return other_vertices


def sumBouyantForces(forces, positions):
    # z position of bouyant forces dont matter
    # Returns a scalar value for a for in the z direction and a acting location in the form of [x, y, z]
    # forces: [] or scalar bouyant forces
    # positions: [] of positions where forces act - assumed to be in same order as forces and in [x, y, z] format

    if len(forces) != len(positions):
        print("Length of forces and positions must be equal to sum!")

    # find the total bouyant force
    total_force = 0
    for force in forces:
        total_force += force

    # this shouldn't ever happen but just in case
    if total_force == 0:
        return 0, [0, 0, 0]

    # find the total torque around the x-axis
    x_torque = 0
    counter = 0
    for position in positions:
        x_torque += position[1] * forces[counter] 
        counter += 1

    #find the total torque around the y-axis
    y_torque = 0
    counter = 0
    for position in positions:
        y_torque += position[0] * forces[counter] 
        counter += 1

    return total_force, [y_torque / total_force, x_torque / total_force, 0]

def calculateBouyancy(raw_vertices, euler_rotation, position, bouyant_density, bouyant_force):
    #calculate roation matrix
    rotation_matrix = np.array(Rotation.from_euler("xyz", euler_rotation, degrees=True).as_matrix())

    #calculate the box cov - different from positon
    #  the relative vector from the position passed to the cov of the box
    relative_postion_cob_vector = np.array([
        (raw_vertices[0].vector_from_center[0] + raw_vertices[7].vector_from_center[0]) / 2,
        (raw_vertices[0].vector_from_center[1] + raw_vertices[7].vector_from_center[1]) / 2,
        (raw_vertices[0].vector_from_center[2] + raw_vertices[7].vector_from_center[2]) / 2,        
    ])

    absolute_postion_cob_vector = np.dot(rotation_matrix, relative_postion_cob_vector)
    cov = absolute_postion_cob_vector + position

    #calculate rotated vertices
    for vertex in raw_vertices:
        vertex.rotated_vector_from_center = np.dot(rotation_matrix, vertex.vector_from_center)

    #adjust vertice for position of robot
    for vertex in raw_vertices:
        vertex.location = (vertex.rotated_vector_from_center + position)

    #if midpoint is above water plane, robot is not submerged; if below, robot is submerged
    #   evaluate space where midpoint (cov not position) is not; if below, evaluate above; vice versa
    submerged = True
    if cov[2] > 0:
        submerged = False

    if not submerged:
        # we need to evaluate volume above water and the simplest shapes possible
        # if the simpler shape is below the water, reflect the box across the water plane and then take that into account when calculating the final bouyancy and acting location
        for vertex in raw_vertices:
            vertex.reflectAcrossWaterPlane()

    surfaced_vertices = []
    for vertex in raw_vertices:
        # find all submerged vertices
        if vertex.location[2] > 0:
            surfaced_vertices.append(vertex)
    
    if len(surfaced_vertices) == 0:
        # if no vertices are above water, bouyant force is trivial
        return bouyant_force, [0,0,0]

    #calculate where the edges of the bow intersect with the water plane
    surface_intersections = []
    for vertex in surfaced_vertices:
        vertex.calculateAbsoluteVectors(rotation_matrix, position)
        
        intersections = vertex.returnAdjacentIntersections()
        for intersection in intersections:
            surface_intersections.append([intersection])

    if len(surface_intersections) < 3:
        print(f"This should not happen, it takes three vertices to define a plane but there are only {len(surface_intersections)}!")
        return 0, [0,0,0]

    #calcuate the two bounds of the n-gon created by the surface_intersection points
    ordered_intersections = []
    for point in surface_intersections:
        ordered_intersections.append(point[0])
    
    lowerSurfacePaths, upperSurfacePath = calculateBoundingPaths(ordered_intersections)

    surfaced_segments = []
    if(len(surfaced_vertices) == 4):
        # there is a plane the is entirely above the surface of the water -- only possiblity is 4 in theory

        #determine all six possible line segments that are completely on top of the water
        segments = [
            LineSegment(surfaced_vertices[0], surfaced_vertices[1]),
            LineSegment(surfaced_vertices[0], surfaced_vertices[2]),
            LineSegment(surfaced_vertices[0], surfaced_vertices[3]),
            LineSegment(surfaced_vertices[1], surfaced_vertices[2]),
            LineSegment(surfaced_vertices[1], surfaced_vertices[3]),
            LineSegment(surfaced_vertices[2], surfaced_vertices[3]),
        ]

        #eliminate longest two to leave outer bounds of rectangle only
        def returnLength(val):
            return val.getLength()

        segments.sort(key=returnLength)
        surfaced_segments = segments[:4]

        # there will be five planes each bounded by four sides
        # one plane will be completely above the water, the rest will intersect it

        #above water plane
        l1, u1 = calculateBoundingPaths([
            [surfaced_vertices[0].location[0], surfaced_vertices[0].location[1], 0],
            [surfaced_vertices[1].location[0], surfaced_vertices[1].location[1], 0],
            [surfaced_vertices[2].location[0], surfaced_vertices[2].location[1], 0],
            [surfaced_vertices[3].location[0], surfaced_vertices[3].location[1], 0],
        ])
        p1 = pointsToPlane(surfaced_vertices[0].location, surfaced_vertices[1].location, surfaced_vertices[2].location)
        iz1 = IntegrationZone(u1, l1, p1, bouyant_density)

        #if the intersections and surfaced vertices are vertical iz1 is the only non zero integration zone
        if submerged and (surfaced_vertices[0].location[0] == surfaced_vertices[0].intersections[0][0]) and (surfaced_vertices[0].location[1] == surfaced_vertices[0].intersections[0][1]):
            # for a box, if one is vertical, they all are vertical
            return sumBouyantForces([bouyant_force, -iz1.total_force], [cov, iz1.bouyant_force_acting_loaction])
        elif (surfaced_vertices[0].location[0] == surfaced_vertices[0].intersections[0][0]) and (surfaced_vertices[0].location[1] == surfaced_vertices[0].intersections[0][1]):
            return [iz1.total_force, iz1.bouyant_force_acting_loaction]

        # some of the zones in this case represent negative area above water - all but two - it is important to figure out which
        # this can be done by seeing of the intersections and the surfaced vertices not forming the bound of the integration zone are on the same side of a perfectly vertical plane formed by the two surfaced vertices bounding the zone

        # side plane one
        l2, u2 = calculateBoundingPaths([
            [surfaced_segments[0].vertex_one.location[0], surfaced_segments[0].vertex_one.location[1], 0], 
            [surfaced_segments[0].vertex_two.location[0], surfaced_segments[0].vertex_two.location[1], 0],
            surfaced_segments[0].vertex_one.intersections[0],
            surfaced_segments[0].vertex_two.intersections[0]                
        ])
        p2 = pointsToPlane(surfaced_segments[0].vertex_one.location, surfaced_segments[0].vertex_one.intersections[0], surfaced_segments[0].vertex_two.intersections[0])
        iz2 = IntegrationZone(u2, l2, p2, bouyant_density)

        other_vertices = findOtherVertices(surfaced_vertices, [surfaced_segments[0].vertex_one, surfaced_segments[0].vertex_two])
        if not determineIfAdditiveArea(surfaced_segments[0].vertex_one.location, surfaced_segments[0].vertex_two.location,
            surfaced_segments[0].vertex_one.intersections[0], surfaced_segments[0].vertex_two.intersections[0],
            other_vertices[0].location, other_vertices[1].location
        ):
            iz2.invertForceDirection()

        # side plane two
        l3, u3 = calculateBoundingPaths([
            [surfaced_segments[1].vertex_one.location[0], surfaced_segments[1].vertex_one.location[1], 0], 
            [surfaced_segments[1].vertex_two.location[0], surfaced_segments[1].vertex_two.location[1], 0],
            surfaced_segments[1].vertex_one.intersections[0],
            surfaced_segments[1].vertex_two.intersections[0]                
        ])
        p3 = pointsToPlane(surfaced_segments[1].vertex_one.location, surfaced_segments[1].vertex_one.intersections[0], surfaced_segments[1].vertex_two.intersections[0])
        iz3 = IntegrationZone(u3, l3, p3, bouyant_density)

        other_vertices = findOtherVertices(surfaced_vertices, [surfaced_segments[1].vertex_one, surfaced_segments[1].vertex_two])
        if not determineIfAdditiveArea(surfaced_segments[1].vertex_one.location, surfaced_segments[1].vertex_two.location,
            surfaced_segments[1].vertex_one.intersections[0], surfaced_segments[1].vertex_two.intersections[0],
            other_vertices[0].location, other_vertices[1].location
        ):
            iz3.invertForceDirection()

        # side plane three
        l4, u4 = calculateBoundingPaths([
            [surfaced_segments[2].vertex_one.location[0], surfaced_segments[2].vertex_one.location[1], 0], 
            [surfaced_segments[2].vertex_two.location[0], surfaced_segments[2].vertex_two.location[1], 0],
            surfaced_segments[2].vertex_one.intersections[0],
            surfaced_segments[2].vertex_two.intersections[0]                
        ])
        p4 = pointsToPlane(surfaced_segments[2].vertex_one.location, surfaced_segments[2].vertex_one.intersections[0], surfaced_segments[2].vertex_two.intersections[0])
        iz4 = IntegrationZone(u4, l4, p4, bouyant_density)

        other_vertices = findOtherVertices(surfaced_vertices, [surfaced_segments[2].vertex_one, surfaced_segments[2].vertex_two])
        if not determineIfAdditiveArea(surfaced_segments[2].vertex_one.location, surfaced_segments[2].vertex_two.location,
            surfaced_segments[2].vertex_one.intersections[0], surfaced_segments[2].vertex_two.intersections[0],
            other_vertices[0].location, other_vertices[1].location
        ):
            iz4.invertForceDirection()

        # side plane four
        l5, u5 = calculateBoundingPaths([
            [surfaced_segments[3].vertex_one.location[0], surfaced_segments[3].vertex_one.location[1], 0], 
            [surfaced_segments[3].vertex_two.location[0], surfaced_segments[3].vertex_two.location[1], 0],
            surfaced_segments[3].vertex_one.intersections[0],
            surfaced_segments[3].vertex_two.intersections[0]                
        ])
        p5 = pointsToPlane(surfaced_segments[3].vertex_one.location, surfaced_segments[3].vertex_one.intersections[0], surfaced_segments[3].vertex_two.intersections[0])
        iz5 = IntegrationZone(u5, l5, p5, bouyant_density)

        other_vertices = findOtherVertices(surfaced_vertices, [surfaced_segments[3].vertex_one, surfaced_segments[3].vertex_two])
        if not determineIfAdditiveArea(surfaced_segments[3].vertex_one.location, surfaced_segments[3].vertex_two.location,
            surfaced_segments[3].vertex_one.intersections[0], surfaced_segments[3].vertex_two.intersections[0],
            other_vertices[0].location, other_vertices[1].location
        ):
            iz5.invertForceDirection()

        delta_bouyant_force, delta_bouyancy_acting_location = sumBouyantForces(
            [iz1.total_force, iz2.total_force, iz3.total_force, iz4.total_force, iz5.total_force],
            [iz1.bouyant_force_acting_loaction, iz2.bouyant_force_acting_loaction, iz3.bouyant_force_acting_loaction, iz4.bouyant_force_acting_loaction, iz5.bouyant_force_acting_loaction]
        )

        if submerged:
            # flip the delta bouyant force because it represents a lack of bouyancy in this senario
            delta_bouyant_force = -delta_bouyant_force
            #calcuate the bouyant force of the entire box - bouyancy if entire box is submerged - delta
            return sumBouyantForces([bouyant_force, delta_bouyant_force], [cov, delta_bouyancy_acting_location])
        else:
            return [delta_bouyant_force, delta_bouyancy_acting_location]


    elif(len(surfaced_vertices) == 3):
        #This one is pretty simple .. 5 planes total -- one pentagon that is bounded by the surfaced vertices and two intersection points(that are coplanar with the surfaced vertices)
        # 2 quadralaterals and two triangles
        # the triangular planes are formed by the double_intersection vertices and their two intersections
        # the quadralateral planes are formed by the singular intersection vertex, its intersection, a double intersection vertex and one of it's intersections
        #The three quadralaterals a can be found by combining 2 surfaced vertice and intersection pairs together in all possible ways

        single_intersection_vertex = None
        double_intersection_vertices = []
        for vertex in surfaced_vertices:
            if(len(vertex.intersections) == 1):
                single_intersection_vertex = vertex
            else:
                double_intersection_vertices.append(vertex)

        #find vertices form double_intersection_vertices that are coplanar with the three surfaced vertices
        surfaced_plane = pointsToPlane(surfaced_vertices[0].location, surfaced_vertices[1].location, surfaced_vertices[2].location)

        # the indexes in these group corrospond the indexes in double_intersection vertices
        coplanar_intersections = []
        nonplanar_intersections = []
        for vertex in double_intersection_vertices:
            for intersection in vertex.intersections:
                if isCoplanar(surfaced_plane, intersection):
                    coplanar_intersections.append(intersection)
                else:
                    nonplanar_intersections.append(intersection)

        # pentagon
        l1, u1 = calculateBoundingPaths([
            coplanar_intersections[0], 
            coplanar_intersections[1], 
            [surfaced_vertices[0].location[0], surfaced_vertices[0].location[1], 0],
            [surfaced_vertices[1].location[0], surfaced_vertices[1].location[1], 0],
            [surfaced_vertices[2].location[0], surfaced_vertices[2].location[1], 0],
        ])
        iz1 = IntegrationZone(u1, l1, surfaced_plane, bouyant_density)

        #rectangle 1
        l2, u2 = calculateBoundingPaths([
            [double_intersection_vertices[0].location[0], double_intersection_vertices[0].location[1], 0], 
            [single_intersection_vertex.location[0], single_intersection_vertex.location[1], 0], 
            single_intersection_vertex.intersections[0],
            nonplanar_intersections[0]
        ])
        p2 = pointsToPlane(double_intersection_vertices[0].location, single_intersection_vertex.location, single_intersection_vertex.intersections[0])
        iz2 = IntegrationZone(u2, l2, p2, bouyant_density)

        #rectangle 2
        l3, u3 = calculateBoundingPaths([
            [double_intersection_vertices[1].location[0], double_intersection_vertices[1].location[1], 0], 
            [single_intersection_vertex.location[0], single_intersection_vertex.location[1], 0], 
            single_intersection_vertex.intersections[0],
            nonplanar_intersections[1]
        ])
        p3 = pointsToPlane(double_intersection_vertices[1].location, single_intersection_vertex.location, single_intersection_vertex.intersections[0])
        iz3 = IntegrationZone(u3, l3, p3, bouyant_density)

        #triangle 1
        l4, u4 = calculateBoundingPaths([
            [double_intersection_vertices[0].location[0], double_intersection_vertices[0].location[1], 0], 
            double_intersection_vertices[0].intersections[0], 
            double_intersection_vertices[0].intersections[1]
        ])
        p4 = pointsToPlane(double_intersection_vertices[0].location, double_intersection_vertices[0].intersections[0], double_intersection_vertices[0].intersections[1])
        iz4 = IntegrationZone(u4, l4, p4, bouyant_density)

        #triangle 2
        l5, u5 = calculateBoundingPaths([
            [double_intersection_vertices[1].location[0], double_intersection_vertices[1].location[1], 0], 
            double_intersection_vertices[1].intersections[0], 
            double_intersection_vertices[1].intersections[1]
        ])
        p5 = pointsToPlane(double_intersection_vertices[1].location, double_intersection_vertices[1].intersections[0], double_intersection_vertices[1].intersections[1])
        iz5 = IntegrationZone(u5, l5, p5, bouyant_density)

        delta_bouyant_force, delta_bouyancy_acting_location = sumBouyantForces(
            [iz1.total_force, iz2.total_force, iz3.total_force, iz4.total_force, iz5.total_force],
            [iz1.bouyant_force_acting_loaction, iz2.bouyant_force_acting_loaction, iz3.bouyant_force_acting_loaction, iz4.bouyant_force_acting_loaction, iz5.bouyant_force_acting_loaction]
        )
        
        if submerged:
            # flip the delta bouyant force because it represents a lack of bouyancy in this senario
            delta_bouyant_force = -delta_bouyant_force
            #calcuate the bouyant force of the entire box - bouyancy if entire box is submerged - delta
            return sumBouyantForces([bouyant_force, delta_bouyant_force], [cov, delta_bouyancy_acting_location])
        else:
            return [delta_bouyant_force, delta_bouyancy_acting_location]
            
    elif(len(surfaced_vertices) >= 2):
        #in this case there should be four intersections and four integration zones.
        # two zones will be rectangular and incorporate both surfaced vertices
        # two will be triangular and only incorporate 1 surfaced vertice
        location_zero = surfaced_vertices[0].location
        location_one = surfaced_vertices[1].location

        # make a list of all of the segments that are on the water plane
        unused_surface_segements = []

        counter = 0
        while counter < (len(upperSurfacePath) - 1):
            unused_surface_segements.append(LineSegment(upperSurfacePath[counter], upperSurfacePath[counter + 1]))
            counter += 1

        counter = 0
        while counter < (len(lowerSurfacePaths) - 1):
            unused_surface_segements.append(LineSegment(lowerSurfacePaths[counter], lowerSurfacePaths[counter + 1]))
            counter += 1

        # find the two zones that's bounding plane includes only one surfaced vertex
        l0_segment = LineSegment(surfaced_vertices[0].intersections[0], surfaced_vertices[0].intersections[1])
        for segment in unused_surface_segements:
            # this should only remove one
            if l0_segment.checkForDuplicate(segment):
                unused_surface_segements.remove(segment)
                break

        l1_segment = LineSegment(surfaced_vertices[1].intersections[0], surfaced_vertices[1].intersections[1])
        for segment in unused_surface_segements:
            # this should only remove one
            if l1_segment.checkForDuplicate(segment):
                unused_surface_segements.remove(segment)
                break

        # define paths and planes
        #u_ - upper bound, l_ - lower bound, p_ - plane
        l1, u1 = calculateBoundingPaths([[location_zero[0], location_zero[1], 0], surfaced_vertices[0].intersections[0], surfaced_vertices[0].intersections[1]])
        p1 = pointsToPlane(location_zero,  surfaced_vertices[0].intersections[0], surfaced_vertices[0].intersections[1])
        l2, u2 = calculateBoundingPaths([[location_one[0], location_one[1], 0], surfaced_vertices[1].intersections[0], surfaced_vertices[1].intersections[1]])
        p2 = pointsToPlane(location_one,  surfaced_vertices[1].intersections[0], surfaced_vertices[1].intersections[1])
        l3, u3 = calculateBoundingPaths([[location_zero[0], location_zero[1], 0], [location_one[0], location_one[1], 0], unused_surface_segements[0].end_point_one, unused_surface_segements[0].end_point_two])
        p3 = pointsToPlane(location_zero, location_one, unused_surface_segements[0].end_point_one)
        l4, u4 = calculateBoundingPaths([[location_zero[0], location_zero[1], 0], [location_one[0], location_one[1], 0], unused_surface_segements[1].end_point_one, unused_surface_segements[1].end_point_two])
        p4 = pointsToPlane(location_zero, location_one, unused_surface_segements[1].end_point_one)

        # define integration zones
        iz1 = IntegrationZone(u1, l1, p1, bouyant_density)
        iz2 = IntegrationZone(u2, l2, p2, bouyant_density)
        iz3 = IntegrationZone(u3, l3, p3, bouyant_density)
        iz4 = IntegrationZone(u4, l4, p4, bouyant_density)

        delta_bouyant_force, delta_bouyancy_acting_location = sumBouyantForces(
            [iz1.total_force, iz2.total_force, iz3.total_force, iz4.total_force],
            [iz1.bouyant_force_acting_loaction, iz2.bouyant_force_acting_loaction, iz3.bouyant_force_acting_loaction, iz4.bouyant_force_acting_loaction]
        )

        if submerged:
            # flip the delta bouyant force because it represents a lack of bouyancy in this senario
            delta_bouyant_force = -delta_bouyant_force
            #calcuate the bouyant force of the entire box - bouyancy if entire box is submerged - delta
            return sumBouyantForces([bouyant_force, delta_bouyant_force], [cov, delta_bouyancy_acting_location])
        else:
            return [delta_bouyant_force, delta_bouyancy_acting_location]


    else:
        #only one vertxex above the water, this means there are three planes - each formed by two intersections and the surfaced vertex

        #four involved points
        surfaced_point_1, surfaced_point_2, surfaced_point_3 = surfaced_vertices[0].intersections
        center_vertex = surfaced_vertices[0].location

        #calculate the upper bounding path, lower bounding path and plane for each of the three planes
        #u_ - upper bound, l_ - lower bound, p_ - plane
        l1, u1 = calculateBoundingPaths([[center_vertex[0], center_vertex[1], 0], surfaced_point_1, surfaced_point_2])
        p1 = pointsToPlane(center_vertex, surfaced_point_1, surfaced_point_2)
        l2, u2 = calculateBoundingPaths([[center_vertex[0], center_vertex[1], 0], surfaced_point_1, surfaced_point_3])
        p2 = pointsToPlane(center_vertex, surfaced_point_1, surfaced_point_3)
        l3, u3 = calculateBoundingPaths([[center_vertex[0], center_vertex[1], 0], surfaced_point_3, surfaced_point_2])
        p3 = pointsToPlane(center_vertex, surfaced_point_3, surfaced_point_2)
    
        # define and integrate all zones
        iz1 = IntegrationZone(u1, l1, p1, bouyant_density)
        iz2 = IntegrationZone(u2, l2, p2, bouyant_density)
        iz3 = IntegrationZone(u3, l3, p3, bouyant_density)

        delta_bouyant_force, delta_bouyancy_acting_location = sumBouyantForces(
            [iz1.total_force, iz2.total_force, iz3.total_force],
            [iz1.bouyant_force_acting_loaction, iz2.bouyant_force_acting_loaction, iz3.bouyant_force_acting_loaction]
        )

        if submerged:
            # flip the delta bouyant force because it represents a lack of bouyancy in this senario
            delta_bouyant_force = -delta_bouyant_force
            #calcuate the bouyant force of the entire box - bouyancy if entire box is submerged - delta
            return sumBouyantForces([bouyant_force, delta_bouyant_force], [cov, delta_bouyancy_acting_location])
        else:
            return [delta_bouyant_force, delta_bouyancy_acting_location]

b = calculateBouyancy(raw_vertices, euler_rotation, position, bouyant_density, bouyant_force)
print(f"Bouyancy is {b}!")
print(time.time())



