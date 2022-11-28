from scipy.spatial.transform import Rotation
import numpy as np

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
                
                
        return intersections



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
euler_rotation = [0, 0, 0] #in degrees for time being
position = [0,0,-.05] #x,y,z COB

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
            surface_intersections.append(vertex.returnAdjacentIntersections())

        if len(surface_intersections) < 3:
            print(f"This should not happen, it takes three vertices to define a plane but there are only two {len(surface_intersections)}!")
            return

        #calcuate the two bounds of the n-gon created by the surface_intersection points
        ordered_intersections = []
        for point in surface_intersections:
            ordered_intersections.append(point[0])

        def return_first(val):
            return val[0]

        # sort the points by x coordinate
        ordered_intersections.sort(key=return_first)

        #initial point for both bounds is the same - so is the final
        lower_bound = [ordered_intersections[0]]
        upper_bound = lower_bound.copy()

        #once the number of points in the two bounds is equal to the total number of points, add the final point 
        #   initial point is double counted but the final point isn't
        counter = 1
        while (len(lower_bound) + len(upper_bound)) != len(surface_intersections):

            midline_slope = (ordered_intersections[len(ordered_intersections) - 1][1] - ordered_intersections[0][1]) / (ordered_intersections[len(ordered_intersections) - 1][0] - ordered_intersections[0][0])

            #points above midline go in upper bound below go in
            if(ordered_intersections[counter][1] > ordered_intersections[0][1] + midline_slope *(ordered_intersections[counter][0] - ordered_intersections[0][0])):
                upper_bound.append(ordered_intersections[counter])
            else: 
                lower_bound.append(ordered_intersections[counter])

            counter += 1

        # finish out bound path
        upper_bound.append(ordered_intersections[len(ordered_intersections) - 1])
        lower_bound.append(ordered_intersections[len(ordered_intersections) - 1])

        


        


b = calculateBouyancy(raw_vertices, euler_rotation, position, bouyant_density, bouyant_force)
print(f"Bouyancy is {b}!")


