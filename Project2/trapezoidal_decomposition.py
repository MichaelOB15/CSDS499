from graph import Graph
from point import Point
import matplotlib.pyplot as plt
from typing import List


class TD():

    def __init__(self, boundary, obstacles, start, end):

        # boundary = [[x1,y1],[x2, y2],...,[xn,yn]]
        self.boundary = boundary
        self.obstacles = obstacles
        self.start = start
        self.end = end
        self.all_rays = []
        self.all_critical_points = []
        self.offset = .0001
        self.midpoints = []

    def calculate_nodes(self):

        all_critical_points = []
        all_rays = []

        last_point = self.boundary[-1]
        for point in self.boundary:
            all_critical_points.append(point)

            all_rays.append([last_point, point])
            last_point = point

        for obstacle in self.obstacles:

            last_point = obstacle[-1]
            for point in obstacle:
                all_critical_points.append(point)

                all_rays.append([last_point, point])
                last_point = point

        self.all_rays = all_rays
        self.all_critical_points = all_critical_points

        vert_boundaries = []

        for CP in all_critical_points:
            valid_increase = self.valid_point([CP[0], CP[1]+self.offset], all_rays)
            valid_decrease = self.valid_point([CP[0], CP[1]-self.offset], all_rays)

            if valid_decrease or valid_increase:
                closest_top_y = float('inf')
                closest_top_point = [0, 0]
                closest_bottom_y = float('inf')
                closest_bottom_point = [0, 0]

                for ray in all_rays:
                    if ray[0][0] < ray[1][0]:
                        xmin = ray[0][0]
                        xmax = ray[1][0]
                    else:
                        xmin = ray[1][0]
                        xmax = ray[0][0]

                    # if point falls within the ray
                    if CP[0] > xmin and CP[0] < xmax:
                        #print("cp:", CP,"in ray:", ray)
                        try:
                            if valid_increase:
                                point_on_ray = self.intersecting_point(ray, CP)
                                if (point_on_ray[1] > CP[1]) and ((point_on_ray[1] - CP[1]) < closest_top_y):
                                    closest_top_y = point_on_ray[1] - CP[1]
                                    closest_top_point = point_on_ray
                            if valid_decrease:
                                point_on_ray = self.intersecting_point(ray, CP)
                                if (point_on_ray[1] < CP[1]) and ((CP[1] - point_on_ray[1]) < closest_bottom_y):
                                    closest_bottom_y = CP[1] - point_on_ray[1]
                                    closest_bottom_point = point_on_ray
                        except:
                            pass

                if closest_top_point != [0, 0]:
                    vert_boundaries.append([CP, closest_top_point])
                if closest_bottom_point != [0, 0]:
                    vert_boundaries.append([closest_bottom_point, CP])

        vert_boundaries.sort()

        self.vert_boundaries = vert_boundaries

        midpoints = self.generate_midpoints(vert_boundaries)

        self.midpoints = midpoints

        midpoints.sort()

        graph = self.generate_graph(midpoints)

        return graph

    def intersect_any_obstacle(self, A, B,):
        for obstacle in self.obstacles:
            for i in range(len(obstacle) - 2):
                p3 = Point(obstacle[i][0], obstacle[i][1])
                p4 = Point(obstacle[i + 1][0], obstacle[i + 1][1])
                if intersect(A, B, p3, p4):
                    return True

            p3 = Point(obstacle[-1][0], obstacle[-1][1])
            p4 = Point(obstacle[0][0], obstacle[0][1])
            if intersect(A, B, p3, p4):
                return True

        return False

    def generate_graph(self, midpoints):
        g = Graph()

        midpoints.append(self.start)
        midpoints.append(self.end)

        midpoints.sort()

        for point in midpoints:
            g.add_node(point[0], point[1])

        for point1 in midpoints:
            connection = True
            point_check_x = float('inf')

            for point2 in midpoints:
                if point2[0] > point1[0] and connection == True:
                    if self.valid_path(point1, point2):
                        g.add_vertex(Point(point1[0], point1[1]), Point(point2[0], point2[1]))
                        point_check_x = point2[0]
                        connection = False

                if point2[0] == point_check_x:
                    if self.valid_path(point1, point2):
                        g.add_vertex(Point(point1[0], point1[1]), Point(point2[0], point2[1]))

        return g

    def valid_path(self, point1, point2):

        rise = point2[1] - point1[1]
        run = point2[0] - point1[0]

        valid_point = True

        for i in range(0,100):
            new_point = [point1[0] + run*(i/100), point1[1] + rise*(i/100)] 
            if self.valid_point(new_point, self.all_rays) == False:
                valid_point = False
                break

        if valid_point:
            return True
        else:
            return False


    def generate_midpoints(self, vert_boundaries):
        midpoints = []
        for line in vert_boundaries:
            midpoints.append([line[0][0], (line[0][1]+line[1][1])/2])
        return midpoints

    def intersecting_point(self, ray, point):
        # pass to this function valid point and ray combinations
        # ray = [[x1,y1],[x2,y2]]
        # point = [x1,y1]

        if ray[0][0] < ray[1][0]:
            first = ray[0]
            second = ray[1]
        else:
            first = ray[1]
            second = ray[0]

        m = (second[1]-first[1]) / (second[0]-first[0])

        b = first[1] - (m * first[0])

        new_y = (m * point[0]) + b

        new_point = [point[0], new_y]

        return new_point

    def valid_point(self, point, rays):
        '''determine whether a point is in an obstacle'''
        try:
            num_intersections = 0

            rays_intersected = []

            for ray in rays:
                if ray[0][0] < ray[1][0]:
                    xmin = ray[0][0]
                    xmax = ray[1][0]
                else:
                    xmin = ray[1][0]
                    xmax = ray[0][0]

                point_on_ray = self.intersecting_point(ray, point)

                # in the boundary and above
                if point[0] < xmax and point[0] >= xmin and point_on_ray[1] > point[1]:
                    rays_intersected.append(ray)
                    num_intersections += 1

            # if odd num of interactions
            if num_intersections % 2 == 1:
                #print("Return: True,  num intersections: ",num_intersections," point: ", point, "rays intersected: ", rays_intersected)
                return True
            else:
                #print("Return: False,  num intersections: ",num_intersections," point: ", point, "rays intersected: ", rays_intersected)
                return False
        except:
            return False

    def vis(self):
        plt.title("Trapezoidal Decomposition")

        # For Boundary
        x: List[float] = []
        y: List[float] = []
        for point in self.boundary:
            x.append(point[0])
            y.append(point[1])

        x.append(self.boundary[0][0])
        y.append(self.boundary[0][1])
        plt.plot(x, y, color="grey")

        # For Obstacles
        xob: List[float] = []
        yob: List[float] = []
        for obstacle in self.obstacles:

            for point in obstacle:
                xob.append(point[0])
                yob.append(point[1])
            xob.append(obstacle[0][0])
            yob.append(obstacle[0][1])
            plt.plot(xob, yob, color="black")
            xob = []
            yob = []

        for line in self.vert_boundaries:
            x = [line[0][0], line[1][0]]
            y = [line[0][1], line[1][1]]
            plt.plot(x, y, color="orange")

        for CP in self.midpoints:
            plt.scatter(CP[0], CP[1], color="blue")

        '''
        for ray in self.all_rays:
            x = [ray[0][0], ray[1][0]]
            y = [ray[0][1], ray[1][1]]
            plt.plot(x, y, color="blue")

        for CP in self.all_critical_points:
            plt.scatter(CP[0], CP[1], color="red")'''

        #plt.show()
