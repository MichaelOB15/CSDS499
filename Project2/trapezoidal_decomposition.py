
class TD():

    def __init__(self, boundary, verticies, start, end):

        # boundary = [[x1,y1],[x2, y2],...,[xn,yn]]
        self.boundary = boundary
        self.verticies = verticies
        self.start = start
        self.end = end

    def calculate_nodes(self):

        all_criticle_points = []
        all_rays = []

        last_point = self.boundary[-1]
        for point in self.boundary:
            all_criticle_points.append(point)

            all_rays.append([last_point, point])
            last_point = point

        for objects in self.verticies:

            last_point = objects[-1]
            for points in objects:
                all_criticle_points.append(points)

                all_rays.append([last_point, point])
                last_point = point

        vert_boundaries = []

        for CP in all_criticle_points:
            offset = .001
            valid_increase = self.valid_points([CP[0], CP[1]+offset], all_rays)
            valid_decrease = self.valid_points([CP[0], CP[1]-offset], all_rays)

            if valid_decrease or valid_increase:
                closest_top_y = 99999999999
                closest_top_point = [0, 0]
                closest_bottom_y = 99999999999
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
                        if valid_increase:
                            point_on_ray = self.intersecting_point(ray, CP)
                            if (point_on_ray[1] - CP[1]) < closest_top_y:
                                closest_top_y = point_on_ray[1] - CP[1]
                                closest_top_point = point_on_ray
                        if valid_decrease:
                            point_on_ray = self.intersecting_point(ray, CP)   
                            if (CP[1] - point_on_ray[1]) < closest_bottom_y:
                                closest_bottom_y = CP[1] - point_on_ray[1]
                                closest_bottom_point = point_on_ray 

                if closest_top_point != [0, 0]:
                    vert_boundaries.append([CP, closest_top_point])
                if closest_top_point != [0, 0]:
                    vert_boundaries.append([closest_bottom_point, CP])

        return vert_boundaries

    def intersecting_point(self, ray, point):
        # pass to this function valid point and ray combinations
        # ray = [[x1,y1],[x2,y2]]
        # point = [x1,y1]

        slope = (ray[1][1]-ray[0][1]) / (ray[1][0]-ray[0][0])

        new_y = ray[0][1] + (point[0] - ray[0, 0]) * slope
        return [point[0], new_y]

    def valid_point(self, point, rays):
        num_intersections = 0
        for ray in all_rays:
            if ray[0][0] < ray[1][0]:
                xmin = ray[0][0]
                xmax = ray[1][0]
            else:
                xmin = ray[1][0]
                xmax = ray[0][0]

            point_on_ray = self.intersecting_point(ray, point)

            # in the boundary and above
            if point[0] < xmax and point[0] > xmin and point_on_ray[1] > point[1]:
                num_intersections += 1

        # if even num of interactions
        if num_intersections % 2:
            return False
        else:
            return True


def test():
    start = []
    end = []
    boundary = []
    vertecies = []
    decom = TD()
