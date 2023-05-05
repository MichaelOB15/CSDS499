
class TD():

    def __init__(self, boundary, verticies, start, end):
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
            small_increase = # need to add small amount of increase to test if a valid point is there
            valid_increase = True # Run gabes code here to test for a valid point

            small_decrease = # need to subtract small amount of increase to test if a valid point is there
            valid_decrease = True # Run gabes code here to test for a valid point

            if valid_decrease or valid_increase:
                closest_top_y = 99999999999
                closest_top_point = [0,0]
                closest_bottom_y = 99999999999
                closest_bottom_point = [0,0]

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
                            if (point_on_ray[1] - CP[1]) < closest_top_y
                                closest_top_y = point_on_ray[1] - CP[1]
                                closest_top_point = point_on_ray
                        if valid_decrease:
                            point_on_ray = self.intersecting_point(ray, CP)   
                            if (CP[1] - point_on_ray[1]) < closest_bottom_y
                                closest_bottom_y = CP[1] - point_on_ray[1]
                                closest_bottom_point = point_on_ray 

                if closest_top_point not [0,0]:
                    vert_boundaries.append([CP, closest_top_point])
                if closest_top_point not [0,0]:
                    vert_boundaries.append([closest_bottom_point, CP])

        return vert_boundaries

    def intersecting_point(self, ray, point):
        # pass to this function valid point and ray combinations
        # ray = [[x1,y1],[x2,y2]]
        # point = [x1,y1]

        slope = (ray[1][1]-ray[0][1]) / (ray[1][0]-ray[0][0])

        new_y = ray[0][1] + (point[0] - ray[0,0]) * slope 

        return [point[0], new_y]
