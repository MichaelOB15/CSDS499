class brushfire():

    def __init__(self, boundary, vertices):

        self.boundary = boundary
        self.vertices = vertices

        self.map = self.generate_map()

    def generate_map(self):
        #make a 2d numpy array that's a map of the space
        #put a 1 where the object is and a zero everywhere else
        return None

    def brushfireAlg(self):
        #expand the map from generate_map so each pixel holds the distance to the nearest object






        #go in a circle around the objects and fill in neighbors with 2s
        #fill in neighbor cells of the 2s with 3s etc
        pass

    def wavefront(self):
        #get the map from generate_map and label the goal as a 2
        #do the same thing we did for the brushfire part where every region is updated in a shell
        #find distance around obstacles to the goal pixel
        pass

    def run(self):
        #sum the wavefront and brushfire
        #find the steepest decline in numbers but stay away from 1s because those are obstacles
        pass