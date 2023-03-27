import math

class Ellipse:
    def __init__(self, error_matrix):
        self.error_matrix = error_matrix
    
    def error_to_ellipse(self):

        # radius of x 
        lam_1 = math.sqrt(((self.error_matrix[1,1]+self.error_matrix[0,0])/2)+(math.sqrt(((self.error_matrix[0,0]-self.error_matrix[1,1])/2)**2+(self.error_matrix[0,1]**2))))

        # radius of y
        lam_2 = math.sqrt(((self.error_matrix[1,1]+self.error_matrix[0,0])/2)-(math.sqrt(((self.error_matrix[0,0]-self.error_matrix[1,1])/2)**2+(self.error_matrix[0,1]**2))))

        # rotation angle
        if self.error_matrix[0,1] == 0 and (self.error_matrix[0,0] >= self.error_matrix[1,1]):
            theta = 0

        elif self.error_matrix[0,1] == 0 and (self.error_matrix[0,0] < self.error_matrix[1,1]):
            theta = math.pi/2

        else:
            theta = math.atan2(lam_1**2-self.error_matrix[0,0],self.error_matrix[1,1])

        return lam_1, lam_2, theta

