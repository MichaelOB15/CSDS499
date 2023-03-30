import math

class LMKC():
    def __init__(self, z_t, x_t, m, sigma_r, sigma_theta):
        z_t[1] = z_t[1] % 2*math.pi
        self.z_t = z_t
        self.x_t = x_t
        self.m = m
        self.sigma_r = sigma_r
        self.sigma_theta = sigma_theta

    def landmark_model_known_correspondence(self):
        r_hat = math.sqrt((self.m[0]-self.x_t[0])**2+(self.m[1]-self.x_t[1])**2)
        theta_hat = math.atan2(self.m[1]-self.x_t[1], self.m[0]-self.x_t[0]) - self.x_t[2]
        prob1 = self.prob_normal_distribution(self.z_t[0]-r_hat,self.sigma_r)
        prob2 = self.prob_normal_distribution(self.z_t[1]-theta_hat, self.sigma_theta)
        q = prob1 * prob2
        return q

    def prob_normal_distribution(self, mean, sigma):
        num = math.exp(-mean**2/(2*sigma))
        denom = math.sqrt(2*math.pi*sigma)
        prob =  num/denom
        return prob

'''
z_t = [2.276,5.249]
x_t = [3,2,0]
m = [4,0]
sigma_r = .1
sigma_theta = .09
correspondance = LMKC(z_t, x_t, m, sigma_r, sigma_theta)
print(correspondance.landmark_model_known_correspondance())
'''