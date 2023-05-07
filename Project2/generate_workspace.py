from polygenerator import random_polygon, random_star_shaped_polygon,random_convex_polygon


# these two are only for demonstration
import matplotlib.pyplot as plt
import random


class Workspace():

    def plot_polygon(self, polygon):

        # so it is plotted as closed polygon
        polygon.append(polygon[0])
        xs, ys = zip(*polygon)

        plt.plot(xs, ys, "k-", linewidth=1)

    def reformat(self, polygon, scaling, xshift, yshift):
        xs, ys = zip(*polygon)
        x=[]
        y=[]

        for i in range(len(xs)):
            x.append(xs[i]*scaling+xshift)
        for i in range(len(ys)):
            y.append(ys[i]*scaling+yshift)

        polygon2=[]
        for i in range(len(x)):
            polygon2.append([x[i],y[i]])

        return polygon2

    def gen(self):
        random.seed(3) # this is to reproduce the same results every time

        #set up shapes
        polygon=random_convex_polygon(num_points=7) #outer boundary
        polygon2=random_star_shaped_polygon(num_points=12) #inner obstacles
        polygon3=random_star_shaped_polygon(num_points=12)
        polygon4=random_star_shaped_polygon(num_points=12)

        scaling=[20,5,5,5] #I did these by hand these are just values that look nice
        xshift=[-10,0,-8,2]
        yshift=[-10,0,0,-8]
        pg=[polygon,polygon2,polygon3,polygon4]

        output=[]
        for i in range(len(scaling)):
            output.append(self.reformat(pg[i],scaling[i],xshift[i],yshift[i]))
        
        boundary=output[0] #this seems like bad practice but I want it in a format that fits well with the other code
        obstacles=[output[1],output[2],output[3]]
        start=[5,5]
        end=[-5,-5]

        return[boundary,obstacles,start,end]
            


        