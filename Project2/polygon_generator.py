from polygenerator import random_polygon, random_star_shaped_polygon,random_convex_polygon


# these two are only for demonstration
import matplotlib.pyplot as plt
import random


def plot_polygon(polygon, scaling, xshift, yshift):
    
    # so it is plotted as closed polygon
    polygon.append(polygon[0])

    xs, ys = zip(*polygon)
    x=[]
    y=[]

    for i in range(len(xs)):
        x.append(xs[i]*scaling+xshift)
    for i in range(len(ys)):
        y.append(ys[i]*scaling+yshift)

    plt.plot(x, y, "k-", linewidth=1)


# this is just so that you can reproduce the same results
random.seed(3)

plt.figure()  #plt.savefig(out_file_name, dpi=300)
plt.gca().set_aspect("equal")
plt.xlim([-10,10])
plt.ylim([-10,10])

polygon = random_convex_polygon(num_points=7) #outer boundary
polygon2=random_star_shaped_polygon(num_points=12) #inner obstacles
polygon3=random_star_shaped_polygon(num_points=12)
polygon4=random_star_shaped_polygon(num_points=12)

scaling=[20,5,5,5] #I did these by hand these are just values that look nice
xshift=[-10,0,-8,2]
yshift=[-10,0,0,-8]
pg=[polygon,polygon2,polygon3,polygon4]

for i in range(len(scaling)):
    plot_polygon(pg[i],scaling[i],xshift[i],yshift[i])

plt.show()


#print(polygon[0][1])