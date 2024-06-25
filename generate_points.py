import numpy as np

def generate_points(n, xmin, xmax, ymin, ymax, filename):
    # Generate x and y uniformly distributed
    x = np.random.uniform(xmin, xmax, n)
    y = np.random.uniform(ymin, ymax, n)
    
    # Generate z normally distributed (mean=0, std=1 by default)
    z = np.random.normal(size=n, scale= 0.1)
    
    # Normal vector is fixed at (0, 0, 1)
    nx = np.zeros(n)
    ny = np.zeros(n)
    nz = np.ones(n)
    
    # Write to file
    with open(filename, "w") as file:
        for xi, yi, zi, nxi, nyi, nzi in zip(x, y, z, nx, ny, nz):
            file.write(f"{xi} {yi} {zi} {nxi} {nyi} {nzi}\n")

# Example usage
n = 100000  # number of points
xmin, xmax = -100, 100
ymin, ymax = -100, 100
filename = "points.txt"
generate_points(n, xmin, xmax, ymin, ymax, filename)
