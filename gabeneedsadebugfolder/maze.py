#maze.py
import random
import numpy as np

class Cell:
    """A cell in the maze.
    A maze "Cell" is a point in the grid which may be surrounded by walls to
    the north, east, south or west.
    """

    # A wall separates a pair of cells in the N-S or W-E directions.
    wall_pairs = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}

    def __init__(self, x, y):
        """Initialize the cell at (x,y). At first it is surrounded by walls."""

        self.x, self.y = x, y
        self.walls = {'N': True, 'S': True, 'E': True, 'W': True}

    def has_all_walls(self):
        """Does this cell still have all its walls?"""

        return all(self.walls.values())

    def knock_down_wall(self, other, wall):
        """Knock down the wall between cells self and other."""

        self.walls[wall] = False
        other.walls[Cell.wall_pairs[wall]] = False


class Maze:
    """A Maze, represented as a grid of cells."""

    def __init__(self, nx, ny, scaling, ix=0, iy=0):
        """Initialize the maze grid.
        The maze consists of nx x ny cells and will be constructed starting
        at the cell indexed at (ix, iy).
        """
        self.nx, self.ny = nx, ny
        self.ix, self.iy = ix, iy

        self.scaling=scaling

        self.maze_map = [[Cell(x, y) for y in range(ny)] for x in range(nx)]

    def cell_at(self, x, y):
        """Return the Cell object at (x,y)."""

        return self.maze_map[x][y]

    def out(self):
        """Return a matrix representation of the maze."""

        xdim,ydim = self.nx*2+1, self.ny*2+1

        scaling=self.scaling

        m=np.zeros((xdim,ydim))

        for x in range(self.nx):
            for y in range(self.ny):

            # Draw the "South" and "East" walls of each cell, if present (these
            # are the "North" and "West" walls of a neighbouring cell)

                xindex=x*2+1
                yindex=y*2+1

                if self.cell_at(x, y).walls['W']:
                    m[xindex-1,yindex-1]=1
                    m[xindex-1,yindex]=1
                    m[xindex-1,yindex+1]=1
                if self.cell_at(x, y).walls['E']:
                    m[xindex+1,yindex-1]=1
                    m[xindex+1,yindex]=1
                    m[xindex+1,yindex+1]=1

                # N/S directions do not match but this returns the proper result

                if self.cell_at(x, y).walls['N']:
                    m[xindex-1,yindex-1]=1
                    m[xindex,yindex-1]=1
                    m[xindex+1,yindex-1]=1    
                if self.cell_at(x, y).walls['S']:
                    m[xindex-1,yindex+1]=1
                    m[xindex,yindex+1]=1
                    m[xindex+1,yindex+1]=1

        xdim=xdim*scaling
        ydim=np.shape(m)[1]*scaling

        out=np.zeros((xdim,ydim))

        for x in range(np.shape(m)[0]):
            for y in range(np.shape(m)[1]):

                val=m[x,y]

                for a in range(scaling):
                    for b in range(scaling):
                        out[x*scaling+a,y*scaling+b]=val

        return np.transpose(out)

    def find_valid_neighbours(self, cell):
        """Return a list of unvisited neighbours to cell."""

        delta = [('W', (-1, 0)),
                 ('E', (1, 0)),
                 ('S', (0, 1)),
                 ('N', (0, -1))]
        neighbours = []
        for direction, (dx, dy) in delta:
            x2, y2 = cell.x + dx, cell.y + dy
            if (0 <= x2 < self.nx) and (0 <= y2 < self.ny):
                neighbour = self.cell_at(x2, y2)
                if neighbour.has_all_walls():
                    neighbours.append((direction, neighbour))
        return neighbours

    def make_maze(self):
        # Total number of cells.
        n = self.nx * self.ny
        cell_stack = []
        current_cell = self.cell_at(self.ix, self.iy)
        # Total number of visited cells during maze construction.
        nv = 1

        while nv < n:
            neighbours = self.find_valid_neighbours(current_cell)

            if not neighbours:
                # We've reached a dead end: backtrack.
                current_cell = cell_stack.pop()
                continue

            # Choose a random neighbouring cell and move to it.
            direction, next_cell = random.choice(neighbours)
            current_cell.knock_down_wall(next_cell, direction)
            cell_stack.append(current_cell)
            current_cell = next_cell
            nv += 1