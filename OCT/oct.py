# ImageFilter for using filter() function
from PIL import Image, ImageFilter
from numpy import asarray
import numpy
from matplotlib import pyplot as plt
import sys

# Python program for Dijkstra's single
# source shortest path algorithm. The program is
# for adjacency matrix representation of the graph


class Graph():

	def __init__(self, vertices):
		self.V = vertices
		self.graph = [[0 for column in range(vertices)]
					for row in range(vertices)]

	def printSolution(self, dist):
		print ("Vertex \tDistance from Source")
		for node in range(self.V):
			print(node, "\t", dist[node])

	# A utility function to find the vertex with
	# minimum distance value, from the set of vertices
	# not yet included in shortest path tree
	def minDistance(self, dist, sptSet):

		# Initialize minimum distance for next node
		min = sys.maxsize

		# Search not nearest vertex not in the
		# shortest path tree
		for u in range(self.V):
			if dist[u] < min and sptSet[u] == False:
				min = dist[u]
				min_index = u

		return min_index

	# Function that implements Dijkstra's single source
	# shortest path algorithm for a graph represented
	# using adjacency matrix representation
	def dijkstra(self, src):

		dist = [sys.maxsize] * self.V
		dist[src] = 0
		sptSet = [False] * self.V

		for cout in range(self.V):

			# Pick the minimum distance vertex from
			# the set of vertices not yet processed.
			# x is always equal to src in first iteration
			x = self.minDistance(dist, sptSet)

			# Put the minimum distance vertex in the
			# shortest path tree
			sptSet[x] = True

			# Update dist value of the adjacent vertices
			# of the picked vertex only if the current
			# distance is greater than new distance and
			# the vertex in not in the shortest path tree
			for y in range(self.V):
				if self.graph[x][y] > 0 and sptSet[y] == False and \
				dist[y] > dist[x] + self.graph[x][y]:
						dist[y] = dist[x] + self.graph[x][y]

		self.printSolution(dist)

def main():
    # Opening the image
    # (R prefixed to string in order to deal with '\' in paths)
    image = Image.open(r"1.jpeg")
    
    #Displaying the original image
    # Displaying the image
    image.show()
    
    # Blurring image by sending the ImageFilter.
    # GaussianBlur predefined kernel argument
    image = image.filter(ImageFilter.GaussianBlur)
    
    # Displaying the image
    image.show()
    
    numpydata_after = asarray(image)
    
    numpydata_after_old = numpydata_after #copying the old matrix for future reference
    
    numpydata_after_horiz = numpydata_after_old
    
    numpydata_after_vert = numpydata_after_old
    
    #getting the number of rows and columns of a matrix
    
    row,col = numpydata_after.shape
    
    #horizontal gradient
    
    m=[]
    for i in range(0,row):
        l=[]
        for j in range(0,col):
            l.append(numpydata_after[i][j])
        m.append(l)
    
    for i in range(0,row):
        numpydata_after_horiz[i] = numpydata_after[i]-min(m[i])
        
    #Vertical Gradient
    m_v=[]
    for i in range(0,row):
        l=[]
        for j in range(0,col):
             l.append(numpydata_after[j][i])
        m_v.append(l)
    for i in range(0,row):
        for j in range(0,col):
            numpydata_after_vert[j][i] = numpydata_after[j][i] - min(m_v[i])
    
    wmn = numpy.min(numpydata_after_old)
    wmx = numpy.max(numpydata_after_old)
    
    numpydata_after_n = numpydata_after-(numpydata_after_horiz+numpydata_after_vert)+1
    
    w = ((numpydata_after_n-wmn)/(wmx-wmn))
    
    t = w[:][:]
    
    for i in range(0,row):
        for j in range(0,col):
            t[i][j] = w[j][i]
    
    #print(numpydata_after_n)
    plt.imshow(t, interpolation='nearest')
    plt.show()
    
    # Driver program
    g = Graph(wmx)
    g.graph = t
    g.dijkstra(100);


main()


