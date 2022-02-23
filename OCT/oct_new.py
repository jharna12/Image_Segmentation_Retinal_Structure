# ImageFilter for using filter() function
from PIL import Image, ImageFilter
from numpy import asarray
import numpy
from matplotlib import pyplot as plt
import sys
import glob
import time
import os

# Python program for Dijkstra's single
# source shortest path algorithm. The program is
# for adjacency matrix representation of the graph


class Graph():

	def __init__(self, vertices):
		self.V = vertices
		self.graph = [[0 for column in range(vertices)]
					for row in range(vertices)]

	def printSolution(self, dist):
		print ("Vertex \t Distance from Source")
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
		file = open('defected_matrix.txt','a+')
		content= str(max(dist))
		file.writelines(content)
		file.write('\n')
		file.close()

def main():
    try:
        os.remove('defected_matrix.txt')
    except:
        pass
    try:
        print("*******************Execution Started**********************")
        time.sleep(3)
        sr = int(input('Enter the position you want to apply the cut: '))
        im=0
        for filename in glob.glob(r'C:\Users\jagrawal\Desktop\image_segmentation\crazy-programmers\OCT\defected/*.jpeg'):
            try:
                image=Image.open(filename)
                # Opening the image
                # (R prefixed to string in order to deal with '\' in paths)
                #image = Image.open(r"2.jpeg")
                
                #Displaying the original image
                # Displaying the image
                #image.show()
                
                # Blurring image by sending the ImageFilter.
                # GaussianBlur predefined kernel argument
                image = image.filter(ImageFilter.GaussianBlur)
                
                # Displaying the image
                #image.show()
                
                # Size of the image in pixels (size of original image)
                # (This is not mandatory)
                width, height = image.size
                 
                # Setting the points for cropped image
                left = 6
                top = height / 4
                right = 174
                bottom = 3 * height / 4
                 
                # Cropped image of above dimension
                # (It will not change original image)
                image = image.crop((left, top, right, bottom))
                newsize = (200, 200)
                image = image.resize(newsize)
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
                
                c =im
                f = r'C:\Users\jagrawal\Desktop\image_segmentation\crazy-programmers\OCT\processed\image_'+str(c)+'.pdf'
                #print(numpydata_after_n)
                plt.imshow(t, interpolation='nearest')
                plt.savefig(f)
                im=im+1
                #plt.show()
                
                # Driver program
                g = Graph(wmx)
                g.graph = t
                g.dijkstra(sr);
            except:
                print('')
        time.sleep(3)
        print("******************Execution Finished************************")
    except:
        print('')

main()

