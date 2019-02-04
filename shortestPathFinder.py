#
#    File: shortestPathFinder.py
# 
#    Description: Find the shortest path between two given points in the matrix
#
#    Input: M x N matrix, start and end coordinates
#    Output: Returns the list of coordinates indicating the shortest path
#

import os
import time
import argparse
from colorama import Fore, Style

class MatrixNode():
    """ MatrixNode class representing the cells in matrix """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

class MatrixPath():
    """ MatrixPath class computes the shortest path between two cells in matrix """

    def __init__(self, input_matrix):
        self.input_matrix = input_matrix
        self.allpaths = []
        self.debug_flag = False

    def debug_print(self, x):
        if self.debug_flag == True:
            print(x)
    
    def debug_mode(self):
        if self.debug_flag == True:
            return(True)
        else:
            return(False)

    def debug_set(self, flag):
        self.debug_flag = flag
            
    def print_matrix(self, matrix_data, path):
        """ Highlight the shortest path in the matrix """

        rows = len(matrix_data)
        columns = len(matrix_data[0])
        marker = 'X'
        
        for node in path:
            matrix_data[node[0]][node[1]] = marker

        for i in range (rows):
            for j in range (columns):
                if matrix_data[i][j] == marker:
                    print(Fore.BLUE + "1", end=" ")
                else:
                    value = str(matrix_data[i][j])
                    print(Style.RESET_ALL + value, end=" ")
            print(Style.RESET_ALL)

    def heuristic(self, curr_node, end_node):
        """ Heuristic function for A star """

        # dx = abs(curr_node.x - end_goal.x)
        self.dx = abs(curr_node.position[0] - end_node.position[0])
        # dy = abs(curr_node.y - end_goal.y)
        self.dy = abs(curr_node.position[1] - end_node.position[1])
        return (self.dx + self.dy)


    def is_node_valid(self, node, last_row, last_column):
        """ Check if the given cell is valid for the matrix """

        # If the node is not in the range of the matrix, it is invalid
        if node.position[0] > last_row or node.position[0] < 0 or node.position[1] > last_column or node.position[1] < 0:
            return False

        return True


    def findpath_astar_method(self, start, end):
        """ Returns the shortest path from start to end nodes in the given matrix """

        matrix = self.input_matrix

        # Find the last cell in the matrix
        last_row = len(matrix) - 1 
        last_column = len(matrix[len(matrix) - 1]) - 1
        self.debug_print("Last cell in the given matrix is (%d, %d)" % (last_row, last_column))

        # Initialize the start and end nodes
        start_node = MatrixNode(None, start)
        start_node.g = start_node.h = start_node.f = 0

        end_node = MatrixNode(None, end)
        end_node.g = end_node.h = end_node.f = 0

        # Check if the start node is within the matrix
        if not self.is_node_valid(start_node, last_row, last_column):
            print("Start node is not valid")
            return []

        # Check if the end node is within the matrix
        if not self.is_node_valid(end_node, last_row, last_column):
            print("End node is not valid")
            return []

        # Check if start node is traversable
        if (matrix[start_node.position[0]][start_node.position[1]] == 0):
            print("Start node is not traversable")
            return []

        # Check if end node is traversable
        if (matrix[end_node.position[0]][end_node.position[1]] == 0):
            print("End node is not traversable")
            return []

        open_list = []
        visited_list = []

        # Add start node to open_list
        open_list.append(start_node)

        while len(open_list) > 0:

            current_node = open_list[0]
            current_index = 0

            # Iterate through the nodes in open_list
            nodes_in_open_list = enumerate(open_list)
            for index, item in nodes_in_open_list:
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # Pop the node from open_list and add to visited_list
            open_list.pop(current_index)
            visited_list.append(current_node)

            # Check if end node is reached
            if current_node == end_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                # Add the complete path to allpaths for furture reference
                self.allpaths.append(path[::-1])
                self.debug_print("Reached the end node. Return shortest path.")
                # Return the path in reverse order
                return path[::-1] 
            else:
                if self.debug_mode():
                    path = []
                    current = current_node
                    while current is not None:
                        path.append(current.position)
                        current = current.parent
                    print(path[::-1])
                    self.debug_print(path[::-1]) 

            # Genenate the list of neighbors    
            neighbors = []

            # Traverse through the adjacent cells including diagonals
            for new_position in [(-1, -1), (-1, 1), (1, -1), (1, 1), (0, -1), (0, 1), (-1, 0), (1, 0) ]:

                # Get the node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

                # Check if the node is within the range of the matrix
                if node_position[0] > last_row or node_position[0] < 0 or node_position[1] > last_column or node_position[1] < 0:
                    continue

                # Continue if the node is not traversable
                if matrix[node_position[0]][node_position[1]] != 1:
                    continue

                # Create a new node and append to neighbor list
                new_node = MatrixNode(current_node, node_position)
                neighbors.append(new_node)

            # Iterate through the neighbor list
            for node in neighbors:
                move_to_next = 0

                # If the node is already in visited list, continue to next node
                for visited_node in visited_list:
                    if node == visited_node:
                        move_to_next = 1

                if move_to_next == 1:
                    continue

                # Populate g, h and f values for A star algorithm
                # g is the cost from start node to present node
                node.g = current_node.g + 1 
                # h determines the heuristics from present node to end node
                node.h = self.heuristic(node, end_node) 
                # f is the summation of g and h
                node.f = node.g + node.h  

                # if present node is present in open list and its cost is higher, continue
                for open_node in open_list:
                    if node == open_node and node.g > open_node.g:
                        move_to_next = 1

                if move_to_next == 1:
                    continue

                # Add the node to open_list
                open_list.append(node)

        # open_list is empty
        self.debug_print("Traversed the matrix. No path found")


    def findpath(self, src, dest):
        """ Invokes the astar method if the start/end nodes are not present in the available paths """

        length = len(self.allpaths)
        if length == 0:
            # First execution where allpaths is empty
            return (self.findpath_astar_method(src, dest))
        else:
            for i in range(length):
                try:
                    # Traverse through existing paths and check if the path is already available
                    self.start = self.allpaths[i].index(src)
                    self.end = self.allpaths[i].index(dest)
                    self.debug_print("Existing path found!")
                    # Existing path between start and end found. Return the path
                    return (self.allpaths[0][self.start:self.end+1])
                except ValueError:
                    # Run A star method to find the shortest path between start and end nodes
                    return (self.findpath_astar_method(src, dest))


def main(debug = False, print_result = False):

    '''
    matrix = [[1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0],
[0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0],
[0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0],
[0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0],
[0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0],
[0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1],
[1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0],
[0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0],
[0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0],
[0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0],
[0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1],
[1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0],
[0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0],
[0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0],
[0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1],
[1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0],
[0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0],
[0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0],
[0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0],
[0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1],
[1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0],
[0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0]]

    start = (0, 2)
    end = (19, 48)

    '''
    matrix = [[1, 1, 0, 1, 1, 1],
[1, 0, 1, 0, 0, 0],
[1, 0, 0, 1, 0, 1],
[1, 0, 1, 1, 1, 1],
[1, 0, 1, 0, 1, 1],
[1, 1, 1, 1, 1, 1]]

    start = (0,0)
    end = (5,5)
   

    start_time = time.time()
    matrix_instance = MatrixPath (matrix)
    matrix_instance.debug_flag = debug
    matrix_instance.debug_mode()

    shortPath = matrix_instance.findpath(start, end)
    print("The Shortest Path is: ")
    print(shortPath)
    print("Execution time: %s seconds" % (time.time() - start_time))

    if print_result:
        matrix_instance.print_matrix(matrix, shortPath)


def parseArgs():
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--debug', help='Debug mode', action='store_true', default=False)
    parser.add_argument('-p', '--print_result', help='Print shortest path in matrix', action='store_true', default=True)
    args = parser.parse_args()
    return args

if __name__ == '__main__':
    args = parseArgs()
    main(args.debug, args.print_result)