'''

The following node preforms the following taskts:
    - heuristic(): Heuristic function
    - astar(): Compute the shortest path to the goal using A* algorithm

INPUT: * Location 
       * source_node
       * target_node
       * heuristic

OUUTPUT: * shortest_path

'''

#### --------- Global Planning with A* --------- ####

## Imports
import rclpy
from rclpy.node import Node

import matplotlib.pyplot as plt
import osmnx  as ox   
import networkx as nx
import numpy as np 
import logging
from openstreetmap_node import Openstreetmap
#from global_planning_zoe.openstreetmap import * 
from pyrosm import OSM, get_data

# log file 
logging.basicConfig(filename='astar_node.log', level=logging.INFO)

# ---------  A* implementation --------- #

class Astar(Node):
    def __init__(self):
        super().__init__("astar_node")

    def heuristic(self, node1, node2):
        logging.info('Heuristic function: Compute distance')
        return ((node2 - node1)**2)**0.5

    def astar(self, graph,start_pos,goal_pos):
        logging.info('Implement A* algorithm !')

        open_set = {start_pos}
        came_from = {}
        g_score = {node: float('inf') for node in graph.nodes}
        f_score = {node: float('inf') for node in graph.nodes}
        g_score[start_pos] = 0
        f_score[start_pos] = self.heuristic(start_pos, goal_pos)

        while open_set:
            current_node = min(open_set, key=lambda node: f_score[node])
            if current_node == goal_pos:
                logging.info('Current node = goal node')
                path = []
                while current_node in came_from:
                    path.append(current_node)
                    current_node = came_from[current_node]
                path.append(start_pos)
                path.reverse()
                logging.info('Goal reached : Return the path !')
                if path:
                    logging.info('Path found !')
                    fig, ax = ox.plot_graph_route(graph, path, route_color='b', node_color='g', edge_linewidth=1, edge_alpha=1.0)
                    logging.info('Plot the path !')
                    plt.show(block=False)
                else:
                    logging.info('Path not found !')
                return path
            open_set.remove(current_node)

            logging.info('Current node != goal node')
            for neighbor in graph.neighbors(current_node):
                temp_g_score = g_score[current_node] + graph[current_node][neighbor].get('length',1)
                if temp_g_score < g_score[neighbor]:
                    came_from[neighbor] = current_node
                    g_score[neighbor] = temp_g_score
                    f_score[neighbor] =  g_score[neighbor] + self.heuristic(neighbor, goal_pos)
                    if neighbor not in open_set:
                        open_set.add(neighbor)    
        return None

# Main function
def main(args=None):
    rclpy.init(args=args)

    # Create node for simulation
    astar = Astar()
    
    # Spin indefinitely..
    #rclpy.spin(astar)

    # On shutdown...
    astar.destroy_node()
    rclpy.shutdown()
    

# Script entry point
if __name__ == "__main__":
    main()