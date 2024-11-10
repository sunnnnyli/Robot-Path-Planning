"""
Robot Path Finding Project
Authors: Anna Teng, Sunny Li
"""

import os
import logging
import heapq
import math
import copy
import argparse
from datetime import datetime


DIRECTIONS = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]


# Set up logging configuration
def setup_logging(input_file_name, output_file_name, k):
    os.makedirs('Logs', exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = os.path.join('Logs', f"{timestamp}.log")

    logging.basicConfig(
        filename=log_filename,
        level=logging.INFO,
        format='%(message)s'
    )

    logging.info("========= A* Search Run Log =========")
    logging.info(f"Input File: {input_file_name}")
    logging.info(f"Output File: Outputs\\{output_file_name}")
    logging.info(f"k: {k}")
    logging.info("=====================================\n")


# OutputModel class to simplify the code relating to the output
class OutputModel:
    def __init__(self, output_dict: dict):
        self.depth = output_dict["depth"]
        self.generated_nodes = len(output_dict["generated_nodes"])
        self.moves = " ".join(map(str, output_dict["moves"]))
        self.f_values = " ".join(map(str, output_dict["f_values"]))
        
        rows = [" ".join(map(str, row)) for row in output_dict["workspace"]]
        self.output_workspace = "\n".join(rows)


# Node class
class Node:
    def __init__(self, pos, path_cost, total_cost, last_angle=0, parent=None):
        self.pos = pos
        self.path_cost = path_cost
        self.total_cost = total_cost
        self.last_angle = last_angle
        self.parent = parent

    def __lt__(self, other):
        # Used for heapq
        return self.total_cost < other.total_cost

    def __repr__(self):
        # Used for debugging
        return str(self.pos)


def process_input(input_file_path):
    """
    Used to process input file into workable data
    :param input_file_path: path of input file
    :return: tuple of starting and goal positions, and a 2D array of the robot workspace
    """
    try:
        input_file = open(input_file_path, 'r')
    except FileNotFoundError:
        print("File not found!")
        return
    first_line = input_file.readline()
    first_line_data = first_line.strip().split()
    start_pos = (int(first_line_data[0]), int(first_line_data[1]))
    goal_pos = (int(first_line_data[2]), int(first_line_data[3]))
    workspace = []
    for line in input_file:
        if line != "\n":
            curr_line = []
            nums = line.strip().split()
            for num in nums:
                curr_line.append(int(num))
            workspace.append(curr_line)
    input_file.close()
    workspace.reverse()
    return start_pos, goal_pos, workspace


def calculate_heuristic(curr_pos, goal_pos):
    """
    Calculate the h(n) value based on current and goal position
    :param curr_pos: tuple of current location
    :param goal_pos: tuple of goal position
    :return: The heuristic value calculated
    """
    return math.sqrt((curr_pos[0]-goal_pos[0])**2+(curr_pos[1]-goal_pos[1])**2)


def is_valid_pos(pos, workspace):
    """
    Returns whether a position is valid (not out of bound and not blocked)
    :param pos: position to check
    :param workspace: 2D list of workspace
    :return: boolean of validity
    """
    if pos[1] < 0 or pos[0] < 0 or pos[1] >= len(workspace) or pos[0] >= len(workspace[0]):
        return False
    return workspace[pos[1]][pos[0]] != 1


def calculate_angle_cost(curr_angle, new_angle, k):
    """
    Calculates the angle cost for changing direction based on curr_angle and new_angle
    :param curr_angle: angle of current node in degrees
    :param new_angle: angle of next node in degrees
    :param k: penalty coefficient for direction change
    :return: angle cost as a float
    """
    delta_theta = abs(new_angle - curr_angle)
    if delta_theta > 180:
        delta_theta = 360 - delta_theta

    return k * (delta_theta / 180)


def calculate_distance_cost(curr_node, new_pos):
    """
    Calculates the distance cost to get to new_pos from curr_pos
    :param curr_node: coordinates of curr position
    :param new_pos: coordinates of new position
    :return: distance cost as a float
    """
    # Check if horizontal or vertical move
    if abs(new_pos[0] - curr_node.pos[0]) + abs(new_pos[1] - curr_node.pos[1]) == 1:
        distance_cost = 1
    else:
        distance_cost = math.sqrt(2)

    return distance_cost


def calculate_step_cost(curr_node, new_pos, k):
    """
    Calculates the step cost by adding distance and angle costs
    :param curr_node: coordinates of curr position
    :param new_pos: coordinates of new position
    :param k: penalty coefficient for direction change
    :return: total step cost as a float
    :return: new angle in degrees
    """
    dx, dy = new_pos[0] - curr_node.pos[0], new_pos[1] - curr_node.pos[1]
    new_angle = math.degrees(math.atan2(dy, dx)) % 360
    
    if curr_node.parent is None:
        angle_cost = 0
        new_angle = 0
    else:
        angle_cost = calculate_angle_cost(curr_node.last_angle, new_angle, k)

    distance_cost = calculate_distance_cost(curr_node, new_pos)

    return distance_cost + angle_cost, new_angle


def a_star_search_algo(start_pos, goal_pos, workspace, k):
    """
    Implementation of the A* search algorithm
    :param start_pos: starting coordinate
    :param goal_pos: goal coordinate
    :param workspace: workspace 2D list
    :return: None if no solution; curr_node, generated if solution is found
    """
    start_node = Node(start_pos, 0, calculate_heuristic(start_pos, goal_pos))
    reached = {start_pos: start_node.total_cost}
    frontier = []
    heapq.heappush(frontier, start_node)

    logging.info(f"Generated node:\t\t{start_node}")

    while frontier:
        # Get the smallest value
        curr_node = heapq.heappop(frontier)

        logging.info(f"Frontier popped:\t{curr_node}")

        # If solution is found
        if curr_node.pos == goal_pos:
            logging.info("======GOAL REACHED!!!======")
            logging.info("Reached: " + str(len(reached)))
            return curr_node, reached

        # Generate all child nodes
        for direction in DIRECTIONS:
            new_pos = (curr_node.pos[0] + direction[0], curr_node.pos[1] + direction[1])

            # Append node to generated and frontier if it's valid
            if is_valid_pos(new_pos, workspace):
                step_cost, new_angle = calculate_step_cost(curr_node, new_pos, k)
                child_path_cost = curr_node.path_cost + step_cost
                child_heuristic = calculate_heuristic(new_pos, goal_pos)
                child_total_cost = child_path_cost + child_heuristic
                if new_pos in reached and reached[new_pos] <= child_total_cost:
                    continue
                child_node = Node(new_pos, child_path_cost, child_total_cost, last_angle=new_angle, parent=curr_node)
                reached[child_node.pos] = child_node.total_cost
                heapq.heappush(frontier, child_node)

                logging.info(f"Generated node:\t\t{child_node}")
                logging.info(f"Added to frontier:\t{child_node}")


def calculate_output_values(final_node, workspace):
    """
    Function used to calculate several values needed for output
    :param final_node: The last node in the path found
    :param workspace: 2D list of the workspace
    :return: dictionary of all the values
    """
    # Just didn't want to change the original workspace
    new_workspace = copy.deepcopy(workspace)

    curr_node = final_node
    depth = -1
    moves = []
    f_values = []

    while curr_node:
        pos = curr_node.pos
        if new_workspace[pos[1]][pos[0]] != 2 and new_workspace[pos[1]][pos[0]] != 5:
            new_workspace[pos[1]][pos[0]] = 4
        depth += 1
        f_values.append(curr_node.total_cost)
        
        if curr_node.parent is not None:
            direction = (curr_node.pos[0] - curr_node.parent.pos[0],
                         curr_node.pos[1] - curr_node.parent.pos[1])
            move = DIRECTIONS.index(direction)
            moves.append(move)

        curr_node = curr_node.parent

    moves.reverse()
    f_values.reverse()
    new_workspace.reverse()

    return {
        "depth": depth,
        "moves": moves,
        "f_values": f_values,
        "workspace": new_workspace
    }


def output_into_file(output: OutputModel, file_name):
    """
    Used to write all output data into a output file
    :param output: The OutputModel used to help with output generation
    :param file: the filepath of the output file
    :return: None
    """
    os.makedirs('Outputs', exist_ok=True)

    # Construct the full path
    output_file_path = os.path.join('Outputs', file_name)

    output_file = open(output_file_path, "w")

    print(output.depth, file=output_file)
    print(output.generated_nodes, file=output_file)
    print(output.moves, file=output_file)
    print(output.f_values, file=output_file)
    print(output.output_workspace, file=output_file)

    output_file.close()


def main():
    parser = argparse.ArgumentParser(description="Run A* search on a robot workspace")
    parser.add_argument("input_file", type=str, help="Path to the input file")
    parser.add_argument("k", type=int, nargs='?', default=0, help="Angle cost penalty parameter (default: 0)")
    parser.add_argument("-o", "--output_file", type=str, default="Output.txt", help="Name of the output file (default: 'Output.txt')")
    args = parser.parse_args()

    setup_logging(args.input_file, args.output_file, args.k)

    start_pos, goal_pos, workspace = process_input(args.input_file)
    result = a_star_search_algo(start_pos, goal_pos, workspace, args.k)
    if result:
        final_node, generated_nodes = result
        output_dict = calculate_output_values(final_node, workspace)
        output_dict["generated_nodes"] = generated_nodes
        output = OutputModel(output_dict)
        output_into_file(output, args.output_file)


if __name__ == "__main__":
    main()
