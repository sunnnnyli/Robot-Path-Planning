import heapq
import math
import copy


# --------------------- Notes:
# Have not implemented some final path tracing
#    Board is reversed so direction might need changing for ginal answers
# Currently not working, not sure what the issue is


# Node class
class Node:
    def __init__(self, pos, path_cost, total_cost, parent = None):
        self.pos = pos
        self.path_cost = path_cost
        self.total_cost = total_cost
        self.parent = parent

    def __lt__(self, other):
        # used for heapq
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
    input_file = open(input_file_path, 'r')
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
    for line in workspace:
        print(line)
    return start_pos, goal_pos, workspace


def calculate_heuristic(curr_pos, goal_pos):
    """
    Calculate the h(n) value based on current and goal position
    :param curr_pos:
    :param goal_pos:
    :return:
    """
    return math.sqrt((curr_pos[0]-goal_pos[0])**2+(curr_pos[1]-goal_pos[1])**2)


def check_pos(pos, workspace):
    """
    Returns whether a position is valid (not out of bound and not blocked)
    :param pos: position to check
    :param workspace: 2D list of workspace
    :return: boolean of validity
    """
    if pos[1] < 0 or pos[0] < 0 or pos[1] >= len(workspace) or pos[0] >= len(workspace[0]):
        return False
    return workspace[pos[1]][pos[0]] == 0


def a_star_search_algo(start_pos, goal_pos, workspace):
    """
    Implementation of the A* search algorithm
    :param start_pos: starting coordinate
    :param goal_pos: goal coordinate
    :param workspace: workspace 2D list
    :return: None if no solution; curr_node, generated if solution is found
    """
    directions = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]
    start_node = Node(start_pos, 0, 0)
    reached = {}
    frontier = []
    heapq.heappush(frontier, start_node)
    generated = [start_node]

    while frontier:
        # Get the smallest value
        curr_node = heapq.heappop(frontier)

        # If solution is found
        if curr_node.pos == goal_pos:
            return curr_node, generated

        # If node is not a repeat
        if curr_node.pos not in reached or curr_node.total_cost < reached[curr_node.pos]:
            reached[curr_node.pos] = curr_node.total_cost

            # Generate all child nodes
            for direction in directions:
                new_pos = (curr_node.pos[0] + direction[0], curr_node.pos[1] + direction[1])

                # Append node to generated and frontier if it's valid
                if check_pos(new_pos, workspace):
                    if direction == (1, 0) or direction == (0, 1) or direction == (-1, 0) or direction == (0, -1):
                        action_cost = 1
                    else:
                        action_cost = math.sqrt(2)
                    child_path_cost = curr_node.path_cost + action_cost
                    child_heuristic = calculate_heuristic(new_pos, goal_pos)
                    child_total_cost = child_path_cost + child_heuristic
                    if new_pos in reached and reached[new_pos] <= child_total_cost:
                        continue
                    child_node = Node(new_pos, child_path_cost, child_total_cost, curr_node)
                    generated.append(child_node)
                    heapq.heappush(frontier, child_node)


def calculate_output_values(final_node, workspace):
    # Just didn't want to change the original workspace
    new_workspace = copy.deepcopy(workspace)
    depth = 0
    curr_node = final_node
    while curr_node:
        pos = curr_node.pos
        print(pos)
        new_workspace[pos[1]][pos[0]] = 4
        depth += 1
        curr_node = curr_node.parent

    return depth, new_workspace


def output_into_file(output_workspace):
    output_file = open("final_output.txt", "w")
    for i in range(len(output_workspace)):
        for j in range(len(output_workspace[0])):
            print(output_workspace[i][j], end=" ", file=output_file)
        print(file=output_file)
    output_file.close()


def main():
    start_pos, goal_pos, workspace = process_input("Sample input.txt")
    result = a_star_search_algo(start_pos, goal_pos, workspace)
    if result:
        final_node, generated_nodes = result
        depth, output_workspace = calculate_output_values(final_node, workspace)
        print(depth)
        output_into_file(output_workspace)


if __name__ == "__main__":
    main()




