"""
A basic but flexible A* implementation and examples on string permutations

mail@kaiploeger.net
"""


from queue import PriorityQueue



class AStarNode:
    """
    Wraps any kind of state description to store additional information for A*
    """
    def __init__(self, state, action=None, parent=None, verbose=False):
        self.state = state    # any sort of state description
        self.action = action  # the last applied action to get here
        self.parent = parent  # the previous node
        self.children = []    # next nodes - only generated on demand
        if parent:
            self.path = parent.path[:]
            self.verbose = parent.verbose
        else:
            self.path = []
            self.verbose = verbose

        self.path.append((state, action))

        if self.verbose:
            print('new nodes path:')
            for p in self.path:
                print(p)

    def get_children(self, get_transitions):
        if not self.children:
            self.children = [AStarNode(state, act, self)
                             for state, act in get_transitions(self.state)]
        return self.children


class AStarSolver:
    """
    A basic A* solver
    You need to provide cost, search heuristic, and transition functions.
    """
    def __init__(self, cost_func, heur_func, get_trans, verbose=False):
        """
        A basic A* solver:

        Parameters:
            cost_func(path)-->(float):
                maps a path [(s, a), ..., (s,a)] to a scalar cost

            heur_func(start, goal)-->(float):
                Compes a lower bound on the cost to go. In cost of the goal
                state needs to be 0 for the alg to terminate. Overestimating
                the cost to go leads to non-optimal solutions.

            get_trans(state):
                returns a list [(s, a), ..., (s, a)] of all possible actions a
                and their corresponding next states s.
        """

        self.cost = cost_func
        self.heur = heur_func
        self.trans = get_trans
        self.verbose = verbose

    def solve(self, start, goal):
        """returns an optimal path from start to goal or [] if no path exists"""

        alive_nodes = PriorityQueue()  # nodes to expand on
        unique_key = 0  # unique secondary key in the PriorityQueue to prevent
                        # direct comparison of two State objects
        start_node = AStarNode(start, verbose=self.verbose)
        alive_nodes.put((0, unique_key, start_node))
        dead_states = []  # visited states

        # first found path is guaranteedto be optimal
        path = []
        while(not path and alive_nodes.qsize() > 0):
            best_candiate_node = alive_nodes.get()[2]
            dead_states.append(best_candiate_node.state)
            for child_node in best_candiate_node.get_children(self.trans):
                if child_node.state not in dead_states:
                    heur = self.heur(child_node.state, goal)
                    if heur == 0:  # goal state reached --> return bath
                        path = child_node.path
                        break
                    priority = self.cost(child_node.path) + heur  # A* magic!
                    unique_key += 1
                    alive_nodes.put((priority, unique_key, child_node))
        return path



def example_swap_2_neighbouring_chars(verbose):
    """Demonstrates how to sort a string by swapping neighbouring chars."""

    start = "bdca"
    goal = "abcd"
    print("\nexample 1 - swap neighbouring chars: {} --> {}".format(start, goal))

    def get_cost_swap_2_neighbouring_chars(path):
        """cost=number of steps"""
        return len(path)

    def get_heuristic_swap_2_neighbouring_chars(state, goal):
        """for each char add half the distance to it's correct position"""
        return sum([abs(i - state.index(goal[i])) / 2 for i in range(len(goal))])

    def get_transitions_swap_2_neighbouring_chars(state):
        """swap all possible pairs of chars neighbouring in a string
        and return state action pairs"""
        return [(swap_neighbours(state, i), "{}->{}".format(i, i+1)) \
                 for i in range(len(state)-1)]

    def swap_neighbours(state, i):
        return state[:i] + state[i+1] + state[i] + state[i+2:]

    a_star = AStarSolver(get_cost_swap_2_neighbouring_chars,
                         get_heuristic_swap_2_neighbouring_chars,
                         get_transitions_swap_2_neighbouring_chars,
                         verbose=verbose)

    path = a_star.solve(start, goal)
    for i in range(0, len(path)):
        print("{}) {}, {}".format(i, path[i][0], path[i][1]))


def example_swap_2_chars(verbose):
    """Demonstrates how to sort a string by swapping any char pairs."""

    start = "bdca"
    goal = "abcd"
    print("\nexample 2 - swap any two chars: {} --> {}".format(start, goal))

    def get_cost_swap_2_chars(path):
        """cost=number of steps"""
        return len(path)

    def get_heuristic_swap_2_chars(state, goal):
        """add 0.5 for each char that is not in it's correct position"""
        return sum([a != b for a, b in zip(list(state), list(goal))]) / 2

    def get_transitions_swap_2_chars(state):
        """swap all possible pairs of chars in a string and return state action pairs"""
        return [(swap(state, i, j), "{}->{}".format(i, j)) \
                 for i in range(len(state)) for j in range(len(state)) \
                 if i != j]

    def swap(s, i, j):
        lst = list(s)
        lst[i], lst[j] = lst[j], lst[i]
        return ''.join(lst)

    a_star = AStarSolver(get_cost_swap_2_chars,
                         get_heuristic_swap_2_chars,
                         get_transitions_swap_2_chars,
                         verbose=verbose)

    path = a_star.solve(start, goal)
    for i in range(0, len(path)):
        print("{}) {}, {}".format(i, path[i][0], path[i][1]))


def example_move_char_to_any_empty_slot(verbose):
    """ Demonstrates how to sort a string by moving chars to empty slots '_'."""

    start = "b_dca"
    goal = "abcd_"
    print("\nexample 3 - move any char to any empty slot: {} --> {}".format(start, goal))

    def get_cost_move_char(path):
        """cost=number of steps"""
        return len(path)

    def get_heuristic_move_char(state, goal):
        """add 1 for each char that is not in it's correct position"""
        return sum([a != b for a, b in zip(list(state), list(goal))])

    def get_transitions_move_char(state):
        """swap all possible pairs of chars in a string and return state action pairs"""
        return [(swap(state, i, j), "{}->{}".format(i, j)) \
                 for i in range(len(state)) for j in range(len(state)) \
                 if state[i] != '_' and state[j] == '_']

    def swap(s, i, j):
        lst = list(s)
        lst[i], lst[j] = lst[j], lst[i]
        return ''.join(lst)

    a_star = AStarSolver(get_cost_move_char,
                         get_heuristic_move_char,
                         get_transitions_move_char,
                         verbose=verbose)

    path = a_star.solve(start, goal)
    for i in range(0, len(path)):
        print("{}) {}, {}".format(i, path[i][0], path[i][1]))



if __name__ == '__main__':
    example_swap_2_neighbouring_chars(verbose=False)
    example_swap_2_chars(verbose=False)
    example_move_char_to_any_empty_slot(verbose=False)
