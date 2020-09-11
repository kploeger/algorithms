"""
An example of how to plan a discrete object soring task with A*
http://www.goal-robots.eu/

Object handles can be of any type an are never copied.

mail@kaiploeger.net
"""


import numpy as np
from a_star import AStarSolver



class Location:
    def __init__(self, position, object_ids=None):
        self.position = position
        self.object_ids = object_ids
        if self.object_ids is None:
            self.object_ids = []

    def is_empty(self):
        return len(self.object_ids) == 0

    def __repr__(self):
        return str(self.position)


class State():
    def __init__(self, objects=[], locations=[]):
        self.objects = objects
        self.locations = [Location(loc) for loc in locations]

    @staticmethod
    def copy(other_state):
        """shallow copy state.objects"""
        objects = other_state.objects[:]
        loc_positions = [loc.position for loc in other_state.locations]

        state = State(objects, loc_positions)
        for loc, other_loc in zip(state.locations, other_state.locations):
            loc.object_ids = other_loc.object_ids[:]

        return state

    def move_object(self, obj_id, loc_id):
        for loc in self.locations:
            loc.object_ids = [obj_id_ for obj_id_ in loc.object_ids if obj_id_ != obj_id]
        self.locations[loc_id].object_ids.append(obj_id)

    def __repr__(self):
        print_string = ""
        for loc in self.locations:
            objs = [self.objects[obj_id] for obj_id in loc.object_ids]
            print_string += "{}->{}  ".format(loc, objs)
        return print_string


class Action:
    def __init__(self, value):
        self.value = value

    def __repr__(self):
        return str(self.value)



def cost_function(path):
    """cost=number of steps"""
    return len(path)


def search_heuristic(state, goal):
    """add 1 for each object that is not in it's correct position"""
    heur = 0
    for loc, goal_loc in zip(state.locations, goal.locations):
        for obj_id in goal_loc.object_ids:
            if not obj_id in loc.object_ids:
                heur += 1
    return heur


def get_transitions(state, constraints=[]):
    """swap all possible pairs of chars in a string and return state action pairs"""
    transitions = []
    for old_loc_id, old_loc in enumerate(state.locations):
        for obj_id in old_loc.object_ids:
            for new_loc_id, new_loc in enumerate(state.locations):
                # check constraints:
                for constr in constraints:
                    cont = False
                    if not constr(state, obj_id, old_loc_id, new_loc_id):
                        cont = True
                        break
                if cont:
                    continue  # skip transition if a constraint is violated

                # add transition
                action = Action("{} from {} to {}".format(state.objects[obj_id],
                                                              old_loc_id, new_loc_id))
                new_state = State.copy(state)
                new_state.move_object(obj_id, new_loc_id)
                transitions.append((new_state, action))
    return transitions



def no_move_in_place(state, obj_id, old_loc_id, new_loc_id):
    """do not move an object to it's current position"""
    return new_loc_id != old_loc_id


def no_object_stacking(state, obj_id, old_loc_id, new_loc_id):
    """make sure objects are not stacked"""
    return state.locations[new_loc_id].is_empty()


def no_collisions(state, obj_id, old_loc_id, new_loc_id, thresh=1.0):
    """check for potential collisions"""
    xs = state.locations[old_loc_id].position
    xg = state.locations[new_loc_id].position

    for loc in state.locations:
        # does loc have an object
        if loc.is_empty():
            continue

        x = loc.position
        if all(x == xs) or all(x == xg):
            continue

        # is loc in the way? ...
        # ... is loc between start and goal?
        if any([not (xs[i] <= x[i] <= xg[i] or xs[i] >= x[i] >= xg[i])
                for i in range(len(xs))]):
            # print('outside')
            continue

        # ... is loc close to the linear path?
        # d = ||(a-p)-((a-p)n)n||;  a = xs;  p = x'  n = (xg-xs)/||xg-xs||
        a = xs
        p = x
        n = (xg-xs) / np.linalg.norm(xg-xs)
        d = np.linalg.norm((a-p)-((a-p).dot(n)*n))
        if d < thresh:
            return False
    return True



def sort_objects_no_constraints(start_state, goal_state, verbose):
    print('\n1) no constraints:')

    a_star = AStarSolver(cost_function,
                         search_heuristic,
                         lambda state: get_transitions(state, [no_move_in_place]),
                         verbose=verbose)
    path = a_star.solve(start_state, goal_state)

    print('Steps:')
    for i, step in enumerate(path):
        print("{}) {}, {}".format(i, step[0], step[1]))


def sort_objects_collisions(start_state, goal_state, verbose):
    print('\n2) w/ collision avoidance:')

    constraints = [no_move_in_place,
                   lambda *args: no_collisions(*args, thresh=0.1)]

    a_star = AStarSolver(cost_function,
                         search_heuristic,
                         lambda state: get_transitions(state, constraints),
                         verbose=verbose)
    path = a_star.solve(start_state, goal_state)

    print('Steps:')
    for i, step in enumerate(path):
        print("{}) {}, {}".format(i, step[0], step[1]))


def sort_objects_collisions_stacking(start_state, goal_state, verbose):
    print('\n3) w/ collision avoidance, w/o stacking:')

    constraints = [no_move_in_place,
                   lambda *args: no_collisions(*args, thresh=0.1),
                   no_object_stacking]

    a_star = AStarSolver(cost_function,
                         search_heuristic,
                         lambda state: get_transitions(state, constraints),
                         verbose=verbose)
    path = a_star.solve(start_state, goal_state)

    print('Steps:')
    for i, step in enumerate(path):
        print("{}) {}, {}".format(i, step[0], step[1]))



def main():
    objects = ["cube", "ball", "cup"]  # could be any type with __repr__()

    locations = [np.array([0., 0., 0.]),
                 np.array([1., 0., 0.]),
                 np.array([2., 0., 0.]),
                 np.array([2., 1., 0.])]

    start_state = State(objects, locations)
    start_state.move_object(0, 2)
    start_state.move_object(1, 0)
    start_state.move_object(2, 3)

    goal_state = State(objects, locations)
    goal_state.move_object(0, 0)
    goal_state.move_object(1, 1)
    goal_state.move_object(2, 2)

    print('Kitchen sorting tasks:')
    print('Start: {}'.format(start_state))
    print('Goal:  {}'.format(goal_state))

    sort_objects_no_constraints(start_state, goal_state, verbose=False)
    sort_objects_collisions(start_state, goal_state, verbose=False)
    sort_objects_collisions_stacking(start_state, goal_state, verbose=False)



if __name__ == '__main__':
    main()

