import pickle
from racetracks import *
from graph_node import Node
import matplotlib.pyplot as plt


seed = np.random.seed(1234)
graph = {}


def build_up_graph(grid, save_path):
    """ build up state transition graph
    """
    max_vel = Node.MAX_VEL

    # velocity dimension
    vels = []
    for vx in range(-max_vel, max_vel + 1):
        for vy in range(-max_vel, max_vel + 1):
            vels.append([vx, vy])

    # position dimension
    x_idx, y_idx = np.where(grid == FREE)
    coord = np.stack([x_idx, y_idx], axis=1)
    for p_idx in range(coord.shape[0]):
        pos = coord[p_idx]
        for vel in vels:
            # build current node:
            state = Node(pos[0], pos[1], vel[0], vel[1])

            # get admissible heuristic:
            distance_to_goal = np.min(
                np.linalg.norm(
                    np.asarray(FINISH_LINE) - np.asarray([state.px, state.py]),
                    axis=1
                )
            )
            state.g_value = distance_to_goal / (np.sqrt(2)*max_vel)

            state.connect_to_graph(grid)
            graph[state.key] = state

    for pos in START_LINE:
        state = Node(pos[0], pos[1], 0, 0)

        # get admissible heuristic for start positions:
        distance_to_goal = np.min(
            np.linalg.norm(
                np.asarray(FINISH_LINE) - np.array([state.px, state.py]), 
                axis=1
            )
        )
        state.g_value = distance_to_goal / (np.sqrt(2)*max_vel)

        state.connect_to_graph(grid)
        graph[state.key] = state

    for pos in FINISH_LINE:
        state = Node(pos[0], pos[1], 0, 0)
        state.is_goal = True
        graph[state.key] = state

    output = open(save_path, 'wb')
    pickle.dump(graph, output)


def explore_action(u_idx, epsilon=0.2):
    """ exploitation or exploration
    """
    if np.random.uniform(0, 1) < epsilon:
        # exploration:
        return np.random.randint(0, len(ACTION_SPACE))
    else:
        # exploitation:
        return u_idx


def greedy_policy(start_state_idx=0, explore=True):
    """ follow greedy policy, pick node at random until goal is reached
    """
    # init current exploration:
    start_node = Node(
        START_LINE[start_state_idx][0], 
        START_LINE[start_state_idx][1], 
        0, 
        0
    )

    curr_state = graph[start_node.key]
    trajectory = [curr_state.key]

    while not curr_state.is_goal:
        # explore neighborhood:
        next_state_costs = []
        for action_idx in range(len(ACTION_SPACE)):
            next_state_key = curr_state.next_prob_9[action_idx]
            next_state = graph[next_state_key]
            next_state_costs.append(next_state.g_value)

        # select action:
        action_idx = np.argmin(next_state_costs)
        if explore:
            action_idx = explore_action(action_idx)

        # transit to next state:
        next_state_key = curr_state.next_prob_9[action_idx]
        
        curr_state = graph[next_state_key]
        trajectory.append(next_state_key)

    return trajectory


def real_time_dynamic_programming(max_iter = 1000, epsilon = 1e-4):
    """ solve the problem with RTDP
    """
    num_nodes_evaluated = 0
    bellman_error_list = []

    for i in range(max_iter):
        bellman_error = 0.0

        # get current plan:
        start_state_idx = np.random.randint(
            low=0, 
            high=len(START_LINE) - 1, 
            size=1
        )[0]
        greedy_plan = greedy_policy(start_state_idx=start_state_idx)
        
        for key in greedy_plan:
            state = graph[key]

            if state.is_goal:
                state.g_value = 0
            else:
                # update node cost:
                value_uk = []
                for child_idx in range(len(ACTION_SPACE)):
                    child_key_9 = state.next_prob_9[child_idx]
                    child_9 = graph[child_key_9]
                    child_key_1 = state.next_prob_1[child_idx]
                    child_1 = graph[child_key_1]

                    expected_cost_uk = 1.0 + (0.9 * child_9.g_value + 0.1 * child_1.g_value)
                    value_uk.append(expected_cost_uk)

                current_value = np.min(value_uk)
                
                # update bellman error:
                bellman_error += np.abs(state.g_value - current_value)

                # update node cost:
                state.g_value = current_value

            num_nodes_evaluated += 1

        # termination criteria:
        bellman_error = bellman_error
        if bellman_error < epsilon:
            print(
                "RTDP converged at {:d}th iteration: {:.4f} @ {:d} nodes evaluated".format(
                    i + 1, 
                    bellman_error, 
                    num_nodes_evaluated
                )
            )
            break

        # track Bellman error:
        bellman_error_list.append(bellman_error)

    # visualize Bellman error:
    plt.figure()
    x_axis = range(len(bellman_error_list))
    plt.plot(x_axis, bellman_error_list)
    plt.show()


def track_the_best_plan(start_state_idx = 0):
    """ extract best plan
    """
    # init trajectory:
    start_node = Node(
        START_LINE[start_state_idx][0], 
        START_LINE[start_state_idx][1], 
        0, 
        0
    )
    start_key = start_node.key
    
    curr_state = graph[start_key]
    trajectory = [curr_state]

    print("best plan is:")
    while not curr_state.is_goal:
        next_state_values = []
        for action_idx in range(len(ACTION_SPACE)):
            next_state_key = curr_state.next_prob_9[action_idx]
            child_9 = graph[next_state_key]
            next_state_values.append(child_9.g_value)
        next_state_key = curr_state.next_prob_9[np.argmin(next_state_values)]

        curr_state = graph[next_state_key]
        trajectory.append(curr_state)

        print(
            "\t({:02d}, {:02d})".format(curr_state.px, curr_state.py)
        )
    return trajectory


def visualize_the_best_plan(plan, grid_para):
    assert isinstance(plan, list)
    plt.figure(figsize=(4.5, 16))
    plt.pcolor(grid_para, edgecolors='k', linewidths=1)
    plan_len = len(plan)
    plan.append(plan[-1])
    for i in range(plan_len):
        plt.arrow(plan[i].py + 0.5, plan[i].px + 0.5,
                  plan[i+1].py - plan[i].py, plan[i+1].px - plan[i].px,
                  color='r', head_width=0.3, head_length=0.1)
    plt.show()


if __name__ == '__main__':
    # first build state transition graph for RTDP:
    path = './graph/graph_rtdp.dat'
    track_map = race_track
    # build_up_graph(track_map, path)
    
    graph = pickle.load(
        open(path, 'rb')
    )

    # solve with RTDP:
    real_time_dynamic_programming()
    plan = track_the_best_plan(start_state_idx=3)
    visualize_the_best_plan(plan, track_map)


