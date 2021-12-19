# Motion Planning for Mobile Robots -- Assignment 07 Real-Time Dynamic Programming

**NOTE** Please open this in **VSCode** with **Python plugin**

Solution guide for **Assignment 07, Real-Time Dynamic Programming**. 

---

## Introduction

Welcome to **Solution Guide for Assignment 07**! Here I will guide you through the **Python** implementations of

* **07 Real-Time Dynamic Programming**

---

## Q & A

Please send e-mail to alexgecontrol@qq.com with title **Motion-Planning-for-Mobile-Robots--Assignment-07--Q&A-[XXXX]**. I will respond to your questions at my earliest convenience.

**NOTE**

* I will **NOT** help you debug your code and will only give you suggestions on how should you do it on your own.

---

## Real-Time Dynamic Programming

### Overview

All you need is to follow the sample code on **value iteration**, then you'll have the required **real-time DP** implementation 

#### Part 1, Admissible Heuristics

First, implement **your own admissible heuristics** in **build graph** to estimate the num. actions needed to reach goal state.
```python
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

            # TODO -- get admissible heuristic:
            distance_to_goal = 0
            state.g_value = distance_to_goal

            state.connect_to_graph(grid)
            graph[state.key] = state

    for pos in START_LINE:
        state = Node(pos[0], pos[1], 0, 0)

        # TODO -- get admissible heuristic:
        distance_to_goal = 0
        state.g_value = distance_to_goal

        state.connect_to_graph(grid)
        graph[state.key] = state

    for pos in FINISH_LINE:
        state = Node(pos[0], pos[1], 0, 0)
        state.is_goal = True
        graph[state.key] = state

    output = open(save_path, 'wb')
    pickle.dump(graph, output)
```

#### Part 2, Value Update in Real-Time DP

Then implement the value update logic in **Real-Time DP**

```python
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
                # TODO -- update node cost:
                value_uk = []
                for child_idx in range(len(ACTION_SPACE)):

                current_value = np.min(value_uk)
                
                # TODO -- update bellman error:
                bellman_error += 0.0

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

```

---

## Wrap-Up

Happy Learning & Happy Coding!

Yao

* [GitHub](https://github.com/AlexGeControl/Motion-Planning-for-Mobile-Robots)

* [LinkedIn](https://www.linkedin.com/in/yao-ge-765315a0/)