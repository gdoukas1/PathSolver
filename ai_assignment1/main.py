import os
import sys

import matplotlib as mpl
import matplotlib.pyplot as plt

try:
    mpl.use('Qt5Agg')
except ImportError:
    mpl.use('TkAgg')

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer

# add current directory to python path for local imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from SMP.maneuver_automaton.maneuver_automaton import ManeuverAutomaton
from SMP.motion_planner.motion_planner import MotionPlanner
from SMP.motion_planner.plot_config import StudentScriptPlotConfig


def print_results(path_scenario, name_planner, h_weight, h_option, found_path, visited_nodes, h_cost, g_cost):
    scenario = path_scenario.split('/')

    if h_option == 1:
        heuristic = "Euclidean"
    elif h_option == 2:
        heuristic = "Manhattan variation"
    else:
        heuristic = "None"

    if found_path is not None:
        path = ""
        for node in found_path:
            x = str("{:.2f}".format(node[0]))
            y = str("{:.2f}".format(node[1]))
            path = path + "(" + x + ", " + y + ") -> "
        path = path[:-3]
    else:
        path = "None"

    print("==================================================")
    print("<" + scenario[1] + ">")
    print(name_planner + " (w=" + str(h_weight) + "  Heuristic: " + heuristic + ")" + ":")
    print("\tVisited Nodes number: " + str(visited_nodes))
    print("\tPath: " + path)
    print("\tHeuristic Cost (initial node): " + str("{:.2f}".format(h_cost)))
    print("\tEstimated Cost: " + str("{:.2f}".format(g_cost)))
    print("==================================================")

    file = open('results.txt', 'a')
    file.write("==================================================\n")
    file.write("<" + scenario[1] + ">\n")
    file.write(name_planner + " (w=" + str(h_weight) + "  Heuristic: " + heuristic + ")" + ":\n")
    file.write("\tVisited Nodes number: " + str(visited_nodes) + "\n")
    file.write("\tPath: " + path + "\n")
    file.write("\tHeuristic Cost (initial node): " + str("{:.2f}".format(h_cost)) + "\n")
    file.write("\tEstimated Cost: " + str("{:.2f}".format(g_cost)) + "\n")
    file.write("==================================================\n")
    file.close()


def main():
    # configurations
    path_scenario = 'Scenarios/scenario1.xml'
    file_motion_primitives = 'V_9.0_9.0_Vstep_0_SA_-0.2_0.2_SAstep_0.4_T_0.5_Model_BMW320i.xml'
    config_plot = StudentScriptPlotConfig(DO_PLOT=True)
    heuristic_option = 1  # 1 -> Euclidean,  2 -> Manhattan
    heuristic_weight = 1

    # load scenario and planning problem set
    scenario, planning_problem_set = CommonRoadFileReader(path_scenario).open()
    # retrieve the first planning problem
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    # create maneuver automaton and planning problem
    automaton = ManeuverAutomaton.generate_automaton(file_motion_primitives)

    # comment out the planners which you don't want to execute
    dict_motion_planners = {
        0: (MotionPlanner.DepthFirstSearch, "Depth First Search"),
        1: (MotionPlanner.Astar, "A* Search"),
        2: (MotionPlanner.IterativeDeepeningAstar, "Iterative Deepening A* Search")
    }

    for (class_planner, name_planner) in dict_motion_planners.values():
        planner = class_planner(scenario=scenario, planning_problem=planning_problem,
                                automaton=automaton, plot_config=config_plot)

        # start search
        print(name_planner + " started..")
        found_path, v_nodes, h_cost, g_cost = planner.execute_search(time_pause=0.001,
                                                                     heuristic_option=heuristic_option,
                                                                     heuristic_weight=heuristic_weight)

        print_results(path_scenario=path_scenario, name_planner=name_planner, h_weight=heuristic_weight,
                      h_option=heuristic_option, found_path=found_path, visited_nodes=v_nodes, h_cost=h_cost,
                      g_cost=g_cost)


print('Done')

if __name__ == '__main__':
    main()
