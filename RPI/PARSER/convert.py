import sys 
import ast

# EXTERNAL PATHS
sys.path.append('../SLAM/RANSAC')

# EXTERNAL LIBRARIES
import RANSAC as ransac

RUNS = ["../../sample_logs/2020-1-9_22-28-2-DIAGNOSTIC_RUN.txt", 
        "../../sample_logs/2020-1-9_22-42-41-NICK1.txt",
        "../../sample_logs/2020-1-9_22-44-47-NICK2.txt",
        "../../sample_logs/2020-1-9_22-53-29-LIGHTSOUT.txt",
        "../../sample_logs/2020-1-9_22-53-55-LIGHTSOUTNICK.txt",
        "../../sample_logs/2020-1-9_22-54-27-MOVINGNICK.txt",
        "../../sample_logs/2020-1-9_22-57-23-BALLBOX.txt",
        "../../sample_logs/2020-1-9_22-58-1-BALLRAISEDBOX.txt",
        "../../sample_logs/2020-1-9_22-58-33-SNOWMAN.txt"]

# count = 0 
# for files in RUNS:
#     with open(files, "r") as file:
#         x = file.readlines()

#     curr = [] 

#     for i in range(len(x)):
#         curr.append(ast.literal_eval(x[i]))

#     coor = ransac.ConvertToCartesian(curr)

#     curr_name = str(count) + ".txt"

#     with open(curr_name, "w") as file:
#         for coordinates in coor.values():
#             current = str(coordinates[0]) + ' ' + str(coordinates[1]) + ' ' + str(coordinates[2]) + '\n'
#             file.write(current)

#     count = count + 1
    

