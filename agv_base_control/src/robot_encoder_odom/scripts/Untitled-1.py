
from math import sqrt


(x, y) = (0, 0)
goals: list[tuple] = [(5, 0), (5, 5)]

for index, (goal_x, goal_y) in enumerate(goals):
    distance_error = sqrt((goal_x - x)**2 + (goal_y - y)**2)
    print(f"{index = }, {goal_x = }, {goal_y = }, {distance_error = }")