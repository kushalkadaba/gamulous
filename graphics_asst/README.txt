Changes made in Picker.cpp
1) Added code in all the TODOs, to maintain node stack 
2) Implemented getRbtNodeAtXY using colorToId

Changes made in scenegraph.cpp
1) Added code in visit and postvisit to maintain node stack
2) Implemented the getAccumulatedRbt

Changes made in asst4.cpp
1) Copied from asst3.cpp
2) Added new shader files used while picking.
3) Replaced the pointers to the cubes with shared_ptr to robot nodes.
4) Implemented the constructRobot function to draw the robots.
5) Changed drawStuff function to use a different shader when picking.
6) Changed the reset function to reset all the parts.
7) Added functionality for 'p' key.