Changes made in rigtform.h
1) Added constructor with parameters (Cvec3, Quant)
2) Added constructor with parameter (Cvec3)
3) Added transFact, linFact, doMwrtA, makeMixedFrame and rigTFormToMatrix in terms of RigTForm instead of Matrix4 form.

Changes made in asst3.cpp
1) Line no. 200 added g_arcball as a global handle to Geometry object
2) Line no. 245 - 255, added initArcBall to make the sphere geometry for arc ball
3) Line no. 288- 297, added getArcBallRBT to return the current arc ball based on the viewmode and current object being manipulated.
4) Line no. 339 - 358, added code to draw the arcball in wireframe mode.
5) Line no. 383 - 391, added getDirection to get the normalized vector between the screen arcball center and the mouse click.
6) Line no. 394 - 434, added getMRbt() to get the M (manipulation rigtform).
7) Line no. 591 - 606, added case 'a' to handle enable/disable arcball. 

Changes made in matrix4.h
1) Removed doMwrtA, transFact & linFact.