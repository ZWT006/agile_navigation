State Trajectory `[px,py,pq,vx,vy,vq]`  
[TRAJ_DATA_SHORT](TRAJ_DATA_SHORT.csv)  
[TRAJ_DATA_LONG](TRAJ_DATA_LONG.csv)  
[TRAJ_DATA_SEMICIRCLE](TRAJ_DATA_SEMICIRCLE.csv)

一段比较长的 State Path Nodes  
[WAYPOINTS](WAYPOINTS.csv)  
TRAJ_DATA = [ path, [ts; 0] ];  
[COEFFS](COEFFS.csv)  
TRAJ_DATA = [poly_coef_x_seg, poly_coef_y_seg, poly_coef_q_seg];  
[REAEQBEQ](REAEQBEQ.csv)  
TRAJ_DATA = [Re_Aeq, Re_beq];  
[AEQBEQ](AEQBEQ.csv)  
TRAJ_DATA = [Aeq, beq];

### test traj
dataname:[TRAJ_DATA_·map·_·type·]  
map: `map13.png`
type: PRM,OSQP,OPT,OVAL

#### TRAJ_DATA_map13
start:[1.30,2.25,0]
end:[1.30,5.80,180]
TRAJ_DATA_map13_PRM.csv
TRAJ_DATA_map13_OSQP.csv
TRAJ_DATA_map13_OPT.csv
TRAJ_DATA_map13_OVAL.csv

Init T =5.4726;Opt T =5.3753
############################# FEASIBLE CHECK ################################
Aeq bais sum = 1.269646e-02
obsCheck: whole count = 1007; near bound count =  324; unfeasible count =    0 
ovaCheck: whole count =  211; near bound count =    0; unfeasible count =    0 
dynCheck: whole count =  211; 
dynCheck vel Maximum state    : x =3.0000; y =3.0000; q =3.0000; 
dynCheck vel unfeasible count : x =     0; y =     0; q =    12; 
dynCheck vel near bound count : x =     0; y =     0; q =    23; 
dynCheck acc Maximum state    : x =4.0000; y =4.0000; q =4.0000; 
dynCheck acc unfeasible count : x =     7; y =     8; q =    21; 
dynCheck acc near bound count : x =    16; y =    14; q =    31;  

#### TRAJ_DATA_map14
start:[1.30,3.00,0]
end:[1.30,5.00,180]
TRAJ_DATA_map14_PRM.csv
TRAJ_DATA_map14_OSQP.csv
TRAJ_DATA_map14_OPT.csv
TRAJ_DATA_map14_OVAL.csv

#### TRAJ_DATA_map15
start:[1.30,1.50,0]
end:[1.30,6.50,180]
TRAJ_DATA_map15_PRM.csv
TRAJ_DATA_map15_OSQP.csv
TRAJ_DATA_map15_OPT.csv
TRAJ_DATA_map15_OVAL.csv

#### TRAJ_DATA_map16
start:[1.50,1.50,0]
end:[6.50,6.50,90]
TRAJ_DATA_map16_PRM.csv
TRAJ_DATA_map16_OSQP.csv
TRAJ_DATA_map16_OPT.csv
TRAJ_DATA_map16_OVAL.csv

Init T =5.4455;Opt T =5.4455
############################# FEASIBLE CHECK ################################
Aeq bais sum = 2.979076e-07
obsCheck: whole count =  749; near bound count =  231; unfeasible count =    0 
ovaCheck: whole count =  157; near bound count =   11; unfeasible count =    0 
dynCheck: whole count =  157; 
dynCheck vel Maximum state    : x =3.0000; y =3.0000; q =3.0000; 
dynCheck vel unfeasible count : x =     0; y =     0; q =     0; 
dynCheck vel near bound count : x =     0; y =     0; q =     0; 
dynCheck acc Maximum state    : x =4.0000; y =4.0000; q =4.0000; 
dynCheck acc unfeasible count : x =    12; y =     0; q =    14; 
dynCheck acc near bound count : x =    14; y =     0; q =    19; 
############################# TIME CHECK ################################
Segments =  9, Optimal Values =   81 
Quadratic Optimization Clock = 0.015664 
Nonlinear Optimization Clock = 4.469931 