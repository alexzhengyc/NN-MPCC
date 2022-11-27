# NN-MPCC
Replenished Model Predictive Contouring Control (MPCC) to incorporate vehicle dynamics expressed by neural networks



![snapshot](images\snapshot.jpg)

### Framework

1. Collect car states and input data by manual control or simple auto control. In the simulation, car states are integrated using the real model **(Not included)**
2. Create Dataset and train a neural network model to get a representation of the car dynamics model **(Not included)**

3. Obtain time-varying linear system ODE function by inputting past states and control values to the neural network model
4.  Set Model Predictive Contouring Control to get the optimal control value
5. Compared the results among the ground truth model,  neural network model, and the physics-based model with measure error







