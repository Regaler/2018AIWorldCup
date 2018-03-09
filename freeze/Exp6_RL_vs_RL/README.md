# EXP5 CHARACTERISTICS: 
# Topic: RL vs RL

----
## Specification
* epsilon = 1.000
* final_epsilon = 0.01
* dec_epsilon = 0.001
* gamma = 0.99
* batch_size = 64
* learning_rate = 1e-4
* memory length = 100000

----
## Information
* 4 agents use FC respectively
* Initialized with pre-trained model (rulebased v13)
* Match between the same codes
* When 'Frame Delay Break' occurs, don't write weights, losses, memory, etc..