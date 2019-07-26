# Ararabots Verysmall - LIA

## Project Repository:
* Robot
* Vision
* Logical Board
* Interfaces

# Strategy
The strategy module aims to implement all the robots behaviours (goalkeeper, defender and attacker). Currently (2019) we use behaviour trees to model all the robots actions. For a getting started with behavior trees see: https://www.pirobot.org/blog/0030/.

- ## Submodules
    - Behaviour: In this file there is the building blocks of our behaviour tree architecture. It has the implementation of the Sequence class wich represents a sequence node, the Selector class represents the Selector node and the Blackboard class contains all the variables that compose de game state.
    - Base Trees:
