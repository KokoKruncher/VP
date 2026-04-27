Group 11 Baseline laptime simulator.

The simulator is separated into several files to aid organisation and parallel development.
Most files are just class files which contain data and have methods to act on said data.

There are 3 ways to run the simulator:
1. Run the script "run_sim.m".
   This is the standard way. The vehicle parameters can be modified in the first section, and the standard plots will be displayed.

2. Run the script "run_sim_GUI_v5.m".
   This will open up a user interface where vehicle parameters can be entered and a lap can be simulated. Open the "Documentation" tab in the GUI for further instructions.
   
3. Create custom script.
   This may be useful for running parameter sweeps. Note that when entering parameters into the "Vehicle" object, they must be converted to SI units.