Group 11 Extended laptime simulator.

The simulator is separated into several files to aid organisation and parallel development.
Most files are just class files which contain data and have methods to act on said data.

There are 4 ways to run the simulator:
1. Run the script "run_sim.m".
   This is the standard way. The vehicle parameters can be modified in the first section, and the standard plots will be displayed.

2. Run the script "run_sim_GUI_v7.m".
   This will open up a user interface where vehicle parameters can be entered and a lap can be simulated. Open the "Documentation" tab in the GUI for further instructions.
   
3. Run one of the provided parameter sweep scripts
   There are 2 files: "sweep_mechbal.m" and "sweep_decay_coefficient.m" which are set up to sweep ARB rate and tyre decay coefficient respectively.
   
4. Create custom script.
   This may be useful for running parameter sweeps.
   An easy way to get these started is to copy from one of the two provided parameter sweep scripts mentioned above.
   Note that when entering parameters into the "Vehicle" object, they must be converted to SI units.