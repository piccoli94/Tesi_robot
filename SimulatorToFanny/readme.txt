*************************************************************
Dynamic simulator of the Kuka LWR4+ robot for IK vs. ID paper

					v 3.2.1

G. Antonelli & C. Natale

October 2015
*************************************************************

Tested under the following system conditions (not all strictly required)

1. Linux 64bit (Ubuntu 15.04) and Windows 8 64bit
2. MATLAB 2014a 64bit
4. Robotics Toolbox (P. Corke) v9.7 (not strictly req.)

Main file list

1. main script: 
   loadparam.m
5. Simulink model containing the model of the robot and the De Luca's algorithm for external torque estimation: 
   LWR4_ElasticJoints.slx

Use instructions

1. Launch the man script and then run the simulation
2. Setting the dynamic model parameters:
   a. only some dynamic paramters can be changed in the model (all other
      parameters are hard-coded in the symbolic model), they are:
      il1x il1y il1z il2x il2y il2z il3x il3y il3z il4x il4y il4z il5x il5y il5z il6x il6y il6z il7x il7y il7z...
      g pl1y pl2z pl3y pl4z pl5y pl7z pm1z pm3z pm5z im1 im2 im3 im4 im5 im6 im7...
      fv1 fv2 fv3 fv4 fv5 fv6 fv7 fc1 fc2 fc3 fc4 fc5 fc6 fc7
      The exact meaning of the symbols can be found in the functions described
      in 2.b.
   b. nominal and uncertain dynamic paramters are defined at lines 51 and 54
      of the main script through the two functions: DefineNominalParameters.m
      and DefineUncertainParameters.m. Please refer to the help of this last
      function to see how it works to introduce uncertainties.
   c. nominal paramters are used in the controller and uncertain ones are
      used in the robot model
   d. the parameters of the torque losses in the gearbox are eta and omega_T
3. It is possible to activate or not the simulation of the dry friction
   by setting the flag ACTIVATE_COULOMB_FRICTION
4. The dynamic model is now compued in a dedicated block that ouptuts all the matrices and vectors (M, C, fv, fc, g) computed on the basis of the nominal dynamic parameters. This allows avoiding recomputing in case they are needed in the control algorithm in more than one place, e.g., in the inverse dynamics control and in the residual estimation algorithm (De Luca) as in the example.
5. The CoriolisMatrix function has been rewritten to fix a bug in the previous version and now is really complex!
6. Be aware that the compilation time can be more than 1 hour depending on the MATLAB version and your PC, but the run time is acceptably fast.

