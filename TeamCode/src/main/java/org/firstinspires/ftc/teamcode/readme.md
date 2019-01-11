FTC 4390 Storm Robotics Typhoons Software readme.md
=======
This is the teamcode section of our application, where our OpModes are located. We have two major kinds of Op Modes, Autonomous and TeleOp (Linear)
## Autonomous
We have four files dedicated to autonomous:
1. Crater-Claim-Park.java
    * This autonomous OpMode starts on the crater side and:
    * Drops down from the Lander
    * Claims the Depot
    * Parks on the Crater
2. Crater-Claim.java
    * This autonomous OpMode starts on the crater side and:
    * Drops down from the Lander
    * Claims the Depot
3. Crater-Park.java
    * This autonomous OpMode starts on the crater side and:
    * Drops down from the Lander
    * Parks on the Crater
4. No-Lift.java
    * This autonomous OpMode can start at either the crater side or the depot side:
    * DOES NOT drop down from the Lander
    * EITHER claims the depot or parks on the crater (depending on side started at)
We have these OpModes so that we can be flexible and work with any alliance partner. The First one scores the most points, however there are times when omitting missions is necessary in order to prevent collision and interference between robots.
## Teleop
We have only one Linear OpMode dedicated to the Driver Control Period. It is MainTeleop.java and its controls can be found in a comment at the top of the file.





# Thanks for visiting FTC 4390 Storm Robotics Typhoons
