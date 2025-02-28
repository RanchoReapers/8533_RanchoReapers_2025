Team 8533 Robot Code Documentation - FRC REEFSCAPE 2024
----


What Needs to be Done?
----
(In order of importance)
- Update kTrackWidth (distance between right and left wheels) & kWheelBase (distance between front and back wheels)
- Have Mechanical fix arm and claw -- then mount to robot. IMPORTANT THAT CLAW AND ARM ARE SET TO DESIRED '0' POSITION BEFORE ATTACHMENT!!!!
- After arm mounted, have Electrical mount all components and SparkMAXes to base. Need to get robot legal & competition ready.
- Modify arm, intake, and claw code to use the Trigger functionality for calling in commands (WPILib docs)
    - Deploy new arm & claw code. Test arm, then test claw.
- For the 2 nested steps, first learn how to simulate a controller
    - Write code for autonomous mode using a controller simulation
    - Translate aim assist code from tank to swerve
- Implement pathing to autonomous and aim assist using Choreo
- Setup Elastic as a dashboard


What's Happening?
----
- Mechanical is fixing the arm
- Bumpers are being made
- While waiting for Mechanical to finish the arm on Friday, work on the following:
    - Update kTrackWidth (distance between right and left wheels) & kWheelBase (distance between front and back wheels)
    - Start to setup Elastic
        - Set Elastic as default dashboard on competition PC: https://docs.wpilib.org/en/stable/docs/software/driverstation/manually-setting-the-driver-station-to-start-custom-dashboard.html
        - UNDER NO CIRCUMSTANCES SHOULD THE ROBOT BE ENABLED!!!!! This will screw up the absoluteEncoder variables. Wait until the arm is installed properly to enable.
        - To setup Elastic and have the new variables be recognized, deploy the code to the robot. Just remember -- DON'T ENABLE IT!!!!


Controls
----
![REVISION 1 -- 2025 FRC REEFSCAPE Controller Layout - 8533RR](https://github.com/user-attachments/assets/d10d7e39-8d24-4df8-80bb-e167bba0b169)


logic for aim assist -- please enter edit mode to read the code below easier:
----
when robot IsEnabled {
enableAimAssistLoop
}

ex. 1a (detects when to implement aim assist)

PROCEDURE enableAimAssistLoop {
REPEAT WHILE (( robotIsEnabled == True )) AND (( aimAssistActive == False )) {
IF (( currentAprilTag == (# of april tag by coral station for red team) OR currentAprilTag == (# of april tag by coral station for blue team) )) AND ((         
   distanceFromCurrentTag < (# of meters/inches/feet/cm? away from tag -- need to testing for distance) )) {
   
   aimAssist("pickupCoral");
   
} ELSE IF (( currentAprilTag == (# of april tag by reef station for red team) OR currentAprilTag == (# of april tag by reef station for blue team) )) AND (( 
             distanceFromCurrentTag < (# of meters/inches/feet/cm? away from tag -- need to testing for distance) )) {

AimAssist("placeCoral");
    }
  }
}

ex. 2 (implementation of procedure aimAssist)

PROCEDURE startCooldown {
WAIT seconds (5)
enableAimAssistLoop
}

PROCEDURE aimAssist(param) {
SET aimAssistActive <- True
IF (( param == "pickupCoral" )) {
REPEAT WHILE (( exampleDeactivationControllerButton.isPressed == False )) {
// aim assist logic for picking up coral
  }
SET aimAssistActive <- False
} ELSE IF (( param == "placeCoral" )) {
REPEAT WHILE (( exampleDeactivationControllerButton.isPressed == False )) {
// aim assist logic for placing coral
    }
SET aimAssistActive <- False
  }
startCooldown
}

PLEASE NOTE: to implement aim assist the steps are:
1) auto activate when by reef & coral pickup
2) deactivate aim assist when a key on the controller is pressed and have a cooldown (like 5 ish seconds -- enough to allow the driver to move out of auto activation range) before aim assist can auto reactivate itself





