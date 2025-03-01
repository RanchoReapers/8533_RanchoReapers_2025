Team 8533 Robot Code Documentation - FRC REEFSCAPE 2024
----


What Needs to be Done?
----
(In order of importance)
- Base needs to be replaced.
- Mechanical fixes arm -- then mount to robot. IMPORTANT THAT CLAW AND ARM ARE SET TO DESIRED '0' POSITION BEFORE ATTACHMENT!!!!
- After arm mounted, Electrical mounts all components and SparkMAXes to base.
- Modify arm, intake, and claw code to use the Trigger functionality for calling in commands (WPILib docs)
    - Deploy new arm & claw code. Test arm, then test claw.
- For the 2 nested steps, first learn how to simulate a controller
    - Write code for autonomous mode using a controller simulation
    - Translate aim assist code from tank to swerve
- Implement pathing to autonomous and aim assist using Choreo


What's Happening?
----
- Base is being replaced
- Robot is being rewired
- Mechanical is fixing the arm
        - ^^^ In the meantime:
          - Turn on robot (DON'T ENABLE) & redeploy code. Then update Elastic on both computers (add average velocity radial gauges to both debug & driver and add all individual as numbers in list to debug)
          - Fix the way that button press commands are called using trigger objects
          - Simulate a controller and program autonomous
- Bumpers are being made




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





