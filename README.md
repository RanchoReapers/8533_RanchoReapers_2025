![2025 FRC REEFSCAPE Controller Layout - 8533RR](https://github.com/user-attachments/assets/337cc4f3-3377-44b8-8f25-817e9f8147bc)
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

    a![2025 FRC REEFSCAPE Controller Layout - 8533RR](https://github.com/user-attachments/assets/59d0ea16-ea2c-4055-883b-54c0c7e93727)
imAssist("placeCoral");
    // depends on if we are able to pick up algae whether or not we should add a function to place coral... if we need to line up to pickup algae aim assist will        // hinder & we are not able to distingush between intent to place coral and remove algae..
    // we could map the aim assist for the coral dropoff to a button press, but still have the pickup be automatic
    }
  }
}


ex. 1b (autoimplements pickup assist but dropoff assist must be toggled)

PROCEDURE enableAimAssistLoop {
REPEAT WHILE (( robotIsEnabled == True )) AND (( aimAssistActive == False)) {
IF (( currentAprilTag == (# of april tag by coral station for red team) OR currentAprilTag == (# of april tag by coral station for blue team) )) AND ((         
   distanceFromCurrentTag < (# of meters/inches/feet/cm? away from tag -- need to testing for distance) )) {
   
   aimAssist("pickupCoral");
   
} ELSE IF (( currentAprilTag == (# of april tag by coral station for red team) OR currentAprilTag == (# of april tag by coral station for blue team) )) AND ((     
     distanceFromCurrentTag < (# of meters/inches/feet/cm? away from tag -- need to testing for distance) ))  AND (( exampleActivationControllerButton.isPressed == True )) {
   
   aimAssist("placeCoral");
   
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
1a) auto activate when by reef & coral pickup OR 1b) auto activate when by coral pickup and **require a controller press to activate** when by reef (since when by reef the driver may just want to remove algae, and the aim assist will be a nuisance. NOTE THAT 1B SHOULD ONLY BE IMPLEMENTED IF THE ROBOT IS DESIGNED TO ALSO REMOVE ALGAE, OTHERWISE IMPLEMENT 1A!!!
2) deactivate aim assist when a key on the controller is pressed and have a cooldown (like 5 ish seconds -- enough to allow the driver to move out of auto activation range) before aim assist can auto reactivate itself





