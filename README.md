Need to add:
2 motors in back of robot (2 sparkMAX each) -- reverse?? -- move arm up & down
Motor in middle of arm (claw to grab onto cage and release)  -- both directions (1 sparkMAX)

1 motor to control the rollers at the front of arm (1 sparkMAX)

(possibly 2)


-----------------------
  Executing task: gradlew build   -Dorg.gradle.java.home="C:\Users\Public\wpilib\2025\jdk" 


FAILURE: Build failed with an exception.

* Where:
Build file 'C:\Users\Rancho Robos\Documents\GitHub\RanchoReapers_2025\8533_RanchoReapers_2025\build.gradle' line: 90

* What went wrong:
Could not determine the dependencies of task ':jar'.
> Could not resolve all files for configuration ':runtimeClasspath'.
   > Could not resolve choreo:ChoreoLib-java:2025.0.2.
     Required by:
         root project :
      > Could not resolve choreo:ChoreoLib-java:2025.0.2.
         > Could not get resource 'https://lib.choreo.autos/dep/choreo/ChoreoLib-java/2025.0.2/ChoreoLib-java-2025.0.2.pom'.
            > Could not GET 'https://lib.choreo.autos/dep/choreo/ChoreoLib-java/2025.0.2/ChoreoLib-java-2025.0.2.pom'.
               > Got SSL handshake exception during request. It might be caused by SSL misconfiguration
                  > PKIX path building failed: sun.security.provider.certpath.SunCertPathBuilderException: unable to find valid certification path to requested target

* Try:
> Run with --stacktrace option to get the stack trace.
> Run with --info or --debug option to get more log output.
> Run with --scan to get full insights.
> Get more help at https://help.gradle.org.

BUILD FAILED in 4s

 *  The terminal process "cmd.exe /d /c gradlew build   -Dorg.gradle.java.home="C:\Users\Public\wpilib\2025\jdk"" terminated with exit code: 1. 
 *  Terminal will be reused by tasks, press any key to close it. 
