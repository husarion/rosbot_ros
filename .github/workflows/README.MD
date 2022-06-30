# Navigation Testing
Every Monday at 18:00 a navigation test will be launched from ```navigation_testing``` package. ROSbot has to navigate around a corner in less than 30 seconds. Destination and timeout are both defined in ```navigation_testing/configtheb/nav_test.yaml```. If test results in failure it is possible to launch test locally with GUI (gazebo and rviz) using 
```
docker compose -f compose.simulation.gui.test
```  
in order to find what causes naviation errors.
