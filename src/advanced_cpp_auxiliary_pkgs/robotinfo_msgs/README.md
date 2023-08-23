# The robotinfo_msgs package

Author: Roberto Zegers  
Date: February 2023  

## Notes on the creation of this custom message package

###  Create a new package

The best practice here is to create a package dedicated to a ROS custom messages, and only that.
```
cd ~/catkin_ws/src
```
As a convention you can end your package name with “_msgs”:  
```
catkin_create_pkg robotinfo_msgs roscpp
```

Note: You don't need to add `std_msgs` as dependency if your package is only using ROS primitive data types.


### Create a new msg file
```
cd robotinfo_msgs
mkdir msg
cd msg
```

Use CamelCase for the name of the message:  
```
touch RobotInfo10Fields.msg
```

### Write the message definition

You can use any number of ROS primitive data types for messages or already existing messages that you’ve created or from other packages.

Add one field per line. First the data type, then the name, this package for instance has this definition:  
```
string data_field_01
string data_field_02
string data_field_03
string data_field_04
string data_field_05
string data_field_06
string data_field_07
string data_field_08
string data_field_09
string data_field_10
```

The above is an example message that creates 10 data fields using the primitive datatype `string`.  

### Modify the CMakeLists.txt script 

- You must add `message_generation` inside the `find_package()` instruction:  
```
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
)
```

- You must call the `add_message_files()` function:  

```
add_message_files(
  FILES
  RobotInfo10Fields.msg
)

``` 
- You must ensure the `generate_messages()` function is called:

```
generate_messages(
  DEPENDENCIES
  # std_msgs  # Or other packages containing msgs
)
``` 


### Modify the package.xml file

- Add these 3 lines:  
```
  <build_depend>message_generation</build_depend>
  <build_export_depend>message_runtime</build_export_depend>
  <exec_depend>message_runtime</exec_depend>
```

Then build and source:
``` 
cd ~/catkin_ws && catkin_make && source devel/setup.bash
```

Check our custom message is correctly generated or not using below command:  
```
rosmsg show samplemsg
```

---

## Use a custom messages in a ROS node

- Add to your package.xml file:
```
  <!-- Depend on custom message -->
  <depend>robotinfo_msgs</depend> 
```

- Add to your `CMakeLists.txt` the following extra lines just below the `target_link_libraries()` instruction:  
```
add_dependencies(robot_info_node robotinfo_msgs_generate_messages_cpp)
```

Otherwise you will see this error message at compilation:
```
fatal error: robotinfo_msgs/RobotInfo10Fields.h: No such file or directory
       #include "robotinfo_msgs/RobotInfo10Fields.h"
```

- Inside your source code add this include:  
```
#include "robotinfo_msgs/RobotInfo10Fields.h"
```

- To create a message object called `msg`:  
```
robotinfo_msgs::RobotInfo10Fields msg;
```

- To assign a value to one of the message fields:  
```
msg.data_field_01 = "Value assigned";
```
