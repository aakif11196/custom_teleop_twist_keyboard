Custom Teleop Twist Keyboard is a user-friendly ROS 2 package designed to make robot control simple, smooth, and fun — just like playing a game.
This customized version of the standard teleop_twist_keyboard provides intuitive key mappings for easy robot movement in all directions, including forward, backward, sideways, and diagonal motions.

It also adds:

Extra movement controls for flexible robot navigation

Emergency stop (space key) for safety during operation

Game-like key layout for a more natural and engaging teleoperation experience

You can easily modify or extend the key mappings by editing only the main Python script inside the package to suit your specific robot or application.



🧭 How to Change or Add Keys in Custom Teleop Twist Keyboard

Follow these steps to modify or add new keys for controlling your robot using the Custom Teleop Twist Keyboard package.

🪜 Step-by-Step Instructions

 - Step 1: Go to your ROS2 workspace.
   This is the main directory where your ROS2 packages are located.

- Step 2: Open the src folder inside your workspace.
  This folder contains all your source packages.

- Step 3: Clone this repository into the src folder (if you haven’t already done so).
  This will add the custom_teleop_twist_keyboard package to your workspace.

- Step 4: Open the package folder.
  Go to the folder named custom_teleop_twist_keyboard, and then open the subfolder with the same name inside it.

Example path:

ros2_ws/src/custom_teleop_twist_keyboard/custom_teleop_twist_keyboard

- Step 5: Find the main Python file inside this folder.
  It is usually named something like:

custom_teleop_twist_keyboard.py or teleop_twist_keyboard.py
This file contains all the code that defines how the keyboard keys control your robot.

- Step 6: Open this Python file in your code editor.
 You can use any editor such as VS Code, Atom, or Notepad++.

- Step 7: Locate the section that defines key bindings.
  Inside the Python file, you’ll find a part where keys are mapped to robot motions.

- Step 9: Save the file after making your changes.

- Step 10: Build your workspace again so that ROS2 applies the new updates.

- Step 11: Run your teleop keyboard node to test the changes.
  When you press the newly added or modified keys, your robot should respond accordingly.

- Step 12: Test and verify.
 Check your robot’s movement to make sure all keys work as expected.
 If any key doesn’t respond, recheck your code changes and ensure the file was saved properly.

✅ Tip:
You can experiment with adding special functions like emergency stop, extra motion keys, or even gripper controls — just follow the same pattern used in the key bindings.
