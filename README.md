readme:
  filename: README.md
  content: |
    <h1 align="center" style="font-size:56px; margin-bottom:0.2em;">🕹️ Custom Teleop Twist Keyboard</h1>
    <p align="center" style="font-size:16px; margin-top:0.1em; color: #6b7280;">
      A user-friendly ROS 2 package that makes robot control simple, smooth and enjoyable — like playing a game.
    </p>

    <p align="center">
      <img src="https://raw.githubusercontent.com/aakif11196/custom_teleop_twist_keyboard/dc4497470b95bd5abfd2528f106e27bb126993d3/Keys.jpg" alt="Custom Teleop Preview" width="900"/>
    </p>

    ---

    <h2 style="font-size:32px;">🎮 Overview</h2>

    **Custom Teleop Twist Keyboard** is a user-friendly ROS 2 package designed to make robot control feel natural and intuitive — just like a game controller!  
    It extends the standard `teleop_twist_keyboard` with:

    - **Extra motion keys** for enhanced robot movement (forward, backward, diagonal, sideways)
    - **Emergency Stop (space bar)** for instant halt
    - **Game-style layout** for a smoother teleoperation experience
    - **Easily customizable key bindings** — edit only one Python file

    ---

    <h2 style="font-size:28px;">🧭 How to Change or Add Keys</h2>

    Follow the steps below to modify or add new keys for controlling your robot using the Custom Teleop Twist Keyboard package.

    ---

    ### 🪜 Step-by-Step Instructions

    1. **Open your ROS 2 workspace**  
       This is the main directory where all your ROS 2 packages are stored.  
       ```bash
       cd ~/ros2_ws
       ```

    2. **Go to the `src` folder**  
       The `src` folder contains all source packages.  
       ```bash
       cd ~/ros2_ws/src
       ```

    3. **Clone this repository** (if not already added)  
       ```bash
       git clone https://github.com/aakif11196/custom_teleop_twist_keyboard.git
       ```

    4. **Open the package folder**  
       Inside your workspace, go to:  
       ```bash
       cd ~/ros2_ws/src/custom_teleop_twist_keyboard/custom_teleop_twist_keyboard
       ```

    5. **Find the main Python file**  
       It is usually named:  
       ```
       custom_teleop_twist_keyboard.py
       ```
       or  
       ```
       teleop_twist_keyboard.py
       ```
       This file defines how keys control your robot’s motion.

    6. **Open this Python file in your code editor**  
       (VS Code, Atom, Sublime Text, or Notepad++)

    7. **Locate the key binding section**  
       Look for a section like:
       ```python
       move_bindings = {
           'w': (1, 0, 0, 0),   # forward
           's': (-1, 0, 0, 0),  # backward
           'a': (0, 0, 1, 0),   # turn left
           'd': (0, 0, -1, 0),  # turn right
       }
       ```

    8. **Add or modify keys**

       ➕ *To add a new key:*
       ```python
       'e': (0, -1, 0, 0),   # move right (strafe)
       ' ': (0, 0, 0, 0),    # space = emergency stop
       ```

       ✏️ *To change an existing key:*
       ```python
       'w': (2, 0, 0, 0),    # faster forward speed
       ```

    9. **Save your file**  
       Make sure to save all changes.

    10. **Build your workspace**
        ```bash
        cd ~/ros2_ws
        colcon build
        source install/setup.bash
        ```

    11. **Run your teleop node**
        ```bash
        ros2 run custom_teleop_twist_keyboard custom_teleop_twist_keyboard
        ```

    12. **Test your keys**  
        Press your newly added or modified keys and confirm your robot moves as expected.

    ---

    <h3 style="font-size:22px;">✅ Tips & Best Practices</h3>

    - Keep all screenshots or illustrations in an `images/` folder.
    - Use clear, short key labels.
    - Add comments in the Python file for each key’s purpose.
    - Try new features such as:
      - **Gripper control keys**
      - **Speed mode toggles**
      - **Emergency reverse or brake**
    - Update this README when you add new controls.

    ---

    <h3 style="font-size:22px;">📘 Example Keymap</h3>

    | Key | Action |
    |-----|--------|
    | `w` | Move forward |
    | `s` | Move backward |
    | `a` | Turn left |
    | `d` | Turn right |
    | `e` | Strafe right |
    | `Space` | Emergency stop |

    ---

    <h3 style="font-size:22px;">💡 Example Layout</h3>

    <p align="center">
      <img src="https://raw.githubusercontent.com/aakif11196/custom_teleop_twist_keyboard/dc4497470b95bd5abfd2528f106e27bb126993d3/Keys.jpg" alt="Teleop Key Layout" width="800"/>
    </p>

    ---


    <h3 align="center" style="font-size:20px;">⭐ Star this repository if you find it helpful!</h3>
