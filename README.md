<h1 align="center" style="font-size: 48px;">🕹️ Custom Teleop Twist Keyboard</h1>

**Custom Teleop Twist Keyboard** is a user-friendly **ROS 2 package** designed to make robot control simple, smooth, and fun — just like playing a game.  
This customized version of the standard `teleop_twist_keyboard` provides **intuitive key mappings** for easy robot movement in all directions — forward, backward, sideways, and diagonal.

---

### 🚀 Features
- Extra movement controls for flexible robot navigation  
- Emergency stop (`space` key) for safety during operation  
- Game-like key layout for a more natural and engaging teleoperation experience  
- Easily customizable key bindings for your specific robot  

You can easily modify or extend the key mappings by editing **only the main Python file** inside the package.

---

<h2 align="center" style="font-size: 32px;">🧭 How to Change or Add Keys in Custom Teleop Twist Keyboard</h2>

Follow these steps to modify or add new keys for controlling your robot using this package.

---

### 🪜 Step-by-Step Instructions

#### **1. Go to your ROS 2 workspace**
```bash
cd ~/ros2_ws
```

#### **2. Go to the src folder**
```bash
cd ~/ros2_ws/src
```
#### **3. Clone this repository (if not already added)**
```bash
git clone https://github.com/aakif11196/custom_teleop_twist_keyboard.git
```


#### **4. Open the package folder**
```bash
ros2_ws/src/custom_teleop_twist_keyboard/custom_teleop_twist_keyboard
```
#### **5. Find the main Python file**
```bash
custom_teleop_twist_keyboard.py
```
#### **6. Open this Python file in your editor**
Use any code editor like VS Code, Atom, or Notepad++.

#### **6. Locate the key binding section Look for a section like this:**
```bash

# movement keys
moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (diag, -diag, 0, 0),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (diag, diag, 0, 0),
    'k': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, 1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, -1, 0, 0),
    'K': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
    ' ': (0, 0, 0, 0),
    'h': (0, 1, 0, 0),
    ';': (0, -1, 0, 0),
}
```
#### **7. Add or modify keys**

#### **8.Save your file**
Make sure all your edits are saved properly.

#### **9. Build your workspace**
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

#### **11. Run your teleop node**
```bash
ros2 run custom_teleop_twist_keyboard custom_teleop_twist_keyboard
```
<h3 align="center" style="font-size:20px;">⭐ Star this repository if you find it helpful!</h3>




