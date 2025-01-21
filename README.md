<h1>ESP32-S3 driver for "Logitech Extreme 3D Pro" joystick</h1> 

The Logitech Extreme 3D Pro is an ubitiquous and inexpensive joystick commonly used for games and flight simulators on the PC.
This repository ports it to be used with ESP32-S3 on the ESP-IDF platform. 

<h2>Setup and Usage</h2>

NOTE: In theory, this code can be used on any ESP32 variant which has a USB Host functionality, but it has only been tested on ESP32-S3. 
NOTE 2: You will likely need to supply extra power to the joystick.

<li>Clone the repository to your computer, such as using <code>git clone https://github.com/sgushy/esp-idf-logitech-joystick.git</code></li>

<li><a href="https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html">Make sure you have installed ESP-IDF with VS Code, Eclipse, or so on...</a></li>

<li>Example usage is found in <code>/main/example_usage.c</code></li>
