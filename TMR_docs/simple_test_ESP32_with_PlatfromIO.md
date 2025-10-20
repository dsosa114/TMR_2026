
Here is a simple walkthrough to create a basic "Hello World" serial program for an ESP32 using PlatformIO.

-----

### **Objective**

Create a simple program for an ESP32 that repeatedly prints a counter message to the Serial Monitor. This will confirm that you can successfully compile, upload, and monitor your ESP32 with PlatformIO.

### **Prerequisites**

  * Visual Studio Code with the PlatformIO IDE extension installed.
  * An ESP32 development board (e.g., ESP32 DevKitC, NodeMCU-32S).
  * A USB cable to connect the ESP32 to your computer.

-----

### **Step 1: Create a New PlatformIO Project**

1.  Open VS Code and go to the PlatformIO Home screen (click the alien head icon on the left sidebar or use the command palette).
2.  Click on "**+ New Project**".
3.  Fill out the Project Wizard:
      * **Name:** `esp32_serial_test`
      * **Board:** Search for and select "**ESP32 Dev Module**". This is a good generic option that works for most common boards.
      * **Framework:** Select "**Arduino**".
      * Uncheck "Use default location" if you want to save the project in a specific folder.
4.  Click "**Finish**" and wait for PlatformIO to set up the project files.

### **Step 2: Write the Arduino Code**

PlatformIO will create a project structure. Open the `src` folder and find the `main.cpp` file. Replace its entire contents with the following code:

```cpp
#include <Arduino.h>

// Define the built-in LED pin, which is usually GPIO 2 on most ESP32 dev boards.
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// A counter variable to see the program running
int counter = 0;

void setup() {
  // Initialize Serial communication at 115200 bits per second.
  // This is a common speed for ESP32.
  Serial.begin(115200);

  // A small delay to allow the serial monitor to connect before printing starts
  delay(2000); 

  // Configure the built-in LED pin as an output
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("===============================");
  Serial.println("ESP32 Serial Test Initialized");
  Serial.println("===============================");
}

void loop() {
  // Print the current counter value to the Serial port
  Serial.print("Message count: ");
  Serial.println(counter);
  
  // Increment the counter for the next loop
  counter++;

  // Blink the LED to give a visual confirmation that the code is running
  digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on
  delay(500);                     // Wait for half a second
  digitalWrite(LED_BUILTIN, LOW);  // Turn the LED off
  delay(500);                     // Wait for another half a second
}
```

### **Step 3: Configure `platformio.ini`**

The `platformio.ini` file tells PlatformIO how to build and upload your project. For this simple test, we just need to ensure the monitor speed is set correctly.

Open `platformio.ini` from the project's root directory. It should look like this. Add the `monitor_speed` line if it's not there.

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino

; Set the speed for the serial monitor to match Serial.begin() in the code
monitor_speed = 115200
```

### **Step 4: Build, Upload, and Monitor**

Now, use the PlatformIO toolbar at the bottom of the VS Code window.

1.  **Connect your ESP32** to your computer via USB.
2.  **Build:** Click the **checkmark icon (✓)** on the toolbar. This compiles your code. You should see a `SUCCESS` message in the terminal.
3.  **Upload:** Click the **right-arrow icon (→)** on the toolbar. PlatformIO will automatically detect the port, compile if needed, and upload the firmware.
      * *Note:* Some ESP32 boards require you to hold down the "BOOT" button when the terminal shows `Connecting........_____.....`.
4.  **Monitor:** Once the upload is successful, click the **plug icon (🔌)** on the toolbar. This will open the **Serial Monitor**.

### **Expected Output**

In the Serial Monitor terminal pane, you should see the following output, with the count increasing every second. The built-in LED on your ESP32 should also be blinking.

```
===============================
ESP32 Serial Test Initialized
===============================
Message count: 0
Message count: 1
Message count: 2
Message count: 3
...
```

If you see this output, your entire toolchain is working perfectly\! You are ready to move back to more complex projects like Micro-ROS, confident that the fundamentals are solid.

### **Troubleshooting**

  * **Garbled Text in Monitor:** The `monitor_speed` in `platformio.ini` does not match the value in `Serial.begin()`. Make sure both are `115200`.
  * **Upload Fails / Port Not Found:**
      * Ensure you are using a proper data USB cable (not a charge-only cable).
      * Make sure you have the necessary USB-to-UART drivers installed for your ESP32's chip (usually CP210x or CH340). Ubuntu typically has these built-in, but it's worth checking.
      * Try holding the `BOOT` button during the upload process.