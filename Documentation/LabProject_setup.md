# **Surgery Robotics Project setup**

The objectives of this section are:
- Setup the project in student's github
- Review the needed tools
- Create a simple ESP32 Blink project with PlatformIO and VS Code

## **1. Setup the project**

You have to select a local folder to clone the Project repository. 
- Open Visual Studio Code on this folder

### First time
- Open a git terminal and clone the forked github project from Director's repository.
- add credential information from your github account
  ````shell
  git config --global user.email "mail@alumnes.ub.edu" 
  git config --global user.name "your github username"
  ````
- You will have to add the `.vscode/settings.json` in your local repository

### Next times
- before working on your project you have to update your local repository with last changes on remote github repository:
  ````shell
  git pull
  ````
- Now you can work on the project locally
- to update your changes:
  ````shell
  git add .
  git commit -m "your message"
  git push
  ````
- You can check the changes in your github repository

## **2. Review the needed tools**

You will need to use a new tool to program the ESP32 microcontroller.:
- Visual Studio Code with extension "PlatformIO IDE" (PlatformIO)


## **3. Create a NEW ESP32 Blink project**

This project demonstrates how to create a new project. This project is to blink an LED using an ESP32 board programmed with PlatformIO and the Arduino framework inside Visual Studio Code.

## 🚀 Create a New Blink Project

- Click the alien icon (PlatformIO Home)
- Click `Create New Project` (In Project Tasks - Lefft-side bar menu) and `New Project` (on quick Access)
- You have to specify:
  - Name the project: `ESP32Test_Blink`
  - Select board: `Espressif ESP32 Dev Module`
  - Framework: `Arduino`
  - Unselect "Location: Use defauld location" and select your `src` Project_Surgery_Robotics folder
  - Click `Finish`
  - After few minutes your created project will appear in src folder and also an instance outside in VScode root folder

  PlatformIO will create the project structure and download necessary tools. The final structure will be like this:

    ```
    ESP32Test_Blink/
    ├── src/
    │   └── main.cpp
    ├── lib
    └── platformio.ini
    ```

## ⚙️ Configuration File: `platformio.ini`

Verify `platformIO.ini`

Here we will add the "monitor_speed" option to set the serial monitor baud rate to 115200.

  ```ini
  [env:esp32dev]
  platform = espressif32
  board = esp32dev
  framework = arduino
  monitor_speed = 115200
  ```

## 🧾 Source Code: `src/main.cpp`

For this speciffic exemple, replace the content of `src/main.cpp` with the following code:
```cpp
#include <Arduino.h>

const int ledPin = 2; // GPIO2, sovint connectat a un LED integrat

void setup() {
  Serial.begin(115200); // Inicialitza el port sèrie
  pinMode(ledPin, OUTPUT);
}

void loop() {
  digitalWrite(ledPin, HIGH);
  Serial.println("Led switched ON");
  delay(1000);

  digitalWrite(ledPin, LOW);
  Serial.println("Led switched OFF");
  delay(1000);
}
```

## ⚙️ Add libraries

For this exemple you do not need any library, but in a general project to add libraries:

### Simple libraries
- Go to the PlatformIO Home → Libraries tab.
- Search for the library you want.
- Click "Add to Project" and select your project.
### Custom libraries
- Add the library folder to the `lib` folder of your project.

This is the case of IMU_Robotics_UB library in `Endowrist_IMU` project.

## 🚀 Upload and Monitor

To upload your code to the ESP32 board and monitor the serial output:

- Connect your ESP32 board to your computer via USB.
- In PlatformIO, click the "Upload" button (right arrow icon) in the bottom toolbar.
- After the upload is complete, click the "Serial Monitor" button to open the serial monitor. You have to close the terminal to Close the serial port.
- You can also see the serial monitor from the Terminal menu `SERIAL MONITOR`. Choose the correct COM port and click `Start Monitoring`. You have to `Stop Monitoring` to Close the serial port.
- The onboard LED should blink on and off every second.

## 🚀 Make Copy of an existing Project 

To copy an existing project and modify its name and code to have a new version:

- Close VScode. The workspace without saving!!.
- Open a Filesystem explorer and copy the existing project folder to a new one with a different name.
- Open VScode on the new folder.
- Open the PlatformIO and select `open project`.
- Select the New project with the changed name
- Select `src/main.cpp` and modify the code.
- Upload and Monitor the new project.