# SmartFireAlarm-Audio-Processing-WiFi-Alert-with-ESP32-FreeRTOS-
**Project Description:**

The project is an implementation of a sound monitoring and notification system using an ESP32 microcontroller board. The system is designed to detect specific frequencies in the surrounding environment and generate alarms or notifications accordingly. It utilizes the ESP32's built-in I2S interface for audio input, performs Fast Fourier Transform (FFT) analysis on the captured audio samples, and applies signal processing techniques to detect specific frequencies of interest.

The system consists of the following main components:

**1. ESP32 Microcontroller:** The ESP32 board serves as the main control unit of the system. It handles the audio input, signal processing, frequency detection, and communication tasks.

**2. I2S Interface:** The ESP32's I2S interface is used to capture audio samples from an external microphone or audio source. The I2S interface is configured to receive audio data in 32-bit format at a specific sample rate.

**3. Fast Fourier Transform (FFT):** The captured audio samples are processed using the arduinoFFT library, which performs a Fast Fourier Transform to convert the time-domain audio signal into its frequency-domain representation.

**4. Frequency Detection:** The system analyzes the frequency spectrum obtained from the FFT to detect specific frequencies of interest. It uses threshold-based techniques and compares the peak frequency values with predefined frequency ranges to identify the presence of certain sounds, such as doorbells or fire alarms.

**5. Firebase Integration:** The system integrates with the Firebase Realtime Database to store and retrieve data. It uses the Firebase ESP Client library to establish a connection with the Firebase server, authenticate the user, and send the detected sound events and timestamps to the database for storage.

**6. Notifications and Alarms:** When a specific sound event is detected, the system generates an alarm or notification. It can send notifications to connected devices or trigger external devices such as sirens or lights. The notifications and alarms are implemented using Firebase's real-time capabilities and can be customized based on the detected sound events.

**7. Web Server:** The system includes a web server running on the ESP32 board that provides a user interface for monitoring and controlling the system. It allows users to view real-time sound data, configure system parameters, and receive updates on sound events.

The overall goal of the project is to create a sound monitoring and notification system that can be deployed in various environments, such as homes, offices, or public spaces, to enhance safety and security by detecting and alerting users to specific sound events.
