# Automated Silhouette Component Counter
**An Edge-Computed Machine Vision Workstation on the ESP32-CAM**

![Project Overview](Hardware%20Implementation.jpg)


## Project Overview
This project is an automated machine-vision workstation designed to eliminate the time-consuming bottleneck of manually counting small electronic components—such as resistors, ICs, and connectors—for bulk customer orders.

Traditional standard cameras often fail in this context because metallic pins cause glare, overhead lighting creates shadows, and dumped parts overlap, ruining standard vision algorithms.Furthermore, manual counting results in slow fulfillment and human error, leading to under-counting (upsetting customers) or over-counting (direct inventory loss).

To solve this, this system utilizes optical backlighting and mechanical agitation, powered entirely by a single ESP32-CAM. It processes these silhouettes natively at 20-30 frames per second, instantly streaming an accurate, real-time count to the operator's web browser to ensure fast and flawless order fulfillment.

## Key Features & Solutions

* **Optical Backlighting (Zero Glare):** A bottom-lit LED stage turns components into razor-sharp dark silhouettes, completely eliminating shadows and color noise.
* **Mechanical Agitation (Zero Overlap):** A 4-corner vibration motor array physically shakes clumped parts apart so they can be counted individually.
* **Instant Edge Counting:** The ESP32-CAM processes the separated silhouettes in real-time (20-30 FPS), giving the operator an exact count instantly on any web browser.

## Components & Tools

The hardware implementation relies on the following primary tools and components:
* ESP32-CAM
* MG90S Servo Motor
* 650nm Laser
* Motor Vibrators (4 units)
* 2N2222 / 2N2222A NPN Transistors
* 5mm LEDs
* 5V 3A Charger Adapter
* 3.3V Voltage Regulator AMS1117
* Wires and Φ6 Heat Shrinks
* CCTV Female DC Power Connector
* 400P Breadboard

![Circuit Diagram](Circuit%20Diagram.png)


## Software Process Pipeline

1. **Raw Grayscale Capture:** The OV2640 camera captures a 160x120 (QQVGA) grayscale image. Automatic white balance is disabled to maintain a fixed exposure against the bright LED background.
2. **Binary Thresholding:** The software evaluates every pixel against an adjustable brightness threshold. Dark pixels (the silhouettes) are classified as Foreground (1), and bright pixels (the light pad) are classified as Background (0).
3. **Connected-Component Labeling (8-Way BFS):** The algorithm scans the binary map. When it finds a foreground pixel, it launches a Breadth-First Search (BFS) using a pre-allocated ring queue to group all connecting pixels in 8 directions, forming a complete "blob" and determining its bounding box.
4. **Size & Noise Filtering:** Detected blobs are filtered mathematically to guarantee an accurate count.A minimum area of 15 pixels rejects dust specks, sensor noise, or scratches on the light pad. A maximum area of 3000 pixels rejects large shadows, tools, or the operator's hands entering the frame.
5. **Output Generation:** The validated blobs increment the final component count. The coordinates for the red bounding boxes and the compressed binary image are then packaged for real-time streaming.
## Challenges & Hardware Debugging

During development, several hardware challenges were identified and solved:

* **Thermal Management (ESP32 Overheating):** Powering the ESP32-CAM via the 5V pin caused the onboard voltage regulator to severely overheat under the combined load of the camera and Wi-Fi, melting the silicon camera mount. **Solution:** Bypassed the overburdened onboard regulator entirely by integrating a dedicated external 3.3V regulator, supplying stable power directly to the 3.3V pin and eliminating all thermal stress.
* **GPIO Current Protection (ESP32 Brownout Resets):** Activating the vibration motors via the NPN transistors initially caused the ESP32 to instantly crash and reset. **Solution:** Identified that driving the transistors without base resistors was drawing excessive current directly from the GPIO pins; installed current-limiting base resistors to protect the pins, resulting in stable logic-level switching without triggering brownout resets.
* **IoT Integration of a Capacitive Interface:** The commercial pad only featured a capacitive touch button with no digital data pins for the ESP32 to interface with. **Solution:** Engineered an electromechanical bridge by programming a web-controlled servo motor equipped with a conductive tip (a water-dampened sponge) to successfully mimic human capacitance and physically actuate the button on command.

## Future Improvements

* **Solid-State Capacitive Switching:** Replace the electromechanical servo and wet sponge with a highly reliable, zero-moving-parts solution. This involves applying conductive copper tape directly over the light pad's capacitive sensor and routing it through a digital relay or optocoupler to Ground (GND) to electrically simulate the capacitance of a human finger.
* **Integrated HMI Touch Display:** Upgrade from a web-based dashboard to a built-in physical touch screen to create a fully standalone industrial workstation. To bypass the ESP32-CAM's severe lack of free GPIO pins, the implementation will integrate low-pin-count communication protocols, such as a UART intelligent display using only TX/RX, or an I2C touch screen using only SDA/SCL.
* **Edge AI & Object Classification:** Transition from deterministic blob counting (BFS flood-fill) to neural network-based object detection. By training a lightweight machine learning model (using TinyML or TensorFlow Lite for Microcontrollers), the system can not only count the parts but accurately classify them by type (e.g., distinguishing a resistor from a ceramic capacitor).

---
**Author:** Morteda Hokman
[View the Project Presentation (PDF)](Automated%20Silhouette%20Component%20Counter.pdf)
