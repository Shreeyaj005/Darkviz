# Darkviz 

**Autonomous Omni-Directional Robotic System for Dark-Store Inventory Handling and Order Fulfillment**

---

## Team Details

| Sr. No. | Name of Student | Roll No. | Branch | Email ID |
|---|---|---|---|---|
| 1 | Shreeeya Jadhav | 45  | Automation and Robotics | 2023.shreeya.jadhav@ves.ac.in |
| 2 | Ayush Likhar | 11 | Automation and Robotics | 2023.ayush.likhar@ves.ac.in |
| 3 | Yash Gupta | 44 | Automation and Robotics | 2023.yash.gupta@ves.ac.in |
| 4 | Vedant Chavan | 40 | Automation and Robotics | 2023.vedant.chavan@ves.ac.in |

---
## Proposal 
https://youtu.be/RFOlutvGod4

## Guide Details

**Project Guide: Amudha Senthilkumar**  
**Department:** Automation and Robotics  
**Institute:** VESIT, Mumbai  

---

## Problem Statement

> The aim of this project is to develop an autonomous robotic solution for inventory retrieval in quick-commerce dark stores by integrating omni-directional mobility, SLAM based navigation, and a tray based product retrieval mechanism to reduce human intervention, minimize picking errors, and improve the speed and consistency of order preparation.

---

## Abstract

### Abstract

Quick-commerce dark stores depend on rapid and accurate inventory retrieval to fulfill customer orders within short delivery times. Manual product picking can lead to delays, human errors, and increased operational effort. **DarkViz** is proposed as an autonomous robot to automate inventory retrieval and improve order preparation in dark stores.

The robot uses an **omni-directional drive system** and **SLAM (Simultaneous Localization and Mapping)** for indoor mapping, localization, and autonomous navigation. It is equipped with a **tray-based retrieval mechanism** to collect products from designated storage locations and transport them to the order-fulfillment area. Product locations can be assigned through an existing inventory management system.

The expected outcome is a functional prototype capable of accurately navigating the dark-store environment and retrieving products with minimal human intervention. The system can serve as a foundation for future **fully automated dark-store operations**, with applications in quick-commerce, warehouse automation, inventory management, and logistics.


---

## Objectives

1. To study existing inventory retrieval and automation challenges in quick-commerce dark stores and analyze available robotic solutions.
2. To design an omni-directional mobile robot with a suitable hardware, software, navigation, and tray-based product retrieval architecture.
3. To implement SLAM-based mapping and autonomous navigation along with the tray mechanism for automated product retrieval.
4. To test and validate the robot’s navigation, localization, product retrieval, and overall system performance in a simulated and physical dark-store     environment.
5. To document and present the design, implementation, results, and future scope of the DarkViz robotic system.

---

## Scope of the Project

* Development of the omni-directional robot prototype with tray based retrieval mechanism.
* Simulation of the robot and dark-store environment.
* Implementation of SLAM for mapping and localization.
* Development of autonomous navigation and robot control.
* Hardware-software integration and real-world testing.
* Performance testing of navigation and product retrieval.
* Foundation for a fully automated dark-store system.

---

## Existing System

Quick-commerce dark stores currently use inventory management software combined with manual human picking. While software manages inventory, order allocation, and picking routes, workers physically locate, retrieve, and transport products.

Limitations
- High dependence on manual labor.
- Increased risk of picking and fulfillment errors.
- Longer processing time during peak demand.
- Limited automation of inventory retrieval.
- Higher operational and labor costs.
- Poor scalability with increasing order volumes.
- Inconsistent performance due to human fatigue and errors.

---

## Proposed System

**DarkViz** is an autonomous omni-directional mobile robot designed to automate product retrieval in quick-commerce dark stores.

### How It Works

The robot uses **LiDAR-based SLAM** to map the dark store and determine its position. Once a product location is assigned by the inventory system, the robot autonomously navigates to the required shelf using **path planning and omni-directional motion**. A **motorized tray mechanism** then extends to retrieve the product and carries it to the designated order-fulfillment area.

### Major Components

* Omni-directional drive system
* LiDAR and SLAM
* ROS 2-based navigation and control
* Motor drivers and DC motors
* Automated tray retrieval mechanism
* Onboard processing unit
* Dark-store simulation environment

### Expected Benefits

* Reduced manual intervention
* Faster and more accurate product retrieval
* Reduced picking errors
* Consistent operation
* Foundation for a fully automated dark-store system


---

## System Architecture

Add block diagram or system architecture image here.

```markdown
![System Architecture](images/system_architecture.png)
````

Briefly explain the architecture.

---

## Hardware Requirements

| Sr. No. | Component | Specification | Quantity | Purpose |
| ------- | --------- | ------------- | -------- | ------- |
| 1       |       TB6612FNG motor driver module     |   maximum voltage of 15VDC             1.2A per channel (or up to 3.2A for a short, single pulse)            |      2    |    To control the speed and direction of the robot’s DC motors by receiving control signals from the microcontroller and supplying the required current to each motor.     |
| 2       |     Pro-range Johnson motor      |      High Torque DC Motor 12V 600RPM             encoder compatible        |     3     |    To provide the required torque and rotational motion for the robot’s omni-directional wheels, enabling controlled movement in different directions.     |
| 3       |    Omni Wheels      |       Body material: Aluminium          Diameter: 60 mm.  Load capacity: 3 kg   Number of rollers: 10    Roller material: Rubber  Number of plates: 6 |    3      |    To provide omni-directional movement, allowing the robot to move forward, backward, sideways, and rotate in place for precise navigation within the dark store.     |
| 4       |     ESP WROOM 32 MCU Module Version: 1.1 microcontroller      |        generic WiFi-BT-BLE MCU module       |      1    |    To serve as the robot’s main microcontroller, processing control commands and managing motor drivers, sensors, and communication for autonomous robot operation.     |
| 5       |     raspi 4 model B      |        Broadcom BCM2711 Quad-core 64-bit Cortex-A72 @ 1.8 GHz, 4 GB LPDDR4 RAM, Wi-Fi, Bluetooth 5.0, Gigabit Ethernet, 40-pin GPIO       |      1    |    Main onboard computer for running ROS 2, SLAM, navigation algorithms, sensor processing, and high-level robot control     |
| 6       |     Battery      |       8000mah       |      1    |    To provide the primary power supply for the robot’s motors, microcontrollers, sensors, and other electronic components.     |
---

## Software Requirements

| Sr. No. | Software / Tool | Version | Purpose |
| ------- | --------------- | ------- | ------- |
| 1       |        Gazebo/Ignition         |   Gazebo Fortress      |     Simulation of the robot, sensors, environment, and navigation    |
| 2       |        RViz2        |    ROS 2 Humble     |    Visualization of SLAM maps, robot pose, LiDAR data, TF, and navigation     |
| 3       |        Ubuntu         |    22.04 LTS     |    Operating system for running ROS 2, Gazebo, RViz2, and robot software     |

---

## Technologies Used

- ROS 2 Humble – Robot software framework
- Python / C++ – Algorithm development and robot control
- SLAM – Indoor mapping and localization
- Nav2 – Autonomous navigation and path planning
- Gazebo – Robot and environment simulation
- RViz2 – Visualization and monitoring
- Raspberry Pi 4B – Main processing unit
- ESP32 – Motor and hardware control
- LiDAR – Environment sensing and mapping
- Autodesk Fusion – 3D CAD design of the robot and tray mechanism
- Embedded Systems – Motor drivers, motors, sensors, and actuator integration

---

## Methodology

Explain the step-by-step approach.

1. Literature survey
2. Problem identification
3. Requirement analysis
4. System design
5. Hardware/software development
6. Integration
7. Testing and validation
8. Documentation and publication

---

## Project Timeline

| Week / Month | Task Planned          | Status                            |
| ------------ | --------------------- | --------------------------------- |
| Week 1       | Problem finalization  |           Completed               |
| Week 2       | Literature survey     |           Completed               |
| Week 3       | Requirement analysis  |           Completed               |
| Week 4       | System design         |           in progress             |
| Week 5       | Prototype development |           in progress             |
| Week 6       | Testing               |           pending                 |
| Week 7       | Documentation         |           in progress             |
| Week 8       | Paper writing         |           pending                 |

---

## Weekly Progress Updates

Students must update this section every week.

| Week   | Date | Work Completed | Work Planned for Next Week | Issues / Challenges | GitHub Commit Link |
| ------ | ---- | -------------- | -------------------------- | ------------------- | ------------------ |
| Week 1 |      |                |                            |                     |                    |
| Week 2 |      |                |                            |                     |                    |
| Week 3 |      |                |                            |                     |                    |
| Week 4 |      |                |                            |                     |                    |
| Week 5 |      |                |                            |                     |                    |
| Week 6 |      |                |                            |                     |                    |
| Week 7 |      |                |                            |                     |                    |
| Week 8 |      |                |                            |                     |                    |

---

## Design Files

Upload and link all design files here.

| File Type       | File Name / Link | Description |
| --------------- | ---------------- | ----------- |
| CAD Model       |                  |             |
| Circuit Diagram |                  |             |
| PCB Design      |                  |             |
| Flowchart       |                  |             |
| Simulation File |                  |             |

---

## Circuit Diagram

Add circuit diagram image here.

```markdown
![Circuit Diagram](images/circuit_diagram.png)
```

---

## Flowchart / Algorithm

Add flowchart image here.

```markdown
![Flowchart](images/flowchart.png)
```

### Algorithm

1. Start
2. Initialize the system
3. Read input from sensors/user
4. Process the data
5. Generate output/control action
6. Display/store/transmit result
7. Stop

---

## Implementation Details

Explain the actual implementation of the project.

### Hardware Implementation

Write details about connections, components, power supply, sensors, actuators, PCB, enclosure, etc.

### Software Implementation

Write details about code structure, libraries used, algorithms, communication protocols, database, app, cloud, etc.

---

## Code Structure

```text
BE-Capstone-Project/
│
├── README.md
├── docs/
│   ├── literature_survey.md
│   ├── project_report.pdf
│   └── presentation.pptx
│
├── hardware/
│   ├── circuit_diagram.png
│   ├── pcb_design/
│   └── cad_model/
│
├── software/
│   ├── src/
│   ├── include/
│   └── tests/
│
├── images/
│   ├── system_architecture.png
│   ├── prototype_photo.jpg
│   └── results.png
│
└── references/
    └── papers/
```

---

## How to Run the Project

### Step 1: Clone the Repository

```bash
git clone https://github.com/username/project-name.git
```

### Step 2: Install Dependencies

```bash
pip install -r requirements.txt
```

or mention specific software/library installation steps.

### Step 3: Upload / Run the Code

```bash
python main.py
```

or

```bash
arduino-cli upload -p COMx --fqbn board_name
```

### Step 4: Observe the Output

Mention the expected output of the project.

---

## Testing and Results

| Test No. | Test Description | Expected Result | Actual Result | Status      |
| -------- | ---------------- | --------------- | ------------- | ----------- |
| 1        |                  |                 |               | Pass / Fail |
| 2        |                  |                 |               | Pass / Fail |
| 3        |                  |                 |               | Pass / Fail |

---

## Result Images / Videos

Add images or videos of the working prototype.

```markdown
![Prototype](images/prototype_photo.jpg)
```

Video Link:

```markdown
[Project Demo Video](https://drive.google.com/your-video-link)
```

---

## Applications

Mention real-world applications of the project.

1.
2.
3.
4.

---

## Advantages

1.
2.
3.
4.

---

## Limitations

1.
2.
3.
4.

---

## Future Scope

Mention possible improvements.

1.
2.
3.
4.

---

## Research Paper / Publication

| Item                      | Details                                                   |
| ------------------------- | --------------------------------------------------------- |
| Paper Title               |                                                           |
| Conference / Journal Name |                                                           |
| Paper Status              | Not Started / Drafting / Submitted / Accepted / Published |
| Submission Date           |                                                           |
| Paper Link                |                                                           |

---

## References

Add references in IEEE format.

Example:

```text
[1] A. Author, B. Author, "Title of the Paper," Journal/Conference Name, vol. X, no. Y, pp. xx-yy, Year.
[2] Datasheet / Website / Book reference.
```

---

## Repository Update Guidelines

Each student team must update the GitHub repository regularly.

Minimum expected updates:

* Update README every week.
* Push code changes regularly.
* Upload circuit diagrams, CAD files, PCB files, reports and presentations.
* Add weekly progress in the progress table.
* Maintain proper folder structure.
* Do not upload unnecessary temporary files.
* Each major update should have a meaningful commit message.

Example commit messages:

```text
Added problem statement and objectives
Updated system architecture diagram
Added sensor interfacing code
Updated weekly progress for Week 3
Added testing results and prototype images
```

---

## Declaration

We declare that this project work is carried out by our team as part of the BE Capstone Project. The work will be regularly updated on GitHub and all references used will be properly cited.

---

## License

This project is for academic use only.

Optional:

```text
MIT License / Creative Commons / Institute Use Only
```

```
```
