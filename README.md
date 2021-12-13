# Github Repo:

https://github.com/MohamedEl-Naggar/Contactless-Elevator

# Group Members:

* Mohamed El-Naggar
* Mohamed Barakat
* Abdelrahman Anis

# Description:
Most existing elevators use buttons which contact the user's skin directly. So, Elevator buttons accumulate many viruses & bacteria due to frequent contact with different people. We propose a new contactless elevator user interface to minimize the transmission of viruses & bacteria. We will utilize infrared sensors to implement touchless elevator buttons.

# Components:

## **Hardware**

* STM32L432KC MCU
* 6 LEDs 
* 6 Sensors (Infrared Object Detection Sensors FC-51) 
* Buzzer

## **Software**

* Keil uVision
* Stm32CubeMX
* Teraterm

# Flowchart:
![](https://github.com/MohamedEl-Naggar/Contactless-Elevator/blob/main/Blank%20diagram.png)

# Code Structure:
## **FreeRTOS**
* Real-time System
* Task Synchronization

## **Stm configuration**
![](https://github.com/MohamedEl-Naggar/Contactless-Elevator/blob/main/Capture.PNG)

## **Circuit**
![](https://github.com/MohamedEl-Naggar/Contactless-Elevator/blob/main/circuit.jpeg)

## **Tasks**
request_elevator():

* Manages the external requests (elevator requests at the different floors)
* If a hand is placed in front of the sensor for 2 seconds or more, it generates a request for the elevator
* Handles outputs that indicate that the requests were fulfilled

request_floor(): 

* Manages the internal requests (floor requests from inside the elevator)
* Validates users' requests:
* * Warns the user if their hand was placed in front of two sensors at the same time
* * If input is valid, it generates a request and output indicating the request was fulfilled

handle_requests(): 

* Receives the generated requests
* Decides which request to be fulfilled first based on our scheduling algorithm
* Sends signal for the elevator to move or to open the doors

[Prototype Demo](https://drive.google.com/file/d/1NlmrpBbz8J_54Y5JU7PmCWxHZi3i8n57/view?usp=sharing)

[Slides](https://docs.google.com/presentation/d/1RgeUVc1pOAIswsxYElLA5HQPsSrmf2Wv/edit?usp=sharing&ouid=116938565731541364527&rtpof=true&sd=true)

References:

[stm32 datasheet](https://www.st.com/resource/en/datasheet/stm32l432kc.pdf)
