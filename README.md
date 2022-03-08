![just front cropped again](https://user-images.githubusercontent.com/33253725/157230167-9e78a972-df2e-4d81-b1e7-94a75708e23d.png)
# StarControl
This project is about some lighting for my mercedes-benz. 
It initially started with an illuminated star in the front grill, which I wanted to control automated or manually via an AP and a html page. 
Later I decided to add some underglow LEDs, therefore I added two additional MCUs. One ESP8266 inside the cabin to quickly switch between different preset modes without the need of visiting the html page. The other MCU is controlling the 600 WS2812 digital LEDs attached to the underbody of my car.

I started this project in early September of 2021 (I had four boring weeks of vacation from my job as starting electronics engineer ;)) and finished the only "star-controlled" version in this timeframe. In late December I started thinking about how awesome an additional underglow would be on my car and that beeing the case decided to continue my work on this project. So I ordered the needed parts to start working again during my christmas vacation. Sadly, there were only two weeks fully packed with christmas meetings and new-year preperations to work on my new concepts. That's why I didn't finish everything and am currently still working on this project :)

## MCUs
I currently have 3 MCUs to control everything
### StarController | ESP32:
It's the main controller which's task it is to initally connect to the "Emergency-Client" via WiFi and establish a MQTT session. Afterwards it is exposing a html page and computes the inputs, which are made online. The StarController is also connected to external hardware like some MOSFETs to control the illuminated Star and Daylight-Running-Lights. Via the second hardware-serial ports it communicates with the "Underglow-Client" and transmitts newly selected modes, colors, etc, which are then applied to the underglow LEDs. It is mounted in the fuse box inside the motor compartment of my Mercedes-Benz C-Class (W204 / 2011).
### Emergeny-Client | ESP8266:
The main purpose of this controller is to give the driver the ability to quickly switch off the "additional lights" in case there is a "suspicious vehicle" nearby, because of german traffic laws, which not really intend in having an underglow equipped on your car - especially while your motor is running :). This chip is more like a hardware quick-access controller. It's simply equipped with a big switch that can adopt 4 different states and therefore enables the user to select from four different modes which are applied to the Star, DRLs and Underglow-LEDs. Currently I added a blackout-, html-controlled-, favorite- and strobe-mode to select from. The emergency client also serves as MQTT-Broker and therefore it also exposes the WiFi-AP. And yes...why is the ESP32 not the AP? Simply because of stability problems and the lack of a quality MQTT-broker library for ESP32 MCUs. Hence, the "StarController" will connect to the "Emergency-Client" and exchange data, like what's the currently selected mode. The "Emergency-Client" is mounted below my steering wheel to quickly access the switch.
### Underglow-Client | ESP8266:
This MCU was added, because the ESP32 had severe problems during my hardware tests with pushing the data for the WS2812 LEDs out. In several threads was reported this is because of the WiFi instance periodically interrupting other processes and therefore disturbs the transmission of the data stream to the LEDs. That's why I decided to an additional MCU wth the task of only controlling the 600 digital LEDs (which demands a lot computing power) without having to communicate over WiFi. Consequently I am relying on a serial-only-communication between the "Underglow-Client" and the "StarController" - so every MCU is focused on it's own tasks independently of each other :)

## Version 2.1
In this revision problems concerning LED flickering were adressed. The error's origin is the WS2812 chip's high level treshold voltage of 3.5V. Because of a not level adjusted data signal of 3.3V (StarClient UGLW - GPIO4 (D2)) there may occur issues like flickering, since the chip has a difficult time differentiating the 3.3V level between a 0 or 1. Therefore I added a 3.3V to 5V level shifter before contacting the LEDs.
StarClient Emergency also received a new status pixel consisting of a single WS2812 pixel to indicate different connection states and active modes.

![Schematic_Star-Control-V2-1_2022-03-08](https://user-images.githubusercontent.com/33253725/157222556-1298c5ad-3888-4ee7-94ff-8447ce703deb.png)

## Version 2.0
The new revision's major changes are the migration to the more performant ESP32 D1 Mini, aswell as the other two additional MCUs. StarClient UGLW handles the digital LEDs, while StarClient Emergency is located in the driver's cabin to expose a WiFi-AP and is further more equipped with a 4 mode slider to easily switch between different light scenes.

![Schematic_Star-Control-V2_2022-01-19](https://user-images.githubusercontent.com/33253725/150177973-799380a1-2141-4049-8bb5-3796c29a30c9.png)

## Version 1.0
This is a circuit diagramm from the older version, in which only the StarController existed which controlled the Star and DRLs.

![Schematic_Star-Control_2021-10-06](https://user-images.githubusercontent.com/33253725/149627361-69b01865-dca2-4f18-b78a-5a95abb0b29b.png)

## Battery concept & Vehicle wiring  
All of the above controllers relys on the power of my vehicle's battery. That beeing the case, I created a concept on how to power the whole circuitry without draining the battery while the vehicle is sleeping.
I decided to only power the MCUs if the car wakes up and therefore activates relay clamp 15. Additionally, there are clamp 15R and the starter clamp 50 which I use as input signals to compute the state of the motor (running or not). If the motor gets started clamp 50 is pulled high. Clamp 15R remains high until the key is rotated to state 15 in which the motor switches off again. Besides, I am also reading the DRLs state, because I decided to control them myself with 2 MOSFETs. Unfortunately, I ran into the problem that the car's ECU notices the lack of current draw on the original DRL lines and therefore displays a warning message in my instrument cluster. I solved this problem by choosing accordingly dimensioned power resisitors, connecting the DRL lines via these resistor to GND and finally converting the ECUs output current into heat, which is then safely spreaded by using the power resistors' metal heatsinks. Below is a rough concept from the early development stages of this project.

![Flowchart](https://user-images.githubusercontent.com/33253725/149628122-031fb700-c198-4e6d-90bc-a2dc74feb59b.png)
