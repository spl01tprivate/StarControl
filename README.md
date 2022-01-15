![Illuminated Star Cropped](https://user-images.githubusercontent.com/33253725/149628611-a457c843-b964-4c8b-be25-fd635a33d951.jpg)
# StarControl
This project is about some lighting for my mercedes-benz. 
It initially started with an illuminated star in the front grill, which I wanted to control automated or manually via an AP and a html page. 
Later I decided to add some underglow LEDs, therefore I added two additional MCUs. One ESP8266 inside the cabin to quickly switch between different preset modes without the need of visiting the html page. The other MCU is controlling the 600 WS2812 digital LEDs attached to the underbody of my car.

I started this project in early October of 2021 (I had four boring weeks of vacation from my job as starting electronics engineer ;)) and finished the only "star-controlled" version in this timeframe. In late December I started thinking about how awesome an additional underglow would be on my car and that beeing the case decided to continue my work on this project. So I ordered the needed parts to start working again during my christmas vacation. Sadly, there were only two weeks fully packed with christmas meetings and new-year preperations to work on my new concepts. That's why I didn't finish everything and am currently still working on this project :)

## MCUs
I currently have 3 MCUs to control everything
### StarController | ESP32:
It's the main controller which's task it is to initally connect with the "Emergency-Client" via WiFi and MQTT. Afterwards it is exposing the html page and computes the inputs made online. The StarController is also connected to external hardware like some MOSFETs to control the illuminated Star and Daylight-Running-Lights. Via the second hardware-serial ports it communicates with the "Underglow-Client" and transmitts newly selected modes, colors, etc. It is mounted in the fuse box inside the engine compartment of my W204 (2011).
### Emergeny-Client | ESP8266:
The main purpose of this controller is to give the driver the ability to quickly switch off the "additional lights" in case there is a police vehicle nearby, because of german traffic laws, which not really allow to have an underglow equipped on your car - especially while your motor is running :). This chip is more like a hardware quick-access controller. It's simply equipped with a big switch that can adopt 4 different states and therefore the user can select different modes which are applied on the Star and Underglow-LEDs. For example there is a blackout-, html-controlled-, favorite- and strobe-mode to select. It is the MCU to expose the WiFi AP because of stability issues combined with the MQTT Broker service. The "StarController" will connect to it and exchange data, like what's the currently selected mode. The "Emergency-Client" is mounted below my steering wheel to quickly access the switch.
### Underglow-Client | ESP8266:
This MCU was added, because the ESP32 had severe problems during my hardware tests with pushing the data for the WS2812 LEDs out. In several threads was reported this is because of the WiFi instance is periodically interrupting other processes and therefore disturbs the transmission of the data stream to the LEDs. That's why I decided to add another MCU to only control the 600 digital LEDs (which additionally demands a lot computing power) without having to do WiFi communication. Consequently I am relying on a serial-only-communication between the "Underglow-Client" and the "StarController" - so everyone concentrates on their tasks independently of each other :)

## Old Version
This is a circuit diagramm from the older version, in which only the StarController existed and controlled just the Star and DRLs. For the new version the only changes are the migration to the ESP32 as StarController and the newly added "Underglow-Part".

![Schematic_Star-Control_2021-10-06](https://user-images.githubusercontent.com/33253725/149627361-69b01865-dca2-4f18-b78a-5a95abb0b29b.png)

## Battery concept & Vehicle wiring  
Because all of this relys on the power of my vehicle's battery I made a concept on how to power the whole circutry without draining my battery if the car is sleeping.
I decided to only power the MCUs if the car wakes up and therefore acitvates relay clamp 15. Then there is clamp 15R and the starter clamp 50 which I use as input signals to compute the state of the enigine (running or not). If the motor gets clamp 50 is pulled high during the starting process and 15R remains high until the key is rotated to state 15 in which the engine switches off. I also read the state of the DRLs which I decided to control myself with 2 MOSFETs. Unfortunately I ran into the problem that the ECU of the car notices that the current draw on the original DRL lines is 0 mA if not connected to ground and therefore displays a warning message on my head unit. I solved this problem by choosing accordingly dimensioned high power resisitors with metal heatsinks to spread the heat safely. Attached is a rough concept from the early development states of this project.

![Flowchart](https://user-images.githubusercontent.com/33253725/149628122-031fb700-c198-4e6d-90bc-a2dc74feb59b.png)
