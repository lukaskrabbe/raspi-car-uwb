# Raspberry Pi based Car - UWB

This Repository contains all necessary Code to control a Raspberry Pi based Micro Car, the Car uses a UWB Tracker (DWM1001) for Localization.

***Master project - Networked and Distributed Systems*** 

***Participants:*** 
 - Lukas Krabbe (stu212947@mail.uni-kiel.de) 
 - Birkan Denizer (stu219500@mail.uni-kiel.de)

***Supervisor:***
 - Patrick Rathje (pra@informatik.uni-kiel.de)

# Structure:

There are 3 Folders, containing the Code for the UWB-Trackers and the Raspberry Pi.

 1. dwm_ses_ss_dwr_init 

	In this Folder are all necessary Files to deploy the Initialization Programm on the DWM1001 UWB Tracker. The Initialization Programm is deployed on the UWB Tracker on the Raspberry Pi Car. 
	We used the [example Programm](https://github.com/Decawave/dwm1001-examples) from Decawave as a reference. 
	The main.c is located under: raspi-car-uwb/dwm_ses_ss_dwr_init/examples/ss_twr_init


 2. dwm_ses_ss_dwr_resp

	In this Folder are all necessary Files to deploy the Response Programm on the DWM1001 UWB Trackers. The Response Programm is deployed on the UWB Tracker which are used as a Anchor, each Anchor has a unique ID and a specified known Position. 
	We used the [example Programm](https://github.com/Decawave/dwm1001-examples) from Decawave as a reference.  
	The main.c is located under: raspi-car-uwb/dwm_ses_ss_dwr_resp/examples/ss_twr_resp

 3. raspi-car

	In this Folder are all necessary Files to deploy the Calculation and the Driving Process on the Raspberry Pi 4. The Raspberry  computes it Position using the Distances between the Onboard UWB Tracker and the UWB Anchors.

 4. evaluation

	Here you will find all Data of the Distance Evaluation.

## Template of the Project:

![Project Template](https://lh3.googleusercontent.com/pw/ACtC-3fmIxg5sEALCZ-G2ZWRMsrrNV0WP4xTR5Qs9F1FyXGjCN-QWdw8M_ZVp1TBhg-wNzOzc6PYAg8DhK9kOoxC8Wm2sO4CxqIhFGO1gzop6mvF46aKSrhn-PMnk4emC00LF-lLwt9bZjWACr09JLoQ-dpmPg=w1842-h1224-no?authuser=0)

Der Pojektaufbau ergibt sich aus mehrern Meilensteinen:
|Milestone|Deadline|Titel| Optional | Goal | Done |
|--|--|--|--|--|--|
| 1 |KW 01|Distance Measurment + Evaluation||Distance measurement with the UWB Tracker|Done
|2|KW 03|Localization with Anchors||Localization of a UWB module with at least three (fixed) anchors (or more)|Done
|3|KW 04|Driving the Car based on Position||The car drives autonomously, using its own position|Done
|4|KW 05 - KW 06|Evaluation||Evaluation of our results|Done
|5|KW 08|Movable Anchors|Optional||

We planed a little Buffer in the End, the final Presentation will be around KW 10 - KW 12.
***Milestones 5 is Optional,*** we have not decided if and how we tackle them.


## Comments on the Milestones:

Here are some Comments and Infos of each Milestone, but of Cause it represent NOT all Minds and Problems we dicussed and solved!

### Distance Mesaurment + Evaluation:
- The Connection between the UWB Tracker and the Raspberry was one of our first Challenges, we achived this by using the Min Protokol (https://github.com/min-protocol/min). This Protocol gives us the Possibility to send Frames over the Serial Port.
- The Evaluation showed us that the DWM1001 has a small inaccuracy (around 8-10%) from factory. But we where able to calibrate the antennas and reduce the inaccuracy to 4 %. Also a Bias helped us to improve our Results furthermore.
- The Evaluation is palced in the evaluation/ folder.

### Localization with Anchors:
- We implemented a 3-dimensional Localizaiton with a Localizaion Python Package (https://github.com/kamalshadi/Localization) which uses the Least Squeare Error Solver
- To improve our Localizaion, esapcially during the Driving, we added a Kalmann Filter (https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html)
- The Number of Anchor-Nodes, which are used for the Localizaion is variable, also the number of Distances to one Anchor can be choosen flexible (1 is recomended). To represent this nested Structure we implemented the distance_data.py.

### Driving the Car based on Position
- We had a Problem to implemnt a continious Driving Process (the Car drove, stoped and calculated his new Position), therfore we added Multithreading to seperate the Driving and the Position Estimation (located in driving.py) 
- The Car which tries to reach the given FINAL_POSITON, but the given ERROR_RATE helps the car to be more flexible
- The Direciton Detection on the Car was a tricky Part of our Project, because UWB has a total Error Rate of +/- 10 cm. Therfore we let the Car drive a little bit forwared (for 1 Second) and calculate the Direction based on the old and new Position. Furthermore we thought about using two Antennas on the Car and implemented such a Version (Second Branch experimental-two-initiator). 
- The Car has a high failour Rate because of its construction and used engines, therfore we calibrated our Wheels to let the Car drive better forwared (Calibration-Values for both Wheels in driving.py) 

### Evaluation
- The Evaluation of our Driving was due to Corona limited, therfore you will find two Videos with diffrent Number of Anchors below.

### Movable Anchors
- Due to Time Reasons we didn't achived our optinal Milestone
- Therfore we spend more Time on the Driving Process and implemented for example the Kalman Filter or Multiple Antennas

## Outlook:
Sadly this Project had a limited amount of Time, but we had lots of Ideas to improve our Results furthermore.
For Example in the Future it would be interresting to spend some time to increase the Kalman Filter or to implement the Double Sided Two Way Ranging Protocol on the Decawave UWB modules. Or we thought about using Machine Learning to drive the Car or of Cause impelement Movable Anchors.

## Videos:
### 3 Anchors and 15 cm Error Rate:
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/l350i6ac3N8/0.jpg)](https://www.youtube.com/watch?v=l350i6ac3N8)

### 7 Anchors and 15 cm Error Rate:
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/AmkS3YYmF2s/0.jpg)](https://www.youtube.com/watch?v=AmkS3YYmF2s)

## Presentations:
## Mid-Term Presentation 
https://docs.google.com/presentation/d/1CROpJXQXtrV8CjuFPcOnOhdHkz81XQL2lHxxf0xJW-Y/edit?usp=sharing
## Final Presentation
https://docs.google.com/presentation/d/1gTdxQjcSCDWhJXh9naOBoXHbLK5728GzqHpVrQFZ8wQ/edit?usp=sharing
