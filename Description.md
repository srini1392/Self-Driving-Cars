Author : [Srinikethan Mankala
](sriz1392@gmail.com)
#  *Self Driving Cars* 
<br></br>

<H1> Course 1 : Introduction to Self driving Cars</H1>

## Glossary of Terms

* __ACC: Adaptive Cruise Control__</br>
    A cruise control system for vehicles which controls longitudinal speed. ACC can maintain a desired reference speed or adjust its speed accordingly to maintain safe driving distances to other vehicles.

* __Ego__</br>
    A term to express the notion of self, which is used to refer to the vehicle being controlled autonomously, as opposed to other vehicles or objects in the scene. It is most often used in the form ego-vehicle, meaning the self-vehicle.

* __FMEA: Failure Mode and Effects Analysis__</br>
    A bottom up approach of failure analysis which examines individual causes and determines their effects on the higher level system.

* __GNSS: Global Navigation Satellite System__</br>
    A generic term for all satellite systems which provide position estimation. The Global Positioning System (GPS) made by the United States is a type of GNSS. Another example is the Russian made GLONASS (Globalnaya Navigazionnaya Sputnikovaya Sistema).

* __HAZOP: Hazard and Operability Study__</br>
    A variation of FMEA (Failure Mode and Effects Analysis) which uses guide words to brainstorm over sets of possible failures that can arise.

* __IMU: Inertial Measurement Unit__</br>
    A sensor device consisting of an accelerometer and a gyroscope. The IMU is used to measure vehicle acceleration and angular velocity, and its data can be fused with other sensors for state estimation.

* __LIDAR: Light Detection and Ranging__ </br>
    A type of sensor which detects range by transmitting light and measuring return time and shifts of the reflected signal.

* __LTI: Linear Time Invariant__</br>
    A linear system whose dynamics do not change with time. For example, a car using the unicycle model is a LTI system. If the model includes the tires degrading over time (and changing the vehicle dynamics), then the system would no longer be LTI.

* __LQR: Linear Quadratic Regulation__</br>
    A method of control utilizing full state feedback. The method seeks to optimize a quadratic cost function dependent on the state and control input.

* __MPC: Model Predictive Control__</br>
    A method of control whose control input optimizes a user defined cost function over a finite time horizon. A common form of MPC is finite horizon LQR (linear quadratic regulation).

* __NHTSA: National Highway Traffic Safety Administration__</br>
    An agency of the Executive Branch of the U.S. government who has developed a 12-part framework to structure safety assessment for
    autonomous driving.The framework can be found here. </br>
    https://www.nhtsa.gov/sites/nhtsa.dot.gov/files/documents/13069a-ads2.0_090617_v9a_tag.pdf

* __ODD: Operational Design Domain__</br>
    The set of conditions under which a given system is designed to function. For example, a self driving car can have a control system designed for driving in urban environments, and another for driving on the highway.

* __OEDR: Object and Event Detection and Response__</br>
    The ability to detect objects and events that immediately affect the driving task, and to react to them appropriately.

* __PID: Proportional Integral Derivative Control__</br>
    A common method of control defined by 3 gains.

    1) A proportional gain which scales the control output based on the amount of the error

    2) An integral gain which scales the control output based on the amount of accumulated error

    3) A derivative gain which scales the control output based on the error rate of change

* __RADAR: Radio Detection And Ranging__</br>
    A type of sensor which detects range and movement by transmitting radio waves and measuring return time and shifts of the reflected signal.

* __SONAR: Sound Navigation And Ranging__ </br>
    A type of sensor which detects range and movement by transmitting sound waves and measuring return time and shifts of the reflected signal.




## Taxonomy

1. <H3>Driving task</H3>  </br>   

    ___Perceiving the environment___
    >>
        This includes tracking a car's motion in identifying the various elements in the world around us,
        like the road surface, road signs,vehicles, pedestrians and so on. 
        Need to track all moving objects and predict their future motions.
    >>


    ___Motion Planning___
    >>
        This allows us to reach our destination successfully. 
        For example you may want to drive from your home to your office. 
        So you'll need to consider which roads you should take,when you should change lanes or 
        cross an intersection and how to execute a swerve maneuver around a pothole along the way.
    >>


    ___Driving Vehicle itself___
    >>
        Appropriate steering, break and acceleration decisions to control the vehicle's 
        position and velocity on the road. 
    >>

    ___Operational Design Domain (ODD)___
    >>
        The ODD constitutes the operating conditions under which a given system is designed to function. 
        It includes environmental, time of day,roadway and other characteristics under 
        which the car will perform reliably.Clearly defining the operating conditions for which 
        a self-driving car is designed, is crucial to ensuring the safety of the system
    >>

    ___Classify Driving System Automation___
    >>
         -> Driver attention requirements
  
         -> What exactly makes up a driving task?
            1. Lateral control - task of steering laterally on the road.
               i.e, turning left, right,tackling curve etc.

            2. OEDR (Object & Event Detection & Response)- OEDR is essentially the ability to detect 
               objects and events that immediately affect the driving task and to react to them appropriately

            3. Planning - Is primarily concerned with the long and short term plans needed to travel
               to a destination or execute maneuvers such as lane changes and intersection crossings.

            4. Miscellaneous tasks -signaling with indicators, hand-waving,interacting with other drivers etc
    >>


    ___Levels of Automation___
    >>
         1. Level 0 - *No automation*
         2. Level 1 - *Driving Assistance* - Lateral or Logitudinal control 
                       (Ex. Adaptive Cruise Control, Lane Keep Assist)
         3. Level 2 - *Partial Automation Driving* - Lateral & Logitudinal control by system
                        (ex. GM super cruise, Nissan pro-pilot)
         4. Level 3 - *Condition Driving Automation* - Longitudinal, lateral and OEDR 
                        (Driver needs to take control when system fails)(Ex. Audi A8 sedan)
         5. Level 4 - *High Driving automation* - Lateral, longitudinal, OEDR, fallback s/m (Ex. Waymo)
         6. Level 5 - *Fully Autonomous* - All control by system, Lateral,longitudinal,OEDR, unlimited ODD
    >>


2. <H3> Perception Task </H3>

    In particular, for any agent or element on the road, we need to first identify what it is; a car, a cyclist, a bus, etc. And second, we want to understand its motion; has it been moving in a certain way that can tell us what it will do next. It's still difficult for computer systems to be able to recognize these same patterns around us as quickly. </br>
    >>
                     _ _ _ _ _ _ _ _ _ _ _       _ _ _ _ _ _ _ _ _ _ _
                    |  Analyze ego motion |     |  Decide on and      |
                    |       &             |     |  Plan a Maneuver    |
        Input--->   |   environment       |---> |   (planning)        | ---> Drive
                    |   (perception)      |     |                     |
                    |_ _ _ _ _ _ _ _ _ _ _|     |_ _ _ _ _ _ _ _ _ _ _|
    >>


    ___Various elements in perception___
    >>
        a. Static elements
           1. roads and lane markings
           2. zebra crossings
           3. off-road elements like curbs
           4. on-road traffic signals 
           5. road obstructions
           6. cones used while construction
    
        b. Dynamic elements
           1. Other vehicles (motorcycles, 4 wheelers, trucks etc.)
           2. pedestrians
    
        c. Ego Localization
           1. Position (gps)
           2. velocity, acceleration (imu)
           3. Orientation, angular motion (odometry sensors)
    >>

    ___Challenges to Perception___
    >>
        a. Robust detection and segmentation
        b. Sensor uncertainity
        c. Occlusion & reflection (camera data)
        d. Illumination, lens flare
        e. Weather & precipitation
    >>