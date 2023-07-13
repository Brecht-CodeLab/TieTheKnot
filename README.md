# Tie the Knot
### Team: Olivier Deforche, Timon Huysse, Brecht Vandekerkhove
### Abstract
In this first proposal, we want to take a closer look into the task of tying a knot. Knots are used in various industries and sectors. In the medical sector, knots are used to end sutures. For very small or accurate knots, the experience of a doctor may not suffice. In the ship industry loops have to be made at the end of thick steel cables. These loops have to be knotted as well. This requires several people to make these knots. Something worth trying with a robot as well. Tying the knots of shoes seems very obvious and simple to most human beings. However, for disabled people who like to wear shoes with shoelaces, the task of putting on shoes and tying the knots seems an interesting problem to tackle.



## Proposal
### End user requirement:
- Tying a knot made from x number of ropes
  - Which type of rope?
  - How many ropes?
  - What type of knot?
	- depends on:
    - Petri net: people input on tablet
    - How many ropes
    - Obstacles

### Central Components:
- Workspace with flat horizontal surface
- Rope(s)
- (Beam to attach the rope to)
- Possible disturbances:
  - Initial configuration of the rope
  - Unnecessary objects in workspace

### Object Features:
- Object sensor features:
  - Position of rope(s)
  - Already a knot in the rope(s)
- Object task features:
  - Position of arm and grippers
  - Position of rope(s)

### Tasks:
- Flags:
  - Beam present
  - Rope already knotted
  - Rope is untied
  - Rope is tied
  - ...
- FSM:
  - (Detect beam)
  - Detect ropes
  - Detect ends of rope
  - Untie rope
    - Detect end of one knotted rope
    - Pull on the knot
    - ...
  - Ropes are untied
  - Sort ropes
  - Position rope on surface
  - Move to rope
  - Grab rope
  - Picke up rope
  - Position rope in space
  - Tie rope
  - Pull rope
  - Put rope down
  - ...
- Switching between states
  - If the number of ends of ropes are detected (aka the number of ropes) --> program should go to position detection of ropes
  - If the position detection is done (visually) --> Are ropes knotted
  - If ropes are knotted (visually or force confirmed) --> untie ropes
  - If ropes are untied (visual and force confirmation) --> sort ropes
  - If ropes are sorted (visual confirmation of order of ropes) --> move to one rope
  - If arrived at rope (visually or force sensor) --> pick up rope
  - If rope is picked up (visul confirmation) --> tie knot
  - If rope is tied (force and visual confirmation) --> put rope down
  - ...

- Petri nets:
  - Move to rope (position control) --> grab rope (impediance)
  - Rope knotted (position control) --> Tie rope (impedance control)
  - Tie rope (impedance control) --> put rope down (position control)
  - ...

### Perception:
- Encoders in arms (positioning)
- Force sensors (Strenght of knot)
- One camera:
  - Can work in combination with force sensor to pick up rope
- Dual camera:
  - Can be usefull for localization of ropes in 3D to pick it up

### Control:
- Position control (grab ends op ropes)
- Impedance control (to tighten the rope/knot)

## Diagram
![no picture](diagram.png)

# MVP
For us, the MVP can be split up into two parts:
1) Detection, representation and localisation of the rope
2) Picking up the rope
3) Making a simple loop/knot in the rope

## Gripper design
- a standard two finger gripper can be used for both the arms

## Visual sensing
- there will be one (mono) camera on a static position in space

## Assumptions
### 1) Detection, representation and localisation 
- There is a clear color difference between the rope and the respective background (table)
- The camera image is never disturbed

### 2) Picking up the rope
- The rope is already in the perfect position to initiate the knot tying phase (in a straight line, somewhat perpendicular to the Robotic arms, without âny already existing knots)

### 3) Tying the knot
- Only one type of loop/knot will be made
- There are no obstacles in the workspace

## World and rope representation
The rope can be represented as a discretized set of points with coordinates to it. The gripper can be determined by boxes arount it. The detection of the rope would be by a clear color difference with the table and environment. The gripper by border/shape detection.
The table can be determined by the camera in x and y direction and through impediance control of the gripper in the z direction. The table will be represented through the localization of the 4 corners in the reference frame.

![no picture](world_representation.jpg)
![no picture](rope_representation.jpg)



## Activities
These are the first drafts of the activities for the presented MVP.
### Arm Activities
![no picture](arm_activities.png)
### Camera Perception Activities
![no picture](camera_activities.png)

## Possible extensions (ordered by priority)
### 1) Detection, representation and localisation 
1.1 Optimise colour differences
1.2 Introduce edge detection to combine with colour detection 
1.3 Use shape detection algorithms to combine with previous techniques

### 2) Tying the knot
2.1 Other types of knots (can either be specified by the user or by sme other facter: e.g., type of rope, some feature in the surroundings,...)
2.2 Introduce multiple ropes

### 3) Preperation rope
A third phase gets introduced, hapenning between the phase 1 and 2. Assuming the rope is already in an ideal position is very idealistic, therefore, this step will prepare the rope in the sense that it will position the rope ideally.

3.1 Noose(s) present in the rope
3.2 Knot(s) present in the rope


# Background context
Tying knots can be proven useful in several industries: it could be implemented to support elderly people getting dressed, performing robot surgery, and braiding steel cables for the ship industry and most importantly the cabling (electrical, water, gas ...) of various industrial sites. For most problems, several approaches were already tested, resulting in one (up until this point) optimal solution method. 

1) In al types of facilities, factories, server rooms, ... cables and water/gas hoses have to installed. Doing this by hand is quite a job and requires several manhours. To simulate this application, we think detection a rope, picking it up, and tying a knot is an easier version of the intended practice.

2) Robotic arms that can tie shoelaces are already available. However, it is not yet proven to be economically beneficial. Currently, only when the shoe is placed in certain positions, or the laces are already put into the 'hands' of the robot, the outcome is successful (in most cases). Although the problem is interesting, given the size of the available equipment, researching tying shoes seems less valuable. Obtaining the correct motion, however, in a scale model can still prove useful.

3) Performing robotic surgeries is already wildly researched, as it introduces several advantages (from overall higher precision to fewer perturbations on the movement, ...). Again, the available material is not effective for this problem. But the behavior of tying the knot(s) can still be researched, assuming the robot arms are seen as a scaled setup, enlarging errors. 

4)  Interweaving steel cables for the shipping industry is currently limited to only one solution and is done manually at this moment. As this still can be researched, trying to tie such a knot could be very challenging. Up until this point, no research on this subject was found.

# Scources
### Ship steel cable knots:
- https://www.youtube.com/watch?v=eRaMG2RyhvI
- https://www.youtube.com/watch?v=jsfll_7CdEQ

### Surgical knots
- https://www.youtube.com/watch?v=2iXq_IU4c6c&ab_channel=AugustusGleason
- https://www.youtube.com/watch?v=MzYnRr7900s&t=79s&ab_channel=Dr.R.K.Mishra
- https://www.youtube.com/watch?v=wEalUB2Oce4&ab_channel=MauriceZwart

### Shoe tying
- https://www.youtube.com/watch?v=xSCCmPc5yvc&ab_channel=EricLi
- https://www.youtube.com/watch?v=erNi07dH5pw&ab_channel=QuantuMope

### Differte types of knots with double arm robot
- https://www.youtube.com/watch?v=AJllD3AiSqs&ab_channel=PieterAbbeel
- https://rlab.cs.dartmouth.edu/research-tying/index.html
- https://www.researchgate.net/publication/254041086_A_new_strategy_for_making_a_knot_with_a_general-purpose_arm
