# **StackBot: An Applied, Multi-layer Swarm Exploration System**

Izzy Morales, Andy Navarro, and Douglas Lilly  
Professor Markus Nemitz  
Tufts University  
Spring 2025 – ME134: Robotics  
---

# **Problem**

Within the wide net of robotics applications, many lack an effective means to reach a certain elevation, perform a task, then descend back to the ground. This dynamic feature set becomes especially pressing in settings like shipping warehouses or topographically ranging terrains. Imagine a large warehouse, with aisle after aisle, each with shelves upon shelves or, imagine a rocky terrain, with tall, hard-to-climb walls. In either case, performing a task in those settings may soon become dangerous, if not impossible, for a human, due to physical limitations. As such, we aimed to develop a robotic swarm system that could tackle these challenges.

# **Approach**

To simulate these larger scale warehouse/ rocky terrain applications, we used 3 XRP robots to develop a miniature application. We aimed to develop a modular, stackable swarm where the 3 bots could perform the same set of tasks, regardless of the order in which they are stacked. We designed and 3D-printed custom, snap-on plates for the XRPs with indentations/ grooves for another XRP to be placed on top of it, in perfect vertical alignment. We also built an elevated wooden platform, set at the height of 2 XRPs with our custom plate, laying it with a complex curvature path. This line following operation is meant to demonstrate the potential for navigation in unknown terrains. Additionally, for our entire operation, we used MQTT topics for the robots to communicate between states. We also had a laptop set-up to monitor the MQTT messages being sent.

# **Results**

Upon initialization, the bots determine where they are in the stack. We attempted to implement a timing-based calculation for the bots to figure this out, but, due to the scope of this project, we opted to manually assign each of our bots a number, the lowest being the base and highest being the top. Again, due to the scope, we decided to manually stack the 3 bots upon initialization. To start the autonomous series of tasks, a user presses the button of the ‘top’ XRP. The stack then navigates to the elevated surface (through use of April tags and computer vision), prompting the ‘top’ robot to drive on the elevation. As the ‘top’ bot tackles navigation on the platform, the remaining stack (‘base’ and ‘middle’) are left to navigate to our designated pick-up spot at the diagonally opposite corner of the platform. The now 2-stack swarm implements a random-walk, wall following operation. Upon reaching the corner, the 2-stack swarm, using the ultrasonic wall detection, turns steadily until it detects the wall again. Following this wall until April tags are detected on the ground, the 2-stack turns and reverses into alignment against the platform. If the ‘top’ bot has already reached the edge of the platform, recognized via an April tag, it waits for that message from the 2-stack that it is ready to receive the ‘top’ bot. The top bot drives on the stack, and, finally, the full stack drives off, terminating the simulation.

We certainly faced our fair share of bugs, mostly dealing with the order of operations within our state system and having the bots effectively communicate state changes over MQTT and conditional arguments. Perfect alignment also posed some challenges, as the ‘base’ bot, upon restacking, may not perfectly align itself with the ‘top’ bot’s location on the table above. 

# **Impact**

Our swarm displays a coordinated, parallel state-based operation resulting in deployment, exploration, retrieval, and reintegration. In the aforementioned examples, our swarm system would navigate otherwise inaccessible elevated surfaces, expanding both the capabilities of robotics and for human users.

In the future, given more time, we would work to refine our April tag detection, implementing more control-based  logic for precise positioning. We would also build on the previously mentioned, role-assignment procedure so that each bot is fully autonomous and aware of its position in the stack. We would, perhaps, also expand on the navigation aspect, using computer visualization and accelerometry to map out the landscape of the unknown surface. Finally, we would also create some method for the robots to autonomously stack themselves, starting on the same elevation.

Our multi-layer, stackable swarm system opens up numerous applications in surveying, search and rescue (SAR), warehouse operations, and beyond, where each robot disperses, performs their task, regroups, and returns back with a meshed dataset from each of their individual ventures. Ultimately, our swarm would provide users the capabilities to do what has previously been too challenging or inaccessible.

# **Links**
Full demo video: https://drive.google.com/file/d/1CRqhzQtFp7dnBnA8Vf5r17e3jlT65QNt/view?usp=drive_link 

Restacking demo video: https://drive.google.com/file/d/1iocgk28Fq7gxm-3mMfZQ-_hEI1nxBUSk/view?usp=drive_link
