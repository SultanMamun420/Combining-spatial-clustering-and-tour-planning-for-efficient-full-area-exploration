# Combining-spatial-clustering-and-tour-planning-for-efficient-full-area-exploration
Combining spatial clustering and tour planning for efficient full area exploration - Robotica- Cambridge University Press 

Combining spatial clustering and tour planning for efficient full area exploration

Jiatong Bao1 , Sultan Mamun1, Jiawei Bao1, Wenbing Zhang2, Yuequan Yang3 and Aiguo Song4

1School of Electrical, Energy and Power Engineering, Yangzhou University, Yangzhou, China
2School of Mathematical Sciences, Yangzhou University, Yangzhou, China
3School of Information Engineering and the School of Artificial Intelligence, Yangzhou University, Yangzhou, China
4School of Instrument Science and Engineering, Southeast University, Nanjing, China
Corresponding author: Jiatong Bao; Email: jtbao@yzu.edu.cn
Received: 24 December 2023; Revised: 27 February 2024; Accepted: 28 May 2024


Keywords: mobile robot; autonomous exploration; viewpoint planner; spatial clustering; tour planning


Abstract

Autonomous exploration in unknown environments has become a critical capability of mobile robots. Many methods often suffer from problems such as exploration goal selection based solely on information gain and inefficient tour optimization. Recent reinforcement learning-based methods do not consider full area coverage and the performance of transferring learned policy to new environments cannot be guaranteed. To address these issues, a dual-stage exploration method has been proposed, which combines spatial clustering of possible exploration goals and Traveling Salesman Problem (TSP) based tour planning on both local and global scales, aiming for effi- cient full-area exploration in highly convoluted environments. Our method involves two stages: exploration and relocation. During the exploration stage, we introduce to generate local navigation goal candidates straight fromclusters of all possible local exploration goals. The local navigation goal is determined through tour planning, utilizing the TSP framework. Moreover, during the relocation stage, we suggest clustering all possible global
exploration goals and applying TSP-based tour planning to efficiently direct the robot toward previously detected but yet-to-be-explored areas. The proposed method is validated in various challenging simulated and real-world environments. Experimental results demonstrate its effectiveness and efficiency.


1. Introduction
   
Mobile robots are being increasingly deployed across a wide range of applications [1, 2], such as searchand rescue operations, hospital services, and office deliveries. In these scenarios, the robots are required to autonomously navigate and explore unknown environments in order to gather information and effi- ciently accomplish their tasks. For instance, search and rescue robots must rapidly and autonomously search through disaster-stricken areas, while hospital service and office delivery robots need to efficiently explore and map large and complex environments without the need for additional human intervention. Therefore, the autonomously exploration and navigation through unknown environments represent critical capabilities for mobile robots.
Conventional exploration methods typically involve detecting frontiers [3, 4], sampling viewpoints,and navigating the robot toward the viewpoint with the highest information gain. These methods often rely on either frontiers or randomly sampled viewpoints. Frontiers are special locations that separate explored areas from unexplored ones and were first introduced for autonomous exploration by Yamauchi et al. [4]. However, the challenge remains in how to detect frontiers and select optimal frontiers for exploration to maximize navigation efficiency and area coverage. On the other hand, most viewpoint-based approaches tend to be greedy, with viewpoints densely sampled around the robot and 


<img width="681" height="413" alt="image" src="https://github.com/user-attachments/assets/eda33f72-8d74-48bc-ba9c-a346c81f4227" />

Figure 1. Overview of our method. The example environment which the robot is required to explore is shown as a 2D map. Explored areas are depicted in blue, while unexplored regions are represented by white spaces. Our method comprises two stages: the exploration stage and the relocation stage. During the exploration stage, an Rapidly-exploring Random Tree is expanded within the local planning horizon, where each node serves as a viewpoint. Frontiers are identified from these viewpoints. The identified local frontiers are considered as possible exploration goals. They are clustered, with each cluster corresponding to a local navigation goal candidate. By using local tour planning, the optimal local navigation goal is selected for execution, considering the remaining goals as possible global exploration goals. Once no clusters in the local planning horizon remain, the robot switches to the relocation stage.
In this stage, all updated global exploration goals are clustered and employed for global tour planning. The robot is then guided toward the selected global navigation goal. These two stages are executed back and forth until no global exploration goals remain. maintained throughout the exploration process. Without proper tour planning, the robot may spend considerable time navigating back to viewpoints that are close to already explored regions. Therefore, both frontier-based and viewpoint-based exploration methods frequently encounter challenges such as relying exclusively on information gain [5] for the selection of navigation goals and not optimizing the exploration path effectively. This leads to reduced efficiency in exploration, particularly in highly convoluted environments. Recent reinforcement learning-based methods [6–8] focus on learning exploration policies and do not consider full area coverage. The performance cannot be guaranteed when transferring learned policy to new environments. In this paper, we propose a dual-stage exploration method that combines spatial clustering and tour planning to address these challenges. Instead of individually assessing the information gained for each potential exploration goal, spatially clustering all possible densely distributed exploration goals allows the robot to rapidly determine sparse navigation goals. Tour planning based on Traveling Salesman Problem (TSP) framework is executed on these sparse navigation goals, optimizing the robot’s path by considering place-to-place costs and reducing computation load by limiting the number of tour locations. Spatial clustering and tour planning are performed on both local and global scales. Fig. 1 offers an overview of our method. During the exploration stage, frontiers are identified from an Rapidly-exploring Random Tree (RRT) without any bias, all within the local planning horizon. These frontiers are always densely distributed and considered as possible local exploration goals. They are subsequently clustered and transformed into individual local navigation goal candidates. Planning tours based on the local navigation goal candidates and the robot’s home position allows for the rapid expansion of the exploration boundary. When no local exploration goal clusters remain, the robot transitions to the relocation stage. Our method again clusters all possible global exploration goals that have not been visited before and perform global tour planning. This empowers the robot to optimally navigate to different recognized but unexplored sub-areas. These two stages are repeated until no global exploration goals remain. In addition, a retrying mechanism is introduced to employ before the robot switches to the relocation stage from the exploration stage, in order to enhance the exploration robustness.


1.1. Contributions

Our contributions can be summarized as follows:

• We propose to combine spatial clustering and TSP-based tour planning technologies in both exploration and relocation stages.

• A retrying mechanism is suggested to enhance the robustness of exploration.

• Our code is open-source, allowing others to reproduce our results and conduct comparative analyses with various exploration techniques. 



3. Problem description
   
Let S ⊂ R3 be the full space being explored, which comprises of the known occupied space Socc, the known-free space Sfree, and the currently unknown space Sunk. The primary objective of the exploration task is to navigate autonomously within S and discover as much of the known space Socc ∪ Sfree as possible within a given time limit Tlim. The evaluation criteria for assessing the exploration performance include metrics such as area coverage (i.e., explored area volume, travel distance, and overall time) and exploration efficiency (i.e., explored area volume per second). In addition, the evaluation assumes precise robot localization and mapping, when measuring the exploration performance.



5. Our approach
   
As shown in Fig. 2, the proposed framework consists of several blocks of data structures and functions that can be classified into two stages: exploration and relocation. In the exploration stage, a local RRT is expanded within the free space of the environment, concurrently maintaining a local viewpoint graph. The identification of local frontiers is accomplished either by examining the graph vertices or the RRT nodes (Section 4.1.1). Additionally, these frontier are considered as possible exploration goals and further processed into individual clusters, yielding local navigation goal candidates (Section 4.1.2), which, in turn, serve as input for local tour planning (Section 4.1.3). The planned tour destination is taken as the navigation goal, with the local viewpoint graph facilitating the determination of the shortest path from the current robot position to the destination. When no clusters are within the current local planning horizon, the robot transitions to the relocation stage. In this stage, global exploration goals are clustered
(Section 4.2.1) and the global viewpoint graph is updated (Section 4.2.2). Each global exploration goal cluster is also represented by a navigation goal. All global navigation goal candidates are further fed into tour optimization to ascertain the next navigation goal (Section 4.2.3) for relocation. The robot transitions between the two stages until no navigation goals remain. Furthermore, to enhance the method’s robustness, a retrying mechanism is introduced, enabling the regeneration of the RRT for a repeated attempt at the exploration stage before transitioning to the relocation stage.


5. Experiments
   
We begin by evaluating the performance of our proposed method in various simulated environments and comparing it with the DSVP method [11], which is a challenging autonomous exploration method. Given that DSVP has already been compared with the state-of-the-art methods such as NBVP [19] and GBP [20], demonstrating its superior performance, we do not replicate those results in this paper. Additionally, we compare our method with the FAEL method [21], which also employs tour planning for exploration. Furthermore, we conduct ablation studies to investigate the necessity of specific key operations within our method. Finally, we validate our proposed method through testing on our mobile robot in real-world environments.


6. Conclusions
   
In conclusion, we have presented a dual-stage exploration method that prioritizes simplicity and effectiveness. The spatial clustering and TSP-based tour planning technologies are employed in both stages. In the exploration stage, possible local exploration goals are densely detected from RRT nodes and then spatially clustered. The sparse local navigation goal candidates are extracted from the clusters rather than calculating gains of all possible RRT branches. The local tour planner is further employed to select
optimal navigation goals for efficient exploration. In the relocation stage, the number of global exploration goals is also controlled by clustering and the global tour planning effectively guides the robot to revisit unexplored spaces. Our proposed retrying mechanism significantly enhances the success rate of exploration. To evaluate our approach, we conduct tests in five benchmark simulation environments and two real-world environments. Both the simulation and real-world tests demonstrate the effectiveness and efficiency of our method.
However, both DSVP and our methods have certain limitations. Firstly, the system heavily relies on mapping performance and the accuracy of robot localization. It may not perform as expected in open spaces, such as the outdoor areas on campus. Relying solely on laser scans may prove insufficient for detecting obstacles like road curbs or green belts that could pose a danger to the robot. To enhance terrain traversability analysis, integrating additional sensors, such as cameras, would be highly benefi- cial. Secondly, during the exploration stage, the local graph is utilized to plan the path from the current position to the target goal. Due to the sparse sampling of nodes in the local graph, the planned path
may not always be optimal. Furthermore, the planned path is not smoothed in accordance with the robot’s kinematics model. This can result in the robot making frequent heading adjustments, leading to increased power consumption, longer travel times, and posing additional challenges to the simultaneous localization and mapping process. Future work will be dedicated to addressing these identified shortcomings.


Author contributions. 
Jiatong Bao and Sultan Mamun designed and implemented the exploration method, verified its effectiveness, and wrote the paper. 
Jiawei Bao set up the simulated and real experimental environments. Wenbing Zhang and Yuequan Yang did the experimental analysis. Aiguo Song guided the progress and reviewed the paper.

 
Financial support. 
This research work is supported by the National Natural Science Foundation of China (Grant No. 61806175, 62073322).

 
Competing interests. 
The authors declare no competing interests exist.

Ethical standards.
Not applicable under the heading.


References

[1] M. B. Alatise and G. P. Hancke, “A review on challenges of autonomous mobile robot and sensor fusion methods,” IEEE
Access 8, 39830–39846 (2020).
[2] J. Delmerico, S. Mintchev, A. Giusti, B. Gromov, K. Melo, T. Horvat, C. Cadena, M. Hutter, A. Ijspeert, D. Floreano, L.
M. Gambardella, R. Siegwart and D. Scaramuzza, “The current state and future outlook of rescue robotics,” J Field Robot
36(7), 1171–1191 (2019).
[3] Z. Sun, B. Wu, C. Xu and H. Kong, “Ada-Detector: Adaptive Frontier Detector for Rapid Exploration,” In: International
Conference on Robotics and Automation (ICRA), (2022) pp. 3706–3712.
[4] B. Yamauchi, “A Frontier-Based Approach for Autonomous Exploration,” In: Proceedings 1997 IEEE International
Symposium on Computational Intelligence in Robotics and Automation CIRA’97. ’Towards New Computational Principles
for Robotics and Automation’, (1997) pp. 146–151.
[5] J. Liu, C. Wang, W. Chi, G. Chen and L. Sun, “Estimated path information gain-based robot exploration under perceptual
uncertainty,” Robotica 40(8), 2748–2764 (2022).
[6] J. Hu, H. Niu, J. Carrasco, B. Lennox and F. Arvin, “Voronoi-based multi-robot autonomous exploration in unknown
environments via deep reinforcement learning,” IEEE Trans Veh Technol 69(12), 14413–14423 (2020).
[7] N. Khlif, K. Nahla and B. Safya, “Reinforcement learning with modified exploration strategy for mobile robot path planning,”
Robotica 41(9), 2688–2702 (2023).
[8] Y. Xu, J. Yu, J. Tang, J. Qiu, J. Wang, Y. Shen, Y. Wang and H. Yang, “Explore-Bench: Data Sets, Metrics and Evaluations
for Frontier-Based and Deep-Reinforcement-Learning-Based Autonomous Exploration,” In: International Conference on
Robotics and Automation (ICRA), (2022) pp. 6225–6231.
[9] H. Umari and S. Mukhopadhyay, “Autonomous Robotic Exploration Based on Multiple Rapidly-Exploring Randomized
Trees,” In: IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), (2017) pp. 1396–1402.
[10] C. Wang, W. Chi, Y. Sun and M. Q.-H. Meng, “Autonomous robotic exploration by incremental road map construction,”
IEEE Trans Autom Sci Eng 16(4), 1720–1731 (2019).
[11] H. Zhu, C. Cao, Y. Xia, S. Scherer, J. Zhang and W. Wang, “Dsvp: Dual-Stage Viewpoint Planner for Rapid Exploration
by Dynamic Expansion,” In: IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), (2021) pp.
7623–7630.
[12] A. Dai, S. Papatheodorou, N. Funk, D. Tzoumanikas and S. Leutenegger, “Fast Frontier-Based Information-Driven
Autonomous Exploration with an Mav,” In: IEEE International Conference on Robotics and Automation (ICRA), (2020)
pp. 9570–9576.
[13] B. Zhou, Y. Zhang, X. Chen and S. Shen, “Fuel: Fast uav exploration using incremental frontier structure and hierarchical
planning,” IEEE Robot Automa Lett 6(2), 779–786 (2021).
[14] Q. Bi, X. Zhang, J. Wen, Z. Pan, S. Zhang, R. Wang and J. Yuan, “Cure: A hierarchical framework for multi-robot
autonomous exploration inspired by centroids of unknown regions,” IEEE Trans Autom Sci Eng 1–14 (2023).
[15] R. S. D. Muddu, D. Wu and L. Wu, “A Frontier Based Multi-Robot Approach for Coverage of Unknown Environments,”
In: IEEE International Conference on Robotics and Biomimetics (ROBIO), (2015) pp. 72–77.
[16] A. Soni, C. Dasannacharya, A. Gautam, V. S. Shekhawat and S. Mohan, “Multi-Robot Unknown Area Exploration Using
Frontier Trees,” In: IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), (2022) pp. 9934–9941.
[17] S. M. LaValle, Rapidly-exploring random trees : A new tool for path planning, (1998). The annual research report.
[18] Z. Meng, H. Qin, Z. Chen, X. Chen, H. Sun, F. Lin and M. H. Ang, “A two-stage optimized next-view planning framework
for 3-d unknown environment exploration, and structural reconstruction,” IEEE Robot Automa Lett 2(3), 1680–1687 (2017).
[19] A. Bircher, M. Kamel, K. Alexis, H. Oleynikova and R. Siegwart, “Receding Horizon “Next-Best-View” Planner for 3D
exploration,” In: IEEE International Conference on Robotics and Automation (ICRA), (2016) pp. 1462–1468.
[20] T. Dang, M. Tranzatto, S. Khattak, F. Mascarich, K. Alexis and M. Hutter, “Graph-based subterranean exploration path
planning using aerial and legged robots,” J Field Robot 37(8), 1363–1388 (2020).
[21] J. Huang, B. Zhou, Z. Fan, Y. Zhu, Y. Jie, L. Li and H. Cheng, “FAEL: Fast autonomous exploration for large-scale
environments with a mobile robot,” IEEE Robot Automa Lett 8(3), 1667–1674 (2023).
[22] C. Cao, H. Zhu, F. Yang, Y. Xia, H. Choset, J. Oh and J. Zhang, “Autonomous Exploration Development Environment and
the Planning Algorithms,” In: International Conference on Robotics and Automation (ICRA), (2022) pp. 8921–8928.
[23] K. Helsgaun, “An effective implementation of the Lin–Kernighan traveling salesman heuristic,” Eur J Oper Res 126(1),
106–130 (2000).
[24] T. Shan, B. Englot, D. Meyers, W. Wang, C. Ratti and R. Daniela, “Lio-Sam: Tightly-Coupled lidar Inertial Odometry
Via Smoothing and Mapping,” In: IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), (2020)
pp.5135–5142.





Combining Spatial Clustering and TSP-based Planning for Efficient Full Area Exploration


Short with Image and Video Description of the Ptoject

<img width="841" height="412" alt="image" src="https://github.com/user-attachments/assets/eba00530-ef57-497c-bc7a-264dfac885b8" />

Fig. 2. The framework of our exploration method. The blue blocks represent components employed during the exploration stage, whereas the orange blocks denote those utilized in the relocation stage.


Exploration Stage Detecting Local Frontiers

In traditional frontier-based exploration [20], frontiers F are defined as known-free voxels adjacent to unknown voxels. Here, a known-free voxel 𝑣 is inspected by examining its neighborhood voxels {𝑢} within a cuboidal region B 𝑣 ⊂ R 3 centered at 𝑣. If there exist unknown neighbor voxels with the number exceeding a given threshold 𝜆𝑛𝑢𝑚, the voxel is classified as a frontier:   
𝑣 ∈ F ⇐⇒ 𝑛(𝑢) > 𝜆𝑛𝑢𝑚, ∀𝑢 ∈ B𝑣 ∧ 𝑣 ∈ S 𝑓 𝑟𝑒𝑒 ∧ 𝑢 ∈ S𝑢𝑛𝑘                   (1)
Local frontiers FL are those found within a local planning horizon denoted as H ⊂ R 3, centered at the robot position at the start of each exploration planning iteration. A voxel 𝑣 is considered a local frontier if it meets: 
𝑣 ∈ F𝐿 ⇐⇒ 𝑣 ∈ F ∧ 𝑣 ∈ H	                                     (2)
Figure 1 also illustrates the process of detecting local frontiers. Initially, an RRT is dynamically expanded from the robot position to generate viewpoints within H. Each node on the tree corresponds to a viewpoint, and a local viewpoint graph G 𝐿 is constructed using these viewpoints as vertices. The local viewpoint graph helps plan a local navigation path from the current position to a local destination. Subsequently, frontiers are chosen from these viewpoints, defined as {F𝑖}, 
𝑖 = 1, · · ·, 𝑀, where 𝑀 is the number of frontiers. Specifically, any vertex 𝑣 on the graph that meets Equation (2) is regarded as a local frontier. Each local frontier F𝑖 is associated with a gain value: 
𝑉(F𝑖) = 𝑛 (𝑢), ∀ 𝑢 ∈ B F𝑖 ∧ 𝑢 ∈ S𝑢𝑛𝑘                                                (3) 
which is used to select the most promising frontiers for building the global viewpoint graph.


Clustering Local Frontiers

The identified local frontiers F𝐿 are divided into 𝐾 clusters based on their spatial proximity: 
F𝐿 = F 1𝐿 ∪ · · · ∪ F 𝐾𝐿, F 𝑖𝐿 ∩ F 𝑗𝐿 = ∅, 1 ≤ 𝑖, 𝑗 ≤ 𝐾, 𝑖 ≠ 𝑗                                   (4)
Frontiers are assigned to the same cluster if their Euclidean distance is less than a tolerance parameter 𝜖, specifying the minimum distance between any two clusters: 
F𝑖𝐿 ∩ F 𝑗𝐿 = ∅ ⇐⇒ 𝑑 (𝑢, 𝑣) > 𝜖, ∀𝑢 ∈ F𝑖𝐿, ∀𝑣 ∈ F 𝑗𝐿                                           (5)
where 𝑑 (·, ·) denotes the Euclidean distance between two spatial points. Note that there is another inherent constraint for frontier clustering. The frontiers are sampled from a local RRT, and the robot can travel freely between any two frontiers without encountering obstacles. This means that, in our context, two frontier clusters positioned on opposite sides of a thin wall can be merged into a single cluster, as long as they originate from the same RRT sampling. For each local frontier cluster F 𝑘𝐿, its centroid is calculated by averaging the 3D positions of all frontiers within the cluster:
 𝑐 𝑘𝐿 = 1 Fk L∑︁𝑖 F𝑖, F𝑖 ∈ F 𝑘𝐿                                                                                           (6)
where F k L is the number of local exploration goals in cluster F k L.
As shown in Figure 3, a navigation goal candidate g k L is selected for each local cluster F k L. This goal is the furthest point along the direction from the robot position Prob to c k L. In addition, a slight tolerance of direction angle between ⃗ g k L−c k L ∥g k L−c k L∥ and ⃗ c k L−Prob ∥c k L−Prob∥ is allowed. This is done to ensure that the local navigation goal is distant from the current robot position, promoting the coverage of more unknown areas for the next iteration of planning. In the end, K local exploration goal clusters will result in K local navigation goal candidates  {g k L}, 1 ≤ k ≤ K


<img width="681" height="413" alt="image" src="https://github.com/user-attachments/assets/20d2bcca-0163-421f-beec-5839ff0e20fd" />

Fig. 3. The process of generating local navigation goal candidates. For each exploration goal cluster, the centroid is calculated. The vector from the robot position to the centroid is determined. The furthest point in the vector direction with a slight angle tolerance is selected as the navigation goal candidate.


Local Tour Planning

The next question is how to determine the best local exploration goal from the candidate set. Inspired by [21], we propose solving it as a variant of the Traveling Salesman Problem (TSP). The local candidate goals { 𝑔 𝑘𝐿 } , 1 ≤ 𝑘 ≤ 𝐾 and the global home position 𝑔ℎ𝑜𝑚𝑒 serve as places to be visited by a salesman, starting from the current place 𝑃𝑟𝑜𝑏. The goal is to compute an optimal open-loop tour that passes through all the local candidate goals and ends at the global home position. Unlike the standard TSP, where the final place to visit is the starting place, here the final place is a specified place (i.e., 𝑔ℎ𝑜𝑚𝑒). We model this as an Asymmetric TSP (ATSP) by designing the cost matrix 𝐶 𝐴𝑇𝑆𝑃 accordingly. 𝐶 𝐴𝑇𝑆𝑃 is a ( 𝐾 + 1) × ( 𝐾 + 1 ) square matrix, where the entry 𝐶 𝐴𝑇𝑆𝑃 𝑟,𝑐 at row 𝑟 and column 𝑐 denotes the traveling cost from place 𝑟 to place 𝑐. The cost to travel from the current robot position 𝑃𝑟𝑜𝑏 to the 𝑘-th local candidate goal 𝑔 𝑘L is:
𝐶𝐴𝑇𝑆𝑃 0,𝑘 = 𝐿 (𝑃𝑟𝑜𝑏, 𝑔 𝑘𝐿 )+𝐻 ( 𝑃𝑟𝑜𝑏, 𝑔 𝑘𝐿 ), 𝑘 ∈ { 1 , · · , 𝐾 } ,            (7)
where L(·, ·) calculates the shortest path length between two positions according to the local viewpoint graph, and H(·, ·) calculates the cost of changing the robot heading to the next position. The cost between any two local goals is:
CAT SP r,c=C AT SP c,r=L(g r L, gc L), r, c ∈ {1, · · · , K}.                 (8) 
Considering the first column of C AT SP , the item C AT SP k,0 normally denotes the cost traveling from g k L to Prob. We substitute it with the cost traveling from g k L to ghome: 
CAT SP k,0= L(g k L, ghome), k ∈ {1,  · , K},                                        (9) 
where the shortest path length is calculated according to the global viewpoint graph. The advantage of including 𝑔ℎ𝑜𝑚𝑒 in local tour planning will be investigated in Section 5.3. Based on the cost matrix, the ATSP can be solved using existing algorithms. Once the optimal open-loop tour is obtained, the robot proceeds to the first destination on the tour. After reaching the initial goal, the local planning horizon H is adjusted accordingly, and the exploration stage is repeated until no frontier clusters remain within H. 


Relocation Stage

When there are no remaining frontier clusters within H, the robot enters the relocation stage. However, it’s important to note that the algorithm’s parameters may not always suit all types of terrains and environmental structures. Randomly sampling spatial points can sometimes result in an RRT with limited variation or even degeneration. In such cases, the robot may fail to explore certain areas effectively. To enhance the success rate of exploration, we introduce a retrying mechanism before transitioning from exploration to relocation. Specifically, when there are no frontier clusters within H, the RRT is regenerated for further exploration. If the robot still cannot find any frontier clusters, it proceeds to the relocation stage. This stage involves generating global frontier clusters, updating the global viewpoint graph, and planning a global tour.


Generating Global Frontier Clusters

In each iteration of local tour planning, any remaining local candidate goals that have not been visited before are added to the list of global candidate goals. Each global candidate goal corresponds to a global frontier cluster. However, because the information of frontiers can change during the exploration process, each global candidate goal is double-checked at the beginning of the relocation stage to confirm whether it still represents a frontier cluster. This verification can be quickly performed using Equation (1), where the searched space B𝑣 is doubled in 𝑥 and 𝑦 directions. Any global candidate goals that no longer satisfy Equation ( 1) are removed from the list. The updated global candidate goals are then denoted as { 𝑔 𝑛𝐺 } , 1 ≤ 𝑛 ≤ 𝑁 .


Updating Global Viewpoint Graph 

The global viewpoint graph plays an important role in computing the travel cost between any two global candidate goals, as well as planning the shortest navigation path from the current robot position to a desired destination within a large-scale space. Following the approach in [22], in each iteration of the exploration stage, the local graph vertices on the shortest path from the robot to vertices with positive gain values are added to the global viewpoint graph. The edges are updated accordingly. The global viewpoint graph provides a sparse representation of the environment while still providing short paths between viewpoints. 


Global Tour Planning 

During the relocation stage, the primary objective is to find an optimal global tour that guides the robot to visit all global candidate goals and return to the global home position. This is also achieved by implementing an ATSP. The cost matrix, which has a size of (𝑁 + 1) × (𝑁 + 1), is built using Equations (7) - (9) without considering the heading cost. Here, the local candidate goals {𝑔𝑘𝐿}, 1 ≤ 𝑘 ≤ 𝐾 are replaced by { 𝑔 𝑛𝐺 } , 1 ≤ 𝑛 ≤ 𝑁 . However, in large-scale and convoluted environments, the number of global candidate goals, denoted as 𝑁, can become excessively large. This poses a significant challenge for the ATSP solver. To address this issue, we propose the utilization of clustering technology to reduce the number of travel destinations for the ATSP solver when 𝑁 exceeds a predefined threshold, denoted as 𝜌𝑛𝑢𝑚. Specifically, each global candidate goal is assigned a cost value representing the shortest path length from the global home position to itself. Any two global candidate goals with a cost difference less than 𝜖𝑔𝑜𝑎𝑙 are grouped into the same cluster. Within each cluster, we select the candidate goal that is furthest from the home position. This approach significantly reduces the number of global candidate goals. As a result, the global tour planning process can be completed with a satisfactory computational load. The number of iterations required for global tour planning or relocation is also decreased, resulting in significant savings in navigation time. We will investigate the effectiveness of this approach in an ablation study, as discussed in Section 5.3 .


Algorithm Implementation 

Algorithm 1 provides an overview of the entire exploration process. Throughout this process, a Lidar Odometry and Mapping (LOAM) system plays a important role in estimating the robot’s states and generating the resultant map. Additionally, a terrain travers ability analysis module comes into play, generating a terrain map that provides the robot with obstacle information. Based on the terrain map and globally aligned laser scans, two maps, 𝑀𝑜𝑐𝑐 and 𝑀𝑜𝑐𝑡𝑜, are updated at both semantic and metric levels, respectively. Within a defined local planning horizon, an RRT is dynamically expanded based on the information from these semantic and metric maps. The subsequent steps (Lines 7 ∼ 17) in the algorithm involve frontier detection, frontier clustering, and local tour planning, as described in Section 4.1. If no local frontier clusters are found during the exploration stage, the algorithm allows for one more attempt before transitioning to the relocation stage. The following steps implement the relocation stage as described in Section 4.2. Line 23 shows the process of updating global candidate goals, which involves double-checking already generated global candidate goals and incorporating newly detected local candidate goals. All global candidate goals are then employed for global tour planning (Lines 25 ∼ 30). The strategic application of clustering technology to reduce traveling destinations for tour planning significantly limits computation overhead, especially in large-scale and convoluted environments. Upon successfully navigating the robot to its planned target, it transitions back to the exploration stage. These two stages alternate until no global candidate goals remain. Eventually, the robot returns to its home position, signifying the completion of the exploration task.


Algorithm 1 Exploration in Unknown Environments Input: LOAM result and terrain map Output: Explored map of the environment 
1: while Try exploration stage do 
2: H ← updatePlanningHorizon() 
3: 𝑀𝑜𝑐𝑐 ← updateOccupancyGridMap() 
4: 𝑀𝑜𝑐𝑡𝑜 ← updateVolumetricMap() 
5: T ← dynamicRRT( H , 𝑀𝑜𝑐𝑐 , 𝑀𝑜𝑐𝑡𝑜 ) 
6: G 𝐿 ← updateLocalGraph( T ) 
7: F𝐿 ← detectLocalFrontiers( T , 𝑀𝑜𝑐𝑡𝑜 ) 
8: if F𝐿 ≠ ∅ then 
9: G 𝐺 ← updateGlobalGraph( F𝐿 ) 
10: {F 𝑗𝐿 } ← clusteringFrontiers( F𝐿 ) 
11: Remove clusters containing no more than 𝜏𝑐 (e.g. 𝜏𝑐 = 3) points. 
12: if {F 𝑗𝐿 } ≠ ∅ then 
13: { 𝑔 𝑘𝐿 } ← generateLocalCandidateGoals({F 𝑗𝐿 } ) 14: 𝑔 ← planningTour( { 𝑔 𝑘𝐿 } , 𝑃𝑟𝑜𝑏 , 𝑔ℎ𝑜𝑚𝑒 , G 𝐿 , G 𝐺 ) 
15: Do path planning and navigate the robot to target 𝑔 . 
16: end if 
17: end if 
18: if F𝐿 = ∅ or {F 𝑗𝐿 } = ∅ then 
19: Retry exploration stage one more time. 
20: end if 
21: end while 
22: Transitions to relocation stage. 
23: { 𝑔 𝑛𝐺 } ← updateGlobalCandidateGoals( { 𝑔 𝑛𝐺 } , { 𝑔 𝑘𝐿 } )
24: if { 𝑔 𝑛𝐺 } ≠ ∅ then 
25: if |{ 𝑔 𝑛𝐺 }| ≥ 𝜌𝑛𝑢𝑚 then 
26: { 𝑔 𝑖𝐺 } ← clusteringGlobalCandidateGoals( { 𝑔 𝑛𝐺 } ) 
27: 𝑔 ← planningTour( { 𝑔 𝑖𝐺 } , 𝑃𝑟𝑜𝑏 , 𝑔ℎ𝑜𝑚𝑒 , G 𝐺 ) 
28: else 
29: 𝑔 ← planningTour( { 𝑔 𝑛𝐺 } , 𝑃𝑟𝑜𝑏 , 𝑔ℎ𝑜𝑚𝑒 , G 𝐺 ) 
30: end if 
31: Do path planning and navigate the robot to target 𝑔 . 
32: Transitions to exploration stage. 
33: else 
34: Return home and complete the exploration task. 
35: end if


ResultsArea Coverage and Exploration Efficiency

Table 1 provides an overview of the volumes explored by the compared methods, with the maximum volume across all runs for each environment serving as the ground truth area volume. A run is considered unsuccessful if the explored volume is less than 98% of the ground truth. Table 2 compares the exploration performances of DSVP and our method. The performance metrics are calculated based on the successful runs. In the indoor environment, characterized by long and narrow corridors connected with lobby areas, both methods may fail to go through spaces with guard rails, as shown in Figure 4, due to the randomness of RRT expansion. But our method demonstrates a slightly higher success rate of 90% compared to DSVP’s 80%. This is attributed to our method’s bias-free RRT expansion, which favors finding sparse local candidate goals that can be reached by the robot. DSVP, on the other hand, biases RRT expansion toward frontiers in large-scale spaces

TABLE I 
ENVIRONMENT VOLUME ESTIMATED BY THE RUNS.

Env.	Maximum Volume (m3)	98% of the
 	DSVP	Ours	All	Maximum Volume (m3)
indoor	5378.8	5377.1	5378.8	5271.2
campus	46101.6	46196.3	46196.3	45272.3
garage	42445.5	42291.6	42445.5	41596.6
tunnel	22072.9	22024.6	22072.9	21631.4
forest	42580.4	43128.4	43128.4	42265.8

<img width="835" height="306" alt="image" src="https://github.com/user-attachments/assets/e80c5fbd-df31-485e-9aa3-25cff21e49d0" />


TABLE II
COMPARISON OF EXPLORATION PERFORMANCE.

 	 	Success	 	Explored Volume	 	Traveling Distance	 	 	Overall Time	 	Explored Volume
Env.	Method	 	 	(m3)	 	 	 	 	 	 	(m)	 	 	 	(s)	 	 	 	 	 	Per Second (m3/s)	 
Rate (%)	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 
 	 	 	 	 	 	 	 	 	Mean	 	 	Std.	 	Mean	 	 	 	Std.	 	 	Mean	 	 	Std.	 	Mean	 	Std.	 
indoor	DSVP	80	 	 	5357.4	 	 	 	11.0	 	 	1469.3	 	92.0	 	 	887.4	 	 	 	67.3	 	 	6.07	 	 	0.41	 	 
Ours	90	 	 	5364.8	 	 	 	10.5	 	 	1444.2	 	42.3	 	 	837.5	 	 	 	28.2	 	 	6.41	 	 	0.21	 	 
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 
campus	DSVP	80	 	 	45804.4	 	 	175.4	 	2613.9	 	39.1	 	 	1348.6	 	 	27.9	 	 	33.98	 	0.68	 
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 
Ours	100	 	46020.1	 	 	122.8	 	2631.5	 	24.2	 	 	1354.6	 	 	12.3	 	 	33.98	 	0.33	 
 	 	 	 	 	 	 	 	 	 	 	 	 	 
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 
garage	DSVP	70	 	 	42367.9	 	 	49.4	 	 	5105.7	 	247.1	 	2728.2	 	 	124.1	 	15.56	 	0.74	 
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 
Ours	100	 	42239.4	 	 	40.2	 	 	4114.7	 	204.3	 	2216.1	 	 	107.1	 	19.10	 	0.87	 
 	 	 	 	 	 	 	 	 	 	 	 	 
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 
tunnel	DSVP	100	 	22046.9	 	 	22.3	 	 	6673.7	 	252.8	 	3612.2	 	 	141.9	 	6.11	 	 	0.23	 
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 
Ours	100	 	21956.5	 	 	90.4	 	 	6033.4	 	152.8	 	3238.6	 	 	88.0	 	 	6.78	 	 	0.17	 
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 
forest	DSVP	50	 	 	42492.0	 	 	89.4	 	 	2104.6	 	44.3	 	 	1113.2	 	 	16.5	 	 	38.18	 	0.50	 
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 
Ours	100	 	42748.1	 	 	252.4	 	2053.5	 	103.7	 	1098.9	 	 	57.7	 	 	38.99	 	1.76	 

 	 	 	 	 	 	 	 	 	 	 	 	 
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 <img width="771" height="387" alt="image" src="https://github.com/user-attachments/assets/b34100f1-4f6d-45ea-88de-300b886a61c7" />



Figure 5 shows the exploration trajectories that are the best of the 10 runs. The best trajectory is from the successful runs and with the highest exploration efficiency. On average, DSVP completes the exploration after traveling 1469.3 meters over 887.4 seconds, while our method takes 837.5 seconds and travels 1444.2 meters. With tour planning, our method facilitates more efficient travel to candidate goals. Despite the need to solve the ATSP, the heuristic solver efficiently handles the limited number of candidate goals in our method. In terms of exploration efficiency, our method covers 6.41 𝑚 3 / 𝑠, while DSVP covers 6.07 𝑚 3 / 𝑠. Notably, the standard variances of all metrics for our method are lower than those for DSVP, indicating that our method is more efficient and stable. In the campus environment with undulating terrains, DSVP achieves a success rate of 80%, while our method achieves 100% by incorporating the retrying mechanism for RRT expansion. However, regenerating the RRT and re-finding local exploration goals add extra time to our method’s process. On average, both methods spend comparable time and travel similar distances to complete the explorations, achieving an exploration efficiency of 33.98 𝑚 3 / 𝑠. As shown in Figure 6, the best trajectory in DSVP outperforms ours. In this environment, where lanes form a star network, tour planning does not significantly impact exploration in our method. Again, our method exhibits lower standard variances across all metrics, indicating greater stability. In the garage environment, featuring multiple floors and sloped terrains, DSVP faces challenges and achieves a success rate of only 70%. In contrast, our method successfully completes the exploration in all runs. On average, our method travels 991 meters less distance, takes 512 seconds less time, and achieves a 3.5 𝑚 3 / 𝑠 higher exploration efficiency compared to DSVP. Moreover, our method exhibits lower standard variances for explored volume, traveling distance, and overall time, indicating superior efficiency and stability. Qualitative comparisons of the best trajectories, as shown in Figure 7, support these findings.



<img width="623" height="335" alt="image" src="https://github.com/user-attachments/assets/612dfcd2-8c34-4816-8240-f8ce2a57cd71" />

Figure 4. The space with guard rails which occasionally hinders the robot’s passage. During exploration failures, the robot may successfully reach Point A but faces difficulties progressing to Point C. This is because the laser scans can penetrate the guard rails, resulting in fewer frontiers being detected at Point B. If the RRT cannot find a feasible path for extending from Point B to Point C, the robot will be unable to reach its destination at Point C via Point B.


<img width="850" height="391" alt="image" src="https://github.com/user-attachments/assets/162fc5cf-808b-4d03-b100-3cae8bceb2c3" />

Figure 5. The resulting maps and trajectories of the compared methods for the indoor environment. 

In the tunnel environment, characterized by a complex network of tunnels, both methods complete the exploration task for all runs. However, our method outperforms DSVP in terms of traveling distance (640 meters less), time (374 seconds less), and exploration efficiency (0.67 𝑚 3 / 𝑠 higher). Additionally, our method has lower standard variances for most metrics compared to DSVP. Figure 8 shows a qualitative comparison of the best trajectories, demonstrating that our method explores a comparable volume of space with less time and distance. In the forest environment with cluttered trees, DSVP tends to explore the space coarsely, resulting in the oversight of many small spaces and a 50% success rate. Our method is able to complete all exploration runs but exhibits slightly higher standard variance. In terms of other metrics, both methods demonstrate comparable performance overall. However, our method achieves a slightly higher exploration efficiency than DSVP, with an additional 0.8 𝑚3/𝑠. Qualitative comparisons of the best trajectories for each method, as shown in Figure 9, further highlight our method’s superior exploration efficiency


<img width="865" height="413" alt="image" src="https://github.com/user-attachments/assets/5b777dbc-b6c8-470c-8305-0636e911ca58" />

Fig 6. The resulting maps and trajectories of the compared methods for the campus environment. Ours DSVP


<img width="850" height="385" alt="image" src="https://github.com/user-attachments/assets/f6878361-bed6-4530-bcc9-af299b06a6ee" />

Fig. 7. The resulting maps and trajectories of the compared methods for the garage environment.


Computational Efficiency

o investigate the computational efficiencies of the methods, the planning iteration count and average planning runtime for each exploration run are recorded. The average values across all runs are presented in Table 3. Our method generally requires fewer planning iterations compared to DSVP in most environments. In the indoor, tunnel, and forest environments, our method also exhibits shorter runtimes than DSVP. However, it’s worth noting that in the campus and garage environments, which feature numerous wide spaces, our method generates a larger number of candidate goals, leading to additional time for planning and target selection. Nevertheless, the average planning runtime across all environments remains at approximately 0.45 seconds, highlighting the efficiency and suitability of our exploration algorithm for real-world mobile robot deployment.


Comparison with other methods

We also execute the open-source FAEL code [11] in the same five simulated environments for multiple runs. As shown in Figure 10, the FAEL method exhibits limitations in fully exploring highly convoluted environments. Sometimes, there may be frequent switches in navigation target selection, particularly within highly convoluted spaces. FAEL appears to be better suited for large unconvoluted spaces, where it balances factors like information gain, movement distance, and coverage efficiency effectively. Since FAEL fails in exploration of the garage environment, characterized by multiple floor and sloped terrains, its compatibility with multi-floor environments remains uncertain. 

<img width="865" height="417" alt="image" src="https://github.com/user-attachments/assets/00685ff9-bc87-4334-a0de-6268d37bb1ad" />

Figure 8. The resulting maps and trajectories of the compared methods for the tunnel environmen


<img width="865" height="417" alt="image" src="https://github.com/user-attachments/assets/447cdac8-4357-4367-b195-0108f113672a" />

Figure 9. The resulting maps and trajectories of the compared methods for the forest environment


Ablation Studies

Local tour planning without considering global home position. As indicated in Equation ( 9), the cost of traveling from any local candidate goal to the current robot position is defined as the distance from the candidate goal to the global home position. We investigate the scenario where the global home position is not taken into account, specifically by setting 𝐶 𝐴𝑇𝑆𝑃 𝑘, 0 to 0. The revised version of our method is referred to as "Ours_R1". We perform 10 exploration runs in the indoor environment. The average performance results are presented in Table 4. It shows that, when the global home position is not considered, the method yields comparable exploration coverage and traveling distance but requires more exploration time. Consequently, the exploration efficiency is lower compared to the method that takes the home position into account. Global tour planning without clustering candidate goals. In the global tour planning process, the traveling destinations are selected from the global candidate goals for the ATSP. In the experiment, if the number of global candidate goals exceeds 40, they are clustered, resulting in a significant decrease in the number of traveling destinations. Figure 11(a) shows the change in the number of selected traveling destinations throughout each iteration of global tour planning, when executing exploration in the garage environment. Solving the ATSP with 40 nodes takes about 11 milliseconds. Figure 11(b) shows the results when clustering is not utilized, during another exploration in the same environment. The maximum number of traveling destinations reaches 114, requiring 48 milliseconds to solve the ATSP

<img width="913" height="373" alt="image" src="https://github.com/user-attachments/assets/00d3f8dc-d8ae-4808-a93c-d52b52ac739a" />

Figure 10. Exploration result of the indoor environment. The white lines denote the trajectories of the robot. It shows that the FAEL method exhibits limitations in fully exploring the highly convoluted environment. 


Test in Real-world Environments

We deploy our exploration package and the benchmark navigation stack [ 4] on our mobile robot platform, which is a four-wheel differential driving mobile robot as shown in Figure 12. For autonomous exploration, our robot is equipped with a Vulvodynia VLP-16 lidar and a 6-axis IMU. To handle robot localization and mapping, we utilize a modified version of the LIO-SAM package [14], which implements a real-time lidar-inertial odometry. The robot’s maximum linear and angular velocities are set to 0.5 m/s and 15 deg/s, respectively. All packages run on an onboard computer with a 4.8GHz i7 CPU and 32GB RAM. The first experiment is conducted in a single floor of a building on our university campus, as shown in Figure 13. The environment consists of several narrow corridors, intersections, doors and dead ends. Figure 13 also displays the resulting map and final trajectory obtained by our method. To prevent the robot from entering the bathroom, we placed some static obstacles within the environment. Overall, our method spends 760.2 seconds traveling a distance of 230.5 meters and covering an area of 1928.3 cubic meters. Given the low-speed setting of our mobile platform and the relatively enclosed corridor environment, the exploration efficiency is measured at 2.54 𝑚 3 / 𝑠 . In another exploration experiment, we navigate the robot through an underground parking lot of a teaching building on our university campus, as illustrated in Figure 14. This environment comprises multiple parking spaces with many cars parked, along with building columns and other obstacles. The entry and exit of the parking lot were blocked with static obstacles. The robot begins the exploration near the entry point. After traveling a distance of 567.8 meters over a duration of 1243.3 seconds, the robot successfully completes the exploration. The explored space volume measures 15077.8 cubic meters, resulting in an exploration efficiency of 12.13 𝑚 3 / 𝑠 . These real-world experiments demonstrate the effectiveness of our algorithm when applied to actual mobile robots, highlighting its capacity to efficiently navigate and explore unfamiliar environments.


<img width="634" height="335" alt="image" src="https://github.com/user-attachments/assets/510489e8-d160-48d6-8049-cdcd6a8dfd3e" />

Figure 12. Our mobile robot. For autonomous exploration, it is equipped with a Velodyne VLP-16 lidar and a 6-axis IMU. All packages run on an onboard computer with a 4.8GHz i7 CPU and 32GB RAM. 

Conclusions

In conclusion, we have presented a dual-stage exploration method that prioritizes simplicity and effectiveness. In the exploration stage, frontiers are detected from RRT nodes, and local candidate goals are extracted from frontier clusters rather than calculating gains of all possible RRT branches. The local tour planner is further employed to select optimal goals for efficient exploration. In the relocation stage, the global tour planner controls the number of candidate goals and guides the robot to revisit unexplored spaces. Our proposed retrying mechanism significantly enhances the success rate of exploration. To evaluate our approach, we conduct tests in five benchmark simulation environments and two real-world environments. Both the simulation and real-world tests demonstrate the effectiveness of our method. However, both DSVP and our methods have certain limitations. Firstly, the system heavily relies on mapping performance and the accuracy of robot localization. It may not perform as expected in open spaces, such as the outdoor areas on campus. Relying solely on laser scans may prove insufficient for detecting obstacles like road curbs or green belts that could pose a danger to the robot. To enhance terrain travers ability analysis, integrating additional sensors, such as cameras, would be highly beneficial. Secondly, during the exploration stage, the local graph is utilized to plan the path from the current position to the target goal. Due to the sparse sampling of nodes in the local graph, the planned path may not always be optimal. 

<img width="624" height="307" alt="image" src="https://github.com/user-attachments/assets/d070a603-60b4-424d-85fe-7f1baa3f760d" />

Figure 13. The resulting map and trajectory of our method for a real corridor environment.

Furthermore, the planned path is not smoothed in accordance with the robot’s kinematics model. This can result in the robot making frequent heading adjustments, leading to increased power consumption, longer travel times, and posing additional challenges to the simultaneous localization and mapping process. Future work will be dedicated to addressing these identified shortcomings.

