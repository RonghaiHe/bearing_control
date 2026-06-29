# Translational and Scaling Formation Maneuver Control via a Bearing-Based Approach

Shiyu Zhao and Daniel Zelazo, Member, IEEE

Abstract—This paper studies distributed maneuver control of multiagent formations in arbitrary dimensions. The objective is to control the translation and scale of the formation while maintaining the desired formation pattern. Unlike conventional approaches where the target formation is defined by relative positions or distances, we propose a novel bearing-based approach where the target formation is defined by inter-neighbor bearings. Since the bearings are invariant to the translation and scale of the formation, the bearing-based approach provides a simple solution to the problem of translational and scaling formation maneuver control. Linear formation control laws for double-integrator dynamics are proposed and the global formation stability is analyzed. This paper also studies bearing-based formation control in the presence of practical problems, including input disturbances, acceleration saturation, and collision avoidance. The theoretical results are illustrated with numerical simulations.

Index Terms—Bearing rigidity, formation scale control, formation tracking control, input saturation, multiagent systems.

## I. INTRODUCTION

E <sup>XISTING</sup> <sup>approaches</sup> <sup>to</sup> <sup>multiagent</sup> <sup>formation</sup> <sup>control</sup> <sup>can</sup> be categorized by how the desired geometric pattern of the target formation is defined. In two popular approaches, the target formation is defined by inter-neighbor relative positions or distances (see [1] for an overview). It is notable that the invariance of the constraints of the target formation has an important impact on the formation maneuverability. For example, since the relative-position constraints are invariant to the translation of the formation, the relative-position-based approach can be applied to realize translational formation maneuvers (see, for example, [2]). Since distance constraints are invariant to both translation and rotation of the formation, the distance-based approach can be applied to realize translational and rotational formation maneuvers (see, for example, [3]).

In addition to the aforementioned two approaches, there has been a growing research interest in a bearing-based formation control approach in recent years [4]–[7]. In the bearing-based approach, the geometric pattern of the target formation is defined by inter-neighbor bearings. Since the bearings are invariant to the translation and scale of the formation, the bearingbased approach provides a simple solution to the problem of translational and scaling formation maneuver control. The translational maneuvers refer to when the agents move at a common velocity such that the formation translates as a rigid body. Scaling maneuvers refer to when the formation scale, which is defined as the average distance from the agents to the formation centroid, varies while the geometric pattern of the formation is preserved. It is worth mentioning that the bearing-based formation control studied in this paper requires relative-position or velocity measurements, which differs from the bearing-only formation control problem where the feedback control relies only on bearing measurements [8]–[16]. Moreover, bearingbased formation control is a linear control problem whereas bearing-only formation control is nonlinear.

Formation scale control is a useful technique in practical formation control tasks. By adjusting the scale of a formation, a team of agents can dynamically respond to their surrounding environment to, for example, avoid obstacles. The problem of formation scale control has been studied by the relative-position and distance-based approaches in [17] and [18]. However, since neither the relative positions nor distances are invariant to the formation scale, these two approaches result in complicated estimation and control schemes in which follower agents must estimate the desired formation scale known only by leader agents. Moreover, the two approaches are so far only applicable in the case where the desired formation scale is constant. Very recently, the work [19] proposed a formation control approach based on the complex Laplacian matrix. In this approach, the target formation is defined by complex linear constraints that are invariant to the translation, rotation, and scale of the formation. As a result, this approach provides a simple solution to formation scale control. However, as shown in [19], the approach is only applicable to formation control in the plane; it is unclear if it can be extended to higher dimensions.

Although the bearing-based approach provides a simple solution to formation scale control, the existing studies on bearingbased formation control focus mainly on the case of static target formations. The case where the translation and scale of the target formation are time-varying has not yet been studied. Moreover, a fundamental problem, which has not been solved in the existing literature, is when the target formation can be uniquely determined by the inter-neighbor bearings and leaders in arbitrary dimensional spaces. The analysis of this fundamental problem requires the bearing rigidity theory proposed in [16] and was addressed in our recent work in [20]. Our previous work [21] considered a single-integrator dynamic model of the agents and proposed a proportional-integral bearing-based formation maneuver control law.

The contributions of this paper are summarized as below. First, we study the problem when a target formation can be uniquely determined by inter-neighbor bearings and leader agents. The necessary and sufficient condition for the uniqueness of the target formation is analyzed based on a special matrix we call the bearing Laplacian, which characterizes the interconnection topology and the inter-neighbor bearings of the formation. Second, we propose two linear bearing-based formation control laws for double-integrator dynamics. With these two control laws, the formation can track constant or time-varying leader velocities. In the proposed control laws, the desired translational and scaling maneuver is only known to the leaders and the followers are not required to estimate it. A global formation stability analysis is presented for each of the control laws. Third, we study bearing-based formation control in the presence of some practical issues. In particular, control laws that can handle constant input disturbances and acceleration saturation are proposed and their global stability is analyzed. Sufficient conditions that ensure no collision between any two agents are also proposed. Finally, it is noteworthy that the results presented in this paper are applicable to formation control in arbitrary dimensions.

The organization of this paper is as follows. Section II presents the problem formulation. Section III proposes and analyzes two linear bearing-based formation control laws. Section IV considers bearing-based formation control in the presence of practical issues, such as input disturbances and acceleration saturation. Conclusions are drawn in Section V.

## II. FORMATION MANEUVER CONTROL PROBLEM AND BEARING-CONSTRAINED TARGET FORMATIONS

Consider a formation of n agents in $\mathbb { R } ^ { d } \left( n \geq 2 , d \geq 2 \right)$ . Let $\mathcal { V } \overset { \Delta } { = } \{ 1 , \dots , n \}$ . Denote $p _ { i } ( t ) \in \mathbb { R } ^ { d }$ and $v _ { i } ( t ) \in \mathbb { R } ^ { d }$ as the position and velocity of agent $i \in \nu$ . Let the first $n _ { \ell }$ agents be called the leaders and the remaining $n _ { f }$ agents the followers $( n _ { \ell } +$ $n _ { f } = n )$ . Let $\mathcal { V } _ { \ell } = \{ 1 , \ldots , n _ { \ell } \}$ and $\mathcal { V } _ { f } = \{ n _ { \ell } + 1 , \ldots , n \}$ be the index sets of the leaders and followers, respectively. The motion (i.e., position and velocity) of each leader is given a priori, and we assume the velocity of each leader is piecewise continuously differentiable. Each follower is modeled as a double-integrator

$$
\dot {p} _ {i} (t) = v _ {i} (t), \quad \dot {v} _ {i} (t) = u _ {i} (t), \quad i \in \mathcal {V} _ {f}
$$

where $u _ { i } ( t ) \in \mathbb { R } ^ { d }$ is the acceleration input to be designed. Let $p _ { \ell } = [ p _ { 1 } ^ { T } , \ldots , p _ { n _ { \ell } } ^ { T } ] ^ { T } , p _ { f } = [ p _ { n _ { \ell } + 1 } ^ { T } , \ldots , p _ { n } ^ { T } ] ^ { T } , v _ { \ell } =$ $\big [ v _ { 1 } ^ { T } , \dots , v _ { n _ { \ell } } ^ { T } \big ] ^ { T }$ , and $v _ { f } = [ v _ { n _ { \ell } + 1 } ^ { T } , \ldots , v _ { n } ^ { T } ] ^ { T }$ . Let $p = [ p _ { \ell } ^ { T } , p _ { f } ^ { T } ] ^ { T }$ and $v = \left[ v _ { \ell } ^ { T } , v _ { \ell } ^ { T } \right] ^ { T }$

The underlying information flow among the agents is described by a fixed graph $\mathcal { G } = ( \nu , \mathcal { E } )$ where $\mathcal { E } \subset \mathcal { V } \times \mathcal { V }$ is the edge set. By mapping the point $p _ { i }$ to the vertex $i ,$ we denote the formation as $\mathcal G ( \boldsymbol p )$ . If $( i , j ) \in \mathcal { E }$ , agent i can access the information of agent j. The set of neighbors of agent i is denoted as $\mathcal { N } _ { i } \overset { \Delta } { = } \{ j \in \mathcal { V } : ( i , j ) \in \mathcal { E } \}$ . We assume that the information flow between any two followers is bidirectional. The bearing of agent j relative to agent i is described by the unit vector

$$
g _ {i j} \triangleq \frac {p _ {j} - p _ {i}}{\| p _ {j} - p _ {i} \|}.
$$

Note $g _ { j i } = - g _ { i j }$ . For $g _ { i j }$ , define

$$
P _ {g _ {i j}} \triangleq I _ {d} - g _ {i j} g _ {i j} ^ {T}
$$

where $I _ { d } \in \mathbb { R } ^ { d \times d }$ is the identity matrix. Note that $P _ { g _ { i j } }$ is an orthogonal projection matrix that geometrically projects any vector onto the orthogonal compliment of $P _ { g _ { i j } }$ . It can be verified that $P _ { g _ { i j } }$ is positive semidefinite and satisfies $P _ { g _ { i j } } ^ { T } = P _ { g _ { i j } }$ $P _ { g _ { i j } } ^ { 2 } = P _ { g _ { i j } }$ , and $\mathrm { N u l l } ( P _ { g _ { i j } } ) = \mathrm { s p a n } \{ g _ { i j } \}$

## A. Bearing-Based Formation Maneuver Control

Suppose the real bearings of the formation at time $t > 0$ are $\{ g _ { i j } ( t ) \} _ { ( i , j ) \in \mathcal { E } }$ , and the desired constant bearings are $\{ g _ { i j } ^ { * } \} _ { ( i , j ) \in \mathcal { E } } .$ The bearing-based formation control problem is formally stated below.

Problem 1 (Bearing-Based Formation Maneuver Control): Consider a formation $\mathcal { G } ( \boldsymbol { p } ( t ) )$ where the (time-varying) position and velocity of the leaders $\{ p _ { i } ( t ) \} _ { i \in \mathcal { V } _ { \ell } }$ and $\{ v _ { i } ( t ) \} _ { i \in \mathcal { V } _ { \ell } }$ are given. Design the acceleration control input $u _ { i } ( t )$ for each follower $i \in \mathcal { V } _ { f }$ based on the relative position $\{ p _ { i } ( t ) - p _ { j } ( t ) \} _ { j \in \mathcal { N } _ { i } }$ and the relative velocit $\{ v _ { i } ( t ) - v _ { j } ( t ) \} _ { j \in \mathcal { N } _ { i } }$ such that $g _ { i j } ( t ) $ $g _ { i j } ^ { * }$ for all $( i , j ) \in \mathcal { E }$ as $t \to \infty$

Problem 1 can be equivalently stated as a problem where the formation is required to converge to a bearing-constrained target formation as defined below.

Definition 1 (Target Formation): The target formation denoted by $\mathcal { G } ( \boldsymbol { p } ^ { * } ( t ) )$ is a formation that satisfies the following constraints for all $t \geq 0 \colon$

a) Bearing $( p _ { i } ^ { * } ( t ) - p _ { i } ^ { * } ( t ) ) / \| p _ { i } ^ { * } ( t ) - p _ { i } ^ { * } ( t ) \| = g _ { i j } ^ { * } , \forall ( i , j ) \in \mathcal { E } ;$ b) Leader $p _ { i } ^ { * } ( \dot { t } ) = p _ { i } ( t )$ $\forall i \in \mathcal { V } _ { \ell }$ .1，

The target formation $\mathcal { G } ( \boldsymbol { p } ^ { * } ( t ) )$ is constrained jointly by the bearing constraints and the leader positions. The bearing constraints are constant, but the leader positions may be timevarying. Given appropriate motion of the leaders, the target formation has the desired translational and scaling maneuver and desired inter-neighbor bearings. If the real formation $p ( t )$ converges to the target formation $p ^ { * } ( t )$ , the desired formation maneuver and formation pattern can be simultaneously achieved. Motivated by this idea, define the position and velocity errors for the followers as

$$
\delta_ {p} (t) = p _ {f} (t) - p _ {f} ^ {*} (t), \quad \delta_ {v} (t) = v _ {f} (t) - v _ {f} ^ {*} (t)\tag{1}
$$

where $p _ { f } ^ { * } ( t )$ and $v _ { f } ^ { \ast } ( t )$ are the position and velocity of the followers in the target formation. The control objective is to design control laws for the followers to drive $\delta _ { p } ( t )  0$ and $\delta _ { v } ( t )  0$ as $t \to \infty$ (see Fig. 1 for an illustration). Note $\dot { \delta } _ { p } ( t ) = \delta _ { v } ( t )$

A fundamental problem regarding the target formation, which is still unexplored so far, is whether $p ^ { * } ( t )$ exists and is on March 19,2026 at 11:34:34 UTC from IEEE Xplore. Restrictions apply.

![](images/da6985cacb953db2441af2cd3887721fb63c53c24e65c75be51b4ed4fc0043e2.jpg)

![](images/59258e9bc0b2ba0f3628a4f3db0d6aa48afac9ebcd5d4af5b23ace7f4e4c276e.jpg)  
Fig. 1. Illustration of the bearing-constrained target formation. Solid dots: leaders; hollow dots: followers. Figure (a) shows the target formation $p ^ { * }$ and the real formation $p .$ Figure (b) shows two target formations that have the same bearings but different translations and scales.

unique. If $p ^ { * } ( t )$ is not unique, there exist multiple formations satisfying the bearing constraints and leader positions and, consequently, the formation may not be able to converge to the desired geometric pattern. This fundamental problem is analyzed in the following subsection.

## B. Properties of the Target Formation

This subsection explores the properties of the target formation that will be used throughout this paper.

1) Bearing Laplacian Matrix: Define a matrix $B ( G ( p ^ { * } ) ) \in$ $\mathbb { R } ^ { d n \times d n }$ with the ijth block as

$$
[ \mathcal {B} \left(G (p ^ {*})\right) ] _ {i j} = \left\{ \begin{array}{l l} \mathbf {0} _ {d \times d}, & i \neq j, (i, j) \not \in \mathcal {E} \\ - P _ {g _ {i j} ^ {*}}, & i \neq j, (i, j) \in \mathcal {E} \\ \sum_ {k \in \mathcal {N} _ {i}} P _ {g _ {i k} ^ {*}}, & i = j, i \in \mathcal {V}. \end{array} \right.
$$

The matrix $\boldsymbol { B } ( G ( \boldsymbol { p } ^ { * } ) )$ , which we write in short as B in the sequel, can be viewed as a matrix-weighted graph Laplacian matrix, where the matrix weight for each edge is a positive semidefinite orthogonal projection matrix. We call B the bearing Laplacian since it characterizes the interconnection topology and the bearings of the formation. The bearing Laplacian matrix naturally emerges and plays important roles in bearingbased formation control and network localization problems [6], [20], [21].

We now state an important property of the bearing Laplacian. In the sequel, $\mathbf { 1 } _ { n } \in \mathbb { R } ^ { n }$ is the vector with all entries equal to one, and $\otimes$ denotes the Kronecker matrix product.

Lemma 1: For any $\mathcal { G } ( p ^ { * } )$ , the bearing Laplacian always satisfies

$$
\operatorname{Null} (\mathcal {B}) \supseteq \operatorname{span} \left\{\mathbf {1} _ {n} \otimes I _ {d}, p ^ {*} \right\}.\tag{2}
$$

Proof: For any $\boldsymbol { x } = \left[ x _ { 1 } ^ { T } , \ldots , x _ { n } ^ { T } \right] ^ { T } \in \mathbb { R } ^ { d n }$ , we have

$$
\mathcal {B} x = \left[ \begin{array}{c} \vdots \\ \sum_ {j \in \mathcal {N} _ {i}} P _ {g _ {i j} ^ {*}} (x _ {i} - x _ {j}) \\ \vdots \end{array} \right].\tag{3}
$$

First, if $x \in \operatorname { s p a n } \{ \mathbf { 1 } _ { n } \otimes I _ { d } \}$ , then $x _ { i } = x _ { j }$ for all $i , j \in \mathcal { V }$ . It then follows from (3) that $B x = 0$ . Second, if $x \in$ span $\{ p ^ { * } \}$ then $x _ { i } = k p _ { i } ^ { * }$ for all $i \in \mathcal V$ where $k \in \mathbb { R }$ . It then follows from $P _ { g _ { i j } ^ { * } } ( p _ { i } ^ { * } - p _ { j } ^ { * } ) = 0$ that $B x = 0$ . To summarize, any vector in span $\{ \mathbf { 1 } _ { n } \otimes \bar { I } _ { d } , p ^ { * } \}$ is also in $\operatorname { N u l l } ( B )$ . -

Remark 1: In fact, any vector in the null space of B corresponds to a motion of the formation that preserves all of the bearings [20]. As a result, the expression in (2) indicates that the bearings are invariant to the translational and scaling motion of the formation. Specifically, ${ \mathbf { 1 } } _ { n } \otimes I _ { d }$ corresponds to the translational motion and $\begin{array} { r } { p ^ { * } - \mathbf { 1 } _ { n } \otimes ( \sum _ { i = 1 } ^ { n } p _ { i } ^ { * } / n ) } \end{array}$ corresponds to the scaling motion. In addition, the bearings may also be invariant to other bearing-preserving motions. [See, for example, Fig. 2(a).] It is of great interest to understand when Null(B) is exactly equal to span $\{ \mathbf { 1 } _ { n } \otimes I _ { d } , p ^ { * } \}$ . As shown in [20], when G is undirected, $\operatorname { N u l l } ( { \mathcal { B } } ) = \operatorname { s p a n } \{ \mathbf { 1 } _ { n } \otimes I _ { d } , p ^ { * } \}$ if and only if $\mathcal G ( \boldsymbol p )$ is infinitesimally bearing rigid. The definition of the infinitesimal bearing rigidity and preliminaries to the bearing rigidity theory are given in the Appendix.

![](images/f4b777fd88732d10b77e9df928c720463a93ab823f2e754fe2e6555edd323b2b.jpg)  
(a)

![](images/bdd0b99c7cca2be8ad3d277b1bc227f080d7ee3035af9e2061920ffdd40103d8.jpg)  
(b)

![](images/fb5646460ee9745b5499cc9ae6e9da809939e9978662eabcdaf309d507ebe0b5.jpg)  
(c)  
Fig. 2. Examples of nonunique and unique target formations. Solid dots: leaders; hollow dots: followers. The formation in (c) is 3-D. (a) Non-unique. (b) Unique. (c) Unique.

We continue with the analysis by partitioning B as

$$
\mathcal {B} = \left[ \begin{array}{c c} \mathcal {B} _ {\ell \ell} & \mathcal {B} _ {\ell f} \\ \mathcal {B} _ {f \ell} & \mathcal {B} _ {f f} \end{array} \right]
$$

where $\mathcal { B } _ { \ell \ell } \in \mathbb { R } ^ { d n _ { \ell } \times d n _ { \ell } } , \mathcal { B } _ { \ell f } \in \mathbb { R } ^ { d n _ { \ell } \times d n _ { f } } \mathcal { B } _ { f \ell } \in \mathbb { R } ^ { d n _ { f } \times d n _ { \ell } }$ , and $B _ { f f } \in \mathbb { R } ^ { d n _ { f } \times d n _ { f } }$ . As will be shown later, the submatrix $\boldsymbol { B } _ { f f }$ plays an important role in this work.

Lemma 2: The submatrix $B _ { f f } \in \mathbb { R } ^ { d n _ { f } \times d n _ { f } }$ is symmetric and positive semidefinite.

Proof: The submatrix $\boldsymbol { B } _ { f f }$ can be written as $\boldsymbol { B } _ { f f } =$ $\boldsymbol { B } _ { 0 } + \boldsymbol { D }$ where $B _ { 0 } \in \mathbb { R } ^ { d n _ { f } \times d n _ { f } ^ { \ast } }$ is the bearing Laplacian for the subgraph of the followers and $\mathcal { D } \in \mathbb { R } ^ { d n _ { f } \times d n _ { f } }$ is a positive semidefinite block-diagonal matrix with $[ \mathcal { D } ] _ { i i } =$ $\sum _ { j \in \mathcal { V } _ { \ell } \cap \mathcal { N } _ { i } } P _ { g _ { i j } ^ { * } }$ for $i \in \mathcal { V } _ { f }$ . Note $B _ { 0 }$ is symmetric because the edges among the followers are assumed to be bidirectional. For any $\boldsymbol { x } = \left[ x _ { 1 } ^ { T } , \ldots , x _ { n _ { f } } ^ { T } \right] ^ { T } \in \mathbb { R } ^ { d n _ { f } }$ , we have $x ^ { T } B _ { 0 } x =$ $\begin{array} { r } { \sum _ { i \in \mathcal { V } _ { f } } \sum _ { j \in \mathcal { V } _ { f } \cap \mathcal { N } _ { i } } \| P _ { g _ { i j } ^ { * } } ( x _ { i } - \bar { x _ { j } } ) \| ^ { 2 } \geq 0 } \end{array}$ and, hence, $B _ { 0 }$ is positive semidefinite. Since D is also positive semidefinite, the matrix $B _ { f f }$ is positive semidefinite. -

2) Uniqueness of the Target Formation: Based on the bearing Laplacian, we can analyze the existence and uniqueness of the target formation $p ^ { * }$ (i.e., the existence and uniqueness of solutions to the equations in Definition 1). The bearing constraints and leader positions are feasible if there exists at least one formation that satisfies them. Feasible bearings and leader positions may be calculated from an arbitrary formation configuration that has the desired geometric pattern. In general, given a set of feasible bearing constraints and leader positions, the target formation may not be unique. [See, for example, Fig. 2(a)]. In fact, the uniqueness problem of the target formation is identical to the localizability problem in bearing-only network localization [20]. We next give the necessary and sufficient condition for uniqueness of the target formation.

Theorem 1 (Uniqueness of the Target Formation): Given feasible bearing constraints and leader positions, the target d on March 19,2026 at 11:34:34 UTC from IEEE Xplore. Restrictions apply.

formation in Definition 1 is unique if and only if $B _ { f f }$ is nonsingular. When $\boldsymbol { B } _ { f f }$ is nonsingular, the position and velocity of the followers in the target formation are uniquely determined as

$$
p _ {f} ^ {*} (t) = - \mathcal {B} _ {f f} ^ {- 1} \mathcal {B} _ {f \ell} p _ {\ell} (t), v _ {f} ^ {*} (t) = - \mathcal {B} _ {f f} ^ {- 1} \mathcal {B} _ {f \ell} v _ {\ell} (t).\tag{4}
$$

Proof: As shown in [20], the target formation is uniquely determined by the bearings and leader positions if and only if $\boldsymbol { B } _ { f f }$ is nonsingular. It follows from Lemma 1 that $B p ^ { * } = 0 .$ which further implies ${ \boldsymbol { { B } } _ { f f } } { \boldsymbol { p } } _ { f } ^ { * } + { \boldsymbol { { B } } _ { f \ell } } { \boldsymbol { p } } _ { \ell } = 0$ . When $B _ { f f }$ is nonsingular, $p _ { f } ^ { * } = - B _ { f f } ^ { - 1 } B _ { f \ell } p _ { \ell }$ . Then, $v _ { f } ^ { * } = \dot { p } _ { f } ^ { * } = - B _ { f f } ^ { - 1 } B _ { f \ell } v _ { \ell }$

A variety of other conditions for uniqueness of the target formation can be found in [20]. Here, we highlight two useful conditions. A useful necessary condition is that a unique target formation must have at least two leaders. In this paper, we always assume there exist at least two leaders. A useful sufficient condition is that the target formation is unique if it is infinitesimally bearing rigid and has at least two leaders. By the sufficient condition, in order to design a unique target formation, we can first design an infinitesimally bearing rigid formation and then arbitrarily assign two agents as leaders. Fig. 2(b) and (c) shows examples of unique target formations. More examples can be found in [20]. For the analysis in the sequel, we adopt the following uniqueness assumption.

Assumption 1: The target formation $\mathcal { G } ( p ^ { * } ( t ) )$ is unique for all $t \geq 0 ,$ which means $\boldsymbol { B } _ { f f }$ is nonsingular.

3) Target Formation Maneuvering: In bearing-based formation maneuver control, the desired translational and scaling maneuver of the formation is known only to the leaders. In order to achieve the desired maneuvers, the leaders must have appropriate motions. We now study how the leaders should move to achieve the desired maneuvers of the target formation. Formation control laws will be designed later such that the real formation is steered to track the target formation.

To describe the translational and scaling maneuvers, we define the centroid, $c ( p ^ { * } ( t ) )$ , and the scale, $s ( p ^ { * } ( t ) )$ ), for the target formation as

$$
\begin{array}{l} c \left(p ^ {*} (t)\right) \triangleq \frac {1}{n} \sum_ {i \in \mathcal {V}} p _ {i} ^ {*} (t) = \frac {1}{n} (\mathbf {1} _ {n} \otimes I _ {d}) ^ {T} p ^ {*} (t) \\ s \left(p ^ {*} (t)\right) \triangleq \sqrt {\frac {1}{n} \sum_ {i \in \mathcal {V}} \| p _ {i} ^ {*} (t) - c (p ^ {*} (t)) \| ^ {2}} \\ = \frac {1}{\sqrt {n}} \left\| p ^ {*} (t) - \mathbf {1} _ {n} \otimes c (p ^ {*} (t)) \right\|. \end{array}
$$

The desired maneuvering dynamics of the centroid and scale of the target formation are given by

$$
\dot {c} \left(p ^ {*} (t)\right) = v _ {c} (t), \quad \dot {s} \left(p ^ {*} (t)\right) = \alpha (t) s \left(p ^ {*} (t)\right)\tag{5}
$$

where $v _ { c } ( t ) \in \mathbb { R } ^ { d }$ denotes the desired velocity common to all agents and $\alpha ( t ) \in \mathbb { R }$ is the varying rate of the scale. The formation scale expands when $\alpha ( t ) > 0$ and contracts when $\alpha ( t ) < 0$ . Suppose $v _ { c } ( t )$ and $\alpha ( t )$ are known by the leaders. We next show how the leaders should move to achieve the desired dynamics in (5).

Theorem 2 (Target Formation Maneuvering): The desired dynamics of the centroid and the scale given in (5) are achieved if the velocities of the leaders have the form of

$$
v _ {i} (t) = v _ {c} (t) + \alpha (t) \left[ p _ {i} (t) - c \left(p ^ {*} (t)\right) \right], \quad i \in \mathcal {V} _ {\ell}.\tag{6}
$$

Proof: The vector form of (6) is $v _ { \ell } ( t ) = \mathbf { 1 } _ { n _ { \ell } } \otimes v _ { c } ( t )$ + $\alpha ( t ) [ p _ { \ell } ( t ) - \mathbf { 1 } _ { n _ { \ell } } \otimes c ( p ^ { * } ( t ) ) ]$ ]. Since span $\{ \mathbf { 1 } _ { n } \otimes I _ { d } , p ^ { * } \} \subseteq \mathrm { N u l l } ( B )$ as given in Lemma 1, we have

$$
\mathcal {B} \left(\mathbf {1} _ {n} \otimes v _ {c} (t) + \alpha (t) \left(p ^ {*} (t) - \mathbf {1} _ {n} \otimes c \left(p ^ {*} (t)\right)\right)\right) = 0.
$$

The above equation implies $B _ { f \ell } v _ { \ell } + B _ { f f } [ { \bf 1 } _ { n _ { f } } \otimes v _ { c } ( t ) +$ $\alpha ( t ) [ p _ { f } ^ { * } ( t ) - \mathbf { 1 } _ { n _ { f } } \otimes c ( p ^ { * } ( t ) ) ] = 0$ . Then, $v _ { f } ^ { \ast } ( t )$ is calculated as

$$
\begin{array}{c} v _ {f} ^ {*} (t) = \mathcal {B} _ {f f} ^ {- 1} \mathcal {B} _ {f \ell} v _ {\ell} (t) \\ = \mathbf {1} _ {n _ {f}} \otimes v _ {c} (t) + \alpha (t) \left[ p _ {f} ^ {*} (t) - \mathbf {1} _ {n _ {f}} \otimes c \left(p ^ {*} (t)\right) \right] \end{array}
$$

whose elementwise form is $v _ { i } ^ { * } ( t ) = v _ { c } ( t ) + \alpha ( t ) ( p _ { i } ^ { * } ( t ) -$ $c ( p ^ { * } ( t ) ) )$ for all $i \in \mathcal { V } _ { f }$ . Note $\dot { p } ^ { * } = \left[ v _ { \ell } ^ { T } , \left( v _ { \ell } ^ { * } \right) ^ { T } \right] ^ { T } = \mathbf { 1 } _ { n } \otimes$ $v _ { c } ( t ) + \alpha ( t ) ( p ^ { * } - { \bf 1 } _ { n } \otimes c ( p ^ { * } ) )$ . Substituting $\dot { p } ^ { * }$ into $\dot { c } ( p ^ { * } )$ and $\dot { s } ( p ^ { * } )$ gives

$$
\begin{array}{r l} & {\dot {c} (p ^ {*}) = \frac {1}{n} (\mathbf {1} _ {n} \otimes I _ {d}) ^ {T} \dot {p} ^ {*}} \\ & {\qquad = \frac {1}{n} (\mathbf {1} _ {n} \otimes I _ {d}) ^ {T} [ \mathbf {1} _ {n} \otimes v _ {c} (t) + \alpha (t) (p ^ {*} - \mathbf {1} _ {n} \otimes c (p ^ {*})) ]} \\ & {\qquad = \frac {1}{n} (\mathbf {1} _ {n} \otimes I _ {d}) ^ {T} (\mathbf {1} _ {n} \otimes v _ {c} (t)) = v _ {c} (t)} \end{array}
$$

$$
\begin{array}{r l} & {\dot {s} (p ^ {*}) = \frac {1}{\sqrt {n}} \frac {(p ^ {*} - \mathbf {1} _ {n} \otimes c (p ^ {*})) ^ {T}}{\| p ^ {*} - \mathbf {1} _ {n} \otimes c (p ^ {*}) \|} (\dot {p} ^ {*} - \mathbf {1} _ {n} \otimes v _ {c} (t))} \\ & {\qquad = \frac {1}{\sqrt {n}} \frac {(p ^ {*} - \mathbf {1} _ {n} \otimes c (p ^ {*})) ^ {T}}{\| p ^ {*} - \mathbf {1} _ {n} \otimes c (p ^ {*}) \|} \alpha (t) (p ^ {*} - \mathbf {1} _ {n} \otimes c (p ^ {*}))} \\ & {\qquad = \alpha (t) s (p ^ {*}).} \end{array}
$$

As shown in (6), the velocity of each leader should be a linear combination of the common translational velocity and the velocity induced by the scaling variation. In addition to $v _ { c } ( t )$ and $\alpha ( t )$ , each leader should also know the centroid $c ( p ^ { * } ( t ) )$ , which is a global information of the target formation. This quantity may be estimated in a distributed way using, for example, consensus filters, as described in [22].

## III. BEARING-BASED FORMATION CONTROL LAWS

In this section, we propose two distributed control laws to steer the followers to track the maneuvering target formation. The first control law requires relative position and velocity feedback; with this control law, the formation tracks target formations with constant velocities. The second control law requires position, velocity, and acceleration feedback; with this control law, the formation tracks target formations with timevarying velocities.

![](images/6319cf68a42dc0db2ce6363ec27a887129152267b440eb8840250c4db23ad9dc.jpg)  
Fig. 3. Geometric meaning of the term $P _ { g _ { i j } ^ { * } } ( p _ { j } - p _ { i } )$ in control law (7).

## A. Formation Control With Constant Leader Velocity

The bearing-based control law for follower $i \in \mathcal { V } _ { f }$ is proposed as

$$
u _ {i} = - \sum_ {j \in \mathcal {N} _ {i}} P _ {g _ {i j} ^ {*}} \left[ k _ {p} (p _ {i} - p _ {j}) + k _ {v} (v _ {i} - v _ {j}) \right]\tag{7}
$$

where $P _ { g _ { i j } ^ { * } } = I _ { d } - g _ { i j } ^ { * } ( g _ { i j } ^ { * } ) ^ { T }$ is a constant orthogonal projection matrix, and $k _ { p }$ and $k _ { v }$ are positive constant control gains. Several remarks on the control law are given below. First, the neighbor $j \in \mathcal N _ { i }$ of agent i may be either a follower or a leader. Second, the proposed control law has a clear geometric meaning illustrated in Fig. 3: the control term $P _ { g _ { i j } ^ { * } } ( p _ { j } - p _ { i } )$ steers agent i to a position where $g _ { i j }$ is aligned with $g _ { i j } ^ { * }$ . Third, the proposed control law has a similar form as the second-order linear consensus protocols [23]–[25]. The difference is that in the consensus protocols, the weight for each edge is a positive scalar, whereas in the proposed control law, the weight for each edge is a positive semidefinite orthogonal projection matrix. It is precisely the special properties of the projection matrices that allows the proposed control law to solve the bearing-based formation control problem. The convergence of control law (7) is analyzed below.

Theorem 3: Under control law $( 7 ) ,$ when the leader velocity $v _ { \ell } ( t )$ is constant, the tracking errors $\delta _ { p } ( t )$ and $\delta _ { v } ( t )$ as defined in (1) globally and exponentially converge to zero.

Proof: With control law (7), the dynamics of the followers can be expressed in a matrix-vector form as

$$
\begin{array}{c} \dot {v} _ {f} = - k _ {p} (\mathcal {B} _ {f f} p _ {f} + \mathcal {B} _ {f \ell} p _ {\ell}) - k _ {v} (\mathcal {B} _ {f f} v _ {f} + \mathcal {B} _ {f \ell} v _ {\ell}) \\ = - k _ {p} \mathcal {B} _ {f f} \delta_ {p} - k _ {v} \mathcal {B} _ {f f} \delta_ {v} \end{array}\tag{8}
$$

where the second equality is due to the fact that $\delta _ { p } = p _ { f } +$ $B _ { f f } ^ { - 1 } B _ { f \ell } p _ { \ell }$ and $\delta _ { v } = v _ { f } + B _ { f f } ^ { - 1 } B _ { f \ell } v _ { \ell }$ as shown in (4). Substituting (8) into the error dynamics gives $\dot { \delta } _ { v } = \dot { v } _ { f } + B _ { f f } ^ { - 1 } B _ { f \ell } \dot { v } _ { \ell } =$ $- k _ { p } B _ { f f } \delta _ { p } - k _ { v } B _ { f f } \delta _ { v } + B _ { f f } ^ { - 1 } B _ { f \ell } \dot { v } _ { \ell }$ , which can be rewritten in a compact form as

$$
\left[ \begin{array}{c} \dot {\delta} _ {p} \\ \dot {\delta} _ {v} \end{array} \right] = \left[ \begin{array}{c c} 0 & I \\ - k _ {p} \mathcal {B} _ {f f} & - k _ {v} \mathcal {B} _ {f f} \end{array} \right] \left[ \begin{array}{c} \delta_ {p} \\ \delta_ {v} \end{array} \right] + \left[ \begin{array}{c} 0 \\ \mathcal {B} _ {f f} ^ {- 1} \mathcal {B} _ {f \ell} \end{array} \right] \dot {v} _ {\ell}.\tag{9}
$$

Let λ be an eigenvalue of the state matrix of (9). The characteristic equation of the state matrix is given by $\operatorname* { d e t } ( \lambda ^ { 2 } I +$ $\lambda k _ { v } B _ { f f } + k _ { p } B _ { f f } ) = 0$ . It can be calculated that $\lambda = ( - k _ { v } \mu \pm$ $\sqrt { k _ { v } ^ { 2 } \mu ^ { 2 } - 4 k _ { p } \mu } ) / 2$ , where $\mu > 0$ is an eigenvalue of $\boldsymbol { B } _ { f f }$ Therefore, $\mathrm { R e } ( \lambda ) < 0$ for any $k _ { p } , k _ { v } , \mu > 0$ . As a result, the state matrix is Hurwitz and, hence, $\delta _ { p }$ and $\delta _ { v }$ globally and exponentially converge to zero when $\dot { v } _ { \ell } \equiv 0$ -

When $v _ { \ell } ( t )$ is time-varying $( \mathrm { i } . \mathrm { e } . , \dot { v } \ell ( t )$ is not identically zero), the tracking errors may not converge to zero according to the error dynamics (9). In order to perfectly track target formations with time-varying $v _ { \ell } ( t )$ , additional acceleration feedback is required as shown in the next subsection. In practical tasks where the desired target formation has piecewise constant velocities, the control law (7) may still give satisfactory performance.

![](images/a681fd03f8cff150907d88c310d02b1f381d9cf773c12ff7303fd4f459f5808b.jpg)  
(a)

kp=0.5, kv=2  
![](images/882303fcc3e2ec9dd0ef557f09880f42aad7ecf40ec2e5e2eda16e3a65bac540.jpg)

(b)  
![](images/94c1cf9dfc1db6246728abd8d12bce58385e2c599cc6f95eacb46044127021ac.jpg)

![](images/c8b2c0a3327c89aae1ffcb05d965a7c70a012e9bd43ef8fc037c73d833bba91b.jpg)  
(c)  
Fig. 4. Simulation example to demonstrate control law (7). (a) Trajectory. (b) Total bearing error $\textstyle \sum _ { ( i , j ) \in { \mathcal { E } } } ^ { \bullet } \| g _ { i j } ( t ) - g _ { i j } ^ { * } \| . ( \mathbf { c } )$ Velocity.

A simulation example is given in Fig. 4 to illustrate control law (7). The target formation in this example is the square shown in Fig. 2(b). There are two leaders and two followers. As shown in Fig. 4(a) and (b), the translation and scale of the formation are continuously varying and, in the meantime, the desired formation pattern is maintained. In Fig. 4(c), the x-velocity of each follower converges to a value smaller than that of the leaders, because the velocity of a follower is a combination of the translational and scaling velocities, and the scaling velocity in the x-direction is negative in this example.

## B. Formation Control With Time-Varying Leader Velocity

Now consider the case where $v _ { \ell } ( t )$ is time-varying $( \mathrm { i } . \mathrm { e } . , \dot { v } \ell ( t )$ is not identically zero). Assume $\dot { v } _ { \ell } ( t )$ is piecewise continuous. The following control law handles the time-varying case:

$$
u _ {i} = - K _ {i} ^ {- 1} \sum_ {j \in \mathcal {N} _ {i}} P _ {g _ {i j} ^ {*}} \left[ k _ {p} (p _ {i} - p _ {j}) + k _ {v} (v _ {i} - v _ {j}) - \dot {v} _ {j} \right]\tag{10}
$$

where $\begin{array} { r } { K _ { i } = \sum _ { j \in \mathcal { N } _ { i } } P _ { g _ { i j } ^ { * } } } \end{array}$ . Compared to control law $( 7 ) .$ , control law (10) requires the acceleration of each neighbor. The design of control law (10) is inspired by the consensus protocols for tracking time-varying references as proposed in [2] and [23].

The nonsingularity of $K _ { i }$ is guaranteed by the uniqueness of the target formation as shown in the following result.

Lemma 3: The matrix $K _ { i }$ is nonsingular for all $i \in \mathcal { V } _ { f }$ if the target formation is unique.

Proof: First of all, the matrix $K _ { i }$ is singular if and only if the bearings $\left\{ g _ { i j } ^ { * } \right\} _ { j \in \mathcal { N } _ { i } }$ are collinear, because for any $x \in \mathbb { R } ^ { d }$ $\begin{array} { r } { x ^ { T } K _ { i } x = 0 \Leftrightarrow \underset { j \in \mathcal { N } _ { i } } { \hookrightarrow } x ^ { - } P _ { g _ { i j } ^ { * } } x = 0 \Leftrightarrow P _ { g _ { i j } ^ { * } } x = 0 , \forall j \in \mathcal { N } _ { i } } \end{array}$ . Since Null $( P _ { g _ { i j } ^ { * } } ) = \mathrm { s p a n } \{ g _ { i j } ^ { * } \}$ , we know $x ^ { T } K _ { i } x = 0$ if and only if $x$ and $\left\{ g _ { i j } ^ { * } \right\} _ { j \in \mathcal { N } _ { i } }$ are collinear. If $\left\{ g _ { i j } ^ { * } \right\} _ { j \in \mathcal { N } _ { i } }$ are collinear, the follower $p _ { i } ^ { * }$ cannot be uniquely determined in the target formation because $p _ { i } ^ { * }$ can move along $g _ { i j } ^ { * }$ without changing any bearings. As a result, if $K _ { i }$ is singular, the target formation is not unique. -

The convergence of control law (10) is analyzed below.

Theorem 4: Under control law (10), for any time-varying leader velocity $v _ { \ell } ( t )$ , the tracking errors $\delta _ { p } ( t )$ and $\delta _ { v } ( t )$ as defined in (1) globally and exponentially converge to zero.

Proof: Multiplying $K _ { i }$ on both sides of control law (10) gives

$$
\sum_ {j \in \mathcal {N} _ {i}} P _ {g _ {i j} ^ {*}} (\dot {v} _ {i} - \dot {v} _ {j}) = \sum_ {j \in \mathcal {N} _ {i}} P _ {g _ {i j} ^ {*}} [ - k _ {p} (p _ {i} - p _ {j}) - k _ {v} (v _ {i} - v _ {j}) ]
$$

whose matrix-vector form is

$$
\begin{array}{c} \mathcal {B} _ {f f} \dot {v} _ {f} + \mathcal {B} _ {f \ell} \dot {v} _ {\ell} = - k _ {p} (\mathcal {B} _ {f f} p _ {f} + \mathcal {B} _ {f \ell} p _ {\ell}) - k _ {v} (\mathcal {B} _ {f f} v _ {f} + \mathcal {B} _ {f \ell} v _ {\ell}) \\ = - k _ {p} \mathcal {B} _ {f f} \delta_ {p} - k _ {v} \mathcal {B} _ {f f} \delta_ {v}. \end{array}
$$

It follows that $\dot { v } _ { f } = - k _ { p } \delta _ { p } - k _ { v } \delta _ { v } - B _ { f f } ^ { - 1 } B _ { f \ell } \dot { v } _ { \ell }$ . Then, the tracking error dynamics are $\dot { \delta } _ { p } = \delta _ { v }$ and $\dot { \delta } _ { v } = \dot { v } _ { f } + B _ { f f } ^ { - 1 } B _ { f \ell } \dot { v } _ { \ell } =$ $- k _ { p } \delta _ { p } + k _ { v } \delta _ { v }$ , which are expressed in a compact form as

$$
\left[ \begin{array}{c} \dot {\delta} _ {p} \\ \dot {\delta} _ {v} \end{array} \right] = \left[ \begin{array}{c c} 0 & I \\ - k _ {p} I & - k _ {v} I \end{array} \right] \left[ \begin{array}{c} \delta_ {p} \\ \delta_ {v} \end{array} \right].\tag{11}
$$

The eigenvalue of the state matrix is $\lambda = ( - k _ { v } \pm$ $\sqrt { k _ { v } ^ { 2 } - 4 k _ { p } } ) / 2$ , which is always in the open left-half plane for any $k _ { p } , k _ { v } > 0$ . The global and exponential convergence result follows. -

By comparing the error dynamics in (11) and (9), we see that the role of the acceleration feedback in control law (10) is to eliminate the term that contains $\dot { v } _ { \ell } ( t )$ so that it does not affect the convergence of the errors.

A simulation example is shown in Fig. 5 to illustrate control law (10). The target formation in this example is the 3-D cube shown in Fig. 2(c), which has two leaders and six followers. As shown in Fig. 5(a) and (b), the translation and scale of the formation are continuously varying and, in the meantime, the formation converges from an initial configuration to the desired pattern. Although the velocities of the leaders are time-varying, the desired formation pattern is maintained exactly during the formation evolution.

![](images/e1a8cdfd7d749465928d0259cc88379db26dd3e7321a7b5e55858b918489e7c7.jpg)

(a)  
![](images/4accb078d54879ad3ce965da18b832130496c5d356514c54d999af73a0233f5d.jpg)

(b)  
![](images/bdbec8d74cf1c021e00ef71a7a8534e1b14e3e5fd5cec64e43da71ac15e910c6.jpg)

![](images/2bcf4686df461b7feb7526932e2f03a8b14997f2366b08ef90f044b3e4a6f242.jpg)

![](images/92b410a91dd8e390ccb85e910cabe29c58fcfd4cbfdb31df26be1807d72c8ae0.jpg)  
(c)  
Fig. 5. Simulation example to demonstrate control law (10). (a) Trajectory. (The dark area stands for an obstacle.) (b) Total bearing error $\begin{array} { r } { \sum _ { ( i , j ) \in \mathcal { E } } \| g _ { i j } ( t ) - g _ { i j } ^ { * } \| } \end{array}$ . (c) Velocity.

The simulation example also demonstrates that the proposed control law can be used for obstacle avoidance, such as passing through narrow passages. In practice, collision avoidance requires sophisticated mechanisms, such as obstacle detection and path generation [26]. Details on obstacle avoidance are out of the scope of this paper.

## IV. BEARING-BASED FORMATION CONTROL WITH PRACTICAL ISSUES

In this section, we consider bearing-based formation control in the presence of some issues that may appear in practical on March 19,2026 at 11:34:34 UTC from IEEE Xplore. Restrictions apply.

implementations, including input disturbances, input saturation, and collision avoidance among the agents.

## A. Constant Input Disturbance

Suppose there exists an unknown constant input disturbance for each follower. The dynamics of follower $i \in \mathcal { V } _ { f }$ are

$$
\dot {p} _ {i} = v _ {i}, \quad \dot {v} _ {i} = u _ {i} + \mathrm{w} _ {i}
$$

where $\mathbf { w } _ { i } \in \mathbb { R } ^ { d }$ is an unknown constant signal, and let $\mathbf { w } _ { f } =$ $\big [ \mathbf { w } _ { 1 } ^ { T } , \dots , \mathbf { w } _ { n _ { f } } ^ { T } \big ] ^ { T }$ . In practice, the constant input disturbance might be caused by, for example, constant sensor or actuator biases. In order to handle the input disturbance, we add an integral control term to control law (7) and obtain

$$
u _ {i} = - \sum_ {j \in \mathcal {N} _ {i}} P _ {g _ {i j} ^ {*}} \left[ k _ {p} (p _ {i} - p _ {j}) + k _ {v} (v _ {i} - v _ {j}) + k _ {I} \int_ {0} ^ {t} (p _ {i} - p _ {j}) \mathrm{d} \tau \right]\tag{12}
$$

where $k _ { I } > 0$ is the constant integral control gain. We next show that the integral control will not only eliminate the impact of the constant disturbance but will also handle the case where $\dot { v } _ { \ell } ( t )$ is nonzero and constant.

Theorem 5: Consider the control law (12) with constant disturbance $\mathbf { W } _ { f }$ and constant leader acceleration $\dot { v } _ { \ell }$ . If the control gains satisfy $0 < k _ { I } < k _ { p } k _ { v } \lambda _ { \operatorname* { m i n } } ( B _ { f f } )$ , then the tracking errors $\delta _ { p } ( t )$ and $\delta _ { v } ( t )$ globally and exponentially converge to zero.

Proof: The matrix-vector form of control law (12) is

$$
\begin{array}{l} \dot {v} _ {f} = - k _ {I} \int_ {0} ^ {t} (\mathcal {B} _ {f f} p _ {f} + \mathcal {B} _ {f \ell} p _ {\ell}) \mathrm{d} \tau - k _ {p} (\mathcal {B} _ {f f} p _ {f} + \mathcal {B} _ {f \ell} p _ {\ell}) \\ \qquad - k _ {v} (\mathcal {B} _ {f f} p _ {f} + \mathcal {B} _ {f \ell} p _ {\ell}) + \mathbf {w} _ {f} \\ \qquad = - k _ {I} \mathcal {B} _ {f f} \int_ {0} ^ {t} \delta_ {p} \mathrm{d} \tau - k _ {p} \mathcal {B} _ {f f} \delta_ {p} - k _ {v} \mathcal {B} _ {f f} \delta_ {v} + \mathbf {w} _ {f}. \end{array}
$$

Denote $\begin{array} { r } { \eta \overset { \Delta } { = } \int _ { 0 } ^ { t } \delta _ { p } \mathrm { d } \tau } \end{array}$ . It then follows that $\dot { \eta } = \delta _ { p } , ~ \dot { \delta } _ { p } = \delta _ { v } .$ and $\dot { \delta } _ { v } = \dot { v } _ { f } + \mathcal { B } _ { f f } ^ { - 1 } \mathcal { B } _ { f \ell } \dot { v } _ { \ell } = - k _ { I } \mathcal { B } _ { f f } \eta - k _ { p } \mathcal { B } _ { f f } \delta _ { p } - k _ { v } \mathcal { B } _ { f f } \delta _ { v }$ $+ \mathbf { w } _ { f } + B _ { f f } ^ { - 1 } B _ { f \ell } \dot { v } _ { \ell }$ , the matrix-vector form of which is given by

$$
\begin{array}{c} \left[ \begin{array}{c} \dot {\eta} \\ \dot {\delta} _ {p} \\ \dot {\delta} _ {v} \end{array} \right] = \left[ \begin{array}{c c c} 0 & I & 0 \\ 0 & 0 & I \\ - k _ {I} \mathcal {B} _ {f f} & - k _ {p} \mathcal {B} _ {f f} & - k _ {v} \mathcal {B} _ {f f} \end{array} \right] \left[ \begin{array}{c} \eta \\ \delta_ {p} \\ \delta_ {v} \end{array} \right] \\ + \left[ \begin{array}{c} 0 \\ 0 \\ \mathrm{w} _ {f} \end{array} \right] + \left[ \begin{array}{c} 0 \\ 0 \\ \mathcal {B} _ {f f} ^ {- 1} \mathcal {B} _ {f \ell} \end{array} \right] \dot {v} _ {\ell}. \end{array}\tag{13}
$$

Denote A as the state matrix of the above dynamics with λ an associated eigenvalue. We next identify the condition for $\mathrm { R e } ( \lambda ) < 0$ . Note the state matrix is in the controllable canonical form. Then, the characteristic polynomial is

$$
\det (\lambda I - A) = \det (\lambda^ {3} I + k _ {v} \mathcal {B} _ {f f} \lambda^ {2} + k _ {p} \mathcal {B} _ {f f} \lambda + k _ {I} \mathcal {B} _ {f f}).
$$

As a result, $\lambda ^ { 3 }$ can be viewed as an eigenvalue of the matrix $- ( k _ { v } \lambda ^ { 2 } + k _ { v } \lambda + k _ { I } ) B _ { f f }$ . By denoting μ as an eigenvalue of $B _ { f f }$ , we have $\lambda ^ { 3 } + k _ { v } \bar { \mu } \lambda ^ { 2 } + k _ { p } \mu \lambda + k _ { I } \mu = 0$ . By the Routh-Hurwitz stability criterion, we have $\mathrm { R e } ( \lambda ) < 0$ if and only $k _ { v } \mu , k _ { p } \mu , k _ { I } \mu > 0$ and $( k _ { v } \mu ) ( k _ { p } \mu ) > k _ { I } \mu$ . Since $k _ { v } , k _ { p } , \mu > 0 ,$ we have $0 < k _ { I } < k _ { v } k _ { p } \mu$ . In order to make $\mathrm { R e } ( \lambda ) < 0$ for all $\mu ,$ it is required $0 < k _ { I } < k _ { v } k _ { p } \lambda _ { \operatorname* { m i n } } ( B _ { f f } )$ where $\lambda _ { \operatorname* { m i n } } ( B _ { f f } )$ is the minimum eigenvalue of $\boldsymbol { B } _ { f f }$ . When A is Hurwitz, given constant $\mathbf { W } _ { f }$ and $\dot { v } _ { \ell } .$ the steady state is $\delta _ { p } ( \infty ) = \delta _ { v } ( \infty ) = 0$ and $\eta ( \infty ) = - \dot { \mathcal { B } } _ { f f } ^ { - 1 } ( \mathbf { w } _ { f } + \mathcal { B } _ { f f } ^ { - 1 } \mathcal { B } _ { f \ell } \dot { v } _ { \ell } ) / k _ { I }$ -

As can be seen from the error dynamics (13), when v˙<sub>-</sub> is constant, it has the same impact as an input disturbance and, hence, is handled by the integral control. The idea of integral control has also been applied in consensus, distance-based, and bearing-based formation maneuver control problems [21], [27], [28]. It is also interesting to note that the integral control gain must be bounded by $\lambda _ { \operatorname* { m i n } } ( B _ { f f } )$ , which we expect should have graph-theoretic interpretations and is the subject of future work.

Similarly, by adding an integral control term to control law (10), we obtain the following control law that can handle the unknown constant input disturbance and time-varying $v _ { \ell } ( t )$

$$
\begin{array}{c} u _ {i} = - K _ {i} ^ {- 1} \sum_ {j \in \mathcal {N} _ {i}} P _ {g _ {i j} ^ {*}} \left[ k _ {p} (p _ {i} - p _ {j}) + k _ {v} (v _ {i} - v _ {j}) - \dot {v} _ {j} \right. \\ \qquad \qquad \qquad \qquad \qquad \qquad \qquad \qquad \qquad \qquad \qquad \qquad \qquad \qquad \qquad + k _ {I} \int_ {0} ^ {t} (p _ {i} - p _ {j}) \mathrm{d} \tau \Bigg ]. \end{array}\tag{14}
$$

The convergence result for control law (14) is given below. The proof is similar to Theorem 5 and omitted.

Theorem 6: Consider the control law (12) with constant disturbance $\mathbf { W } _ { f }$ and time-varying leader velocity $v _ { \ell } ( t )$ . If the control gains satisfy $0 < k _ { I } < k _ { p } k _ { v }$ , then the tracking errors $\delta _ { p } ( t )$ and $\delta _ { v } ( t )$ globally and exponentially converge to zero.

## B. Acceleration Saturation

In practical implementations, the acceleration input is always bounded. In the presence of acceleration saturation, the control law (7) becomes

$$
u _ {i} = \operatorname{sat} \left\{- \sum_ {j \in \mathcal {N} _ {i}} P _ {g _ {i j} ^ {*}} \left[ k _ {p} (p _ {i} - p _ {j}) + k _ {v} (v _ {i} - v _ {j}) \right] \right\}\tag{15}
$$

where $\operatorname { s a t } ( \cdot )$ is a saturation function that is either $\operatorname { s a t } ( x ) =$ sign(x) min $\{ | x | , \beta \}$ or $\mathrm { s a t } ( x ) = \beta \operatorname { t a n h } ( x )$ where $x \in \mathbb { R }$ and $\beta > 0$ is the constant bound for $| x |$ . For a vector $x =$ $[ x _ { 1 } , \dots , x _ { q } ] ^ { T } \in \mathbb { R } ^ { q } , \ \mathrm { s a t } ( x )$ is defined component-wise as s ${ \mathfrak { u } } ( x ) = [ { \bar { \mathrm { s a t } } } ( x _ { 1 } ) , \dots , { \mathrm { s a t } } ( x _ { q } ) ] ^ { T }$

Due to the saturation function, the formation dynamics become nonlinear and the formation stability can be proven by a Lyapunov approach. Inspired by the work in [25], we introduce the integral function $\begin{array} { r } { \Phi ( x ) \stackrel { \Delta } { = } \int _ { 0 } ^ { x } } \end{array}$ sat(τ )dτ for $x \in \mathbb { R }$ . Due to the properties of sat(·), we have that $\Phi ( x ) \geq 0$ for all $x \in \mathbb { R }$ and $\Phi ( x ) = 0$ if and only if $x = 0$ . In the case of $\operatorname { s a t } ( x ) =$ β tanh(x), we have $\Phi ( x ) = \beta \log ( \cosh ( x ) )$ . For a vector $x =$ $[ x _ { 1 } , \ldots , x _ { q } ] ^ { T } \in \mathbb { R } ^ { q } , \Phi ( x )$ is defined component-wise as

$$
\Phi (x) = \left[ \int_ {0} ^ {x _ {1}} \operatorname{sat} (\tau) \mathrm{d} \tau , \dots , \int_ {0} ^ {x _ {q}} \operatorname{sat} (\tau) \mathrm{d} \tau \right] ^ {T} \in \mathbb {R} ^ {q}.
$$

The useful properties of $\Phi ( \cdot )$ and $\operatorname { s a t } ( \cdot )$ are given below.

Lemma 4: Given $\ b { x } ( t ) \in \mathbb { R } ^ { q }$ , the quantity $\mathbf { 1 } ^ { T } \Phi ( x )$ satisfies

a) $\mathbf { 1 } ^ { T } \Phi ( x ) \geq 0$ and $\mathbf { 1 } ^ { T } \Phi ( x ) = 0$ if and only if $x = 0 ;$ b) $\mathrm { d } ( { \bf 1 } ^ { T } \Phi ( x ) ) / \mathrm { d } t = \dot { x } ^ { T } \mathrm { s a t } ( x ) .$

Authorized licensed use limited to: CITY UNIV OF HONG KONG. Downloaded on March 19,2026 at 11:34:34 UTC from IEEE Xplore. Restrictions apply.

Proof: Note $\begin{array} { r } { \mathbf { 1 } ^ { T } \Phi ( x ) = \sum _ { i = 1 } ^ { q } \Phi ( x _ { i } ) } \end{array}$ . Since $\Phi ( x _ { i } ) \geq 0$ and $\Phi ( x _ { i } ) = 0$ if and only if $x _ { i } = 0 ;$ , property (a) is proven. The time derivative of $\mathbf { 1 } ^ { T } \Phi ( x )$ is given by $\mathrm { d } ( \mathbf { 1 } ^ { T } \Phi ( x ) ) / \mathrm { d } t =$ $\begin{array} { r } { \sum _ { i = 1 } ^ { q } \dot { \Phi } ( x _ { i } ) = \sum _ { i = 1 } ^ { q } \dot { x } _ { i } \mathrm { s a t } ( x _ { i } ) = \dot { x } ^ { T } \mathrm { s a t } ( x ) . } \end{array}$

Lemma 5: For any two vectors $x , y \in \mathbb { R } ^ { q } ,$ , it always holds that $y ^ { T } [ \mathrm { s a t } ( x - y ) - \mathrm { s a t } ( x ) ] \leq 0$ and $y ^ { T } [ \mathrm { s a t } ( x ) -$ $\operatorname { s a t } ( x + y ) ] \leq 0$ . Moreover, if sat(·) is strictly monotonic, the equalities hold if and only if $y = 0$

Proof: We only prove the first inequality; the second one can be proven similarly. Note $y ^ { T } ( \operatorname { s a t } ( x - y ) - \operatorname { s a t } ( x ) ) =$ $\textstyle \sum _ { i = 1 } ^ { q } y _ { i } ( \operatorname { s a t } ( x _ { i } - y _ { i } ) - \operatorname { s a t } ( x _ { i } ) )$ . It follows from the monotonicity of the saturation function that sa $\mathsf { t } ( x _ { i } - y _ { i } ) - \mathsf { s a t } ( x _ { i } ) \geq$ $0 \operatorname { i f } y _ { i } < 0$ , and sat $( x _ { i } - y _ { i } ) - \mathrm { s a t } ( x _ { i } ) \leq 0$ if $y _ { i } > 0$ . Therefore, $y _ { i } ( \mathrm { s a t } ( x _ { i } - y _ { i } ) - \mathrm { s a t } ( x _ { i } ) ) \leq 0$ for all $x _ { i } , y _ { i } \in \mathbb { R }$ . If sat(·) is strictly monotonic, $y _ { i } ( \operatorname { s a t } ( x _ { i } - y _ { i } ) - \operatorname { s a t } ( x _ { i } ) ) = 0$ if and only if $y _ { i } = 0$ , which completes the proof. -

With the above preparation, we now analyze the formation stability under control law (15).

Theorem 7: Under control law (15) with a constant leader velocity $v _ { \ell } ( t )$ , the tracking errors $\delta _ { p } ( t )$ and $\delta _ { v } ( t )$ globally and asymptotically converge to zero.

Proof: The matrix-vector form of control law (15) is $\dot { \boldsymbol { v } } _ { f } = \mathrm { s a t } ( - k _ { p } \mathcal { B } _ { f f } \delta _ { p } - k _ { v } \mathcal { B } _ { f f } \delta _ { v } )$ . Substituting $\dot { v } _ { f }$ and $\dot { v } _ { \ell } = 0$ into the tracking error dynamics gives $\begin{array} { r } { \dot { \delta } _ { v } = \dot { v } _ { f } + \dot { B } _ { f f } ^ { - 1 } B _ { f \ell } \dot { v } _ { \ell } = } \end{array}$ sat $\left( - k _ { p } B _ { f f } \delta _ { p } - k _ { v } B _ { f f } \delta _ { v } \right)$ . Consider the Lyapunov function

$$
\begin{array}{r l} V = \mathbf {1} ^ {T} \Phi (- k _ {p} \mathcal {B} _ {f f} \delta_ {p} - k _ {v} \mathcal {B} _ {f f} \delta_ {v}) \\ & + \mathbf {1} ^ {T} \Phi (- k _ {p} \mathcal {B} _ {f f} \delta_ {p}) + k _ {p} \delta_ {v} ^ {T} \mathcal {B} _ {f f} \delta_ {v}. \end{array}
$$

It follows from Lemma 4(a) that $V \geq 0$ and $V = 0$ if and only if $\delta _ { p } = \delta _ { v } = 0$ . According to Lemma $4 ( \mathsf { b } )$ , the time derivative of the Lyapunov function is:

$$
\begin{array}{r} \dot {V} = (- k _ {p} \mathcal {B} _ {f f} \delta_ {v} - k _ {v} \mathcal {B} _ {f f} \dot {\delta} _ {v}) ^ {T} \mathrm{sat} (- k _ {p} \mathcal {B} _ {f f} \delta_ {p} - k _ {v} \mathcal {B} _ {f f} \delta_ {v}) \\ + (- k _ {p} \mathcal {B} _ {f f} \delta_ {v}) ^ {T} \mathrm{sat} (- k _ {p} \mathcal {B} _ {f f} \delta_ {p}) + 2 k _ {p} \delta_ {v} ^ {T} \mathcal {B} _ {f f} \dot {\delta_ {v}}. \end{array}
$$

It follows from sa $\mathrm { t } ( - k _ { p } B _ { f f } \delta _ { p } - k _ { v } B _ { f f } \delta _ { v } ) = \dot { \delta } _ { v }$ that

$$
\begin{array}{r l} & {\dot {V} = - (k _ {p} \mathcal {B} _ {f f} \delta_ {v}) ^ {T} \dot {\delta} _ {v} - (k _ {v} \mathcal {B} _ {f f} \dot {\delta} _ {v}) ^ {T} \dot {\delta} _ {v}} \\ & {\qquad + (- k _ {p} \mathcal {B} _ {f f} \delta_ {v}) ^ {T} \mathrm{sat} (- k _ {p} \mathcal {B} _ {f f} \delta_ {p}) + 2 k _ {p} \delta_ {v} ^ {T} \mathcal {B} _ {f f} \dot {\delta} _ {v}} \\ & {\qquad = - k _ {v} \dot {\delta} _ {v} ^ {T} \mathcal {B} _ {f f} \dot {\delta} _ {v} - (k _ {p} \mathcal {B} _ {f f} \delta_ {v}) ^ {T} \mathrm{sat} (- k _ {p} \mathcal {B} _ {f f} \delta_ {p}) + k _ {p} \delta_ {v} ^ {T} \mathcal {B} _ {f f} \dot {\delta} _ {v}} \\ & {\qquad = - k _ {v} \dot {\delta} _ {v} ^ {T} \mathcal {B} _ {f f} \dot {\delta} _ {v} + k _ {p} \delta_ {v} ^ {T} \mathcal {B} _ {f f}} \\ & {\qquad \times [ \mathrm{sat} (- k _ {p} \mathcal {B} _ {f f} \delta_ {p} - k _ {v} \mathcal {B} _ {f f} \delta_ {v}) - \mathrm{sat} (- k _ {p} \mathcal {B} _ {f f} \delta_ {p}) ]} \end{array}
$$

where the first term $- k _ { v } \dot { \delta } _ { v } ^ { T } B _ { f f } \dot { \delta } _ { \iota }$ is nonpositive and the second term is also nonpositive according to Lemma 5. As a result, $\dot { V } \leq 0$ for all $t \geq 0$

We next identify the invariant set for $\dot { V } = 0$ . When $\dot { V } = 0$ we have

$$
- k _ {v} \dot {\delta} _ {v} ^ {T} \mathcal {B} _ {f f} \dot {\delta} _ {v} = 0\tag{16}
$$

$$
k _ {p} \delta_ {v} ^ {T} \mathcal {B} _ {f f} \left[ \dot {\delta} _ {v} - \operatorname{sat} (- k _ {p} \mathcal {B} _ {f f} \delta_ {p}) \right] = 0.\tag{17}
$$

It follows from (16) that $\dot { \delta } _ { v } = \mathrm { s a t } ( - k _ { p } B _ { f f } \delta _ { p } - k _ { v } B _ { f f } \delta _ { v } ) = 0$ which further implies $k _ { p } B _ { f f } \delta _ { p } = - k _ { v } B _ { f f } \delta _ { v }$ . It then follows from (17) that $k _ { p } \delta _ { v } ^ { T } \mathcal { B } _ { f f } \mathrm { s a t } \rvert ( - \dot { k } _ { v } \mathcal { B } _ { f f } \delta _ { v } ) = 0 .$ , which indicates ${ \cal B } _ { f f } \delta _ { v } = 0 \Leftrightarrow \delta _ { v } \bar { = } 0$ because $B _ { f f } \delta _ { v }$ and sa ${ \bf \nabla } ( B _ { f f } \delta _ { v } )$ have the same sign componentwise. Since $k _ { p } B _ { f f } \delta _ { p } = - k _ { v } B _ { f f } \delta _ { v }$ , we have $\delta _ { p } = 0$ . Therefore, $\dot { V } = 0$ if and only if $\delta _ { p } = \delta _ { v } = 0$ According to the invariance principle, the tracking errors $\delta _ { p }$ and $\delta _ { v }$ globally and asymptotically converge to zero. -

In order to handle input saturation in the case of time-varying $v _ { \ell } ( t )$ , we use the control law

$$
\begin{array}{c} u _ {i} = K _ {i} ^ {- 1} \text {sat} \left\{- \sum_ {j \in \mathcal {N} _ {i}} P _ {g _ {i j} ^ {*}} \left[ k _ {p} (p _ {i} - p _ {j}) + k _ {v} (v _ {i} - v _ {j}) \right] \right\} \\ + K _ {i} ^ {- 1} \sum_ {j \in \mathcal {N} _ {i}} P _ {g _ {i j} ^ {*}} \dot {v} _ {j} \end{array}\tag{18}
$$

where $\begin{array} { r } { K _ { i } = \sum _ { j \in \mathcal { N } _ { i } } P _ { g _ { i j } ^ { * } } } \end{array}$ . Although the saturation function is not applied to the entire acceleration input, the above control law ensures bounded input given arbitrary initial conditions. In particular, under control law (18), the velocity dynamics are $\begin{array} { r } { \sum _ { j \in \mathcal { N } _ { i } } P _ { g _ { i j } ^ { * } } ( \dot { v } _ { i } - \dot { v } _ { j } ) = \mathrm { s a t } ( \star ) } \end{array}$ , where the quantity in the saturation function is written in short as sa $\natural ( \star )$ . Then, the matrixvector form of the velocity dynamics is $\begin{array} { r } { B _ { f f } \dot { v } _ { f } + B _ { f \ell } \dot { v } _ { \ell } = } \end{array}$ $\operatorname { s a t } ( \star )$ , which implies $\dot { v } _ { f } = \dot { \mathcal { B } } _ { f f } ^ { - 1 } \mathrm { s a t } ( \star ) - \mathcal { B } _ { f f } ^ { - 1 } \dot { \mathcal { B } } _ { f \ell } \dot { v } _ { \ell }$ . It follows that

$$
\| \dot {v} _ {f} \| _ {\infty} \leq \| \mathcal {B} _ {f f} ^ {- 1} \| _ {\infty} \| \mathrm{sat} (\star) \| _ {\infty} + \| \mathcal {B} _ {f f} ^ {- 1} \mathcal {B} _ {f \ell} \| _ {\infty} \| \dot {v} _ {\ell} \| _ {\infty}.
$$

The upper bound for the acceleration as shown above is independent of the initial conditions of the formation position or velocity. It relies on the rigidity structure of the target formation and the magnitude of the accelerations of the leaders. We next characterize the global formation stability under control law (18).

Theorem 8: Under control law (18) and for any timevarying leader velocity $v _ { \ell } ( t )$ , the tracking errors $\delta _ { p } ( t )$ and $\delta _ { v } ( t )$ globally and asymptotically converge to zero.

Proof: Let $\begin{array} { r } { \varepsilon _ { i } \overset { \Delta } { = } \sum _ { j \in \mathcal { N } _ { i } } P _ { g _ { i j } ^ { * } } ( p _ { i } - p _ { j } ) } \end{array}$ . It follows from (18) that $\dot { \varepsilon } _ { i } = \mathrm { s a t } \big ( - k _ { p } \varepsilon _ { i } - k _ { v } \dot { \varepsilon } _ { i } \big )$ . By denoting $\boldsymbol { \varepsilon } = [ \varepsilon _ { 1 } , \dots , \varepsilon _ { n _ { f } } ] ^ { T }$ we obtain

$$
\dot {\varepsilon} = \operatorname{sat} (- k _ {p} \varepsilon - k _ {v} \dot {\varepsilon}).
$$

Note $\varepsilon = B _ { f f } p _ { f } + B _ { f \ell } p _ { \ell } = B _ { f f } \delta _ { p }$ and, hence, $\dot { \varepsilon } = B _ { f f } \delta _ { v } . \mathrm { A s }$ a result, $\varepsilon = \dot { \varepsilon } = 0 \Leftrightarrow \delta _ { p } = \delta _ { v } = 0$ . We prove $\delta _ { p } , \delta _ { v }  0$ by showing $\varepsilon , \dot { \varepsilon } \to 0$ . To that end, consider the Lyapunov function

$$
V = \mathbf {1} ^ {T} \Phi (- k _ {p} \varepsilon - k _ {v} \dot {\varepsilon}) + \mathbf {1} ^ {T} \Phi (- k _ {p} \varepsilon) + k _ {p} \dot {\varepsilon} ^ {T} \dot {\varepsilon}.
$$

The time derivative of V is given by

$$
\begin{array}{r l} & {\dot {V} = (- k _ {p} \dot {\varepsilon} - k _ {v} \ddot {\varepsilon}) \mathbf {s a t} (- k _ {p} \varepsilon - k _ {v} \dot {\varepsilon})} \\ & {\qquad + (- k _ {p} \dot {\varepsilon}) \mathbf {s a t} (- k _ {p} \varepsilon) + 2 k _ {p} \dot {\varepsilon} ^ {T} \ddot {\varepsilon}} \\ & {\qquad = - k _ {v} \ddot {\varepsilon} ^ {T} \ddot {\varepsilon} + k _ {p} \dot {\varepsilon} ^ {T} [ \mathbf {s a t} (k _ {p} \varepsilon) - \mathbf {s a t} (k _ {p} \varepsilon + k _ {v} \dot {\varepsilon}) ].} \end{array}
$$

Similar to the proof of Theorem 7, it can be shown that $\dot { V } \leq 0$ and the invariant set where $\dot { V } = 0 \mathrm { ~ i s ~ } \varepsilon = \dot { \varepsilon } = 0$ . Therefore, by the invariance principle, ε and ε˙ globally and asymptotically converge to zero, and so do $\delta _ { p }$ and $\delta _ { v }$ -

## C. Collision-Free Condition

Collision avoidance among the agents is an important issue in practical formation control problems. The proposed control laws can be implemented together with, for example, artificial potentials [29] to ensure collision avoidance. In this paper, we propose a sufficient condition on the initial formation that ensures no collision between any pair of agents (even if they are not neighbors). Suppose $\gamma$ is the desired minimum distance that should be guaranteed between any two agents and γ satisfies

$$
0 \leq \gamma <   \min _ {i, j \in \mathcal {V}, t \geq 0} \left\| p _ {i} ^ {*} (t) - p _ {j} ^ {*} (t) \right\|.
$$

Theorem 9: Under control law (7), for any constant leader velocity $v _ { \ell } ,$ , it is guaranteed that

$$
\left\| p _ {i} (t) - p _ {j} (t) \right\| > \gamma , \quad \forall i, j \in \mathcal {V}, \quad \forall t \geq 0
$$

if $\delta _ { p } ( 0 )$ and $\delta _ { v } ( 0 )$ satisfy

$$
\begin{array}{l} k _ {p} \delta_ {p} ^ {T} (0) \mathcal {B} _ {f f} \delta_ {p} (0) + \delta_ {v} ^ {T} (0) \delta_ {v} (0) \\ <   \frac {k _ {p} \lambda_ {\min} (\mathcal {B} _ {f f})}{n _ {f}} \left(\min _ {i, j \in \mathcal {V}, t \geq 0} \| p _ {i} ^ {*} (t) - p _ {j} ^ {*} (t) \| - \gamma\right) ^ {2}. \end{array}\tag{19}
$$

Proof: For any $i , j \in \mathcal { V }$ , it always holds that

$$
p _ {i} (t) - p _ {j} (t) \equiv \left[ p _ {i} ^ {*} (t) - p _ {j} ^ {*} (t) \right] + \left[ p _ {i} (t) - p _ {i} ^ {*} (t) \right] - \left[ p _ {j} (t) - p _ {j} ^ {*} (t) \right]
$$

where $p _ { i } ^ { * } ( t )$ and $p _ { j } ^ { * } ( t )$ are the expected positions for agents i and j in the target formation. Note $p _ { i } ( t ) - p _ { i } ^ { * } ( t ) \equiv 0 { \mathrm { f o r } } i \in \mathcal V _ { \ell }$ It follows that

$$
\begin{array}{l} \left\| p _ {i} (t) - p _ {j} (t) \right\| \\ \geq \left\| p _ {i} ^ {*} (t) - p _ {j} ^ {*} (t) \right\| - \left\| p _ {i} (t) - p _ {i} ^ {*} (t) \right\| - \left\| p _ {j} (t) - p _ {j} ^ {*} (t) \right\| \\ \geq \left\| p _ {i} ^ {*} (t) - p _ {j} ^ {*} (t) \right\| - \sum_ {k \in \mathcal {V} _ {f}} \left\| p _ {k} (t) - p _ {k} ^ {*} (t) \right\| \\ \geq \left\| p _ {i} ^ {*} (t) - p _ {j} ^ {*} (t) \right\| - \sqrt {n _ {f}} \left\| p _ {f} (t) - p _ {f} ^ {*} (t) \right\| \\ = \left\| p _ {i} ^ {*} (t) - p _ {j} ^ {*} (t) \right\| - \sqrt {n _ {f}} \| \delta_ {p} (t) \|, \quad \forall t \geq 0. \end{array} \tag {20}\tag{20}
$$

The above inequality gives a lower bound for $\| p _ { i } ( t ) - p _ { j } ( t ) \|$ If we can find a condition such that the lower bound is always greater than $\gamma ,$ then the minimum distance $\gamma$ can be guaranteed. In this direction, consider the Lyapunov function

$$
V \left(\delta_ {p} (t), \delta_ {v} (t)\right) = k _ {p} \delta_ {p} ^ {T} (t) \mathcal {B} _ {f f} \delta_ {p} (t) + \delta_ {v} ^ {T} (t) \delta_ {v} (t).
$$

With the error dynamics as given in (9), the time derivative of $V$ along the error dynamics is $\dot { V } = - 2 k _ { v } \delta _ { v } ^ { T } \mathcal { B } _ { f f } \delta _ { v } \le 0$ . As a result, we have

$$
\begin{array}{r l} & k _ {p} \lambda_ {\min} (\mathcal {B} _ {f f}) \| \delta_ {p} (t) \| ^ {2} \leq k _ {p} \delta_ {p} ^ {T} (t) \mathcal {B} _ {f f} \delta_ {p} (t) \\ & \qquad \leq k _ {p} \delta_ {p} ^ {T} (t) \mathcal {B} _ {f f} \delta_ {p} (t) + \delta_ {v} ^ {T} (t) \delta_ {v} (t) \\ & \qquad \leq V (\delta_ {p} (0), \delta_ {v} (0)) \end{array}
$$

which implies

$$
\| \delta_ {p} (t) \| \leq \sqrt {\frac {V (\delta_ {p} (0) , \delta_ {v} (0))}{k _ {p} \lambda_ {\min} (\mathcal {B} _ {f f})}}.\tag{21}
$$

By combining (20) and (21), we have that $\| p _ { i } ( t ) - p _ { j } ( t ) \| > \gamma$ for all $t \geq 0$ and all $i , j \in \mathcal { V } \mathrm { i f } \delta _ { p } ( 0 )$ and $\delta _ { v } ( 0 )$ satisfies

$$
\min _ {i, j \in \mathcal {V}, t \geq 0} \left\| p _ {i} ^ {*} (t) - p _ {j} ^ {*} (t) \right\| - \sqrt {\frac {n _ {f} V (\delta_ {p} (0) , \delta_ {v} (0))}{k _ {p} \lambda_ {\min} (\mathcal {B} _ {f f})}} > \gamma
$$

which can be rewritten as (19).

The intuition behind the condition in Theorem 9 is that collision avoidance is guaranteed if the initial formation is sufficiently close to the target formation. Theorem 9 is merely applicable in the case of constant $v _ { \ell } ( t )$ . For time-varying $v _ { \ell } ( t )$ we have a similar condition for control law (10). The proof is similar to Theorem 9 and omitted.

Theorem 10: Under control law (10), for any time-varying leader velocity $v _ { \ell } ( t )$ , it can be guaranteed that

$$
\left\| p _ {i} (t) - p _ {j} (t) \right\| > \gamma , \quad \forall i, j \in \mathcal {V}, \quad \forall t \geq 0
$$

if $\delta _ { p } ( 0 )$ and $\delta _ { v } ( 0 )$ satisfy

$$
\begin{array}{l} k _ {p} \delta_ {p} ^ {T} (0) \delta_ {p} (0) + \delta_ {v} ^ {T} (0) \delta_ {v} (0) \\ <   \frac {k _ {p}}{n _ {f}} \left(\min _ {i, j \in \mathcal {V}, t \geq 0} \left\| p _ {i} ^ {*} (t) - p _ {j} ^ {*} (t) \right\| - \gamma\right) ^ {2}. \end{array}
$$

The sufficient conditions given in Theorems 9 and 10 are likely conservative in practice. For example, in the simulation example shown in Fig. 4, no two agents collide during the formation evolution even though the inequality (19) does not hold. Specifically, the left-hand side of (19) is equal to 325.88, whereas the right-hand side with $\gamma = 0$ is equal to 14.53.

## V. CONCLUSION

This work proposed and analyzed a bearing-based approach to the problem of translational and scaling formation maneuver control in arbitrary dimensional spaces. We proposed a variety of bearing-based formation control laws and analyzed their global formation stability. There are several important directions for future research. For example, in this paper, we assume that the information flow between any two followers is bidirectional. In the directional case, a new notion called bearing persistence emerges and plays an important role in the formation stability analysis [30]. Second, although the doubleintegrator dynamics can approximately model some practical physical systems, more complicated models, such as nonholonomic models, should be considered in the future.

## APPENDIX

## A. Preliminaries to Bearing Rigidity Theory

Some basic concepts and results in the bearing rigidity theory are revisited here. Details can be found in [16]. For a formation $\mathcal G ( \boldsymbol p )$ with undirected graph G, assign a direction to each edge in G to obtain an oriented graph. Express the edge vector and the bearing for the kth directed edge in the ed on March 19,2026 at 11:34:34 UTC from IEEE Xplore. Restrictions apply.

oriented graph, respectively, as $e _ { k }$ and $g _ { k } \triangleq e _ { k } / \lVert e _ { k } \rVert$ for $k \in$ $\{ 1 , \ldots , m \}$ where $m = | \mathcal { E } |$ . Define the bearing function $F _ { B } :$ $\mathbb { R } ^ { d n }  \mathbb { R } ^ { d m }$ as $F _ { B } ( \boldsymbol { p } ) \overset { \Delta } { = } \left[ g _ { 1 } ^ { T } , \dots , g _ { m } ^ { T } \right] ^ { T }$ . The bearing rigidity matrix is defined as the Jacobian of the bearing function $R _ { B } ( p ) \triangleq { \partial F _ { B } ( p ) } / { \partial p } \in \mathbb { R } ^ { d m \times d n }$ . The bearing rigidity matrix satisfies rank $( R _ { B } ) \leq d n - d - 1$ and span $\left\{ \mathbf { 1 } \otimes I _ { d } , p \right\} \subseteq$ $\mathrm { N u l l } ( R _ { B } )$ [16]. Let δp be a variation of $p .$ If $R _ { B } ( p ) \delta p = 0$ then $\delta p$ is called an infinitesimal bearing motion of $\mathcal G ( \boldsymbol p )$ . A formation always has two kinds of trivial infinitesimal bearing motions: translation and scaling of the entire formation. A formation is called infinitesimally bearing rigid if all of the infinitesimal bearing motions are trivial. The infinitesimal bearing rigidity has the following important properties.

Theorem $I I ( I I 6 I )$ : The following statements are equivalent:

a) $\mathcal { G } ( \boldsymbol { p } )$ is infinitesimally bearing rigid;

b) $\mathcal { G } ( \boldsymbol { p } )$ can be uniquely determined up to a translational and scaling factor by the inter-neighbor bearings $\{ g _ { i j } \} _ { ( i , j ) \in \mathcal { E } } ;$ c) rank $( R _ { B } ) = d n - d - 1 ;$

d) Nul $. ( R _ { B } ) = \operatorname { s p a n } \{ \mathbf { 1 } _ { n } \otimes I _ { d } , p \}$

## REFERENCES

[1] K.-K. Oh, M.-C. Park, and H.-S. Ahn, “A survey of multi-agent formation control,” Automatica, vol. 53, pp. 424–440, Mar. 2015.

[2] W. Ren, “Multi-vehicle consensus with a time-varying reference state,” Syst. Control Lett., vol. 56, pp. 474–483, 2007.

[3] Z. Sun and B. D. O. Anderson, “Rigid formation control with prescribed orientation,” in Proc. IEEE Multi-Conf. Syst. Control, Sep. 2015, pp. 639–645.

[4] A. N. Bishop, “Stabilization of rigid formations with direction-only constraints,” in Proc. 50th IEEE Conf. Dec. Control Eur. Control Conf., Orlando, FL, USA, Dec. 2011, pp. 746–752.

[5] T. Eren, “Formation shape control based on bearing rigidity,” Int. J. Control, vol. 85, no. 9, pp. 1361–1379, 2012.

[6] S. Zhao and D. Zelazo, “Bearing-based distributed control and estimation in multi-agent systems,” in Proc. Eur. Control Conf., Linz, Austria, Jul. 2015, pp. 2207–2212.

[7] D. Zelazo, A. Franchi, and P. R. Giordano, “Rigidity theory in SE(2) for unscaled relative position estimation using only bearing measurements,” in Proc. Eur. Control Conf., Strasbourgh, France, Jun. 2014, pp. 2703–2708.

[8] N. Moshtagh, N. Michael, A. Jadbabaie, and K. Daniilidis, “Visionbased, distributed control laws for motion coordination of nonholonomic robots,” IEEE Trans. Robot., vol. 25, no. 4, pp. 851–860, Aug. 2009.

[9] R. Tron, J. Thomas, G. Loianno, J. Polin, V. Kumar, and K. Daniilidis, “Vision-based formation control of aerial vehicles,” presented at the Workshop Distrib. Control Estimation for Robotic Vehicle Netw., Berkeley, CA, USA, 2014.

[10] M. Basiri, A. N. Bishop, and P. Jensfelt, “Distributed control of triangular formations with angle-only constraints,” Syst. Control Lett., vol. 59, pp. 147–154, 2010.

[11] A. Franchi, C. Masone, V. Grabe, M. Ryll, H. H. Bulthoff, and P. R. Giordano, “Modeling and control of UAV bearing formations with bilateral high-level steering,” Int. J. Robot. Res., vol. 31, no. 12, pp. 1504–1525, 2012.

[12] A. Cornejo, A. J. Lynch, E. Fudge, S. Bilstein, M. Khabbazian, and J. McLurkin, “Scale-free coordinates for multi-robot systems with bearing-only sensors,” Int. J. Robot. Res., vol. 32, no. 12, pp. 1459–1474, 2013.

[13] R. Zheng and D. Sun, “Rendezvous of unicycles: A bearings-only and perimeter shortening approach,” Syst. Control Lett., vol. 62, no. 5, pp. 401–407, May 2013.

[14] S. Zhao, F. Lin, K. Peng, B. M. Chen, and T. H. Lee, “Distributed control of angle-constrained cyclic formations using bearing-only mea surements,” Syst. Control Lett., vol. 63, no. 1, pp. 12–24, 2014.

[15] E. Schoof, A. Chapman, and M. Mesbahi, “Bearing-compass formation control: A human-swarm interaction perspective,” in Proc. Amer. Control Conf. USA, Jun. 2014, pp. 3881–3886.

[16] S. Zhao and D. Zelazo, “Bearing rigidity and almost global bearing-only formation stabilization,” IEEE Trans. Autom. Control, vol. 61, no. 5, pp. 1255–1268, May 2016.

[17] S. Coogan and M. Arcak, “Scaling the size of a formation using relative position feedback,” Automatica, vol. 48, no. 10, pp. 2677–2685, Oct. 2012.

[18] M.-C. Park, K. Jeong, and H.-S. Ahn, “Formation stabilization and resizing based on the control of inter-agent distances,” Int. J. Robust Nonlinear Control, vol. 25, no. 14, pp. 2532–2546, Sep. 25, 2015.

[19] Z. Lin, L. Wang, Z. Han, and M. Fu, “Distributed formation control of multi-agent systems using complex laplacian,” IEEE Trans. Autom. Control, vol. 59, no. 7, pp. 1765–1777, Jul. 2014.

[20] S. Zhao and D. Zelazo, “Localizability and distributed protocols for bearing-based network localization in arbitrary dimensions,” preprint arXiv:1502.00154.

[21] S. Zhao and D. Zelazo, “Bearing-based formation maneuvering,” in Proc. IEEE Multi-Conf. Syst. Control, Sydney, Australia, Sep. 2015, pp. 658–663.

[22] P. Yang, R. A. Freeman, G. J. Gordon, K. M. Lynch, S. S. Srinivasa, and R. Sukthankar, “Decentralized estimation and control of graph connectivity for mobile sensor networks,” Automatica, vol. 46, no. 2, pp. 390–396, 2010.

[23] W. Ren, “On consensus algorithms for double-integrator dynamics,” IEEE Trans. Autom. Control, vol. 53, no. 6, pp. 1503–1509, Jul. 2008.

[24] W. Yu, G. Chen, and M. Cao, “Some necessary and sufficient conditions for second-order consensus in multi-agent dynamical systems,” Automatica, vol. 46, pp. 1089–1095, 2010.

[25] Z. Meng, Z. Zhao, and Z. Lin, “On global leader-following consensus of identical linear dynamic systems subject to actuator saturation,” Syst. Control Lett., vol. 62, pp. 132–142, 2013.

[26] A. K. Das, R. Fierro, V. Kumar, J. P. Ostrowski, J. Spletzer, and C. J. Taylor, “A vision-based formation control framework,” IEEE Trans. Robot. Autom., vol. 18, no. 5, pp. 813–825, Oct. 2002.

[27] M. Andreasson, D. V. Dimarogonas, H. Sandberg, and K. H. Johansson, “Distributed control of networked dynamical systems: Static feedback, integral action and consensus,” IEEE Trans. Autom. Control, vol. 59, no. 7, pp. 1750–1764, Jul. 2014.

[28] O. Rozenheck, S. Zhao, and D. Zelazo, “A proportional-integral controller for distance-based formation tracking,” in Proc. Eur. Control Conf., Linz, Austria, Jul. 2015, pp. 1687–1692.

[29] Z. Han, Z. Lin, Z. Chen, and M. Fu, “Formation maneuvering with collision avoidance and connectivity maintenance,” in Proc. IEEE Multi-Conf. Syst. Control, Sep. 2015, pp. 652–657.

[30] S. Zhao and D. Zelazo, “Bearing-based formation stabilization with directed interaction topologies,” presented at the 54th IEEE Conf. Dec. Control, Osaka, Japan, pp. 6115–6120, Dec. 2015.

![](images/a558971b96239566d039583773426293b5ebbb31757b4e811697579b7f6d741f.jpg)

Shiyu Zhao received the B.Eng. and M.Eng. degrees in electrical engineering from Beijing University of Aeronautics and Astronautics, Beijing, China, in 2006 and 2009, respectively, and the Ph.D. degree in electrical engineering from the National University of Singapore, Singapore, in 2014.

He is a Postdoctoral Research Associate in the Department of Mechanical Engineering at the University of California, Riverside, CA, USA. From 2014 to 2015, he was a Postdoctoral Research As sociate in the Faculty of Aerospace Engineering at the Technion-Israel Institute of Technology, Haifa, Israel. His research interests lie in distributed control and estimation over networked dynamical systems.

![](images/ae633d278e47ea795e2de98e4f7d293f4238b82cd6fa69eef4f35ecd6696969a.jpg)

Daniel Zelazo (M’11) received the B.Sc. and M.Eng. degrees in electrical engineering from the Massachusetts Institute of Technology, Cambridge, MA, USA, in 1999 and 2001, respectively, and the Ph.D. degree in aeronautics and astronautics from the University of Washington, Seattle, WA, USA, in 2009.

He is an Assistant Professor of aerospace engineering at the Technion-Israel Institute of Technology, Haifa, Israel. From 2010 to 2012, he was a Postdoctoral Research Associate and Lecturer at the

Institute for Systems Theory & Automatic Control, University of Stuttgart, Stuttgart, Germany. His research interests include topics related to multi-agent systems, optimization, and graph theory.