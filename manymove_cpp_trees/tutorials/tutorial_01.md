# Tutorial 01 - Building your first Pick & Place application with ManyMove

## Introduction

As you may have noticed, all of the executable in ManyMove are structured as a sort of tutorial: all sections follow a certain logic, which may vary from executable to executable to highlight different possible trains of thought. They contain lots of useful information, but there's lots that is given for granted.
This tutorial serves as a base to understand how to start using ManyMove with a focus on obtaining a basic series of actions; I won't be talking about the implementation details, but I'll cover the overall high-level logic to go straight to the goal of performing an action with a manipulator.

At the end of the tutorial you'll get an understanding on how to create this kind of application:

<img src="./media/tutorial_01.gif" alt="Tutorial_01" width="640"/>

## Prerequisites

You will need to have installed ROS2 Humble or Jazzy, Moveit2 and xarm_ros2 repositories for the corresponding distro. Follow the instructions in ManyMove's github repo and in xarm_ros2 repo and you should be able to have it all up and running.
You would also be nice for you to have some knowledge about ROS2, MoveIt2 and BehaviorTree.CPP, as they are the founding blocks of ManyMove. If you don't know much about Moveit and BehaviorTrees you may still try and follow this tutorial, as it will be very practical and as abstract as possible.

## 0. Prologue

Ok, so you want to create an application with your manipulator in ROS.

The good news is that you have a huge range of packages that can help you in that!
The bad news is that you'll have to get to know most of these packages quite well before you can do anything practical, and even then you need quite some skills and time to create the logic to orchestrate it all. The packages you'll need to use won't be perfectly documented, and sometimes you'll have to fix some problems or develop new functionalities to get where you need to.
This tutorial is meant to help a little with these problems.

We'll develop a simple pick and place application with a Ufactory Lite6, a little cheap 6 axis cobot that has the exact same software features of its bigger brothers: this means that we can use it to simulate our application, then test the application with a real robot. When we are ready to scale it up to production we won't need many changes to be able to use a more industial model from the same manufacturer.

Since I want to focus on how ManyMove works, in the first tutorial we'll use a launcher that already brings up all the required nodes, including the robot, Rviz, MoveIt, and so on.

The file we'll edit is `manymove_cpp_trees/src/tutorial_01.cpp`.
You will copy the code shown in each section to the corrisponding section of the file. I suggest you to type it in, instead of just copy-paste it: this will help you notice every detail of code used, and to get used to editing it for your future projects!

If you get stuck, you find the whole complete code for the tutorial in `manymove_cpp_trees/src/tutorial_01_complete.cpp`.

For now, you can ignore section `0` and section `6` of the tutorial, as you only rarely need to touch it. We'll cover it in the documentation or in some future tutorial, if there's request for it.

After you install all the required packages and source the workspace, you can use this command to execute the tutorial:

```bash
ros2 launch manymove_bringup tutorial_01.launch.py
```

At the beginning the scene is empty, and pressing the buttons does nothing:

<img src="./media/empty_scene.png" alt="Tutorial_01 empty scene" width="640"/>

---

## 1. Create the scene

One of the advantages of ROS and MoveIt is being able to plan dynamically while considering the objects in the scene. Here we'll create the tutorial scene to leverage these functionalities, so it will contain 3 box objects:
* a box representing the ground or the table the robot is fixed to
* a box representing an obstacle wall
* a box representing an object to grasp

Let's start by creating the ground, using a function that simplifies creating all the tree's elements to work with objects:

```cpp
    ObjectSnippets ground = createObjectSnippets(
        blackboard, keys,
        "ground",                                       /* object name */
        "box",                                          /* shape */
        createPoseRPY(0.0, 0.0, -0.051, 0.0, 0.0, 0.0), /* pose of the object */
        {1.0, 1.0, 0.1},                                /* primitive dimensions */
        "",                                             /* mesh file path */
        {1.0, 1.0, 1.0},                                /* scale */
        "",                                             /* link name to attach/detach */
        {}                                              /* contact links to attach/detach */
    );
```

This function creates a series of xml snippets that can be directly used to compose the BehaviorTree:
* check_xml: checks if the object exists in the scene
* add_xml: adds the object to the scene
* init_xml: initializes the object by adding it only if not already in the scene
* remove_xml: removes the object from the scene
* attach_xml: attaches the object to the specified robot's link
* detach_xml: detaches the object from the specified robot's link

Calling the snippet will also create a blackboard key for each input of the function, and their meaning is quite self-explanatory.
Given **name** the name chosen as the name variable of createObjectSnippets:
* name + "_key"
* name + "_shape_key"
* name + "_pose_key"
* name + "_dimension_key"
* name + "_scale_key"
* name + "_file_key"

Without going to deep into details, just know that using these keys you can change the values relative to the object at runtime.
Some of the keys also allows to be modified from the HMI, but that'll be covered in another tutorial.
Most of the following functions use blackboard keys as input instead of their respective values, to be able to make them more flexible.
For example, if you want to access the name of the ground object you'll have to access the "ground_key" key.

You can leave empty the fields that are not required for a given shape type.
* **Primitives** (box, cylinder, sphere) will require to have dimensions specified, and they need to have the correct number of values
* **Meshes** need a mesh file path specified, and they can be scaled freely with the scale parameter

As you may notice, `createPoseRPY()` is an helper function that takes as input (x, y, z, roll, pitch, yaw) and use that to create a pose containing a quaternion, as required by `geometry_msgs::msg::Pose`.

For the wall object we can skip some of the unused input params:

```cpp
    ObjectSnippets wall = createObjectSnippets(
        blackboard, keys, "wall", "box",
        createPoseRPY(0.0, -0.15, 0.1, 0.0, 0.0, 0.0), {1.0, 0.02, 0.2});
```

The wall will be on the way when we'll try and reach the object, and also when we move to the drop position.
The size and position are made to work with the Lite6, when we'll use another robot we'll have to move and/or resize the wall for it to serve its purpose. When you'll simulate a real scenario, you'll either model the obstacles as the fixed objects on the scene or use a camera to localize them and add them to the scene accordingly. If you don't need to interact with them, you can also use the camera to generate an octomap of the scene.

The last object is the graspable box:

```cpp
    ObjectSnippets graspable = createObjectSnippets(
        blackboard, keys, "graspable", "box",
        createPoseRPY(0.25, -0.25, 0.1, 0.0, 0.0, 0.0), {0.1, 0.005, 0.005},
    "", {1.0, 1.0, 1.0}, "tcp_frame_name_key");
```

It's placed within reach of the Lite6 and it's shape and size already match the gripper orientation relative to the robot's TCP.
But beware: the graspable object's Z axis is facing up, while the robot's TCP's Z axis is facing down. We will want them to match when we'll go grab the object, and we'll see that when defining the variable poses in the next section.

Before moving on, please notice that the `tcp_frame_name_key` field was added as input: without a blackboard key that refers to the robot's tcp, the attach_xml snippet will give you an error at runtime, as it won't know to what link you want to attach the object to. Here `tcp_frame_name_key` is just a string, there's no key associated yet: we'll see in the next section how to create it.

The variable `rp` contains al the data of the robot. We'll talk about it in the future, but for now let's just use a couple of its fields:
* `prefix` is an optional field, here is empty. It stores the prefix for the robot, to differentiate the various topics, services and action servers
* `tcp_frame` contains the name of the link that represents the robot's tcp: it's the link we want to attach the object to. In some scenarios, you may want to use another frame to attach the object, for ease of reference: for example, on Franka Emika Panda robot the tcp is not aligned with the robot's flange, so it may be easier to refer to the flange itself and use a offset when using an object's pose.

### Section 1 output (OPTIONAL):

You might skip this part if you prefer to set up the tree at the end, but I bet you are curious to see the object you created in the scene!
Let's start modifying the code to assemble the tree: find the variable `startup_sequence_xml` and add the elements created within the curly brackets:

```cpp
    std::string startup_sequence_xml = sequenceWrapperXML(
        "StartUpSequence",
        {
            ground.init_xml, wall.init_xml, graspable.init_xml,
        });
```

Build the scene and launch the tutorial again, and you'll see the object appear in the scene:

<img src="./media/tutorial_01_section_1.png" alt="Tutorial_01 section_1" width="640"/>

---

## 2. Define the variable poses

We created the xml snippets for all the objects. If we want, we could define some fixed poses that make the robot correctly pick and place the object, as we usually do in many common industrial scenarios. The graspable object's position is already known as we placed it there, so we could use the same pose to guide the robot, we just need to give it some offset.
One of the main reasons I wrote ManyMove is to be try and develop a bin picking solution, and for that we need a way to know the pose of an object dynamically.

Since it's not that much harder than just using the known pose, I decided to add this to the first tutorial as well, as I think it's very important to create a flexible application. If you really don't need it at all, you may skip to the next session and just focus on fixed poses.

So, we want to get the pose of an object and refer to it when moving the robot: to get the pose of an object we can use the helper function `createGetObjectPose()`, wrapped in a `buildObjectActionXML()` function that will output the complete xml snippet for the logic tree.
Let's see how it works:

```cpp
    std::string get_pick_pose_xml = buildObjectActionXML(
        "get_pick_pose", createGetObjectPose(
                             "graspable_key",
                             "pick_target_key",
                             "world_frame_key",
                             "identity_transform_key",
                             "post_transform_xyz_rpy_1_key"));

    std::string get_approach_pose_xml = buildObjectActionXML(
        "get_approach_pose", createGetObjectPose(
                                 "graspable_key",
                                 "approach_pick_target_key",
                                 "world_frame_key",
                                 "approach_pre_transform_xyz_rpy_1_key",
                                 "post_transform_xyz_rpy_1_key"));
```

The `buildObjectAction()` function just needs as input a name and an object function. Up till now, we masked most of the object functions' usage in the `createObjectSnippets()` function, but here we use one directly.

Let's see what each component does:
* `"graspable_key"`: it's the blackboard key that stores the name of the object. The object name it contains will be used to search for the corresponding object in the scene to get its pose. We need to get the `"graspable"` object's pose, so we use the name key generated with the `createObjectSnippets()` function.
* `"pick_target_key"` and `"approach_pick_target_key"`: the keys containing the `geometry_msgs::msg::Pose` that the robot will use. These are the containers that will store the resulting pose, after the tranformations computated depending of the last two keys of the function. Since we will get the pose dynamically, the pose initially contained by this keys can be empty, but we need to take care not using it before we get the updated value of the object's pose.
* `"world_frame_key"`: this key contains `"world"` as we defined at the beginning of the file. Most of the time we want the pose to refer to the world frame, but we may choose some other frame here.
* `"identity_transform_key"` and `"approach_pre_transform_xyz_rpy_1_key"`: here's the key that references the first tranform to adapt the pose of the object to the one of the TCP.
* `"post_transform_xyz_rpy_1_key"`: the key to the second transform to apply to the pose of the object, we'll see later why this is useful

We should spend some words on the transforms. We created the graspable object with this pose: `createPoseRPY(0.25, -0.25, 0.1, 0.0, 0.0, 0.0)`.

If we just get this pose, the approach_pick target and the pick target will be the same. We need to move up the approach target to correctly approach this object. The pick target can be represented by the identity transform key we created as utility. We can reuse this key every time we need a neutral transform. The approach_pick target must move up by about 5 cm in the world's Z+ direction compared to the original pose of the object, but we wont the gripper to approach the object from above. The TCP frame in the Lite6 robot (as in many other) have the Z+ axis exiting from the flange, so it will be pointing downward when aligned above the object.
Imagine you want to align the object with the TCP, and the object is stuck with the Z+ axis going upwards: then the TCP will have to approach from below, and if we want to get some distance we need to move towards Z-. Thus, we get this transform:

```cpp
    blackboard->set("approach_pre_transform_xyz_rpy_1_key", std::vector<double>{0.0, 0.0, -0.05, 0.0, 0.0, 0.0});
```

As we already said, we want to approach from above, so we flip the pose 180 degrees in the X axis:

```cpp
    blackboard->set("post_transform_xyz_rpy_1_key", std::vector<double>{0.0, 0.0, 0.0, 3.14, 0.0, 0.0});
```

You may wonder: couldn't we use just one tranform?
This is a simple scenario, so yes, we could. But using a single tranform may get confusing quicly.
In more complex scenarios, like those in the other executables, the transforms are a bit more complicated and we benefit from separating them in two transforms.
For example: if I were to rotate in the X axis of 45 degrees, but then I needed to rotate in the original Z axis, I would need a different approach as it wouldn't be as easy as inputting the required values. When I rotate the X axis, the Z axis is not vertical anymore, so I'd need a composite transform among the 3 axes. Splitting the transforms makes it much simpler: the second transform is aligned with the original Z axis.

### Visual examples of frames trasformations

<img src="./media/tutorial_01_section_2.png" alt="Tutorial_01 section_1" width="640"/>

In the image above you can visualize why you need to flip the pose: the frame of the TCP is upside down in respect to the one of the graspable object. Rotating 180 degrees in X axis makes the two frames match.
Remember that, if you want to change the inclination of the gripper, the position you start from is the graspable object's one.

**Excercise**: if you want to grasp the object not vertically but with a 15 degrees inclination from outside the wall, how do you modify the transform?

<details>
<summary>Click to see the solution</summary>

```cpp
    blackboard->set("post_transform_xyz_rpy_1_key", std::vector<double>{0.0, 0.0, 0.0, 3.4, 0.0, 0.0});
```

If you use this line in the final program you can see how the inclination of the pick movements change!

</details>

---

## 3. Define the moves

We have the objects and their poses. Now we need to create the moves.
The `buildMoveXML()` function helps us build an xml snippet that will enable us to execute a move from the behavior tree. One of its inputs is a vector of Move structs, defined in `manymove_cpp_trees`'s `move.hpp`. The reason to make it a vector is to be able to group together moves that are logically tied.
At the moment, the resulting trajectory of the concatenated moves won't be blended together: they just represent a cohese sequence that won't be interrupted by other logic elements (I/O signals, checks, ...), but the robot will still come to an halt between moves.

For example the pick_sequence is a short sequence of moves composed by a "pose" move to get in a position to be ready to approach the object, and the "cartesian" move to get the gripper to the grasp position moving linearly to minimize chances of collisions.
As we'll se later, we can then compose these sequences of moves together to build bigger blocks of logically corralated moves. We could also keep all moves separated, but it'd be harder to obtain an easily understandable tree later, expecially if we need to reuse a series of moves in a certain logic order.

Each move is defined with:
* **Robot prefix**: each move is to be executed with a certain robot, trying to execute moves that are made for another robot will end up in an error. Only one robot can be without prefix, in which case we can skip this field. In this example we keep it parametric, but it will be left empty in the launcher.
* **TCP frame**: this is always required, even for `joint` moves. It's needed to compute the pose on `pose` and `cartesian` moves, but also to calculate the cartesian speed of the robot for safety regulations. Many cobots and some industrial robot have a cartesian speed limit set in the safety functions.
* **Move type**, which can be `"joint"`, `"pose"`, `"cartesian"` or `"named"`: each type of move will require some of the other inputs below, and will be described later in more detail.
* **Move config**: this set represents all the parameters needed for a move; for now we'll use one of the presets you can find in `move.hpp`.
* **Pose key**: the blackboard key containing the pose for `pose` and `cartesian` moves, of type `geometry_msgs::msg::Pose`.
* **Joint target**: for `joint` moves, a vector of double values whose length must match the number of axes of the robot.
* **Named target**: for `named` moves, a string with the exact name of the preset joint target, corresponding to a group_state in the robot's SRDF. For example, the Lite6 only has `"home"` target by default.

Before beginning to build the moves, let's define the TCP frame name for all the moves: we have one robot with one TCP, so the one created at the beginning of the file can be anywhere. But be careful to also use the prefix correctly:

```cpp
    std::string tcp_frame_name = rp.prefix + rp.tcp_frame;
```

### 3.1. Named targets, joint targets and poses

The simplest target is the `named` target: it's set up in the robot's SRDF, and all we have to do is to set it as target in the move.
We set a string variable in case we reuse it, but we can also use it directly in the struct move's constructor:

```cpp
    std::string named_home = "home";
```

We also need to define pose targets for `pose` and `cartesian` movements.
If you remember the previous section, we defined two xml snippets that transfer the object's pose to `pick_target_key` and `approach_pick_target_key`, but right now these keys are not defined anywhere. Since we want to read them at runtime, we can create two blackboard key with empty poses:

```cpp
    blackboard->set("pick_target_key", Pose());
    blackboard->set("approach_pick_target_key", Pose());
```

Before using them, we need to remember to call the `get_pick_pose_xml` and `get_approach_pose_xml` action nodes to populate blackboard keys.

The `drop_pose` to release the object doesn't need to be reference to some object, as we didn't model the holders of the workpieces. In a real applciation we may want to position an holder and then reference the drop pose to its position, but for now we'll set the drop pose directly. We'll place the graspable object to be in the way for the homing movement, just to be able to appreciate better the potentialities offered by the probabilistic path planning algorithms used as default in MoveIt2 (here, OMPL):

```cpp
    Pose drop_target = createPose(0.2, 0.0, 0.2, 1.0, 0.0, 0.0, 0.0);
    blackboard->set("drop_target_key", drop_target);
```

Remember you still have to create the blackboard key, as all `pose` and `cartesian` moves need to read them from blackboard keys, not from Pose variables. This make them potentially dynamic at runtime, while joint targets and named targets are static: they are never dynamic at runtime.

[Note: This may change in the future, but for now I didn't feel the need to use dynamic joint targets. Let me know if you'd need dynamic joint targets!]

We can then define an approach pose to the drop position. Since the pose is fixed, we can copy it with a new name and offset the Z axis to achieve a vertical approach:

```cpp
    Pose approach_drop_target = drop_target;
    approach_drop_target.position.z += 0.02;
    blackboard->set("approach_drop_target_key", approach_drop_target);
```

Last thing we need is a joint target pose: you need to define a vector of doubles of the length of the number of axes of the robot, with all the values within the limits of the robot's joints. There's no compile time check for the number or validity of the joint values, but the move won't be planned if invalid. Here's an example with valid length and values for the Lite6:

```cpp
    std::vector<double> joint_rest = {0.0, -0.785, 0.785, 0.0, 1.57, 0.0};
```

In other examples you'll find joint target vectors for uf850, xarm7 and panda robots.

### 3.2. Creating the move sequences

How do we use the targets we just created?
We need to define sequences of moves that are logically tied together. Let's begin with a minimal sequence containing just a `named` move:

```cpp
    std::vector<Move> home_position = {
        {rp.prefix, tcp_frame_name, "named", move_configs["max_move"], "", {}, named_home},
    };
```

If you used some proprietary robot programming language (eg.: RAPID or KRL), this may feel more familiar than directly sending a move commad to MoveIt2!

We define the `named_home` move using the robot's prefix, the `tcp_frame_name` we defined earlier, the move type `"named"`, one of the default `move_configs`, and the `named_home` string. We also need to input an empty string for the pose key and an empty vector for the joint target before the named target. Even if we put a valid value there, it would just be ignored.

Since we are using the `move_position` sequence to home the robot after opening the gripper, we add an exit move to the sequence.
**Modify the previously defined vector of moves adding one line before the named one**:

```cpp
    std::vector<Move> home_position = {
        {rp.prefix, tcp_frame_name, "cartesian", move_configs["cartesian_mid_move"], "approach_drop_target_key"},
        {rp.prefix, tcp_frame_name, "named", move_configs["max_move"], "", {}, named_home},
    };
```

As you can see the structure of the move is similar, but the type is `cartesian`. We also use a different `move_configs` preset, and a blackboard key to indicate where to find the pose for the move. We don't need to specify the empty joint target and named move, as the default values take care of that.

Let's build the rest of the moves in a logical order. The robot will be in the home position on startup. We want to send it to the `joint_rest` position. Here's how:

```cpp
    std::vector<Move> rest_position = {
        {rp.prefix, tcp_frame_name, "joint", move_configs["max_move"], "", joint_rest},
    };
```

As you can see, we use the type `joint` and one of the preset move configs. We can use an empty pose key and just use the `joint_rest` variable directly, which won't be dynamic.

Next we want to grab the object: we first approach it with a `pose` move, then we move to grasping position with a `cartesian` move.

```cpp
    std::vector<Move> pick_sequence = {
        {rp.prefix, tcp_frame_name, "pose", move_configs["mid_move"], "approach_pick_target_key"},
        {rp.prefix, tcp_frame_name, "cartesian", move_configs["cartesian_slow_move"], "pick_target_key"},
    };
```

Note that they both use a pose key, but the type and move configs preset change.

After we grab the object, we need a sequence to drop it in the right place: we exit vertically with a `cartesian` move, get to the approach pose with a `pose` move and finally we go in the drop position with a `pose` move.

```cpp
    std::vector<Move> drop_sequence = {
        {rp.prefix, tcp_frame_name, "cartesian", move_configs["cartesian_mid_move"], "approach_pick_target_key"},
        {rp.prefix, tcp_frame_name, "pose", move_configs["max_move"], "approach_drop_target_key"},
        {rp.prefix, tcp_frame_name, "cartesian", move_configs["cartesian_slow_move"], "drop_target_key"},
    };
```

### 3.3. Building the moves

Now we have all the move sequences, but how can we use them in our behavior tree?
Turns out that this is very easy thanks to the `buildMoveXML()` function:

```cpp
    std::string to_rest_xml = buildMoveXML(
        rp.prefix, rp.prefix + "toRest", rest_position, blackboard);

    std::string pick_object_xml = buildMoveXML(
        rp.prefix, rp.prefix + "pick", pick_sequence, blackboard);

    std::string drop_object_xml = buildMoveXML(
        rp.prefix, rp.prefix + "drop", drop_sequence, blackboard);

    std::string to_home_xml = buildMoveXML(
        rp.prefix, rp.prefix + "home", home_position, blackboard);
```

As you can see, you just specify the robot prefix for the sequence, its name (I use the robot prefix here too for clarity), the sequence and the blackboard: the `buildMoveXML()` function will automatically build the xml snippets to use on the behavior tree, and also initialize all the necessary blackboard keys.

## 4. Build higher level snippets

Time to use the building blocks we created to assemble some more meaningful behavior tree constructs!
You may have noticed that some of the variables I defined end with `_xml`: these are the ones that contain a usable xml snippet that represents a part of the tree.
To ease the process of combining the snippets we created until now, we can leverage some builder functions; for example:
* `sequenceWrapperXML()`: forms a sequentially executed group of nodes, consisting of one or more child nodes; it succeeds only if all of its nodes succeed
* `fallbackWrapperXML()`: forms a sequential group of nodes, and executes them sequentially, but as soon as one node succeeds the whole fallback node succeeds
* `repeatSequenceWrapperXML`: creates a sequence that repeats a specified number of times, as long as its child sequence succeeds; if it fails even once, the whole reapeat node will fail. If the number of repeats specified is `-1` it will run endlessly or until failure.
* `retrySequenceWrapperXML`: creates a sequence that retries a specified number of times if its child sequence fails; if its child sequence succeeds even once, the whole retry node will succeed. If the number of retries is `-1` it will run endlessly or until success.

We'll only use 3 of those builder functions in this tutorial, but there are more in the `tree_helper` source code. Many are used in the various executable examples in the `manymove_cpp_trees` repo, if you need some input on how to leverage them.

With just 3 builder functions we'll be able now to create a pretty powerful behavior tree. Let's start with the `sequenceWrapperXML()`.

When we create the scene, the fixed object will need to get created just once. We said towards the beginning that the init_xml snippet of each object creates the object in the scene only if it's not there already. Let's see what this snippet looks like in XML:

```xml
<Fallback name="init_wall_obj">
    <CheckObjectExistsAction name="check_wall_CheckObjectExistsAction" object_id="{wall_key}" />
    <AddCollisionObjectAction name="add_wall_AddCollisionObjectAction"
        object_id="{wall_key}" shape="{wall_shape_key}"
        dimensions="{wall_dimension_key}" mesh_file="{}"
        scale_mesh="{wall_scale_key}" pose="{wall_pose_key}" />
</Fallback>
```
If you use Groot to visualize the tree, this part of the tree will look like this:

<img src="./media/init_ground_obj.png" alt="Init Ground Object" width="640"/>

---

<details>
<summary>Need instructions on how to start Groot?</summary>

Once you install Groot [the offical Groot's Github repo](https://github.com/BehaviorTree/Groot), open a new terminal and source it.
To start it,     use this line:

```bash
ros2 run groot Groot
```

Then select `Monitor` icon and then `START`.

The `Connect` button will work only if the tree already started, so try this either after section 1 optional instructions, or at the end of the tutorial.

</details>

---

The snippet automatically generated already contains a Fallback node with 2 child nodes.
Since we need to create all the fixed objects at the beginning of the program, let's combine all of their snippets:

```cpp
    std::string spawn_fixed_objects_xml = sequenceWrapperXML("SpawnFixedObjects", {ground.init_xml, wall.init_xml});
```

As you can see, to use the `sequenceWrapperXML` you just need a name for the node and a sequence of xml snippets that you need to execute sequentially. The result itself is another xml snippet that you can combine further.

The resulting xml snippet will look like this:

```xml
<Sequence name="SpawnFixedObjects">

    <Fallback name="init_ground_obj">
        <CheckObjectExistsAction name="check_ground_CheckObjectExistsAction" object_id="{ground_key}" />
        <AddCollisionObjectAction name="add_ground_AddCollisionObjectAction"
        object_id="{ground_key}" shape="{ground_shape_key}"
        dimensions="{ground_dimension_key}" mesh_file="{}"
        scale_mesh="{ground_scale_key}" pose="{ground_pose_key}" />
    </Fallback>

    <Fallback name="init_wall_obj">
        <CheckObjectExistsAction name="check_wall_CheckObjectExistsAction" object_id="{wall_key}" />
        <AddCollisionObjectAction name="add_wall_AddCollisionObjectAction" object_id="{wall_key}"
        shape="{wall_shape_key}" dimensions="{wall_dimension_key}" mesh_file="{}"
        scale_mesh="{wall_scale_key}" pose="{wall_pose_key}" />
    </Fallback>

</Sequence>
```

And here's the an image on how it looks on Groot:

<img src="./media/SpawnFixedObjects.png" alt="SpawnFixedObjects" width="640"/>

---

Since we only have one graspable object, we could use its init_xml directly, but remember that we must update the poses of the graspable object before executing the moves to pick it up!
To avoid mistakes, we create the sequence to init the graspable object and read both poses:

```cpp
    std::string spawn_graspable_objects_xml = sequenceWrapperXML(
        "SpawnGraspableObjects",
        {
            graspable.init_xml,
            get_pick_pose_xml,
            get_approach_pose_xml
        });
```

---

At the end of the cycle we'll need to remove the graspable object, and we have no other actions related to it: we'll just use the remove_xml snippet directly.

We can also create some sequences to enhance the comprehension of what an action represents: right now we are just simulating the gripper opening and closing by directly attaching and detaching the object. In a real scenario we would probably execute the attach/detach nodes only if the object is actually attached/detached, maybe reading some sensor.
We want to keep the original name of the snippet that is automatically generated to use in the real application where we'll need it, but here we can define a couple of wrappers to give more meaning to the snippets:

```cpp
    std::string close_gripper_xml = sequenceWrapperXML("CloseGripper", {graspable.attach_xml});
    std::string open_gripper_xml = sequenceWrapperXML("OpenGripper", {graspable.detach_xml});
```

Using these snippets is much clearer, even though it actually adds a `Sequence` note that is not strictly needed. On the real application we might want to reduce the redundant nodes as much as possible, for clarity and efficiency. Here, being a very simple application, we can accept a bit more of complexity to have a better understanding of the snippets.

We'll use these new snippets right away: when we move to the pick position we'll want to close the gripper, and when we move to the drop position we'll want to open the gripper.

```cpp

    std::string pick_sequence_xml = sequenceWrapperXML("PickSequence", {pick_object_xml, close_gripper_xml});
    std::string drop_sequence_xml = sequenceWrapperXML("DropSequence", {drop_object_xml, open_gripper_xml});
```

---

ManyMove lets you reset the cycle when needed: to put it simply, using the Reset button will trigger a blackboard key the will force the failure of the current branch. If this happens, and the cycle will restart, we want the graspable object to be gone. We'll open the gripper and remove the object. Let's create a snippet to reset the graspable object in the scene:

```cpp
    std::string reset_graspable_objects_xml = sequenceWrapperXML("reset_graspable_objects", {open_gripper_xml, graspable.remove_xml});
```

---

We can also further combine the move sequence blocks in logic sequences. In this tutorial, after the first homing move, the `home_position` move will always be followed by `rest_position` move, so we can combine them sequentially:

```cpp
    std::string home_sequence_xml = sequenceWrapperXML(
        rp.prefix + "ComposedHomeSequence", {to_home_xml, to_rest_xml});
```

The move snippets are quite complex, and I'll leave the details to the documentation and to future tutorials.

## 5. Assembling the tree

Great, now we have all the required components to build our tree!

Before continuing, remove the line that defines `retry_forever_wrapper_xml`, as it will be replaced by new code in this section.

In this section we'll create the 3 components of the `GlobalMasterSequence`:
* Startup Sequence
* Robot Cycle
* Reset Handler

The **Startup Sequence** combines together all the required snippets to spawn the fixed objects, reset the graspable object and move the robot to rest position. If you already modified this snippet in section 1, delete the line you added and replace it with these:

```cpp
    std::string startup_sequence_xml = sequenceWrapperXML(
        "StartUpSequence",
        {
            spawn_fixed_objects_xml,
            reset_graspable_objects_xml,
            to_rest_xml,
         });
```

If we never reset the cycle, this will run just once at the beginning.

---

The **Robot Cycle** is the sequence that runs perpetually (or until something goes wrong!), so this time we use the `repeatSequenceWrapperXML()`. This is almost identical to the `sequenceWrapperXML()`, but you can set it to repeat a number of times when successful. We want it to continue forever, so we set the repetitions to `-1`.
The comments beside the sequence elements are quite self explanatory: we reap the benefits of building the snippets with a clear logic in mind.

```cpp
    std::string repeat_forever_wrapper_xml = repeatSequenceWrapperXML(
        "RobotCycle",
        {spawn_graspable_objects_xml, //< Add the graspable object to the scene and update the relative poses
         pick_sequence_xml,           //< Pick sequence: move to position and close gripper
         drop_sequence_xml,           //< Drop sequence: move to position and open gripper
         home_sequence_xml,           //< Homing sequence: go home positionand then to rest pose
         graspable.remove_xml},       //< Delete the object for it to be added on the next cycle in the original position
        -1);                          //< num_cycles=-1 for infinite
```

---

If we only put these two nodes in sequence, when we reset the robot cycle the tree would just fail and the execution would end.
We need to handle the reset more gracefully. To do this, we wrap both of the startup and the cycle nodes in a Retry node. Here we use the `retrySequenceWrapperXML()` function, which lets us determine how many time we want to retry. We want to be able to reset as many times as we need, so we set the retries to `-1`.
When the cycle resets, `StartUpSequence` will be executed again, resetting the scene before the `RobotCycle` starts.
Update the `retry_forever_wrapper_xml`'s code to include the `repeat_forever_wrapper_xml`

```cpp
    std::string retry_forever_wrapper_xml = retrySequenceWrapperXML("ResetHandler", {startup_sequence_xml, repeat_forever_wrapper_xml}, -1);
```

---

Finally, you can see in the tutorial's code that we create ths `GlobalMasterSequence` using the `retry_forever_wrapper_xml` as only child node, and then use the `GlobalMasterSequence` to instantiate the `MasterTree`.

---

The tree is now complete!
You can **build manymove_cpp_trees** again and run the launch command to see the complete scene. Pressing `START`, `STOP` and `RESET` buttons now have the intended behavior.

```bash
ros2 launch manymove_bringup tutorial_01.launch.py
```

As you see, after preparing the snippets correctly you can handle the logic of the tree with a very limited number of lines of code. The resulting tree looks like this:

![CompleteTree](./media/complete_tree_tutorial_01.png)
