# FoundationPose Refactor Plan

## Goal

Refactor ManyMove's current FoundationPose integration so that:

- FoundationPose inference is triggered on demand through ROS 2 action servers.
- The current `FoundationPoseAlignmentNode` behavior is split into distinct responsibilities.
- Existing behavior is preserved for current users and trees.
- The existing Isaac ROS 3 pipeline in [`src/isaac_ros_custom_bringup/isaac_ros_3/README.md`](/home/tndlux/workspaces/isaac_ros-dev/src/isaac_ros_custom_bringup/isaac_ros_3/README.md) continues to work unchanged.
- A new pipeline is added under `isaac_ros_4` later, without disrupting the ROS 3 path.
- Isaac Manipulator's grasping setup is evaluated separately as an alternative to the alignment part only, not to pose fetching.

## Packaging Constraint

For now, ManyMove will keep following its existing optional-dependency pattern:

- do not add hard `package.xml` dependencies on Isaac ROS / Isaac Manipulator packages in the core ManyMove packages
- allow the Isaac-specific FoundationPose path to exist only when built inside the Isaac ROS container with the needed packages already installed
- keep the legacy topic-based path always available in the base ManyMove build

This is not the ideal package split, but it matches the current repository strategy for other optional integrations such as xArm.

## Current Implementation Status

Implemented in `manymove_cpp_trees`:

- `FoundationPoseFetchTopicNode`: legacy topic fetch + defensive unwrapping of `Detection3DArray`
- `FoundationPoseAdaptPoseNode`: extracted pose adaptation / modification logic
- `FoundationPoseAlignmentNode`: still available as a compatibility wrapper, now reusing the same adaptation path
- `buildFoundationPoseSequenceXML(...)`: kept with the same public signature, but now emits the split fetch/adapt sequence instead of a single monolithic node

Still pending:

- Isaac-style on-demand `FoundationPoseFetchActionNode`
- ROS 4 launch integration for YOLOv8 + action-gated FoundationPose
- runtime validation inside the Isaac ROS container

## Current State

### ManyMove today

The current ManyMove FoundationPose path is topic-driven:

- [`bt_client_foundationpose.cpp`](/home/tndlux/workspaces/isaac_ros-dev/src/manymove/manymove_cpp_trees/src/bt_client_foundationpose.cpp#L223) builds a FoundationPose sequence against the live topic `"/output"`.
- [`tree_helper.cpp`](/home/tndlux/workspaces/isaac_ros-dev/src/manymove/manymove_cpp_trees/src/tree_helper.cpp#L311) expands that into a single `FoundationPoseAlignmentNode`.
- [`action_nodes_isaac.cpp`](/home/tndlux/workspaces/isaac_ros-dev/src/manymove/manymove_cpp_trees/src/action_nodes_isaac.cpp#L405) subscribes to `Detection3DArray`, waits for a fresh message, runs `pickDetection()` over the returned detections and hypotheses, then transforms the chosen pose, normalizes orientation, applies Z constraints, applies local pick and approach transforms, and writes the results to the blackboard.

This means pose acquisition and pose post-processing are currently coupled in one BT node.

Important nuance:

- The current topic-driven path does have explicit post-FoundationPose selection logic in ManyMove via `pickDetection()`.
- However, in the current YOLOv8 ROS 3 pipeline, the 2D detections are already reduced upstream before FoundationPose by `Detection2DArrayFilter`, which selects the highest-confidence detection or a desired class in [`yolov8_foundationpose_realsense_remote.launch.py`](/home/tndlux/workspaces/isaac_ros-dev/src/isaac_ros_custom_bringup/isaac_ros_3/launch/yolov8_foundationpose_realsense_remote.launch.py#L188).
- The README for that pipeline also documents that the mask path takes the highest-score detection in [`README.md`](/home/tndlux/workspaces/isaac_ros-dev/src/isaac_ros_custom_bringup/isaac_ros_3/README.md#L263).
- The Nitros/ROS conversion for `Detection3DArray` supports multiple detections and multiple hypotheses per detection in [`nitros_detection3_d_array.cpp`](/home/tndlux/workspaces/isaac_ros-dev/src/isaac_ros_nitros/isaac_ros_nitros_type/isaac_ros_nitros_detection3_d_array_type/src/nitros_detection3_d_array.cpp#L86), but hypotheses within one detection all reuse the same pose. Distinct poses correspond to distinct `Detection3D` entries, not to multiple `results` entries.
- `FoundationPoseNode` itself also uses only the first pose for TF broadcasting in [`foundationpose_node.cpp`](/home/tndlux/workspaces/isaac_ros-dev/src/isaac_ros_pose_estimation/isaac_ros_foundationpose/src/foundationpose_node.cpp#L584), which is another indication that the intended operational path is effectively single-object.
- Therefore ManyMove's post-FP selection is real code today, but in the current pipeline it is best understood as defensive unwrapping and filtering of a generic `Detection3DArray` message, not as the primary object-selection step. In the current single-ROI or single-mask pipeline it is likely redundant in normal operation.

### Isaac Manipulator today

Isaac Manipulator uses an action-gated bridge in front of FoundationPose:

- [`foundation_pose_server.cpp`](/home/tndlux/workspaces/isaac_ros-dev/src/isaac_manipulator/isaac_manipulator_servers/src/foundation_pose_server.cpp#L146) waits for an `EstimatePoseFoundationPose` goal, republishes one synchronized set of image/depth/camera info plus ROI or segmentation, then waits for one FoundationPose result.
- [`object_info_server.cpp`](/home/tndlux/workspaces/isaac_ros-dev/src/isaac_manipulator/isaac_manipulator_servers/src/object_info_server.cpp#L300) triggers that action from `GetObjectPose`.
- [`foundationpose.launch.py`](/home/tndlux/workspaces/isaac_ros-dev/src/isaac_manipulator/isaac_manipulator_bringup/launch/include/foundationpose.launch.py#L146) still launches `foundationpose_node` continuously.

Important clarification:

- Isaac Manipulator does not spawn the FoundationPose node only when needed.
- It keeps the node loaded, but only feeds it inputs when requested.
- Isaac Manipulator selects the target object before FoundationPose, then sends only that object's ROI or segmentation mask into the pose-estimation action in [`object_info_server.cpp`](/home/tndlux/workspaces/isaac_ros-dev/src/isaac_manipulator/isaac_manipulator_servers/src/object_info_server.cpp#L336).
- In practice, the returned `Detection3DArray` is then treated as a single-object result, for example by taking `poses.detections[0]` in [`object_info_server.cpp`](/home/tndlux/workspaces/isaac_ros-dev/src/isaac_manipulator/isaac_manipulator_servers/src/object_info_server.cpp#L363).

That is the behavior ManyMove should mirror.

## Refactor Strategy

### Target split

Split the current `FoundationPoseAlignmentNode` into at least these responsibilities:

1. Pose fetch:
   - Request FoundationPose on demand through an action client.
   - Receive the raw FoundationPose result.
   - Store the raw result on the blackboard.

2. Legacy result unwrapping:
   - For legacy topic-based compatibility, preserve the existing `pickDetection()` behavior inside the compatibility wrapper when consuming a raw `Detection3DArray`.
   - For the new Isaac-style action-gated path, assume the target was selected upstream and do not introduce a separate selection node.

3. Pose adaptation:
   - Transform pose into the desired frame.
   - Normalize orientation if requested.
   - Apply Z threshold.
   - Apply pick and approach transforms.
   - Preserve writes to `pick_pose_key`, `approach_pose_key`, `object_pose_key`, and `header_key`.

4. Optional validation:
   - Keep bounds checking as a separate node or wrapper step so it remains reusable and explicit.

This preserves behavior while making acquisition independent from pose modification.

## Proposed Node Breakdown

### 1. `FoundationPoseFetchActionNode`

Purpose:

- Trigger FoundationPose on demand through a ROS 2 action client.

Inputs:

- action server name
- ROI input or segmentation mask input
- optional mesh path
- optional object frame name
- timeout

Outputs:

- raw `vision_msgs::msg::Detection3DArray`
- raw result header or metadata
- success or failure

Notes:

- This node should not do any alignment or adaptation.
- It should be usable by future pipelines outside the current pick flow.
- It should support the same action shape as Isaac Manipulator's `EstimatePoseFoundationPose`.

### 2. `FoundationPoseAdaptPoseNode`

Purpose:

- Perform the current pose modification logic with behavior preserved.

Inputs:

- raw pose
- source header and frame
- planning frame
- transform timeout
- `normalize_pose`
- `force_z_vertical`
- `z_threshold_activation`
- `z_threshold`
- `pick_transform`
- `approach_transform`
- output blackboard keys

Outputs:

- final pick pose
- approach pose
- object pose
- header

Notes:

- This should be a direct extraction of the current post-processing block from [`action_nodes_isaac.cpp`](/home/tndlux/workspaces/isaac_ros-dev/src/manymove/manymove_cpp_trees/src/action_nodes_isaac.cpp#L564).
- The first implementation goal is behavior preservation, not redesign.

### 3. Compatibility wrapper: `FoundationPoseAlignmentNode`

Purpose:

- Preserve current tree compatibility.

Behavior:

- Keep the existing XML interface and public ports as much as possible.
- Internally delegate to the new fetch and adapt logic, and optionally to selection logic when consuming legacy `Detection3DArray` outputs.

Why keep it:

- Existing ManyMove trees and examples should keep working.
- This reduces migration risk.
- New trees can adopt the split nodes directly, while legacy trees can remain unchanged.

Recommendation:

- Phase 1 keeps `FoundationPoseAlignmentNode` available and backward compatible.
- Phase 2 introduces new XML builders for split-node usage.
- Phase 3 gradually migrates reference trees to the new nodes.

## Action Server Architecture

### Requirement

FoundationPose must be called on demand through action servers.

### Planned architecture

ManyMove should consume a server analogous to Isaac Manipulator's `foundation_pose_server`:

- camera streams remain live
- the bridge caches latest image, camera info, and depth
- a goal arrives with ROI or segmentation mask
- the bridge republishes one inference request into FoundationPose
- the bridge waits for one result and returns it

### Where this should live

Do not modify the current Isaac ROS 3 YOLOv8 pipeline used today.

Instead:

- Keep the existing ROS 3 path unchanged.
- Build the new action-gated path under `src/isaac_ros_custom_bringup/isaac_ros_4`.

This new ROS 4 path should:

- preserve YOLOv8 as object detector
- preserve the current detection source assumptions
- add a FoundationPose action bridge instead of relying on a continuously consumed output topic

### Practical implication for ManyMove

ManyMove should not depend directly on a live `pose_estimation/output` topic in the new path.

Instead it should depend on:

- a detection source, still from YOLOv8 pipeline
- a FoundationPose action server endpoint

This is compatible with preserving the detection stack while changing only the pose-estimation trigger model.

## Backward Compatibility Plan

### Behavior compatibility

The following behavior must remain unchanged for current trees:

- legacy result filtering by best score
- legacy optional filtering by class id
- transformation to the planning frame
- orientation normalization
- optional vertical Z enforcement
- Z floor thresholding
- local pick transform application
- local approach transform application
- blackboard outputs and key names
- bounds validation behavior

### API compatibility

Preserve the current `buildFoundationPoseSequenceXML(...)` path initially.

Planned approach:

1. Keep `buildFoundationPoseSequenceXML(...)` available.
2. Re-implement it using the new split architecture, either:
   - by composing multiple nodes in generated XML, or
   - by preserving a wrapper node with the old interface.
3. Add new builder helpers for the split architecture, for example:
   - `buildFoundationPoseFetchXML(...)`
   - `buildFoundationPoseAdaptXML(...)`
   - `buildFoundationPoseOnDemandSequenceXML(...)`

Recommendation:

- Prefer a wrapper first, then expose the split builders.
- This minimizes breakage while still enabling cleaner new trees.

## Suggested Phases

### Phase 0: design freeze

- Define the blackboard contract for fetched raw detections and selected raw pose.
- Define the ROS action client interface for fetch.
- Decide whether bounds checking stays inside the sequence helper or becomes its own explicit node.

Deliverable:

- agreed node ports and blackboard keys

### Phase 1: extract adaptation logic

- Extract the current post-processing logic from `FoundationPoseAlignmentNode` into reusable helpers.
- Implement `FoundationPoseAdaptPoseNode`.
- Keep behavior byte-for-byte equivalent where practical.

Deliverable:

- adaptation separated from acquisition

### Phase 2: implement on-demand fetch node

- Implement `FoundationPoseFetchActionNode`.
- Use a ROS 2 action client to call the FoundationPose bridge.
- Store raw `Detection3DArray` on the blackboard.

Deliverable:

- on-demand fetching without topic subscription

### Phase 3: compatibility layer

- Rework `FoundationPoseAlignmentNode` into a compatibility wrapper or composite implementation using the new pieces.
- Keep existing BT clients functioning unchanged.
- Preserve the legacy `pickDetection()` behavior internally only where needed for backward compatibility with raw `Detection3DArray` inputs.

Deliverable:

- legacy behavior preserved

### Phase 4: new XML helpers and example tree

- Add builder helpers for split-node usage.
- Add at least one new reference ManyMove tree demonstrating the on-demand action path.

Deliverable:

- migration example for future trees

### Phase 5: new Isaac ROS 4 pipeline

- Implement the new action-gated pipeline in `isaac_ros_4`.
- Keep YOLOv8 as the detection frontend.
- Keep the Isaac ROS 3 pipeline unchanged.

Deliverable:

- parallel new pipeline without regression risk to current users

## Separate Evaluation: Isaac Manipulator Grasping Setup

This is an evaluation of whether Isaac Manipulator's grasping logic can replace ManyMove's current alignment logic. This is separate from the on-demand pose fetch refactor.

### What Isaac Manipulator does

Isaac Manipulator has a grasp pipeline built around grasp libraries and planner integration:

- grasp poses are read from YAML grasp sets via `GraspReaderManager`
- grasp candidates are generated from the object pose
- approach and retract offsets are planner-aware parameters
- the planner selects a grasp candidate that is actually reachable

Relevant code paths include:

- [`read_grasp_poses.py`](/home/tndlux/workspaces/isaac_ros-dev/src/isaac_manipulator/isaac_manipulator_orchestration/isaac_manipulator_orchestration/behaviors/motion_behaviors/read_grasp_poses.py#L114)
- [`plan_to_grasp.py`](/home/tndlux/workspaces/isaac_ros-dev/src/isaac_manipulator/isaac_manipulator_orchestration/isaac_manipulator_orchestration/behaviors/motion_behaviors/plan_to_grasp.py#L179)
- grasp config files such as [`grasps.yaml`](/home/tndlux/workspaces/isaac_ros-dev/src/isaac_manipulator/isaac_manipulator_robot_description/config/grasps.yaml)

### Could it replace ManyMove's current alignment logic?

Partially, but not directly.

What it could replace:

- fixed heuristic pick and approach transforms as the main grasp-definition mechanism
- some of the custom orientation normalization assumptions, if the grasp library already encodes the desired end-effector relation to the object

What it does not replace:

- on-demand FoundationPose fetching
- raw pose acquisition
- any upstream object-selection logic needed before FoundationPose

### Pros

- Better long-term grasp quality for known objects
- Object-specific grasp candidates instead of one generic offset heuristic
- Natural integration of pregrasp, grasp, and retract semantics
- Planner can choose among several grasps instead of committing to one adapted pose

### Cons

- Higher migration cost than the split-node refactor
- Requires grasp datasets or YAML grasp definitions per object class
- Less suitable if ManyMove needs a generic object-pose-to-pick-pose path for arbitrary objects
- More invasive change to current behavior than simply splitting fetch and adapt

### Recommendation

Treat Isaac Manipulator grasp setup as a future alternative to `FoundationPoseAdaptPoseNode`, not as part of the first refactor.

Recommended strategy:

1. First complete the on-demand fetch refactor while preserving current adaptation behavior.
2. Later add an alternative branch that consumes the fetched object pose and generates grasp candidates using Isaac-style grasp definitions.
3. Make the grasp strategy configurable per object or per tree:
   - heuristic alignment path
   - grasp-library path

This avoids coupling two large changes in one migration.

## Pipeline Constraints from Current README

The following constraints from [`src/isaac_ros_custom_bringup/isaac_ros_3/README.md`](/home/tndlux/workspaces/isaac_ros-dev/src/isaac_ros_custom_bringup/isaac_ros_3/README.md) must remain true:

- YOLOv8 remains the object detector.
- The existing ROS 3 pipeline remains unchanged.
- Existing launch instructions and examples continue to work.
- The new work should be added as a separate path under `isaac_ros_4`.

This means the refactor should not require changing:

- the current ROS 3 launch flow
- the current detector choice
- the current end-user instructions for the existing pipeline

## Risks

### Risk 1: behavior drift during extraction

The current alignment node bundles several subtle behaviors together. Extracting them risks changing grasp pose outputs.

Mitigation:

- Move logic with minimal rewriting first.
- Add focused unit tests for legacy selection and adaptation logic before broader tree migration.

### Risk 2: action server contract mismatch

ManyMove may expect data fields or timing behavior different from the eventual `isaac_ros_4` action server.

Mitigation:

- Freeze the fetch-node blackboard contract early.
- Match Isaac Manipulator's `EstimatePoseFoundationPose` action where practical.

### Risk 3: hidden dependence on live topic timing

Some current tree behavior may implicitly rely on continuous topic updates.

Mitigation:

- Keep a compatibility wrapper node.
- Preserve current timeout semantics in the wrapper.

### Risk 4: mixing grasp-strategy migration with fetch refactor

Switching to grasp libraries at the same time as changing the fetch model would make failures hard to isolate.

Mitigation:

- Keep grasp-strategy evaluation separate.
- Do not change grasp strategy in the first implementation phase.

## Validation Plan

ROS 2 and Isaac ROS are not currently installed in this system, so no runtime validation can be done now.

For now, only design and code refactor planning should proceed.

Once the container is available, validation should include:

1. Unit tests for compatibility-wrapper behavior on legacy `Detection3DArray` inputs.
2. Unit tests for pose adaptation equivalence.
3. Integration test with a mocked FoundationPose action server.
4. End-to-end test in the container with the new `isaac_ros_4` action-gated pipeline.
5. Regression test showing the existing ROS 3 pipeline still works unchanged.
6. Comparison test between:
   - legacy `FoundationPoseAlignmentNode`
   - compatibility wrapper implementation
   - split-node implementation

## Recommended First Implementation Order

1. Extract adaptation logic into a helper and `FoundationPoseAdaptPoseNode`.
2. Keep `FoundationPoseAlignmentNode` working by calling the extracted helper code.
3. Introduce `FoundationPoseFetchActionNode`.
4. Add a new on-demand XML sequence builder.
5. Implement the new `isaac_ros_4` action-gated pipeline.
6. Evaluate the grasp-library alternative after the fetch refactor is stable.

## Non-Goals for the First Refactor

- Replacing YOLOv8
- Changing the current ROS 3 README workflow
- Replacing heuristic alignment with grasp libraries immediately
- Requiring users to rewrite current behavior trees before the new path is ready
