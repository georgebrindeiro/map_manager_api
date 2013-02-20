:Info: A map manager API for 3D mapping, navigation and scene interpretation
:Author: Stéphane Magnenat <stephane at magnenat dot net>
:Date: 2013-02-11

=======================================================================
 A map manager API for 3D mapping, navigation and scene interpretation
=======================================================================

Rationale
=========

A modern autonomous robotic system is composed of many building blocks (SLAM, path planning, scene interpretation), most of them depending on some form of localisation and mapping.
Therefore, there is a need for an API that allows these different elements to communicate, with as little interdependency as possible.
This document describes such an API, based on the concept of a transactional map database.
It aims at being as generic as possible and agnostic with respect to the implementation framework.

Framework requirements
======================

The implementation framework shall allow modularity based on the concept of components.
It shall provide both message passing (called *message* in the rest of this document) and remote procedure invocation (called *RPC* in the rest of this document) for interfacing modules together.

Writing conventions
===================

When variables are described, the convention ``variable: Type`` is used, with ``variable`` having its first letter in lower case and ``Type`` in upper case, and using *CamelCase* for multiple words.

When RPC or messages are described, the convention ``name(arguments) -> ReturnType`` is used, in CameCase as well.

Concepts
========

Frame
  A frame is a coordinate frame located in 3-D space, linked with other frames through a set of probabilistic transformations.
  Frames have unique identifiers, and can optionally have human-readable names.
Link
  A probabilistic transformation between two frames.
  These are encoding as **SM: What is this encoding for probabilistic transformations?**
  **PTF: In my code, there is no explicit encoding. Frames are vertices in the graph. Data can be stored there but that does not have anything to do with the graph itself. Transformations are stored on the edges so there is no need to put geometric data in the graph at the vertices.**
EstimatedFrame
  A graph of frames can be relaxed to have non-probabilistic poses.
  **PTF: I'm not sure about the concept that frames are non-probabilistic after relaxation. Most estimation methods will still allow one to back out the uncertainty and you are back to a probabilistic graph.**
  **SM: We need it for the clients (such as path-planning), that most probably will not work with probabilistic transformations. These maximum likelihood non-probabilistic transformations depend on what frames have been considered for the relaxation (out of your concept of local relaxation).**
Transaction
  All map queries (excepted trigger bookkeeping) must be performed in a transaction, during which the world is assured to be consistent when viewed from the client.
  A transaction might fail in case of write conflict.
  The suggested paradigm for implementating transactions is *multiversion concurrency control*.
Trigger
  Clients can create triggers to watch part of the database and be notified of changes asynchronously.

Data types used in interfaces
=============================

``TransactionId``
  The unique identifier of a transaction, a ``Uint64``.
``FrameId``
  The unique identifier of a frame, a ``Uint64``.
``FrameIds``
  A list of ``FrameId``.
``FrameName``
  A human-readable text naming important frames, like "world", a ``String``.
``Transform``
  A non-probabilistic 3-D transformation, in SO(3), implemented as a 4x4 matrix of ``Float64``.
``EstimatedFrame``
  A tuple ``(id: FrameId, transform: Transform)`` representing a frame with coordinates estimated with respect to another frame.
``EstimatedFrames``
  A list of ``EstimatedFrame``.
``ProbTransform``
  A probabilistic 3-D transformation.
  
  **SM: What is this encoding?**
  **PTF: I know what encoding I like. It is different than the encoding suggested for ROS. Whatever we pick, it should be clearly documented (mathematically) with a little library attached. My implementation is here https://github.com/furgalep/Schweizer-Messer/tree/master/sm_kinematics but it probably needs some more editing and documentation.**
``LinkId``
  A tuple ``(frame0: FrameId, frame1: FrameId)``.
``LinkIds``
  A list of ``LinkId``.
``Link``
  **PTF: Here I added an edge type to allow graph searches only on a subset of edge types**
  **SM: Ok, this especially makes sense if we consider multiple links between two frames.**
  
  **PTF: I would probably also give each link a unique ide separate from the frame0/frame1 tuple. This would allow users to delete a specific link**
  **SM: I do not like the idea of having a key in addition to the tuple (frame0, frame1, LinkType)**
  **SM: Should the link type be a string? It might be more natural to use and avoid having to allocate integers across subsystems.**
  
  A tuple ``(link: LinkId, transformation: ProbTransform, confidence: Float64, LinkType: Int64)``, in which ``confidence`` expresses how much the link creator was confident that this link actually exists. This is not the same information as ``transformation``, which expresses a probabilistic transformation from ``link.frame1`` to ``link.frame0``, assuming that the link exists.
``Links``
  A list of ``Link``.
``DataType``
  A type of data to be attached to a frame, a ``String``.
``DataTypes``
  A list of ``DataType``.
``DataBlob``
  Opaque binary data.
``Data``
  Data with type as a tuple ``(type: DataType, value: DataBlob)``
``DataSet``
  **PTF: users may also want to store data on edges**
  **SM: why not, but I am not sure to see now what, do you have an exemple?**
  
  A (multi)map of ``FrameId -> Data``.
``Box``
  A three-dimensional box in space defined by its two opposite corners, hence a pair of tuples ``((xmin: Float64, ymin: Float64, zmin: Float64), (xmax: Float64, ymax: Float64, zmax: Float64))``.
  **SM: say something about degenerate case, like if we are in 2D**
``TriggerId``
  Trigger identifier; because it refers to the transport mechanism and not to the database scheme, its type is implementation-dependent.
``TriggerIds``
  A list of ``TriggerId``.
  
Map queries (RPC)
=================

Transaction
-----------

``startTransaction() -> TransactionId``
  Create a new transaction and return its identifier.
``commitTransaction(transaction: TransactionId) -> (Bool, String)``
  Attempt to commit a transaction, return whether it succeeded or failed, and the message.
  Read-only transactions always succeed.
  Transactions involving write might fail if there is a write conflict.
  The granularity of their detection depends on the implementation.
``abortTransaction(transaction: TransactionId, reason: String)``
  Abort a transaction, giving a reason for server logs.
  
All further messages in this section are assumed to take a ``TransactionId`` as first parameter.
For clarity, these are not written explicitely in the following RPC signatures.

Relaxation
----------

``estimateFrames(origin: FrameId) -> EstimatedFrames``
  Return all frames linked to ``origin``
  Their coordinates are relative to ``origin``, which therefore is identity.
``estimateFramesWithinBox(origin: FrameId, box: Box) -> EstimatedFrames``
  Return all frames linked to ``origin`` within ``box`` (relative to ``origin``).
  
  **PTF: The box isn't just centered on origin, it is expressed in the origin coordinate frame. This makes me think we may want to allow users to add another transformation here.**
  **SM: "Centered on origin was wrong", I changed to "relative", this avoids requiring another transformation".**
  
  **PTF: What happens if part of the pose graph is within the box, but the part connecting it to ``origin`` is outside of the box?**
  **SM: good questions, I guess that it will not be considered**
  
  Their coordinates are relative to ``origin``, which therefore is identity.
``estimateFramesWithinSphere(origin: FrameId, radius: Float64) -> EstimatedFrames``
  Return all frames linked to ``origin`` within ``radius`` (centered on ``origin``).
  Their coordinates are relative to ``origin``, which therefore is identity.
``estimateNeighboringFrames(origin: FrameId, linkDist: Uint64, radius: Float64) -> EstimatedFrames``
  Return frames linked to ``origin`` within ``radius`` (centered on ``origin``) and at maximum ``linkDist`` number of links.
  Their coordinates are relative to ``origin``, which therefore is identity.

Data access
-----------
  
``getData(frames: FrameIds, types: DataTypes) -> DataSet``
  Return all data of ``types`` contained in ``frames``.
``getLinks(links: LinkIds) -> Links``
  Return requested links, if they exist.
  In ``LinkId`` in ``links``, also consider permuted frame identifiers.
``getFrameLinks(frame: FrameId) -> Links``
  Return all links touching frame.
``getFrameName(frame: FrameId) -> String``
  Get the human-readable name of a frame.

Setters
-------

``setLink(frame0: FrameId, frame1: FrameId, transform: ProbTransform, confidence: Float64, edgeType: UInt64 )``
  Set a link between two frames, if the link (or its reverse) exists, its transform and confidence are replaced.
``deleteLink(frame0: FrameId, frame1: FrameId)``
  **PTF: I think this should have a link ID. What if there is more than one link between frames? The real question is: Do we think of these links as pseudomeasurments, where you can have more than one connecting two frames? Or do we think of these links as our best guess for geometry, where pseudomeasurements used in graph relaxation should be stored at the edge?**
  **SM: I agree about the real question, and I do not have a definitive answer to it. About the link ID, if we have multiple link types I think that the tuple (frame0, frame1, LinkType) should be the ID.**
  
  Remove the link (or its reverse) between two frames.
``setFrameData(frame: FrameId, Data: data)``
  Set data for ``frame``, if ``data.type`` already exists, the corresponding data are overwritten.
``deleteData(frame: FrameId, type: DataType)``
  Delete data of a give type in a given frame.
``createFrame() -> FrameId``
  Create and return a new FrameId, which is guaranteed to be unique.
``setFrameName(frame: FrameId, name: String)``
  Set the human-readable name of a frame.
``deleteFrame(frame: FrameId)``
  Delete a frame, all its links and all its data.

  
Triggers (messages)
===================

Available types
---------------

``linksChanged(added: Links, removed: Links)``
  Links have been added to or removed from a set of watched frames.
``dataChanged(frames: FrameIds, type: DataType)``
  Data have been changed for a set of watched frames and a data type.
``framesMoved(frames: FrameIds, origin: FrameId)``
  A set of frames have been moved with respect to ``origin``.
  
Trigger setters
---------------

These trigger-bookkeeping queries do not operate within transactions and might fail, by returning invalid trigger identifiers.

``watchLinks(frames: FrameIds, existingTrigger = null: TriggerId) -> TriggerId``
  Watch a set of frames for link changes, return the trigger identifier.
  Optionally reuse an existing trigger of the same type.
  All frames must exist, otherwise this query fails.
``watchData(frames: FrameIds, type: DataType, existingTrigger = null: TriggerId) -> TriggerId``
  Watch a set of frames for data changes, return the trigger identifier.
  Optionally reuse an existing trigger of the same type.
  All frames must exist, otherwise this query fails.
``watchEstimatedTransforms(frames: FrameIds, origin: FrameId, epsilon: (Float64, Float64), existingTrigger = null: TriggerId) -> TriggetId``
  Watch a set of frames for estimated pose changes with respect to origin.
  Set the threshold in (translation, rotation) below which no notification occurs.
  All frames must exist and have a link to origin, otherwise this query fails.
``deleteTriggers(triggers: TriggerIds)``
  Delete triggers if they exist.
  
