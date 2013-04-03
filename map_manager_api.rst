:Info: A map manager API for 3D mapping, navigation and scene interpretation
:Author: Stéphane Magnenat <stephane at magnenat dot net>
:Date: 2013-04-03

=======================================================================
 A map manager API for 3D mapping, navigation and scene interpretation
=======================================================================

Rationale
=========

A modern autonomous robotic system is composed of many software building blocks (SLAM, path planning, scene interpretation), most of them depending on some form of localisation and mapping.
Therefore, there is a need for an API that allows these different elements to communicate, with as little interdependency as possible.
This document describes such an API, and discusses its implementation, based on the concept of a transactional map database.
It aims at being as generic as possible and agnostic with respect to the implementation framework.
It is designed to allow for a fully distributed back-end, while abstracting it in such a way that API users do not need to care about the back-end.

Framework requirements
======================

The implementation framework shall allow modularity based on the concept of components.
It shall provide both message passing (called *message* in the rest of this document) and remote procedure invocation (called *RPC* in the rest of this document) for interfacing modules together.

Writing conventions
===================

When variables are described, the convention ``variable: Type`` is used, with ``variable`` having its first letter in lower case and ``Type`` in upper case, and using *CamelCase* for multiple words.

When RPC or messages are described, the convention ``name(arguments) -> ReturnType`` is used, in CamelCase as well.

Concepts
========

Frame
  A frame is a coordinate frame located in 3-D space, linked to other frames through a set of uncertain transformations at different times.
  The uncertainty of the transformations are represented by a probability distribution over SE(3).
  Frames have unique identifiers, and can optionally have human-readable names.
  Frames can have user-specified, arbitrary data attached.

Link
  A probabilistic transformation between two frames, with a type and a time stamp.
  Several links are allowed between the same two frames, provided they are of different types.
  Links can have user-specified, arbitrary data attached.

Estimated Frame Set
  A graph of frames can be relaxed to estimate deterministic poses, with respect to a given frame.
  Several strategy can be provided by the implementation that will lead to different relaxed values.
  The result is a set of transforms in SE(3), each associated to a frame, and a common origin and time.

Data
  Arbitrary binary user data can be attached to Frames and Links.
  
Distribution System
  This API aims at abstracting the database back-end in a way that would allow the latter to scale to a distributed system, conceptually able to scale to encompass all running robots on earth. This implies that the API will talk to a local node that cannot hold the whole data, and that data can be lost if part of the network collapses. This is shocking from a database point of view, but perfectly reasonable from a robotics perspective.

Transaction
  All map queries (excepted trigger bookkeeping) must be performed in a transaction, during which the world is assured to be consistent when viewed from the client.
  A transaction might fail in case of write conflict.
  This approach ensures the `Atomicity` and `Consistency` properties of the commonly used ACID model in database literature, but does not ensure `Isolation` (parallel writes working on different part of a distributed database might break this property) or `Durability` (a sufficiently-large number of servers dying in a distributed database might result in data loss, as total replicability is not a realistic goal). We do not even aim at an *eventual consistency* model because the typical amount of data produced by modern robotics system hinders the notion of complete replicas.
  The suggested paradigm for implementating transactions is *multiversion concurrency control*.

Trigger
  Clients can create triggers to watch part of the database and be notified of changes asynchronously.

Data types used in interfaces
=============================

Data types are specified in detail to ensure cross-implementation compatibility. They are in general chosen to be generous enough in what they can represent, in order to avoid overflows and limitations. Client libraries are allowed to use other and shorter data types (for instance ``ROS::Time`` for ``TimeStamp`` in ROS) for the ease of interfacing, but the backend must use the proposed types.

For a given type ``T``, we implicitely defines ``Ts`` to be a list of ``T``. We assume common scalar types (bool, int, float) to be available and defined implicitely.

``TransactionId``
  The unique identifier of a transaction, a ``Uint64``.
``TimeStamp``
  A high-precision time stamp, a tuple ``(Int64, Int32)`` in which the first element represents the number of seconds since 1 January 1970, not counting leap seconds (POSIX time), and the second element in the nano-second sub-precision. 
``Interval``
  A tuple ``(start:TimeStamp, end:TimeStamp)`` denoting a time interval.
``FrameId``
  The unique identifier of a frame, a ``Uint64``.
``FrameName``
  A human-readable text naming important frames, like "world", a ``String``.
  Frame names are unique in the whole system.
``Transform``
  A deterministic 3-D transformation, in SE(3), implemented as a 4x4 matrix of ``Float64``.
``AttachedTransform``
  A transformation attached to a frame (the coordinate frame this transform defines), a tuple ``(id: FrameId, transform: Transform)``.
``EstimatedFrameSet``
  The result of a graph relaxation operation.
  A structure ``(origin: FrameId, time: Interval, estimates: AttachedTransform)``, in which ``estimates`` are frames expressed with respect to ``origin``, during the interval ``time``.
``UncertainTransform``
  An uncertain 3-D transformation in SE(3), composed of a ``Transform`` and a Gaussian uncertainty of the transformation in the tangent space of SE(3) (a 6x6 covarience matrix).
``LinkId``
  The unique identifier of a link, a ``Uint64``.
``Link``
  A structure ``(link: LinkId, childFrame: FrameId, parentFrame: FrameId, label: String, time: TimeStamp, transformation: UncertainTransform, confidence: Float64)``.
  This links ``childFrame`` to ``parentFrame``, by expressing how to transform points from the first to the second, with uncertainty and at a give ``time``.
  The ``confidence`` value expresses how much the link creator was confident that this link actually exists. This is not the same information as ``transformation``, which expresses an uncertain transformation of points from ``childFrame`` to ``parentFrame``, assuming that the link exists.
  The ``label`` string allows the user to label links.
``DataType``
  A type of data to be attached to a frame or a link, a ``String``.
``DataBlob``
  Opaque binary data.
``Data``
  Data with type as a tuple ``(type: DataType, value: DataBlob)``
``FrameDataSet``
  A (multi)map of ``FrameId -> Data``.
``LinkDataSet``
  A (multi)map of ``LinkIds -> Data``.
``Box``
  A three-dimensional box in space defined by its two opposite corners, hence a pair of tuples ``((xmin: Float64, ymin: Float64, zmin: Float64), (xmax: Float64, ymax: Float64, zmax: Float64))``.
``EstimationStrategy``
  The estimation strategy to use to estimate non-probabilistic frames, a ``String``.
``TriggerId``
  Trigger identifier; because it refers to the transport mechanism and not to the database scheme, its type is implementation-dependent.
  
    SM: TODO: split this into different types for different triggers.
  
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

    SM: TODO: add more time option, such as "most recent one", etc. 
  
    SM: TODO: allow to filter using labels
    
    SM: TODO: maybe unify these above two and the following 4 functions using the concepts of TimeFilter, LabelFilter and SpaceFilter.

``estimateFrames(time: Interval, origin: FrameId, strategy: EstimationStrategy = "") -> EstimatedFrameSet``
  Return all frames linked to ``origin`` during ``time``, using a given ``strategy``; if none given, use the default provided by the implementation.
  The frames' coordinates are relative to ``origin``, which therefore is identity.
  If the implementation does not provide ``strategy``, it is allowed to use its default one.
``estimateFramesWithinBox(time: Interval, origin: FrameId, box: Box, strategy: EstimationStrategy = "") -> EstimatedFrameSet``
  Return all frames linked to ``origin`` during ``time``, within ``box`` (relative to ``origin``) using a given ``strategy``; if none given, use the default provided by the implementation.
  The frames' coordinates are relative to ``origin``, which therefore is identity.
  If part of the pose graph is within the box, but the part connecting it to ``origin`` is outside of the box, the inclusion of this part is left to the implementation.
  If the implementation does not provide ``strategy``, it is allowed to use its default one.
``estimateFramesWithinSphere(time: Interval, origin: FrameId, radius: Float64, strategy: EstimationStrategy = "") -> EstimatedFrameSet``
  Return all frames linked to ``origin`` during ``time``, within ``radius`` (centered on ``origin``) using a given ``strategy``; if none given, use the default provided by the implementation.
  The frames' coordinates are relative to ``origin``, which therefore is identity.
  If the implementation does not provide ``strategy``, it is allowed to use its default one.
``estimateNeighboringFrames(time: Interval, origin: FrameId, linkDist: Uint64, radius: Float64, strategy: EstimationStrategy = "") -> EstimatedFrameSet``
  Return frames linked to ``origin`` during ``time``, within ``radius`` (centered on ``origin``) and at maximum ``linkDist`` number of links, using a given ``strategy``; if none given, use the default provided by the implementation.
  The frames' coordinates are relative to ``origin``, which therefore is identity.
  If the implementation does not provide ``strategy``, it is allowed to use its default one.
  
    SM: TODO: define what it means being "inside" as we have uncertain transformations. Should we ignore the uncertainty? Or on the contrary make an iterative relaxation-selection process? Maybe this is part of strategy and should be left to the backend.
    
    SM: TODO: separate link selections from relaxation.

Data access
-----------
  
``getFrameData(frames: FrameIds, types: DataTypes) -> FrameDataSet``
  Return all data of ``types`` contained in ``frames``.
``getLinkData(links: LinkIds, types: DataTypes) -> LinkDataSet``
  Return all data of ``types`` contained in ``links``.
``getLinks(links: LinkIds) -> Links``
  Return requested links, if they exist.
  In ``LinkId`` in ``links``, also consider permuted frame identifiers.
``getFrameLinks(frame: FrameId) -> Links``
  Return all links touching frame.
``getFrameName(frame: FrameId) -> String``
  Get the human-readable name of a frame.
  
  SM: TODO: add the selecting of links, mostly from relaxation section.

Setters
-------

``setLink(childFrame: FrameId, parentFrame: FrameId, label: String, time: Timestamp, transform: UncertainTransform, confidence: Float64, edgeType: UInt64 ) -> LinkId``
  Set a link between two frames and return its identifier.
``deleteLink(link: LinkId)``
  Remove a give link between two frames.
  Remove the link (or its reverse) of a given type between two frames.
  This removes this link for all time stamps, and deletes all data associated with this link.
``setFrameData(frame: FrameId, Data: data)``
  Set data for ``frame``, if ``data.type`` already exists, the corresponding data are overwritten.
``deleteFrameData(frame: FrameId, type: DataType)``
  Delete data of a give type in a given frame.
``setLinkData(link: LinkId, Data: data)``
  Set data for ``link``, if ``data.type`` already exists, the corresponding data are overwritten.
``deleteLinkData(link: LinkId, type: DataType)``
  Delete data of a give type in a given link.
``createFrame(name: String = none) -> FrameId``
  Create and return a new FrameId, which is guaranteed to be unique.
  Optionally pass a name.
  If a name is passed, this call requires accessing a global name registry, and therefore might take time to complete.
``setFrameName(frame: FrameId, name: String) -> Bool``
  Set the human-readable name of a frame.
  Return true if the name has been set, false if another frame already has this name.
  Because this call require accessing a global name registry, it might take time to complete.
``deleteFrame(frame: FrameId)``
  Delete a frame, all its links and all its data.

  
Triggers (messages)
===================

Available types
---------------

``linksChanged(added: LinkIds, removed: LinkIds)``
  Links have been added to or removed from a set of watched frames.
``estimatedFramesMoved(frames: FrameIds, origin: FrameId)``
  The estimated postition of a set of frames have been moved with respect to ``origin``.
``frameDataChanged(frames: FrameIds, type: DataType)``
  Data have been changed for a set of watched frames and a data type.
``linkDataChanged(links: LinkIds, type: DataType)``
  Data have been changed for a set of watched links and a data type.

  
Trigger book-keeping
--------------------

These trigger-bookkeeping queries do not operate within transactions and might fail, by returning invalid trigger identifiers.

``watchLinks(frames: FrameIds, existingTrigger = none: TriggerId) -> TriggerId``
  Watch a set of frames for link changes, return the trigger identifier.
  Optionally reuse an existing trigger of the same type.
  All frames must exist, otherwise this query fails.
``watchEstimatedTransforms(frames: FrameIds, origin: FrameId, epsilon: (Float64, Float64), existingTrigger = none: TriggerId) -> TriggetId``
  Watch a set of frames for estimated pose changes with respect to origin.
  Set the threshold in (translation, rotation) below which no notification occurs.
  All frames must exist and have a link to origin, otherwise this query fails.
  
  SM: TODO: clean up rexation API and then revamp this call
  
``watchFrameData(frames: FrameIds, type: DataType, existingTrigger = none: TriggerId) -> TriggerId``
  Watch a set of frames for data changes, return the trigger identifier.
  Optionally reuse an existing trigger of the same type.
  All frames must exist, otherwise this query fails.
``watchLinkData(links: LinkIds, type: DataType, existingTrigger = none: TriggerId) -> TriggerId``
  Watch a set of links for data changes, return the trigger identifier.
  Optionally reuse an existing trigger of the same type.
  All frames must exist, otherwise this query fails.
``deleteTriggers(triggers: TriggerIds)``
  Delete triggers if they exist.


Notes for distributed implementations
=====================================
 
Unique identifiers
------------------
 
In this documents, unique identifiers (``FrameId`` and ``LinkId``) have type ``Uint64``, whose range is large enough to refer objects between the client and the backend.
However, in a distributed system where multiple backend have to communicate asynchronously, this might not be large enough.
In such a system, we propose to use a 32 byte identifier.
The first 16 bytes shall identify the host (for instance holding an IPv6 address); in a centralised system, these can be 0.
The last 16 bytes shall implement an identifier that is unique on this host, for instance an ever-increasing number.
The identifier space generated by 16 bytes is large enough such the host will never produce the same number twice during its life time.
The backend shall provide a bijective mapping between the identifiers used by the API and the ones used between backends.