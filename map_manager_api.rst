:Info: A map manager API for 3D mapping, navigation and scene interpretation
:Author: Stéphane Magnenat <stephane at magnenat dot net>
:Date: 2013-02-20

=======================================================================
 A map manager API for 3D mapping, navigation and scene interpretation
=======================================================================

Rationale
=========

A modern autonomous robotic system is composed of many software building blocks (SLAM, path planning, scene interpretation), most of them depending on some form of localisation and mapping.
Therefore, there is a need for an API that allows these different elements to communicate, with as little interdependency as possible.
This document describes such an API, based on the concept of a transactional map database.
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
  A frame is a coordinate frame located in 3-D space, linked with other frames through a set of probabilistic transformations.
  Frames have unique identifiers, and can optionally have human-readable names.
  Frames can hold arbitrary data.

    FC: Transformations are not probabilistic. The knowledge we have of them may be uncertain and we may use probability to represent this uncertainty (with some prior knkowledge/model). I'm not sure how we want to write that.

    FC: I don't understand: "Frames can hold arbitrary data". Coordinate frames usually don't hold data, even if some data is related to coordinate frames.

    FC: Are the frames assumed to be static or can they move? From the lack of time component in the EstimatedFrame data, it seems that we're talking about static frames. I'm not sure I like that, but ROS people will really expect it unless being explicitely told otherwise (and even then... ;)).

Link
  A probabilistic transformation between two frames, with a type and a time stamp.
  Several links are allowed between the same two frames, provided they are of different types.
  Links can hold arbitrary data.

    FC: again what is that arbitrary data supposed to be? If it's the confidence, it seems mandatory.

EstimatedFrame
  A graph of frames can be relaxed to have non-probabilistic poses.
  Several strategy can be provided by the implementation that will lead to different relaxed values.

    FC: It seems to me it's not the Frame that is estimated, but the transformation between two of them. Except if you assume an implicit world frame but that is probably worth telling.

    FC: If you have two estimations with two different contexts, how do you handle it/invalidate ? (I would propose spawning a RelaxedWorld_XX frame for each relaxation event but there might be better solutions.)

    FC: The description of EstimatedFrame is only implicit (you only talk about the graph, not the frame).

Transaction
  All map queries (excepted trigger bookkeeping) must be performed in a transaction, during which the world is assured to be consistent when viewed from the client.
  A transaction might fail in case of write conflict.
  The suggested paradigm for implementating transactions is *multiversion concurrency control*.

    FC: If you go for transaction, why not have atomicity (actually ACID properties) to avoid write conflict?

Trigger
  Clients can create triggers to watch part of the database and be notified of changes asynchronously.

Data types used in interfaces
=============================

``TransactionId``
  The unique identifier of a transaction, a ``Uint64``.
``TimeStamp``
  A high-precision time stamp, a tuple ``(Int64, Int32)`` in which the first element represents the number of seconds since 1 January 1970, not counting leap seconds (POSIX time), and the second element in the nano-second sub-precision.

    FC: ROS is using ``(Int32, Int32)``, C++11 proposes ``(Rep, Ratio)`` where Rep is a number type and Ratio a multiplier in seconds. Why being so specific about it? and why this specific choice different from (some) (reasonnable) standards?

``FrameId``
  The unique identifier of a frame, a 32-byte array.
  In a distributed system, the first 16 bytes shall identify the host (for instance holding an IPv6 address); in a centralised system, these can be 0.
  The last 16 bytes shall implement an identifier that is unique on this host, for instance an ever-increasing number.
  The identifier space generated by 16 bytes is large enough such the host will never produce the same number twice during its life time.
``FrameIds``
  A list of ``FrameId``.

    FC: gathering lists of stuff somewhere else may ease reading. Or you can define a generic list of Stuff as Stuffs.

``FrameName``
  A human-readable text naming important frames, like "world", a ``String``.

    FC: shouldn't there be a Frame data: ``(FrameId, FrameName)`` (cf [g|s]etFrameName below)?

    FC: It seems you can have several frames with the same name and then you use a unique index to differentiate. That's a choice worth of emphasis (I'm not sure I share that).

``Transform``
  A non-probabilistic 3-D transformation, in SE(3), implemented as a 4x4 matrix of ``Float64``.
``EstimatedFrame``
  A tuple ``(id: FrameId, transform: Transform)`` representing a frame with coordinates estimated with respect to another frame.

    FC: here there is an implicit reference frame. Why not similar to Link?

    FC: time is missing here. Cf static frame discussion above.

``EstimatedFrames``
  A list of ``EstimatedFrame``.
``ProbTransform``
  A probabilistic 3-D transformation in SE(3), composed of a ``Transform`` and a 6x6 covarience matrix.
  
  **SM: What is this encoding?**
  **PTF: I know what encoding I like. It is different than the encoding suggested for ROS. Whatever we pick, it should be clearly documented (mathematically) with a little library attached. My implementation is here https://github.com/furgalep/Schweizer-Messer/tree/master/sm_kinematics but it probably needs some more editing and documentation.**
``LinkType``
  A type of link, a ``String``.
  This allows multiple links of different types between two frames.

    FC: I'm mixed between calling it a LinkLabel and keeping LinkType but with an enum instead of a string.

    FC: Allowance of multiple links of different types is described below, but does not seem necessary here.

``LinkId``
  A tuple ``(frame0: FrameId, frame1: FrameId, type: LinkType, time: TimeStamp)``.
  This uniquely identifies a link.
  The map manager cannot hold two ``LinkId`` with similar ``type`` and ``time``, and similar but inverted frames ``frame0`` and ``frame1``.
``LinkIds``
  A list of ``LinkId``.
``Link``
  A tuple ``(link: LinkId, transformation: ProbTransform, confidence: Float64)``, in which ``confidence`` expresses how much the link creator was confident that this link actually exists. This is not the same information as ``transformation``, which expresses a probabilistic transformation from ``link.frame1`` to ``link.frame0``, assuming that the link exists.

    FC: there is a lack of symmetry between the FrameId and the LinkId. FrameId is a unique index and the (missing) Frame object links the id to its name and you propose to use FrameId as a sort of handle for all the API. Why not doing the same with links? Have a unique LinkId and the Link table would have corresponding info: frame0, frame1, type, stamp, transform, confidence. But having the current LinkID structure seems inconvenient as a handle and splitting the time/transformation+confidence in this manner is rather arbitrary. 

``Links``
  A list of ``Link``.
``DataType``
  A type of data to be attached to a frame or a link, a ``String``.
``DataTypes``
  A list of ``DataType``.
``DataBlob``
  Opaque binary data.
``Data``
  Data with type as a tuple ``(type: DataType, value: DataBlob)``
``FrameDataSet``
  A (multi)map of ``FrameId -> Data``.
``LinkDataSet``
  A (multi)map of ``LinkIds -> Data``.

    FC: I understand now the "Frame can hold data" sentence, I would actually add Data as a concept and say that Data can be linked to frames and links.

``Box``
  A three-dimensional box in space defined by its two opposite corners, hence a pair of tuples ``((xmin: Float64, ymin: Float64, zmin: Float64), (xmax: Float64, ymax: Float64, zmax: Float64))``.
``EstimationStrategy``
  The estimation strategy to use to estimate non-probabilistic frames, a ``String``.
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

  FC: In all this section, it might be unclear whether ``origin`` is just there for defining which neighborhood or it's also the reference in which the transform will be expressed.

``estimateFrames(origin: FrameId, strategy: EstimationStrategy = "") -> EstimatedFrames``
  Return all frames linked to ``origin`` using a given ``strategy``; if none given, use the default provided by the implementation.
  The frames' coordinates are relative to ``origin``, which therefore is identity.
  If the implementation does not provide ``strategy``, it is allowed to use its default one.
``estimateFramesWithinBox(origin: FrameId, box: Box, strategy: EstimationStrategy = "") -> EstimatedFrames``
  Return all frames linked to ``origin`` within ``box`` (relative to ``origin``) using a given ``strategy``; if none given, use the default provided by the implementation.
  The frames' coordinates are relative to ``origin``, which therefore is identity.
  If part of the pose graph is within the box, but the part connecting it to ``origin`` is outside of the box, the inclusion of this part is left to the implementation.
  If the implementation does not provide ``strategy``, it is allowed to use its default one.
``estimateFramesWithinSphere(origin: FrameId, radius: Float64, strategy: EstimationStrategy = "") -> EstimatedFrames``
  Return all frames linked to ``origin`` within ``radius`` (centered on ``origin``) using a given ``strategy``; if none given, use the default provided by the implementation.
  The frames' coordinates are relative to ``origin``, which therefore is identity.
  If the implementation does not provide ``strategy``, it is allowed to use its default one.
``estimateNeighboringFrames(origin: FrameId, linkDist: Uint64, radius: Float64, strategy: EstimationStrategy = "") -> EstimatedFrames``
  Return frames linked to ``origin`` within ``radius`` (centered on ``origin``) and at maximum ``linkDist`` number of links, using a given ``strategy``; if none given, use the default provided by the implementation.
  The frames' coordinates are relative to ``origin``, which therefore is identity.
  If the implementation does not provide ``strategy``, it is allowed to use its default one.

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

Setters
-------

``setLink(frame0: FrameId, frame1: FrameId, transform: ProbTransform, confidence: Float64, edgeType: UInt64 )``
  Set a link between two frames, if the link (or its reverse) exists, its transform and confidence are replaced.
``deleteLink(frame0: FrameId, frame1: FrameId, type: LinkType)``
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
``createFrame() -> FrameId``
  Create and return a new FrameId, which is guaranteed to be unique.

    FC: I would create a symmetry between frames and links by fusing createFrame and setFrameName, and having setLink return a unique index.

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
``framesMoved(frames: FrameIds, origin: FrameId)``
  A set of frames have been moved with respect to ``origin``.

    FC: Is it the frame that has moved or the knowledge we have of its transformation with respect to origin that has changed? This is unclear throughout all the API actually.

``frameDataChanged(frames: FrameIds, type: DataType)``
  Data have been changed for a set of watched frames and a data type.
``linkDataChanged(links: LinkIds, type: DataType)``
  Data have been changed for a set of watched links and a data type.

  
Trigger book-keeping
--------------------

These trigger-bookkeeping queries do not operate within transactions and might fail, by returning invalid trigger identifiers.

``watchLinks(frames: FrameIds, existingTrigger = null: TriggerId) -> TriggerId``
  Watch a set of frames for link changes, return the trigger identifier.
  Optionally reuse an existing trigger of the same type.
  All frames must exist, otherwise this query fails.
``watchEstimatedTransforms(frames: FrameIds, origin: FrameId, epsilon: (Float64, Float64), existingTrigger = null: TriggerId) -> TriggetId``
  Watch a set of frames for estimated pose changes with respect to origin.
  Set the threshold in (translation, rotation) below which no notification occurs.
  All frames must exist and have a link to origin, otherwise this query fails.
``watchFrameData(frames: FrameIds, type: DataType, existingTrigger = null: TriggerId) -> TriggerId``
  Watch a set of frames for data changes, return the trigger identifier.
  Optionally reuse an existing trigger of the same type.
  All frames must exist, otherwise this query fails.
``watchLinkData(links: LinkIds, type: DataType, existingTrigger = null: TriggerId) -> TriggerId``
  Watch a set of links for data changes, return the trigger identifier.
  Optionally reuse an existing trigger of the same type.
  All frames must exist, otherwise this query fails.
``deleteTriggers(triggers: TriggerIds)``
  Delete triggers if they exist.
  
