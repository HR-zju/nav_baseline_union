"""
  Generated by Eclipse Cyclone DDS idlc Python Backend
  Cyclone DDS IDL version: v0.11.0
  Module: nav_msgs.msg.dds_
  IDL file: Odometry_.idl

"""

from enum import auto
from typing import TYPE_CHECKING, Optional
from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types

# root module import for resolving types
# import nav_msgs

# if TYPE_CHECKING:
#     import geometry_msgs.msg.dds_
#     import std_msgs.msg.dds_



@dataclass
@annotate.final
@annotate.autoid("sequential")
class Odometry_(idl.IdlStruct, typename="nav_msgs.msg.dds_.Odometry_"):
    header: 'unitree_sdk2py.idl.std_msgs.msg.dds_.Header_'
    child_frame_id: str
    pose: 'unitree_sdk2py.idl.geometry_msgs.msg.dds_.PoseWithCovariance_'
    twist: 'unitree_sdk2py.idl.geometry_msgs.msg.dds_.TwistWithCovariance_'


