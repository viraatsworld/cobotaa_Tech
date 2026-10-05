# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Pick-to-bin: pick an object up from the table's spawn zone and drop it into the blue pallet.

The object is a parameter (``grasp_object``, see ``scene/grasp_objects.py``):
the hammer and the soda can are registered. The arm is learned (top-down TCP
steps); the gripper is a fixed rule that closes at the grasp point and opens
over the bin. See ``plan-hammer-to-bin.md``.
"""
