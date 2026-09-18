# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""USD spawners that apply the scene assets' corrections at spawn time.

Only the configs are exported here. The spawn functions live in
:mod:`.spawners` and are loaded by name when a scene is built, because they
import ``pxr``, which must wait for Kit.
"""

from techtory_cobotta_isaaclab.spawners.spawners_cfg import (
    CobottaRg6UsdFileCfg as CobottaRg6UsdFileCfg,
)
from techtory_cobotta_isaaclab.spawners.spawners_cfg import (
    GraspableUsdFileCfg as GraspableUsdFileCfg,
)
from techtory_cobotta_isaaclab.spawners.spawners_cfg import (
    StaticUsdFileCfg as StaticUsdFileCfg,
)
