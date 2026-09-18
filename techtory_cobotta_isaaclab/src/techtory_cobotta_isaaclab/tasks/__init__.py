# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Task registrations. Importing this module registers every task with gymnasium."""

from isaaclab_tasks.utils import import_packages

import_packages(__name__, ["utils", ".mdp"])
