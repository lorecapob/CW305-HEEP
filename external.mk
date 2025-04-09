# Copyright 2025 Politecnico di Torino.
# Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
# SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
#
# File: external.mk
# Author: Lorenzo Capobianco
# Date: 09/04/2025
# Description: this file is used to include the CW305-X-HEEP Makefile in the current Makefile.

MAKE	= make

# Any target that was not present on the uppermost Makefile (the one in which this file is included) 
# will be passed to the CW305-X-HEEP Makefile, which includes the X-HEEP Makefile in turn.
%:
	@echo Calling CW305 X-HEEP Makefile
	$(MAKE) -C $(CW305_XHEEP_DIR) $(MAKECMDGOALS)
