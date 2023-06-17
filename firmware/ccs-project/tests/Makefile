# This file is part of the OpenAXES project, a wireless IMU.
# Copyright 2023 Nils Stanislawski and Fritz Webering
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

CFLAGS=-g3 -I../Application/ -Wall -Werror -pedantic -O0
LDFLAGS=-lm

TESTS = test_matrix
TEST_EXECUTABLES = $(addprefix $(BUILDDIR)/,$(TESTS))
BUILDDIR = build

all: $(TEST_EXECUTABLES) # Build all test cases

clean: # Remove all built test cases
	rm -rf $(BUILDDIR)

test: $(TEST_EXECUTABLES) # Run all test cases
	@echo "-----------------------"
	@echo "Running all test cases:"
	@set -o errexit; for t in $(TEST_EXECUTABLES); do echo -n " + $$t... "; $$t; echo "success"; done
	@echo "---- All succeeded ----"

$(BUILDDIR)/:
	mkdir -p $@

$(TEST_EXECUTABLES): | $(BUILDDIR)/ # "order-only prerequisite": must exist, but is not part of $^

# Recipes to build individual test cases. Add new tests here.
$(BUILDDIR)/test_matrix: test_matrix.c ../Application/vectormath.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

.PHONY: test all clean
