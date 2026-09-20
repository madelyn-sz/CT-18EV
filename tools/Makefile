# Host tests for the HAL-independent firmware modules.
#
#   make          build and run everything
#   make clean    remove the binaries
#
# These run on the build machine, not the target, and are deliberately kept
# out of the STM32CubeIDE build.

CC      ?= cc
CFLAGS  ?= -O1 -g -Wall -Wextra -I../Core/Inc
LDLIBS   = -lm

TESTS = soc_kf_test lc_test

.PHONY: all test clean
all: test

test: $(TESTS)
	@for t in $(TESTS); do \
		echo "== $$t"; \
		./$$t || exit 1; \
	done

# soc_kf reaches the HAL for its critical sections; SOC_KF_HOST swaps in stubs.
soc_kf_test: soc_kf_test.c ../Core/Src/soc_kf.c ../Core/Inc/soc_kf.h ../Core/Inc/soc_kf_tables.h
	$(CC) $(CFLAGS) -DSOC_KF_HOST -o $@ soc_kf_test.c $(LDLIBS)

# launch_control has no HAL dependency, so it needs no shim.
lc_test: lc_test.c ../Core/Src/launch_control.c ../Core/Inc/launch_control.h
	$(CC) $(CFLAGS) -o $@ lc_test.c $(LDLIBS)

clean:
	rm -rf $(TESTS) *.dSYM
