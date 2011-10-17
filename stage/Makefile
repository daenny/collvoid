include $(shell rospack find mk)/cmake.mk

# Hack to allow upgrade from svn checkout to tarball without 'make clean'
# in between.  Something is caching old pkg-config state, and I haven't
# been able to track it down.
all: clean-old-build
clean-old-build:
	if [ -d build/stage-svn ]; then rm -rf build; fi

clean: extra-clean
extra-clean:
	make -f Makefile.stage clean
