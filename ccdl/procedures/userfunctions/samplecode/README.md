# A sample library that implements a custom extension to the CCD language.

## Building

### Preparation
1. Run 'make -f /opt/trm/lib/Makefile targets' to list installed execution targets.
2. Edit Makefile to adjust the execution target, your company's name and the library install path.
3. Run 'make configure' to generate some template files.

### Hacking
1. Edit the source code.
2. Edit the 'build-succeeds' test so that each function in your library is called at least once.
3. Run 'make lib' to build your library.

## Testing

### Preparation
1. Run 'make tests' to build the infrastructure for testing.
2. Edit file */custom-targets.mk, if needed.

### Running tests

1.  Run `make run-tests` or `make all` to run all tests.

## Releasing the library

1. Run 'make lib' to build your library.
2. Run 'make install DESTDIR=/tmp' to test the installation process.

Consider building an rpm package instead of installing the library with 'make install'!
