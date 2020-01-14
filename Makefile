
BASE = .

target: src

DEPEND = $(shell find rai -mindepth 1 -maxdepth 1 -printf "%f ")

################################################################################

src_paths =  $(shell find rai -mindepth 1 -maxdepth 3 -type d -not -name 'retired' -printf "%f ")

test_paths = $(shell find test -mindepth 3 -maxdepth 3 -name 'Makefile' -printf "%h ")

bin_paths = $(shell find bin -mindepth 2 -maxdepth 2 -name 'Makefile' -printf "%h ")

################################################################################

installUbuntuAll: force
	@echo "##### calling make installUbuntu in each lib"
	@find rai -mindepth 1 -maxdepth 1 -type d -exec make installUbuntu -C {} \;

printUbuntuAll: $(DEPEND:%=inPath_printUbuntu/%) printUbuntu

printDependAll: $(DEPEND:%=inPath_printDepend/%) printDepend

tests: $(test_paths:%=inPath_make/%)

bin: $(bin_paths:%=inPath_make/%)

src: $(src_paths:%=inPath_makeLib/%)

dependAll: $(src_paths:%=inPath_depend/%)

clean: $(src_paths:%=inPath_clean/%) cleanLocks

cleanStart: force
	@read -p " *** WARNING: This will rm ALL local files/changes (e.g. project/temporary/data files) - abort if you don't want to continue" yn
	git clean -f -d -x
	cp build/config.mk.default build/config.mk

################################################################################

# test: setConfigFlag $(exa_paths:%=inPath_clean/%) cleanLocks $(exa_paths:%=inPath_make/%)

# setConfigFlag: force
# 	echo "RAI_TESTS = 1" > build/z.mk

runTests: tests
	@rm -f z.test-report
	@find test -mindepth 2 -maxdepth 2 -type d \
		-exec build/run-path.sh {} \;

################################################################################

deletePotentiallyNonfreeCode: force
	@rm -Rf rai/Kin/SWIFT rai/Kin/SWIFT_decomposer rai/Geo/Lewiner

################################################################################

include $(BASE)/build/generic.mk
