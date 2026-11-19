BUILD ?= build

CLANG_TIDY ?= $(firstword $(wildcard \
  $(HOME)/.vscode/extensions/ms-vscode.cpptools-*/LLVM/bin/clang-tidy) \
  clang-tidy)
CPP_FILES  := $(wildcard src/*.cpp src/*.hpp tests/*.cpp)
TIDY_FLAGS := --quiet -p $(BUILD) \
  --extra-arg=-isysroot$(shell xcrun --show-sdk-path) \
  --extra-arg=-resource-dir=$(shell clang -print-resource-dir)

.PHONY: help build test check clean

help:
	@echo "make build     compile glidepath and the tests"
	@echo "make test      build, then run the tests"
	@echo "make check     clang-format and clang-tidy"
	@echo "make clean     remove compiled objects"

$(BUILD)/CMakeCache.txt:
	cmake -S . -B $(BUILD) -DCMAKE_BUILD_TYPE=Release

build: $(BUILD)/CMakeCache.txt
	cmake --build $(BUILD) -j8

test: build
	ctest --test-dir $(BUILD)

check: build
	clang-format --dry-run --Werror $(CPP_FILES)
	$(CLANG_TIDY) $(TIDY_FLAGS) src/*.cpp

clean:
	cmake --build $(BUILD) --target clean