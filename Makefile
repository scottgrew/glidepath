BUILD ?= build

CLANG_TIDY ?= $(firstword $(wildcard \
  $(HOME)/.vscode/extensions/ms-vscode.cpptools-*/LLVM/bin/clang-tidy) \
  clang-tidy)
CPP_FILES  := $(wildcard fsw/*.hpp fsw/*.cpp sim/*.hpp sim/*.cpp app/*.hpp app/*.cpp tests/*/*.cpp)
TIDY_FLAGS := --quiet -p $(BUILD) \
  --extra-arg=-isysroot$(shell xcrun --show-sdk-path) \
  --extra-arg=-resource-dir=$(shell clang -print-resource-dir) \
  --extra-arg=-DEIGEN_DONT_VECTORIZE

SCENARIO ?= autoland_xwind_left_10

.PHONY: flightgear fly live record help build test check clean

help:
	@echo "make build     compile glidepath and the tests"
	@echo "make test      build, then run the tests"
	@echo "make check     clang-format and clang-tidy"
	@echo "make flightgear  start FlightGear as the 3D view for glidepath --flightgear"
	@echo "make fly       fly SCENARIO (default $(SCENARIO)) in FlightGear (start make flightgear first)"
	@echo "make live      fly SCENARIO with live telemetry for Lichtblick on ws://localhost:8765"
	@echo "make record    fly SCENARIO and write the telemetry to SCENARIO.mcap"
	@echo "make clean     remove compiled objects"

$(BUILD)/CMakeCache.txt:
	cmake -S . -B $(BUILD) -DCMAKE_BUILD_TYPE=Release

build: $(BUILD)/CMakeCache.txt
	cmake --build $(BUILD) -j8

test: build
	ctest --test-dir $(BUILD)

check: build
	clang-format --dry-run --Werror $(CPP_FILES)
	@! grep -rnE '#include .*(sim/|FG[A-Z]|simgear|JSBSim)' fsw/ || (echo "fsw must not depend on the simulator" && false)
	$(CLANG_TIDY) $(TIDY_FLAGS) $(wildcard fsw/*.cpp sim/*.cpp app/*.hpp app/*.cpp)

clean:
	cmake --build $(BUILD) --target clean

fly: build
	$(BUILD)/glidepath --scenario $(SCENARIO) --flightgear

live: build
	$(BUILD)/glidepath --scenario $(SCENARIO) --live

record: build
	$(BUILD)/glidepath --scenario $(SCENARIO) --record $(SCENARIO).mcap

FLIGHTGEAR ?= /Applications/FlightGear.app/Contents/MacOS/FlightGear
FG_ROOT ?= $(HOME)/Library/Application Support/FlightGear/fgdata_2024_1
FG_RENDER ?= --prop:/sim/rendering/multi-sample-buffers=true \
	--prop:/sim/rendering/multi-samples=4 \
	--prop:/sim/rendering/shadows/enabled=true \
	--prop:/sim/rendering/specular-highlight=true \
	--prop:/sim/rendering/shaders/landmass=4 \
	--prop:/sim/rendering/shaders/transition=3 \
	--prop:/sim/rendering/shaders/water=3 \
	--prop:/sim/rendering/shaders/model=3 \
	--prop:/sim/rendering/shaders/clouds=3 \
	--prop:/sim/rendering/shaders/vegetation-effects=3 \
	--prop:/sim/rendering/shaders/wind-effects=2
# glidepath flies the aircraft, so FlightGear's own c172p tanks are filled to
# stop its out-of-fuel warning.
FG_FUEL ?= --prop:/consumables/fuel/tank[0]/level-gal_us=20 \
	--prop:/consumables/fuel/tank[1]/level-gal_us=20 \
	--prop:/consumables/fuel/tank[0]/selected=true \
	--prop:/consumables/fuel/tank[1]/selected=true

# FlightGear draws the aircraft that glidepath flies: it takes position and
# attitude from glidepath --flightgear over UDP 5550 instead of its own model.
flightgear:
	"$(FLIGHTGEAR)" --fg-root="$(FG_ROOT)" --aircraft=c172p \
		--fdm=external --native-fdm=socket,in,60,,5550,udp \
		--lat=37.4 --lon=-122.0 --altitude=4000 --heading=90 \
		--timeofday=afternoon --terrasync --disable-ai-traffic \
		--disable-real-weather-fetch $(FG_RENDER) $(FG_FUEL)

