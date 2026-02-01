CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Wpedantic -Dcimg_display=0 -I../src -I../include -I./include -I./matplotplusplus/source \
    -I./matplotplusplus/source/3rd_party/nodesoup/include \
    -I./matplotplusplus/source/3rd_party/nodesoup/src \
    -I./matplotplusplus/source/3rd_party/cimg
LDFLAGS ?=

APP := flight_computer_standalone
MATPLOT_DIR := matplotplusplus/source
MATPLOT_SRCS := \
    $(MATPLOT_DIR)/matplot/backend/backend_interface.cpp \
    $(MATPLOT_DIR)/matplot/backend/backend_registry.cpp \
    $(MATPLOT_DIR)/matplot/backend/gnuplot.cpp \
    $(MATPLOT_DIR)/matplot/core/axes_object.cpp \
    $(MATPLOT_DIR)/matplot/core/axes_type.cpp \
    $(MATPLOT_DIR)/matplot/core/axis_type.cpp \
    $(MATPLOT_DIR)/matplot/core/figure_registry.cpp \
    $(MATPLOT_DIR)/matplot/core/figure_type.cpp \
    $(MATPLOT_DIR)/matplot/core/legend.cpp \
    $(MATPLOT_DIR)/matplot/core/line_spec.cpp \
    $(MATPLOT_DIR)/matplot/util/colors.cpp \
    $(MATPLOT_DIR)/matplot/util/common.cpp \
    $(MATPLOT_DIR)/matplot/util/contourc.cpp \
    $(MATPLOT_DIR)/matplot/util/popen.cpp \
    $(MATPLOT_DIR)/matplot/util/world_cities.cpp \
    $(MATPLOT_DIR)/matplot/util/world_map_10m.cpp \
    $(MATPLOT_DIR)/matplot/util/world_map_50m.cpp \
    $(MATPLOT_DIR)/matplot/util/world_map_110m.cpp \
    $(MATPLOT_DIR)/matplot/axes_objects/bars.cpp \
    $(MATPLOT_DIR)/matplot/axes_objects/box_chart.cpp \
    $(MATPLOT_DIR)/matplot/axes_objects/circles.cpp \
    $(MATPLOT_DIR)/matplot/axes_objects/contours.cpp \
    $(MATPLOT_DIR)/matplot/axes_objects/error_bar.cpp \
    $(MATPLOT_DIR)/matplot/axes_objects/filled_area.cpp \
    $(MATPLOT_DIR)/matplot/axes_objects/function_line.cpp \
    $(MATPLOT_DIR)/matplot/axes_objects/histogram.cpp \
    $(MATPLOT_DIR)/matplot/axes_objects/labels.cpp \
    $(MATPLOT_DIR)/matplot/axes_objects/line.cpp \
    $(MATPLOT_DIR)/matplot/axes_objects/matrix.cpp \
    $(MATPLOT_DIR)/matplot/axes_objects/network.cpp \
    $(MATPLOT_DIR)/matplot/axes_objects/parallel_lines.cpp \
    $(MATPLOT_DIR)/matplot/axes_objects/stair.cpp \
    $(MATPLOT_DIR)/matplot/axes_objects/string_function.cpp \
    $(MATPLOT_DIR)/matplot/axes_objects/surface.cpp \
    $(MATPLOT_DIR)/matplot/axes_objects/vectors.cpp \
    $(MATPLOT_DIR)/matplot/freestanding/axes_functions.cpp \
    $(MATPLOT_DIR)/matplot/freestanding/histcounts.cpp \
    $(MATPLOT_DIR)/3rd_party/nodesoup/src/algebra.cpp \
    $(MATPLOT_DIR)/3rd_party/nodesoup/src/fruchterman_reingold.cpp \
    $(MATPLOT_DIR)/3rd_party/nodesoup/src/kamada_kawai.cpp \
    $(MATPLOT_DIR)/3rd_party/nodesoup/src/layout.cpp \
    $(MATPLOT_DIR)/3rd_party/nodesoup/src/nodesoup.cpp

SRCS := main.cpp arduino_stubs.cpp $(MATPLOT_SRCS)
OBJS := $(SRCS:.cpp=.o) flight_computer.o

all: $(APP)

$(APP): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) $(LDFLAGS) -o $@

main.o: main.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

arduino_stubs.o: arduino_stubs.cpp include/Arduino.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

flight_computer.o: ../cm5/src/flight_computer.cpp ../cm5/src/flight_computer.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

.PHONY: clean

clean:
	rm -f $(APP) $(OBJS)
