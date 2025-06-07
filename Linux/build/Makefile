###############################################################
#
# Purpose: Makefile for "DARwIn Linux Framework"
# Author.: robotis
# Version: 0.2
# License: GPL
#
###############################################################

CC = g++
AR = ar
ARFLAGS = cr

TARGET = darwin.a

# --- Add the ONNX Runtime include path here ---
INCLUDE_DIRS = -I../include -I../../Framework/include -I/home/darwin/onnxruntime/include

# CXXFLAGS will now inherit the correct include path
CXXFLAGS += -O2 -DLINUX -Wall -shared $(INCLUDE_DIRS)
#CXXFLAGS += -O2 -DDEBUG -DLINUX -Wall -shared $(INCLUDE_DIRS)

# --- Add the ONNX Runtime library paths and libraries here ---
# Even though this Makefile doesn't use LFLAGS, the variable will be passed to
# child Makefiles, like the one for your webots_tcp_server.
# The "-Wl,-rpath" part is important.
LFLAGS += -lpthread -ldl -L/home/darwin/onnxruntime/build/Linux/Release -Wl,-rpath=/home/darwin/onnxruntime/build/Linux/Release -lonnxruntime

OBJS =  ../../Framework/src/MX28.o      \
        ../../Framework/src/CM730.o             \
         ../../Framework/src/Globals.o          \
        ../../Framework/src/math/Matrix.o   \
        ../../Framework/src/math/Plane.o    \
        ../../Framework/src/math/Point.o    \
        ../../Framework/src/math/Vector.o   \
        ../../Framework/src/motion/JointData.o          \
        ../../Framework/src/motion/Kinematics.o         \
        ../../Framework/src/motion/MotionManager.o  \
        ../../Framework/src/motion/MotionStatus.o   \
        ../../Framework/src/motion/modules/Action.o \
        ../../Framework/src/motion/modules/Head.o   \
        ../../Framework/src/motion/modules/Walking.o\
        ../../Framework/src/motion/modules/LeftArmController.o\
        ../../Framework/src/motion/modules/RightArmController.o\
        ../../Framework/src/motion/modules/LegsController.o\
        ../../Framework/src/vision/BallFollower.o   \
        ../../Framework/src/vision/BallTracker.o    \
        ../../Framework/src/vision/ColorFinder.o    \
        ../../Framework/src/vision/Image.o              \
        ../../Framework/src/vision/ImgProcess.o         \
        ../../Framework/src/vision/Camera.o     \
        ../../Framework/src/vision/HeadTracking.o \
        ../../Framework/src/minIni/minIni.o     \
        ../../Framework/src/gripper/GripperControl.o \
        streamer/httpd.o           \
        streamer/jpeg_utils.o      \
        streamer/mjpg_streamer.o   \
        LinuxActionScript.o   \
        LinuxCamera.o   \
        LinuxCM730.o    \
        LinuxMotionTimer.o    \
        LinuxNetwork.o

$(TARGET): $(OBJS)
        $(AR) $(ARFLAGS) ../lib/$(TARGET) $(OBJS)
#       $(CC) -o $(TARGET) $(OBJS) $(LFLAGS)
all: $(TARGET)
clean:
        rm -f $(OBJS) ../lib/$(TARGET)