################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/KltWithCov.cpp \
../src/dq2omega.cpp \
../src/featureManager.cpp \
../src/img2pcd.cpp \
../src/imgFtRelated.cpp \
../src/input.cpp \
../src/keypointsManager.cpp \
../src/kltUncertainty.cpp \
../src/main.cpp \
../src/pcVIsual.cpp \
../src/pclStereo.cpp \
../src/plotData.cpp \
../src/poseEKF.cpp \
../src/ppTransCov.cpp \
../src/ppTransEst.cpp \
../src/stereoSolver.cpp \
../src/utility.cpp 

OBJS += \
./src/KltWithCov.o \
./src/dq2omega.o \
./src/featureManager.o \
./src/img2pcd.o \
./src/imgFtRelated.o \
./src/input.o \
./src/keypointsManager.o \
./src/kltUncertainty.o \
./src/main.o \
./src/pcVIsual.o \
./src/pclStereo.o \
./src/plotData.o \
./src/poseEKF.o \
./src/ppTransCov.o \
./src/ppTransEst.o \
./src/stereoSolver.o \
./src/utility.o 

CPP_DEPS += \
./src/KltWithCov.d \
./src/dq2omega.d \
./src/featureManager.d \
./src/img2pcd.d \
./src/imgFtRelated.d \
./src/input.d \
./src/keypointsManager.d \
./src/kltUncertainty.d \
./src/main.d \
./src/pcVIsual.d \
./src/pclStereo.d \
./src/plotData.d \
./src/poseEKF.d \
./src/ppTransCov.d \
./src/ppTransEst.d \
./src/stereoSolver.d \
./src/utility.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -D__GXX_EXPERIMENTAL_CXX0X__ -I/usr/local/include/vtk-5.10 -I/home/xwong/Downloads/gnuIoStream/gnuplot-iostream -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


