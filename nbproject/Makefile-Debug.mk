#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-MacOSX
CND_DLIB_EXT=dylib
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/Classifier.o \
	${OBJECTDIR}/Constants.o \
	${OBJECTDIR}/LucasKanade.o \
	${OBJECTDIR}/Watcher.o \
	${OBJECTDIR}/helper.o \
	${OBJECTDIR}/main.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=-lopencv_videostab -lopencv_videostab.2.4 -lopencv_videostab.2.4.3 -lopencv_video -lopencv_video.2.4 -lopencv_video.2.4.3 -lopencv_ts -lopencv_ts.2.4 -lopencv_ts.2.4.3 -lopencv_stitching -lopencv_stitching.2.4 -lopencv_stitching.2.4.3 -lopencv_photo -lopencv_photo.2.4 -lopencv_photo.2.4.3 -lopencv_objdetect -lopencv_objdetect.2.4 -lopencv_objdetect.2.4.3 -lopencv_nonfree -lopencv_nonfree.2.4 -lopencv_nonfree.2.4.3 -lopencv_ml -lopencv_ml.2.4 -lopencv_ml.2.4.3 -lopencv_legacy -lopencv_legacy.2.4 -lopencv_legacy.2.4.3 -lopencv_imgproc -lopencv_imgproc.2.4 -lopencv_imgproc.2.4.3 -lopencv_highgui -lopencv_highgui.2.4 -lopencv_highgui.2.4.3 -lopencv_gpu -lopencv_gpu.2.4 -lopencv_gpu.2.4.3 -lopencv_flann -lopencv_flann.2.4 -lopencv_flann.2.4.3 -lopencv_features2d -lopencv_features2d.2.4 -lopencv_features2d.2.4.3 -lopencv_core -lopencv_core.2.4 -lopencv_core.2.4.3 -lopencv_contrib -lopencv_contrib.2.4 -lopencv_contrib.2.4.3 -lopencv_calib3d -lopencv_calib3d.2.4 -lopencv_calib3d.2.4.3

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/watcher

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/watcher: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/watcher ${OBJECTFILES} ${LDLIBSOPTIONS}

${OBJECTDIR}/Classifier.o: Classifier.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -I/usr/local/include/opencv2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/Classifier.o Classifier.cpp

${OBJECTDIR}/Constants.o: Constants.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -I/usr/local/include/opencv2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/Constants.o Constants.cpp

${OBJECTDIR}/LucasKanade.o: LucasKanade.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -I/usr/local/include/opencv2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/LucasKanade.o LucasKanade.cpp

${OBJECTDIR}/Watcher.o: Watcher.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -I/usr/local/include/opencv2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/Watcher.o Watcher.cpp

${OBJECTDIR}/helper.o: helper.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -I/usr/local/include/opencv2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/helper.o helper.cpp

${OBJECTDIR}/main.o: main.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -I/usr/local/include/opencv2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/main.o main.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/watcher

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
