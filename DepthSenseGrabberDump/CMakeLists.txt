cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(DepthSenseGrabberDump)

#add_library (DepthSenseGrabberDump SHARED
add_executable (DepthSenseGrabberDump
	DepthSenseGrabberDump.cxx
	DepthSenseGrabberDump.hxx
	../shared/ConversionTools.cxx
	../shared/ConversionTools.hxx
	../shared/AcquisitionParameters.hxx
    ../shared/FrameColor.cxx
	../shared/FrameColor.hxx
)

target_link_libraries (DepthSenseGrabberDump ${DEPTHSENSE_LIBRARIES} ${Boost_LIBRARIES})

