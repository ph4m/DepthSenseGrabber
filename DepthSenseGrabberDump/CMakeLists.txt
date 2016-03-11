cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(DepthSenseGrabberDump)

add_executable (DepthSenseGrabberDump
	DepthSenseGrabberDump.cxx
	DepthSenseGrabberDump.hxx
	../shared/ConversionTools.cxx
	../shared/ConversionTools.hxx
	../shared/AcquisitionParameters.hxx
    ../shared/Frame.cxx
	../shared/Frame.hxx
)

target_link_libraries (DepthSenseGrabberDump ${DEPTHSENSE_LIBRARIES} ${Boost_LIBRARIES})

add_executable (DepthSenseGrabberPostProcess
	DepthSenseGrabberPostProcess.cxx
	DepthSenseGrabberPostProcess.hxx
	../shared/ConversionTools.cxx
	../shared/ConversionTools.hxx
	../shared/AcquisitionParameters.hxx
    ../shared/Frame.cxx
	../shared/Frame.hxx
)

target_link_libraries (DepthSenseGrabberPostProcess
                       ${DEPTHSENSE_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

