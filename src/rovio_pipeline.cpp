/*
 * Copyright (c) 2014, Autonomous Systems Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <memory>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

#pragma GCC diagnostic pop

#include "rovio/RovioFilter.hpp"
#include "rovio/RovioNode_sensors.hpp"
#include "rovio/Struct_definition.hpp"
#ifdef MAKE_SCENE
#include "rovio/RovioScene.hpp"
#endif

using namespace std;

#ifdef ROVIO_NMAXFEATURE
static constexpr int nMax_ = ROVIO_NMAXFEATURE;
#else
static constexpr int nMax_ = 30; // Maximal number of considered features in the filter state.
#endif

#ifdef ROVIO_NLEVELS
static constexpr int nLevels_ = ROVIO_NLEVELS;
#else
static constexpr int nLevels_ = 4; // // Total number of pyramid levels considered.
#endif

#ifdef ROVIO_PATCHSIZE
static constexpr int patchSize_ = ROVIO_PATCHSIZE;
#else
static constexpr int patchSize_ = 6; // Edge length of the patches (in pixel). Must be a multiple of 2!
#endif

#ifdef ROVIO_NCAM
static constexpr int nCam_ = ROVIO_NCAM;
#else
static constexpr int nCam_ = 1; // Used total number of cameras.
#endif

#ifdef ROVIO_NPOSE
static constexpr int nPose_ = ROVIO_NPOSE;
#else
static constexpr int nPose_ = 0; // Additional pose states.
#endif

typedef rovio::RovioFilter<
		rovio::FilterState<nMax_, nLevels_, patchSize_, nCam_, nPose_>> mtFilter;

#ifdef MAKE_SCENE
static rovio::RovioScene<mtFilter> mRovioScene;

void idleFunc() {
	cv::waitKey(1);
	mRovioScene.drawScene(mRovioScene.mpFilter_->safe_);
}
#endif

int main(int argc, char** argv) {
	std::string filter_config = "./cfg/rovio.info";
	// Filter
	std::shared_ptr<mtFilter> mpFilter(new mtFilter);
	mpFilter->readFromInfo(filter_config);
	std::string camera_config = "./cfg/euroc_cam0.yaml";
	mpFilter->cameraCalibrationFile_[0] = camera_config;
	mpFilter->refreshProperties();
	rovio::RovioNode_sensors<mtFilter> rovioNode(argc, argv, mpFilter);

#ifdef MAKE_SCENE
	// Scene
	std::string mVSFileName = "./shaders/shader.vs";
	std::string mFSFileName = "./shaders/shader.fs";
	mRovioScene.initScene(argc,argv,mVSFileName,mFSFileName,mpFilter);
	mRovioScene.setIdleFunction(idleFunc);
	//mRovioScene.addKeyboardCB('r',[&rovioNode]() mutable {rovioNode.requestReset();});
	glutMainLoop();
#endif

	return 0;
}
