/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * options.cpp - common program options helpers
 */
#include "libcamera_app_options.hpp"

void Options::Print() const
{
	std::cerr << "Options:" << std::endl;
	std::cerr << "    verbose: " << verbose << std::endl;
	std::cerr << "    info_text:" << info_text << std::endl;
	std::cerr << "    timeout: " << timeout << std::endl;
    std::cerr << "    photo resolution: " << photo_width << " x "<< photo_height << std::endl;
    std::cerr << "    video resolution: " << video_width << " x " << video_height << std::endl;
	std::cerr << "    rawfull: " << rawfull << std::endl;
	std::cerr << "    transform: " << transformToString(transform) << std::endl;
	if (roi_width == 0 || roi_height == 0)
		std::cerr << "    roi: all" << std::endl;
	else
		std::cerr << "    roi: " << roi_x << "," << roi_y << "," << roi_width << "," << roi_height << std::endl;
	if (shutter)
		std::cerr << "    shutter: " << shutter << std::endl;
	if (gain)
		std::cerr << "    gain: " << gain << std::endl;
	std::cerr << "    metering: " << metering_index << std::endl;
	std::cerr << "    exposure: " << exposure_index << std::endl;
	std::cerr << "    ev: " << ev << std::endl;
	std::cerr << "    awb: " << awb_index << std::endl;
	if (awb_gain_r && awb_gain_b)
		std::cerr << "    awb gains: red " << awb_gain_r << " blue " << awb_gain_b << std::endl;
	std::cerr << "    brightness: " << brightness << std::endl;
	std::cerr << "    contrast: " << contrast << std::endl;
	std::cerr << "    saturation: " << saturation << std::endl;
	std::cerr << "    sharpness: " << sharpness << std::endl;
	std::cerr << "    framerate: " << framerate << std::endl;
	std::cerr << "    denoise: " << denoise << std::endl;
}
