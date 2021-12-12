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
	if (!config_file.empty())
		std::cerr << "    config file: " << config_file << std::endl;
	std::cerr << "    info_text:" << info_text << std::endl;
	std::cerr << "    timeout: " << timeout << std::endl;
	std::cerr << "    width: " << width << std::endl;
	std::cerr << "    height: " << height << std::endl;
	std::cerr << "    post_process_file: " << post_process_file << std::endl;
	std::cerr << "    rawfull: " << rawfull << std::endl;
	if (nopreview)
		std::cerr << "    preview: none" << std::endl;
	else if (fullscreen)
		std::cerr << "    preview: fullscreen" << std::endl;
	else if (preview_width == 0 || preview_height == 0)
		std::cerr << "    preview: default" << std::endl;
	else
		std::cerr << "    preview: " << preview_x << "," << preview_y << "," << preview_width << ","
					<< preview_height << std::endl;
	std::cerr << "    qt-preview: " << qt_preview << std::endl;
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
	std::cerr << "    flush: " << (flush ? "true" : "false") << std::endl;
	std::cerr << "    wrap: " << wrap << std::endl;
	std::cerr << "    brightness: " << brightness << std::endl;
	std::cerr << "    contrast: " << contrast << std::endl;
	std::cerr << "    saturation: " << saturation << std::endl;
	std::cerr << "    sharpness: " << sharpness << std::endl;
	std::cerr << "    framerate: " << framerate << std::endl;
	std::cerr << "    denoise: " << denoise << std::endl;
	std::cerr << "    viewfinder-width: " << viewfinder_width << std::endl;
	std::cerr << "    viewfinder-height: " << viewfinder_height << std::endl;
	std::cerr << "    tuning-file: " << (tuning_file == "-" ? "(libcamera)" : tuning_file) << std::endl;
	std::cerr << "    lores-width: " << lores_width << std::endl;
	std::cerr << "    lores-height: " << lores_height << std::endl;
}
