/****************************************************************************
 *
 *   Copyright (c) 2018-2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/conversion/rotation.h>
#include <px4_platform_common/module_params.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_gyro_fifo.h>

class PX4Gyroscope : public ModuleParams
{
public:
	PX4Gyroscope(uint32_t device_id, ORB_PRIO priority = ORB_PRIO_DEFAULT, enum Rotation rotation = ROTATION_NONE);
	~PX4Gyroscope() override;

	uint32_t get_device_id() const { return _device_id; }

	float get_max_rate_hz() const { return _param_imu_gyro_rate_max.get(); }

	void set_device_id(uint32_t device_id) { _device_id = device_id; }
	void set_device_type(uint8_t devtype);
	void set_error_count(uint64_t error_count) { _error_count = error_count; }
	void increase_error_count() { _error_count++; }
	void set_range(float range) { _range = range; }
	void set_scale(float scale) { _scale = scale; }
	void set_temperature(float temperature) { _temperature = temperature; }

	void update(const hrt_abstime &timestamp_sample, float x, float y, float z);

	void updateFIFO(sensor_gyro_fifo_s &sample);

private:
	void Publish(const hrt_abstime &timestamp_sample, float x, float y, float z);

	uORB::PublicationQueuedMulti<sensor_gyro_s> _sensor_pub;
	uORB::PublicationMulti<sensor_gyro_fifo_s>  _sensor_fifo_pub;

	uint32_t		_device_id{0};
	const enum Rotation	_rotation;

	float			_range{math::radians(2000.f)};
	float			_scale{1.f};
	float			_temperature{NAN};

	uint32_t		_error_count{0};

	int16_t			_last_sample[3] {};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::IMU_GYRO_RATEMAX>) _param_imu_gyro_rate_max
	)
};