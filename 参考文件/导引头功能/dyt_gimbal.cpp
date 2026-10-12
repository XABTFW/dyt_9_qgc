/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "DytPixelLos.hpp"

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/dyt_command.h>
#include <uORB/topics/dyt_status_reply.h>
#include <uORB/topics/dyt_target.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sdm50_status.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class DytGimbal : public ModuleBase<DytGimbal>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	explicit DytGimbal(const char *device_path);
	~DytGimbal() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;
	void show_status();

private:
	static constexpr uint8_t SYNC_1{0x55};
	static constexpr uint8_t SYNC_2{0xAA};
	static constexpr size_t MAX_FRAME_LEN{128};
	static constexpr uint8_t FRAME_ID_MODE{0x00};
	static constexpr uint8_t FRAME_ID_VISIBLE{0x01};
	static constexpr uint8_t FRAME_ID_INFRARED{0x02};
	static constexpr uint8_t FRAME_ID_LASER{0x03};
	static constexpr uint8_t FRAME_ID_TRACKER{0x04};
	static constexpr uint8_t FRAME_ID_IMU{0x06};
	static constexpr uint8_t FRAME_ID_FLIGHT_DATA{0x07};
	static constexpr uint8_t MODE_FRAME_DATA_LEN{0x1D};
	static constexpr size_t MODE_FRAME_LEN{33};
	static constexpr uint8_t TRACKER_FRAME_DATA_LEN{0x0A};
	static constexpr size_t TRACKER_FRAME_LEN{14};
	static constexpr uint8_t LASER_FRAME_DATA_LEN{0x0A};
	static constexpr size_t LASER_FRAME_LEN{14};
	static constexpr size_t FLIGHT_DATA_FRAME_LEN{42};
	static constexpr hrt_abstime LASER_FRESHNESS{500_ms};

	static constexpr uint8_t MODE_DISABLE{0x00};
	static constexpr uint8_t MODE_HOME{0x02};
	static constexpr uint8_t MODE_FOLLOW_ANGLE{0x03};
	static constexpr uint8_t MODE_LOCK{0x04};
	static constexpr uint8_t MODE_SCAN{0x05};
	static constexpr uint8_t MODE_TRACK{0x06};
	static constexpr uint8_t MODE_GEO_FOLLOW{0x07};
	static constexpr uint8_t TRACKER_TARGET_TYPE{0x21};
	static constexpr uint8_t TRACKER_AI_ENABLE{0x22};
	static constexpr uint8_t TRACKER_AI_DISABLE{0x23};
	static constexpr uint8_t TRACKER_ASSIST_ENABLE{0x24};
	static constexpr uint8_t TRACKER_ASSIST_DISABLE{0x25};
	static constexpr uint8_t TRACKER_TARGET_ID{0x26};
	static constexpr uint8_t TRACKER_IMAGE_MODE{0x37};
	static constexpr uint16_t IMAGE_MODE_VISIBLE{0};
	static constexpr uint16_t IMAGE_MODE_INFRARED{1};
	static constexpr uint8_t IMAGE_STATUS_VISIBLE{0x01};
	static constexpr uint8_t IMAGE_STATUS_INFRARED{0x02};
	static constexpr uint16_t RECOGNITION_TARGET_DETECTED{100};
	static constexpr hrt_abstime IMAGE_SOURCE_CONFIRM_TIMEOUT{1_s};
	static constexpr uint8_t TRACKER_OSD_ENABLE{0x07};
	static constexpr uint8_t TRACKER_OSD_DISABLE{0x08};
	static constexpr uint8_t TRACKER_DETECTION_START{0x2B};
	static constexpr uint8_t LASER_POWER_ON{0x01};
	static constexpr uint8_t LASER_POWER_OFF{0x02};
	static constexpr uint8_t LASER_CONTINUOUS_ON{0x05};
	static constexpr uint8_t LASER_CONTINUOUS_OFF{0x06};

	void Run() override;
	bool open_serial();
	void close_serial();
	int configure_serial(int fd, int baud);
	speed_t baud_to_speed(int baud) const;
	void update_params_if_needed();
	void update_arming_state();

	void read_serial();
	void process_byte(uint8_t byte);
	void reset_parser();
	bool validate_frame(const uint8_t *frame, size_t frame_len) const;
	void handle_frame(const uint8_t *frame, size_t frame_len, hrt_abstime now);
	void handle_servo_status(const uint8_t *frame, size_t frame_len, hrt_abstime now);
	void handle_payload_status(const uint8_t *frame, size_t frame_len, hrt_abstime now);
	void handle_laser_status(const uint8_t *frame, size_t frame_len, hrt_abstime now);
	void publish_generic_reply(const uint8_t *frame, size_t frame_len, hrt_abstime now);
	void publish_link_state(hrt_abstime now, uint8_t tracking_state);
	void maybe_log_target(const dyt_target_s &target, uint8_t raw_tracking_state);
	void maybe_log_raw_frame(const char *label, const uint8_t *frame, size_t frame_len);

	void handle_command_updates();
	void send_protocol_command(const dyt_command_s &cmd);
	bool send_mode_once();
	void set_mode(uint8_t control);
	void set_angle_mode(float yaw_deg, float pitch_deg);
	void set_tracking_mode(int16_t x_px, int16_t y_px);
	void set_scan_mode(const dyt_command_s &cmd);
	void set_geo_mode(double lat_deg, double lon_deg, float alt_m);
	bool send_tracker_command(uint8_t control, uint32_t value = 0);
	bool send_tracker_u16_command(uint8_t control, uint16_t value);
	bool send_laser_command(uint8_t control);
	bool send_flight_data(const dyt_command_s &cmd);
	bool write_frame(const uint8_t *buffer, size_t buffer_len);
	bool send_detection_start_if_needed();
	void send_startup_home_if_needed(hrt_abstime now);

	void publish_shell_command(uint8_t command);
	void publish_shell_angle_command(float yaw_deg, float pitch_deg);
	void publish_shell_track_point_command(int16_t x_px, int16_t y_px);
	void publish_shell_value_command(uint8_t command, uint32_t value);
	static bool parse_float_arg(const char *arg, float &value);
	static bool parse_int32_arg(const char *arg, int32_t &value, int32_t min_value, int32_t max_value);
	static bool parse_uint32_arg(const char *arg, uint32_t &value);

	static uint8_t checksum8(const uint8_t *buffer, size_t checksum_index);
	static uint16_t read_be_u16(const uint8_t *buffer, size_t index);
	static int16_t read_be_s16(const uint8_t *buffer, size_t index);
	static float read_be_float(const uint8_t *buffer, size_t index);
	static void put_be_u16(uint8_t *buffer, size_t index, uint16_t value);
	static void put_be_u32(uint8_t *buffer, size_t index, uint32_t value);
	static void put_be_float(uint8_t *buffer, size_t index, float value);
	static int16_t angle_deg_to_cdeg(float angle_deg);

	int _uart_fd{-1};
	char _device_path[32]{};
	uint8_t _rx_frame[MAX_FRAME_LEN]{};
	size_t _rx_index{0};
	size_t _expected_frame_len{0};

	hrt_abstime _last_open_attempt{0};
	hrt_abstime _last_servo_time{0};
	hrt_abstime _last_state_publish{0};
	hrt_abstime _last_target_log_time{0};
	hrt_abstime _startup_home_time{0};
	hrt_abstime _last_laser_time{0};
	hrt_abstime _tracking_mode_guard_until{0};

	uint64_t _rx_byte_count{0};
	uint32_t _frame_counter{0};
	uint32_t _frame_id_count[8]{};
	uint32_t _command_tx_count{0};
	uint16_t _parse_error_count{0};
	uint16_t _read_error_count{0};
	uint16_t _write_error_count{0};
	int _last_write_errno{0};
	int _last_write_result{0};
	uint8_t _last_command{dyt_command_s::CMD_NONE};
	uint8_t _last_raw_tracking_state{0};
	uint8_t _mode_control{MODE_DISABLE};
	uint8_t _mode_selection{0};
	uint8_t _mode_params[20]{};
	float _visible_zoom{NAN};
	float _infrared_zoom{NAN};
	float _visible_horizontal_fov_deg{NAN};
	float _infrared_horizontal_fov_deg{NAN};
	float _last_range_m{NAN};
	PX4Rangefinder _sdm50_rangefinder{0, distance_sensor_s::ROTATION_FORWARD_FACING};
	uint32_t _sdm50_device_id{0};
	hrt_abstime _last_velocity_sample{0};
	float _last_velocity_distance_m{NAN};
	float _closing_speed_m_s{NAN};
	bool _startup_home_sent{false};
	bool _armed{false};
	bool _detection_start_pending{false};
	size_t _detection_start_tx_offset{0};
	bool _image_source_confirmation_pending{false};
	bool _image_source_confirmation_warned{false};
	uint8_t _requested_image_status{0};
	hrt_abstime _image_source_request_time{0};

	dyt_target_s _last_target{};

	uORB::Subscription _dyt_command_sub{ORB_ID(dyt_command)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Publication<dyt_command_s> _dyt_command_pub{ORB_ID(dyt_command)};
	uORB::Publication<dyt_status_reply_s> _dyt_status_reply_pub{ORB_ID(dyt_status_reply)};
	uORB::Publication<dyt_target_s> _dyt_target_pub{ORB_ID(dyt_target)};
	uORB::Publication<sdm50_status_s> _sdm50_status_pub{ORB_ID(sdm50_status)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DYT_BAUD>) _param_dyt_baud,
		(ParamInt<px4::params::DYT_TO_MS>) _param_dyt_timeout_ms,
		(ParamInt<px4::params::DYT_RTRY_MS>) _param_dyt_retry_ms,
		(ParamInt<px4::params::DYT_LOG_MS>) _param_dyt_log_ms,
		(ParamInt<px4::params::DYT_RAWLOG>) _param_dyt_rawlog,
		(ParamInt<px4::params::DYT_HOME_EN>) _param_dyt_home_en,
		(ParamInt<px4::params::DYT_HOME_DLY>) _param_dyt_home_delay_ms,
		(ParamFloat<px4::params::DYT_HOME_YAW>) _param_dyt_home_yaw_deg,
		(ParamFloat<px4::params::DYT_HOME_PIT>) _param_dyt_home_pitch_deg,
		(ParamFloat<px4::params::DYT_LOS_SC>) _param_dyt_los_scale_deg,
		(ParamInt<px4::params::DYT_VIS_W>) _param_dyt_visible_width_px,
		(ParamInt<px4::params::DYT_VIS_H>) _param_dyt_visible_height_px,
		(ParamInt<px4::params::DYT_IR_W>) _param_dyt_infrared_width_px,
		(ParamInt<px4::params::DYT_IR_H>) _param_dyt_infrared_height_px,
		(ParamInt<px4::params::DYT_TRK_VAL>) _param_dyt_tracking_value,
		(ParamInt<px4::params::DYTG_LOCK_MS>) _param_dyt_lock_hold_ms
	)
};

DytGimbal::DytGimbal(const char *device_path) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	strncpy(_device_path, device_path, sizeof(_device_path) - 1);
	_device_path[sizeof(_device_path) - 1] = '\0';
	_last_target.range_m = NAN;
	_last_target.bbox_width_px = NAN;
	_last_target.bbox_height_px = NAN;
	device::Device::DeviceId device_id{};
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_SDM50;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;
	_sdm50_device_id = device_id.devid;
	_sdm50_rangefinder.set_device_id(_sdm50_device_id);
	_sdm50_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_LASER);
	_sdm50_rangefinder.set_min_distance(0.05f);
	_sdm50_rangefinder.set_max_distance(50.f);
	_sdm50_rangefinder.set_fov(math::radians(1.7f));
}

DytGimbal::~DytGimbal()
{
	close_serial();
}

bool DytGimbal::init()
{
	ScheduleOnInterval(5_ms);
	return true;
}

int DytGimbal::print_status()
{
	show_status();
	return 0;
}

void DytGimbal::show_status()
{
	PX4_INFO("protocol: Tweety V2.0.9.6 (0x55 0xAA)");
	PX4_INFO("SDM50: automatic laser reports on shared UART, range %.3f m, closing speed %.2f m/s",
		 static_cast<double>(_last_range_m), static_cast<double>(_closing_speed_m_s));
	PX4_INFO("port: %s fd: %d baud: %ld", _device_path, _uart_fd, static_cast<long>(_param_dyt_baud.get()));
	PX4_INFO("rx bytes: %llu frames: %lu parse/read/write errors: %u/%u/%u",
		 static_cast<unsigned long long>(_rx_byte_count), static_cast<unsigned long>(_frame_counter),
		 _parse_error_count, _read_error_count, _write_error_count);
	PX4_INFO("frames servo/vis/ir/laser/tracker/imu/flight: %lu/%lu/%lu/%lu/%lu/%lu/%lu",
		 static_cast<unsigned long>(_frame_id_count[0]), static_cast<unsigned long>(_frame_id_count[1]),
		 static_cast<unsigned long>(_frame_id_count[2]), static_cast<unsigned long>(_frame_id_count[3]),
		 static_cast<unsigned long>(_frame_id_count[4]), static_cast<unsigned long>(_frame_id_count[6]),
		 static_cast<unsigned long>(_frame_id_count[7]));
	PX4_INFO("last servo: %.3f s raw tracking: 0x%02x lock value: 0x%02lx manual LOS scale: %.6f deg/px",
		 static_cast<double>(_last_servo_time > 0 ? (hrt_absolute_time() - _last_servo_time) * 1e-6 : -1.0),
		 static_cast<unsigned>(_last_raw_tracking_state), static_cast<unsigned long>(_param_dyt_tracking_value.get()),
		 static_cast<double>(_param_dyt_los_scale_deg.get()));
	PX4_INFO("LOS cameras: VIS %.1f deg %ldx%ld IR %.1f deg %ldx%ld",
		 static_cast<double>(_visible_horizontal_fov_deg), static_cast<long>(_param_dyt_visible_width_px.get()),
		 static_cast<long>(_param_dyt_visible_height_px.get()), static_cast<double>(_infrared_horizontal_fov_deg),
		 static_cast<long>(_param_dyt_infrared_width_px.get()), static_cast<long>(_param_dyt_infrared_height_px.get()));
	PX4_INFO("image source: raw=0x%02x requested=0x%02x pending=%d zoom=%.1f",
		 static_cast<unsigned>(_last_target.status3), static_cast<unsigned>(_requested_image_status),
		 _image_source_confirmation_pending, static_cast<double>(_last_target.zoom_ratio));
	PX4_INFO("mode control: 0x%02x tx: %lu last command: %u write result/errno: %d/%d",
		 static_cast<unsigned>(_mode_control), static_cast<unsigned long>(_command_tx_count),
		 static_cast<unsigned>(_last_command), _last_write_result, _last_write_errno);
}

void DytGimbal::Run()
{
	if (should_exit()) {
		ScheduleClear();
		close_serial();
		exit_and_cleanup();
		return;
	}

	update_params_if_needed();
	update_arming_state();
	const hrt_abstime now = hrt_absolute_time();

	if (_uart_fd < 0) {
		const hrt_abstime retry_us = static_cast<hrt_abstime>(math::max(_param_dyt_retry_ms.get(), int32_t{100})) * 1000ULL;

		if (_last_open_attempt == 0 || now - _last_open_attempt >= retry_us) {
			_last_open_attempt = now;
			open_serial();
		}

		publish_link_state(now, dyt_target_s::TRACKING_STATE_TIMEOUT);
		return;
	}

	// This frame has priority over all other gimbal commands. If the non-blocking
	// UART accepts only part of it, resume it before issuing another frame.
	if (!send_detection_start_if_needed()) {
		return;
	}

	read_serial();
	const hrt_abstime now_after_read = hrt_absolute_time();
	handle_command_updates();
	send_startup_home_if_needed(now_after_read);

	const hrt_abstime timeout_us = static_cast<hrt_abstime>(math::max(_param_dyt_timeout_ms.get(), int32_t{50})) * 1000ULL;
	const bool servo_timed_out = _last_servo_time == 0 || now_after_read < _last_servo_time ||
				     now_after_read - _last_servo_time > timeout_us;

	if (servo_timed_out) {
		publish_link_state(now_after_read, dyt_target_s::TRACKING_STATE_TIMEOUT);
	}
}

void DytGimbal::update_arming_state()
{
	vehicle_status_s vehicle_status{};

	if (_vehicle_status_sub.update(&vehicle_status)) {
		const bool armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;

		if (armed && !_armed) {
			_detection_start_pending = true;
			_detection_start_tx_offset = 0;
		}

		_armed = armed;
	}
}

void DytGimbal::update_params_if_needed()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s update{};
		_parameter_update_sub.copy(&update);
		updateParams();

		if (_uart_fd >= 0) {
			configure_serial(_uart_fd, _param_dyt_baud.get());
		}
	}
}

bool DytGimbal::open_serial()
{
	_uart_fd = ::open(_device_path, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_uart_fd < 0) {
		PX4_WARN("open %s failed (%d)", _device_path, errno);
		return false;
	}

	if (configure_serial(_uart_fd, _param_dyt_baud.get()) != PX4_OK) {
		close_serial();
		return false;
	}

	reset_parser();
	_last_laser_time = 0;
	_last_range_m = NAN;
	_last_velocity_sample = 0;
	_last_velocity_distance_m = NAN;
	_closing_speed_m_s = NAN;
	_last_servo_time = 0;
	_visible_horizontal_fov_deg = NAN;
	_infrared_horizontal_fov_deg = NAN;
	_visible_zoom = NAN;
	_infrared_zoom = NAN;
	_mode_control = MODE_DISABLE;
	memset(_mode_params, 0, sizeof(_mode_params));
	_startup_home_sent = false;

	if (_armed) {
		// The gimbal or UART may have restarted in flight. Re-send the start
		// command after reconnect so target detection is restored.
		_detection_start_pending = true;
		_detection_start_tx_offset = 0;
	}

	_startup_home_time = hrt_absolute_time() +
		static_cast<hrt_abstime>(math::max(_param_dyt_home_delay_ms.get(), int32_t{0})) * 1000ULL;
	PX4_INFO("opened %s @ %ld", _device_path, static_cast<long>(_param_dyt_baud.get()));
	return true;
}

void DytGimbal::close_serial()
{
	if (_uart_fd >= 0) {
		::close(_uart_fd);
		_uart_fd = -1;
	}

	_image_source_confirmation_pending = false;
	_image_source_confirmation_warned = false;
	_requested_image_status = 0;
	_image_source_request_time = 0;
}

speed_t DytGimbal::baud_to_speed(int baud) const
{
	switch (baud) {
	case 9600: return B9600;
	case 19200: return B19200;
	case 38400: return B38400;
	case 57600: return B57600;
	case 115200: return B115200;
#ifdef B230400
	case 230400: return B230400;
#endif
#ifdef B460800
	case 460800: return B460800;
#endif
#ifdef B921600
	case 921600: return B921600;
#endif
	default: return 0;
	}
}

int DytGimbal::configure_serial(int fd, int baud)
{
	const speed_t speed = baud_to_speed(baud);

	if (speed == 0) {
		PX4_ERR("unsupported baud %d", baud);
		return PX4_ERROR;
	}

	struct termios config {};

	if (tcgetattr(fd, &config) < 0) {
		PX4_ERR("tcgetattr failed (%d)", errno);
		return PX4_ERROR;
	}

	cfmakeraw(&config);
	config.c_cflag |= CLOCAL | CREAD;
	config.c_cflag &= ~CSIZE;
	config.c_cflag |= CS8;
	config.c_cflag &= ~PARENB;
	config.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
	config.c_cflag &= ~CRTSCTS;
#endif
	config.c_cc[VMIN] = 0;
	config.c_cc[VTIME] = 0;

	if (cfsetispeed(&config, speed) < 0 || cfsetospeed(&config, speed) < 0 || tcsetattr(fd, TCSANOW, &config) < 0) {
		PX4_ERR("serial config failed (%d)", errno);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void DytGimbal::read_serial()
{
	uint8_t buffer[128]{};

	// Bound work on the shared queue even if the UART is continuously busy.
	for (unsigned reads = 0; reads < 8; ++reads) {
		const ssize_t nread = ::read(_uart_fd, buffer, sizeof(buffer));

		if (nread > 0) {
			_rx_byte_count += static_cast<uint64_t>(nread);

			for (ssize_t i = 0; i < nread; ++i) {
				process_byte(buffer[i]);
			}

		} else if (nread == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
			break;

		} else if (errno != EINTR) {
			++_read_error_count;
			close_serial();
			break;
		}
	}
}

void DytGimbal::reset_parser()
{
	_rx_index = 0;
	_expected_frame_len = 0;
}

void DytGimbal::process_byte(uint8_t byte)
{
	if (_rx_index == 0) {
		if (byte == SYNC_1) {
			_rx_frame[_rx_index++] = byte;
		}

		return;
	}

	if (_rx_index == 1) {
		if (byte == SYNC_2) {
			_rx_frame[_rx_index++] = byte;

		} else if (byte != SYNC_1) {
			reset_parser();
		}

		return;
	}

	if (_rx_index == 2) {
		const size_t total_length = static_cast<size_t>(byte) + 4;

		if (byte < 1 || total_length > MAX_FRAME_LEN) {
			++_parse_error_count;
			reset_parser();
			return;
		}

		_expected_frame_len = total_length;
	}

	_rx_frame[_rx_index++] = byte;

	if (_expected_frame_len > 0 && _rx_index == _expected_frame_len) {
		const hrt_abstime now = hrt_absolute_time();

		if (validate_frame(_rx_frame, _expected_frame_len)) {
			handle_frame(_rx_frame, _expected_frame_len, now);

		} else {
			++_parse_error_count;
		}

		reset_parser();
	}
}

bool DytGimbal::validate_frame(const uint8_t *frame, size_t frame_len) const
{
	return frame_len >= 5 && frame[0] == SYNC_1 && frame[1] == SYNC_2 &&
	       frame_len == static_cast<size_t>(frame[2]) + 4 && checksum8(frame, frame_len - 1) == frame[frame_len - 1];
}

void DytGimbal::handle_frame(const uint8_t *frame, size_t frame_len, hrt_abstime now)
{
	++_frame_counter;
	const uint8_t frame_id = frame[3];

	if (frame_id < sizeof(_frame_id_count) / sizeof(_frame_id_count[0])) {
		++_frame_id_count[frame_id];
	}

	maybe_log_raw_frame("DYT V2 rx", frame, frame_len);

	switch (frame_id) {
	case FRAME_ID_MODE:
		handle_servo_status(frame, frame_len, now);
		break;

	case FRAME_ID_VISIBLE:
	case FRAME_ID_INFRARED:
		handle_payload_status(frame, frame_len, now);
		break;

	case FRAME_ID_LASER:
		handle_laser_status(frame, frame_len, now);
		break;

	default:
		publish_generic_reply(frame, frame_len, now);
		break;
	}
}

void DytGimbal::handle_servo_status(const uint8_t *frame, size_t frame_len, hrt_abstime now)
{
	if (frame_len != 45 || frame[2] != 0x29) {
		++_parse_error_count;
		return;
	}

	dyt_target_s target{};
	target.timestamp = now;
	target.timestamp_sample = now;
	target.frame_counter = _frame_counter;
	target.parse_error_count = _parse_error_count;
	target.self_test_raw = frame[4];
	const uint16_t recognition_status = read_be_u16(frame, 36);
	const bool recognition_detected = recognition_status == RECOGNITION_TARGET_DETECTED;
	target.status1 = recognition_status <= UINT8_MAX ? static_cast<uint8_t>(recognition_status) : UINT8_MAX;
	target.status2 = frame[31];
	target.status3 = frame[5];
	const bool image_status_valid = frame[5] == IMAGE_STATUS_VISIBLE || frame[5] == IMAGE_STATUS_INFRARED;

	if (_image_source_confirmation_pending && frame[5] == _requested_image_status) {
		_image_source_confirmation_pending = false;
		_image_source_confirmation_warned = false;

	} else if (_image_source_confirmation_pending && !_image_source_confirmation_warned
		   && now >= _image_source_request_time
		   && now - _image_source_request_time > IMAGE_SOURCE_CONFIRM_TIMEOUT) {
		PX4_WARN("image source 0x%02x not confirmed, raw status 0x%02x",
			 static_cast<unsigned>(_requested_image_status), static_cast<unsigned>(frame[5]));
		_image_source_confirmation_warned = true;
	}

	const bool image_source_confirmed = image_status_valid && !_image_source_confirmation_pending;
	const bool infrared_source = frame[5] == IMAGE_STATUS_INFRARED;
	target.video_source = infrared_source ? dyt_target_s::VIDEO_SOURCE_IR_1 : dyt_target_s::VIDEO_SOURCE_VIS_1;
	target.zoom_ratio = image_source_confirmed ? (infrared_source ? _infrared_zoom : _visible_zoom) : NAN;

	target.gimbal_yaw_rate_rad_s = math::radians(read_be_float(frame, 7));
	target.gimbal_pitch_rate_rad_s = math::radians(read_be_float(frame, 11));
	target.gimbal_roll_rate_rad_s = math::radians(read_be_float(frame, 15));
	target.gimbal_yaw_rad = math::radians(read_be_float(frame, 19));
	target.gimbal_pitch_frame_rad = math::radians(read_be_float(frame, 23));
	target.gimbal_pitch_rad = target.gimbal_pitch_frame_rad;
	target.gimbal_roll_rad = math::radians(read_be_float(frame, 27));

	const int16_t raw_los_x = read_be_s16(frame, 32);
	const int16_t raw_los_y = read_be_s16(frame, 34);
	const float horizontal_fov_deg = image_source_confirmed ?
					 (infrared_source ? _infrared_horizontal_fov_deg : _visible_horizontal_fov_deg) : NAN;
	const int32_t image_width_px = infrared_source ? _param_dyt_infrared_width_px.get() :
				       _param_dyt_visible_width_px.get();
	const int32_t image_height_px = infrared_source ? _param_dyt_infrared_height_px.get() :
					_param_dyt_visible_height_px.get();
	const bool los_valid = dyt::pixelMissToLos(raw_los_x, raw_los_y, horizontal_fov_deg, image_width_px,
			       image_height_px, _param_dyt_los_scale_deg.get(), target.los_x_rad, target.los_y_rad);

	_last_raw_tracking_state = frame[31];
	const int32_t tracking_value = _param_dyt_tracking_value.get();
	const bool tracking_value_valid = tracking_value >= 0 && tracking_value <= UINT8_MAX;
	const bool tracking_raw_matches = tracking_value_valid && frame[31] == static_cast<uint8_t>(tracking_value);
	const bool lock_reported = frame[6] == MODE_TRACK && tracking_raw_matches;
	target.tracking_state = lock_reported ? dyt_target_s::TRACKING_STATE_LOCKED : dyt_target_s::TRACKING_STATE_SEARCH;
	// Report the payload lock independently, but only expose a guidance-valid target
	// when the pixel miss distance can be converted to a finite angular LOS.
	target.target_valid = lock_reported && image_source_confirmed && los_valid;
	target.auto_hint = recognition_detected;
	target.follow_mode = frame[6] == MODE_FOLLOW_ANGLE;
	target.motor_on = frame[6] != MODE_DISABLE;
	target.laser_on = _last_laser_time > 0 && now - _last_laser_time <= LASER_FRESHNESS && PX4_ISFINITE(_last_range_m);
	target.range_m = target.laser_on ? _last_range_m : NAN;
	const uint16_t bbox_width_px = read_be_u16(frame, 40);
	const uint16_t bbox_height_px = read_be_u16(frame, 42);
	const bool bbox_valid = (recognition_detected || lock_reported) && image_source_confirmed
				&& bbox_width_px > 0 && bbox_height_px > 0
				&& image_width_px > 0 && image_height_px > 0
				&& bbox_width_px <= static_cast<uint32_t>(image_width_px)
				&& bbox_height_px <= static_cast<uint32_t>(image_height_px);
	target.bbox_width_px = bbox_valid ? static_cast<float>(bbox_width_px) : NAN;
	target.bbox_height_px = bbox_valid ? static_cast<float>(bbox_height_px) : NAN;
	target.frame_dt_s = _last_servo_time > 0 ? (now - _last_servo_time) * 1e-6f : 0.f;
	target.last_rx_age_s = 0.f;

	const bool finite_attitude = PX4_ISFINITE(target.gimbal_yaw_rad) && PX4_ISFINITE(target.gimbal_pitch_frame_rad) &&
		PX4_ISFINITE(target.gimbal_roll_rad);

	if (!finite_attitude) {
		target.tracking_state = dyt_target_s::TRACKING_STATE_ERROR;
		target.target_valid = false;
		++_parse_error_count;
	}

	_last_servo_time = now;
	_last_state_publish = now;
	_last_target = target;
	maybe_log_target(target, frame[31]);
	_dyt_target_pub.publish(target);
}

void DytGimbal::handle_payload_status(const uint8_t *frame, size_t frame_len, hrt_abstime now)
{
	if (frame_len != 20 || frame[2] != 0x10) {
		++_parse_error_count;
		return;
	}

	const float horizontal_fov_deg = static_cast<float>(read_be_u16(frame, 5)) * 0.1f;
	const float zoom = static_cast<float>(read_be_u16(frame, 7)) * 0.1f;
	const float valid_horizontal_fov_deg = horizontal_fov_deg > 0.f && horizontal_fov_deg < 179.f ?
					       horizontal_fov_deg : NAN;

	if (frame[3] == FRAME_ID_VISIBLE) {
		_visible_horizontal_fov_deg = valid_horizontal_fov_deg;
		_visible_zoom = zoom;

	} else {
		_infrared_horizontal_fov_deg = valid_horizontal_fov_deg;
		_infrared_zoom = zoom;
	}

	publish_generic_reply(frame, frame_len, now);
}

void DytGimbal::handle_laser_status(const uint8_t *frame, size_t frame_len, hrt_abstime now)
{
	if (frame_len != 16 || frame[2] != 0x0C) {
		++_parse_error_count;
		return;
	}

	// Both automatic (0x00) and requested (0x01) range reports carry
	// the SDM50 distance in metres as a big-endian float at bytes 7..10.
	// Self-test/count/settings replies must not enter the range pipeline.
	if (frame[4] == 0x00 || frame[4] == 0x01) {
		const float range_m = read_be_float(frame, 7);
		const bool valid = PX4_ISFINITE(range_m) && range_m >= 0.05f && range_m < 50.f;

		if (valid) {
			// Preserve the standalone SDM50 closing-speed estimator: at most
			// 50 Hz differentiation and a 0.3 s low-pass time constant.
			bool update_baseline = _last_velocity_sample == 0 || !PX4_ISFINITE(_last_velocity_distance_m);

			if (!update_baseline) {
				const float dt_s = (now - _last_velocity_sample) * 1e-6f;

				if (dt_s >= 0.02f && dt_s <= 2.5f) {
					const float raw_speed = (_last_velocity_distance_m - range_m) / dt_s;

					if (PX4_ISFINITE(raw_speed) && fabsf(raw_speed) <= 50.f) {
						const float alpha = dt_s / (0.3f + dt_s);
						_closing_speed_m_s = PX4_ISFINITE(_closing_speed_m_s) ?
							_closing_speed_m_s + alpha * (raw_speed - _closing_speed_m_s) : raw_speed;
					}

					update_baseline = true;

				} else if (dt_s > 2.5f) {
					_closing_speed_m_s = NAN;
					update_baseline = true;
				}
			}

			if (update_baseline) {
				_last_velocity_sample = now;
				_last_velocity_distance_m = range_m;
			}

			_last_range_m = range_m;
			_last_laser_time = now;

		} else {
			_last_range_m = NAN;
			_last_laser_time = 0;
			_last_velocity_sample = 0;
			_last_velocity_distance_m = NAN;
			_closing_speed_m_s = NAN;
		}

		_sdm50_rangefinder.update(now, valid ? range_m : NAN, valid ? 100 : 0);
		sdm50_status_s status{};
		status.timestamp = now;
		status.timestamp_sample = now;
		status.device_id = _sdm50_device_id;
		status.distance_m = valid ? range_m : NAN;
		status.closing_speed_m_s = valid ? _closing_speed_m_s : NAN;
		status.valid = valid;
		_sdm50_status_pub.publish(status);
	}

	publish_generic_reply(frame, frame_len, now);
}

void DytGimbal::publish_generic_reply(const uint8_t *frame, size_t frame_len, hrt_abstime now)
{
	dyt_status_reply_s reply{};
	reply.timestamp = now;
	reply.timestamp_sample = now;
	reply.control_code = frame[3];
	const size_t payload_length = frame_len > 5 ? frame_len - 5 : 0;
	reply.param_length = static_cast<uint8_t>(math::min(payload_length, sizeof(reply.params)));
	reply.truncated = payload_length > sizeof(reply.params);
	reply.parse_error_count = _parse_error_count;

	for (size_t i = 0; i < reply.param_length; ++i) {
		reply.params[i] = frame[4 + i];
	}

	_dyt_status_reply_pub.publish(reply);
}

void DytGimbal::publish_link_state(hrt_abstime now, uint8_t tracking_state)
{
	if (_last_state_publish != 0 && now - _last_state_publish < 100_ms) {
		return;
	}

	dyt_target_s target = _last_target;
	target.timestamp = now;
	target.timestamp_sample = _last_servo_time;
	target.tracking_state = tracking_state;
	target.target_valid = false;
	target.frame_counter = _frame_counter;
	target.parse_error_count = _parse_error_count;
	target.last_rx_age_s = _last_servo_time > 0 && now >= _last_servo_time ?
			      (now - _last_servo_time) * 1e-6f : NAN;
	_dyt_target_pub.publish(target);
	_last_state_publish = now;
}

void DytGimbal::maybe_log_target(const dyt_target_s &target, uint8_t raw_tracking_state)
{
	const int32_t period_ms = _param_dyt_log_ms.get();

	if (period_ms <= 0 || (_last_target_log_time != 0 &&
	    target.timestamp - _last_target_log_time < static_cast<hrt_abstime>(period_ms) * 1000ULL)) {
		return;
	}

	_last_target_log_time = target.timestamp;
	PX4_INFO("DYT V2 state=%u raw=0x%02x valid=%u los=(%.3f,%.3f) deg bbox=(%.0f,%.0f) px gimbal=(%.2f,%.2f,%.2f) deg dt=%.3f",
		 static_cast<unsigned>(target.tracking_state), static_cast<unsigned>(raw_tracking_state),
		 static_cast<unsigned>(target.target_valid), static_cast<double>(math::degrees(target.los_x_rad)),
		 static_cast<double>(math::degrees(target.los_y_rad)), static_cast<double>(target.bbox_width_px),
		 static_cast<double>(target.bbox_height_px), static_cast<double>(math::degrees(target.gimbal_roll_rad)),
		 static_cast<double>(math::degrees(target.gimbal_pitch_frame_rad)),
		 static_cast<double>(math::degrees(target.gimbal_yaw_rad)), static_cast<double>(target.frame_dt_s));
}

void DytGimbal::maybe_log_raw_frame(const char *label, const uint8_t *frame, size_t frame_len)
{
	if (_param_dyt_rawlog.get() <= 0) {
		return;
	}

	char line[MAX_FRAME_LEN * 3 + 1]{};
	size_t offset = 0;

	for (size_t i = 0; i < frame_len && offset < sizeof(line); ++i) {
		const int written = snprintf(&line[offset], sizeof(line) - offset, "%02X%s", frame[i], i + 1 < frame_len ? " " : "");

		if (written <= 0 || static_cast<size_t>(written) >= sizeof(line) - offset) {
			break;
		}

		offset += static_cast<size_t>(written);
	}

	PX4_INFO("%s: %s", label, line);
}

void DytGimbal::handle_command_updates()
{
	dyt_command_s cmd{};

	while (_dyt_command_sub.update(&cmd)) {
		_startup_home_sent = true;
		send_protocol_command(cmd);
	}
}

void DytGimbal::send_protocol_command(const dyt_command_s &cmd)
{
	_last_command = cmd.command;
	bool send_mode = false;

	switch (cmd.command) {
	case dyt_command_s::CMD_AUTO_LOCK:
		set_tracking_mode(0, 0);
		send_mode = true;
		break;

	case dyt_command_s::CMD_STOP_TRACK:
		send_tracker_command(0x27);
		set_angle_mode(0.f, 0.f);
		send_mode = true;
		break;

	case dyt_command_s::CMD_RETRIGGER:
		send_tracker_command(0x27);
		send_tracker_command(TRACKER_ASSIST_ENABLE);
		set_tracking_mode(0, 0);
		send_mode = true;
		break;

	case dyt_command_s::CMD_NOFOLLOW:
	case dyt_command_s::CMD_LOCK_VIEW:
		set_mode(MODE_LOCK);
		send_mode = true;
		break;

	case dyt_command_s::CMD_YAW_FOLLOW:
		set_mode(MODE_FOLLOW_ANGLE);
		send_mode = true;
		break;

	case dyt_command_s::CMD_CENTER:
		set_mode(MODE_HOME);
		send_mode = true;
		break;

	case dyt_command_s::CMD_CENTER_GIMBAL:
		set_angle_mode(static_cast<float>(cmd.param_x) * 0.01f, static_cast<float>(cmd.param_y) * 0.01f);
		send_mode = true;
		break;

	case dyt_command_s::CMD_SET_FRAME_ANGLE:
		if (hrt_absolute_time() >= _tracking_mode_guard_until) {
			set_angle_mode(static_cast<float>(cmd.param_x) * 0.01f, static_cast<float>(cmd.param_y) * 0.01f);
			send_mode = true;
		}
		break;

	case dyt_command_s::CMD_SET_INERTIAL_ANGLE:
		// This gimbal protocol has no inertial-space attitude guidance command.
		break;

	case dyt_command_s::CMD_SEARCH_RATE:
		set_scan_mode(cmd);
		send_mode = true;
		break;

	case dyt_command_s::CMD_SEND_OWNSHIP_STATE:
		send_flight_data(cmd);
		break;

	case dyt_command_s::CMD_GEO_TRACK:
		if (hrt_absolute_time() >= _tracking_mode_guard_until) {
			set_geo_mode(cmd.lat, cmd.lon, cmd.alt);
			send_mode = true;
		}
		break;

	case dyt_command_s::CMD_GEO_TRACK_EXIT:
		if (hrt_absolute_time() >= _tracking_mode_guard_until) {
			set_mode(MODE_LOCK);
			send_mode = true;
		}
		break;

	case dyt_command_s::CMD_TRACK_POINT:
		set_tracking_mode(cmd.param_x, cmd.param_y);
		send_mode = true;
		break;

	case dyt_command_s::CMD_AI_ENABLE:
		send_tracker_command(TRACKER_AI_ENABLE);
		break;

	case dyt_command_s::CMD_AI_DISABLE:
		send_tracker_command(TRACKER_AI_DISABLE);
		break;

	case dyt_command_s::CMD_TARGET_TYPE:
		if (cmd.value >= 1 && cmd.value <= 3) {
			send_tracker_u16_command(TRACKER_TARGET_TYPE, static_cast<uint16_t>(cmd.value));
		}
		break;

	case dyt_command_s::CMD_ASSIST_ENABLE:
		send_tracker_command(TRACKER_ASSIST_ENABLE);
		break;

	case dyt_command_s::CMD_ASSIST_DISABLE:
		send_tracker_command(TRACKER_ASSIST_DISABLE);
		break;

	case dyt_command_s::CMD_TRACK_ID:
		send_tracker_command(TRACKER_TARGET_ID, cmd.value);
		break;

	case dyt_command_s::CMD_IMAGE_MODE: {
		if (cmd.value <= 9) {
			const uint16_t image_mode = static_cast<uint16_t>(cmd.value);

			if (send_tracker_u16_command(TRACKER_IMAGE_MODE, image_mode)) {
				if (image_mode == IMAGE_MODE_VISIBLE || image_mode == IMAGE_MODE_INFRARED) {
					_requested_image_status = image_mode == IMAGE_MODE_INFRARED ?
								  IMAGE_STATUS_INFRARED : IMAGE_STATUS_VISIBLE;
					_image_source_confirmation_pending = true;
					_image_source_confirmation_warned = false;
					_image_source_request_time = hrt_absolute_time();
				}
			}
		}
		break;
	}

	case dyt_command_s::CMD_LASER_ON:
		send_laser_command(LASER_POWER_ON);
		break;

	case dyt_command_s::CMD_LASER_CONTINUOUS:
		send_laser_command(LASER_CONTINUOUS_ON);
		break;

	case dyt_command_s::CMD_LASER_OFF:
		send_laser_command(LASER_CONTINUOUS_OFF);
		send_laser_command(LASER_POWER_OFF);
		break;

	case dyt_command_s::CMD_OSD_ENABLE:
		send_tracker_command(TRACKER_OSD_ENABLE);
		break;

	case dyt_command_s::CMD_OSD_DISABLE:
		send_tracker_command(TRACKER_OSD_DISABLE);
		break;

	default:
		break;
	}

	if (send_mode) {
		const bool mode_sent = send_mode_once();

		if (mode_sent && _mode_control == MODE_TRACK) {
			const int32_t lock_hold_ms = math::constrain(_param_dyt_lock_hold_ms.get(), int32_t{100}, int32_t{10000});
			_tracking_mode_guard_until = hrt_absolute_time() + static_cast<hrt_abstime>(lock_hold_ms) * 1000ULL;
		}
	}
}

void DytGimbal::set_mode(uint8_t control)
{
	_mode_control = control;
	_mode_selection = 0;
	memset(_mode_params, 0, sizeof(_mode_params));

	if (control != MODE_TRACK) {
		_tracking_mode_guard_until = 0;
	}
}

void DytGimbal::set_angle_mode(float yaw_deg, float pitch_deg)
{
	set_mode(MODE_FOLLOW_ANGLE);
	put_be_float(_mode_params, 0, math::constrain(yaw_deg, -180.f, 180.f));
	put_be_float(_mode_params, 4, math::constrain(pitch_deg, -90.f, 90.f));
}

void DytGimbal::set_tracking_mode(int16_t x_px, int16_t y_px)
{
	set_mode(MODE_TRACK);
	_mode_selection = 0x00;
	put_be_u16(_mode_params, 0, static_cast<uint16_t>(x_px));
	put_be_u16(_mode_params, 2, static_cast<uint16_t>(y_px));
}

void DytGimbal::set_scan_mode(const dyt_command_s &cmd)
{
	set_mode(MODE_SCAN);
	_mode_selection = 0x01;
	const float yaw_boundary = math::constrain(static_cast<float>(cmd.param_x) * 0.01f, -180.f, 180.f);
	const float pitch_boundary = math::constrain(static_cast<float>(cmd.param_y) * 0.01f, -90.f, 90.f);
	put_be_float(_mode_params, 0, -fabsf(yaw_boundary));
	put_be_float(_mode_params, 4, fabsf(yaw_boundary));
	put_be_float(_mode_params, 8, fabsf(pitch_boundary));
}

void DytGimbal::set_geo_mode(double lat_deg, double lon_deg, float alt_m)
{
	if (!PX4_ISFINITE(static_cast<float>(lat_deg)) || !PX4_ISFINITE(static_cast<float>(lon_deg)) || !PX4_ISFINITE(alt_m) ||
	    lat_deg < -90.0 || lat_deg > 90.0 || lon_deg < -180.0 || lon_deg > 180.0) {
		return;
	}

	set_mode(MODE_GEO_FOLLOW);
	put_be_float(_mode_params, 0, static_cast<float>(lon_deg));
	put_be_float(_mode_params, 4, static_cast<float>(lat_deg));
	put_be_u16(_mode_params, 8, static_cast<uint16_t>(math::constrain(static_cast<int>(roundf(alt_m)), -32768, 32767)));
}

bool DytGimbal::send_mode_once()
{
	if (_uart_fd < 0) {
		return false;
	}

	uint8_t frame[MODE_FRAME_LEN]{};
	frame[0] = SYNC_1;
	frame[1] = SYNC_2;
	frame[2] = MODE_FRAME_DATA_LEN;
	frame[3] = FRAME_ID_MODE;
	frame[4] = _mode_control;
	frame[11] = _mode_selection;
	memcpy(&frame[12], _mode_params, sizeof(_mode_params));
	frame[MODE_FRAME_LEN - 1] = checksum8(frame, MODE_FRAME_LEN - 1);
	return write_frame(frame, sizeof(frame));
}

bool DytGimbal::send_tracker_command(uint8_t control, uint32_t value)
{
	uint8_t frame[TRACKER_FRAME_LEN]{};
	frame[0] = SYNC_1;
	frame[1] = SYNC_2;
	frame[2] = TRACKER_FRAME_DATA_LEN;
	frame[3] = FRAME_ID_TRACKER;
	frame[4] = control;
	put_be_u32(frame, 5, value);
	frame[TRACKER_FRAME_LEN - 1] = checksum8(frame, TRACKER_FRAME_LEN - 1);
	return write_frame(frame, sizeof(frame));
}

bool DytGimbal::send_tracker_u16_command(uint8_t control, uint16_t value)
{
	uint8_t frame[TRACKER_FRAME_LEN]{};
	frame[0] = SYNC_1;
	frame[1] = SYNC_2;
	frame[2] = TRACKER_FRAME_DATA_LEN;
	frame[3] = FRAME_ID_TRACKER;
	frame[4] = control;
	put_be_u16(frame, 5, value);
	frame[TRACKER_FRAME_LEN - 1] = checksum8(frame, TRACKER_FRAME_LEN - 1);
	return write_frame(frame, sizeof(frame));
}

bool DytGimbal::send_laser_command(uint8_t control)
{
	uint8_t frame[LASER_FRAME_LEN]{};
	frame[0] = SYNC_1;
	frame[1] = SYNC_2;
	frame[2] = LASER_FRAME_DATA_LEN;
	frame[3] = FRAME_ID_LASER;
	frame[4] = control;
	frame[LASER_FRAME_LEN - 1] = checksum8(frame, LASER_FRAME_LEN - 1);
	return write_frame(frame, sizeof(frame));
}

bool DytGimbal::send_flight_data(const dyt_command_s &cmd)
{
	if (!PX4_ISFINITE(static_cast<float>(cmd.lat)) || !PX4_ISFINITE(static_cast<float>(cmd.lon)) || !PX4_ISFINITE(cmd.alt) ||
	    !PX4_ISFINITE(cmd.rel_alt) || !PX4_ISFINITE(cmd.roll_rad) || !PX4_ISFINITE(cmd.pitch_rad) ||
	    !PX4_ISFINITE(cmd.yaw_rad) || cmd.lat < -90.0 || cmd.lat > 90.0 || cmd.lon < -180.0 || cmd.lon > 180.0) {
		return false;
	}

	uint8_t frame[FLIGHT_DATA_FRAME_LEN]{};
	frame[0] = SYNC_1;
	frame[1] = SYNC_2;
	frame[2] = 0x26;
	frame[3] = FRAME_ID_FLIGHT_DATA;
	frame[4] = 0x01;
	put_be_float(frame, 5, math::degrees(cmd.yaw_rad));
	put_be_float(frame, 9, math::degrees(cmd.pitch_rad));
	put_be_float(frame, 13, math::degrees(cmd.roll_rad));
	put_be_float(frame, 17, static_cast<float>(cmd.lon));
	put_be_float(frame, 21, static_cast<float>(cmd.lat));
	put_be_float(frame, 25, cmd.alt);
	put_be_float(frame, 29, cmd.rel_alt);

	time_t unix_time = time(nullptr);
	struct tm utc {};

	if (unix_time > 946684800 && gmtime_r(&unix_time, &utc) != nullptr) {
		put_be_u16(frame, 33, static_cast<uint16_t>(utc.tm_year + 1900));
		frame[35] = static_cast<uint8_t>(utc.tm_mon + 1);
		frame[36] = static_cast<uint8_t>(utc.tm_mday);
		frame[37] = static_cast<uint8_t>(utc.tm_hour);
		frame[38] = static_cast<uint8_t>(utc.tm_min);
		put_be_u16(frame, 39, static_cast<uint16_t>(utc.tm_sec * 1000));
	}

	frame[FLIGHT_DATA_FRAME_LEN - 1] = checksum8(frame, FLIGHT_DATA_FRAME_LEN - 1);
	return write_frame(frame, sizeof(frame));
}

bool DytGimbal::write_frame(const uint8_t *buffer, size_t buffer_len)
{
	if (_uart_fd < 0 || buffer == nullptr || buffer_len == 0) {
		return false;
	}

	_last_write_errno = 0;
	const ssize_t written = ::write(_uart_fd, buffer, buffer_len);
	_last_write_result = static_cast<int>(written);

	if (written != static_cast<ssize_t>(buffer_len)) {
		_last_write_errno = written < 0 ? errno : 0;
		++_write_error_count;
		return false;
	}

	++_command_tx_count;
	maybe_log_raw_frame("DYT V2 tx", buffer, buffer_len);
	return true;
}

bool DytGimbal::send_detection_start_if_needed()
{
	if (!_detection_start_pending) {
		return true;
	}

	if (_uart_fd < 0) {
		return false;
	}

	// 55 AA 0A 04 2B 00 01 00 00 00 00 00 00 DB
	static constexpr uint8_t detection_start_frame[TRACKER_FRAME_LEN] {
		SYNC_1, SYNC_2, TRACKER_FRAME_DATA_LEN, FRAME_ID_TRACKER, TRACKER_DETECTION_START,
		0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDB
	};

	while (_detection_start_tx_offset < sizeof(detection_start_frame)) {
		const ssize_t written = ::write(_uart_fd, detection_start_frame + _detection_start_tx_offset,
						  sizeof(detection_start_frame) - _detection_start_tx_offset);
		_last_write_result = static_cast<int>(written);

		if (written > 0) {
			_detection_start_tx_offset += static_cast<size_t>(written);
			continue;
		}

		if (written < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
			_last_write_errno = errno;
			return false;
		}

		_last_write_errno = written < 0 ? errno : 0;
		++_write_error_count;
		_detection_start_tx_offset = 0;
		close_serial();
		return false;
	}

	_detection_start_pending = false;
	_detection_start_tx_offset = 0;
	_last_write_errno = 0;
	++_command_tx_count;
	maybe_log_raw_frame("DYT V2 tx", detection_start_frame, sizeof(detection_start_frame));
	PX4_INFO("target detection started on arm");
	return true;
}

void DytGimbal::send_startup_home_if_needed(hrt_abstime now)
{
	if (_startup_home_sent || _param_dyt_home_en.get() <= 0 || now < _startup_home_time) {
		return;
	}

	set_angle_mode(_param_dyt_home_yaw_deg.get(), _param_dyt_home_pitch_deg.get());
	send_mode_once();
	_startup_home_sent = true;
}

void DytGimbal::publish_shell_command(uint8_t command)
{
	dyt_command_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.command = command;
	_dyt_command_pub.publish(cmd);
}

void DytGimbal::publish_shell_angle_command(float yaw_deg, float pitch_deg)
{
	dyt_command_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.command = dyt_command_s::CMD_SET_FRAME_ANGLE;
	cmd.param_x = angle_deg_to_cdeg(yaw_deg);
	cmd.param_y = angle_deg_to_cdeg(pitch_deg);
	_dyt_command_pub.publish(cmd);
}

void DytGimbal::publish_shell_track_point_command(int16_t x_px, int16_t y_px)
{
	dyt_command_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.command = dyt_command_s::CMD_TRACK_POINT;
	cmd.param_x = x_px;
	cmd.param_y = y_px;
	_dyt_command_pub.publish(cmd);
}

void DytGimbal::publish_shell_value_command(uint8_t command, uint32_t value)
{
	dyt_command_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.command = command;
	cmd.value = value;
	_dyt_command_pub.publish(cmd);
}

uint8_t DytGimbal::checksum8(const uint8_t *buffer, size_t checksum_index)
{
	uint8_t checksum = 0xFF;

	for (size_t i = 2; i < checksum_index; ++i) {
		checksum ^= buffer[i];
	}

	return checksum;
}

uint16_t DytGimbal::read_be_u16(const uint8_t *buffer, size_t index)
{
	return static_cast<uint16_t>((static_cast<uint16_t>(buffer[index]) << 8) | buffer[index + 1]);
}

int16_t DytGimbal::read_be_s16(const uint8_t *buffer, size_t index)
{
	return static_cast<int16_t>(read_be_u16(buffer, index));
}

float DytGimbal::read_be_float(const uint8_t *buffer, size_t index)
{
	const uint32_t raw = (static_cast<uint32_t>(buffer[index]) << 24) |
			     (static_cast<uint32_t>(buffer[index + 1]) << 16) |
			     (static_cast<uint32_t>(buffer[index + 2]) << 8) |
			     static_cast<uint32_t>(buffer[index + 3]);
	float value = NAN;
	memcpy(&value, &raw, sizeof(value));
	return value;
}

void DytGimbal::put_be_u16(uint8_t *buffer, size_t index, uint16_t value)
{
	buffer[index] = static_cast<uint8_t>((value >> 8) & 0xFF);
	buffer[index + 1] = static_cast<uint8_t>(value & 0xFF);
}

void DytGimbal::put_be_u32(uint8_t *buffer, size_t index, uint32_t value)
{
	buffer[index] = static_cast<uint8_t>((value >> 24) & 0xFF);
	buffer[index + 1] = static_cast<uint8_t>((value >> 16) & 0xFF);
	buffer[index + 2] = static_cast<uint8_t>((value >> 8) & 0xFF);
	buffer[index + 3] = static_cast<uint8_t>(value & 0xFF);
}

void DytGimbal::put_be_float(uint8_t *buffer, size_t index, float value)
{
	uint32_t raw = 0;
	memcpy(&raw, &value, sizeof(raw));
	put_be_u32(buffer, index, raw);
}

int16_t DytGimbal::angle_deg_to_cdeg(float angle_deg)
{
	if (!PX4_ISFINITE(angle_deg)) {
		return 0;
	}

	return static_cast<int16_t>(roundf(math::constrain(angle_deg, -180.f, 180.f) * 100.f));
}

bool DytGimbal::parse_float_arg(const char *arg, float &value)
{
	if (arg == nullptr || arg[0] == '\0') {
		return false;
	}

	char *end = nullptr;
	errno = 0;
	const float parsed = strtof(arg, &end);

	if (end == arg || *end != '\0' || errno == ERANGE || !PX4_ISFINITE(parsed)) {
		return false;
	}

	value = parsed;
	return true;
}

bool DytGimbal::parse_int32_arg(const char *arg, int32_t &value, int32_t min_value, int32_t max_value)
{
	if (arg == nullptr || arg[0] == '\0') {
		return false;
	}

	char *end = nullptr;
	errno = 0;
	const long parsed = strtol(arg, &end, 10);

	if (end == arg || *end != '\0' || errno == ERANGE || parsed < min_value || parsed > max_value) {
		return false;
	}

	value = static_cast<int32_t>(parsed);
	return true;
}

bool DytGimbal::parse_uint32_arg(const char *arg, uint32_t &value)
{
	if (arg == nullptr || arg[0] == '\0' || arg[0] == '-') {
		return false;
	}

	char *end = nullptr;
	errno = 0;
	const unsigned long long parsed = strtoull(arg, &end, 10);

	if (end == arg || *end != '\0' || errno == ERANGE || parsed > UINT32_MAX) {
		return false;
	}

	value = static_cast<uint32_t>(parsed);
	return true;
}

int DytGimbal::task_spawn(int argc, char *argv[])
{
	const char *device = "/dev/ttyS3";
	int ch = 0;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		if (ch == 'd') {
			device = myoptarg;

		} else {
			return print_usage("unknown option");
		}
	}

	DytGimbal *instance = new DytGimbal(device);

	if (instance != nullptr) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int DytGimbal::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		return print_usage("module not running");
	}

	if (!strcmp(argv[0], "status")) {
		get_instance()->show_status();
		return PX4_OK;
	}

	if (!strcmp(argv[0], "autolock")) {
		get_instance()->publish_shell_command(dyt_command_s::CMD_AUTO_LOCK);
		return PX4_OK;
	}

	if (!strcmp(argv[0], "stoptrk")) {
		get_instance()->publish_shell_command(dyt_command_s::CMD_STOP_TRACK);
		return PX4_OK;
	}

	if (!strcmp(argv[0], "center")) {
		get_instance()->publish_shell_command(dyt_command_s::CMD_CENTER);
		return PX4_OK;
	}

	if (!strcmp(argv[0], "laser")) {
		if (argc < 2) {
			return print_usage("usage: dyt_gimbal laser <on|continuous|off>");
		}

		uint8_t command = dyt_command_s::CMD_NONE;

		if (!strcmp(argv[1], "on")) {
			command = dyt_command_s::CMD_LASER_ON;

		} else if (!strcmp(argv[1], "continuous")) {
			command = dyt_command_s::CMD_LASER_CONTINUOUS;

		} else if (!strcmp(argv[1], "off")) {
			command = dyt_command_s::CMD_LASER_OFF;

		} else {
			return print_usage("usage: dyt_gimbal laser <on|continuous|off>");
		}

		get_instance()->publish_shell_command(command);
		return PX4_OK;
	}

	if (!strcmp(argv[0], "osd")) {
		if (argc < 2 || (strcmp(argv[1], "on") && strcmp(argv[1], "off"))) {
			return print_usage("usage: dyt_gimbal osd <on|off>");
		}

		const uint8_t command = !strcmp(argv[1], "on") ? dyt_command_s::CMD_OSD_ENABLE :
					dyt_command_s::CMD_OSD_DISABLE;
		get_instance()->publish_shell_command(command);
		return PX4_OK;
	}

	if (!strcmp(argv[0], "angle")) {
		if (argc < 3) {
			return print_usage("usage: dyt_gimbal angle <yaw_deg> <pitch_deg>");
		}

		float yaw_deg = NAN;
		float pitch_deg = NAN;

		if (!parse_float_arg(argv[1], yaw_deg) || !parse_float_arg(argv[2], pitch_deg)) {
			return print_usage("invalid angle argument");
		}

		get_instance()->publish_shell_angle_command(yaw_deg, pitch_deg);
		return PX4_OK;
	}

	if (!strcmp(argv[0], "trackxy")) {
		if (argc < 3) {
			return print_usage("usage: dyt_gimbal trackxy <x_px> <y_px>");
		}

		int32_t x_px = 0;
		int32_t y_px = 0;

		if (!parse_int32_arg(argv[1], x_px, INT16_MIN, INT16_MAX) ||
		    !parse_int32_arg(argv[2], y_px, INT16_MIN, INT16_MAX)) {
			return print_usage("invalid tracking pixel coordinates");
		}

		get_instance()->publish_shell_track_point_command(static_cast<int16_t>(x_px), static_cast<int16_t>(y_px));
		return PX4_OK;
	}

	if (!strcmp(argv[0], "trackid")) {
		if (argc < 2) {
			return print_usage("usage: dyt_gimbal trackid <target_id>");
		}

		uint32_t target_id = 0;

		if (!parse_uint32_arg(argv[1], target_id)) {
			return print_usage("invalid target id");
		}

		get_instance()->publish_shell_value_command(dyt_command_s::CMD_TRACK_ID, target_id);
		return PX4_OK;
	}

	if (!strcmp(argv[0], "ai") || !strcmp(argv[0], "assist")) {
		if (argc < 2 || (strcmp(argv[1], "on") && strcmp(argv[1], "off"))) {
			return print_usage(!strcmp(argv[0], "ai") ? "usage: dyt_gimbal ai <on|off>" :
				   "usage: dyt_gimbal assist <on|off>");
		}

		const bool enable = !strcmp(argv[1], "on");
		uint8_t command = dyt_command_s::CMD_NONE;

		if (!strcmp(argv[0], "ai")) {
			command = enable ? dyt_command_s::CMD_AI_ENABLE : dyt_command_s::CMD_AI_DISABLE;

		} else {
			command = enable ? dyt_command_s::CMD_ASSIST_ENABLE : dyt_command_s::CMD_ASSIST_DISABLE;
		}

		get_instance()->publish_shell_command(command);
		return PX4_OK;
	}

	if (!strcmp(argv[0], "target")) {
		if (argc < 2) {
			return print_usage("usage: dyt_gimbal target <all|person|vehicle>");
		}

		uint32_t target_type = 0;

		if (!strcmp(argv[1], "all")) {
			target_type = 1;

		} else if (!strcmp(argv[1], "person")) {
			target_type = 2;

		} else if (!strcmp(argv[1], "vehicle")) {
			target_type = 3;

		} else {
			return print_usage("invalid target type");
		}

		get_instance()->publish_shell_value_command(dyt_command_s::CMD_TARGET_TYPE, target_type);
		return PX4_OK;
	}

	if (!strcmp(argv[0], "imagemode")) {
		if (argc < 2) {
			return print_usage("usage: dyt_gimbal imagemode <vis|ir|pip1|pip2|up1|up2|left1|left2|bw|color>");
		}

		static constexpr const char *image_mode_names[] = {
			"vis", "ir", "pip1", "pip2", "up1", "up2", "left1", "left2", "bw", "color"
		};
		uint32_t image_mode = UINT32_MAX;

		for (uint32_t i = 0; i < sizeof(image_mode_names) / sizeof(image_mode_names[0]); ++i) {
			if (!strcmp(argv[1], image_mode_names[i])) {
				image_mode = i;
				break;
			}
		}

		if (image_mode == UINT32_MAX) {
			return print_usage("invalid image mode");
		}

		get_instance()->publish_shell_value_command(dyt_command_s::CMD_IMAGE_MODE, image_mode);
		return PX4_OK;
	}

	return print_usage("unknown command");
}

int DytGimbal::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Tweety V2.0.9.6 RS422 protocol driver for gimbal status and control.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("dyt_gimbal", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("-d <device>", "UART device connected through an RS422 transceiver", true);
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND("autolock");
	PRINT_MODULE_USAGE_COMMAND("stoptrk");
	PRINT_MODULE_USAGE_COMMAND("center");
	PRINT_MODULE_USAGE_COMMAND_DESCR("laser", "Control laser power and continuous ranging");
	PRINT_MODULE_USAGE_ARG("<on|continuous|off>", "Laser command", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("osd", "Enable or disable tracker character display");
	PRINT_MODULE_USAGE_ARG("<on|off>", "OSD character display state", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("angle", "Set follow-mode yaw and pitch angles in degrees");
	PRINT_MODULE_USAGE_ARG("<yaw_deg> <pitch_deg>", "Yaw and pitch angles", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("trackxy", "Track a target at image pixel coordinates");
	PRINT_MODULE_USAGE_ARG("<x_px> <y_px>", "Signed target coordinates; 0 0 selects image center", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("trackid", "Convert an AI detection target ID to tracking");
	PRINT_MODULE_USAGE_ARG("<target_id>", "Unsigned 32-bit target ID", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("ai", "Enable or disable AI detection");
	PRINT_MODULE_USAGE_ARG("<on|off>", "AI detection state", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("assist", "Enable or disable intelligent assisted tracking");
	PRINT_MODULE_USAGE_ARG("<on|off>", "Assisted tracking state", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("target", "Select the AI detection target type");
	PRINT_MODULE_USAGE_ARG("<all|person|vehicle>", "AI target category", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("imagemode", "Select tracker display mode");
	PRINT_MODULE_USAGE_ARG("<vis|ir|pip1|pip2|up1|up2|left1|left2|bw|color>", "Display mode", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int dyt_gimbal_main(int argc, char *argv[])
{
	return DytGimbal::main(argc, argv);
}
