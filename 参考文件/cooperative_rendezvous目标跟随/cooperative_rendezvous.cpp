#include "cooperative_rendezvous.hpp"

#include <commander/px4_custom_mode.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#include <float.h>
#include <math.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

CooperativeRendezvous::CooperativeRendezvous(const Options &options) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_options(options)
{
}

bool CooperativeRendezvous::init()
{
	ScheduleOnInterval(50_ms);
	return true;
}

void CooperativeRendezvous::update_params_if_needed()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s update{};
		_parameter_update_sub.copy(&update);
		const int32_t previous_history_enable = _param_history_enable.get();
		const float previous_history_duration = _param_history_duration.get();
		updateParams();

		if (_param_history_enable.get() != previous_history_enable ||
		    fabsf(_param_history_duration.get() - previous_history_duration) > FLT_EPSILON) {
			reset_target_history();
		}
	}
}

void CooperativeRendezvous::update_vehicle_id()
{
	if (_vehicle_id_initialized) {
		return;
	}

	int32_t mav_sys_id = 0;
	param_t handle = param_find("MAV_SYS_ID");

	if (handle != PARAM_INVALID && param_get(handle, &mav_sys_id) == PX4_OK && mav_sys_id > 0) {
		_vehicle_id = static_cast<uint32_t>(mav_sys_id);
		_vehicle_id_initialized = true;
	}
}

CooperativeRendezvous::Role CooperativeRendezvous::active_role() const
{
	if (_options.role != Role::Auto) {
		return _options.role;
	}

	return _vehicle_id == 2 ? Role::Rendezvous : Role::Broadcast;
}

float CooperativeRendezvous::aux_value(int index) const
{
	switch (index) {
	case 1: return _manual_control.aux1;
	case 2: return _manual_control.aux2;
	case 3: return _manual_control.aux3;
	case 4: return _manual_control.aux4;
	case 5: return _manual_control.aux5;
	case 6: return _manual_control.aux6;
	default: return NAN;
	}
}

bool CooperativeRendezvous::aux_switch_active(int index) const
{
	const float value = aux_value(index);
	return PX4_ISFINITE(value) && value > 0.5f;
}

bool CooperativeRendezvous::button_active(int button) const
{
	if (button < 0 || button > 15) {
		return false;
	}

	return (_manual_control.buttons & (1u << button)) != 0;
}

bool CooperativeRendezvous::rendezvous_switch_enabled() const
{
	if (dyt_status_fresh()) {
		if (_dyt_guidance_status.gcs_phase_request == dyt_guidance_status_s::PHASE_MIDCOURSE) {
			return true;
		}

		if (_dyt_guidance_status.gcs_phase_request == dyt_guidance_status_s::PHASE_TERMINAL) {
			return _gcs_midcourse_engaged;
		}
	}

	const int act_aux = _param_act_aux.get();
	const int act_btn = _param_act_btn.get();

	return (act_aux == 0 && act_btn < 0) || aux_switch_active(act_aux) || button_active(act_btn);
}

bool CooperativeRendezvous::dyt_status_fresh() const
{
	return _dyt_guidance_status.timestamp != 0 &&
	       hrt_elapsed_time(&_dyt_guidance_status.timestamp) < 500_ms;
}

bool CooperativeRendezvous::dyt_guidance_active() const
{
	// Yield to the seeker only while it is actually commanding aircraft motion
	// (camera locked and tracking). While the seeker is just searching or has lost
	// the target it controls the gimbal only, so the position-sharing follower keeps
	// driving the aircraft. This is the single handoff boundary between the two
	// controllers so they never publish trajectory setpoints at the same time.
	return _dyt_guidance_status.controlling_vehicle && _dyt_guidance_status.timestamp != 0 &&
	       hrt_elapsed_time(&_dyt_guidance_status.timestamp) < 500_ms;
}

bool CooperativeRendezvous::vehicle_status_fresh(const vehicle_status_s &status) const
{
	return status.timestamp != 0 && status.timestamp <= hrt_absolute_time() && hrt_elapsed_time(&status.timestamp) < 1_s;
}

bool CooperativeRendezvous::protected_navigation_state(uint8_t nav_state) const
{
	return nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION
	       || nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER
	       || nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL
	       || nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
}

bool CooperativeRendezvous::offboard_control_active(const vehicle_status_s &status) const
{
	return vehicle_status_fresh(status) && !status.failsafe
	       && status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD
	       && status.nav_state_user_intention == vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
}

bool CooperativeRendezvous::offboard_preparation_allowed(const vehicle_status_s &status) const
{
	if (!vehicle_status_fresh(status) || status.failsafe) {
		return false;
	}

	if (protected_navigation_state(status.nav_state_user_intention) && !_geofence_resume_pending) {
		return false;
	}

	if (!protected_navigation_state(status.nav_state)) {
		return true;
	}

	// A protected mode keeps ownership until the operator explicitly selects Offboard,
	// except for the existing geofence-clear path which is allowed to resume automatically.
	return _geofence_resume_pending
	       || status.nav_state_user_intention == vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
}

bool CooperativeRendezvous::target_data_fresh() const
{
	const hrt_abstime now = hrt_absolute_time();

	if (_param_gcs_enable.get() > 0 && _gcs_target_active && _last_gcs_setpoint_time != 0
	    && _last_gcs_setpoint_time <= now) {
		const float timeout_param = _param_gcs_timeout.get();
		const float timeout_s = PX4_ISFINITE(timeout_param) ? math::constrain(timeout_param, 0.1f, 5.f) : 0.6f;

		if (now - _last_gcs_setpoint_time <= static_cast<hrt_abstime>(timeout_s * 1_s)) {
			return true;
		}
	}

	if (!_map_ref_initialized || _last_target_time == 0 || _last_target_time > now
	    || !PX4_ISFINITE(_target_info.lat) || !PX4_ISFINITE(_target_info.lon) || !PX4_ISFINITE(_target_info.alt)) {
		return false;
	}

	const float timeout_param = _param_target_timeout.get();
	const float timeout_s = PX4_ISFINITE(timeout_param) ? math::constrain(timeout_param, 0.1f, 30.f) : 2.f;
	return now - _last_target_time <= static_cast<hrt_abstime>(timeout_s * 1_s);
}

void CooperativeRendezvous::publish_status(const vehicle_status_s &status, bool local_position_is_valid,
		bool controlling_vehicle)
{
	cooperative_rendezvous_status_s cooperative_status{};
	cooperative_status.timestamp = hrt_absolute_time();
	cooperative_status.command_sequence = dyt_status_fresh() ? _dyt_guidance_status.command_sequence : 0;
	cooperative_status.role = active_role() == Role::Rendezvous ?
				     cooperative_rendezvous_status_s::ROLE_RENDEZVOUS :
				     cooperative_rendezvous_status_s::ROLE_BROADCAST;
	cooperative_status.enabled = rendezvous_switch_enabled();
	cooperative_status.controlling_vehicle = controlling_vehicle;
	cooperative_status.active = cooperative_status.enabled && local_position_is_valid &&
				    status.arming_state == vehicle_status_s::ARMING_STATE_ARMED &&
				    controlling_vehicle;
	cooperative_status.target_valid = _last_target_time != 0 &&
					  hrt_elapsed_time(&_last_target_time) <=
					  static_cast<hrt_abstime>(_options.target_timeout_s * 1_s);
	_status_pub.publish(cooperative_status);
}

bool CooperativeRendezvous::local_position_valid(const vehicle_local_position_s &local_pos) const
{
	return local_pos.xy_valid && local_pos.z_valid &&
	       local_pos.xy_global && local_pos.z_global &&
	       PX4_ISFINITE(local_pos.x) && PX4_ISFINITE(local_pos.y) && PX4_ISFINITE(local_pos.z) &&
	       PX4_ISFINITE(local_pos.ref_lat) && PX4_ISFINITE(local_pos.ref_lon) && PX4_ISFINITE(local_pos.ref_alt);
}

bool CooperativeRendezvous::update_map_projection(const vehicle_local_position_s &local_pos)
{
	if (!local_position_valid(local_pos)) {
		return false;
	}

	if (!_map_ref_initialized) {
		_map_ref.initReference(local_pos.ref_lat, local_pos.ref_lon, local_pos.ref_timestamp);
		_map_ref_initialized = true;
		PX4_INFO("cooperative map ref: lat=%.7f lon=%.7f", local_pos.ref_lat, local_pos.ref_lon);
	}

	return true;
}

void CooperativeRendezvous::publish_own_position(const vehicle_local_position_s &local_pos)
{
	if (!_vehicle_id_initialized || !update_map_projection(local_pos)) {
		return;
	}

	double lat = static_cast<double>(NAN);
	double lon = static_cast<double>(NAN);
	_map_ref.reproject(local_pos.x, local_pos.y, lat, lon);

	cooperative_position_s position{};
	position.timestamp = hrt_absolute_time();
	position.mavid = _vehicle_id;
	position.lat = lat;
	position.lon = lon;
	position.alt = static_cast<double>(static_cast<float>(local_pos.ref_alt) - local_pos.z);
	position.vx = local_pos.vx;
	position.vy = local_pos.vy;
	position.vz = local_pos.vz;
	position.yaw = local_pos.heading_good_for_control ? local_pos.heading : static_cast<float>(NAN);
	position.yawspeed = local_pos.delta_heading;

	_cooperative_position_pub.publish(position);
}

bool CooperativeRendezvous::update_target_from_link()
{
	follower_info_s info{};
	bool updated = false;

	while (_follower_info_sub.update(&info)) {
		const bool real_position_source = info.source == follower_info_s::SOURCE_REAL_POSITION ||
						  info.source == follower_info_s::SOURCE_LEADER_REAL_POSITION;

		if (info.mavid == _options.target_id && info.mavid != _vehicle_id &&
		    real_position_source &&
		    PX4_ISFINITE(info.lat) && PX4_ISFINITE(info.lon) && PX4_ISFINITE(info.alt)) {
			_target_info = info;
			_last_target_time = hrt_absolute_time();
			updated = true;
		}
	}

	return updated;
}

void CooperativeRendezvous::update_gcs_setpoint()
{
	gcs_trajectory_setpoint_s setpoint{};

	if (_param_gcs_enable.get() <= 0) {
		_gcs_trajectory_setpoint_sub.update(&setpoint);
		_last_gcs_setpoint_time = 0;
		_gcs_target_active = false;
		return;
	}

	const float timeout_param = _param_gcs_timeout.get();
	const float timeout_s = PX4_ISFINITE(timeout_param) ? math::constrain(timeout_param, 0.1f, 5.f) : 0.6f;
	const hrt_abstime timeout = static_cast<hrt_abstime>(timeout_s * 1_s);
	const hrt_abstime update_time = hrt_absolute_time();

	if (_last_gcs_setpoint_time > update_time ||
	    (_last_gcs_setpoint_time != 0 && update_time - _last_gcs_setpoint_time > timeout)) {
		_last_gcs_setpoint_time = 0;
		_gcs_target_active = false;
	}

	while (_gcs_trajectory_setpoint_sub.update(&setpoint)) {
		const hrt_abstime now = hrt_absolute_time();

		if (setpoint.timestamp == 0 || setpoint.timestamp > now || now - setpoint.timestamp > timeout) {
			continue;
		}

		const matrix::Vector3f position(setpoint.position);

		if (!position.isAllFinite()) {
			continue;
		}

		_gcs_setpoint = setpoint;
		_last_gcs_setpoint_time = setpoint.timestamp;
		_gcs_target_active = true;
	}
}

bool CooperativeRendezvous::gcs_setpoint_active(const vehicle_local_position_s &local_pos,
		matrix::Vector3f &target_position, matrix::Vector3f &target_velocity, float &yaw)
{
	if (_param_gcs_enable.get() <= 0 || !_gcs_target_active || _last_gcs_setpoint_time == 0) {
		return false;
	}

	const hrt_abstime now = hrt_absolute_time();
	const float timeout_param = _param_gcs_timeout.get();
	const float timeout_s = PX4_ISFINITE(timeout_param) ? math::constrain(timeout_param, 0.1f, 5.f) : 0.6f;

	if (_last_gcs_setpoint_time > now
	    || now - _last_gcs_setpoint_time > static_cast<hrt_abstime>(timeout_s * 1_s)) {
		_last_gcs_setpoint_time = 0;
		_gcs_target_active = false;
		return false;
	}

	target_position = matrix::Vector3f(_gcs_setpoint.position);
	target_velocity = matrix::Vector3f(_gcs_setpoint.velocity);

	if (!target_velocity.isAllFinite()) {
		target_velocity.zero();
	}

	yaw = PX4_ISFINITE(_gcs_setpoint.yaw) ? _gcs_setpoint.yaw : local_pos.heading;
	return true;
}

void CooperativeRendezvous::enforce_target_minimum_height(matrix::Vector3f &target_position) const
{
	if (_param_minimum_height_enable.get() <= 0) {
		return;
	}

	const float minimum_height = _param_minimum_height.get();

	if (!PX4_ISFINITE(minimum_height) || fabsf(minimum_height) < 0.0001f || !PX4_ISFINITE(target_position(2))) {
		return;
	}

	const float highest_allowed_down = -math::constrain(minimum_height, -100.f, 100.f);

	if (target_position(2) > highest_allowed_down) {
		target_position(2) = highest_allowed_down;
	}
}

void CooperativeRendezvous::push_target_history(const matrix::Vector3f &target_position)
{
	if (_param_history_enable.get() <= 0 || !target_position.isAllFinite()) {
		return;
	}

	_target_history[_target_history_head].timestamp = hrt_absolute_time();
	_target_history[_target_history_head].position = target_position;
	_target_history_head = (_target_history_head + 1) % kTargetHistoryLength;
	_target_history_count = math::min(_target_history_count + 1, kTargetHistoryLength);
}

bool CooperativeRendezvous::delayed_target_position(matrix::Vector3f &target_position) const
{
	if (_param_history_enable.get() <= 0) {
		return false;
	}

	const float delay_s = _param_track_delay.get();
	const float history_duration_param = _param_history_duration.get();
	const float history_duration_s = PX4_ISFINITE(history_duration_param) ?
					 math::constrain(history_duration_param, 0.1f, kMaxTargetHistoryDurationS) : 2.f;

	if (!PX4_ISFINITE(delay_s) || delay_s <= 0.f || _target_history_count <= 0) {
		return false;
	}

	const hrt_abstime now = hrt_absolute_time();
	const hrt_abstime history_duration = static_cast<hrt_abstime>(history_duration_s * 1_s);
	const hrt_abstime delay = static_cast<hrt_abstime>(math::constrain(delay_s, 0.f, history_duration_s) * 1_s);
	const hrt_abstime target_time = now > delay ? now - delay : 0;
	int before_index = -1;
	int after_index = -1;
	int oldest_index = -1;
	hrt_abstime before_time = 0;
	hrt_abstime after_time = 0;
	hrt_abstime oldest_time = 0;

	for (int i = 0; i < _target_history_count; i++) {
		const TargetHistorySample &sample = _target_history[i];

		if (sample.timestamp == 0 || sample.timestamp > now || now - sample.timestamp > history_duration) {
			continue;
		}

		if (oldest_index < 0 || sample.timestamp < oldest_time) {
			oldest_index = i;
			oldest_time = sample.timestamp;
		}

		if (sample.timestamp <= target_time && sample.timestamp >= before_time) {
			before_index = i;
			before_time = sample.timestamp;
		}

		if (sample.timestamp >= target_time && (after_index < 0 || sample.timestamp <= after_time)) {
			after_index = i;
			after_time = sample.timestamp;
		}
	}

	if (before_index < 0) {
		before_index = oldest_index;
		after_index = oldest_index;
	}

	if (before_index < 0) {
		return false;
	}

	if (after_index < 0 || before_index == after_index || after_time <= before_time) {
		target_position = _target_history[before_index].position;
		return true;
	}

	const float alpha = math::constrain((target_time - before_time) / static_cast<float>(after_time - before_time), 0.f,
					   1.f);
	target_position = _target_history[before_index].position +
			  (_target_history[after_index].position - _target_history[before_index].position) * alpha;
	return true;
}

void CooperativeRendezvous::reset_target_history()
{
	_target_history_head = 0;
	_target_history_count = 0;
}

bool CooperativeRendezvous::target_state_local(const vehicle_local_position_s &local_pos,
		matrix::Vector3f &target_position, matrix::Vector3f &target_velocity)
{
	if (!_map_ref_initialized || _last_target_time == 0) {
		reset_target_filter();
		reset_target_history();
		return false;
	}

	const float timeout_param = _param_target_timeout.get();
	const float timeout_s = PX4_ISFINITE(timeout_param) ? math::constrain(timeout_param, 0.1f, 30.f) : 2.f;

	if ((hrt_absolute_time() - _last_target_time) > static_cast<hrt_abstime>(timeout_s * 1_s)) {
		reset_target_filter();
		reset_target_history();
		return false;
	}

	float x = NAN;
	float y = NAN;
	_map_ref.project(_target_info.lat, _target_info.lon, x, y);

	if (!PX4_ISFINITE(x) || !PX4_ISFINITE(y)) {
		reset_target_filter();
		reset_target_history();
		return false;
	}

	const float target_altitude_amsl = static_cast<float>(_target_info.alt);
	const float target_z = static_cast<float>(local_pos.ref_alt) - target_altitude_amsl;
	const float vertical_offset = _options.target_offset(2) - _param_alt_diff.get();
	const float vertical_setpoint_error = fabsf(target_z + vertical_offset - local_pos.z);
	const float configured_max_altitude_error = _param_max_altitude_error.get();
	const float max_altitude_error = PX4_ISFINITE(configured_max_altitude_error) ?
					 math::constrain(configured_max_altitude_error, 1.f, 500.f) : 100.f;

	if (!PX4_ISFINITE(target_z) || !PX4_ISFINITE(vertical_offset) || !PX4_ISFINITE(vertical_setpoint_error) ||
	    vertical_setpoint_error > max_altitude_error) {
		const hrt_abstime now = hrt_absolute_time();

		if (now - _last_status_log > 2_s) {
			PX4_ERR("cooperative rendezvous: target altitude rejected amsl=%.1f sp_z=%.1f current_z=%.1f error=%.1f limit=%.1f",
				 (double)target_altitude_amsl, (double)(target_z + vertical_offset),
				 (double)local_pos.z, (double)vertical_setpoint_error, (double)max_altitude_error);
			_last_status_log = now;
		}

		reset_target_filter();
		reset_target_history();
		return false;
	}

	matrix::Vector3f raw_position(x, y, target_z);
	matrix::Vector3f raw_velocity(static_cast<float>(_target_info.vx), static_cast<float>(_target_info.vy),
				      static_cast<float>(_target_info.vz));

	if (!PX4_ISFINITE(raw_velocity(0)) || !PX4_ISFINITE(raw_velocity(1)) || !PX4_ISFINITE(raw_velocity(2))) {
		raw_velocity.zero();
	}

	apply_target_filter(raw_position, raw_velocity, target_position, target_velocity);

	float offset_x = _options.target_offset(0);
	float offset_y = _options.target_offset(1);

	if (_param_xy_offset_enable.get() > 0) {
		const float x_offset = _param_x_offset.get();
		const float y_offset = _param_y_offset.get();

		offset_x = PX4_ISFINITE(x_offset) ? x_offset : 0.f;
		offset_y = PX4_ISFINITE(y_offset) ? y_offset : 0.f;

	} else {
		const float target_distance = _param_dist.get();

		if (PX4_ISFINITE(target_distance) && target_distance >= 0.f) {
			const float offset_norm = sqrtf(offset_x * offset_x + offset_y * offset_y);

			if (offset_norm > 0.001f) {
				const float scale = target_distance / offset_norm;
				offset_x *= scale;
				offset_y *= scale;

			} else {
				offset_x = -target_distance;
				offset_y = 0.f;
			}
		}
	}

	target_position(0) += offset_x;
	target_position(1) += offset_y;
	target_position(2) += vertical_offset;
	enforce_target_minimum_height(target_position);
	push_target_history(target_position);
	delayed_target_position(target_position);
	enforce_target_minimum_height(target_position);

	return PX4_ISFINITE(target_position(2));
}

void CooperativeRendezvous::apply_target_filter(const matrix::Vector3f &raw_position,
		const matrix::Vector3f &raw_velocity, matrix::Vector3f &target_position, matrix::Vector3f &target_velocity)
{
	const float position_tc = _param_target_position_tc.get();
	const float velocity_tc = _param_target_velocity_tc.get();
	const float max_position_jump = _param_target_position_jump.get();
	const bool filter_enabled = (PX4_ISFINITE(position_tc) && position_tc > 0.f) ||
				    (PX4_ISFINITE(velocity_tc) && velocity_tc > 0.f) ||
				    (PX4_ISFINITE(max_position_jump) && max_position_jump > 0.f);

	if (!filter_enabled) {
		reset_target_filter();
		target_position = raw_position;
		target_velocity = raw_velocity;
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	const bool new_sample = _last_target_filter_sample_time != _last_target_time;

	if (!_target_filter_initialized) {
		_target_position_input = raw_position;
		_target_position_filtered = raw_position;
		_target_velocity_input = raw_velocity;
		_target_velocity_filtered = raw_velocity;
		_last_target_filter_time = now;
		_last_target_filter_sample_time = _last_target_time;
		_target_filter_initialized = true;
	}

	if (new_sample) {
		matrix::Vector3f position_input = raw_position;

		if (PX4_ISFINITE(max_position_jump) && max_position_jump > 0.f) {
			const matrix::Vector3f delta = raw_position - _target_position_input;
			const float delta_norm = delta.norm();
			const float constrained_jump = math::constrain(max_position_jump, 0.1f, 200.f);

			if (PX4_ISFINITE(delta_norm) && delta_norm > constrained_jump) {
				position_input = _target_position_input + delta.normalized() * constrained_jump;
			}
		}

		_target_position_input = position_input;
		_target_velocity_input = raw_velocity;
		_last_target_filter_sample_time = _last_target_time;
	}

	const float dt = math::constrain((now - _last_target_filter_time) * 1e-6f, 0.005f, 0.5f);

	if (PX4_ISFINITE(position_tc) && position_tc > 0.f) {
		const float alpha = dt / (math::constrain(position_tc, 0.02f, 10.f) + dt);
		_target_position_filtered += (_target_position_input - _target_position_filtered) * alpha;

	} else {
		_target_position_filtered = _target_position_input;
	}

	if (PX4_ISFINITE(velocity_tc) && velocity_tc > 0.f) {
		const float alpha = dt / (math::constrain(velocity_tc, 0.02f, 10.f) + dt);
		_target_velocity_filtered += (_target_velocity_input - _target_velocity_filtered) * alpha;

	} else {
		_target_velocity_filtered = _target_velocity_input;
	}

	_last_target_filter_time = now;
	target_position = _target_position_filtered;
	target_velocity = _target_velocity_filtered;
}

void CooperativeRendezvous::reset_target_filter()
{
	_last_target_filter_time = 0;
	_last_target_filter_sample_time = 0;
	_target_filter_initialized = false;
	_target_position_input.zero();
	_target_position_filtered.zero();
	_target_velocity_input.zero();
	_target_velocity_filtered.zero();
}

bool CooperativeRendezvous::update_arrival_hold(matrix::Vector3f &target_position,
		matrix::Vector3f &target_velocity, const vehicle_local_position_s &local_pos, float &yaw)
{
	if (_param_arrival_hold_enable.get() <= 0) {
		reset_arrival_hold();
		return false;
	}

	const float hold_horizontal_radius = math::constrain(_param_arrival_hold_horizontal_radius.get(), 0.1f, 20.f);
	const float hold_vertical_radius = math::constrain(_param_arrival_hold_vertical_radius.get(), 0.1f, 20.f);
	const float hold_velocity = math::constrain(_param_arrival_hold_velocity.get(), 0.f, 10.f);
	const float hold_time_s = math::constrain(_param_arrival_hold_time.get(), 0.f, 10.f);
	const int32_t hold_mode = math::constrain(_param_arrival_hold_mode.get(), static_cast<int32_t>(0),
				  static_cast<int32_t>(2));
	const float release_radius = math::max(math::constrain(_param_arrival_hold_release.get(), 0.1f, 50.f),
					       math::max(hold_horizontal_radius, hold_vertical_radius));
	const hrt_abstime now = hrt_absolute_time();
	const matrix::Vector3f live_target_position = target_position;
	const matrix::Vector3f live_target_velocity = target_velocity;
	const matrix::Vector3f current_position(local_pos.x, local_pos.y, local_pos.z);
	const matrix::Vector3f position_error = live_target_position - current_position;
	const float horizontal_error = matrix::Vector2f(position_error(0), position_error(1)).norm();
	const float vertical_error = fabsf(position_error(2));
	const float target_horizontal_speed = matrix::Vector2f(live_target_velocity(0), live_target_velocity(1)).norm();
	const float target_vertical_speed = fabsf(live_target_velocity(2));
	const bool horizontal_arrived = horizontal_error <= hold_horizontal_radius;
	const bool vertical_arrived = vertical_error <= hold_vertical_radius;
	const bool position_arrived = hold_mode == 1 ? horizontal_arrived :
				      (hold_mode == 2 ? vertical_arrived : (horizontal_arrived && vertical_arrived));
	const bool target_slow = target_horizontal_speed <= hold_velocity && target_vertical_speed <= hold_velocity;

	if (_arrival_hold_active) {
		const matrix::Vector3f live_delta = live_target_position - _arrival_hold_position;
		const float live_horizontal_delta = matrix::Vector2f(live_delta(0), live_delta(1)).norm();
		const float live_vertical_delta = fabsf(live_delta(2));

		if (live_horizontal_delta > release_radius || live_vertical_delta > release_radius) {
			PX4_INFO("cooperative rendezvous: arrival hold released delta=%.1fm",
				 (double)math::max(live_horizontal_delta, live_vertical_delta));
			reset_arrival_hold();

		} else {
			target_position = _arrival_hold_position;
			target_velocity.zero();

			if (PX4_ISFINITE(_arrival_hold_yaw)) {
				yaw = _arrival_hold_yaw;
			}

			return true;
		}
	}

	if (_arrival_follow_active) {
		if (horizontal_error > release_radius || vertical_error > release_radius) {
			PX4_INFO("cooperative rendezvous: arrival follow released error=%.1fm",
				 (double)math::max(horizontal_error, vertical_error));
			reset_arrival_hold();

		} else if (target_slow && position_arrived) {
			if (_arrival_hold_candidate_since == 0) {
				_arrival_hold_candidate_since = now;

			} else if ((now - _arrival_hold_candidate_since) >= static_cast<hrt_abstime>(hold_time_s * 1_s)) {
				_arrival_follow_active = false;
				_arrival_hold_active = true;
				_arrival_hold_position = live_target_position;
				_arrival_hold_yaw = yaw;
				target_position = _arrival_hold_position;
				target_velocity.zero();
				PX4_INFO("cooperative rendezvous: arrival hold active setpoint=(%.1f %.1f %.1f)",
					 (double)_arrival_hold_position(0), (double)_arrival_hold_position(1),
					 (double)_arrival_hold_position(2));
			}

			return true;

		} else {
			_arrival_hold_candidate_since = 0;
			return true;
		}
	}

	if (!position_arrived) {
		_arrival_hold_candidate_since = 0;
		return false;
	}

	if (_arrival_hold_candidate_since == 0) {
		_arrival_hold_candidate_since = now;
		return true;
	}

	if ((now - _arrival_hold_candidate_since) < static_cast<hrt_abstime>(hold_time_s * 1_s)) {
		return true;
	}

	if (target_slow) {
		_arrival_hold_active = true;
		_arrival_hold_position = live_target_position;
		_arrival_hold_yaw = yaw;
		target_position = _arrival_hold_position;
		target_velocity.zero();
		PX4_INFO("cooperative rendezvous: arrival hold active setpoint=(%.1f %.1f %.1f)",
			 (double)_arrival_hold_position(0), (double)_arrival_hold_position(1),
			 (double)_arrival_hold_position(2));

	} else {
		_arrival_follow_active = true;
		PX4_INFO("cooperative rendezvous: arrival follow active");
	}

	return true;
}

void CooperativeRendezvous::reset_arrival_hold()
{
	_arrival_hold_active = false;
	_arrival_follow_active = false;
	_arrival_hold_candidate_since = 0;
	_arrival_hold_position.zero();
	_arrival_hold_yaw = static_cast<float>(NAN);
}

void CooperativeRendezvous::publish_offboard_heartbeat(bool position_control, bool velocity_control)
{
	offboard_control_mode_s mode{};
	mode.timestamp = hrt_absolute_time();
	mode.position = position_control;
	mode.velocity = velocity_control;
	mode.acceleration = false;
	mode.attitude = false;
	mode.body_rate = false;
	_offboard_control_mode_pub.publish(mode);
}

void CooperativeRendezvous::publish_trajectory_setpoint(const matrix::Vector3f &position, const matrix::Vector3f &velocity,
		float yaw)
{
	if (!_trajectory_publication_allowed) {
		return;
	}

	trajectory_setpoint_s setpoint{};
	setpoint.timestamp = hrt_absolute_time();

	for (int i = 0; i < 3; i++) {
		setpoint.position[i] = position(i);
		setpoint.velocity[i] = velocity(i);
		setpoint.acceleration[i] = static_cast<float>(NAN);
		setpoint.jerk[i] = static_cast<float>(NAN);
	}

	setpoint.yaw = yaw;
	setpoint.yawspeed = static_cast<float>(NAN);
	_trajectory_setpoint_pub.publish(setpoint);
}

void CooperativeRendezvous::request_offboard(const vehicle_status_s &status)
{
	const hrt_abstime now = hrt_absolute_time();

	if (!_options.auto_offboard || status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD ||
	    status.timestamp == 0 || now - status.timestamp >= 1_s || now - _last_mode_request < 1_s) {
		return;
	}

	vehicle_command_s command{};
	command.timestamp = now;
	command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	command.param1 = 1.f;
	command.param2 = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
	command.target_system = status.system_id;
	command.target_component = status.component_id;
	command.source_system = status.system_id;
	command.source_component = status.component_id;
	command.from_external = false;
	_vehicle_command_pub.publish(command);
	_last_mode_request = now;
}

void CooperativeRendezvous::request_rtl(const vehicle_status_s &status)
{
	const hrt_abstime now = hrt_absolute_time();

	if (status.arming_state != vehicle_status_s::ARMING_STATE_ARMED ||
	    status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL ||
	    status.timestamp == 0 || now - status.timestamp >= 1_s ||
	    now - _last_rtl_request < 1_s) {
		return;
	}

	vehicle_command_s command{};
	command.timestamp = now;
	command.command = vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH;
	command.target_system = status.system_id;
	command.target_component = status.component_id;
	command.source_system = status.system_id;
	command.source_component = status.component_id;
	command.from_external = false;
	_vehicle_command_pub.publish(command);
	_last_rtl_request = now;
}

bool CooperativeRendezvous::geofence_avoidance_required(const vehicle_status_s &status)
{
	const hrt_abstime now = hrt_absolute_time();
	const bool result_fresh = _geofence_result.timestamp != 0 && now - _geofence_result.timestamp < 1_s;
	const bool custom_fence_triggered = result_fresh && _geofence_result.geofence_custom_fence_triggered;

	if (custom_fence_triggered) {
		if (!_geofence_rtl_active) {
			PX4_WARN("cooperative rendezvous: geofence buffer reached, requesting RTL");
		}

		_geofence_rtl_active = true;
		_geofence_resume_pending = false;
		_geofence_clear_time = 0;
	}

	if (_geofence_rtl_active) {
		// Do not resume guidance until Navigator has explicitly published a fresh clear state.
		if (!result_fresh || custom_fence_triggered) {
			request_rtl(status);
			return true;
		}

		_geofence_rtl_active = false;
		_geofence_resume_pending = true;
		_geofence_clear_time = now;
		PX4_INFO("cooperative rendezvous: geofence clear, preparing guidance resume");
	}

	return false;
}

void CooperativeRendezvous::request_arm(const vehicle_status_s &status)
{
	if (!_options.auto_arm || status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (now - _last_arm_request < 1_s) {
		return;
	}

	vehicle_command_s command{};
	command.timestamp = now;
	command.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
	command.param1 = 1.f;
	command.target_system = status.system_id;
	command.target_component = status.component_id;
	command.source_system = status.system_id;
	command.source_component = status.component_id;
	command.from_external = false;
	_vehicle_command_pub.publish(command);
	_last_arm_request = now;
}

void CooperativeRendezvous::configure_relaxed_failsafes()
{
	if (!_options.relax_failsafes || _failsafes_configured) {
		return;
	}

	struct ParamSetInt {
		const char *name;
		int32_t value;
	};

	struct ParamSetFloat {
		const char *name;
		float value;
	};

	const ParamSetInt int_params[] = {
		{"NAV_DLL_ACT", 0},     // GCS datalink loss: disabled
		{"COM_DLL_EXCEPT", 7},  // ignore datalink loss in Mission/Hold/Offboard
		{"COM_RC_IN_MODE", 3},  // keep SITL RC/joystick input enabled instead of disabling sticks
		{"NAV_RCL_ACT", 0},     // custom communication emergency module owns link-loss handling
		{"COM_RCL_EXCEPT", 7},  // ignore RC loss in Mission/Hold/Offboard
		{"COM_OBL_RC_ACT", 5},  // offboard loss fallback: Hold
	};

	const ParamSetFloat float_params[] = {
		{"COM_OF_LOSS_T", 5.f},
	};

	for (const ParamSetInt &item : int_params) {
		const param_t handle = param_find(item.name);

		if (handle != PARAM_INVALID) {
			param_set(handle, &item.value);
		}
	}

	for (const ParamSetFloat &item : float_params) {
		const param_t handle = param_find(item.name);

		if (handle != PARAM_INVALID) {
			param_set(handle, &item.value);
		}
	}

	_failsafes_configured = true;
	PX4_WARN("cooperative_rendezvous: relaxed simulation failsafes enabled");
}

void CooperativeRendezvous::hold_position(const vehicle_local_position_s &local_pos)
{
	keep_current_position_setpoint(local_pos);
}

void CooperativeRendezvous::keep_current_position_setpoint(const vehicle_local_position_s &local_pos)
{
	const matrix::Vector3f position(local_pos.x, local_pos.y, local_pos.z);
	const matrix::Vector3f velocity(0.f, 0.f, 0.f);

	publish_offboard_heartbeat(true, false);
	publish_trajectory_setpoint(position, velocity, local_pos.heading);
}

void CooperativeRendezvous::run_rendezvous(const vehicle_local_position_s &local_pos, const vehicle_status_s &status)
{
	matrix::Vector3f target_position{};
	matrix::Vector3f target_velocity{};
	float yaw = local_pos.heading;

	if (gcs_setpoint_active(local_pos, target_position, target_velocity, yaw)) {
		publish_offboard_heartbeat(true, false);
		publish_trajectory_setpoint(target_position, target_velocity, yaw);
		reset_velocity_slew();
		reset_target_filter();
		reset_target_history();
		reset_arrival_hold();

		request_arm(status);

		if (status.nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION &&
		    status.nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER &&
		    status.nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_RTL &&
		    status.nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) {
			request_offboard(status);
		}

		return;
	}

	if (!target_state_local(local_pos, target_position, target_velocity)) {
		const hrt_abstime now = hrt_absolute_time();

		if (now - _last_status_log > 2_s) {
			if (_last_target_time == 0) {
				PX4_WARN("cooperative rendezvous: waiting for target=%" PRIu32 " position", _options.target_id);

			} else {
				const double age_s = (double)(now - _last_target_time) * 1e-6;
				PX4_WARN("cooperative rendezvous: target=%" PRIu32 " position stale age=%.1fs",
					 _options.target_id, age_s);
			}

			_last_status_log = now;
		}

		hold_position(local_pos);
		reset_velocity_slew();
		reset_arrival_hold();
		return;
	}

	matrix::Vector3f current_position(local_pos.x, local_pos.y, local_pos.z);
	yaw = PX4_ISFINITE(_target_info.yaw) ? static_cast<float>(_target_info.yaw) : local_pos.heading;
	const bool arrival_holding = update_arrival_hold(target_position, target_velocity, local_pos, yaw);
	matrix::Vector3f to_target = target_position - current_position;
	const float distance = to_target.norm();
	matrix::Vector3f velocity_sp = target_velocity;
	matrix::Vector2f horizontal_error(to_target(0), to_target(1));
	const float horizontal_distance = horizontal_error.norm();
	const float configured_speed = _param_app_speed.get();
	const float closing_speed = PX4_ISFINITE(configured_speed) && configured_speed > 0.f ?
				     math::constrain(configured_speed, 0.5f, 80.f) :
				     math::constrain(_options.max_speed, 0.5f, 80.f);
	const float slow_radius = math::max(_param_slow_radius.get(), 0.5f);

	if (!arrival_holding && horizontal_distance > 0.5f) {
		const float speed_scale = math::constrain(horizontal_distance / slow_radius, 0.f, 1.f);
		const float approach_speed = closing_speed * speed_scale;
		const matrix::Vector2f approach_xy = horizontal_error / horizontal_distance * approach_speed;
		velocity_sp(0) += approach_xy(0);
		velocity_sp(1) += approach_xy(1);
	}

	if (arrival_holding) {
		reset_velocity_slew();

	} else {
		slew_horizontal_velocity(velocity_sp, local_pos);
	}

	publish_offboard_heartbeat(true, false);
	publish_trajectory_setpoint(target_position, velocity_sp, yaw);

	request_arm(status);

	if (_geofence_resume_pending) {
		request_offboard(status);

		if (status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD && status.timestamp != 0 &&
		    hrt_elapsed_time(&status.timestamp) < 1_s) {
			_geofence_resume_pending = false;
			_geofence_clear_time = 0;
		}

	} else if (status.nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION &&
	    status.nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER &&
	    status.nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_RTL &&
	    status.nav_state != vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) {
		request_offboard(status);
	}

	const hrt_abstime now = hrt_absolute_time();

	if (now - _last_status_log > 2_s) {
		PX4_INFO("cooperative rendezvous: target=%" PRIu32 " distance=%.1fm app_spd=%.1fm/s hold=%d setpoint=(%.1f %.1f %.1f)",
			 _options.target_id, (double)distance,
			 (double)closing_speed, arrival_holding,
			 (double)target_position(0), (double)target_position(1), (double)target_position(2));
		_last_status_log = now;
	}
}

void CooperativeRendezvous::slew_horizontal_velocity(matrix::Vector3f &velocity_sp,
		const vehicle_local_position_s &local_pos)
{
	const float slew_rate = _param_velocity_slew.get();

	if (!PX4_ISFINITE(slew_rate) || slew_rate <= 0.f) {
		reset_velocity_slew();
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	matrix::Vector2f target_velocity_xy(velocity_sp(0), velocity_sp(1));

	if (!PX4_ISFINITE(target_velocity_xy(0)) || !PX4_ISFINITE(target_velocity_xy(1))) {
		target_velocity_xy.zero();
	}

	if (_last_velocity_slew_time == 0) {
		_last_velocity_sp_xy(0) = PX4_ISFINITE(local_pos.vx) ? local_pos.vx : 0.f;
		_last_velocity_sp_xy(1) = PX4_ISFINITE(local_pos.vy) ? local_pos.vy : 0.f;
		_last_velocity_slew_time = now;
	}

	if (!PX4_ISFINITE(_last_velocity_sp_xy(0)) || !PX4_ISFINITE(_last_velocity_sp_xy(1))) {
		_last_velocity_sp_xy.zero();
	}

	const float dt = math::constrain((now - _last_velocity_slew_time) * 1e-6f, 0.005f, 0.2f);
	const float max_delta = math::constrain(slew_rate, 0.1f, 20.f) * dt;
	const matrix::Vector2f delta = target_velocity_xy - _last_velocity_sp_xy;

	if (delta.norm() > max_delta) {
		target_velocity_xy = _last_velocity_sp_xy + delta.normalized() * max_delta;
	}

	_last_velocity_sp_xy = target_velocity_xy;
	_last_velocity_slew_time = now;
	velocity_sp(0) = target_velocity_xy(0);
	velocity_sp(1) = target_velocity_xy(1);
}

void CooperativeRendezvous::reset_velocity_slew()
{
	_last_velocity_slew_time = 0;
	_last_velocity_sp_xy.zero();
}

void CooperativeRendezvous::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	update_params_if_needed();
	update_vehicle_id();
	configure_relaxed_failsafes();

	vehicle_local_position_s local_pos{};
	vehicle_status_s status{};
	_vehicle_local_position_sub.copy(&local_pos);
	_vehicle_status_sub.copy(&status);
	_trajectory_publication_allowed = offboard_control_active(status);
	_manual_control_sub.update(&_manual_control);
	_dyt_guidance_status_sub.update(&_dyt_guidance_status);
	_geofence_result_sub.update(&_geofence_result);

	if (status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		_gcs_midcourse_engaged = false;

	} else if (dyt_status_fresh() &&
		   _dyt_guidance_status.gcs_phase_request == dyt_guidance_status_s::PHASE_MIDCOURSE) {
		_gcs_midcourse_engaged = true;
	}

	const bool position_valid = local_position_valid(local_pos);

	if (position_valid) {
		publish_own_position(local_pos);
	}

	update_target_from_link();
	update_gcs_setpoint();

	if (!position_valid) {
		publish_status(status, false, false);
		return;
	}

	if (active_role() == Role::Rendezvous) {
		if (!rendezvous_switch_enabled()) {
			_geofence_rtl_active = false;
			_geofence_resume_pending = false;
			_geofence_clear_time = 0;
			reset_velocity_slew();
			reset_target_filter();
			reset_target_history();
			reset_arrival_hold();
			publish_status(status, true, false);
			return;
		}

		if (geofence_avoidance_required(status) || dyt_guidance_active()) {
			reset_velocity_slew();
			reset_target_filter();
			reset_target_history();
			reset_arrival_hold();
			publish_status(status, true, false);
			return;
		}

		if (!offboard_control_active(status)) {
			// Pre-stream only the Offboard heartbeat while changing modes. Publishing a
			// trajectory here would race the active FlightTask on the shared uORB topic.
			if (_geofence_resume_pending) {
				// A geofence clear and the Commander failsafe state are not published in the
				// same cycle. Keep the heartbeat alive while Commander releases RTL, but do
				// not request Offboard until the failsafe is clear and a target is fresh.
				if (vehicle_status_fresh(status)
				    && status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
					publish_offboard_heartbeat(true, false);
					const hrt_abstime now = hrt_absolute_time();

					if (!status.failsafe && target_data_fresh() && _geofence_clear_time != 0
					    && _geofence_clear_time <= now && now - _geofence_clear_time >= 1_s) {
						request_offboard(status);
					}
				}

			} else if (offboard_preparation_allowed(status)) {
				publish_offboard_heartbeat(true, false);
				request_arm(status);
				request_offboard(status);
			}

			reset_velocity_slew();
			reset_target_filter();
			reset_arrival_hold();
			publish_status(status, true, false);
			return;
		}

		// Offboard is now the confirmed trajectory owner. Clearing this latch here
		// also covers the GCS-target branch in run_rendezvous().
		_geofence_resume_pending = false;
		_geofence_clear_time = 0;

		run_rendezvous(local_pos, status);
		publish_status(status, true, true);

	} else if (active_role() == Role::Broadcast && status.arming_state == vehicle_status_s::ARMING_STATE_ARMED &&
		   rendezvous_switch_enabled() && !dyt_guidance_active()) {
		reset_velocity_slew();
		reset_target_filter();
		reset_target_history();
		reset_arrival_hold();

		if (offboard_control_active(status)) {
			keep_current_position_setpoint(local_pos);
			publish_status(status, true, true);

		} else {
			if (offboard_preparation_allowed(status)) {
				publish_offboard_heartbeat(true, false);
			}

			publish_status(status, true, false);
		}

	} else {
		reset_velocity_slew();
		reset_target_filter();
		reset_target_history();
		reset_arrival_hold();
		publish_status(status, true, false);
	}
}

int CooperativeRendezvous::print_status()
{
	const char *role = "auto";

	switch (active_role()) {
	case Role::Broadcast:
		role = "broadcast";
		break;

	case Role::Rendezvous:
		role = "rendezvous";
		break;

	case Role::Auto:
		break;
	}

	PX4_INFO("running: vehicle=%" PRIu32 " role=%s target=%" PRIu32 " offset=(%.1f %.1f %.1f) xy_en=%ld xy=(%.1f %.1f) dist=%.1f app_spd=%.1f slow=%.1f vslew=%.1f tpos_tc=%.2f tvel_tc=%.2f tpos_jmp=%.1f alt_diff=%.1f",
		 _vehicle_id, role, _options.target_id,
		 (double)_options.target_offset(0),
		 (double)_options.target_offset(1),
		 (double)_options.target_offset(2),
		 static_cast<long>(_param_xy_offset_enable.get()),
		 (double)_param_x_offset.get(),
		 (double)_param_y_offset.get(),
		 (double)_param_dist.get(),
		 (double)_param_app_speed.get(),
		 (double)_param_slow_radius.get(),
		 (double)_param_velocity_slew.get(),
		 (double)_param_target_position_tc.get(),
		 (double)_param_target_velocity_tc.get(),
		 (double)_param_target_position_jump.get(),
		 (double)_param_alt_diff.get());
	PX4_INFO("activation aux=%d enabled=%d dyt_active=%d",
		 static_cast<int>(_param_act_aux.get()), rendezvous_switch_enabled(), dyt_guidance_active());
	PX4_INFO("activation button=%d buttons=0x%04x",
		 static_cast<int>(_param_act_btn.get()), static_cast<unsigned>(_manual_control.buttons));
	PX4_INFO("geofence: rtl=%d resume=%d result_age=%.1f s custom_triggered=%d",
		 _geofence_rtl_active, _geofence_resume_pending,
		 _geofence_result.timestamp == 0 ? -1.0 : (double)hrt_elapsed_time(&_geofence_result.timestamp) * 1e-6,
		 _geofence_result.geofence_custom_fence_triggered);
	PX4_INFO("altitude: reference=AMSL max_error=%.1f m diff=%.1f m",
		 (double)_param_max_altitude_error.get(),
		 (double)_param_alt_diff.get());
	PX4_INFO("feature switches: gcs=%ld min_height=%ld history=%ld",
		 static_cast<long>(_param_gcs_enable.get()),
		 static_cast<long>(_param_minimum_height_enable.get()),
		 static_cast<long>(_param_history_enable.get()));
	PX4_INFO("arrival hold: en=%ld hold=%d follow=%d mode=%ld hrad=%.1f vrad=%.1f vel=%.2f time=%.1f rel=%.1f",
		 static_cast<long>(_param_arrival_hold_enable.get()), _arrival_hold_active, _arrival_follow_active,
		 static_cast<long>(_param_arrival_hold_mode.get()),
		 (double)_param_arrival_hold_horizontal_radius.get(),
		 (double)_param_arrival_hold_vertical_radius.get(),
		 (double)_param_arrival_hold_velocity.get(), (double)_param_arrival_hold_time.get(),
		 (double)_param_arrival_hold_release.get());
	return 0;
}

static bool parse_role(const char *arg, CooperativeRendezvous::Role &role)
{
	if (!strcmp(arg, "auto")) {
		role = CooperativeRendezvous::Role::Auto;
		return true;
	}

	if (!strcmp(arg, "broadcast")) {
		role = CooperativeRendezvous::Role::Broadcast;
		return true;
	}

	if (!strcmp(arg, "rendezvous")) {
		role = CooperativeRendezvous::Role::Rendezvous;
		return true;
	}

	return false;
}

int CooperativeRendezvous::task_spawn(int argc, char *argv[])
{
	Options options{};
	bool error_flag = false;
	bool offset_x_set = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "r:t:d:x:y:z:v:T:AFh", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'r':
			if (!parse_role(myoptarg, options.role)) {
				error_flag = true;
			}
			break;

		case 't':
			options.target_id = strtoul(myoptarg, nullptr, 10);
			break;

		case 'd':
			if (!offset_x_set) {
				options.target_offset(0) = -fabsf(strtof(myoptarg, nullptr));
			}
			break;

		case 'x':
			options.target_offset(0) = strtof(myoptarg, nullptr);
			offset_x_set = true;
			break;

		case 'y':
			options.target_offset(1) = strtof(myoptarg, nullptr);
			break;

		case 'z':
			options.target_offset(2) = strtof(myoptarg, nullptr);
			break;

		case 'v':
			options.max_speed = math::constrain(strtof(myoptarg, nullptr), 0.5f, 50.f);
			break;

		case 'T':
			options.target_timeout_s = math::constrain(strtof(myoptarg, nullptr), 0.5f, 10.f);
			break;

		case 'A':
			options.auto_arm = true;
			break;

		case 'F':
			options.relax_failsafes = true;
			break;

		case 'h':
		case '?':
		default:
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return PX4_ERROR;
	}

	CooperativeRendezvous *instance = new CooperativeRendezvous(options);

	if (instance) {
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

int CooperativeRendezvous::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CooperativeRendezvous::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Two-aircraft cooperative rendezvous module.

MAV_SYS_ID=1 broadcasts its local position converted to WGS84.
MAV_SYS_ID=2 flies to a configurable offset near aircraft 1.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("cooperative_rendezvous", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('r', "auto", "auto|broadcast|rendezvous", "Role selection", true);
	PRINT_MODULE_USAGE_PARAM_INT('t', 1, 1, 255, "Target MAV_SYS_ID for rendezvous", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('d', 5.f, 0.f, 100.f, "Distance behind target when x offset is not set", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('x', -5.f, -100.f, 100.f, "Target NED x offset", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('y', 0.f, -100.f, 100.f, "Target NED y offset", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('z', 0.f, -50.f, 50.f, "Target NED z offset", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('v', 3.f, 0.5f, 50.f, "Maximum approach speed", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('T', 2.f, 0.5f, 10.f, "Target timeout", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('A', "Auto arm rendezvous aircraft", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('F', "Relax link/manual/offboard failsafes for simulation", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int cooperative_rendezvous_main(int argc, char *argv[])
{
	return CooperativeRendezvous::main(argc, argv);
}
