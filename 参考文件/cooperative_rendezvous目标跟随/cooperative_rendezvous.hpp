#pragma once

#include <lib/geo/geo.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/cooperative_position.h>
#include <uORB/topics/cooperative_rendezvous_status.h>
#include <uORB/topics/dyt_guidance_status.h>
#include <uORB/topics/follower_info.h>
#include <uORB/topics/gcs_trajectory_setpoint.h>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include <math.h>

using namespace time_literals;

class CooperativeRendezvous : public ModuleBase<CooperativeRendezvous>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	enum class Role {
		Auto,
		Broadcast,
		Rendezvous
	};

	struct Options {
		Role role{Role::Auto};
		uint32_t target_id{1};
		matrix::Vector3f target_offset{-5.f, 0.f, 0.f};
		float max_speed{3.f};
		float target_timeout_s{2.f};
		bool auto_arm{false};
		bool auto_offboard{true};
		bool relax_failsafes{false};
	};

	CooperativeRendezvous(const Options &options);
	~CooperativeRendezvous() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;

	void update_params_if_needed();
	void update_vehicle_id();
	Role active_role() const;
	float aux_value(int index) const;
	bool aux_switch_active(int index) const;
	bool button_active(int button) const;
	bool rendezvous_switch_enabled() const;
	bool dyt_status_fresh() const;
	bool dyt_guidance_active() const;
	bool vehicle_status_fresh(const vehicle_status_s &status) const;
	bool protected_navigation_state(uint8_t nav_state) const;
	bool offboard_control_active(const vehicle_status_s &status) const;
	bool offboard_preparation_allowed(const vehicle_status_s &status) const;
	bool target_data_fresh() const;
	void publish_status(const vehicle_status_s &status, bool local_position_is_valid, bool controlling_vehicle);
	bool local_position_valid(const vehicle_local_position_s &local_pos) const;
	bool update_map_projection(const vehicle_local_position_s &local_pos);
	void publish_own_position(const vehicle_local_position_s &local_pos);
	bool update_target_from_link();
	void update_gcs_setpoint();
	bool gcs_setpoint_active(const vehicle_local_position_s &local_pos, matrix::Vector3f &target_position,
				 matrix::Vector3f &target_velocity, float &yaw);
	void enforce_target_minimum_height(matrix::Vector3f &target_position) const;
	void push_target_history(const matrix::Vector3f &target_position);
	bool delayed_target_position(matrix::Vector3f &target_position) const;
	void reset_target_history();
	bool target_state_local(const vehicle_local_position_s &local_pos, matrix::Vector3f &target_position,
				matrix::Vector3f &target_velocity);
	void apply_target_filter(const matrix::Vector3f &raw_position, const matrix::Vector3f &raw_velocity,
				 matrix::Vector3f &target_position, matrix::Vector3f &target_velocity);
	void reset_target_filter();
	bool update_arrival_hold(matrix::Vector3f &target_position, matrix::Vector3f &target_velocity,
				 const vehicle_local_position_s &local_pos, float &yaw);
	void reset_arrival_hold();
	void run_rendezvous(const vehicle_local_position_s &local_pos, const vehicle_status_s &status);
	void slew_horizontal_velocity(matrix::Vector3f &velocity_sp, const vehicle_local_position_s &local_pos);
	void reset_velocity_slew();
	void keep_current_position_setpoint(const vehicle_local_position_s &local_pos);
	void publish_offboard_heartbeat(bool position_control, bool velocity_control);
	void publish_trajectory_setpoint(const matrix::Vector3f &position, const matrix::Vector3f &velocity, float yaw);
	void request_offboard(const vehicle_status_s &status);
	void request_rtl(const vehicle_status_s &status);
	void request_arm(const vehicle_status_s &status);
	void hold_position(const vehicle_local_position_s &local_pos);
	void configure_relaxed_failsafes();
	bool geofence_avoidance_required(const vehicle_status_s &status);

	struct TargetHistorySample {
		hrt_abstime timestamp{0};
		matrix::Vector3f position{};
	};

	Options _options{};
	uint32_t _vehicle_id{0};
	bool _vehicle_id_initialized{false};
	bool _map_ref_initialized{false};
	MapProjection _map_ref{};

	follower_info_s _target_info{};
	hrt_abstime _last_target_time{0};
	hrt_abstime _last_mode_request{0};
	hrt_abstime _last_rtl_request{0};
	hrt_abstime _last_arm_request{0};
	hrt_abstime _last_status_log{0};
	hrt_abstime _last_velocity_slew_time{0};
	hrt_abstime _last_target_filter_time{0};
	hrt_abstime _last_target_filter_sample_time{0};
	hrt_abstime _last_gcs_setpoint_time{0};
	bool _failsafes_configured{false};
	bool _target_filter_initialized{false};
	bool _gcs_target_active{false};
	bool _arrival_hold_active{false};
	bool _arrival_follow_active{false};
	bool _geofence_rtl_active{false};
	bool _geofence_resume_pending{false};
	hrt_abstime _geofence_clear_time{0};
	bool _trajectory_publication_allowed{false};
	bool _gcs_midcourse_engaged{false};
	hrt_abstime _arrival_hold_candidate_since{0};
	int _target_history_head{0};
	int _target_history_count{0};
	matrix::Vector2f _last_velocity_sp_xy{};
	matrix::Vector3f _target_position_input{};
	matrix::Vector3f _target_position_filtered{};
	matrix::Vector3f _target_velocity_input{};
	matrix::Vector3f _target_velocity_filtered{};
	matrix::Vector3f _arrival_hold_position{};
	gcs_trajectory_setpoint_s _gcs_setpoint{};
	float _arrival_hold_yaw{NAN};
	static constexpr float kMaxTargetHistoryDurationS = 10.f;
	static constexpr int kTargetHistoryLength = 220;
	TargetHistorySample _target_history[kTargetHistoryLength]{};

	manual_control_setpoint_s _manual_control{};
	dyt_guidance_status_s _dyt_guidance_status{};
	geofence_result_s _geofence_result{};

	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _follower_info_sub{ORB_ID(follower_info)};
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _dyt_guidance_status_sub{ORB_ID(dyt_guidance_status)};
	uORB::Subscription _gcs_trajectory_setpoint_sub{ORB_ID(gcs_trajectory_setpoint)};
	uORB::Subscription _geofence_result_sub{ORB_ID(geofence_result)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<cooperative_position_s> _cooperative_position_pub{ORB_ID(cooperative_position)};
	uORB::Publication<cooperative_rendezvous_status_s> _status_pub{ORB_ID(cooperative_rendezvous_status)};
	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CRDZ_ACT_AUX>) _param_act_aux,
		(ParamInt<px4::params::CRDZ_ACT_BTN>) _param_act_btn,
		(ParamFloat<px4::params::CRDZ_DIST>) _param_dist,
		(ParamInt<px4::params::CRDZ_XY_OFF_EN>) _param_xy_offset_enable,
		(ParamFloat<px4::params::CRDZ_X_OFF>) _param_x_offset,
		(ParamFloat<px4::params::CRDZ_Y_OFF>) _param_y_offset,
		(ParamFloat<px4::params::CRDZ_APP_SPD>) _param_app_speed,
		(ParamFloat<px4::params::CRDZ_SLOW_RAD>) _param_slow_radius,
		(ParamInt<px4::params::CRDZ_HOLD_EN>) _param_arrival_hold_enable,
		(ParamInt<px4::params::CRDZ_HOLD_MODE>) _param_arrival_hold_mode,
		(ParamFloat<px4::params::CRDZ_HOLD_HRAD>) _param_arrival_hold_horizontal_radius,
		(ParamFloat<px4::params::CRDZ_HOLD_VRAD>) _param_arrival_hold_vertical_radius,
		(ParamFloat<px4::params::CRDZ_HOLD_VEL>) _param_arrival_hold_velocity,
		(ParamFloat<px4::params::CRDZ_HOLD_T>) _param_arrival_hold_time,
		(ParamFloat<px4::params::CRDZ_HOLD_REL>) _param_arrival_hold_release,
		(ParamFloat<px4::params::CRDZ_VSLEW>) _param_velocity_slew,
		(ParamFloat<px4::params::CRDZ_TPOS_TC>) _param_target_position_tc,
		(ParamFloat<px4::params::CRDZ_TVEL_TC>) _param_target_velocity_tc,
		(ParamFloat<px4::params::CRDZ_TPOS_JMP>) _param_target_position_jump,
		(ParamFloat<px4::params::CRDZ_ALT_ERR>) _param_max_altitude_error,
		(ParamFloat<px4::params::CRDZ_ALT_DIFF>) _param_alt_diff,
		(ParamInt<px4::params::CRDZ_GCS_EN>) _param_gcs_enable,
		(ParamFloat<px4::params::CRDZ_GCS_TOUT>) _param_gcs_timeout,
		(ParamFloat<px4::params::CRDZ_TGT_TOUT>) _param_target_timeout,
		(ParamInt<px4::params::CRDZ_MINH_EN>) _param_minimum_height_enable,
		(ParamFloat<px4::params::CRDZ_MIN_HGT>) _param_minimum_height,
		(ParamInt<px4::params::CRDZ_HIST_EN>) _param_history_enable,
		(ParamFloat<px4::params::CRDZ_HIST_T>) _param_history_duration,
		(ParamFloat<px4::params::CRDZ_TRK_DLY>) _param_track_delay
	)
};
