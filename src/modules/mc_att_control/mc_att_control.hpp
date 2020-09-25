/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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

#include <lib/mixer/mixer.h>
#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/vehicle_estimate_attitude.h>
#include <uORB/topics/estimate_disturb.h>
#include <uORB/topics/vehicle_tsmc_attitude.h>
#include <uORB/topics/actuator_outputs.h>

#include <vtol_att_control/vtol_type.h>


#include <AttitudeControl.hpp>
#include <RateControl.hpp>

/**
 * Multicopter attitude control app start / stop handling function
 */
extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[]);

class MulticopterAttitudeControl : public ModuleBase<MulticopterAttitudeControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	MulticopterAttitudeControl();

	virtual ~MulticopterAttitudeControl();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void Run() override;

	bool init();

private:

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void		parameters_updated();

	/**
	 * Check for parameter update and handle it.
	 */
	void		parameter_update_poll();
	bool		vehicle_attitude_poll();
	void		vehicle_motor_limits_poll();
	void		vehicle_status_poll();

	void		publish_actuator_controls();
	void		publish_rates_setpoint();
	void		publish_rate_controller_status();

	float		throttle_curve(float throttle_stick_input);

	/**
	 * Generate & publish an attitude setpoint from stick inputs
	 */
	void		generate_attitude_setpoint(float dt, bool reset_yaw_sp);

	/**
	 * Get the landing gear state based on the manual control switch position
	 * @return vehicle_attitude_setpoint_s::LANDING_GEAR_UP or vehicle_attitude_setpoint_s::LANDING_GEAR_DOWN
	 */
	float		get_landing_gear_state();


	/**
	 * Attitude controller.
	 */
	void		control_attitude();

	/**
	 * Attitude rates controller.
	 */
	void		control_attitude_rates(float dt, const matrix::Vector3f &rates);


	// 子函数


	matrix::Vector3f nolinear_part_TSMC(matrix::Vector3f rates_err, matrix::Vector3f alpha_TSMC);
	void control_attitude_motorloss_smc(float dt, const matrix::Vector3f &rates);
	void control_attitude_rates_motorloss_smc(float dt, const matrix::Vector3f &rates);

	void get_funcg_integPart(matrix::Vector3f c_TSMC, matrix::Vector3f alpha_TSMC, matrix::Vector3f rates_err, const float dt, matrix::Vector3f &x1);

	void get_un_TSMC(matrix::Vector3f T_TSMC, matrix::Vector3f eta_TSMC, matrix::Vector3f sign_S, const float dt, matrix::Vector3f &x1, matrix::Vector3f &x2);

	void estimator_update(matrix::Vector3f x, const float dt, matrix::Vector3f &x1, matrix::Vector3f &x2);

	void estimator_model(matrix::Vector3f &rates, matrix::Vector3f &x1, matrix::Vector3f &x2,
						matrix::Vector3f &x1_dot, matrix::Vector3f &x2_dot);
	void un_model(matrix::Vector3f &T_TSMC, matrix::Vector3f &eta_TSMC, matrix::Vector3f &sign_S, matrix::Vector3f &x1, matrix::Vector3f &x2,
	                      matrix::Vector3f &x1_dot, matrix::Vector3f &x2_dot);
	void funcg_model(matrix::Vector3f &c_TSMC, matrix::Vector3f &alpha_TSMC, matrix::Vector3f &rates_err, matrix::Vector3f &x1, matrix::Vector3f &x1_dot);

	void print_vector3(const char *name, matrix::Vector3f &vector3, bool print_time = false);
	inline float motor_max_thrust(float battery_voltage);
	/**
	 * Transform computed torques to equivalent normalized inputs
	 */
	matrix::Vector3f torque_to_attctrl(matrix::Vector3f &torque);

	/**
	 * Used for control when there is loss of single motor( motor 4)
	 */
	matrix::Vector3f torque_to_attctrl_motorloss(matrix::Vector3f &torque);

	/**
	 * Torque around 3-axis generated by four motor thrusts
	 */
	matrix::Vector3f compute_actuate_torque();


	AttitudeControl _attitude_control; ///< class for attitude control calculations
	RateControl _rate_control; ///< class for rate control calculations

	uORB::Subscription _v_att_sub{ORB_ID(vehicle_attitude)};			/**< vehicle attitude subscription */
	uORB::Subscription _v_att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};		/**< vehicle attitude setpoint subscription */
	uORB::Subscription _v_rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};		/**< vehicle rates setpoint subscription */
	uORB::Subscription _v_control_mode_sub{ORB_ID(vehicle_control_mode)};		/**< vehicle control mode subscription */
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};		/**< parameter updates subscription */
	uORB::Subscription _manual_control_sp_sub{ORB_ID(manual_control_setpoint)};	/**< manual control setpoint subscription */
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};			/**< vehicle status subscription */
	uORB::Subscription _motor_limits_sub{ORB_ID(multirotor_motor_limits)};		/**< motor limits subscription */
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};			/**< battery status subscription */
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};	/**< vehicle land detected subscription */
	uORB::Subscription _landing_gear_sub{ORB_ID(landing_gear)};

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::PublicationMulti<rate_ctrl_status_s>	_controller_status_pub{ORB_ID(rate_ctrl_status), ORB_PRIO_DEFAULT};	/**< controller status publication */
	uORB::Publication<landing_gear_s>		_landing_gear_pub{ORB_ID(landing_gear)};
	uORB::Publication<vehicle_rates_setpoint_s>	_v_rates_sp_pub{ORB_ID(vehicle_rates_setpoint)};			/**< rate setpoint publication */

	uORB::Subscription _actuator_outputs_sub{ORB_ID(actuator_outputs)};	/**< actuator outputs subscription */

	orb_advert_t	_v_esti_att_pub{nullptr};		/**< estimated attitude speed/acc publication */
	orb_advert_t	_ext_torque_pub{nullptr};		/**< estimated external torque disturbance publication*/
	orb_advert_t	_v_TSMC_att_pub{nullptr};		/**< TSMC intermediate variables publication*/

	orb_advert_t	_actuators_0_pub{nullptr};		/**< attitude actuator controls publication */
	orb_advert_t	_vehicle_attitude_setpoint_pub{nullptr};

	orb_id_t _actuators_id{nullptr};	/**< pointer to correct actuator controls0 uORB metadata structure */
	orb_id_t _attitude_sp_id{nullptr};	/**< pointer to correct attitude setpoint uORB metadata structure */

	bool		_actuators_0_circuit_breaker_enabled{false};	/**< circuit breaker to suppress output */

	struct vehicle_attitude_s		_v_att {};		/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s	_v_att_sp {};		/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s		_v_rates_sp {};		/**< vehicle rates setpoint */
	struct manual_control_setpoint_s	_manual_control_sp {};	/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode {};	/**< vehicle control mode */
	struct actuator_controls_s		_actuators {};		/**< actuator controls */
	struct vehicle_status_s			_vehicle_status {};	/**< vehicle status */
	struct battery_status_s			_battery_status {};	/**< battery status */
	struct vehicle_land_detected_s		_vehicle_land_detected {};
	struct landing_gear_s 			_landing_gear {};

	struct vehicle_estimate_attitude_s  _v_esti_att {};		/**< estimated attitude speed/acc */
	struct estimate_disturb_s		_ext_torque {};		/** estimated external torque disturbance */
	struct vehicle_tsmc_attitude_s		_v_TSMC_att {};		/** TSMC intermediate variables */

	struct actuator_outputs_s		_actuator_outputs {};		/**< actuator outputs to motors */

	MultirotorMixer::saturation_status _saturation_status{};

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	static constexpr const float initial_update_rate_hz = 250.f; /**< loop update rate used for initialization */
	float _loop_update_rate_hz{initial_update_rate_hz};          /**< current rate-controller loop update rate in [Hz] */

	matrix::Vector3f _rates_sp;			/**< angular rates setpoint */

	matrix::Vector3f _att_control;			/**< attitude control vector */
	float		_thrust_sp{0.0f};		/**< thrust setpoint */

	float _man_yaw_sp{0.f};				/**< current yaw setpoint in manual mode */
	bool _gear_state_initialized{false};		/**< true if the gear state has been initialized */

	hrt_abstime _task_start{hrt_absolute_time()};
	hrt_abstime _last_run{0};
	float _dt_accumulator{0.0f};
	int _loop_counter{0};

	bool _reset_yaw_sp{true};
	float _attitude_dt{0.0f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_ROLL_P>) _param_mc_roll_p,
		(ParamFloat<px4::params::MC_ROLLRATE_P>) _param_mc_rollrate_p,
		(ParamFloat<px4::params::MC_ROLLRATE_I>) _param_mc_rollrate_i,
		(ParamFloat<px4::params::MC_RR_INT_LIM>) _param_mc_rr_int_lim,
		(ParamFloat<px4::params::MC_ROLLRATE_D>) _param_mc_rollrate_d,
		(ParamFloat<px4::params::MC_ROLLRATE_FF>) _param_mc_rollrate_ff,
		(ParamFloat<px4::params::MC_ROLLRATE_K>) _param_mc_rollrate_k,

		(ParamFloat<px4::params::MC_PITCH_P>) _param_mc_pitch_p,
		(ParamFloat<px4::params::MC_PITCHRATE_P>) _param_mc_pitchrate_p,
		(ParamFloat<px4::params::MC_PITCHRATE_I>) _param_mc_pitchrate_i,
		(ParamFloat<px4::params::MC_PR_INT_LIM>) _param_mc_pr_int_lim,
		(ParamFloat<px4::params::MC_PITCHRATE_D>) _param_mc_pitchrate_d,
		(ParamFloat<px4::params::MC_PITCHRATE_FF>) _param_mc_pitchrate_ff,
		(ParamFloat<px4::params::MC_PITCHRATE_K>) _param_mc_pitchrate_k,

		(ParamFloat<px4::params::MC_YAW_P>) _param_mc_yaw_p,
		(ParamFloat<px4::params::MC_YAWRATE_P>) _param_mc_yawrate_p,
		(ParamFloat<px4::params::MC_YAWRATE_I>) _param_mc_yawrate_i,
		(ParamFloat<px4::params::MC_YR_INT_LIM>) _param_mc_yr_int_lim,
		(ParamFloat<px4::params::MC_YAWRATE_D>) _param_mc_yawrate_d,
		(ParamFloat<px4::params::MC_YAWRATE_FF>) _param_mc_yawrate_ff,
		(ParamFloat<px4::params::MC_YAWRATE_K>) _param_mc_yawrate_k,

		(ParamFloat<px4::params::MC_DTERM_CUTOFF>) _param_mc_dterm_cutoff,			/**< Cutoff frequency for the D-term filter */

		(ParamFloat<px4::params::MC_ROLLRATE_MAX>) _param_mc_rollrate_max,
		(ParamFloat<px4::params::MC_PITCHRATE_MAX>) _param_mc_pitchrate_max,
		(ParamFloat<px4::params::MC_YAWRATE_MAX>) _param_mc_yawrate_max,
		(ParamFloat<px4::params::MPC_MAN_Y_MAX>) _param_mpc_man_y_max,			/**< scaling factor from stick to yaw rate */

		(ParamFloat<px4::params::MC_ACRO_R_MAX>) _param_mc_acro_r_max,
		(ParamFloat<px4::params::MC_ACRO_P_MAX>) _param_mc_acro_p_max,
		(ParamFloat<px4::params::MC_ACRO_Y_MAX>) _param_mc_acro_y_max,
		(ParamFloat<px4::params::MC_ACRO_EXPO>) _param_mc_acro_expo,				/**< expo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_EXPO_Y>) _param_mc_acro_expo_y,				/**< expo stick curve shape (yaw) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPO>) _param_mc_acro_supexpo,			/**< superexpo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPOY>) _param_mc_acro_supexpoy,			/**< superexpo stick curve shape (yaw) */

		(ParamFloat<px4::params::MC_RATT_TH>) _param_mc_ratt_th,

		(ParamBool<px4::params::MC_BAT_SCALE_EN>) _param_mc_bat_scale_en,

		/* Stabilized mode params */
		(ParamFloat<px4::params::MPC_MAN_TILT_MAX>) _param_mpc_man_tilt_max,			/**< maximum tilt allowed for manual flight */
		(ParamFloat<px4::params::MPC_MANTHR_MIN>) _param_mpc_manthr_min,			/**< minimum throttle for stabilized */
		(ParamFloat<px4::params::MPC_THR_MAX>) _param_mpc_thr_max,				/**< maximum throttle for stabilized */
		(ParamFloat<px4::params::MPC_THR_HOVER>)
		_param_mpc_thr_hover,			/**< throttle at which vehicle is at hover equilibrium */
		(ParamInt<px4::params::MPC_THR_CURVE>) _param_mpc_thr_curve,				/**< throttle curve behavior */

		(ParamInt<px4::params::MC_AIRMODE>) _param_mc_airmode,

		(ParamInt<px4::params::CBRK_RATE_CTRL>) _param_cbrk_rate_ctrl,

		(ParamFloat<px4::params::MC_LC_RP>) _lossctrl_roll_p,
		(ParamFloat<px4::params::MC_LC_PP>) _lossctrl_pitch_p,
		(ParamFloat<px4::params::MC_LC_NBA>) _lossctrl_nb_angle,
		(ParamInt<px4::params::CONTROLLER_FLAG>) controller_flag

	)

	bool _is_tailsitter{false};

	matrix::Vector3f _acro_rate_max;	/**< max attitude rates in acro mode */
	float _man_tilt_max;			/**< maximum tilt allowed for manual flight [rad] */

	/* for disturbance estimator */
	matrix::Vector3f	_x1;		// rates_hat
	matrix::Vector3f	_x2;		// estimate of angular acceleration
//	matrix::Vector3f	_x1_dot;
//	matrix::Vector3f	_x2_dot;
	matrix::Vector3f	_torque_hat;	// estimate of torque
	matrix::Vector3f	_torque_motor;	// current torque computed by motor thrust model
	matrix::Vector3f	_torque_dist;	// disturbed torque
	matrix::Vector3f	_torque_dist_last;
	matrix::Vector3f	_x1_trq;
	matrix::Vector3f	_x2_trq;

	matrix::Vector3f	_n_err_prev;

	/***************************************/
	matrix::Vector3f	_un;
	matrix::Vector3f	_un_v;
	matrix::Vector3f	_func_g;		// function g, used for get sign(s)
	matrix::Vector3f	_func_g_prev;	// used for stor the last func_g
	/***************************************/

	/***************************************/
	matrix::Vector3f	_un_pri_axis;
	matrix::Vector3f	_un_v_pri_axis;
	matrix::Vector3f	_func_g_pri_axis;		// function g, used for get sign(s)
	matrix::Vector3f	_func_g_prev_pri_axis;	// used for stor the last func_g
	/***************************************/


	int32_t	_pwm_min_value;
	int32_t _pwm_max_value;
	float 	_omega_att = 50.0f;
	float	_zeta_att = 0.7f;
	float	_half_length = 0.101f;
	float	_half_width	 = 0.08f;
	float	_C_M	= 	0.0097f;
	float	_I_XX	= 3e-3f;
	float	_I_YY	= 2.7e-3f;
	float	_I_ZZ	= 6e-3f;
	float	_thrust_factor = 0.4785f;
	int32_t _ndrc_att_enable;
	int32_t _ndi_enable;
	int32_t _lossctrl_enable;

	int32_t _smc_enable;
	int32_t _nde_enable;

	float _i_alpha = 0.9f;
	float _i_c = 0.5f;
	float _i_t = 0.5f;
	float _i_eta = 1.0f;

	int32_t _smc_enable_pri_axis;
	int32_t _nde_enable_pri_axis;

	float _i_alpha_pri_axis = 0.9f;
	float _i_c_pri_axis = 0.5f;
	float _i_t_pri_axis = 0.5f;
	float _i_eta_pri_axis = 1.0f;

	int _num_update = 0;
	matrix::Vector3f _rates_int;
	matrix::Vector3f _rate_int_lim;		/**< integrator state limit for rate loop */
	matrix::Vector3f _rates_prev;			/**< angular rates on previous step */
	matrix::Vector3f _rates_prev_filtered;		/**< angular rates on previous step (low-pass filtered) */
	math::LowPassFilter2pVector3f _lp_filters_d{0.f, 0.f}; ///< low-pass filters for D-term (roll, pitch & yaw)

};


		// (ParamFloat<px4::params::MC_OMEGA_ATT>) _omega_att,
		// (ParamFloat<px4::params::MC_ZETA_ATT>) _zeta_att,
		// (ParamFloat<px4::params::MC_HALF_LENGTH>) _half_length,
		// (ParamFloat<px4::params::MC_HALF_WIDTH>) _half_length,
		// (ParamFloat<px4::params::MC_C_M>) _C_M,
		// (ParamFloat<px4::params::MC_I_XX>) _I_XX,
		// (ParamFloat<px4::params::MC_I_YY>) _I_YY,
		// (ParamFloat<px4::params::MC_I_ZZ>) _I_ZZ,
		// (ParamInt<px4::params::MC_NDRC_ATT_EN>) _ndrc_att_enable,
		// (ParamInt<px4::params::MC_NDI_ENABLE>) _ndi_enable,

		// (ParamFloat<px4::params::I_ALPHA>) _i_alpha,
		// (ParamFloat<px4::params::I_C>) _i_c,
		// (ParamFloat<px4::params::I_T>) _i_t,
		// (ParamFloat<px4::params::I_ETA>) _i_eta,
		// (ParamInt<px4::params::MC_SMC_ENABLE>) _smc_enable,
		// (ParamInt<px4::params::MC_NDE_ENABLE>) _nde_enable,
		// (ParamFloat<px4::params::I_ALPHA_PA>) _i_alpha_pri_axis,
		// (ParamFloat<px4::params::I_C_PA>) _i_c_pri_axis,
		// (ParamFloat<px4::params::I_T_PA>) _i_t_pri_axis,
		// (ParamFloat<px4::params::I_ETA_PA>) _i_eta_pri_axis,
		// (ParamInt<px4::params::MC_SMC_ENABLE_PA>) _smc_enable_pri_axis,
		// (ParamInt<px4::params::MC_NDE_ENABLE_PA>) _nde_enable_pri_axis
