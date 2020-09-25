/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

/**
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Matthias Grob	<maetugr@gmail.com>
 * @author Beat Küng		<beat-kueng@gmx.net>
 *
 */

#include "mc_att_control.hpp"

#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>


#define MIN_TAKEOFF_THRUST    0.2f
#define TPA_RATE_LOWER_LIMIT 0.05f

#define AXIS_INDEX_ROLL 0
#define AXIS_INDEX_PITCH 1
#define AXIS_INDEX_YAW 2
#define AXIS_COUNT 3

/**
 * Used for Model Based Control.
 * Model Based Controller outputs commands of thrust and torques around three-axis,
 * while the commands send by px4 to low-level mixer and motors are normalized values in 0~1(thrust) or -1~1(torque).
 * By converting F&T commands to normalized commands, we can implement Model Based Control methods readily,
 * using existing mixer and saturation limiter.
 */
/* Model of QAV250 */
#define HALF_LENGTH		0.101f
#define HALF_WIDTH		0.080f

#define C_M				0.0097f		// C_M = drag torque / motor thrust.
//#define THRUST_FACTOR	0.0f
#define THRUST_FACTOR	0.4785f

#define I_XX			3e-3f
#define I_YY			2.7e-3f
#define I_ZZ			6.0e-3f
// Parameters of attitude(acc) estimator
#define OMEGA_ATT		50.0f
#define ZETA_ATT		0.8f



using namespace matrix;

MulticopterAttitudeControl::MulticopterAttitudeControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, "mc_att_control"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	/* initialize quaternions in messages to be valid */
	_v_att.q[0] = 1.f;
	_v_att_sp.q_d[0] = 1.f;

	_rates_sp.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();

	parameters_updated();
}

MulticopterAttitudeControl::~MulticopterAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterAttitudeControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity callback registration failed!");
		return false;
	}

	return true;
}

void
MulticopterAttitudeControl::parameters_updated()
{
	// Store some of the parameters in a more convenient way & precompute often-used values
	_attitude_control.setProportionalGain(Vector3f(_param_mc_roll_p.get(), _param_mc_pitch_p.get(), _param_mc_yaw_p.get()));

	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());
	_rate_control.setGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));
	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));
	_rate_control.setDTermCutoff(_loop_update_rate_hz, _param_mc_dterm_cutoff.get(), false);

	_rate_int_lim = Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get());
	_lp_filters_d.set_cutoff_frequency(_loop_update_rate_hz, _param_mc_dterm_cutoff.get());
	_lp_filters_d.reset(_rates_prev);

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));

	// angular rate limits
	using math::radians;
	_attitude_control.setRateLimit(Vector3f(radians(_param_mc_rollrate_max.get()), radians(_param_mc_pitchrate_max.get()),
						radians(_param_mc_yawrate_max.get())));

	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	_man_tilt_max = math::radians(_param_mpc_man_tilt_max.get());

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled_by_val(_param_cbrk_rate_ctrl.get(), CBRK_RATE_CTRL_KEY);
}

void
MulticopterAttitudeControl::parameter_update_poll()
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
		parameters_updated();
	}
}

void
MulticopterAttitudeControl::vehicle_status_poll()
{
	/* check if there is new status information */
	if (_vehicle_status_sub.update(&_vehicle_status)) {
		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (_actuators_id == nullptr) {
			if (_vehicle_status.is_vtol) {
				_actuators_id = ORB_ID(actuator_controls_virtual_mc);
				_attitude_sp_id = ORB_ID(mc_virtual_attitude_setpoint);

				int32_t vt_type = -1;

				if (param_get(param_find("VT_TYPE"), &vt_type) == PX4_OK) {
					_is_tailsitter = (static_cast<vtol_type>(vt_type) == vtol_type::TAILSITTER);
				}

			} else {
				_actuators_id = ORB_ID(actuator_controls_0);
				_attitude_sp_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}
}

void
MulticopterAttitudeControl::vehicle_motor_limits_poll()
{
	/* check if there is a new message */
	multirotor_motor_limits_s motor_limits{};

	if (_motor_limits_sub.update(&motor_limits)) {
		_saturation_status.value = motor_limits.saturation_status;
	}
}

bool
MulticopterAttitudeControl::vehicle_attitude_poll()
{
	/* check if there is a new message */
	const uint8_t prev_quat_reset_counter = _v_att.quat_reset_counter;

	if (_v_att_sub.update(&_v_att)) {
		// Check for a heading reset
		if (prev_quat_reset_counter != _v_att.quat_reset_counter) {
			// we only extract the heading change from the delta quaternion
			_man_yaw_sp += Eulerf(Quatf(_v_att.delta_q_reset)).psi();
		}

		return true;
	}

	return false;
}

float
MulticopterAttitudeControl::throttle_curve(float throttle_stick_input)
{
	float throttle_min = _vehicle_land_detected.landed ? 0.0f : _param_mpc_manthr_min.get();

	// throttle_stick_input is in range [0, 1]
	switch (_param_mpc_thr_curve.get()) {
	case 1: // no rescaling to hover throttle
		return throttle_min + throttle_stick_input * (_param_mpc_thr_max.get() - throttle_min);

	default: // 0 or other: rescale to hover throttle at 0.5 stick
		if (throttle_stick_input < 0.5f) {
			return (_param_mpc_thr_hover.get() - throttle_min) / 0.5f * throttle_stick_input +
			       throttle_min;

		} else {
			return (_param_mpc_thr_max.get() - _param_mpc_thr_hover.get()) / 0.5f * (throttle_stick_input - 1.0f) +
			       _param_mpc_thr_max.get();
		}
	}
}

float
MulticopterAttitudeControl::get_landing_gear_state()
{
	// Only switch the landing gear up if we are not landed and if
	// the user switched from gear down to gear up.
	// If the user had the switch in the gear up position and took off ignore it
	// until he toggles the switch to avoid retracting the gear immediately on takeoff.
	if (_vehicle_land_detected.landed) {
		_gear_state_initialized = false;
	}

	float landing_gear = landing_gear_s::GEAR_DOWN; // default to down

	if (_manual_control_sp.gear_switch == manual_control_setpoint_s::SWITCH_POS_ON && _gear_state_initialized) {
		landing_gear = landing_gear_s::GEAR_UP;

	} else if (_manual_control_sp.gear_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
		// Switching the gear off does put it into a safe defined state
		_gear_state_initialized = true;
	}

	return landing_gear;
}

void
MulticopterAttitudeControl::generate_attitude_setpoint(float dt, bool reset_yaw_sp)
{
	vehicle_attitude_setpoint_s attitude_setpoint{};
	const float yaw = Eulerf(Quatf(_v_att.q)).psi();

	/* reset yaw setpoint to current position if needed */
	if (reset_yaw_sp) {
		_man_yaw_sp = yaw;

	} else if (_manual_control_sp.z > 0.05f || _param_mc_airmode.get() == (int32_t)Mixer::Airmode::roll_pitch_yaw) {

		const float yaw_rate = math::radians(_param_mpc_man_y_max.get());
		attitude_setpoint.yaw_sp_move_rate = _manual_control_sp.r * yaw_rate;
		_man_yaw_sp = wrap_pi(_man_yaw_sp + attitude_setpoint.yaw_sp_move_rate * dt);
	}

	/*
	 * Input mapping for roll & pitch setpoints
	 * ----------------------------------------
	 * We control the following 2 angles:
	 * - tilt angle, given by sqrt(x*x + y*y)
	 * - the direction of the maximum tilt in the XY-plane, which also defines the direction of the motion
	 *
	 * This allows a simple limitation of the tilt angle, the vehicle flies towards the direction that the stick
	 * points to, and changes of the stick input are linear.
	 */
	const float x = _manual_control_sp.x * _man_tilt_max;
	const float y = _manual_control_sp.y * _man_tilt_max;

	// we want to fly towards the direction of (x, y), so we use a perpendicular axis angle vector in the XY-plane
	Vector2f v = Vector2f(y, -x);
	float v_norm = v.norm(); // the norm of v defines the tilt angle

	if (v_norm > _man_tilt_max) { // limit to the configured maximum tilt angle
		v *= _man_tilt_max / v_norm;
	}

	Quatf q_sp_rpy = AxisAnglef(v(0), v(1), 0.f);
	Eulerf euler_sp = q_sp_rpy;
	attitude_setpoint.roll_body = euler_sp(0);
	attitude_setpoint.pitch_body = euler_sp(1);
	// The axis angle can change the yaw as well (noticeable at higher tilt angles).
	// This is the formula by how much the yaw changes:
	//   let a := tilt angle, b := atan(y/x) (direction of maximum tilt)
	//   yaw = atan(-2 * sin(b) * cos(b) * sin^2(a/2) / (1 - 2 * cos^2(b) * sin^2(a/2))).
	attitude_setpoint.yaw_body = _man_yaw_sp + euler_sp(2);

	/* modify roll/pitch only if we're a VTOL */
	if (_vehicle_status.is_vtol) {
		// Construct attitude setpoint rotation matrix. Modify the setpoints for roll
		// and pitch such that they reflect the user's intention even if a large yaw error
		// (yaw_sp - yaw) is present. In the presence of a yaw error constructing a rotation matrix
		// from the pure euler angle setpoints will lead to unexpected attitude behaviour from
		// the user's view as the euler angle sequence uses the  yaw setpoint and not the current
		// heading of the vehicle.
		// However there's also a coupling effect that causes oscillations for fast roll/pitch changes
		// at higher tilt angles, so we want to avoid using this on multicopters.
		// The effect of that can be seen with:
		// - roll/pitch into one direction, keep it fixed (at high angle)
		// - apply a fast yaw rotation
		// - look at the roll and pitch angles: they should stay pretty much the same as when not yawing

		// calculate our current yaw error
		float yaw_error = wrap_pi(attitude_setpoint.yaw_body - yaw);

		// compute the vector obtained by rotating a z unit vector by the rotation
		// given by the roll and pitch commands of the user
		Vector3f zB = {0.0f, 0.0f, 1.0f};
		Dcmf R_sp_roll_pitch = Eulerf(attitude_setpoint.roll_body, attitude_setpoint.pitch_body, 0.0f);
		Vector3f z_roll_pitch_sp = R_sp_roll_pitch * zB;

		// transform the vector into a new frame which is rotated around the z axis
		// by the current yaw error. this vector defines the desired tilt when we look
		// into the direction of the desired heading
		Dcmf R_yaw_correction = Eulerf(0.0f, 0.0f, -yaw_error);
		z_roll_pitch_sp = R_yaw_correction * z_roll_pitch_sp;

		// use the formula z_roll_pitch_sp = R_tilt * [0;0;1]
		// R_tilt is computed from_euler; only true if cos(roll) not equal zero
		// -> valid if roll is not +-pi/2;
		attitude_setpoint.roll_body = -asinf(z_roll_pitch_sp(1));
		attitude_setpoint.pitch_body = atan2f(z_roll_pitch_sp(0), z_roll_pitch_sp(2));
	}

	/* copy quaternion setpoint to attitude setpoint topic */
	Quatf q_sp = Eulerf(attitude_setpoint.roll_body, attitude_setpoint.pitch_body, attitude_setpoint.yaw_body);
	q_sp.copyTo(attitude_setpoint.q_d);
	attitude_setpoint.q_d_valid = true;

	attitude_setpoint.thrust_body[2] = -throttle_curve(_manual_control_sp.z);
	attitude_setpoint.timestamp = hrt_absolute_time();

	if (_attitude_sp_id != nullptr) {
		orb_publish_auto(_attitude_sp_id, &_vehicle_attitude_setpoint_pub, &attitude_setpoint, nullptr, ORB_PRIO_DEFAULT);
	}

	_landing_gear.landing_gear = get_landing_gear_state();
	_landing_gear.timestamp = hrt_absolute_time();
	_landing_gear_pub.publish(_landing_gear);
}

/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp'
 */
void
MulticopterAttitudeControl::control_attitude()
{
	_v_att_sp_sub.update(&_v_att_sp);

	// reinitialize the setpoint while not armed to make sure no value from the last mode or flight is still kept
	if (!_v_control_mode.flag_armed) {
		Quatf().copyTo(_v_att_sp.q_d);
		Vector3f().copyTo(_v_att_sp.thrust_body);
	}

	// physical thrust axis is the negative of body z axis
	// thrust设定值直接来自于位置环控制结果
	_thrust_sp = -_v_att_sp.thrust_body[2];

	_rates_sp = _attitude_control.update(Quatf(_v_att.q), Quatf(_v_att_sp.q_d), _v_att_sp.yaw_sp_move_rate);
}

/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp'
 */
void
MulticopterAttitudeControl::control_attitude_motorloss_smc(float dt, const Vector3f &rates)
{
	_v_att_sp_sub.update(&_v_att_sp);

	// reinitialize the setpoint while not armed to make sure no value from the last mode or flight is still kept
	if (!_v_control_mode.flag_armed) {
		Quatf().copyTo(_v_att_sp.q_d);
		Vector3f().copyTo(_v_att_sp.thrust_body);
	}

	// physical thrust axis is the negative of body z axis
	// thrust设定值直接来自于位置环控制结果
	_thrust_sp = -_v_att_sp.thrust_body[2];

	// Primary-axis attitude control
	// compute corrected thrust_sp
	float angle_n = math::min(abs(_lossctrl_nb_angle.get()), 10.0f) * 0.01745329f;	// deg->rad
	_thrust_sp *= 1.0f / cosf(angle_n);

	// compute n_B
	Vector3f n_B(sinf(angle_n)*0.7071f, sinf(angle_n)*0.7071f, -cosf(angle_n));
	n_B.normalize();

	// compute ndes_B
	Quatf q(_v_att.q);
	Quatf qd(_v_att_sp.q_d);
	q.normalize();
	qd.normalize();
	Dcmf R = Quatf(q);
	Dcmf Rd = Quatf(qd);

	Vector3f nd_I(-Rd(0, 2), -Rd(1, 2), -Rd(2, 2));
	nd_I.normalize();
	Vector3f nd_B = R.T() * nd_I;
	nd_B.normalize();

	Vector3f alpha_TSMC(_i_alpha_pri_axis, _i_alpha_pri_axis, _i_alpha_pri_axis);
	Vector3f c_TSMC(_i_c_pri_axis, _i_c_pri_axis, _i_c_pri_axis);
	Vector3f T_TSMC(_i_t_pri_axis, _i_t_pri_axis, _i_t_pri_axis);
	Vector3f eta_TSMC(_i_eta_pri_axis, _i_eta_pri_axis, _i_eta_pri_axis);
	Vector3f sign_S(0,0,0);
	Vector3f attitude_err(0,0,0);

	attitude_err(0) = n_B(0) - nd_B(0);
	attitude_err(1) = n_B(1) - nd_B(1);

	get_funcg_integPart(c_TSMC, alpha_TSMC, attitude_err, dt, _func_g_pri_axis);
	_func_g_pri_axis = attitude_err + _func_g_pri_axis;
	for(int i = 0; i <= 1; i++)
	{
		sign_S(i) = math::signNoZero(_func_g_pri_axis(i) - _func_g_prev_pri_axis(i));
	}
	_func_g_prev_pri_axis = _func_g_pri_axis;

	get_un_TSMC(T_TSMC, eta_TSMC, sign_S, dt, _un_pri_axis, _un_v_pri_axis);
	Vector3f eq_cont_part(0,0,0);
	eq_cont_part = c_TSMC.emult(nolinear_part_TSMC(attitude_err, alpha_TSMC));


	// control law part
	if(_smc_enable_pri_axis < 0.5){
		Vector2f miu_out;
		miu_out(0) = _lossctrl_roll_p.get() * (n_B(0) - nd_B(0));
		miu_out(1) = _lossctrl_pitch_p.get() * (n_B(1) - nd_B(1));

		// neglect nd_I_dot
		//_rates_sp(0) = 1.0f / nd_B(2) * (miu_out(1) + nd_B(0) * rates(2));
		//_rates_sp(1) = -1.0f / nd_B(2) * (miu_out(0) - nd_B(1) * rates(2));
		_rates_sp(0) = 1.0f / nd_B(2) * (miu_out(1) + nd_B(0) * rates(2));
		_rates_sp(1) = -1.0f / nd_B(2) * (miu_out(0) - nd_B(1) * rates(2));
		_rates_sp(2) = 0.0f; // just to initialize
	}
	else{
		_rates_sp(0) = 1.0f / nd_B(2) * (eq_cont_part(1) + nd_B(0) * rates(2) - _un_pri_axis(2));
		_rates_sp(1) = -1.0f / nd_B(2) * (eq_cont_part(0) - nd_B(1) * rates(2) - _un_pri_axis(1));
		_rates_sp(2) = 0.0f; // just to initialize
	}

}

/**
 * Get integral par of func_g
 */
void
MulticopterAttitudeControl::get_funcg_integPart(Vector3f c_TSMC, Vector3f alpha_TSMC, Vector3f rates_err, const float dt, Vector3f &x1)
{
    Vector3f x1_dot_1, x1_dot_2, x1_dot_3, x1_dot_4;
    Vector3f x1_tmp;

    x1_tmp = x1;	// x1: estimate of x
    funcg_model(c_TSMC, alpha_TSMC, rates_err, x1_tmp, x1_dot_1);

    x1_tmp = x1 + x1_dot_1 * dt/2.0f;
    funcg_model(c_TSMC, alpha_TSMC, rates_err, x1_tmp, x1_dot_2);

    x1_tmp = x1 + x1_dot_2 * dt/2.0f;
    funcg_model(c_TSMC, alpha_TSMC, rates_err, x1_tmp, x1_dot_3);

    x1_tmp = x1 + x1_dot_3 * dt;
    funcg_model(c_TSMC, alpha_TSMC, rates_err, x1_tmp, x1_dot_4);

    x1 += (x1_dot_1 + x1_dot_2 * 2.0f + x1_dot_3 * 2.0f + x1_dot_4) * dt/6.0f;
}
void MulticopterAttitudeControl::funcg_model(Vector3f &c_TSMC, Vector3f &alpha_TSMC, Vector3f &rates_err, Vector3f &x1, Vector3f &x1_dot)
{
    x1_dot = c_TSMC.emult(nolinear_part_TSMC(rates_err, alpha_TSMC));
}


/**
 * Get un of TSMC
 */
void MulticopterAttitudeControl::get_un_TSMC(Vector3f T_TSMC, Vector3f eta_TSMC, Vector3f sign_S, const float dt, Vector3f &x1, Vector3f &x2)
{
    Vector3f x1_dot_1, x2_dot_1, x1_dot_2, x2_dot_2, x1_dot_3, x2_dot_3, x1_dot_4, x2_dot_4;
    Vector3f x1_tmp, x2_tmp;

    x1_tmp = x1;	// x1: estimate of x
    x2_tmp = x2;	// x2: estimate of x' acceleration
    un_model(T_TSMC, eta_TSMC, sign_S, x1_tmp, x2_tmp, x1_dot_1, x2_dot_1);

    x1_tmp = x1 + x1_dot_1 * dt/2.0f;
    x2_tmp = x2 + x2_dot_1 * dt/2.0f;
    un_model(T_TSMC, eta_TSMC, sign_S, x1_tmp, x2_tmp, x1_dot_2, x2_dot_2);

    x1_tmp = x1 + x1_dot_2 * dt/2.0f;
    x2_tmp = x2 + x2_dot_2 * dt/2.0f;
    un_model(T_TSMC, eta_TSMC, sign_S, x1_tmp, x2_tmp, x1_dot_3, x2_dot_3);

    x1_tmp = x1 + x1_dot_3 * dt;
    x2_tmp = x2 + x2_dot_3 * dt;
    un_model(T_TSMC, eta_TSMC, sign_S, x1_tmp, x2_tmp, x1_dot_4, x2_dot_4);

    x1 += (x1_dot_1 + x1_dot_2 * 2.0f + x1_dot_3 * 2.0f + x1_dot_4) * dt/6.0f;
    x2 += (x2_dot_1 + x2_dot_2 * 2.0f + x2_dot_3 * 2.0f + x2_dot_4) * dt/6.0f;

}

void MulticopterAttitudeControl::un_model(Vector3f &T_TSMC, Vector3f &eta_TSMC, Vector3f &sign_S, Vector3f &x1, Vector3f &x2, Vector3f &x1_dot, Vector3f &x2_dot)
{
    Vector3f temp_x2(0,0,0);
    x2_dot.zero();
    for(int i = 0; i <= 2; i++)
    {
        temp_x2(i) = - eta_TSMC(i)*sign_S(i);
    }
    x1_dot = temp_x2 - T_TSMC.emult(x1);
}

Vector3f
MulticopterAttitudeControl::nolinear_part_TSMC(Vector3f rates_err, Vector3f alpha_TSMC)
{
    Vector3f temp_nolinear(0,0,0);
    for(int i = 0; i <=2; i++)
    {
        temp_nolinear(i) = pow(fabsf(rates_err(i)), alpha_TSMC(i))*math::signNoZero(rates_err(i));
    }
    return temp_nolinear;
}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
void
MulticopterAttitudeControl::control_attitude_rates(float dt, const Vector3f &rates)
{
	// reset integral if disarmed
	if (!_v_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		_rate_control.resetIntegral();
		_rates_int.zero();
	}

	const bool landed = _vehicle_land_detected.maybe_landed || _vehicle_land_detected.landed;
	_rate_control.setSaturationStatus(_saturation_status);
	_att_control = _rate_control.update(rates, _rates_sp, dt, landed);
}

//smc
void
MulticopterAttitudeControl::control_attitude_rates_motorloss_smc(float dt, const Vector3f &rates)
{
	// reset integral if disarmed
	if (!_v_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		_rate_control.resetIntegral();
		_rates_int.zero();
		_un.zero();
		_un_v.zero();
		_func_g.zero();
		_func_g_prev.zero();
	}

	Vector3f rates_p_scaled = Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get());
	Vector3f rates_i_scaled = Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get());
	Vector3f rates_d_scaled = Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get());

	Vector3f rates_err = _rates_sp - rates;

	/* apply low-pass filtering to the rates for D-term */
	Vector3f rates_filtered(_lp_filters_d.apply(rates));

	/* estimate attitude rates and accelerations */
	estimator_update(rates, dt, _x1, _x2);

	/* compute torque_hat */
	Vector3f inertia(_I_XX, _I_YY, _I_ZZ);
	_torque_hat = inertia.emult(_x2) + _x1.cross(inertia.emult(_x1));	// tao = I*w_dot + w x I*w

	_actuator_outputs_sub.update(&_actuator_outputs);
	/* compute torque_motor through motor thrust model */
	_torque_motor = compute_actuate_torque();
	estimator_update(_torque_motor, dt, _x1_trq, _x2_trq);

	_torque_dist = _torque_hat - _x1_trq;
	_torque_dist_last = _torque_dist;
	/* compute gyro moment */
	Vector3f torque_affix = rates.cross(inertia.emult(rates));

	/* Compensate nonlinear gyro moment and disturbed moment */
	Vector3f compen_torque(0,0,0);

	if(_nde_enable > 0.5){
		compen_torque = torque_affix * 1.0f - _torque_dist * 1.0f;
		}
		else{
		compen_torque = torque_affix * 1.0f - _torque_dist * 0.0f;
	}

	/* publish estimated disturbance */
	_ext_torque.esti_dist[0] = _torque_dist(0);
	_ext_torque.esti_dist[1] = _torque_dist(1);
	_ext_torque.esti_dist[2] = _torque_dist(2);

	_ext_torque.ft_hat[0] = _torque_hat(0);
	_ext_torque.ft_hat[1] = _torque_hat(1);
	_ext_torque.ft_hat[2] = _torque_hat(2);

	_ext_torque.ft_motor[0] = _torque_motor(0);
	_ext_torque.ft_motor[1] = _torque_motor(1);
	_ext_torque.ft_motor[2] = _torque_motor(2);
	_ext_torque.ft_motor_filter[0] = _x1_trq(0);
	_ext_torque.ft_motor_filter[1] = _x1_trq(1);
	_ext_torque.ft_motor_filter[2] = _x1_trq(2);

	_ext_torque.timestamp = hrt_absolute_time();

	if(_ext_torque_pub != nullptr){
		orb_publish(ORB_ID(estimate_torque), _ext_torque_pub, &_ext_torque);
	} else {
		_ext_torque_pub = orb_advertise(ORB_ID(estimate_torque), &_ext_torque);
	}

	/* publish angular vel/acc */
	_v_esti_att.rollspeed = _x1(0);
	_v_esti_att.pitchspeed = _x1(1);
	_v_esti_att.yawspeed = _x1(2);
	_v_esti_att.rollacc = _x2(0);
	_v_esti_att.pitchacc = _x2(1);
	_v_esti_att.yawacc = _x2(2);
	_v_esti_att.timestamp = hrt_absolute_time();

	if(_v_esti_att_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_estimate_attitude), _v_esti_att_pub, &_v_esti_att);
	} else {
		_v_esti_att_pub = orb_advertise(ORB_ID(vehicle_estimate_attitude), &_v_esti_att);
	}

	/*********************************************************/
	Vector3f alpha_TSMC(_i_alpha, _i_alpha, _i_alpha);
	Vector3f c_TSMC(_i_c,_i_c,_i_c);
	Vector3f T_TSMC(_i_t,_i_t,_i_t);
	Vector3f eta_TSMC(_i_eta,_i_eta,_i_eta);
	//   Vector3f S_TSMC(0,0,0);
	Vector3f sign_S(0,0,0);

	c_TSMC = c_TSMC.emult(rates_p_scaled);
	//   T_TSMC = T_TSMC.emult(rates_i_scaled);
	//   eta_TSMC = eta_TSMC.emult(rates_i_scaled);

	//    S_TSMC = -_x2 + c_TSMC.emult(nolinear_part_TSMC(rates_err, alpha_TSMC));
	get_funcg_integPart(c_TSMC, alpha_TSMC, rates_err, dt, _func_g);
	_func_g = rates_err + _func_g;
	for(int i = 0; i <= 2; i++)
	{
		sign_S(i) = math::signNoZero(_func_g(i) - _func_g_prev(i));
	}
	_func_g_prev = _func_g;

	get_un_TSMC(T_TSMC, eta_TSMC, sign_S, dt, _un, _un_v);
	Vector3f eq_cont_part(0,0,0);
	eq_cont_part = - c_TSMC.emult(nolinear_part_TSMC(rates_err, alpha_TSMC));
	Vector3f torque_att_ctrl(0,0,0);
	if(_smc_enable > 0.5){
		Vector3f att_ctrl = - c_TSMC.emult(nolinear_part_TSMC(rates_err, alpha_TSMC)) + _un*1.0f;
		torque_att_ctrl = - inertia.emult(att_ctrl);
		torque_att_ctrl = torque_att_ctrl + compen_torque * 1.f;
		}
	else{

		Vector3f att_ctrl = rates_p_scaled.emult(rates_err) + _rates_int - rates_d_scaled.emult(rates_filtered - _rates_prev_filtered) / dt * 1.0f;
		torque_att_ctrl = inertia.emult(att_ctrl);
	}

	if(_num_update%500 == 0){
		// PX4_INFO("Sliding Mode Surfaces = %8.6f, %8.6f, %8.6f", (double)S_TSMC(0), (double)S_TSMC(1), (double)S_TSMC(2));
		// PX4_INFO("un = %8.6f, %8.6f, %8.6f", (double)_un(0), (double)_un(1), (double)_un(2));
		// PX4_INFO("att_ctrl = %8.6f, %8.6f, %8.6f", (double)att_ctrl(0), (double)att_ctrl(1), (double)att_ctrl(2));
		// PX4_INFO("rates_p_scaled = %8.6f, %8.6f, %8.6f", (double)rates_p_scaled(0), (double)rates_p_scaled(1), (double)rates_p_scaled(2));
		// PX4_INFO("rates_i_scaled = %8.6f, %8.6f, %8.6f", (double)rates_i_scaled(0), (double)rates_i_scaled(1), (double)rates_i_scaled(2));
		// PX4_INFO("_smc_enable = %8.6f", (double)_smc_enable);

	}


	/* publish TSMC intermediate variables */
	_v_TSMC_att.eq_part_roll= eq_cont_part(0);
	_v_TSMC_att.eq_part_pitch = eq_cont_part(1);
	_v_TSMC_att.eq_part_yaw = eq_cont_part(2);

	_v_TSMC_att.disc_part_roll = _un(0);
	_v_TSMC_att.disc_part_pitch = _un(1);
	_v_TSMC_att.disc_part_yaw = _un(2);

	_v_TSMC_att.sign_s_roll = sign_S(0);
	_v_TSMC_att.sign_s_pitch = sign_S(1);
	_v_TSMC_att.sign_s_yaw = sign_S(2);

	_v_TSMC_att.rates_err_roll = rates_err(0);
	_v_TSMC_att.rates_err_pitch = rates_err(1);
	_v_TSMC_att.rates_err_yaw = rates_err(2);

	_v_TSMC_att.timestamp = hrt_absolute_time();

	if(_v_TSMC_att_pub != nullptr){
		orb_publish(ORB_ID(vehicle_tsmc_attitude), _v_TSMC_att_pub, &_v_TSMC_att);
	} else {
		_v_TSMC_att_pub = orb_advertise(ORB_ID(vehicle_tsmc_attitude), &_v_TSMC_att);
	}

    /**********************************************************/

	/* Convert torque command to normalized input */
//    torque_att_ctrl = torque_att_ctrl + compen_torque * 1.f;


//	_att_control = torque_to_attctrl(torque_att_ctrl);
	if(_lossctrl_enable){
		_att_control = torque_to_attctrl_motorloss(torque_att_ctrl);
	}else {
		_att_control = torque_to_attctrl(torque_att_ctrl);
	}


	_rates_prev = rates;
	_rates_prev_filtered = rates_filtered;

	/* update integral only if motors are providing enough thrust to be effective */
	if (_thrust_sp > MIN_TAKEOFF_THRUST) {
		for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
			// Check for positive control saturation
			bool positive_saturation =
				((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_pos) ||
				((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_pos) ||
				((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_pos);

			// Check for negative control saturation
			bool negative_saturation =
				((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_neg) ||
				((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_neg) ||
				((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_neg);

			// prevent further positive control saturation
			if (positive_saturation) {
				rates_err(i) = math::min(rates_err(i), 0.0f);

			}

			// prevent further negative control saturation
			if (negative_saturation) {
				rates_err(i) = math::max(rates_err(i), 0.0f);

			}

			// Perform the integration using a first order method and do not propagate the result if out of range or invalid
			float rate_i = _rates_int(i) + rates_i_scaled(i) * rates_err(i) * dt;

			if (PX4_ISFINITE(rate_i) && rate_i > -_rate_int_lim(i) && rate_i < _rate_int_lim(i)) {
				_rates_int(i) = rate_i;
			}
		}
	}

	/* explicitly limit the integrator state */
	for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
		_rates_int(i) = math::constrain(_rates_int(i), -_rate_int_lim(i), _rate_int_lim(i));

	}
}

/**
 * Estimator for attitude rates and accelerations using Runge-Kutta methods
 */
void MulticopterAttitudeControl::estimator_update(Vector3f x, const float dt, Vector3f &x1, Vector3f &x2)
{
	Vector3f x1_dot_1, x2_dot_1, x1_dot_2, x2_dot_2, x1_dot_3, x2_dot_3, x1_dot_4, x2_dot_4;
	Vector3f x1_tmp, x2_tmp;

	x1_tmp = x1;	// x1: estimate of x
	x2_tmp = x2;	// x2: estimate of x' acceleration
	estimator_model(x, x1_tmp, x2_tmp, x1_dot_1, x2_dot_1);

	x1_tmp = x1 + x1_dot_1 * dt/2.0f;
	x2_tmp = x2 + x2_dot_1 * dt/2.0f;
	estimator_model(x, x1_tmp, x2_tmp, x1_dot_2, x2_dot_2);

	x1_tmp = x1 + x1_dot_2 * dt/2.0f;
	x2_tmp = x2 + x2_dot_2 * dt/2.0f;
	estimator_model(x, x1_tmp, x2_tmp, x1_dot_3, x2_dot_3);

	x1_tmp = x1 + x1_dot_3 * dt;
	x2_tmp = x2 + x2_dot_3 * dt;
	estimator_model(x, x1_tmp, x2_tmp, x1_dot_4, x2_dot_4);

	x1 += (x1_dot_1 + x1_dot_2 * 2.0f + x1_dot_3 * 2.0f + x1_dot_4) * dt/6.0f;
	x2 += (x2_dot_1 + x2_dot_2 * 2.0f + x2_dot_3 * 2.0f + x2_dot_4) * dt/6.0f;

}

/**
 * Estimator model
 * model: x_hat = wa^2 / (s^2 + 2*zeta*wa*s + wa^2) * x
 * x1 = x_hat; x2 = x_dot_hat
 * ==> x2_dot + 2*zeta*wa*x2 + wa^2*x1 = wa^2 * x
 * ==> x1_dot = x2;
 * 	   x2_dot = wa^2*(x - x1) - 2*zeta*wa*x2
 */
void MulticopterAttitudeControl::estimator_model(Vector3f &rates, Vector3f &x1, Vector3f &x2, Vector3f &x1_dot, Vector3f &x2_dot)
{
	x1_dot = x2;
//	x2_dot =  (rates - x1) * OMEGA_ATT * OMEGA_ATT - x2 * 2.0f * ZETA_ATT * OMEGA_ATT;
	x2_dot =  (rates - x1) * _omega_att * _omega_att - x2 * 2.0f * _zeta_att * _omega_att;
}

/**
 * Transform computed torques to equivalent normalized inputs
 */
Vector3f
MulticopterAttitudeControl::torque_to_attctrl(Vector3f &computed_torque)
{
	/* motor maximum thrust model */
	float thrust_max = motor_max_thrust(_battery_status.voltage_filtered_v);

	/* mixer matrix */
	float array_mixer[4][3] = {
			{-0.707107f,  0.707107f,  1.0f},
			{0.707107f,  -0.707107f,  1.0f},
			{0.707107f,  0.707107f,  -1.0f},
			{-0.707107f,  -0.707107f,  -1.0f}
	};
	Matrix<float, 4, 3> mixer_matrix(array_mixer);

//	if(_lossctrl_enable){
////		int motor_fail_num = math::constrain(_lossctrl_enable, 1, 4);
//
//		for(int i = 0; i < 3; i++){
//			mixer_matrix(motor_fail_num - 1, i) = 0.0f;
//		}
//	}

	/* matrix from thrust of four propellers to torque about 3-axes */
//	float array_torque[3][4] = {
//			{-HALF_LENGTH, HALF_LENGTH, HALF_LENGTH, -HALF_LENGTH},
//			{HALF_WIDTH, -HALF_WIDTH, HALF_WIDTH, -HALF_WIDTH},
//			{C_M, C_M, -C_M, -C_M}
//	};
	float array_torque[3][4] = {
			{-_half_length, _half_length, _half_length, -_half_length},
			{_half_width, -_half_width, _half_width, -_half_width},
			{_C_M, _C_M, -_C_M, -_C_M}
	};
	Matrix<float, 3, 4> gentrq_matrix(array_torque);

	/* input = (Gamma * Mixer * Tmax)^(-1) * computed_torque */
	SquareMatrix<float, 3> gentrq_mixer = Matrix<float, 3, 3>(gentrq_matrix * mixer_matrix);
	Matrix<float, 3, 3> trq_to_attctrl = gentrq_mixer.I() / thrust_max;

	return trq_to_attctrl * computed_torque;
}
/**
 * Motor thrust model
 */
//　电压与推力的模型？
inline float MulticopterAttitudeControl::motor_max_thrust(float battery_voltage)
{
	if(battery_voltage < 11.1f){
		battery_voltage = 11.1f;
	} else if(battery_voltage > 12.6f){
		battery_voltage = 12.6;
	}
	return 1.0f*5.488f * sinf(battery_voltage * 0.4502f + 2.2241f);
//	return 4.77f;
}

//　力矩与归一化控制量的模型？
Vector3f
MulticopterAttitudeControl::torque_to_attctrl_motorloss(Vector3f &computed_torque)
{
	/* motor maximum thrust model */
	float thrust_max = motor_max_thrust(_battery_status.voltage_filtered_v);

	/* mixer matrix */
	/*float array_mixer[3][2] = {
			{-1.0f,	 0.0f},
			{0.0f,	-1.0f},
			{1.0f,	1.0f}
	};*/
	float array_mixer[3][2] = {
				{-0.867091f,	0.006471f},
				{0.006471f,	-0.867091f},
				{0.864934f,	0.864934f}
	};
	Matrix<float, 3, 2> mixer_matrix(array_mixer);

	/* matrix from thrust of four propellers to torque about x-y axis */
	float array_torque[2][3] = {
			{-_half_length, _half_length, _half_length},
			{_half_width, -_half_width, _half_width}
	};
	Matrix<float, 2, 3> gentrq_matrix(array_torque);

	/* input = (Gamma * Mixer * Tmax)^(-1) * computed_torque */
	SquareMatrix<float, 2> gentrq_mixer = Matrix<float, 2, 2>(gentrq_matrix * mixer_matrix);
	Matrix<float, 2, 2> trq_to_attctrl = gentrq_mixer.I() / thrust_max;

	Vector2f torque_xy = Vector2f(computed_torque(0), computed_torque(1));
	Vector2f attctrl_xy = trq_to_attctrl * torque_xy;
	Vector3f attctrl = Vector3f(attctrl_xy(0), attctrl_xy(1), 0.0f);

	return attctrl;
}

/**
 * Torque around 3-axis generated by four motor thrusts
 */
//　由油门反解当前力矩
Vector3f
MulticopterAttitudeControl::compute_actuate_torque()
{
	float thrust_max = motor_max_thrust(_battery_status.voltage_filtered_v);
//	float array_torque[3][4] = {
//			{-HALF_LENGTH, HALF_LENGTH, HALF_LENGTH, -HALF_LENGTH},
//			{HALF_WIDTH, -HALF_WIDTH, HALF_WIDTH, -HALF_WIDTH},
//			{C_M, C_M, -C_M, -C_M}
//	};
	float array_torque[3][4] = {
			{-_half_length, _half_length, _half_length, -_half_length},
			{_half_width, -_half_width, _half_width, -_half_width},
			{_C_M, _C_M, -_C_M, -_C_M}
	};
	Matrix<float, 3, 4> gentrq_matrix(array_torque);
	// motor thrust model:
	// thrust = (1 - _thrust_factor) * PWM + _thrust_factor * PWM^2
	float pwm[4];
	float pwm_amp = _pwm_max_value - _pwm_min_value;
//	pwm[0] = (1.0f + _actuator_outputs.output[0]) / 2.0f;
//	pwm[1] = (1.0f + _actuator_outputs.output[1]) / 2.0f;
//	pwm[2] = (1.0f + _actuator_outputs.output[2]) / 2.0f;
//	pwm[3] = (1.0f + _actuator_outputs.output[3]) / 2.0f;
	pwm[0] = math::constrain((_actuator_outputs.output[0] - _pwm_min_value) / pwm_amp, 0.f, 1.f);
	pwm[1] = math::constrain((_actuator_outputs.output[1] - _pwm_min_value) / pwm_amp, 0.f, 1.f);
	pwm[2] = math::constrain((_actuator_outputs.output[2] - _pwm_min_value) / pwm_amp, 0.f, 1.f);
	pwm[3] = math::constrain((_actuator_outputs.output[3] - _pwm_min_value) / pwm_amp, 0.f, 1.f);
	Vector<float, 4> throttle, thrust;
//	throttle(0) = (1.0f - THRUST_FACTOR) * pwm[0] + THRUST_FACTOR * pwm[0] * pwm[0];
//	throttle(1) = (1.0f - THRUST_FACTOR) * pwm[1] + THRUST_FACTOR * pwm[1] * pwm[1];
//	throttle(2) = (1.0f - THRUST_FACTOR) * pwm[2] + THRUST_FACTOR * pwm[2] * pwm[2];
//	throttle(3) = (1.0f - THRUST_FACTOR) * pwm[3] + THRUST_FACTOR * pwm[3] * pwm[3];
	throttle(0) = (1.0f - _thrust_factor) * pwm[0] + _thrust_factor * pwm[0] * pwm[0];
	throttle(1) = (1.0f - _thrust_factor) * pwm[1] + _thrust_factor * pwm[1] * pwm[1];
	throttle(2) = (1.0f - _thrust_factor) * pwm[2] + _thrust_factor * pwm[2] * pwm[2];
	throttle(3) = (1.0f - _thrust_factor) * pwm[3] + _thrust_factor * pwm[3] * pwm[3];
	thrust = throttle * thrust_max;

	if(_lossctrl_enable){
		int motor_fail_num = math::constrain(_lossctrl_enable, 1, 4);
		throttle(motor_fail_num -1) = 0.0f;
		thrust(motor_fail_num -1) = 0.0f;
	}

//	if(_num_update%200 == 0 and _lossctrl_enable){
////		PX4_INFO("thrust_max = %8.6f", (double)thrust_max);
////		PX4_INFO("_pwm_min_value = %d", _pwm_min_value);
////		PX4_INFO("_pwm_max_value = %d", _pwm_max_value);
//		PX4_INFO("pwm = %8.6f, %8.6f, %8.6f, %8.6f", (double)pwm[0], (double)pwm[1], (double)pwm[2], (double)pwm[3]);
//		PX4_INFO("throttle = %8.6f, %8.6f, %8.6f, %8.6f", (double)throttle(0), (double)throttle(1), (double)throttle(2), (double)throttle(3));
////		PX4_INFO("thrust = %8.6f, %8.6f, %8.6f, %8.6f", (double)thrust(0), (double)thrust(1), (double)thrust(2), (double)thrust(3));
//	}

	return gentrq_matrix * thrust;
}


void
MulticopterAttitudeControl::publish_rates_setpoint()
{
	_v_rates_sp.roll = _rates_sp(0);
	_v_rates_sp.pitch = _rates_sp(1);
	_v_rates_sp.yaw = _rates_sp(2);
	_v_rates_sp.thrust_body[0] = 0.0f;
	_v_rates_sp.thrust_body[1] = 0.0f;
	_v_rates_sp.thrust_body[2] = -_thrust_sp;
	_v_rates_sp.timestamp = hrt_absolute_time();

	_v_rates_sp_pub.publish(_v_rates_sp);
}

void
MulticopterAttitudeControl::publish_rate_controller_status()
{
	rate_ctrl_status_s rate_ctrl_status = {};
	rate_ctrl_status.timestamp = hrt_absolute_time();
	_rate_control.getRateControlStatus(rate_ctrl_status);
	_controller_status_pub.publish(rate_ctrl_status);
}

void
MulticopterAttitudeControl::publish_actuator_controls()
{
	_actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
	_actuators.control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
	_actuators.control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
	_actuators.control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
	_actuators.control[7] = (float)_landing_gear.landing_gear;
	// note: _actuators.timestamp_sample is set in MulticopterAttitudeControl::Run()
	_actuators.timestamp = hrt_absolute_time();

	/* scale effort by battery status */
	if (_param_mc_bat_scale_en.get() && _battery_status.scale > 0.0f) {
		for (int i = 0; i < 4; i++) {
			_actuators.control[i] *= _battery_status.scale;
		}
	}

	if (!_actuators_0_circuit_breaker_enabled) {
		orb_publish_auto(_actuators_id, &_actuators_0_pub, &_actuators, nullptr, ORB_PRIO_DEFAULT);
	}
}

// 主函数
void
MulticopterAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {
		const hrt_abstime now = hrt_absolute_time();

		// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);
		_last_run = now;

		const Vector3f rates{angular_velocity.xyz};

		_actuators.timestamp_sample = angular_velocity.timestamp_sample;

		/* run the rate controller immediately after a gyro update */
		//　首先运行角速度环控制器
		if (_v_control_mode.flag_control_rates_enabled)
		{
			if (controller_flag.get() == 0)
			{
				//　运行角度环控制器
				control_attitude_rates(dt, rates);
			}
			else if (controller_flag.get()  == 1)
			{
				control_attitude_rates_motorloss_smc(dt, rates);
			}

			publish_actuator_controls();
			publish_rate_controller_status();
		}

		/* check for updates in other topics */
		_v_control_mode_sub.update(&_v_control_mode);
		_battery_status_sub.update(&_battery_status);
		_vehicle_land_detected_sub.update(&_vehicle_land_detected);
		_landing_gear_sub.update(&_landing_gear);
		vehicle_status_poll();
		vehicle_motor_limits_poll();
		const bool manual_control_updated = _manual_control_sp_sub.update(&_manual_control_sp);
		const bool attitude_updated = vehicle_attitude_poll();

		_attitude_dt += dt;

		/* Check if we are in rattitude mode and the pilot is above the threshold on pitch
			* or roll (yaw can rotate 360 in normal att control). If both are true don't
			* even bother running the attitude controllers */
		if (_v_control_mode.flag_control_rattitude_enabled) {
			_v_control_mode.flag_control_attitude_enabled =
				fabsf(_manual_control_sp.y) <= _param_mc_ratt_th.get() &&
				fabsf(_manual_control_sp.x) <= _param_mc_ratt_th.get();
		}

		bool attitude_setpoint_generated = false;

		const bool is_hovering = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
					 && !_vehicle_status.in_transition_mode;

		// vehicle is a tailsitter in transition mode
		const bool is_tailsitter_transition = _vehicle_status.in_transition_mode && _is_tailsitter;

		bool run_att_ctrl = _v_control_mode.flag_control_attitude_enabled && (is_hovering || is_tailsitter_transition);


		if (run_att_ctrl) {
			if (attitude_updated)
			{
				// Generate the attitude setpoint from stick inputs if we are in Manual/Stabilized mode
				if (_v_control_mode.flag_control_manual_enabled &&
				    !_v_control_mode.flag_control_altitude_enabled &&
				    !_v_control_mode.flag_control_velocity_enabled &&
				    !_v_control_mode.flag_control_position_enabled)
				    {
					generate_attitude_setpoint(_attitude_dt, _reset_yaw_sp);
					attitude_setpoint_generated = true;}

				if (controller_flag.get()  == 0)
				{
					//　运行角度环控制器
					control_attitude();
				}
				else if (controller_flag.get()  == 1)
				{
					control_attitude_motorloss_smc(_attitude_dt, rates);
				}



				if (_v_control_mode.flag_control_yawrate_override_enabled) {
					/* Yaw rate override enabled, overwrite the yaw setpoint */
					_v_rates_sp_sub.update(&_v_rates_sp);
					const auto yawrate_reference = _v_rates_sp.yaw;
					_rates_sp(2) = yawrate_reference;
				}

				publish_rates_setpoint();
			}

		} else {
			/* attitude controller disabled, poll rates setpoint topic */
			if (_v_control_mode.flag_control_manual_enabled && is_hovering) {
				if (manual_control_updated) {
					/* manual rates control - ACRO mode */
					Vector3f man_rate_sp(
						math::superexpo(_manual_control_sp.y, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
						math::superexpo(-_manual_control_sp.x, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
						math::superexpo(_manual_control_sp.r, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get()));
					_rates_sp = man_rate_sp.emult(_acro_rate_max);
					_thrust_sp = _manual_control_sp.z;
					publish_rates_setpoint();
				}

			} else {
				/* attitude controller disabled, poll rates setpoint topic */
				if (_v_rates_sp_sub.update(&_v_rates_sp)) {
					_rates_sp(0) = _v_rates_sp.roll;
					_rates_sp(1) = _v_rates_sp.pitch;
					_rates_sp(2) = _v_rates_sp.yaw;
					_thrust_sp = -_v_rates_sp.thrust_body[2];
				}
			}
		}

		if (_v_control_mode.flag_control_termination_enabled) {
			if (!_vehicle_status.is_vtol) {
				_rates_sp.zero();
				_rate_control.resetIntegral();
				_rates_int.zero();
				_thrust_sp = 0.0f;
				_att_control.zero();
				publish_actuator_controls();
			}
		}

		if (attitude_updated) {
			// reset yaw setpoint during transitions, tailsitter.cpp generates
			// attitude setpoint for the transition
			_reset_yaw_sp = (!attitude_setpoint_generated && !_v_control_mode.flag_control_rattitude_enabled) ||
					_vehicle_land_detected.landed ||
					(_vehicle_status.is_vtol && _vehicle_status.in_transition_mode);

			_attitude_dt = 0.f;
		}

		/* calculate loop update rate while disarmed or at least a few times (updating the filter is expensive) */
		if (!_v_control_mode.flag_armed || (now - _task_start) < 3300000) {
			_dt_accumulator += dt;
			++_loop_counter;

			if (_dt_accumulator > 1.f) {
				const float loop_update_rate = (float)_loop_counter / _dt_accumulator;
				_loop_update_rate_hz = _loop_update_rate_hz * 0.5f + loop_update_rate * 0.5f;
				_dt_accumulator = 0;
				_loop_counter = 0;
				_rate_control.setDTermCutoff(_loop_update_rate_hz, _param_mc_dterm_cutoff.get(), true);
			}
		}

		parameter_update_poll();
	}

	perf_end(_loop_perf);
}

int MulticopterAttitudeControl::task_spawn(int argc, char *argv[])
{
	MulticopterAttitudeControl *instance = new MulticopterAttitudeControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterAttitudeControl::print_status()
{
	PX4_INFO("Running");

	perf_print_counter(_loop_perf);

	print_message(_actuators);

	return 0;
}

int MulticopterAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter attitude and rate controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) or rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has two loops: a P loop for angular error and a PID loop for angular rate error.

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

### Implementation
To reduce control latency, the module directly polls on the gyro topic published by the IMU driver.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int mc_att_control_main(int argc, char *argv[])
{
	return MulticopterAttitudeControl::main(argc, argv);
}
