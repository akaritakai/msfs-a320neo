#include <MSFS/MSFS.h>
#include <MSFS/MSFS_Render.h>
#include <MSFS/Legacy/gauges.h>
#include <SimConnect.h>
#include <cmath>
#include <complex>

/*
 * ============== *
 * PID CONTROLLER *
 * ============== *
 */
constexpr double sign(double value)
{
	return (value > 0) ? 1.0 : -1.0;
}
constexpr double clamp(const double value, const double min, const double max)
{
	return (value < min) ? min : (value > max) ? max : value;
}
constexpr double radians(const double value)
{
	return value * (M_PI / 180);
}
constexpr double degrees(const double value)
{
	return value * (180 / M_PI);
}
constexpr bool is_between(const double value, const double left, const double right, const bool strictly)
{
	const auto min = min(left, right);
	const auto max = max(left, right);
	return strictly ? value > min && value < max : value >= min && value <= max;
}
constexpr double linear_decay_coefficient(const double position, const double start, const double end)
{
	// This function provides a coefficient such that the effectiveness is maximal (1.0) before the start,
	// reduces linearly up to the end, and after which is no longer effective (0.0).
	//
	// Effect in the positive direction:
	// <--- maximum effectiveness (1.0) ---> start <--- linear change to effectiveness ---> end <--- no effectiveness (0.0) --->
	// Effect in the negative direction:
	// <--- no effectiveness (0.0) ---> end <--- linear change to effectiveness ---> start <--- maximum effectiveness (1.0) --->

	if ((start < end && position <= start) || (start >= end && position >= start))
	{
		return 1.0;
	}
	else if ((start < end && position >= end) || (start >= end && position <= end))
	{
		return 0.0;
	}
	else
	{
		return 1.0 - ((position - start) / (end - start));
	}
}
constexpr double linear_range(const double coefficient, const double min, const double max)
{
	return (max - min) * coefficient + min;
}

class PIDController
{
public:
	PIDController(const double output_min, const double output_max, const double Kp, const double Ki, const double Kd)
		: output_min(output_min), output_max(output_max),
		  Kp(Kp), Kd(Kd), Ki(Ki),
		  integral(0),
		  last_error(0), last_output(0)  {};
	void Modify(const double new_output_min, const double new_output_max, const double new_Kp, const double new_Ki, const double new_Kd)
	{
		output_min = new_output_min;
		output_max = new_output_max;
		Kp = new_Kp;
		Ki = new_Ki;
		Kd = new_Kd;
	}
	double Update(const double error, const double dt)
	{
		// Proportional term
		auto P = Kp * error;

		integral += (last_error * dt) + (0.5 * dt * (error - last_error));
		auto I = Ki * integral;

		// Derivative term
		auto D = Kd * ((error - last_error) / dt);

		auto output = P + I + D;
		output = clamp(output, output_min, output_max);

		//printf("Err = %lf, Dt = %lf, LastErr = %lf, LastOut = %lf, Int = %lf, P = %lf, I = %lf, D = %lf, Out = %lf\n", error, dt, last_error, last_output, integral, P, I, D, output);

		// Save terms
		last_output = output;
		last_error = error;

		return output;
	}
	void Reset()
	{
		integral = 0;
		last_error = 0;
		last_output = 0;
	}
protected:
	double output_min, output_max;
	double Kp, Kd, Ki;
	double integral;
	double last_error, last_output;
};

class AntiWindupPIDController : public PIDController
{
public:
	AntiWindupPIDController(const double output_min, const double output_max, const double Kp, const double Ki, const double Kd)
		: PIDController(output_min, output_max, Kp, Ki, Kd) {};
	double Update(const double error, const double dt)
	{
		// Guard against integrator windup
		if (last_output >= output_min && last_output <= output_max || sign(error) != sign(last_output))
		{
			integral -= (last_error * dt) + (0.5 * dt * (error - last_error));
		}
		return PIDController::Update(error, dt);
	}
};

/*
 * ========================== *
 * SimVars via non-SimConnect *
 * ========================== *
 */
struct SIM_VARS
{
	// Units
	ENUM bool_units; // Bool
	ENUM degrees_units; // Degrees
	ENUM degrees_per_second_units; // Degrees per second
	ENUM feet_units; // Feet
	ENUM feet_per_second_units; // Feet per second
	ENUM gforce_units; // GForce
	ENUM knots_units; // Knots
	ENUM mach_units; // Mach
	ENUM number_units; // Number
	ENUM percent_units; // Percent
	ENUM radians_units; // Radians
	// Variables
	ENUM airspeed_barber_pole; // AIRSPEED BARBER POLE
	ENUM airspeed_indicated; // AIRSPEED INDICATED
	ENUM airspeed_mach; // AIRSPEED MACH
	ENUM airspeed_true; // AIRSPEED TRUE
	ENUM autopilot_master; // AUTOPILOT MASTER
	ENUM barber_pole_mach; // BARBER POLE MACH
	ENUM elevator_deflection; // ELEVATOR DEFLECTION
	ENUM elevator_deflection_pct; // ELEVATOR DEFLECTION PCT
	ENUM flaps_handle_index; // FLAPS HANDLE INDEX
	ENUM gforce; // G FORCE
	ENUM incidence_alpha; // INCIDENCE ALPHA
	ENUM plane_bank_degrees; // PLANE BANK DEGREES
	ENUM plane_pitch_degrees; // PLANE PITCH DEGREES
	ENUM radio_height; // RADIO HEIGHT
	ENUM rotation_velocity_body_x; // ROTATION VELOCITY BODY X
	ENUM rotation_velocity_body_y; // ROTATION VELOCITY BODY Y
	ENUM sim_on_ground; // SIM ON GROUND
	ENUM stall_alpha; // STALL ALPHA
	ENUM vertical_speed; // VERTICAL SPEED
} sim_vars;

bool HandleSimVars(FsContext ctx, const int service_id, void* pData)
{
	switch (service_id)
	{
	case PANEL_SERVICE_PRE_INSTALL:
	{
		// Units
		sim_vars.bool_units = get_units_enum("Bool");
		sim_vars.degrees_units = get_units_enum("Degrees");
		sim_vars.degrees_per_second_units = get_units_enum("Degrees per second");
		sim_vars.feet_units = get_units_enum("Feet");
		sim_vars.feet_per_second_units = get_units_enum("Feet per second");
		sim_vars.gforce_units = get_units_enum("GForce");
		sim_vars.knots_units = get_units_enum("Knots");
		sim_vars.mach_units = get_units_enum("Mach");
		sim_vars.number_units = get_units_enum("Number");
		sim_vars.percent_units = get_units_enum("Percent");
		sim_vars.radians_units = get_units_enum("Radians");
		// Variables
		sim_vars.airspeed_barber_pole = get_aircraft_var_enum("AIRSPEED BARBER POLE");
		sim_vars.airspeed_indicated = get_aircraft_var_enum("AIRSPEED INDICATED");
		sim_vars.airspeed_mach = get_aircraft_var_enum("AIRSPEED MACH");
		sim_vars.airspeed_true = get_aircraft_var_enum("AIRSPEED TRUE");
		sim_vars.autopilot_master = get_aircraft_var_enum("AUTOPILOT MASTER");
		sim_vars.barber_pole_mach = get_aircraft_var_enum("BARBER POLE MACH");
		sim_vars.elevator_deflection = get_aircraft_var_enum("ELEVATOR DEFLECTION");
		sim_vars.elevator_deflection_pct = get_aircraft_var_enum("ELEVATOR DEFLECTION PCT");
		sim_vars.flaps_handle_index = get_aircraft_var_enum("FLAPS HANDLE INDEX");
		sim_vars.gforce = get_aircraft_var_enum("G FORCE");
		sim_vars.incidence_alpha = get_aircraft_var_enum("INCIDENCE ALPHA");
		sim_vars.plane_bank_degrees = get_aircraft_var_enum("PLANE BANK DEGREES");
		sim_vars.plane_pitch_degrees = get_aircraft_var_enum("PLANE PITCH DEGREES");
		sim_vars.radio_height = get_aircraft_var_enum("RADIO HEIGHT");
		sim_vars.rotation_velocity_body_x = get_aircraft_var_enum("ROTATION VELOCITY BODY X");
		sim_vars.rotation_velocity_body_y = get_aircraft_var_enum("ROTATION VELOCITY BODY Y");
		sim_vars.sim_on_ground = get_aircraft_var_enum("SIM ON GROUND");
		sim_vars.stall_alpha = get_aircraft_var_enum("STALL ALPHA");
		sim_vars.vertical_speed = get_aircraft_var_enum("VERTICAL SPEED");
		return true;
	}
	break;
	case PANEL_SERVICE_POST_INSTALL:
	{
		return true;
	}
	break;
	case PANEL_SERVICE_PRE_DRAW:
	{
		return true;
	}
	break;
	case PANEL_SERVICE_PRE_KILL:
	{
		return true;
	}
	break;
	}
	return false;
}

/*
 * =================== *
 * SimConnect          *
 * =================== *
 */
HANDLE hSimConnect;
bool HandleSimConnect(FsContext ctx, const int service_id, void* pData)
{
	switch (service_id)
	{
	case PANEL_SERVICE_PRE_INSTALL:
	{
		return true;
	}
	break;
	case PANEL_SERVICE_POST_INSTALL:
	{
		if (SUCCEEDED(SimConnect_Open(&hSimConnect, "A32NX", nullptr, 0, 0, 0)))
		{
			printf("A32NX: Connected via SimConnect\n");
			return true;
		}
		printf("A32NX: Failed to connect via SimConnect\n");
		return false;
	}
	break;
	case PANEL_SERVICE_PRE_DRAW:
	{
		return true;
	}
	break;
	case PANEL_SERVICE_PRE_KILL:
	{
		if (hSimConnect)
		{
			SimConnect_Close(hSimConnect);
			hSimConnect = 0;
		}
		return true;
	}
	break;
	default: break;
	}
	return false;
}

/*
 * ============================== *
 * Flight Controls/Path Variables *
 * ============================== *
 */

/*
 * Describes the current status of the plane
 */
struct FLIGHT_PATH_DATA
{
	double airspeed_indicated = 0; // knots
	double airspeed_mach = 0; // mach
	bool autopilot_enabled = false; // autopilot enabled
	double aoa = 0; // degrees
	double gforce = 1; // gforce
	double gforce_rate = 0; // gforce rate
	int flaps_pos = 0; // number (0-4)
	double mmo = DBL_MAX; // mach
	double pitch = 0; // degrees
	double pitch_rate = 0; // degrees per second
	double radio_height = 0; // feet
	double roll = 0; // degrees (rolled right == positive numbers, rolled left == negative numbers)
	double roll_rate = 0; // degrees per second
	double stall_alpha = DBL_MAX; // degrees
	double vertical_fpa = 0; // degrees
	double vertical_fpa_rate = 0; // degrees per second
	double vertical_speed = 0; // feet per second
	double vertical_speed_rate = 0; // feet per second per second
	double vmo = DBL_MAX; // knots
} flight_path_data;

// TODO: Can we calculate alpha prot, alpha floor, and alpha max using the stall alpha?

double GetAlphaFloorAngle()
{
	// These values are hardcoded in the FCOM in 1.27.20 under "High Angle of Attack Protection"
	// Note: 2. a.floor is activated through A/THR system when:
	// - a > a floor (9.5 degrees in configuration 0; 15 degrees in configuration 1, 2; 14 degrees in
	//   configuration 3; 13 degrees in configuration FULL), or,...
	switch (flight_path_data.flaps_pos)
	{
	case 0: // Flaps up
		return 9.5;
		break;
	case 1: // Flaps 1
		return 15;
		break;
	case 2: // Flaps 2
		return 15;
		break;
	case 3: // Flaps 3
		return 14;
		break;
	case 4: // Flaps full
		return 13;
		break;
	default: // Unreachable
		return 9.5;
		break;
	}
}

double GetAlphaProtAngle()
{
	// This ratio was estimated using the graph in the FCOM in 1.27.20 under "High Angle of Attack Protection"
	// The graph plots CL (lift coefficient) to alpha.
	// The ratio was guesstimated using a ruler and hoping the graph was accurate.
	const auto ratio_with_alpha_floor = 19.0 / 21.0;
	return ratio_with_alpha_floor * GetAlphaFloorAngle();
}

double GetAlphaMaxAngle()
{
	// This ratio was estimated using the graph in the FCOM in 1.27.20 under "High Angle of Attack Protection"
	// The graph plots CL (lift coefficient) to alpha.
	// The ratio was guesstimated using a ruler and hoping the graph was accurate.
	const auto ratio_with_alpha_floor = 7.0 / 6.0;
	return ratio_with_alpha_floor * GetAlphaFloorAngle();
}

bool HandleFlightPathDataUpdate(FsContext ctx, const int service_id, void* pData)
{
	switch (service_id)
	{
	case PANEL_SERVICE_PRE_INSTALL:
	{
		return true;
	}
	break;
	case PANEL_SERVICE_POST_INSTALL:
	{
		return true;
	}
	break;
	case PANEL_SERVICE_PRE_DRAW:
	{
		// Sent before the gauge is drawn. The pData parameter points to a sGaugeDrawData structure:
		// - The dt member gives the time elapsed since last frame.
		auto* p_draw_data = static_cast<sGaugeDrawData*>(pData);
		auto dt = p_draw_data->dt;

		// Update indicated airspeed info
		flight_path_data.airspeed_indicated = aircraft_varget(sim_vars.airspeed_indicated, sim_vars.knots_units, 0);
		flight_path_data.airspeed_indicated = isnan(flight_path_data.airspeed_indicated) ? 0 : flight_path_data.airspeed_indicated;

		// Update mach airspeed info
		flight_path_data.airspeed_mach = aircraft_varget(sim_vars.airspeed_mach, sim_vars.mach_units, 0);
		flight_path_data.airspeed_mach = isnan(flight_path_data.airspeed_mach) ? 0 : flight_path_data.airspeed_mach;

		// Update AoA info
		flight_path_data.aoa = aircraft_varget(sim_vars.incidence_alpha, sim_vars.degrees_units, 0);
		flight_path_data.aoa = isnan(flight_path_data.aoa) ? 0 : flight_path_data.aoa;

		// Update autopilot info
		flight_path_data.autopilot_enabled = aircraft_varget(sim_vars.autopilot_master, sim_vars.bool_units, 0) == 1.0;

		// Update gforce info
		auto last_gforce = flight_path_data.gforce;
		flight_path_data.gforce = aircraft_varget(sim_vars.gforce, sim_vars.gforce_units, 0);
		flight_path_data.gforce = isnan(flight_path_data.gforce) ? 1 : flight_path_data.gforce;
		flight_path_data.gforce_rate = (flight_path_data.gforce - last_gforce) / dt;

		// Update flaps info
		flight_path_data.flaps_pos = aircraft_varget(sim_vars.flaps_handle_index, sim_vars.number_units, 0);
		flight_path_data.flaps_pos = isnan(flight_path_data.flaps_pos) ? 0 : flight_path_data.flaps_pos;

		// Update mmo info
		flight_path_data.mmo = aircraft_varget(sim_vars.barber_pole_mach, sim_vars.mach_units, 0);
		flight_path_data.mmo = isnan(flight_path_data.mmo) ? DBL_MAX : flight_path_data.mmo;
			
		// Update pitch info
		auto last_pitch = flight_path_data.pitch;
		flight_path_data.pitch = -aircraft_varget(sim_vars.plane_pitch_degrees, sim_vars.degrees_units, 0);
		flight_path_data.pitch = isnan(flight_path_data.pitch) ? 0 : flight_path_data.pitch;
		flight_path_data.pitch_rate = (flight_path_data.pitch - last_pitch) / dt;

		// Update radio height
		flight_path_data.radio_height = aircraft_varget(sim_vars.radio_height, sim_vars.feet_units, 0);
		flight_path_data.radio_height = isnan(flight_path_data.radio_height) ? 0 : flight_path_data.radio_height;
			
		// Update roll info
		double last_roll = flight_path_data.roll;
		flight_path_data.roll = -aircraft_varget(sim_vars.plane_bank_degrees, sim_vars.degrees_units, 0);
		flight_path_data.roll = isnan(flight_path_data.roll) ? 0 : flight_path_data.roll;
		flight_path_data.roll_rate = (flight_path_data.roll - last_roll) / dt;

		// Update stall alpha info
		flight_path_data.stall_alpha = aircraft_varget(sim_vars.stall_alpha, sim_vars.degrees_units, 0);
		flight_path_data.stall_alpha = isnan(flight_path_data.stall_alpha) ? DBL_MAX : flight_path_data.stall_alpha;

		// Update vertical fpa info
		const auto last_vertical_fpa = flight_path_data.vertical_fpa;
		// Vertical FPA = arctan(vertical speed / true airspeed)
		auto vertical_speed = aircraft_varget(sim_vars.vertical_speed, sim_vars.feet_per_second_units, 0);
		vertical_speed = isnan(vertical_speed) ? 0 : vertical_speed;
		auto true_airspeed = aircraft_varget(sim_vars.airspeed_true, sim_vars.knots_units, 0);
		true_airspeed = isnan(true_airspeed) ? 0 : true_airspeed;
		if (true_airspeed == 0 && vertical_speed == 0) flight_path_data.vertical_fpa = 0; // neutral
		else if (true_airspeed == 0 && vertical_speed < 0) flight_path_data.vertical_fpa = -90; // straight down
		else if (true_airspeed == 0 && vertical_speed > 0) flight_path_data.vertical_fpa = 90; // straight up
		else
		{
			const auto knots_to_feet_per_minute_ratio = (60.0 / 1.0) * (1.0 / 6076.0); // (60 min / 1 h) * (1 nm / 6076 ft)
			flight_path_data.vertical_fpa = degrees(atan(vertical_speed * (1 / true_airspeed) * knots_to_feet_per_minute_ratio));
		}
		flight_path_data.vertical_fpa_rate = (flight_path_data.vertical_fpa - last_vertical_fpa) / dt;

		// Update vertical speed info
		const auto last_vertical_speed = flight_path_data.vertical_speed;
		flight_path_data.vertical_speed = aircraft_varget(sim_vars.vertical_speed, sim_vars.feet_per_second_units, 0);
		flight_path_data.vertical_speed = isnan(flight_path_data.vertical_speed) ? 0 : flight_path_data.vertical_speed;
		flight_path_data.vertical_speed_rate = (flight_path_data.vertical_speed - last_vertical_speed) / dt;

		// Update vmmo info
		flight_path_data.vmo = aircraft_varget(sim_vars.airspeed_barber_pole, sim_vars.knots_units, 0);
		flight_path_data.vmo = isnan(flight_path_data.vmo) ? DBL_MAX : flight_path_data.vmo;

		return true;
	}
	break;
	case PANEL_SERVICE_PRE_KILL:
	{
		return true;
	}
	break;
	default: break;
	}
	return false;
}

/*
 * =================== *
 * FLIGHT CONTROL LAWS *
 * =================== *
 */
// TODO: Handle updating/querying more laws/states
static bool in_normal_law = true;
bool InNormalLaw()
{
	return in_normal_law;
}
const char * IN_NORMAL_LAW_VAR_NAME = "IN NORMAL LAW";
bool HandleFlightControlLaws(FsContext ctx, const int service_id, void* pData)
{
	switch (service_id)
	{
	case PANEL_SERVICE_PRE_INSTALL:
	{
		register_var_by_name(&in_normal_law, TYPE_BOOL, const_cast<PSTRINGZ>(IN_NORMAL_LAW_VAR_NAME));
		return true;
	}
	break;
	case PANEL_SERVICE_POST_INSTALL:
	{
		return true;
	}
	break;
	case PANEL_SERVICE_PRE_DRAW:
	{
		return true;
	}
	break;
	case PANEL_SERVICE_PRE_KILL:
	{
		unregister_var_by_name(const_cast<PSTRINGZ>(IN_NORMAL_LAW_VAR_NAME));
		return true;
	}
	break;
	default: break;
	}
	return false;
}

/*
 * ================== *
 * PITCH CONTROL MODE *
 * ================== *
 */
enum PITCH_CONTROL_MODE
{
	GROUND_MODE,
	FLIGHT_MODE,
	FLARE_MODE
};
PITCH_CONTROL_MODE pitch_control_mode = GROUND_MODE;
double pitch_control_mode_ground_effect = 1.0;
double pitch_control_mode_flight_effect = 0.0;
double pitch_control_mode_flare_effect = 0.0;
double saved_flare_pitch_attitude = 0; // The attitude of the airplane at 50 feet RA
const char* PITCH_CONTROL_MODE_VAR_NAME = "PITCH CONTROL MODE";

void PitchControlMode_BlendEffect(double * blend_in, double * blend_out, const double increment)
{
	*blend_in = clamp(*blend_in + increment, 0, 1);
	*blend_out = clamp(*blend_out - increment, 0, 1);
}

double ground_to_flight_condition_start_time = 0;
void PitchControlMode_HandleGroundTransitions(const double dt)
{
	if (InNormalLaw())
	{
		// Handle ground to flight transition
		auto radio_altimeter = aircraft_varget(sim_vars.radio_height, sim_vars.feet_units, 0);
		auto in_flight = aircraft_varget(sim_vars.sim_on_ground, sim_vars.bool_units, 0) == 0;
		auto pitch_attitude = aircraft_varget(sim_vars.plane_pitch_degrees, sim_vars.degrees_units, 0);
		if ((radio_altimeter > 50) || (in_flight && (pitch_attitude > 8)))
		{
			// Blend in flight effect and blend out ground effect
			PitchControlMode_BlendEffect(&pitch_control_mode_flight_effect, &pitch_control_mode_ground_effect, dt / 5);
			if (pitch_control_mode_flight_effect == 1)
			{
				pitch_control_mode = FLIGHT_MODE;
				pitch_control_mode_ground_effect = 0;
				pitch_control_mode_flare_effect = 0;
			}
		}
		else
		{
			// Blend in ground effect and blend out flight effect
			PitchControlMode_BlendEffect(&pitch_control_mode_ground_effect, &pitch_control_mode_flight_effect, dt / 5);
		}
	}
	// TODO: Handle other laws
}

void PitchControlMode_HandleFlightTransitions(const double dt)
{
	if (InNormalLaw())
	{
		// Handle flight to flare transition
		auto radio_altimeter = aircraft_varget(sim_vars.radio_height, sim_vars.feet_units, 0);
		if (radio_altimeter <= 50)
		{
			if (pitch_control_mode_flare_effect == 0)
			{
				// First time blending in flare, so let's save the pitch attitude
				// Why? The FCOM says so:
				// "The system memorizes the attitude at 50 feet, and that attitude becomes the initial reference for pitch attitude control."
				saved_flare_pitch_attitude = flight_path_data.pitch;
			}
			
			// Blend in flare effect and blend out flight effect
			PitchControlMode_BlendEffect(&pitch_control_mode_flare_effect, &pitch_control_mode_flight_effect, dt / 1);
			if (pitch_control_mode_flare_effect == 1)
			{
				pitch_control_mode = FLARE_MODE;
				pitch_control_mode_ground_effect = 0;
				pitch_control_mode_flight_effect = 0;
			}
		}
		else
		{
			// Blend in flight effect and blend out flare effect
			PitchControlMode_BlendEffect(&pitch_control_mode_flight_effect, &pitch_control_mode_flare_effect, dt / 1);
		}
	}
	// TODO: Handle other laws
}

void PitchControlMode_HandleFlareTransitions(const double dt)
{
	if (pitch_control_mode == FLARE_MODE)
	{
		if (InNormalLaw())
		{
			const auto radio_altimeter = aircraft_varget(sim_vars.radio_height, sim_vars.feet_units, 0);
			const auto on_ground = aircraft_varget(sim_vars.sim_on_ground, sim_vars.bool_units, 0) != 0;
			const auto pitch_attitude = aircraft_varget(sim_vars.plane_pitch_degrees, sim_vars.degrees_units, 0);
			// Handle flare to flight transition
			if (radio_altimeter > 50)
			{
				// Blend in flight effect and blend out flare effect
				PitchControlMode_BlendEffect(&pitch_control_mode_flight_effect, &pitch_control_mode_flare_effect, dt / 1);
				if (pitch_control_mode_flight_effect == 1)
				{
					pitch_control_mode = FLIGHT_MODE;
					pitch_control_mode_ground_effect = 0;
					pitch_control_mode_flare_effect = 0;
				}
			}
			// Handle flare to ground transition
			else if (on_ground && (pitch_attitude < 2.5))
			{
				// Blend in ground effect and blend out flare effect
				PitchControlMode_BlendEffect(&pitch_control_mode_ground_effect, &pitch_control_mode_flare_effect, dt / 5);
				if (pitch_control_mode_ground_effect == 1)
				{
					pitch_control_mode = GROUND_MODE;
					pitch_control_mode_flight_effect = 0;
					pitch_control_mode_flare_effect = 0;
				}
			}
			else
			{
				// Blend in flare effect and blend out flight and ground effects
				if (pitch_control_mode_ground_effect > 0)
				{
					PitchControlMode_BlendEffect(&pitch_control_mode_flare_effect, &pitch_control_mode_ground_effect, dt / 5);
				}
				if (pitch_control_mode_flight_effect > 0)
				{
					PitchControlMode_BlendEffect(&pitch_control_mode_flare_effect, &pitch_control_mode_flight_effect, dt / 1);
				}
			}
		}
		// TODO: Handle other laws
	}
}

bool HandlePitchControlMode(FsContext ctx, const int service_id, void* pData)
{
	switch (service_id)
	{
	case PANEL_SERVICE_PRE_INSTALL:
	{
		register_var_by_name(&pitch_control_mode, TYPE_ENUM, const_cast<PSTRINGZ>(PITCH_CONTROL_MODE_VAR_NAME));
		return true;
	}
	break;
	case PANEL_SERVICE_POST_INSTALL:
	{	
		return true;
	}
	break;
	case PANEL_SERVICE_PRE_DRAW:
	{	
		// Sent before the gauge is drawn. The pData parameter points to a sGaugeDrawData structure:
		// - The dt member gives the time elapsed since last frame.
		auto* p_draw_data = static_cast<sGaugeDrawData*>(pData);
		const auto dt = p_draw_data->dt;
		switch (pitch_control_mode)
		{
		case GROUND_MODE:
			PitchControlMode_HandleGroundTransitions(dt);
			break;
		case FLIGHT_MODE:
			PitchControlMode_HandleFlightTransitions(dt);
			break;
		case FLARE_MODE:
			PitchControlMode_HandleFlareTransitions(dt);
			break;
		}
		return true;
	}
	break;
	case PANEL_SERVICE_PRE_KILL:
	{
		unregister_var_by_name(const_cast<PSTRINGZ>(PITCH_CONTROL_MODE_VAR_NAME));
		return true;
	}
	break;
	default: break;
	}
	return false;
}

/*
 * ===================== *
 * INPUT CONTROL CAPTURE *
 * ===================== *
 */
enum GROUP_ID
{
	ELEVATOR_GROUP,
	AILERON_GROUP,
	RUDDER_GROUP
};

enum EVENT_ID
{
	// Elevator Group
	ELEVATOR_SET_EVENT, // AXIS_ELEVATOR_SET
	// Aileron Group
	AILERONS_SET_EVENT, // AXIS_AILERONS_SET
	CENTER_AILERONS_RUDDER_EVENT, // CENTER_AILER_RUDDER
	// Rudder group
	RUDDER_SET_EVENT, // AXIS_RUDDER_SET
	RUDDER_CENTER_EVENT, // RUDDER_CENTER
};

/*
 * Describes the current user input of the control surfaces (yoke + rudder)
 */
struct USER_INPUT
{
	double yoke_y; // -1 is full down, and +1 is full up
	double yoke_x; // -1 is full left, and +1 is full right
	double rudder; // -1 is full left, and +1 is full right
} user_input;

void CALLBACK OnInputCaptureEvent(SIMCONNECT_RECV* pData, DWORD cbData, void* pContext)
{
	if (pData->dwID == SIMCONNECT_RECV_ID_EVENT)
	{
		auto evt = static_cast<SIMCONNECT_RECV_EVENT*>(pData);
		switch (evt->uEventID)
		{
		case ELEVATOR_SET_EVENT:
			user_input.yoke_y = 0 - (static_cast<long>(evt->dwData) / 16384.0); // scale from [-16384,16384] to [-1,1] and reverse the sign
			break;
		case AILERONS_SET_EVENT:
			user_input.yoke_x = 0 - (static_cast<long>(evt->dwData) / 16384.0); // scale from [-16384,16384] to [-1,1] and reverse the sign
			break;
		case CENTER_AILERONS_RUDDER_EVENT:
			user_input.yoke_x = 0;
			user_input.rudder = 0;
			break;
		case RUDDER_SET_EVENT:
			user_input.rudder = 0 - (static_cast<long>(evt->dwData) / 16384.0); // scale from [-16384,16384] to [-1,1] and reverse the sign
			break;
		case RUDDER_CENTER_EVENT:
			user_input.rudder = 0;
			break;
		}
	}
}

bool HandleInputCapture(FsContext ctx, const int service_id, void* pData)
{
	switch (service_id)
	{
	case PANEL_SERVICE_PRE_INSTALL:
	{
		return true;
	}
	break;
	case PANEL_SERVICE_POST_INSTALL:
	{
		// Register input capture
		// Client events reference: http://www.prepar3d.com/SDKv3/LearningCenter/utilities/variables/event_ids.html
		// Elevator group
		SimConnect_MapClientEventToSimEvent(hSimConnect, ELEVATOR_SET_EVENT, "AXIS_ELEVATOR_SET");
		SimConnect_AddClientEventToNotificationGroup(hSimConnect, ELEVATOR_GROUP, ELEVATOR_SET_EVENT, TRUE);

		// Aileron group
		SimConnect_MapClientEventToSimEvent(hSimConnect, AILERONS_SET_EVENT, "AXIS_AILERONS_SET");
		SimConnect_MapClientEventToSimEvent(hSimConnect, CENTER_AILERONS_RUDDER_EVENT, "CENTER_AILER_RUDDER");
		SimConnect_AddClientEventToNotificationGroup(hSimConnect, AILERON_GROUP, AILERONS_SET_EVENT, TRUE);
		SimConnect_AddClientEventToNotificationGroup(hSimConnect, AILERON_GROUP, CENTER_AILERONS_RUDDER_EVENT, TRUE);

		// Rudder group
		SimConnect_MapClientEventToSimEvent(hSimConnect, RUDDER_SET_EVENT, "AXIS_RUDDER_SET");
		SimConnect_MapClientEventToSimEvent(hSimConnect, RUDDER_CENTER_EVENT, "RUDDER_CENTER");
		SimConnect_AddClientEventToNotificationGroup(hSimConnect, RUDDER_GROUP, RUDDER_SET_EVENT, TRUE);
		SimConnect_AddClientEventToNotificationGroup(hSimConnect, RUDDER_GROUP, RUDDER_CENTER_EVENT, TRUE);

		// Set maskable notification priorities
		SimConnect_SetNotificationGroupPriority(hSimConnect, ELEVATOR_GROUP, SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE);
		SimConnect_SetNotificationGroupPriority(hSimConnect, AILERON_GROUP, SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE);
		SimConnect_SetNotificationGroupPriority(hSimConnect, RUDDER_GROUP, SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE);
		return true;
	}
	break;
	case PANEL_SERVICE_PRE_DRAW:
	{
		SimConnect_CallDispatch(hSimConnect, OnInputCaptureEvent, nullptr);
		return true;
	}
	break;
	case PANEL_SERVICE_PRE_KILL:
	{
		return true;
	}
	break;
	default: break;
	}
	return false;
}

/*
 * ====================== *
 * Flight Controls System *
 * ====================== *
 */

/*
 * Describes the current position of the control surfaces
 */
struct CONTROL_SURFACES_DATA
{
	double elevator = 0; // -1 is full down, and +1 is full up
	double aileron = 0; // -1 is full left, and +1 is full right
	double rudder = 0; // -1 is full left, and +1 is full right
} control_surfaces;

enum DEFINITION_ID
{
	CONTROL_SURFACES_DEFINITION
};

double FlightControlSystem_GetUserYokeYPosition()
{
	// Get the user stick position with a null zone
	const auto null_zone_error = 0.05 * 2; // 5% of the stick movement is null (2 units of total travel distance over [-1,1])
	return fabs(user_input.yoke_y) < null_zone_error ? 0 : user_input.yoke_y;
}

double FlightControlSystem_GetUserYokeXPosition()
{
	// Get the user stick position with a null zone
	const auto null_zone_error = 0.05 * 2; // 5% of the stick movement is null (2 units of total travel distance over [-1,1])
	return fabs(user_input.yoke_x) < null_zone_error ? 0 : user_input.yoke_x;
}

struct NORMAL_LAW_PROTECTIONS
{
	double maximum_bank_angle = 67; // Maximum bank angle in degrees
	                                // - Normally: 67 degrees
	                                // - High Angle of Attack Protection: 45 degrees
	                                // - High Speed Protection: 45 degrees
	double nominal_bank_angle = 33; // Spiral static stability in degrees
	                                // - Normally: 33 degrees
	                                // - High Angle of Attack Protection: 0 degrees
	                                // - High Speed Protection: 0 degrees
	double min_load_factor = -1; // -1g for clean configuration, 0g for other configurations
	double max_load_factor = 2.5; // 2.5g for clean configuration, 2g for other configurations
	double max_pitch_angle = 30; // Maximum pitch attitude in degrees
	                                 // 30 degrees nose up in conf 0-3 (progressively reduced to 25 degrees at low speed)
	                                 // 25 degrees nose up in conf FULL (progressively reduced to 20 degrees at low speed)
	                                 // TODO: To implement the 'progressively reduced' limitation, we need to find a way to
	                                 //       calculate speeds like V_alpha_prot or V_alpha_max, for which I think we will
	                                 //       have to work out via experimentation.
	double min_pitch_angle = -15; // Minimum pitch attitude in degrees
	
	double last_update = 0; // Last time we updated any of these values
	bool aoa_demand_active = false;
	double aoa_demand_deactivation_timer = 0;
	bool high_speed_protection_active = false;
} normal_law_protections;


void FlightControlSystem_CalculateNormalLawSettings(const double t, const double dt)
{
	if (normal_law_protections.last_update != t)
	{
		// Do the calculation only if we haven't yet
		normal_law_protections.last_update = t;

		// Check if we are in AoA demand mode (as dictated by High Angle of Attack Protection)
		if (normal_law_protections.aoa_demand_active)
		{
			// Should we leave AoA demand mode?
			// Exit condition 1: Sidestick must be pushed more than 8 degrees forward (assuming this is ~50% down)
			const auto condition1 = FlightControlSystem_GetUserYokeYPosition() <= -0.5;
			// Exit condition 2: Sidestick must be pushed more than 0.5 degrees forward for at least 0.5 seconds when alpha < alpha_max
			const auto condition2 = normal_law_protections.aoa_demand_deactivation_timer >= 0.5;
			if (condition1 || condition2)
			{
				normal_law_protections.aoa_demand_active = false;
				normal_law_protections.aoa_demand_deactivation_timer = 0;
			}
			else if (FlightControlSystem_GetUserYokeYPosition() < 0 && flight_path_data.aoa < GetAlphaMaxAngle())
			{
				// We're still building the target duration to meet condition 2
				normal_law_protections.aoa_demand_deactivation_timer += dt;
			}
			else
			{
				// We don't match any of the exit conditions
				normal_law_protections.aoa_demand_deactivation_timer = 0;
			}
		}
		else
		{
			// Should we enter AoA demand mode?
			// Enter condition 1: Sidestick must not be pushed down, and AoA is greater than alpha_prot
			const auto condition1 = FlightControlSystem_GetUserYokeYPosition() >= 0 && flight_path_data.aoa > GetAlphaProtAngle();
			// Enter condition 2: We are at or above alpha max
			const auto condition2 = flight_path_data.aoa >= GetAlphaMaxAngle();
			if (condition1 || condition2)
			{
				normal_law_protections.aoa_demand_active = true;
				normal_law_protections.aoa_demand_deactivation_timer = 0;
			}
		}

		// Check if high speed protection is active
		normal_law_protections.high_speed_protection_active = flight_path_data.airspeed_indicated > flight_path_data.vmo
		                                                   || flight_path_data.airspeed_mach > flight_path_data.mmo;

		// Update bank angles
		if (normal_law_protections.aoa_demand_active || normal_law_protections.high_speed_protection_active)
		{
			normal_law_protections.maximum_bank_angle = 45;
			normal_law_protections.nominal_bank_angle = 0;
		}
		else
		{
			normal_law_protections.maximum_bank_angle = 67;
			normal_law_protections.nominal_bank_angle = 33;
		}

		// Update load and pitch factors
		switch (flight_path_data.flaps_pos)
		{
		case 0:
			normal_law_protections.min_load_factor = -1;
			normal_law_protections.max_load_factor = 2.5;
			normal_law_protections.max_pitch_angle = 30;
			break;
		case 1:
			normal_law_protections.min_load_factor = 0;
			normal_law_protections.max_load_factor = 2;
			normal_law_protections.max_pitch_angle = 30;
			break;
		case 2:
			normal_law_protections.min_load_factor = 0;
			normal_law_protections.max_load_factor = 2;
			normal_law_protections.max_pitch_angle = 30;
			break;
		case 3:
			normal_law_protections.min_load_factor = 0;
			normal_law_protections.max_load_factor = 2;
			normal_law_protections.max_pitch_angle = 30;
			break;
		case 4:
			normal_law_protections.min_load_factor = 0;
			normal_law_protections.max_load_factor = 2;
			normal_law_protections.max_pitch_angle = 25;
			break;
		default: // Unreachable
			normal_law_protections.min_load_factor = -1;
			normal_law_protections.max_load_factor = 2.5;
			normal_law_protections.max_pitch_angle = 30;
			break;
		}
	}
}

static PIDController aoa_controller(-2, 2, 0.002, 0, 0.0002); // AoA error -> elevator handle movement rate
static PIDController gforce_controller(-2, 2, 0.01, 0, 0); // GForce error -> elevator handle movement rate
static PIDController speed_knots_controller(-0.1, 0.1, -0.001, 0, -0.0001); // Knots speed error -> elevator handle movement rate
static PIDController speed_mach_controller(-0.1, 0.1, -0.001, 0, -0.0001); // Mach speed error -> elevator handle movement rate
static PIDController vertical_fpa_controller(-2, 2, 0.04, 0.001, 0.1); // Vertical FPA error -> elevator handle movement rate
static PIDController pitch_angle_controller(-2, 2, 0.005, 0, 0.0002); // Pitch angle error -> elevator handle movement rate
static PIDController pitch_rate_controller(-2, 2, 0.006, 0, 0.0008); // Pitch rate error -> elevator handle movement rate

double pitch_hold_time = 0;
double held_vertical_fpa = 0;
double FlightControlSystem_LoadFactorLimitation(double const elevator_position_change, const double dt)
{
	const auto clamping_zone = 0.25; // When we're within 0.25g of our LF limitations, reduce elevator authority
	
	if (flight_path_data.gforce > normal_law_protections.max_load_factor)
	{
		const auto proposed_elevator_position_change = gforce_controller.Update(normal_law_protections.max_load_factor - flight_path_data.gforce, dt);
		const auto new_elevator_position_change = min(elevator_position_change, proposed_elevator_position_change);
		printf(", LF_MAX_VIOL: LF = %lf, PreElevD = %lf, PostElevD = %lf", flight_path_data.gforce, elevator_position_change, new_elevator_position_change);
		return new_elevator_position_change;
	}
	else if (flight_path_data.gforce < normal_law_protections.min_load_factor)
	{
		const auto proposed_elevator_position_change = gforce_controller.Update(normal_law_protections.min_load_factor - flight_path_data.gforce, dt);
		const auto new_elevator_position_change = max(elevator_position_change, proposed_elevator_position_change);
		printf(", LF_MIN_VIOL: LF = %lf, PreElevD = %lf, PostElevD = %lf", flight_path_data.gforce, elevator_position_change, new_elevator_position_change);
		return new_elevator_position_change;
	}
	else if (flight_path_data.gforce + clamping_zone >= normal_law_protections.max_load_factor && elevator_position_change >= 0)
	{
		const auto new_elevator_position_change = elevator_position_change * linear_decay_coefficient(flight_path_data.gforce, normal_law_protections.max_load_factor - clamping_zone, normal_law_protections.max_load_factor);
		printf(", LF_MAX: LF = %lf, PreElevD = %lf, PostElevD = %lf", flight_path_data.gforce, elevator_position_change, new_elevator_position_change);
		return new_elevator_position_change;
	}
	else if (flight_path_data.gforce - clamping_zone <= normal_law_protections.min_load_factor && elevator_position_change <= 0)
	{
		const auto new_elevator_position_change = elevator_position_change * linear_decay_coefficient(flight_path_data.gforce, normal_law_protections.min_load_factor + clamping_zone, normal_law_protections.min_load_factor);
		printf(", LF_MIN: LF = %lf, PreElevD = %lf, PostElevD = %lf", flight_path_data.gforce, elevator_position_change, new_elevator_position_change);
		return new_elevator_position_change;
	}
	else
	{
		return elevator_position_change;
	}
}

double FlightControlSystem_HighSpeedProtection(double const elevator_position_change, const double dt)
{
	if (normal_law_protections.high_speed_protection_active)
	{
		pitch_hold_time = 0;
		
		auto user_elevator_position_change = elevator_position_change;
		if (elevator_position_change < 0)
		{
			// The FCOM says "As the speed increases above VMO/MMO, the sidestick nose-down authority is progressively reduced"
			// Let's make the user have no authority above Vmo + 8, Mmo + 0.012 (arbitrarily chosen)
			// We'll pick whichever path leaves the user with the least control
			const auto user_elevator_position_change_knots = elevator_position_change * linear_decay_coefficient(flight_path_data.airspeed_indicated, flight_path_data.vmo, flight_path_data.vmo + 8);
			const auto user_elevator_position_change_mach = elevator_position_change * linear_decay_coefficient(flight_path_data.airspeed_mach, flight_path_data.mmo, flight_path_data.mmo + 0.012);
			user_elevator_position_change = max(user_elevator_position_change_knots, user_elevator_position_change_mach);
		}
		
		// Now let's get the nose-up input necessary from the speed controller (we'll use knots)
		// We'll aim the speed for Vmo - 1, Mmo - 0.0015 (arbitrarily chosen)
		// We'll pick whichever path gives the most positive input
		const auto pitch_up_elevator_position_change_knots = speed_knots_controller.Update((flight_path_data.vmo - 1) - flight_path_data.airspeed_indicated, dt);
		const auto pitch_up_elevator_position_change_mach = speed_mach_controller.Update((flight_path_data.mmo - 0.0015) - flight_path_data.airspeed_mach, dt);
		const auto pitch_up_elevator_position_change = max(pitch_up_elevator_position_change_knots, pitch_up_elevator_position_change_mach);

		// Let's blend the two together
		const auto new_elevator_position_change = user_elevator_position_change + pitch_up_elevator_position_change;
		printf(", OVERSPD: PreElevD = %lf, UserRedElevD = %lf, PitchUpElevD = %lf, PostElevD = %lf", elevator_position_change, user_elevator_position_change, pitch_up_elevator_position_change, new_elevator_position_change);

		return new_elevator_position_change;
	}
	else
	{
		return elevator_position_change;
	}
}

double FlightControlSystem_PitchAttitudeProtection(double const elevator_position_change, const double dt)
{
	const auto clamping_zone = 5; // When we're within 5 degrees of our pitch limitations, reduce authority
	const auto max_pitch_rate = 5; // Max degrees per second when approaching our clamping zone

	if (flight_path_data.pitch > normal_law_protections.max_pitch_angle)
	{
		const auto proposed_elevator_position_change = pitch_angle_controller.Update(normal_law_protections.max_pitch_angle - flight_path_data.pitch, dt);
		const auto new_elevator_position_change = min(elevator_position_change, proposed_elevator_position_change);
		printf(", PITCH_MAX_VIOL: Pitch = %lf, PreElevD = %lf, PostElevD = %lf", flight_path_data.pitch, elevator_position_change, new_elevator_position_change);
		return new_elevator_position_change;
	}
	else if (flight_path_data.pitch < normal_law_protections.min_pitch_angle)
	{
		const auto proposed_elevator_position_change = pitch_angle_controller.Update(normal_law_protections.min_pitch_angle - flight_path_data.pitch, dt);
		const auto new_elevator_position_change = max(elevator_position_change, proposed_elevator_position_change);
		printf(", PITCH_MIN_VIOL: Pitch = %lf, PreElevD = %lf, PostElevD = %lf", flight_path_data.pitch, elevator_position_change, new_elevator_position_change);
		return new_elevator_position_change;
	}
	else if (flight_path_data.pitch + clamping_zone >= normal_law_protections.max_pitch_angle && elevator_position_change >= 0)
	{
		// Add some corrective position change if we are approaching the pitch limit too quickly
		double corrective_elevator_position_change = 0;
		const auto max_pitch_rate_allowed = max_pitch_rate * linear_decay_coefficient(flight_path_data.pitch, normal_law_protections.max_pitch_angle - clamping_zone, normal_law_protections.max_pitch_angle);
		if (flight_path_data.pitch_rate > max_pitch_rate_allowed) {
			corrective_elevator_position_change = pitch_rate_controller.Update(max_pitch_rate_allowed - flight_path_data.pitch_rate, dt);
		}

		// Linearly reduce user nose up authority
		const auto user_elevator_position_change = elevator_position_change * linear_decay_coefficient(flight_path_data.pitch, normal_law_protections.max_pitch_angle - clamping_zone, normal_law_protections.max_pitch_angle);
		const auto new_elevator_position_change = user_elevator_position_change + corrective_elevator_position_change;
		
		printf(", PITCH_MAX: Pitch = %lf, PreElevD = %lf, UserElevD = %lf, CorElevD = %lf, PostElevD = %lf", flight_path_data.pitch, elevator_position_change, user_elevator_position_change, corrective_elevator_position_change, new_elevator_position_change);
		return new_elevator_position_change;
	}
	else if (flight_path_data.pitch - clamping_zone <= normal_law_protections.min_pitch_angle && elevator_position_change <= 0)
	{
		// Add some corrective position change if we are approaching the pitch limit too quickly
		double corrective_elevator_position_change = 0;
		const auto min_pitch_rate_allowed = -max_pitch_rate * linear_decay_coefficient(flight_path_data.pitch, normal_law_protections.min_pitch_angle + clamping_zone, normal_law_protections.min_pitch_angle);
		if (flight_path_data.pitch_rate < min_pitch_rate_allowed) {
			corrective_elevator_position_change = pitch_rate_controller.Update(min_pitch_rate_allowed - flight_path_data.pitch_rate, dt);
		}
		
		const auto user_elevator_position_change = elevator_position_change * linear_decay_coefficient(flight_path_data.pitch, normal_law_protections.min_pitch_angle + clamping_zone, normal_law_protections.min_pitch_angle);
		const auto new_elevator_position_change = user_elevator_position_change + corrective_elevator_position_change;
		printf(", PITCH_MIN: Pitch = %lf, PreElevD = %lf, UserElevD = %lf, CorElevD = %lf, PostElevD = %lf", flight_path_data.pitch, elevator_position_change, user_elevator_position_change, corrective_elevator_position_change, new_elevator_position_change);
		return new_elevator_position_change;
	}
	else
	{
		return elevator_position_change;
	}
}

double FlightControlSystem_FlightModePitchControl(const double t, const double dt)
{
	double elevator_position_change;

	if (normal_law_protections.aoa_demand_active)
	{
		pitch_hold_time = 0;

		// AoA demand mode
		const auto commanded_aoa = FlightControlSystem_GetUserYokeYPosition() >= 0 ?
			// Neutral -> Full Up = AoA proportional range from alpha_prot -> alpha_max
			linear_range(FlightControlSystem_GetUserYokeYPosition(), GetAlphaProtAngle(), GetAlphaMaxAngle())
			// Neutral -> Full Down = AoA proportional range from alpha_prot -> 0 AoA
			: linear_range(FlightControlSystem_GetUserYokeYPosition(), GetAlphaProtAngle(), 0);
		elevator_position_change = aoa_controller.Update(commanded_aoa - flight_path_data.aoa, dt);
		printf("AOA: AoA = %lf, CmdAoA = %lf", flight_path_data.aoa, commanded_aoa);

		// Apply protections
		elevator_position_change = FlightControlSystem_LoadFactorLimitation(elevator_position_change, dt);
		elevator_position_change = FlightControlSystem_PitchAttitudeProtection(elevator_position_change, dt);
	}
	else
	{
		// Load factor demand mode
		if (FlightControlSystem_GetUserYokeXPosition() == 0 && FlightControlSystem_GetUserYokeYPosition() == 0)
		{
			// Neutral x and y = Hold FPA
			if (pitch_hold_time < 5)
			{
				// Hold the current pitch for 5 seconds to allow VFPA to stabilize
				elevator_position_change = pitch_rate_controller.Update(0 - flight_path_data.pitch_rate, dt);
				printf("HOLD_PITCH: Pitch = %lf, PitchRate = %lf", flight_path_data.pitch, flight_path_data.pitch_rate);
				held_vertical_fpa = flight_path_data.vertical_fpa;
				pitch_hold_time += dt;
			}
			else
			{
				// Hold the VFPA
				elevator_position_change = vertical_fpa_controller.Update(held_vertical_fpa - flight_path_data.vertical_fpa, dt);
				printf("HOLD_VFPA: Pitch = %lf, PitchRate = %lf, VFPA = %lf, DesVFPA = %lf, VFPAErr = %lf, VFPARate = %lf", flight_path_data.pitch, flight_path_data.pitch_rate, flight_path_data.vertical_fpa, held_vertical_fpa, held_vertical_fpa - flight_path_data.vertical_fpa, flight_path_data.vertical_fpa_rate);
			}
		}
		else
		{
			pitch_hold_time = 0;
			if (FlightControlSystem_GetUserYokeYPosition() == 0)
			{
				if (fabs(flight_path_data.roll) > normal_law_protections.nominal_bank_angle)
				{
					// Neutral y, but we're rolling and bank angle is greater than our nominal bank angle = Drop pitch to 1G LF
					elevator_position_change = gforce_controller.Update(1 - flight_path_data.gforce, dt);
					printf("ROLL_1G: Pitch = %lf, PR = %lf, CurrLF = %lf", flight_path_data.pitch, flight_path_data.pitch_rate, flight_path_data.gforce);
				}
				else
				{
					// Neutral y, but we're rolling and bank angle is less than our nominal bank angle = Hold pitch
					elevator_position_change = pitch_rate_controller.Update(0 - flight_path_data.pitch_rate, dt);
					printf("HOLD_PITCH: Pitch = %lf, PitchRate = %lf", flight_path_data.pitch, flight_path_data.pitch_rate);
				}
			}
			else
			{
				// Determine the normal load factor for our bank angle
				const auto normal_load_factor = 1 / cos(radians(flight_path_data.roll));

				// Determine the user's requested load factor
				const auto requested_load_factor = FlightControlSystem_GetUserYokeYPosition() >= 0 ?
					linear_range(FlightControlSystem_GetUserYokeYPosition(), normal_load_factor, normal_law_protections.max_load_factor)
					: linear_range(-FlightControlSystem_GetUserYokeYPosition(), normal_load_factor, normal_law_protections.min_load_factor);

				elevator_position_change = gforce_controller.Update(requested_load_factor - flight_path_data.gforce, dt);
				printf("CMD_LF: LF = %lf, DesLF = %lf", flight_path_data.gforce, requested_load_factor);
			}
		}

		// Apply protections
		elevator_position_change = FlightControlSystem_HighSpeedProtection(elevator_position_change, dt);
		elevator_position_change = FlightControlSystem_LoadFactorLimitation(elevator_position_change, dt);
		elevator_position_change = FlightControlSystem_PitchAttitudeProtection(elevator_position_change, dt);
	}
	return elevator_position_change;
}

double FlightControlSystem_FlareModePitchControl(const double t, const double dt)
{
	// From the FCOM
	// "The system memorizes the attitude at 50 feet, and that attitude becomes the initial
	// reference for pitch attitude control."
	// We saved that variable as 'saved_flare_pitch_attitude'.
	// Honestly, I can't see why it is saved though.

	// From the FCOM
	// "As the aircraft descends through 30 feet, the system begins to reduce the pitch attitude,
	// reducing it to 2 degrees nose down over a period of 8 seconds. This means that it takes
	// gentle nose-up action by the pilot to flare the aircraft."

	// Let's make flare mode just a pitch rate mode
	auto commanded_pitch_rate = 5 * FlightControlSystem_GetUserYokeYPosition(); // 5 degrees/sec at maximum deflection
	if (flight_path_data.radio_height <= 30)
	{
		commanded_pitch_rate -= 2.0 / 8.0; // 2 degrees nose down per 8 seconds
	}

	return pitch_rate_controller.Update(commanded_pitch_rate - flight_path_data.pitch_rate, dt);
}


void FlightControlSystem_ManagePitchControl(const double t, const double dt)
{
	// TODO: Handle other laws
	FlightControlSystem_CalculateNormalLawSettings(t, dt);

	if (pitch_control_mode == FLIGHT_MODE || pitch_control_mode == FLARE_MODE)
	{
		const auto flight_mode_effect = pitch_control_mode_flight_effect * FlightControlSystem_FlightModePitchControl(t, dt);
		const auto flare_mode_effect = pitch_control_mode_flare_effect * FlightControlSystem_FlareModePitchControl(t, dt);
		const auto elevator_position_change = flight_mode_effect + flare_mode_effect;
		control_surfaces.elevator += elevator_position_change;
		control_surfaces.elevator = clamp(control_surfaces.elevator, -1, 1);
		printf(", ElevD = %lf, Elev = %lf\n", elevator_position_change, control_surfaces.elevator);
	}
	else
	{
		control_surfaces.elevator = user_input.yoke_y;
	}
}

double roll_angle = 0; // The desired bank angle
static PIDController roll_controller(-1, 1, 0.10, 0, 0.02);
void FlightControlSystem_ManageRollControl(const double t, const double dt)
{
	// TODO: Handle other laws
	FlightControlSystem_CalculateNormalLawSettings(t, dt);
	if (pitch_control_mode == FLIGHT_MODE)
	{
		if (flight_path_data.aoa >= GetAlphaProtAngle())
		{
			// High AoA weights - justification: ailerons are less effective at high AoA and so it's easier to overshoot
			roll_controller.Modify(-0.25, 0.25, 0.05, 0, 0.01);
		}
		else
		{
			// Normal AoA weights
			roll_controller.Modify(-1, 1, 0.10, 0, 0.02);
		}
		
		if (FlightControlSystem_GetUserYokeXPosition() == 0)
		{
			// If we are banked beyond the nominal bank angle, roll back to the nominal bank angle
			if (fabs(roll_angle) > normal_law_protections.nominal_bank_angle)
			{
				// Roll opposite at a rate of up to 5 degrees/second
				roll_angle += 5 * -sign(roll_angle) * dt;
				if (fabs(roll_angle) < normal_law_protections.nominal_bank_angle)
				{
					roll_angle = sign(roll_angle) * normal_law_protections.nominal_bank_angle;
				}
			}
			// We should be holding the specified roll angle
		}
		else
		{
			// We should be responsive to the user's roll request
			roll_angle += 15 * FlightControlSystem_GetUserYokeXPosition() * dt; // 15 degrees/sec at maximum deflection
			roll_angle = clamp(roll_angle, -normal_law_protections.maximum_bank_angle, normal_law_protections.maximum_bank_angle);
		}

		control_surfaces.aileron = roll_controller.Update(roll_angle - flight_path_data.roll, dt);
		//printf("Roll: CurRoll = %lf, DesRoll = %lf, Error = %lf, Rate = %lf, Ailerons = %lf\n", flight_path_data.roll, roll_angle, roll_angle - flight_path_data.roll, flight_path_data.roll_rate, control_surfaces.aileron);
	}
	else
	{
		control_surfaces.aileron = user_input.yoke_x;
	}
}

bool HandleControlSurfaces(FsContext ctx, const int service_id, void* pData)
{
	switch (service_id)
	{
	case PANEL_SERVICE_PRE_INSTALL:
	{
		return true;
	}
	break;
	case PANEL_SERVICE_POST_INSTALL:
	{
		SimConnect_AddToDataDefinition(hSimConnect, CONTROL_SURFACES_DEFINITION, "ELEVATOR POSITION", "Position");
		SimConnect_AddToDataDefinition(hSimConnect, CONTROL_SURFACES_DEFINITION, "AILERON POSITION", "Position");
		SimConnect_AddToDataDefinition(hSimConnect, CONTROL_SURFACES_DEFINITION, "RUDDER POSITION", "Position");
		return true;
	}
	break;
	case PANEL_SERVICE_PRE_DRAW:
	{
		// Sent before the gauge is drawn. The pData parameter points to a sGaugeDrawData structure:
        // - The t member gives the absolute simulation time.
        // - The dt member gives the time elapsed since last frame.
		auto* p_draw_data = static_cast<sGaugeDrawData*>(pData);
		auto t = p_draw_data->t;
		auto dt = p_draw_data->dt;
		if (!flight_path_data.autopilot_enabled) {
			FlightControlSystem_ManagePitchControl(t, dt);
			FlightControlSystem_ManageRollControl(t, dt);
			control_surfaces.rudder = user_input.rudder;
		}
		else
		{
			control_surfaces.elevator = user_input.yoke_y;
			control_surfaces.aileron = user_input.yoke_x;
			control_surfaces.rudder = user_input.rudder;
		}
		SimConnect_SetDataOnSimObject(hSimConnect, CONTROL_SURFACES_DEFINITION, SIMCONNECT_OBJECT_ID_USER, 0, 0, sizeof(control_surfaces), &control_surfaces);
		return true;
	}
	break;
	case PANEL_SERVICE_PRE_KILL:
	{
		return true;
	}
	break;
	default: break;
	}
	return false;
}


 /*
  * ============ *
  * Finalization *
  * ============ *
  */
extern "C"
{
	/*
	 * Handles all system states
	 */
	MSFS_CALLBACK bool A32NX_gauge_callback(FsContext ctx, const int service_id, void* pData)
	{
		auto ret = true;
		ret &= HandleSimVars(ctx, service_id, pData);
		ret &= HandleSimConnect(ctx, service_id, pData);
		ret &= HandleFlightPathDataUpdate(ctx, service_id, pData);
		ret &= HandleFlightControlLaws(ctx, service_id, pData);
		ret &= HandlePitchControlMode(ctx, service_id, pData);
		ret &= HandleInputCapture(ctx, service_id, pData);
		ret &= HandleControlSurfaces(ctx, service_id, pData);
		return ret;
	}
}
