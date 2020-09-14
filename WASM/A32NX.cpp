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
constexpr double clamp(double value, double min, double max)
{
	return (value < min) ? min : (value > max) ? max : value;
}
constexpr double radians(double value)
{
	return value * (M_PI / 180);
}
constexpr double degrees(double value)
{
	return value * (180 / M_PI);
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

		// Integral term (trapezoidal area: assuming error moved linearly from last error to current error over dt)
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
private:
	double output_min, output_max;
	double Kp, Kd, Ki;
	double integral;
	double last_error, last_output;
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
 * ===================== *
 * Flight Path Variables *
 * ===================== *
 */
struct FLIGHT_PATH_DATA
{
	double airspeed_indicated = 0; // knots
	double airspeed_mach = 0; // mach
	double alpha_prot = DBL_MAX; // degrees
	double alpha_floor = DBL_MAX; // degrees
	double alpha_max = DBL_MAX; // degrees
	double aoa = 0; // degrees
	double gforce = 1; // gforce
	double gforce_rate = 0; // gforce rate
	double flaps_pos = 0; // number (0-4)
	double mmo = DBL_MAX; // mach
	double pitch = 0; // degrees
	double pitch_rate = 0; // degrees per second
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
	// These values are hardcoded in the FCOM
	if (flight_path_data.flaps_pos == 1.0 || flight_path_data.flaps_pos == 2.0)
	{
		return 15;
	}
	else if (flight_path_data.flaps_pos == 3.0)
	{
		return 14;
	}
	else if (flight_path_data.flaps_pos == 4.0)
	{
		return 13;
	}
	else
	{
		return 9.5;
	}
}

double GetAlphaProtAngle()
{
	// This ratio was estimated using the graph in the FCOM
	const auto ratio_with_alpha_floor = 19.0 / 21.0;
	return ratio_with_alpha_floor * GetAlphaFloorAngle();
}

double GetAlphaMaxAngle()
{
	// This ratio was estimated using the graph in the FCOM
	const auto ratio_with_alpha_floor = 7 / 6;
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
const char* PITCH_CONTROL_MODE_VAR_NAME = "PITCH CONTROL MODE";

void PitchControlMode_BlendEffect(double * blend_in, double * blend_out, const double increment)
{
	*blend_in = clamp(*blend_in + increment, 0, 1);
	*blend_out = clamp(*blend_out - increment, 0, 1);
}

double ground_to_flight_condition_start_time = 0;
void PitchControlMode_HandleGroundTransitions(const double dt)
{
	if (pitch_control_mode == GROUND_MODE)
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
}

void PitchControlMode_HandleFlightTransitions(const double dt)
{
	if (pitch_control_mode == FLIGHT_MODE)
	{
		if (InNormalLaw())
		{
			// Handle flight to flare transition
			auto radio_altimeter = aircraft_varget(sim_vars.radio_height, sim_vars.feet_units, 0);
			if (radio_altimeter < 50)
			{
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
	AXIS_ELEVATOR_SET_EVENT,
	// Aileron Group
	AXIS_AILERONS_SET_EVENT,
	CENTER_AILERONS_RUDDER_EVENT,
	// Rudder group
	AXIS_RUDDER_SET_EVENT,
	RUDDER_SET_EVENT,
	RUDDER_CENTER_EVENT
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
		case AXIS_ELEVATOR_SET_EVENT:
			user_input.yoke_y = 0 - (static_cast<long>(evt->dwData) / 16384.0); // scale from [-16384,16384] to [-1,1] and reverse the sign
			break;
		case AXIS_AILERONS_SET_EVENT:
			user_input.yoke_x = 0 - (static_cast<long>(evt->dwData) / 16384.0); // scale from [-16384,16384] to [-1,1] and reverse the sign
			break;
		case CENTER_AILERONS_RUDDER_EVENT:
			user_input.yoke_x = 0;
			user_input.rudder = 0;
			break;
		case AXIS_RUDDER_SET_EVENT:
			user_input.rudder = 0 - (static_cast<long>(evt->dwData) / 16384.0); // scale from [-16384,16384] to [-1,1] and reverse the sign
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
		SimConnect_MapClientEventToSimEvent(hSimConnect, AXIS_ELEVATOR_SET_EVENT, "AXIS_ELEVATOR_SET");
		SimConnect_AddClientEventToNotificationGroup(hSimConnect, ELEVATOR_GROUP, AXIS_ELEVATOR_SET_EVENT, TRUE);

		// Aileron group
		SimConnect_MapClientEventToSimEvent(hSimConnect, AXIS_AILERONS_SET_EVENT, "AXIS_AILERONS_SET");
		SimConnect_MapClientEventToSimEvent(hSimConnect, CENTER_AILERONS_RUDDER_EVENT, "CENTER_AILER_RUDDER");
		SimConnect_AddClientEventToNotificationGroup(hSimConnect, AILERON_GROUP, AXIS_AILERONS_SET_EVENT, TRUE);
		SimConnect_AddClientEventToNotificationGroup(hSimConnect, AILERON_GROUP, CENTER_AILERONS_RUDDER_EVENT, TRUE);

		// Rudder group
		SimConnect_MapClientEventToSimEvent(hSimConnect, AXIS_RUDDER_SET_EVENT, "AXIS_RUDDER_SET");
		SimConnect_MapClientEventToSimEvent(hSimConnect, RUDDER_SET_EVENT, "RUDDER_SET");
		SimConnect_MapClientEventToSimEvent(hSimConnect, RUDDER_CENTER_EVENT, "RUDDER_CENTER");
		SimConnect_AddClientEventToNotificationGroup(hSimConnect, RUDDER_GROUP, AXIS_RUDDER_SET_EVENT, TRUE);
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
	double elevator; // -1 is full down, and +1 is full up
	double aileron; // -1 is full left, and +1 is full right
	double rudder; // -1 is full left, and +1 is full right
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

double linearly_reduce_authority(const double zone_position, const double min_zone, const double max_zone)
{
	// At min zone, the effectiveness is maximal
	// At max zone, the effectiveness is minimal
	if (min_zone < max_zone)
	{	
		// We are working in the positive direction
		if (zone_position <= max_zone)
		{
			return 1.0; // Full effectiveness
		}
		else if (zone_position >= max_zone)
		{
			return 0.0; // If we've exceeded the max zone, the command effectiveness must be 0.
		}
		else
		{
			return 1.0 - ((zone_position - min_zone) / (max_zone - min_zone));
		}	
	}
	else
	{
		// We are working in the negative direction
		if (zone_position >= min_zone)
		{
			return 1.0; // Full effectiveness
		}
		else if (zone_position <= max_zone)
		{
			return 0.0; // Minimal effectiveness
		}
		else
		{
			return 1.0 - ((zone_position - min_zone) / (max_zone - min_zone));
		}
	}
}


struct AOA_PROTECTIONS
{
	bool in_aoa_mode = false;
	double last_update = 0;
	double aoa_deactivate_time = 0;
	double requested_aoa = 0;
} aoa_prot;

bool InAngleOfAttackProtectionMode(const double t, const double dt)
{
	if (aoa_prot.last_update != t)
	{
		// Update protection info only if we haven't already calculated it this cycle
		if (aoa_prot.in_aoa_mode)
		{
			// Should we leave AoA mode?
			// Exit condition 1: Sidestick must be pushed more than 8 degrees forward (assuming this is ~50% down)
			if (FlightControlSystem_GetUserYokeYPosition() == -0.5)
			{
				aoa_prot.in_aoa_mode = false;
				aoa_prot.aoa_deactivate_time = 0;
			}
			// Exit condition 2: Sidestick must be pushed more than 0.5 degrees forward for at least 0.5 seconds when alpha < alpha_max
			else if (FlightControlSystem_GetUserYokeYPosition() < 0 && flight_path_data.aoa < GetAlphaMaxAngle())
			{
				aoa_prot.aoa_deactivate_time += dt;
				if (aoa_prot.aoa_deactivate_time >= 0.5)
				{
					aoa_prot.in_aoa_mode = false;
					aoa_prot.aoa_deactivate_time = 0;
				}
			}
		}
		else
		{
			// Should we enter AoA mode?
			// Enter condition 1: Sidestick must not be pushed down, and AoA is greater than alpha prot
			if (FlightControlSystem_GetUserYokeYPosition() >= 0 && flight_path_data.aoa > GetAlphaProtAngle())
			{
				aoa_prot.in_aoa_mode = true;
				aoa_prot.requested_aoa = clamp(flight_path_data.aoa, flight_path_data.aoa, GetAlphaMaxAngle());
			}
			// Enter condition 2: We are at or above alpha max
			else if (flight_path_data.aoa >= GetAlphaMaxAngle())
			{
				aoa_prot.in_aoa_mode = true;
				aoa_prot.requested_aoa = clamp(flight_path_data.aoa, flight_path_data.aoa, GetAlphaMaxAngle());
			}
		}
	}
	return aoa_prot.in_aoa_mode;
}

double FlightControlSystem_ApplyLoadFactorLimitation(double commanded_pitch_rate)
{
	const auto clamping_zone = 0.25;
	const auto maximum_load_factor = 2.5; // TODO: Handle other configurations
	const auto minimum_load_factor = -1.0; // TODO: Handle other configurations

	if (flight_path_data.gforce > maximum_load_factor && commanded_pitch_rate >= 0)
	{
		printf(",LF_PROT_HIGH_VIOL: LF = %lf", flight_path_data.gforce);
		commanded_pitch_rate = -1; // Reduce pitch by 1 degree/second to move back within the flight envelope
	}
	else if (flight_path_data.gforce < minimum_load_factor && commanded_pitch_rate <= 0)
	{
		printf(",LF_PROT_LOW_VIOL: LF = %lf", flight_path_data.gforce);
		commanded_pitch_rate = 1; // Increase pitch by 1 degree/second to move back within the flight envelope
	}
	else if (flight_path_data.gforce >= (maximum_load_factor - clamping_zone) && commanded_pitch_rate > 0)
	{
		printf(",LF_PROT_HIGH: LF = %lf", flight_path_data.gforce);
		commanded_pitch_rate *= linearly_reduce_authority(flight_path_data.gforce, maximum_load_factor - clamping_zone, maximum_load_factor);
	}
	else if (flight_path_data.gforce < (minimum_load_factor + clamping_zone) && commanded_pitch_rate < 0)
	{
		printf(",LF_PROT_LOW: LF = %lf", flight_path_data.gforce);
		commanded_pitch_rate *= linearly_reduce_authority(flight_path_data.gforce, minimum_load_factor + clamping_zone, minimum_load_factor);
	}
	return commanded_pitch_rate;
}

double FlightControlSystem_ApplyPitchAttitudeProtection(double commanded_pitch_rate)
{
	const auto clamping_zone = 5;
	const auto maximum_pitch = 30; // TODO: Handle other configurations
	const auto minimum_pitch = -15; // TODO: Handle other configurations
	
	if (flight_path_data.pitch > maximum_pitch && commanded_pitch_rate >= 0)
	{
		// Reduce pitch by up to 1 degree/second to move back within the flight envelope
		printf(",TOO_HIGH_VIOL");
		commanded_pitch_rate = -1 * linearly_reduce_authority(flight_path_data.pitch, maximum_pitch + 1, maximum_pitch);
	}
	else if (flight_path_data.pitch < minimum_pitch && commanded_pitch_rate <= 0)
	{
		// Increase pitch by up to 1 degree/second to move back within the flight envelope
		printf(",TOO_LOW_VIOL");
		commanded_pitch_rate = 1 * linearly_reduce_authority(flight_path_data.pitch, minimum_pitch - 1, minimum_pitch);
	}
	else if (flight_path_data.pitch >= (maximum_pitch - clamping_zone) && commanded_pitch_rate > 0)
	{
		printf(",TOO_HIGH");
		commanded_pitch_rate *= linearly_reduce_authority(flight_path_data.pitch, maximum_pitch - clamping_zone, maximum_pitch);
	}
	else if (flight_path_data.pitch <= (minimum_pitch + clamping_zone) && commanded_pitch_rate < 0)
	{
		printf(",TOO_HIGH");
		commanded_pitch_rate *= linearly_reduce_authority(flight_path_data.pitch, minimum_pitch + clamping_zone, minimum_pitch);
	}
	return commanded_pitch_rate;
}

double FlightControlSystem_ApplyHighSpeedProtection(double commanded_pitch_rate)
{
	if (flight_path_data.airspeed_indicated > flight_path_data.vmo || flight_path_data.airspeed_mach > flight_path_data.mmo)
	{
		if (FlightControlSystem_GetUserYokeYPosition() < 0)
		{
			// Linearly reduce elevator down authority as speed increases
			auto vmo_effectiveness = linearly_reduce_authority(flight_path_data.airspeed_indicated, flight_path_data.vmo, flight_path_data.vmo + 4);
			auto mmo_effectiveness = linearly_reduce_authority(flight_path_data.airspeed_mach, flight_path_data.mmo, flight_path_data.mmo + 0.006);
			auto effectiveness = min(vmo_effectiveness, mmo_effectiveness);
			commanded_pitch_rate *= effectiveness;
		}
		// Add a negative pitch rate to encourage the nose to come down
		commanded_pitch_rate -= 1; // TODO: Tuning?
	}
	return commanded_pitch_rate;
}

double locked_pitch_time = 0;
static PIDController aoa_controller(-10, 10, 1, 0, 0);
static PIDController gforce_rate_controller(-10, 10, 1, 1, 0.05);
static PIDController hold_vertical_fpa_rate_controller(-2, 2, 16, 16, 1);
static PIDController pitch_rate_controller(-1, 1, 0.32, 0.16, 0.05);
void FlightControlSystem_ManagePitchControl(const double t, const double dt)
{
	// TODO: Handle other laws
	if (pitch_control_mode == FLIGHT_MODE)
	{
		double commanded_pitch_rate;

		if (InAngleOfAttackProtectionMode(t, dt))
		{
			// AoA demand mode
			// Kicks in when angle of attack becomes grater than alpha_prot
			// - Stick commands AoA between alpha_prot to alpha_max
			// - alpha_prot < alpha_floor < alpha_max < alpha_cl_max = 1G stall
			// To deactivate AoA protection:
			// - Sidestick must be pushed more than 8 degrees forward, or;
			// - Sidestick must be pushed more than 0.5 degrees forward for at least 0.5 seconds when alpha < alpha_max
			const auto max_aoa_rate = 5; // Maximum sidestick effectiveness in changing AoA. TODO: Tuning?
			const auto aoa_change = max_aoa_rate * FlightControlSystem_GetUserYokeYPosition() * dt;
			if (aoa_change == 0)
			{
				// When sidestick is released, the angle of attack returns to alpha prot
				aoa_prot.requested_aoa = GetAlphaProtAngle();
			}
			else
			{
				// Otherwise, we are just commanding an AoA change.
				aoa_prot.requested_aoa += flight_path_data.aoa + aoa_change;
			}
			aoa_prot.requested_aoa = clamp(aoa_prot.requested_aoa, aoa_prot.requested_aoa, GetAlphaMaxAngle());
			printf("AOA: AA = %lf, CmdAA = %lf", flight_path_data.aoa, aoa_prot.requested_aoa);
			commanded_pitch_rate = aoa_controller.Update(aoa_prot.requested_aoa - flight_path_data.aoa, dt);
			commanded_pitch_rate = FlightControlSystem_ApplyLoadFactorLimitation(commanded_pitch_rate);
		}
		else
		{
			// Load factor demand
			if (FlightControlSystem_GetUserYokeYPosition() == 0 && FlightControlSystem_GetUserYokeXPosition() == 0)
			{
				// Neutral x and y = Hold FPA
				if (locked_pitch_time < 5) // TODO: Tune
				{
					// Lock pitch for 5 seconds to allow the FPA to stabilize
					locked_pitch_time += dt;
					commanded_pitch_rate = 0;
					printf("HOLD_PITCH");
					hold_vertical_fpa_rate_controller.Reset();
				}
				else
				{
					// Hold FPA
					commanded_pitch_rate = hold_vertical_fpa_rate_controller.Update(0 - flight_path_data.vertical_fpa_rate, dt);
					printf("HOLD_VFPA: VFPA = %lf, VFPARate = %lf", flight_path_data.vertical_fpa, flight_path_data.vertical_fpa_rate);
				}
			}
			else if (FlightControlSystem_GetUserYokeYPosition() == 0)
			{
				if (fabs(flight_path_data.roll) > 33)
				{
					// Neutral y, but we're rolling and bank angle is > 33 degrees = Drop pitch to 1G LF
					locked_pitch_time = 0;
					commanded_pitch_rate = gforce_rate_controller.Update(1 - flight_path_data.gforce, dt);
					printf("ROLL_1G: CurLF = %lf", flight_path_data.gforce);
				}
				else
				{
					// Neutral y, but we're rolling and bank angle is <= 33 degrees = Hold pitch
					locked_pitch_time = 0;
					commanded_pitch_rate = 0;
					printf("HOLD_PITCH");
				}
			}
			else
			{
				// Apply the LF demand
				locked_pitch_time = 0;

				// Determine the normal load factor
				auto normal_load_factor = 1 / cos(radians(flight_path_data.roll));

				// Determine the user's load factor
				auto requested_load_factor = 2 * FlightControlSystem_GetUserYokeYPosition() + normal_load_factor; // Linear relationship between request and actual value
				requested_load_factor = clamp(requested_load_factor, -1, 2.5); // TODO: Adjust bounds for different configurations
				printf("CMD_LF: LF = %lf, ReqLF = %lf", flight_path_data.gforce, requested_load_factor);

				// Command LF
				commanded_pitch_rate = gforce_rate_controller.Update(requested_load_factor - flight_path_data.gforce, dt);
			}

			// Apply protections
			commanded_pitch_rate = FlightControlSystem_ApplyHighSpeedProtection(commanded_pitch_rate);
			commanded_pitch_rate = FlightControlSystem_ApplyLoadFactorLimitation(commanded_pitch_rate);
			commanded_pitch_rate = FlightControlSystem_ApplyPitchAttitudeProtection(commanded_pitch_rate);
		}
		
		// Apply changes to the elevator
		control_surfaces.elevator = pitch_rate_controller.Update(commanded_pitch_rate - flight_path_data.pitch_rate, dt);
		printf("Pitch = %lf, PR = %lf, CmdPR = %lf, Elev = %lf\n", flight_path_data.pitch, flight_path_data.pitch_rate, commanded_pitch_rate, control_surfaces.elevator);
	}
	else
	{
		control_surfaces.elevator = user_input.yoke_y;
	}
}

double FlightControlSystem_ApplyRollAngleLimitation(double commanded_roll_rate, const double maximum_bank_angle)
{
	const auto clamping_zone = 10;

	if (fabs(flight_path_data.roll) > maximum_bank_angle)
	{
		// Roll opposite at a rate of up to 5 degrees/second // TODO: Tune?
		commanded_roll_rate = 5 * -sign(flight_path_data.roll) * linearly_reduce_authority(fabs(flight_path_data.roll), maximum_bank_angle + 5, maximum_bank_angle);
	}
	else if (fabs(flight_path_data.roll) >= (maximum_bank_angle - clamping_zone) && sign(commanded_roll_rate) == sign(flight_path_data.roll))
	{
		// Linearly reduce authority as we approach the maximum bank angle
		commanded_roll_rate *= linearly_reduce_authority(fabs(flight_path_data.roll), maximum_bank_angle - clamping_zone, maximum_bank_angle);
	}
	return commanded_roll_rate;
}

bool roll_stable = false; // Indicates if we are in a stable (within nominal bank angle) position
static PIDController roll_rate_controller(-1, 1, 0, 0, 0);
void FlightControlSystem_ManageRollControl(const double dt)
{
	// TODO: Handle other laws
	if (pitch_control_mode == FLIGHT_MODE)
	{
		auto maximum_bank_angle = 67; // Maximum bank angle // TODO: Change this when other kinds of protections are active
		auto nominal_bank_angle = 33; // Maximum bank angle allowed for turns // TODO: Change this when other kinds of protections are active
		
		const auto overspeeding = flight_path_data.airspeed_indicated > flight_path_data.vmo || flight_path_data.airspeed_mach > flight_path_data.mmo;
		if (overspeeding)
		{
			roll_stable = false;
			maximum_bank_angle = 45;
			nominal_bank_angle = 0;
		}

		auto commanded_roll_rate = 15 * FlightControlSystem_GetUserYokeXPosition(); // degrees per second
		if (commanded_roll_rate == 0)
		{
			// If we are banked beyond the nominal bank angle, roll back to the nominal bank angle
			if (!roll_stable && fabs(flight_path_data.roll) > nominal_bank_angle)
			{
				// Roll opposite at up to 5 degrees per second // TODO: Tuning?
				roll_rate_controller.Modify(-0.25, 0.25, 0.64, 0, 0);
				commanded_roll_rate = 5 * -sign(flight_path_data.roll) * linearly_reduce_authority(fabs(flight_path_data.roll), nominal_bank_angle + 5, nominal_bank_angle); 
			}
			else
			{
				// We should be holding the specified angle
				roll_rate_controller.Modify(-0.25, 0.25, 0.32, 0.32, 0);
				roll_stable = true;
			}
		}
		else
		{
			// We should be responsive to the user's roll request
			roll_stable = false;
			roll_rate_controller.Modify(-1, 1, 0.64, 0, 0);

			commanded_roll_rate = FlightControlSystem_ApplyRollAngleLimitation(commanded_roll_rate, maximum_bank_angle);
		}
		control_surfaces.aileron = roll_rate_controller.Update(commanded_roll_rate - flight_path_data.roll_rate, dt);
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
		FlightControlSystem_ManagePitchControl(t, dt);
		FlightControlSystem_ManageRollControl(dt);
		control_surfaces.rudder = user_input.rudder;
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
