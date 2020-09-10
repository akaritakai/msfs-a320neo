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
	ENUM percent_units; // Percent
	ENUM radians_units; // Radians
	// Variables
	ENUM airspeed_true; // AIRSPEED TRUE
	ENUM elevator_deflection; // ELEVATOR DEFLECTION
	ENUM elevator_deflection_pct; // ELEVATOR DEFLECTION PCT
	ENUM gforce; // G FORCE
	ENUM plane_bank_degrees; // PLANE BANK DEGREES
	ENUM plane_pitch_degrees; // PLANE PITCH DEGREES
	ENUM radio_height; // RADIO HEIGHT
	ENUM rotation_velocity_body_x; // ROTATION VELOCITY BODY X
	ENUM rotation_velocity_body_y; // ROTATION VELOCITY BODY Y
	ENUM sim_on_ground; // SIM ON GROUND
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
		sim_vars.percent_units = get_units_enum("Percent");
		sim_vars.radians_units = get_units_enum("Radians");
		// Variables
		sim_vars.airspeed_true = get_aircraft_var_enum("AIRSPEED TRUE");
		sim_vars.elevator_deflection = get_aircraft_var_enum("ELEVATOR DEFLECTION");
		sim_vars.elevator_deflection_pct = get_aircraft_var_enum("ELEVATOR DEFLECTION PCT");
		sim_vars.gforce = get_aircraft_var_enum("G FORCE");
		sim_vars.plane_bank_degrees = get_aircraft_var_enum("PLANE BANK DEGREES");
		sim_vars.plane_pitch_degrees = get_aircraft_var_enum("PLANE PITCH DEGREES");
		sim_vars.radio_height = get_aircraft_var_enum("RADIO HEIGHT");
		sim_vars.rotation_velocity_body_x = get_aircraft_var_enum("ROTATION VELOCITY BODY X");
		sim_vars.rotation_velocity_body_y = get_aircraft_var_enum("ROTATION VELOCITY BODY Y");
		sim_vars.sim_on_ground = get_aircraft_var_enum("SIM ON GROUND");
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
	double gforce; // gforce
	double gforce_rate; // gforce rate
	double pitch; // degrees
	double pitch_rate; // degrees per second
	double roll; // degrees (rolled right == positive numbers, rolled left == negative numbers)
	double roll_rate; // degrees per second
	double vertical_fpa; // degrees
	double vertical_fpa_rate; // degrees per second
	double vertical_speed; // feet per second
	double vertical_speed_rate; // feet per second per second
} flight_path_data;

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

		// Update gforce info
		auto last_gforce = flight_path_data.gforce;
		flight_path_data.gforce = aircraft_varget(sim_vars.gforce, sim_vars.gforce_units, 0);
		flight_path_data.gforce_rate = (flight_path_data.gforce - last_gforce) / dt;
			
		// Update pitch info
		flight_path_data.pitch = aircraft_varget(sim_vars.plane_pitch_degrees, sim_vars.degrees_units, 0);
		flight_path_data.pitch_rate = aircraft_varget(sim_vars.rotation_velocity_body_y, sim_vars.degrees_per_second_units, 0);

		// Update roll info
		double last_roll = flight_path_data.roll;
		flight_path_data.roll = -aircraft_varget(sim_vars.plane_bank_degrees, sim_vars.degrees_units, 0);
		flight_path_data.roll_rate = (flight_path_data.roll - last_roll) / dt;

		// Update vertical fpa info
		const auto last_vertical_fpa = flight_path_data.vertical_fpa;
		// Vertical FPA = arctan(vertical speed / true airspeed)
		const auto vertical_speed = aircraft_varget(sim_vars.vertical_speed, sim_vars.feet_per_second_units, 0);
		const auto true_airspeed = aircraft_varget(sim_vars.airspeed_true, sim_vars.knots_units, 0);
		const auto knots_to_feet_per_minute_ratio = (60.0 / 1.0) * (1.0 / 6076.0); // (60 min / 1 h) * (1 nm / 6076 ft)
		flight_path_data.vertical_fpa = atan(vertical_speed * (1 / true_airspeed) * knots_to_feet_per_minute_ratio) * (180 / M_PI);
		flight_path_data.vertical_fpa_rate = (flight_path_data.vertical_fpa - last_vertical_fpa) / dt;

		// Update vertical speed info
		const auto last_vertical_speed = flight_path_data.vertical_speed;
		flight_path_data.vertical_speed = aircraft_varget(sim_vars.vertical_speed, sim_vars.feet_per_second_units, 0);
		flight_path_data.vertical_speed_rate = (flight_path_data.vertical_speed - last_vertical_speed) / dt;

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
			auto radio_altimeter = aircraft_varget(sim_vars.radio_height, sim_vars.feet_units, 0);
			auto on_ground = aircraft_varget(sim_vars.sim_on_ground, sim_vars.bool_units, 0) != 0;
			auto pitch_attitude = aircraft_varget(sim_vars.plane_pitch_degrees, sim_vars.degrees_units, 0);
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
	auto null_zone_error = 0.05 * 2; // 5% of the stick movement is null (2 units of total travel distance over [-1,1])
	return fabs(user_input.yoke_y) < null_zone_error ? 0 : user_input.yoke_y;
}

double FlightControlSystem_GetUserYokeXPosition()
{
	// Get the user stick position with a null zone
	auto null_zone_error = 0.05 * 2; // 5% of the stick movement is null (2 units of total travel distance over [-1,1])
	return fabs(user_input.yoke_x) < null_zone_error ? 0 : user_input.yoke_x;
}

static PIDController gforce_rate_controller(-1, 1, 0.64, 0.32, 0.005);
void FlightControlSystem_ManagePitchControl(const double dt)
{
	if (FlightControlSystem_GetUserYokeYPosition() == 0)
	{
		gforce_rate_controller.Modify(-1, 1, 0.32, 0.16, 0);
	}
	else
	{
		gforce_rate_controller.Modify(-1, 1, 0.64, 0, 0.32);
	}
	
	auto normal_load_factor = 1 / cos(radians(flight_path_data.roll));

	// Determine the user's load factor
	auto requested_load_factor = 2 * FlightControlSystem_GetUserYokeYPosition() + normal_load_factor; // Linear relationship between request and actual value
	requested_load_factor = clamp(requested_load_factor, -1, 2.5);

	auto current_load_factor = flight_path_data.gforce;
	auto error = requested_load_factor - current_load_factor;
	control_surfaces.elevator = gforce_rate_controller.Update(error, dt);
	printf("GFORCE: CurrLF = %lf, NormLF = %lf, ReqLF = %lf, Error = %lf, Elevator = %lf\n", current_load_factor, normal_load_factor, requested_load_factor, error, control_surfaces.elevator);
}

static PIDController roll_rate_controller(-1, 1, 0, 0, 0);
void FlightControlSystem_ManageRollControl(const double dt)
{
	// TODO: Handle other laws
	
	const auto maximum_bank_angle = 67; // Maximum bank angle // TODO: Change this when other kinds of protections are active
	const auto clamping_bank_angle = maximum_bank_angle - 10; // Bank angle at which to clamp roll response // TODO: Tuning?
	const auto nominal_bank_angle = 33; // Maximum bank angle allowed for turns // TODO: Change this when other kinds of protections are active

	auto commanded_roll_rate = 15 * FlightControlSystem_GetUserYokeXPosition(); // degrees per second
	
	if (commanded_roll_rate == 0)
	{		
		// We should be holding the specified angle
		roll_rate_controller.Modify(-0.25, 0.25, 0.32, 0.32, 0);

		// If we are banked beyond the nominal bank angle, roll back to the nominal bank angle
		if (fabs(flight_path_data.roll) > nominal_bank_angle)
		{
			commanded_roll_rate = 5 * -sign(flight_path_data.roll); // roll opposite at 5 degrees per second // TODO: Tuning?
		}
	}
	else
	{
		// We should be responsive to the user's roll request
		roll_rate_controller.Modify(-1, 1, 0.64, 0, 0);
		
		// Check if we are overbanked
		if (fabs(flight_path_data.roll) > maximum_bank_angle)
		{
			// Apply overbank protections
			commanded_roll_rate = 5 * -sign(flight_path_data.roll); // roll opposite at 5 degrees per second // TODO: Tuning?
		}
		// Check if we are approaching overbank
		else if (fabs(flight_path_data.roll) > clamping_bank_angle && sign(flight_path_data.roll) == sign(commanded_roll_rate))
		{
			// Linearly reduce our roll rate authority as we approach the maximum bank angle
			const auto clamping_zone_size = maximum_bank_angle - clamping_bank_angle;
			const auto clamping_zone_position = fabs(flight_path_data.roll) - clamping_bank_angle;
			const auto effectiveness = 1 - (clamping_zone_position / clamping_zone_size);
			commanded_roll_rate *= effectiveness;
		}
	}
	control_surfaces.aileron = roll_rate_controller.Update(commanded_roll_rate - flight_path_data.roll_rate, dt);
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
		auto dt = p_draw_data->dt;
		FlightControlSystem_ManagePitchControl(dt);
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
