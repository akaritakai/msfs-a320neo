#include <MSFS/MSFS.h>
#include <MSFS/MSFS_Render.h>
#include <MSFS/Legacy/gauges.h>
#include <SimConnect.h>

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
	ENUM feet_units; // Feet
	ENUM gforce_units; // GForce
	// Variables
	ENUM gforce; // G FORCE
	ENUM plane_pitch_degrees; // PLANE PITCH DEGREES
	ENUM radio_height; // RADIO HEIGHT
	ENUM sim_on_ground; // SIM ON GROUND
} sim_vars;

bool HandleSimVars(FsContext ctx, int service_id, void* pData)
{
	switch (service_id)
	{
	case PANEL_SERVICE_PRE_INSTALL:
	{
		// Units
		sim_vars.bool_units = get_units_enum("Bool");
		sim_vars.degrees_units = get_units_enum("Degrees");
		sim_vars.feet_units = get_units_enum("Feet");
		sim_vars.gforce_units = get_units_enum("GForce");
		// Variables
		sim_vars.gforce = get_aircraft_var_enum("G FORCE");
		sim_vars.plane_pitch_degrees = get_aircraft_var_enum("PLANE PITCH DEGREES");
		sim_vars.radio_height = get_aircraft_var_enum("RADIO HEIGHT");
		sim_vars.sim_on_ground = get_aircraft_var_enum("SIM ON GROUND");
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
bool HandleSimConnect(FsContext ctx, int service_id, void* pData)
{
	switch (service_id)
	{
	case PANEL_SERVICE_PRE_INSTALL:
	{
		hSimConnect = SimConnect_Open(&hSimConnect, "A32NX", nullptr, 0, 0, 0);
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
		SimConnect_Close(hSimConnect);
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
bool HandleFlightControlLaws(FsContext ctx, int service_id, void* pData)
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
const char* PITCH_CONTROL_MODE_VAR_NAME = "PITCH CONTROL MODE";

double ground_to_flight_condition_start_time = 0;
void PitchControlMode_HandleGroundTransitions(double current_sim_time)
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
				if (ground_to_flight_condition_start_time == 0)
				{
					ground_to_flight_condition_start_time = current_sim_time;
				}
				else
				{
					if (current_sim_time - ground_to_flight_condition_start_time >= 5)
					{
						printf("A32NX: Ground Mode -> Flight Mode\n");
						pitch_control_mode = FLIGHT_MODE;
						ground_to_flight_condition_start_time = 0;
					}
				}
				return;
			}
			ground_to_flight_condition_start_time = 0;
		}
		// TODO: Handle other laws
	}
}

double flight_to_flare_condition_start_time = 0;
void PitchControlMode_HandleFlightTransitions(double current_sim_time)
{
	if (pitch_control_mode == FLIGHT_MODE)
	{
		if (InNormalLaw())
		{
			// Handle flight to flare transition
			auto radio_altimeter = aircraft_varget(sim_vars.radio_height, sim_vars.feet_units, 0);
			if (radio_altimeter < 50)
			{
				if (flight_to_flare_condition_start_time == 0)
				{
					flight_to_flare_condition_start_time = current_sim_time;
				}
				else
				{
					if (current_sim_time - flight_to_flare_condition_start_time >= 1)
					{
						printf("A32NX: Flight Mode -> Flare Mode\n");
						pitch_control_mode = FLARE_MODE;
						flight_to_flare_condition_start_time = 0;
					}
				}
				return;
			}
			flight_to_flare_condition_start_time = 0;
		}
		// TODO: Handle other laws
	}
}

double flare_to_flight_condition_start_time = 0;
double flare_to_ground_condition_start_time = 0;
void PitchControlMode_HandleFlareTransitions(double current_sim_time)
{
	if (pitch_control_mode == FLARE_MODE)
	{
		if (InNormalLaw())
		{
			// Handle flare to flight transition
			auto radio_altimeter = aircraft_varget(sim_vars.radio_height, sim_vars.feet_units, 0);
			if (radio_altimeter > 50)
			{
				if (flare_to_flight_condition_start_time == 0)
				{
					flare_to_flight_condition_start_time = current_sim_time;
				}
				else
				{
					if (current_sim_time - flare_to_flight_condition_start_time >= 1)
					{
						printf("A32NX: Flare Mode -> Flight Mode\n");
						pitch_control_mode = FLIGHT_MODE;
						flare_to_flight_condition_start_time = 0;
					}
				}
				return;
			}
			flare_to_flight_condition_start_time = 0;
			// Handle flare to ground transition
			auto on_ground = aircraft_varget(sim_vars.sim_on_ground, sim_vars.bool_units, 0) != 0;
			auto pitch_attitude = aircraft_varget(sim_vars.plane_pitch_degrees, sim_vars.degrees_units, 0);
			if (on_ground && (pitch_attitude < 2.5))
			{
				if (flare_to_ground_condition_start_time == 0)
				{
					flare_to_ground_condition_start_time = current_sim_time;
				}
				else
				{
					if (current_sim_time - flare_to_ground_condition_start_time >= 5)
					{
						printf("A32NX: Flare Mode -> Ground Mode\n");
						pitch_control_mode = GROUND_MODE;
						flare_to_ground_condition_start_time = 0;
					}
				}
				return;
			}
			flare_to_ground_condition_start_time = 0;
		}
		// TODO: Handle other laws
	}
}

bool HandlePitchControlMode(FsContext ctx, int service_id, void* pData)
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
		// - The t member gives the absolute simulation time.
		// - The dt member gives the time elapsed since last frame.
		auto* p_draw_data = static_cast<sGaugeDrawData*>(pData);
		auto current_sim_time = p_draw_data->t; // Assuming this is seconds	
		switch (pitch_control_mode)
		{
		case GROUND_MODE:
			printf("CURRENT MODE: GROUND MODE\n");
			PitchControlMode_HandleGroundTransitions(current_sim_time);
			break;
		case FLIGHT_MODE:
			printf("CURRENT MODE: FLIGHT MODE\n");
			PitchControlMode_HandleFlightTransitions(current_sim_time);
			break;
		case FLARE_MODE:
			printf("CURRENT MODE: FLARE MODE\n");
			PitchControlMode_HandleFlareTransitions(current_sim_time);
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

bool HandleInputCapture(FsContext ctx, int service_id, void* pData)
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

bool HandleControlSurfaces(FsContext ctx, int service_id, void* pData)
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
		control_surfaces.elevator = user_input.yoke_y;
		control_surfaces.aileron = user_input.yoke_x;
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
	MSFS_CALLBACK bool A32NX_gauge_callback(FsContext ctx, int service_id, void* pData)
	{
		auto ret = false;
		ret |= HandleSimVars(ctx, service_id, pData);
		ret |= HandleSimConnect(ctx, service_id, pData);
		ret |= HandleFlightControlLaws(ctx, service_id, pData);
		ret |= HandlePitchControlMode(ctx, service_id, pData);
		ret |= HandleInputCapture(ctx, service_id, pData);
		ret |= HandleControlSurfaces(ctx, service_id, pData);
		return ret;
	}
}
