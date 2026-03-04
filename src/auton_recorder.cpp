#include "auton_recorder.hpp"
#include "main.h"
#include "intake.hpp"
#include "subsystems.hpp"
#include "pros/misc.h"
#include <vector>
#include <cstdio>

// =============================================================================
//  Autonomous Recorder for EZ-Template (VEX PROS)
//
//  Records driver inputs during opcontrol and replays them in autonomous.
//  All joystick axes and button states are sampled every ez::util::DELAY_TIME
//  milliseconds, keeping playback timing perfectly in sync with the opcontrol
//  loop. Recordings are saved to the SD card and persist across power cycles.
//
//  CONTROLS (during driver control only):
//    Press RECORD_BUTTON once  — starts recording (controller rumbles + shows "REC STARTED")
//    Press RECORD_BUTTON again — stops recording and saves to SD card
//
//  SETUP:
//    1. Add  #include "auton_recorder.hpp"  to main.h
//    2. Call auton_recorder_init()   inside initialize()
//    3. Call auton_recorder_update() at the top of the opcontrol() while-loop,
//           before any chassis or subsystem code
//    4. Call replay_recorded_auton() from autonomous() or from an auton
//           selector entry to play the recording back
//
//  CUSTOMIZATION:
//    - Change RECORD_BUTTON below to any unused controller button
//    - Update the ButtonBits enum and apply_frame() / auton_recorder_update()
//      to match your robot's subsystem functions and button bindings
// =============================================================================

// ── Button used to start and stop recording ──────────────────────────────────
// Change this to any button that is not already used in opcontrol.
#define RECORD_BUTTON pros::E_CONTROLLER_DIGITAL_UP

// ── SD card path — change the filename if needed ─────────────────────────────
static const char* SD_PATH = "/usd/recorded_auton.bin";

// ── Input frame — one snapshot of all inputs per loop tick ───────────────────
struct InputFrame {
    // Drive axes — captured raw before EZ-Template processes them
    int8_t left_y;   // ANALOG_LEFT_Y  (forward/reverse in split arcade)
    int8_t right_x;  // ANALOG_RIGHT_X (turning in split arcade)

    // All subsystem button states packed into a single bitmask
    uint8_t buttons;
};

// Button bitmask positions — add, remove, or rename these to match your robot.
// Each bit corresponds to one button and maps to a subsystem action below.
enum ButtonBits : uint8_t {
    B_L1         = (1 << 0),  // intake_in()
    B_L2         = (1 << 1),  // slopeScore()
    B_A          = (1 << 2),  // intake_out()
    B_R2         = (1 << 3),  // middleScore()
    B_B          = (1 << 4),  // tongue toggle
    B_R1         = (1 << 5),  // descore toggle
    // bits 6-7 available for future use
};

// ── Module state ─────────────────────────────────────────────────────────────
static std::vector<InputFrame> g_recording;
static bool g_is_recording = false;

// ── SD card helpers ───────────────────────────────────────────────────────────

static void save_to_sd() {
    FILE* f = fopen(SD_PATH, "wb");
    if (!f) {
        master.print(0, 0, "SD save FAILED");
        printf("[Recorder] ERROR: could not open SD card for writing.\n");
        return;
    }
    size_t n = g_recording.size();
    fwrite(&n, sizeof(n), 1, f);
    fwrite(g_recording.data(), sizeof(InputFrame), n, f);
    fclose(f);
    printf("[Recorder] Saved %zu frames (%.2f s) to SD card.\n",
           n, n * ez::util::DELAY_TIME / 1000.0);
}

static bool load_from_sd() {
    FILE* f = fopen(SD_PATH, "rb");
    if (!f) {
        printf("[Recorder] No recording found on SD card.\n");
        return false;
    }
    size_t n = 0;
    fread(&n, sizeof(n), 1, f);
    g_recording.resize(n);
    fread(g_recording.data(), sizeof(InputFrame), n, f);
    fclose(f);
    printf("[Recorder] Loaded %zu frames (%.2f s) from SD card.\n",
           n, n * ez::util::DELAY_TIME / 1000.0);
    return true;
}

// ── Apply a single recorded frame to the drivetrain and subsystems ────────────
// This function is called once per tick during replay. Update the subsystem
// calls here to match whatever functions you use in opcontrol().
static void apply_frame(const InputFrame& fr) {
    // Compute the same split arcade mix that EZ-Template uses internally.
    // EZ-Template's opcontrol functions read the controller live, so during
    // autonomous we bypass them and write motor voltages directly instead.
    int drive  = fr.left_y;
    int turn   = fr.right_x;
    int left   = drive + turn;
    int right  = drive - turn;
    // Clamp to [-127, 127]
    left  = left  >  127 ?  127 : (left  < -127 ? -127 : left);
    right = right >  127 ?  127 : (right < -127 ? -127 : right);
    chassis.drive_set(left, right);

    // Subsystems — update these calls to match your opcontrol() button bindings.
    // The priority order here should be identical to the order in opcontrol().
    if (fr.buttons & B_L1) {
        intake_in();
    } else if (fr.buttons & B_L2) {
        slopeScore();
    } else if (fr.buttons & B_A) {
        intake_out();
    } else if (fr.buttons & B_R2) {
        middleScore();
    } else {
        stopIntake();
    }

    // Toggle-based mechanisms are handled with rising edge detection in
    // replay_recorded_auton() below, not here. The raw button state is stored
    // in the bitmask so edges can be computed by comparing adjacent frames.
}

// ── Public API ────────────────────────────────────────────────────────────────

void auton_recorder_init() {
    load_from_sd();
}

void auton_recorder_update() {
    // Toggle recording with UP button
    if (master.get_digital_new_press(RECORD_BUTTON)) {
        if (!g_is_recording) {
            g_recording.clear();
            g_is_recording = true;
            master.rumble(".");
            master.print(0, 0, "REC STARTED   ");
            printf("[Recorder] Recording started.\n");
        } else {
            g_is_recording = false;
            master.rumble("..");
            master.print(0, 0, "REC SAVED     ");
            printf("[Recorder] Recording stopped.\n");
            save_to_sd();
        }
    }

    // Sample all inputs and store them as a frame
    if (g_is_recording) {
        InputFrame fr;
        fr.left_y  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        fr.right_x = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        fr.buttons = 0;
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) fr.buttons |= B_L1;
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) fr.buttons |= B_L2;
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A))  fr.buttons |= B_A;
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) fr.buttons |= B_R2;
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B))  fr.buttons |= B_B;
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) fr.buttons |= B_R1;

        g_recording.push_back(fr);
    }
}

void replay_recorded_auton() {
    if (g_recording.empty()) {
        if (!load_from_sd() || g_recording.empty()) {
            printf("[Recorder] Nothing to replay!\n");
            return;
        }
    }

    printf("[Recorder] Replaying %zu frames (%.2f s)...\n",
           g_recording.size(),
           g_recording.size() * ez::util::DELAY_TIME / 1000.0);

    uint8_t prev_buttons = 0;

    for (const auto& fr : g_recording) {
        // Drive
        int drive = fr.left_y;
        int turn  = fr.right_x;
        int left  = drive + turn;
        int right = drive - turn;
        left  = left  >  127 ?  127 : (left  < -127 ? -127 : left);
        right = right >  127 ?  127 : (right < -127 ? -127 : right);
        chassis.drive_set(left, right);

        // Subsystem replay — same priority chain as opcontrol()
        if (fr.buttons & B_L1) {
            intake_in();
        } else if (fr.buttons & B_L2) {
            slopeScore();
        } else if (fr.buttons & B_A) {
            intake_out();
        } else if (fr.buttons & B_R2) {
            middleScore();
        } else {
            stopIntake();
        }

        // Toggle mechanisms — fire button_toggle only on a rising edge so the
        // piston fires exactly once per press, matching opcontrol() behavior.
        bool tongue_now  = fr.buttons & B_B;
        bool tongue_prev = prev_buttons & B_B;
        if (tongue_now && !tongue_prev)
            tongue.button_toggle(true);
        else
            tongue.button_toggle(false);

        // Descore toggle
        bool descore_now  = fr.buttons & B_R1;
        bool descore_prev = prev_buttons & B_R1;
        if (descore_now && !descore_prev)
            descore.button_toggle(true);
        else
            descore.button_toggle(false);

        prev_buttons = fr.buttons;
        pros::delay(ez::util::DELAY_TIME);
    }

    // Safe stop
    chassis.drive_set(0, 0);
    stopIntake();
    printf("[Recorder] Replay complete.\n");
}