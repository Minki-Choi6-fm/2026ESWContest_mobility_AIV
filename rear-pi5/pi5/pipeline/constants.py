"""All pipeline timing/limit constants in one place.

Two DIFFERENT OBD criteria -- do not mix them up:
  * OBD_FRAME_TIMEOUT_MS : "has a 0x14 frame ARRIVED recently?" (Pi5 receive-side watchdog on the link
                           STM32->Pi5). 0x14 is ~3 Hz, so 100 ms (rule 5 reference) would always read stale.
                           1000 ms is a PROPOSAL, not agreed (rule 8: team review first).
  * OBD_AGE_MS_MAX       : "how old is the speed value INSIDE the frame?" = payload AGE_MS written by the STM32
                           (time since the OBD module last answered). UART v0.2 proposes GPS fallback when
                           AGE_MS > 500 ms. Also a PROPOSAL from the draft spec.
Everything below that is marked PROPOSAL is not a team-approved safety constant.
"""

# --- frame-arrival timeouts per source [ms] (Pi5 receive side) ---
RADAR_TIMEOUT_MS = 100        # rule 5 reference
SUMMARY_TIMEOUT_MS = 100      # 0x10, 50 Hz
IMU_TIMEOUT_MS = 100          # 0x13, 20 Hz
HEARTBEAT_TIMEOUT_MS = 500    # UART v0.2: STM32 considered offline after 500 ms
OBD_FRAME_TIMEOUT_MS = 1000   # PROPOSAL (see above)

TIMEOUT_MS = {
    "radar": RADAR_TIMEOUT_MS,
    "summary": SUMMARY_TIMEOUT_MS,
    "imu": IMU_TIMEOUT_MS,
    "heartbeat": HEARTBEAT_TIMEOUT_MS,
    "obd": OBD_FRAME_TIMEOUT_MS,
}

# --- get_speed() (rules 4, 12) ---
OBD_AGE_MS_MAX = 500          # PROPOSAL from UART v0.2: payload AGE_MS above this -> use GPS
GPS_MAX_AGE_MS = 1500         # same default as ego_speed_tx.py --max-age-ms (GPS ~1 Hz). PROPOSAL
SPEED_MAX_KMH = 655.35        # 0x22 SPEED is u16 x0.01 km/h

# --- BSM adapter (S3). All PROPOSAL, not team-approved ---
BSM_RATE_HZ = 10              # v2v_node.py BSM_RATE_HZ
GPS_POS_MAX_AGE_MS = 1500     # a GPS position older than this -> BSM GPS_VALID=0 (same as GPS_MAX_AGE_MS)
SPEED_STALE_MS = 1000         # total age of the speed value above this (or no source) -> BSM SPEED_STALE=1

# --- 0x22 EGO_SPEED SOURCE codes (UART v0.2) ---
SOURCE_OBD, SOURCE_GPS, SOURCE_NONE = 0, 1, 0xFF

# ======================= S4 (all [PROPOSAL] unless marked) =======================
# --- target selection (target_select.py) ---
# Defaults follow the numbers given for the vision package's fusion.select_lead_vehicle: ahead 1..100 m, |lat| <= 1.75 m, |vrel| <= 60 m/s.
# The S4 corridor with hysteresis (enter 1.5 / exit 2.0 m) is kept as an OPTION (TGT_USE_HYSTERESIS).
TGT_MIN_LONG_M = 1.0            # ignore objects closer than this (ahead distance)
TGT_MAX_LONG_M = 100.0          # ... or farther than this
TGT_LANE_LAT_M = 1.75           # corridor half width when TGT_USE_HYSTERESIS is False. Radar left/right SIGN is not verified -> |lat| is used
TGT_USE_HYSTERESIS = False
TGT_ENTER_LAT_M = 1.5           # option: enter the corridor when |lat| <= this
TGT_EXIT_LAT_M = 2.0            # option: leave the corridor when |lat| > this
TGT_WINDOW_CYCLES = 5           # confirmation window (radar cycles)
TGT_CONFIRM_HITS = 3            # hits needed inside the window
TGT_HOLD_S = 0.3                # keep the last target this long after it disappears
TGT_SWITCH_MARGIN_M = 2.0       # keep the current radar id unless another candidate is nearer by more than this
TGT_VREL_VALID_MAX_MPS = 60.0   # |vrel| above this -> object ignored (logs also contain exactly +-128 / 127.75 = invalid-value pattern)

# --- K estimation (k_estimator.py) ---
K_MIN, K_MAX = 0.3, 1.0         # EXISTING team rule 8 range (not changed)
K_DEFAULT_REAR = 0.5            # conservative default before any measurement
K_BRAKE_MIN_S = 0.5             # a braking segment shorter than this is ignored
K_BRAKE_GAP_S = 0.2             # IMU brake flag may drop this long inside one segment
K_QUANTILE = 0.75               # [hypothesis: reduces noise] upper quantile of the segment deceleration
K_HOLD_S = 300.0                # a measured K is used this long, then back to the default

# --- danger decision (danger.py) ---
G_MPS2 = 9.8
D0_M = 2.0                      # standstill margin
K_FRONT_DEFAULT = 0.8           # used when no fresh front-car K
T_DANGER_S, T_CAUTION_S = 1.5, 2.5          # reaction-time terms in d_req
TTC_DANGER_S, TTC_CAUTION_S = 2.0, 3.0
MIN_CLOSING_MPS = 0.5           # TTC only for closing speed above this
RISE_CONFIRM_N = 2              # consecutive evaluations needed to go up
FALL_MARGIN = 1.1               # to go down the situation must be safe even with thresholds x1.1 ...
FALL_HOLD_S = 0.5               # ... for this long
DANGER_UNKNOWN = 0xFF

# ======================= S5 (all [PROPOSAL]) =======================
# --- danger rule by required deceleration (danger.py) ---
A_REQ_DANGER_FRAC = 0.7         # danger if a_req >= 0.7*K_r*g  (while closing)
A_REQ_CAUTION_FRAC = 0.4        # caution if a_req >= 0.4*K_r*g
T_FOLLOW_CAUTION_S = 1.0        # caution (only up to caution, also when NOT closing) if d < d_req(1.0 s)
D_FLOOR_M = 0.1                 # max(d - d0, D_FLOOR_M) in a_req
# T_DANGER_S / T_CAUTION_S above are legacy (S4); they are only used for the d_req columns of old logs.

# --- suppression rules (each can be switched off) ---
SUPPRESS_STANDSTILL = True      # own speed < STANDSTILL_KMH  -> at most caution
STANDSTILL_KMH = 2.0
SUPPRESS_STATIONARY_OBJECT = True   # target fixed to the ground -> at most caution, unless camera_confirmed or a fresh V2V front car backs it
STATIONARY_VREL_TOL_MPS = 1.5   # |vrel + v_r| <= this  = "stationary over ground"
STATIONARY_DYN = (1,)           # radar dyn values treated as stationary: only 1 is supported by the logs (89 % ground-fixed); 3/5 are NOT (5 behaves like a moving car)
# (S5's dyn-based oncoming switch was REMOVED in S6: dyn==2 looks oncoming in only 72 % of the samples; oncoming/passing is judged by speeds, see below)
PEER_SUPPORT_V_MAX_KMH = 5.0    # a V2V front car counts as "stopped ahead" below this speed ...
PEER_SUPPORT_POS_TOL_M = 10.0   # ... and if its GPS position agrees with the radar target within this

# --- K estimation from the driver's usual braking (k_estimator.py, S5-b; all [PROPOSAL]) ---
# K = effective braking coefficient = deceleration the DRIVER usually uses / g (not the car's maximum). K*g is the deceleration assumed by d_req / a_req.
# The S4 constants K_BRAKE_MIN_S / K_BRAKE_GAP_S / K_HOLD_S above are legacy (firmware-flag based estimator); the profile has no expiry now.
K_BIAS_STOP_S = 2.0             # OBD speed 0 for this long = standstill used to measure the IMU forward-axis bias
K_BIAS_ALPHA = 0.3              # bias_new = (1-alpha)*bias_old + alpha*bias_stop  (slow update at every further stop)
K_BIAS_SETTLE_S = 0.5           # ignore the first part of a standstill
K_EVENT_A_MIN_G = 0.08          # corrected deceleration threshold of a braking event
K_EVENT_MIN_S = 0.4             # the deceleration must last at least this long
K_EVENT_GAP_S = 0.2             # a dip shorter than this stays inside the same event
K_EVENT_SPEED_DROP_KMH = 2.0    # OBD speed must fall by at least this much over the event
K_EVENT_CENTER_FRAC = 0.5       # representative deceleration = mean of the central part (this fraction) of the event (spikes excluded)
K_N_EVENTS = 10                 # K uses the last N events
K_MIN_EVENTS = 3                # fewer events -> K is not advertised (N/A); K_DEFAULT_REAR is used for calculations only

# ======================= S6 (all [PROPOSAL]) =======================
# --- side-lane monitoring (target_select.py): tracked only, never a warning target ---
SIDE_LAT_MAX_M = 3.5            # side candidates: lane edge < |lat| <= this. Radar left/right sign unverified -> no left/right labelling
SIDE_MEMORY_S = 2.0             # an object seen in the side zone within this time before entering the lane is a "cut-in"
# --- oncoming / passing vehicles (danger.py): judged from speeds only (dyn is NOT used) ---
SUPPRESS_ONCOMING_PASSING = True   # own speed known and c > v_r + ONCOMING_MARGIN_MPS -> the target moves TOWARD us -> at most caution (not excluded)
ONCOMING_MARGIN_MPS = 3.0
# --- output policy (pipeline_main.py / docs/handoff/OUTPUT_POLICY.md) ---
BUZZER_MIN_LEVEL = 2            # only danger (2) is a buzzer candidate; caution (1) is display-only; unknown (0xFF) never sounds and is shown as "?"

# ======================= S7 (all [PROPOSAL]) =======================
# --- implausible headway (danger.py): a lead vehicle closer than IMPLAUSIBLE_HEADWAY_S * own speed at >= IMPLAUSIBLE_MIN_KMH is treated as near-field
#     clutter/ghost -> at most caution (NOT excluded; reason "implausible-headway"). Own speed unknown or < IMPLAUSIBLE_MIN_KMH -> not applied.
SUPPRESS_IMPLAUSIBLE_HEADWAY = True
IMPLAUSIBLE_HEADWAY_S = 0.3
IMPLAUSIBLE_MIN_KMH = 10.0
# --- cut-in promotion (target_select.py) ---
SIDE_CUTIN_MIN_S = 1.0          # the object must have been seen CONTINUOUSLY in the side zone this long ...
SIDE_CUTIN_SLOPE_MPS = -0.1     # ... with |lat| decreasing: least-squares slope of |lat| over time must be below this (the instruction says "< 0"; the small
                                #     margin keeps measurement noise of a parallel-running car from passing as a trend)
SIDE_GAP_S = 0.3                # a gap longer than this in the side-zone observations restarts the track (boundary flicker never accumulates)

# ======================= S8 (analysis switch, not a vehicle setting) =======================
REPLAY_ASSUME_CAMERA_CONFIRMED = False   # s4_replay.py only: pretend EVERY target is camera-confirmed (upper bound of what the camera could lift); real logs have no camera

# --- cut-in (brain.py, S8) ---
CUTIN_REJECT_GROUND_FIXED = True    # a promotion candidate that is stationary over ground (|vrel + v_ego| <= STATIONARY_VREL_TOL_MPS) is an ordinary candidate, not a cut-in
                                    # (a roadside object sweeping into the corridor on a curve). Own speed unknown -> the test is not applied and the reason says so.

# --- rear buzzer (buzzer_out.py, S8-d). Wiring told by the user: physical pin 37 = BCM GPIO26 (signal), physical pin 34 = GND. Active/passive type NOT confirmed. ---
BUZZER_BCM_PIN = 26             # gpiozero BCM numbering (RPi.GPIO is not used)
BUZZER_ON_S = 0.1               # danger pattern = the front car's buzzer.py PATTERNS[2] = (0.1 on, 0.1 off) (S12: aligned with the front car; was 0.15/0.15)
BUZZER_OFF_S = 0.1              # ... repeated while level == 2. Type: ACTIVE (user heard it on Pi5 physical pin 37 = BCM26); same gpiozero active-high on/off as front GpioBuzzer
BUZZER_TTL_S = 0.5              # no update for this long -> output OFF by itself (guards a hung main loop) [PROPOSAL]
BUZZER_PASSIVE_HZ = 3900        # passive piezo: PWM frequency [PROPOSAL; the piezo in HARDWARE_REAR.md is rated ~3.9 kHz], duty 50 %

# ======================= S10 camera_confirmed definition ([PROPOSAL], vision_adapter.CameraConfirm) =======================
# True ONLY when our radar target's projected point lies inside a YOLO vehicle box (one-to-one: if several radar targets project into the same box only the NEAREST
# one counts) for CAMERA_CONFIRM_FRAMES consecutive camera frames. Target gone / box gone / target id changed / frames stop arriving -> False immediately.
CAMERA_CONFIRM_FRAMES = 3          # [PROPOSAL] consecutive frames (~0.1 s at 30 fps); not measured
CAMERA_CONFIRM_MAX_FRAME_GAP_S = 0.5   # [PROPOSAL] if no camera frame was processed for this long the confirmation is dropped (a frozen camera must not keep True)

