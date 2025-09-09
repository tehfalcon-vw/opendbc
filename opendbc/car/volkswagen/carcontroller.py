import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits, apply_std_curvature_limits
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volkswagen import mqbcan, pqcan, mebcan, pandacan
from opendbc.car.volkswagen.values import CanBus, CarControllerParams, VolkswagenFlags
from opendbc.car.volkswagen.mebutils import get_long_jerk_limits, get_long_control_limits, map_speed_to_acc_tempolimit, LatControlCurvature

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.CCP = CarControllerParams(CP)
    self.CAN = CanBus(CP)
    self.CCS = pqcan if CP.flags & VolkswagenFlags.PQ else (mebcan if CP.flags & VolkswagenFlags.MEB else mqbcan)
    self.PC = pandacan
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.aeb_available = not CP.flags & VolkswagenFlags.PQ

    self.apply_torque_last = 0
    self.apply_curvature_last = 0.
    self.steering_power_last = 0
    self.steering_offset = 0.
    self.accel_last = 0.
    self.long_jerk_up_last = 0.
    self.long_jerk_down_last = 0.
    self.long_dy_up_last = 0.
    self.long_dy_down_last = 0.
    self.long_override_counter = 0
    self.long_disabled_counter = 0
    self.gra_acc_counter_last = None
    self.klr_counter_last = None
    self.eps_timer_soft_disable_alert = False
    self.hca_frame_timer_running = 0
    self.hca_frame_same_torque = 0
    self.lead_distance_bars_last = None
    self.distance_bar_frame = 0
    self.long_cruise_control = False
    self.gra_enabled = False
    self.gra_up = False
    self.gra_down = False
    self.speed_limit_last = 0
    self.speed_limit_changed_timer = 0
    self.LateralController = (
      LatControlCurvature(self.CCP.CURVATURE_PID, self.CCP.CURVATURE_LIMITS.CURVATURE_MAX, 1 / (DT_CTRL * self.CCP.STEER_STEP))
      if (CP.flags & VolkswagenFlags.MEB)
      else None
    )

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []
    
    CS.force_rhd_for_bsm = CC.forceRHDForBSM

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      if self.CP.flags & VolkswagenFlags.MEB:
        # Logic to avoid HCA refused state:
        #   * steering power as counter and near zero before OP lane assist deactivation
        # MEB rack can be used continously without time limits
        # maximum real steering angle change ~ 120-130 deg/s
        
        if CC.latActive:
          hca_enabled = True
          if CC.curvatureControllerActive: # PID + (car curv - VM no roll)
            apply_curvature = self.LateralController.update(CS.out, CC, actuators.curvature)
            apply_curvature = apply_curvature + (CS.out.steeringCurvature - (CC.currentCurvature - CC.rollCompensation))
          else: # car curv - VM with roll
            apply_curvature = actuators.curvature + (CS.out.steeringCurvature - CC.currentCurvature)
          apply_curvature = apply_std_curvature_limits(apply_curvature, self.apply_curvature_last, CS.out.vEgoRaw, CS.out.steeringCurvature,
                                                       CS.out.steeringPressed, self.CCP.STEER_STEP, CC.latActive, self.CCP.CURVATURE_LIMITS)

          min_power = max(self.steering_power_last - self.CCP.STEERING_POWER_STEP, self.CCP.STEERING_POWER_MIN)
          max_power = np.interp(CS.out.vEgo, [0., 1.5], [self.CCP.STEERING_POWER_MIN, max(self.steering_power_last + self.CCP.STEERING_POWER_STEP, self.CCP.STEERING_POWER_MAX)])
          target_power = int(np.interp(CS.out.steeringTorque, [self.CCP.STEER_DRIVER_ALLOWANCE, self.CCP.STEER_DRIVER_MAX],
                                                              [self.CCP.STEERING_POWER_MAX, self.CCP.STEERING_POWER_MIN]))
          steering_power = min(max(target_power, min_power), max_power)
          
        else:
          self.LateralController.reset()
          if self.steering_power_last > 0: # keep HCA alive until steering power has reduced to zero
            hca_enabled = True
            apply_curvature = np.clip(CS.out.steeringCurvature, -self.CCP.CURVATURE_LIMITS.CURVATURE_MAX, self.CCP.CURVATURE_LIMITS.CURVATURE_MAX) # synchronize with current curvature
            steering_power = max(self.steering_power_last - self.CCP.STEERING_POWER_STEP, 0)
          else: 
            hca_enabled = False
            apply_curvature = 0. # inactive curvature
            steering_power = 0

        can_sends.append(self.CCS.create_steering_control(self.packer_pt, self.CAN.pt, apply_curvature, hca_enabled, steering_power))
        self.apply_curvature_last = apply_curvature
        self.steering_power_last = steering_power
        
      else:
        # Logic to avoid HCA state 4 "refused":
        #   * Don't steer unless HCA is in state 3 "ready" or 5 "active"
        #   * Don't steer at standstill
        #   * Don't send > 3.00 Newton-meters torque
        #   * Don't send the same torque for > 6 seconds
        #   * Don't send uninterrupted steering for > 360 seconds
        # MQB racks reset the uninterrupted steering timer after a single frame
        # of HCA disabled; this is done whenever output happens to be zero.

        if CC.latActive:
          new_torque = int(round(actuators.torque * self.CCP.STEER_MAX))
          apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)
          self.hca_frame_timer_running += self.CCP.STEER_STEP
          if self.apply_torque_last == apply_torque:
            self.hca_frame_same_torque += self.CCP.STEER_STEP
            if self.hca_frame_same_torque > self.CCP.STEER_TIME_STUCK_TORQUE / DT_CTRL:
              apply_torque -= (1, -1)[apply_torque < 0]
              self.hca_frame_same_torque = 0
          else:
            self.hca_frame_same_torque = 0
          hca_enabled = abs(apply_torque) > 0
        else:
          hca_enabled = False
          apply_torque = 0

        if not hca_enabled:
          self.hca_frame_timer_running = 0

        self.eps_timer_soft_disable_alert = self.hca_frame_timer_running > self.CCP.STEER_TIME_ALERT / DT_CTRL
        self.apply_torque_last = apply_torque
        can_sends.append(self.CCS.create_steering_control(self.packer_pt, self.CAN.pt, apply_torque, hca_enabled))

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
        # to the greatest of actual driver input or 2x openpilot's output (1x openpilot output is not enough to
        # consistently reset inactivity detection on straight level roads). See commaai/openpilot#23274 for background.
        ea_simulated_torque = float(np.clip(apply_torque * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX))
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, self.CAN.cam, CS.eps_stock_values, ea_simulated_torque))

    # Emergency Assist intervention
    if self.CP.flags & VolkswagenFlags.MEB and self.CP.flags & VolkswagenFlags.STOCK_KLR_PRESENT:
      # send capacitive steering wheel touched
      # propably EA is stock activated only for cars equipped with capacitive steering wheel
      # (also stock long does resume from stop as long as hands on is detected additionally to OP resume spam)
      klr_send_ready = CS.klr_stock_values["COUNTER"] != self.klr_counter_last
      if klr_send_ready:
        can_sends.append(mebcan.create_capacitive_wheel_touch(self.packer_pt, self.CAN.cam, CC.latActive, CS.klr_stock_values))
        can_sends.append(mebcan.create_capacitive_wheel_touch(self.packer_pt, self.CAN.pt, CC.latActive, CS.klr_stock_values))
      self.klr_counter_last = CS.klr_stock_values["COUNTER"]

    # **** Blinker Controls ************************************************** #
    # "Wechselblinken" has to be allowed in assistance blinker functions in gateway
    # "Wechselblinken" means switching between hazards and one sided indicators for every indicator cycle (VW MEB full cycle: 0.8 seconds, 1st normal, 2nd hazards)
    # user input has hgher prio than EA indicating, post cycle handover is done via actual indicator signal if EA would already request
    # signaling indicators for 1 frame to trigger the first non hazard cycle, retrigger after the car signals a fully ended cycle
    if self.CP.flags & VolkswagenFlags.MEB:
      if self.frame % 2 == 0:
        blinker_active = CS.left_blinker_active or CS.right_blinker_active
        left_blinker = CC.leftBlinker if not blinker_active else False
        right_blinker = CC.rightBlinker if not blinker_active else False
        can_sends.append(mebcan.create_blinker_control(self.packer_pt, self.CAN.pt, CS.ea_hud_stock_values, left_blinker, right_blinker))

    # **** Cruise Controls ************************************************** #
    
    self.long_cruise_control = True if CS.acc_type == 3 and self.CP.flags & VolkswagenFlags.PQ else False
    
    if self.frame % 15 == 0 and self.CP.openpilotLongitudinalControl and self.long_cruise_control:
      self.gra_enabled = CC.longActive and CS.out.cruiseState.enabled
      set_speed = int(round(CS.out.cruiseState.speed * CV.MS_TO_KPH))
      actuator_speed = int(round(actuators.speed * CV.MS_TO_KPH))
      self.gra_up = True if set_speed < actuator_speed and self.gra_enabled else False
      self.gra_down = True if set_speed > actuator_speed and self.gra_enabled else False
    
    # **** Acceleration Controls ******************************************** #
    
    if self.frame % self.CCP.ACC_CONTROL_STEP == 0 and self.CP.openpilotLongitudinalControl:
      if not self.long_cruise_control:
        stopping = actuators.longControlState == LongCtrlState.stopping
        
        if self.CP.flags & VolkswagenFlags.MEB:
          # Logic to prevent car error with EPB:
          #   * send a few frames of HMS RAMP RELEASE command at the very begin of long override and right at the end of active long control -> clean exit of ACC car controls
          #   * (1 frame of HMS RAMP RELEASE is enough, but lower the possibility of panda safety blocking it)
          starting = actuators.longControlState == LongCtrlState.starting if self.long_override_counter == 0 else False # openpilot sets starting state after overriding ...
          accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.enabled else 0)

          long_override = CC.cruiseControl.override or CS.out.gasPressed
          self.long_override_counter = min(self.long_override_counter + 1, 5) if long_override else 0
          long_override_begin = long_override and self.long_override_counter < 5

          self.long_disabled_counter = min(self.long_disabled_counter + 1, 5) if not CC.enabled else 0
          long_disabling = not CC.enabled and self.long_disabled_counter < 5

          critical_state = hud_control.visualAlert == VisualAlert.fcw
          upper_control_limit, lower_control_limit = get_long_control_limits(CC.enabled, CS.out.vEgo, hud_control.setSpeed, hud_control.leadDistance, hud_control.leadVisible, critical_state)
          self.long_jerk_up_last, self.long_jerk_down_last, self.long_dy_up_last, self.long_dy_down_last = get_long_jerk_limits(CC.enabled, long_override,
                                                                                                                                hud_control.leadDistance, hud_control.leadVisible,
                                                                                                                                accel, self.accel_last, self.long_jerk_up_last,
                                                                                                                                self.long_jerk_down_last, self.long_dy_up_last,
                                                                                                                                self.long_dy_down_last,
                                                                                                                                DT_CTRL * self.CCP.ACC_CONTROL_STEP,
                                                                                                                                critical_state)

          acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled, long_override)          
          acc_hold_type = self.CCS.acc_hold_type(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled, starting, stopping,
                                                 CS.esp_hold_confirmation, long_override, long_override_begin, long_disabling)
          can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, self.CAN.pt, self.CP, CS.acc_type, CC.enabled,
                                                             self.long_jerk_up_last, self.long_jerk_down_last, upper_control_limit, lower_control_limit,
                                                             accel, acc_control, acc_hold_type, stopping, starting, CS.esp_hold_confirmation,
                                                             CS.out.vEgoRaw * CV.MS_TO_KPH, long_override, CS.travel_assist_available))

        else:
          starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)
          accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0)
        
          acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
          can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, self.CAN.pt, CS.acc_type, CC.longActive, accel,
                                                             acc_control, stopping, starting, CS.esp_hold_confirmation))
        self.accel_last = accel

      #if self.aeb_available:
      #  if self.frame % self.CCP.AEB_CONTROL_STEP == 0:
      #    can_sends.append(self.CCS.create_aeb_control(self.packer_pt, False, False, 0.0))
      #  if self.frame % self.CCP.AEB_HUD_STEP == 0:
      #    can_sends.append(self.CCS.create_aeb_hud(self.packer_pt, False, False))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]

      if self.CP.flags & VolkswagenFlags.MEB:
        sound_alert = self.CCP.LDW_SOUNDS["Chime"] if hud_alert == self.CCP.LDW_MESSAGES["laneAssistTakeOver"] else self.CCP.LDW_SOUNDS["None"]
        can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, self.CAN.pt, CS.ldw_stock_values, CC.latActive,
                                                         CS.out.steeringPressed, hud_alert, hud_control, sound_alert))
      else:
        can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, self.CAN.pt, CS.ldw_stock_values, CC.latActive,
                                                         CS.out.steeringPressed, hud_alert, hud_control))

    if hud_control.leadDistanceBars != self.lead_distance_bars_last:
      self.distance_bar_frame = self.frame
    
    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      if not(CS.acc_type == 3 and self.CP.flags & VolkswagenFlags.PQ):
        if self.CP.flags & VolkswagenFlags.MEB:
          fcw_alert = hud_control.visualAlert == VisualAlert.fcw
          show_distance_bars = self.frame - self.distance_bar_frame < 400
          gap = max(8, CS.out.vEgo * hud_control.leadFollowTime)
          distance = max(8, hud_control.leadDistance) if hud_control.leadDistance != 0 else 0
          acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled,
                                                         CC.cruiseControl.override or CS.out.gasPressed)
          
          sl_predicative_active = True if CC.cruiseControl.speedLimitPredicative and CS.out.cruiseState.speedLimitPredicative != 0 else False
          if CC.cruiseControl.speedLimit and CS.out.cruiseState.speedLimit != 0 and self.speed_limit_last != CS.out.cruiseState.speedLimit:
            self.speed_limit_changed_timer = self.frame 
          self.speed_limit_last = CS.out.cruiseState.speedLimit
          sl_active = self.frame - self.speed_limit_changed_timer < 400
          speed_limit = CS.out.cruiseState.speedLimitPredicative if sl_predicative_active else (CS.out.cruiseState.speedLimit if sl_active else 0)
          
          acc_hud_event = self.CCS.acc_hud_event(acc_hud_status, CS.esp_hold_confirmation, sl_predicative_active, sl_active)
          
          can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_hud_status, hud_control.setSpeed * CV.MS_TO_KPH,
                                                           hud_control.leadVisible, hud_control.leadDistanceBars + 1, show_distance_bars,
                                                           CS.esp_hold_confirmation, distance, gap, fcw_alert, acc_hud_event, speed_limit))

        else:
          lead_distance = 0
          if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:  # Don't display lead until we know the scaling factor
            lead_distance = 512 if CS.upscale_lead_car_signal else 8
          acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
          # FIXME: follow the recent displayed-speed updates, also use mph_kmh toggle to fix display rounding problem?
          set_speed = hud_control.setSpeed * CV.MS_TO_KPH
          can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_hud_status, set_speed,
                                                           lead_distance, hud_control.leadDistanceBars))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready:
      bus_send = self.CAN.main if self.CP.flags & VolkswagenFlags.PQ else self.CAN.ext
      if self.CP.pcmCruise and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
        can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, bus_send, CS.gra_stock_values,
                                                             cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))
      elif self.CP.openpilotLongitudinalControl and self.long_cruise_control and self.gra_enabled:
        can_sends.append(self.CCS.create_gra_buttons_control(self.packer_pt, bus_send, CS.gra_stock_values,
                                                             up=self.gra_up, down=self.gra_down))
        self.gra_up = False
        self.gra_down = False

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    new_actuators.curvature = float(self.apply_curvature_last)
    new_actuators.accel = self.accel_last
    new_actuators.speed = actuators.speed

    self.lead_distance_bars_last = hud_control.leadDistanceBars
    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends
