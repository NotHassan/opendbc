"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import DT_CTRL, structs
from opendbc.car.common.conversions import Conversions as CV

try:
  from openpilot.common.params import Params
except Exception:
  Params = None
from opendbc.car.can_definitions import CanData
from opendbc.car.volkswagen import pqcan, mqbcan, mebcan
from opendbc.car.volkswagen.values import VolkswagenFlags
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.CCS = pqcan if CP.flags & VolkswagenFlags.PQ else (mebcan if CP.flags & VolkswagenFlags.MEB else mqbcan)
    # Tiguan MK3: faster setpoint walk for curve assist. Each press is still a single discrete
    # GRA_ACC_01 injection (no hold semantics; VW's +/-10 needs ~0.5 s hold), just more attempts
    # per second -- road logs showed single-cycle presses get missed (~2-3.6 kph/s effective of
    # the theoretical 5 at 0.2 s).
    self.button_interval = 0.12 if CP.flags & VolkswagenFlags.TIGUAN_MK3_TUNING else 0.2
    # Tiguan MK3: big-step (+/-10) presses via GRA_Tip_Hoch/Runter when the target is far away.
    # Measured on this car: stock ACC decel scales with the gap and reaches ~1.0 m/s2 at ~29, so
    # opening the gap fast is what makes curve slowdowns effective.
    self.big_step = bool(CP.flags & VolkswagenFlags.TIGUAN_MK3_TUNING)
    self.last_big_frame = 0
    # Standby memory walk: the VW setpoint memory is adjustable while cruise is disengaged, so a
    # pending restore can move it before the driver re-engages. ONLY the big-step (Hoch/Runter)
    # buttons are used in standby -- Setzen/Wiederaufnahme would ENGAGE cruise. Canary guard: if
    # cruise unexpectedly engages within 1.5 s of a standby press (and the driver's own stalk
    # didn't do it), send cancel immediately and lock standby pressing out.
    self.last_standby_press_frame = -10000
    self.last_user_engage_press_frame = -10000
    self.standby_lockout = False
    self.cruise_enabled_prev = False
    self._params = Params() if Params is not None else None
    self._is_metric = True
    self._unit_frame = 0

  def update(self, CC_SP, CS, packer, frame, CAN) -> list[CanData]:
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame

    up = self.ICBM.sendButton == SendButtonState.increase
    down = self.ICBM.sendButton == SendButtonState.decrease

    if self.big_step:
      # track the driver's own engage-capable presses (stock stalk set/resume)
      try:
        if CS.gra_stock_values.get("GRA_Tip_Setzen", 0) or CS.gra_stock_values.get("GRA_Tip_Wiederaufnahme", 0):
          self.last_user_engage_press_frame = self.frame
      except Exception:
        pass
      cruise_on = bool(CS.out.cruiseState.enabled)
      if cruise_on and not self.cruise_enabled_prev:
        if (self.frame - self.last_standby_press_frame) * DT_CTRL < 1.5 and \
           (self.frame - self.last_user_engage_press_frame) * DT_CTRL > 1.0:
          # engagement right after OUR standby press with no driver stalk input: abort hard
          can_sends.append(self.CCS.create_acc_buttons_control(packer, CAN, CS.gra_stock_values, cancel=True))
          self.standby_lockout = True
      self.cruise_enabled_prev = cruise_on

      # standby walk: big-step presses only, while cruise is available but disengaged
      if (not cruise_on) and CS.out.cruiseState.available and not self.standby_lockout and (up or down):
        if (self.frame - self.last_button_frame) * DT_CTRL > 0.4:
          speed_conv = CV.MS_TO_KPH if self._is_metric else CV.MS_TO_MPH
          diff = self.ICBM.vTarget - CS.out.cruiseState.speedCluster * speed_conv
          if abs(diff) >= 10.:
            can_sends.append(self.CCS.create_acc_buttons_control(packer, CAN, CS.gra_stock_values,
                                                                 up_big=(up and diff > 0), down_big=(down and diff < 0)))
            self.last_button_frame = self.frame
            self.last_standby_press_frame = self.frame
        return can_sends

    # set and resume buttons are used to achieve +1 and -1 button presses
    # make sure cruise state is already enabled to not enable car cruise user unintended
    if CS.out.cruiseState.enabled and (up or down):
      if (self.frame - self.last_button_frame) * DT_CTRL > self.button_interval:
        up_big = down_big = False
        if self.big_step:
          self._unit_frame += 1
          if self._params is not None and self._unit_frame % 100 == 1:
            try:
              self._is_metric = self._params.get_bool("IsMetric")
            except Exception:
              pass
          speed_conv = CV.MS_TO_KPH if self._is_metric else CV.MS_TO_MPH
          diff = self.ICBM.vTarget - CS.out.cruiseState.speedCluster * speed_conv
          # holdoff after a big press: the cluster takes >0.1 s to reflect the step; without it a
          # stale readback double-fires
          if (self.frame - self.last_big_frame) * DT_CTRL > 0.4:
            if down and diff <= -13.:
              down_big, down = True, False
              self.last_big_frame = self.frame
            elif up and diff >= 13.:
              up_big, up = True, False
              self.last_big_frame = self.frame
        if self.big_step:
          can_sends.append(self.CCS.create_acc_buttons_control(packer, CAN, CS.gra_stock_values, up=up, down=down,
                                                               up_big=up_big, down_big=down_big))
        else:
          can_sends.append(self.CCS.create_acc_buttons_control(packer, CAN, CS.gra_stock_values, up=up, down=down))
        self.last_button_frame = self.frame
    
    return can_sends
