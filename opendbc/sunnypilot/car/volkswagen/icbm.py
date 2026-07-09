"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.volkswagen import pqcan, mqbcan, mebcan
from opendbc.car.volkswagen.values import VolkswagenFlags
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

try:
  from openpilot.common.params import Params
except Exception:
  Params = None

BUTTON_INTERVAL_MIN = 0.1   # 10/s hard cap (below this the stock ACC starts dropping presses)
BUTTON_INTERVAL_DEFAULT = 0.2

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.CCS = pqcan if CP.flags & VolkswagenFlags.PQ else (mebcan if CP.flags & VolkswagenFlags.MEB else mqbcan)
    self._params = Params() if Params is not None else None
    self.button_interval = BUTTON_INTERVAL_DEFAULT
    self._param_frame = 0

  def _read_button_interval(self) -> float:
    # tunable press interval (IcbmButtonRate, seconds); clamped so we never spam faster than the
    # stock ACC can register. Defensive against an out-of-sync params registry.
    if self._params is None:
      return BUTTON_INTERVAL_DEFAULT
    try:
      v = float(self._params.get("IcbmButtonRate", return_default=True))
    except Exception:
      return BUTTON_INTERVAL_DEFAULT
    return max(BUTTON_INTERVAL_MIN, v)

  def update(self, CC_SP, CS, packer, frame, CAN) -> list[CanData]:
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame

    self._param_frame += 1
    if self._param_frame % 100 == 0:
      self.button_interval = self._read_button_interval()

    up = self.ICBM.sendButton == SendButtonState.increase
    down = self.ICBM.sendButton == SendButtonState.decrease

    # set and resume buttons are used to achieve +1 and -1 button presses
    # make sure cruise state is already enabled to not enable car cruise user unintended
    if CS.out.cruiseState.enabled and (up or down):
      if (self.frame - self.last_button_frame) * DT_CTRL > self.button_interval:
        can_sends.append(self.CCS.create_acc_buttons_control(packer, CAN, CS.gra_stock_values, up=up, down=down))
        self.last_button_frame = self.frame
    
    return can_sends
