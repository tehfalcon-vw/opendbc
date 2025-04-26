from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volkswagen.values import VolkswagenFlags

SPEED_LIMIT_NOT_SET = 0

class SpeedLimitManager:
  def __init__(self, car_params):
    self.CP = car_params
    self.v_limit = SPEED_LIMIT_NOT_SET
    self.v_limit_receive = False
    self.v_limit_speed_factor = CV.KPH_TO_MS

  def receive_speed_limit(self, mux, psd_06):
    if mux == 2:
      if (self.v_limit_receive and
          psd_06["PSD_Ges_Typ"] == 1 and
          psd_06["PSD_Ges_Gesetzlich_Kategorie"] == 0):

        raw_speed = psd_06["PSD_Ges_Geschwindigkeit"]
        if 0 < raw_speed < 11:
          self.v_limit = (raw_speed - 1) * 5
        elif 11 <= raw_speed < 23:
          self.v_limit = 50 + (raw_speed - 11) * 10
        else:
          self.v_limit = 0

        self.v_limit *= self.v_limit_speed_factor
        self.v_limit_receive = False

    elif mux == 0:
      unit = psd_06["PSD_Sys_Geschwindigkeit_Einheit"]
      self.v_limit_speed_factor = CV.MPH_TO_MS if unit == 1 else CV.KPH_TO_MS if unit == 0 else 0
      self.v_limit_receive = psd_06["PSD_Sys_Quali_Tempolimits"] == 7

  def receive_speed_limit_legal(self, mux, psd_06):
    # TODO
    self.v_limit = self.v_limit

  def update(self, cp):
    if not self.CP.flags & VolkswagenFlags.MEB:
      return

    psd_06 = cp.vl["PSD_06"]
    mux = psd_06["PSD_06_Mux"]

    receive_speed_limit_fusion(mux, psd_06) # try reading speed from car camera + navigation fusion
    if self.v_limit == SPEED_LIMIT_NOT_SET:
      receive_speed_limit_legal(mux, psd_06) # try reading speed from legal limit by current street type
    
  def get_speed_limit(self):
    return self.v_limit
