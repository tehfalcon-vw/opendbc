from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volkswagen.values import VolkswagenFlags

SPEED_LIMIT_NOT_SET = 0
STREET_TYPE_NOT_SET = 0
STREET_TYPE_URBAN = 1
STREET_TYPE_NONURBAN = 2
STREET_TYPE_HIGHWAY = 3


class SpeedLimitManager:
  def __init__(self, car_params):
    self.CP = car_params
    self.v_limit = SPEED_LIMIT_NOT_SET
    self.v_limit_legal = SPEED_LIMIT_NOT_SET
    self.v_limit_receive = False
    self.v_limit_speed_factor = CV.KPH_TO_MS
    self.street_type = STREET_TYPE_NOT_SET

  def receive_speed_factor(self, mux, psd_06):
    if mux == 0:
      unit = psd_06["PSD_Sys_Geschwindigkeit_Einheit"]
      self.v_limit_speed_factor = CV.MPH_TO_MS if unit == 1 else CV.KPH_TO_MS if unit == 0 else 0

  def receive_speed_limit_permission(self, mux, psd_06):
    if mux == 0:
      self.v_limit_receive = psd_06["PSD_Sys_Quali_Tempolimits"] == 7

  def convert_raw_speed(self, raw_speed):
    if 0 < raw_speed < 11:
      speed = (raw_speed - 1) * 5
    elif 11 <= raw_speed < 23:
      speed = 50 + (raw_speed - 11) * 10
    else:
      speed = SPEED_LIMIT_NOT_SET
      
    return speed * self.v_limit_speed_factor

  def receive_speed_limit_fusion(self, mux, psd_06):
    if mux == 2:
      if (self.v_limit_receive and
          psd_06["PSD_Ges_Typ"] == 1 and
          psd_06["PSD_Ges_Gesetzlich_Kategorie"] == 0):

        raw_speed = psd_06["PSD_Ges_Geschwindigkeit"]
        self.v_limit = self.convert_raw_speed(raw_speed)

        self.v_limit_receive = False

  def receive_street_type(self, psd_04):
    if psd_04["PSD_Segment_Komplett"] == 1 and psd_04["PSD_wahrscheinlichster_Pfad"] == 1:
      if psd_04["PSD_Strassenkategorie"] == 1: # base type: urban
        self.street_type = STREET_TYPE_URBAN
          
      elif psd_04["PSD_Strassenkategorie"] in (2, 3, 4): # base type: non urban
        if psd_04["PSD_Bebauung"] == 1:
          self.street_type = STREET_TYPE_URBAN
        else:
          self.street_type = STREET_TYPE_NONURBAN
        
      elif psd_04["PSD_Strassenkategorie"] == 5: # baste type: highway
        self.street_type = STREET_TYPE_HIGHWAY

  def receive_speed_limit_legal(self, mux, psd_06):
    if mux == 2:
      if psd_06["PSD_Ges_Typ"] == 2:
        if ((psd_06["PSD_Ges_Gesetzlich_Kategorie"] == STREET_TYPE_URBAN    and self.street_type == STREET_TYPE_URBAN) or
            (psd_06["PSD_Ges_Gesetzlich_Kategorie"] == STREET_TYPE_NONURBAN and self.street_type == STREET_TYPE_NONURBAN) or
            (psd_06["PSD_Ges_Gesetzlich_Kategorie"] == STREET_TYPE_HIGHWAY  and self.street_type == STREET_TYPE_HIGHWAY)):
          raw_speed = psd_06["PSD_Ges_Geschwindigkeit"]
          self.v_limit_legal = self.convert_raw_speed(raw_speed)

  def update(self, psd_06, psd_04):
    if not self.CP.flags & VolkswagenFlags.MEB:
      return

    mux = psd_06["PSD_06_Mux"]
    
    self.receive_speed_limit_permission(mux, psd_06)
    self.receive_speed_factor(mux, psd_06)
    self.receive_speed_limit_fusion(mux, psd_06) # try reading speed from car camera + navigation fusion
    if psd_04:
      self.receive_street_type(psd_04)
      self.receive_speed_limit_legal(mux, psd_06) # try reading speed from legal limit by current street type
    
  def get_speed_limit(self):
    if self.v_limit == SPEED_LIMIT_NOT_SET:
      return self.v_limit_legal
    else:
      return self.v_limit
