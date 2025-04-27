from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volkswagen.values import VolkswagenFlags

SPEED_LIMIT_NOT_SET = 0
SPEED_SUGGESTED_MAX_HIGHWAY_KPH = 120 # 130 kph in germany, my preference is 120 kph
STREET_TYPE_NOT_SET = 0
STREET_TYPE_URBAN = 1
STREET_TYPE_NONURBAN = 2
STREET_TYPE_HIGHWAY = 3
SANITY_CHECK_DIFF_PERCENT_LOWER = 30
SPEED_LIMIT_UNLIMITED_VZE_MS = 144


class SpeedLimitManager:
  def __init__(self, car_params):
    self.CP = car_params
    self.v_limit_psd = SPEED_LIMIT_NOT_SET
    self.v_limit_psd_legal = SPEED_LIMIT_NOT_SET
    self.v_limit_vze = SPEED_LIMIT_NOT_SET
    self.v_limit_receive = False
    self.v_limit_speed_factor = CV.KPH_TO_MS
    self.street_type = STREET_TYPE_NOT_SET
    self.v_limit_vze_sanity_error = False
    self.v_limit_output_last = SPEED_LIMIT_NOT_SET

  def update(self, psd_06, psd_04, vze):
    if not self.CP.flags & VolkswagenFlags.MEB:
      return

    # try reading speed form traffic sign recognition
    self._receive_speed_limit_vze(vze)
    
    # try reading speed from predicative street data
    mux = psd_06["PSD_06_Mux"]
    self._receive_speed_limit_permission(mux, psd_06)
    self._receive_speed_factor(mux, psd_06)
    self._receive_speed_limit_psd(mux, psd_06)
    
    # try reading speed from legal limit by current street type from psd
    if psd_04:
      self._receive_street_type(psd_04)
      self._receive_speed_limit_psd_legal(mux, psd_06)
    
  def get_speed_limit(self):
    if (self.v_limit_vze != SPEED_LIMIT_NOT_SET and self.v_limit_vze_sanity_error != True):
      v_limit_output = self.v_limit_vze
    elif self.v_limit_psd != SPEED_LIMIT_NOT_SET:
      v_limit_output = self.v_limit_psd
    else:
      v_limit_output = self.v_limit_psd_legal

    self.v_limit_vze_sanity_error = False
    self.v_limit_output_last = v_limit_output
    
    return v_limit_output

  def _speed_limit_vze_sanitiy_check(self, speed_limit_vze_new):
    if self.v_limit_output_last == SPEED_LIMIT_NOT_SET:
      return
      
    diff_p = 100 * speed_limit_vze_new / self.v_limit_output_last
    self.v_limit_vze_sanity_error = True if diff_p < SANITY_CHECK_DIFF_PERCENT_LOWER else False
    if speed_limit_vze_new > SPEED_LIMIT_UNLIMITED_VZE_MS: # unlimited sign detected: use psd logic for setting maximum speed
      self.v_limit_vze_sanity_error = True

  def _receive_speed_factor(self, mux, psd_06):
    if mux == 0:
      unit = psd_06["PSD_Sys_Geschwindigkeit_Einheit"]
      self.v_limit_speed_factor = CV.MPH_TO_MS if unit == 1 else CV.KPH_TO_MS if unit == 0 else 0

  def _receive_speed_limit_permission(self, mux, psd_06):
    if mux == 0:
      self.v_limit_receive = psd_06["PSD_Sys_Quali_Tempolimits"] == 7

  def _convert_raw_speed_psd(self, raw_speed):
    if 0 < raw_speed < 11: # 0 - 45 kph
      speed = (raw_speed - 1) * 5
    elif 11 <= raw_speed < 23: # 50 - 160 kph
      speed = 50 + (raw_speed - 11) * 10
    elif raw_speed == 23: # explicitly no legal speed limit 
      if self.street_type == STREET_TYPE_HIGHWAY:
        speed = SPEED_SUGGESTED_MAX_HIGHWAY_KPH
      else:
        speed = SPEED_LIMIT_NOT_SET
    else:
      speed = SPEED_LIMIT_NOT_SET
      
    return speed * self.v_limit_speed_factor

  def _receive_speed_limit_vze(self, vze):
    if vze["VZE_Verkehrszeichen_1_Typ"] == 0:
      v_limit_vze = int(round(vze["VZE_Verkehrszeichen_1"]))# main traffic sign
      if vze["VZE_Verkehrszeichen_Einheit"] == 1:
        v_limit_vze *= CV.KPH_TO_MS
      else:
        v_limit_vze *= CV.MPH_TO_MS
      self._speed_limit_vze_sanitiy_check(v_limit_vze)
      self.v_limit_vze = v_limit_vze

  def _receive_speed_limit_psd(self, mux, psd_06):
    if mux == 2:
      if (self.v_limit_receive and
          psd_06["PSD_Ges_Typ"] == 1 and
          psd_06["PSD_Ges_Gesetzlich_Kategorie"] == 0):

        raw_speed = psd_06["PSD_Ges_Geschwindigkeit"]
        self.v_limit_psd = self._convert_raw_speed_psd(raw_speed)

        self.v_limit_receive = False

  def _receive_street_type(self, psd_04):
    if psd_04["PSD_Segment_Komplett"] == 1 and psd_04["PSD_wahrscheinlichster_Pfad"] == 1:
      if psd_04["PSD_Strassenkategorie"] == 1: # base type: urban
        self.street_type = STREET_TYPE_URBAN
          
      elif psd_04["PSD_Strassenkategorie"] in (2, 3, 4): # base type: non urban
        if psd_04["PSD_Bebauung"] == 1:
          self.street_type = STREET_TYPE_URBAN
        else:
          self.street_type = STREET_TYPE_NONURBAN
        
      elif psd_04["PSD_Strassenkategorie"] == 5: # base type: highway
        self.street_type = STREET_TYPE_HIGHWAY

  def _receive_speed_limit_psd_legal(self, mux, psd_06):
    if mux == 2:
      if psd_06["PSD_Ges_Typ"] == 2:
        if ((psd_06["PSD_Ges_Gesetzlich_Kategorie"] == STREET_TYPE_URBAN    and self.street_type == STREET_TYPE_URBAN) or
            (psd_06["PSD_Ges_Gesetzlich_Kategorie"] == STREET_TYPE_NONURBAN and self.street_type == STREET_TYPE_NONURBAN) or
            (psd_06["PSD_Ges_Gesetzlich_Kategorie"] == STREET_TYPE_HIGHWAY  and self.street_type == STREET_TYPE_HIGHWAY)):
          raw_speed = psd_06["PSD_Ges_Geschwindigkeit"]
          self.v_limit_psd_legal = self._convert_raw_speed_psd(raw_speed)
