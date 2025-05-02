import time

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volkswagen.values import VolkswagenFlags

NOT_SET = 0
SPEED_SUGGESTED_MAX_HIGHWAY_GER_KPH = 130 # 130 kph in germany
STREET_TYPE_URBAN = 1
STREET_TYPE_NONURBAN = 2
STREET_TYPE_HIGHWAY = 3
SANITY_CHECK_DIFF_PERCENT_LOWER = 30
SPEED_LIMIT_UNLIMITED_VZE_MS = 144
ACCELERATION_PREDICATIVE = 2
SEGMENT_DECAY = 60


class SpeedLimitManager:
  def __init__(self, car_params, speed_limit_max_kph=SPEED_SUGGESTED_MAX_HIGHWAY_GER_KPH, predicative=False):
    self.CP = car_params
    self.v_limit_psd = NOT_SET
    self.v_limit_psd_next = NOT_SET
    self.v_limit_psd_legal = NOT_SET
    self.v_limit_vze = NOT_SET
    self.v_limit_receive = False
    self.v_limit_receive_segment_id = None
    self.v_limit_speed_factor = CV.KPH_TO_MS
    self.v_limit_vze_sanity_error = False
    self.v_limit_output_last = NOT_SET
    self.v_limit_max = speed_limit_max_kph
    self.predicative = predicative
    self.predicative_segments = {}
    self.current_predicative_segment = {"ID": NOT_SET, "Length": NOT_SET, "Speed": NOT_SET, "StreetType": NOT_SET}

  def update(self, current_speed_ms, psd_04, psd_05, psd_06, vze):
    # try reading speed form traffic sign recognition
    if vze and self.CP.flags & VolkswagenFlags.MEB:
      self._receive_speed_limit_vze_meb(vze)
    
    # try reading speed from predicative street data
    if psd_04 and psd_05 and psd_06:
      self._receive_speed_factor_psd(psd_06)
      self._receive_current_segment_psd(psd_05)
      self._refresh_current_segment()
      self._receive_speed_limit_permission(psd_06)
      self._build_predicative_segments(psd_04, psd_06)
      self._receive_speed_limit_psd_legal(psd_06)
      self._get_speed_limit_psd()
      self._get_speed_limit_psd_next(current_speed_ms)
    
  def get_speed_limit(self):
    if (self.predicative == True and self.v_limit_psd_next != NOT_SET and self.v_limit_psd_next < self.v_limit_output_last):
      v_limit_output = self.v_limit_psd_next
    elif (self.v_limit_vze != NOT_SET and self.v_limit_vze_sanity_error != True):
      v_limit_output = self.v_limit_vze
    elif self.v_limit_psd != NOT_SET:
      v_limit_output = self.v_limit_psd
    else:
      v_limit_output = self.v_limit_psd_legal

    if v_limit_output > self.v_limit_max:
      v_limit_output = self.v_limit_max
    
    self.v_limit_vze_sanity_error = False
    self.v_limit_output_last = v_limit_output
    
    return v_limit_output

  def _speed_limit_vze_sanitiy_check(self, speed_limit_vze_new):
    if self.v_limit_output_last == NOT_SET:
      return
      
    diff_p = 100 * speed_limit_vze_new / self.v_limit_output_last
    self.v_limit_vze_sanity_error = True if diff_p < SANITY_CHECK_DIFF_PERCENT_LOWER else False
    if speed_limit_vze_new > SPEED_LIMIT_UNLIMITED_VZE_MS: # unlimited sign detected: use psd logic for setting maximum speed
      self.v_limit_vze_sanity_error = True

  def _receive_speed_factor_psd(self, psd_06):
    if psd_06["PSD_06_Mux"] == 0:
      unit = psd_06["PSD_Sys_Geschwindigkeit_Einheit"]
      self.v_limit_speed_factor = CV.MPH_TO_MS if unit == 1 else CV.KPH_TO_MS if unit == 0 else 0

  def _receive_speed_limit_permission(self, psd_06):
    if psd_06["PSD_06_Mux"] == 0:
      permission = psd_06["PSD_Sys_Quali_Tempolimits"] == 7
      self.v_limit_receive = permission
      self.v_limit_receive_segment_id = psd_06["PSD_Sys_Segment_ID"] if permission else NOT_SET

  def _convert_raw_speed_psd(self, raw_speed, street_type):
    if 0 < raw_speed < 11: # 0 - 45 kph
      speed = (raw_speed - 1) * 5
    elif 11 <= raw_speed < 23: # 50 - 160 kph
      speed = 50 + (raw_speed - 11) * 10
    elif raw_speed == 23: # explicitly no legal speed limit 
      if street_type == STREET_TYPE_HIGHWAY:
        speed = self.v_limit_max
      else:
        speed = NOT_SET
    else:
      speed = NOT_SET
      
    return speed * self.v_limit_speed_factor

  def _receive_speed_limit_vze_meb(self, vze):
    if vze["VZE_Verkehrszeichen_1_Typ"] == 0:
      v_limit_vze = int(round(vze["VZE_Verkehrszeichen_1"]))# main traffic sign
      if vze["VZE_Verkehrszeichen_Einheit"] == 1:
        v_limit_vze *= CV.KPH_TO_MS
      else:
        v_limit_vze *= CV.MPH_TO_MS
      self._speed_limit_vze_sanitiy_check(v_limit_vze)
      self.v_limit_vze = v_limit_vze

  def _receive_current_segment_psd(self, psd_05):
    if psd_05["PSD_Pos_Standort_Eindeutig"] == 1:
      self.current_predicative_segment["ID"] = psd_05["PSD_Pos_Segment_ID"]
      self.current_predicative_segment["Length"] = psd_05["PSD_Pos_Segmentlaenge"]

  def _refresh_current_segment(self):
    current_segment = self.current_predicative_segment["ID"]
    if current_segment != NOT_SET:
      if current_segment in self.predicative_segments:
        self.current_predicative_segment["Speed"] = self.predicative_segments[current_segment]["Speed"]
        self.current_predicative_segment["StreetType"] = self.predicative_segments[current_segment]["StreetType"]

  def _build_predicative_segments(self, psd_04, psd_06):
    now = time.time()
    
    # Schritt 1: Segment erfassen/aktualisieren
    if (psd_04["PSD_ADAS_Qualitaet"] == 1 and
        psd_04["PSD_wahrscheinlichster_Pfad"] == 1 and
        psd_04["PSD_Segment_ID"] != NOT_SET):

      segment_id = psd_04["PSD_Segment_ID"]
      seg = self.predicative_segments.get(segment_id)
      if seg:
        seg["Length"] = psd_04["PSD_Segmentlaenge"]
        seg["StreetType"] = self._get_street_type(psd_04["PSD_Strassenkategorie"], psd_04["PSD_Bebauung"])
        seg["ID_Prev"] = psd_04["PSD_Vorgaenger_Segment_ID"]
        seg["Timestamp"] = now
      else:
        self.predicative_segments[segment_id] = {
          "ID": segment_id,
          "Length": psd_04["PSD_Segmentlaenge"],
          "StreetType": self._get_street_type(psd_04["PSD_Strassenkategorie"], psd_04["PSD_Bebauung"]),
          "ID_Prev": psd_04["PSD_Vorgaenger_Segment_ID"],
          "Speed": NOT_SET,
          "QualityFlag": False,
          "Timestamp": now
        }

    # Schritt 2: Alte Segmente bereinigen
    current_id = self.current_predicative_segment["ID"]
    if current_id != NOT_SET:
      valid_ids = set()
      seg_id = current_id
      while seg_id in self.predicative_segments:
        seg = self.predicative_segments[seg_id]
        if now - seg.get("Timestamp", 0) > SEGMENT_DECAY:
          break  # Segment ist zu alt – Abbruch der Kette
        valid_ids.add(seg_id)
        seg_id = seg.get("ID_Prev")
        if seg_id is None or seg_id in valid_ids:
          break
      self.predicative_segments = {
        sid: seg for sid, seg in self.predicative_segments.items()
        if sid in valid_ids
      }

    # Schritt 3: Geschwindigkeit setzen
    if (self.v_limit_receive and
        psd_06["PSD_06_Mux"] == 2 and
        psd_06["PSD_Ges_Typ"] == 1 and
        psd_06["PSD_Ges_Gesetzlich_Kategorie"] == 0):

      raw_speed = psd_06["PSD_Ges_Geschwindigkeit"]
      segment_id = self.v_limit_receive_segment_id

      if segment_id in self.predicative_segments:
        speed = self._convert_raw_speed_psd(raw_speed, self.predicative_segments[segment_id]["StreetType"])
        self.predicative_segments[segment_id]["Speed"] = speed
        self.predicative_segments[segment_id]["QualityFlag"] = True

      self.v_limit_receive = False
      self.v_limit_receive_segment_id = NOT_SET

  def _get_speed_limit_psd_next(self, current_speed_ms):
    """
    Setzt self.v_limit_psd_next, wenn ein zukünftiges Speed Limit rechtzeitig erreichbar ist.
    """
    current_id = self.current_predicative_segment.get("ID")
    length_remaining = self.current_predicative_segment.get("Length")
    self.v_limit_psd_next = NOT_SET

    if current_id == NOT_SET or length_remaining == NOT_SET:
      return

    current_speed_kmh = int(round(current_speed_ms * CV.MS_TO_KPH))
    total_dist = length_remaining
    visited = set()
    seg_id = current_id

    while seg_id and seg_id not in visited:
      visited.add(seg_id)

      # Finde nächstes Segment in Fahrtrichtung (ID_Prev == aktuelles Segment)
      next_id = None
      for candidate_id, seg in self.predicative_segments.items():
        if seg.get("ID_Prev") == seg_id:
          next_id = candidate_id
          break

      if not next_id or next_id in visited:
        break

      next_seg = self.predicative_segments.get(next_id)
      if not next_seg:
        break

      if not next_seg.get("QualityFlag", False):
        break

      next_speed_kmh = next_seg.get("Speed")
      
      if next_speed_kmh == current_speed_kmh:
        total_dist += next_seg.get("Length", 0)
        seg_id = next_id
        continue
      
      if next_speed_kmh != NOT_SET and next_speed_kmh != current_speed_kmh:
        target_speed = next_speed_kmh * CV.KPH_TO_MS
        delta_v = abs(current_speed_ms - target_speed)
        braking_distance = (delta_v ** 2) / (2 * ACCELERATION_PREDICATIVE)

        if total_dist <= braking_distance:
          self.v_limit_psd_next = next_speed_kmh
        return

      total_dist += next_seg.get("Length", 0)
      seg_id = next_id

  def _get_speed_limit_psd(self):
    seg_id = self.current_predicative_segment.get("ID")
    if seg_id == NOT_SET:
      self.v_limit_psd = NOT_SET
      return

    seg = self.predicative_segments.get(seg_id)
    if seg and seg.get("Speed") != NOT_SET:
      self.v_limit_psd = seg.get("Speed")

  def _get_street_type(self, strassenkategorie, bebauung):
    street_type = NOT_SET
    
    if strassenkategorie == 1: # base type: urban
      street_type = STREET_TYPE_URBAN
          
    elif strassenkategorie in (2, 3, 4): # base type: non urban
      if bebauung == 1:
        street_type = STREET_TYPE_URBAN
      else:
        street_type = STREET_TYPE_NONURBAN
        
    elif strassenkategorie == 5: # base type: highway
      street_type = STREET_TYPE_HIGHWAY

    return street_type

  def _receive_speed_limit_psd_legal(self, psd_06):
    if psd_06["PSD_06_Mux"] == 2:
      if psd_06["PSD_Ges_Typ"] == 2:
        street_type = self.current_predicative_segment["StreetType"]
        if ((psd_06["PSD_Ges_Gesetzlich_Kategorie"] == STREET_TYPE_URBAN    and street_type == STREET_TYPE_URBAN) or
            (psd_06["PSD_Ges_Gesetzlich_Kategorie"] == STREET_TYPE_NONURBAN and street_type == STREET_TYPE_NONURBAN) or
            (psd_06["PSD_Ges_Gesetzlich_Kategorie"] == STREET_TYPE_HIGHWAY  and street_type == STREET_TYPE_HIGHWAY)):
          raw_speed = psd_06["PSD_Ges_Geschwindigkeit"]
          self.v_limit_psd_legal = self._convert_raw_speed_psd(raw_speed, street_type)
