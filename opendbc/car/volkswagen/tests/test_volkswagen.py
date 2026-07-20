import random
import re
import unittest

from opendbc.car import DT_CTRL
from opendbc.car import structs
from opendbc.car.structs import CarParams
from opendbc.car.volkswagen.carcontroller import HCAMitigation
from opendbc.car.volkswagen.carstate import CarState
from opendbc.car.volkswagen.speed_limit_manager import BendPreview, BendPreviewReason, MapConfidence
from opendbc.car.volkswagen.values import CAR, CarControllerParams as CCP, FW_QUERY_CONFIG, WMI
from opendbc.car.volkswagen.fingerprints import FW_VERSIONS

Ecu = CarParams.Ecu

CHASSIS_CODE_PATTERN = re.compile('[A-Z0-9]{2}')
# TODO: determine the unknown groups
SPARE_PART_FW_PATTERN = re.compile(b'\xf1\x87(?P<gateway>[0-9][0-9A-Z]{2})(?P<unknown>[0-9][0-9A-Z][0-9])(?P<unknown2>[0-9A-Z]{2}[0-9])([A-Z0-9]| )')


class TestVolkswagenHCAMitigation(unittest.TestCase):
  STUCK_TORQUE_FRAMES = round(CCP.STEER_TIME_STUCK_TORQUE / (DT_CTRL * CCP.STEER_STEP))

  def test_same_torque_mitigation(self):
    """Same-torque nudge fires at the threshold, in the correct direction, and resets cleanly."""
    hca_mitigation = HCAMitigation(CCP)

    for actuator_value in (-CCP.STEER_MAX, -1, 0, 1, CCP.STEER_MAX):
      hca_mitigation.update(0, 0)  # Reset mitigation state
      for frame in range(self.STUCK_TORQUE_FRAMES + 2):
        should_nudge = actuator_value != 0 and frame == self.STUCK_TORQUE_FRAMES
        expected_torque = actuator_value - (1, -1)[actuator_value < 0] if should_nudge else actuator_value
        assert hca_mitigation.update(actuator_value, actuator_value) == expected_torque, f"{frame=}"


class TestVolkswagenBendPreviewPublication(unittest.TestCase):
  def test_car_state_exposes_and_populates_bend_preview(self):
    ret = structs.CarState()
    ret.cruiseState.speedLimit = 12.3
    preview = BendPreview(
      valid=True,
      curvature=-0.004,
      distance=123.4,
      length=56.7,
      location_error=4,
      map_confidence=MapConfidence.medium,
      map_match_quality=5,
      geometry_quality=6,
      rejection_reason=BendPreviewReason.none,
    )

    self.assertTrue(hasattr(ret.cruiseState, "bendPreview"))
    CarState._publish_bend_preview(ret, preview)

    published = ret.cruiseState.bendPreview
    self.assertTrue(published.valid)
    self.assertAlmostEqual(published.curvature, preview.curvature, places=6)
    self.assertAlmostEqual(published.distance, preview.distance, places=5)
    self.assertAlmostEqual(published.length, preview.length, places=5)
    self.assertEqual(published.locationError, preview.location_error)
    self.assertEqual(published.mapConfidence, preview.map_confidence.name)
    self.assertEqual(published.mapMatchQuality, preview.map_match_quality)
    self.assertEqual(published.geometryQuality, preview.geometry_quality)
    self.assertEqual(published.rejectionReason, preview.rejection_reason.name)
    self.assertAlmostEqual(ret.cruiseState.speedLimit, 12.3, places=5)

  def test_invalid_preview_preserves_replay_diagnostics(self):
    ret = structs.CarState()
    preview = BendPreview(
      location_error=7,
      map_confidence=MapConfidence.none,
      map_match_quality=8,
      geometry_quality=9,
      rejection_reason=BendPreviewReason.locationError,
    )

    CarState._publish_bend_preview(ret, preview)

    published = ret.cruiseState.bendPreview
    self.assertFalse(published.valid)
    self.assertEqual(published.locationError, preview.location_error)
    self.assertEqual(published.mapConfidence, preview.map_confidence.name)
    self.assertEqual(published.mapMatchQuality, preview.map_match_quality)
    self.assertEqual(published.geometryQuality, preview.geometry_quality)
    self.assertEqual(published.rejectionReason, preview.rejection_reason.name)

class TestVolkswagenPlatformConfigs(unittest.TestCase):
  def test_spare_part_fw_pattern(self):
    # Relied on for determining if a FW is likely VW
    for platform, ecus in FW_VERSIONS.items():
      with self.subTest(platform=platform.value):
        for fws in ecus.values():
          for fw in fws:
            assert SPARE_PART_FW_PATTERN.match(fw) is not None, f"Bad FW: {fw}"

  def test_chassis_codes(self):
    for platform in CAR:
      with self.subTest(platform=platform.value):
        assert len(platform.config.wmis) > 0, "WMIs not set"
        assert len(platform.config.chassis_codes) > 0, "Chassis codes not set"
        assert all(CHASSIS_CODE_PATTERN.match(cc) for cc in
                   platform.config.chassis_codes), "Bad chassis codes"

        # No two platforms should share chassis codes
        for comp in CAR:
          if platform == comp:
            continue
          assert set() == platform.config.chassis_codes & comp.config.chassis_codes, \
                           f"Shared chassis codes: {comp}"

  def test_custom_fuzzy_fingerprinting(self):
    all_radar_fw = list({fw for ecus in FW_VERSIONS.values() for fw in ecus[Ecu.fwdRadar, 0x757, None]})

    for platform in CAR:
      with self.subTest(platform=platform.name):
        for wmi in WMI:
          for chassis_code in platform.config.chassis_codes | {"00"}:
            vin = ["0"] * 17
            vin[0:3] = wmi
            vin[6:8] = chassis_code
            vin = "".join(vin)

            # Check a few FW cases - expected, unexpected
            for radar_fw in random.sample(all_radar_fw, 5) + [b'\xf1\x875Q0907572G \xf1\x890571', b'\xf1\x877H9907572AA\xf1\x890396']:
              should_match = ((wmi in platform.config.wmis and chassis_code in platform.config.chassis_codes) and
                              radar_fw in all_radar_fw)

              live_fws = {(0x757, None): [radar_fw]}
              matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fws, vin, FW_VERSIONS)

              expected_matches = {platform} if should_match else set()
              assert expected_matches == matches, "Bad match"
