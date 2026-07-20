from types import SimpleNamespace

import pytest

from opendbc.car.volkswagen.speed_limit_manager import BendPreviewReason, MapConfidence, SpeedLimitManager


NOW = 1_000.0


def psd_04(segment_id, previous_id, length, curvature_begin=0, curvature_end=0, sign=0,
           street_category=2, ramp=0):
  return {
    "PSD_ADAS_Qualitaet": 1,
    "PSD_wahrscheinlichster_Pfad": 1,
    "PSD_Segment_ID": segment_id,
    "PSD_Segmentlaenge": length,
    "PSD_Anfangskruemmung": curvature_begin,
    "PSD_Anfangskruemmung_Vorz": sign,
    "PSD_Endkruemmung": curvature_end,
    "PSD_Endkruemmung_Vorz": sign,
    "PSD_Strassenkategorie": street_category,
    "PSD_Bebauung": 0,
    "PSD_Rampe": ramp,
    "PSD_Vorgaenger_Segment_ID": previous_id,
  }


def psd_05(segment_id=1, remaining=40.0, unique=1, error_class=1):
  return {
    "PSD_Pos_Standort_Eindeutig": unique,
    "PSD_Pos_Fehler_Laengsrichtung": error_class,
    "PSD_Pos_Segment_ID": segment_id,
    "PSD_Pos_Segmentlaenge": remaining,
  }


def psd_06(map_match_quality=2, geometry_quality=3):
  return {
    "PSD_06_Mux": 0,
    "PSD_Sys_Segment_ID": 2,
    "PSD_Sys_Geschwindigkeit_Einheit": 0,
    "PSD_Sys_Mapmatchingguete": map_match_quality,
    "PSD_Sys_Geometrieguete": geometry_quality,
  }


def make_manager(monkeypatch):
  monkeypatch.setattr("opendbc.car.volkswagen.speed_limit_manager.time.time", lambda: NOW)
  return SpeedLimitManager(SimpleNamespace(flags=0))


def add_segment(manager, segment, current, *, unique=1, error_class=1, map_match_quality=2, geometry_quality=3):
  manager.update(30.0, segment, psd_05(remaining=current, unique=unique, error_class=error_class),
                 psd_06(map_match_quality, geometry_quality), {}, False, {})


def manager_with_unique_path(monkeypatch, current_remaining=40.0, intermediate_lengths=(), bend_length=25.0,
                             bend_end_curvature=0.004, bend_sign=0, map_match_quality=2, geometry_quality=3):
  manager = make_manager(monkeypatch)
  add_segment(manager, psd_04(1, 0, 100.0), current_remaining, map_match_quality=map_match_quality, geometry_quality=geometry_quality)

  previous_id = 1
  for offset, length in enumerate(intermediate_lengths, start=2):
    add_segment(manager, psd_04(offset, previous_id, length), current_remaining, map_match_quality=map_match_quality, geometry_quality=geometry_quality)
    previous_id = offset

  raw_curvature = int(round(255 - bend_end_curvature / 2e-5))
  add_segment(manager, psd_04(previous_id + 1, previous_id, bend_length, curvature_end=raw_curvature, sign=bend_sign), current_remaining,
              map_match_quality=map_match_quality, geometry_quality=geometry_quality)
  return manager


def build_preview_for_location(monkeypatch, unique, error_class):
  manager = manager_with_unique_path(monkeypatch)
  manager._receive_current_segment_psd(psd_05(unique=unique, error_class=error_class))
  return manager.get_bend_preview(current_speed_ms=30.0)


def test_distance_is_current_remainder_plus_intermediate_segments_plus_curve_offset(monkeypatch):
  # 40 m remain in current segment, then 60 m intermediate, then a bend
  # whose critical endpoint is 25 m into the next segment.
  manager = manager_with_unique_path(
    monkeypatch,
    current_remaining=40.0,
    intermediate_lengths=[60.0],
    bend_length=25.0,
    bend_end_curvature=0.004,
  )
  preview = manager.get_bend_preview(current_speed_ms=30.0)
  assert preview.valid
  assert preview.distance == pytest.approx(125.0)


@pytest.mark.parametrize(
  ("unique", "error_class", "valid", "confidence", "reason"),
  [
    (1, 1, True, MapConfidence.high, BendPreviewReason.none),
    (1, 2, True, MapConfidence.high, BendPreviewReason.none),
    (1, 3, True, MapConfidence.medium, BendPreviewReason.none),
    (1, 4, True, MapConfidence.medium, BendPreviewReason.none),
    (0, 2, False, MapConfidence.none, BendPreviewReason.ambiguousLocation),
    (1, 0, False, MapConfidence.none, BendPreviewReason.locationError),
    (1, 5, False, MapConfidence.none, BendPreviewReason.locationError),
    (1, 7, False, MapConfidence.none, BendPreviewReason.locationError),
  ],
)
def test_location_quality_gate(monkeypatch, unique, error_class, valid, confidence, reason):
  preview = build_preview_for_location(monkeypatch, unique=unique, error_class=error_class)
  assert preview.valid is valid
  assert preview.map_confidence == confidence
  assert preview.rejection_reason == reason


def test_branching_successors_are_rejected(monkeypatch):
  manager = make_manager(monkeypatch)
  add_segment(manager, psd_04(1, 0, 100.0), 40.0)
  add_segment(manager, psd_04(2, 1, 25.0, curvature_end=55), 40.0)
  add_segment(manager, psd_04(3, 1, 25.0, curvature_end=55), 40.0)

  result = manager.get_bend_preview(current_speed_ms=30.0)

  assert not result.valid
  assert result.rejection_reason == BendPreviewReason.ambiguousPath


def test_ten_second_stale_segment_is_rejected(monkeypatch):
  manager = manager_with_unique_path(monkeypatch)
  manager.predicative_segments[2]["Timestamp"] = NOW - 10.1

  result = manager.get_bend_preview(current_speed_ms=30.0)

  assert not result.valid
  assert result.rejection_reason == BendPreviewReason.staleSegment


def test_signed_curvature_and_raw_quality_are_preserved(monkeypatch):
  manager = manager_with_unique_path(monkeypatch, bend_end_curvature=0.004, bend_sign=1,
                                     map_match_quality=2, geometry_quality=3)

  result = manager.get_bend_preview(current_speed_ms=30.0)

  assert result.curvature < 0.0
  assert result.map_match_quality == 2
  assert result.geometry_quality == 3


def test_missing_curvature_is_rejected(monkeypatch):
  manager = make_manager(monkeypatch)
  add_segment(manager, psd_04(1, 0, 100.0), 40.0)
  add_segment(manager, psd_04(2, 1, 25.0), 40.0)

  result = manager.get_bend_preview(current_speed_ms=30.0)

  assert not result.valid
  assert result.rejection_reason == BendPreviewReason.invalidCurvature


@pytest.mark.parametrize("current_remaining", [-25.0, 0.0])
def test_nonpositive_curve_distance_is_rejected(monkeypatch, current_remaining):
  manager = manager_with_unique_path(monkeypatch, current_remaining=current_remaining, bend_length=0.0)

  result = manager.get_bend_preview(current_speed_ms=30.0)

  assert not result.valid
  assert result.rejection_reason == BendPreviewReason.invalidDistance


@pytest.mark.parametrize(
  ("street_category", "ramp", "output_limit_kph"),
  [
    (2, 1, 0.0),
    (1, 0, 0.0),
    (2, 0, 300.0),
  ],
)
def test_ramp_street_and_gross_sanity_filters_are_rejected(monkeypatch, street_category, ramp, output_limit_kph):
  manager = make_manager(monkeypatch)
  add_segment(manager, psd_04(1, 0, 100.0), 40.0)
  add_segment(manager, psd_04(2, 1, 25.0, curvature_end=1, street_category=street_category, ramp=ramp), 40.0)
  manager.v_limit_output_last = output_limit_kph

  result = manager.get_bend_preview(current_speed_ms=30.0)

  assert not result.valid
  assert result.rejection_reason == BendPreviewReason.sanityFilter


def test_no_psd_source_is_rejected(monkeypatch):
  result = make_manager(monkeypatch).get_bend_preview(current_speed_ms=30.0)

  assert not result.valid
  assert result.rejection_reason == BendPreviewReason.sourceUnavailable
