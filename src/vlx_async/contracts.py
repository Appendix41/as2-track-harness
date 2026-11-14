"""Explicit observation, geometry and slow-planner contracts. SI units."""
from dataclasses import dataclass, field
import math
from typing import Optional


def number(v):
    return type(v) in (int, float) and math.isfinite(v)


def points(value, *, count=None, minimum=2, maximum=64):
    if not isinstance(value, (list, tuple)) or not minimum <= len(value) <= maximum:
        raise ValueError("invalid_waypoint_count")
    if count is not None and len(value) != count:
        raise ValueError("invalid_waypoint_count")
    result = []
    for p in value:
        if not isinstance(p, (list, tuple)) or len(p) != 3 or not all(number(v) for v in p):
            raise ValueError("invalid_waypoint_values")
        if abs(p[0]) > 50 or abs(p[1]) > 50 or abs(p[2]) > math.pi:
            raise ValueError("waypoint_out_of_bounds")
        result.append(tuple(float(v) for v in p))
    return tuple(result)


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float
    timestamp_ns: int
    frame_id: str

    def __post_init__(self):
        if not all(number(v) for v in (self.x, self.y, self.yaw)):
            raise ValueError("invalid_pose")
        if type(self.timestamp_ns) is not int or self.timestamp_ns <= 0 or not self.frame_id:
            raise ValueError("invalid_pose_stamp")

    @classmethod
    def parse(cls, value):
        return cls(**value) if value is not None else None


@dataclass(frozen=True)
class Context:
    pose: Optional[Pose] = None
    obstacles: tuple = ()  # (x, y, radius), in current VLX motion frame
    geometry_verified: bool = False
    scene_timestamp_ns: int = 0
    target_id: Optional[str] = None
    target_timestamp_ns: int = 0
    target_local: Optional[tuple] = None
    local_candidates: tuple = ()  # LocalCandidate records from a fast verifier/tracker
    candidates_complete: bool = False
    observed_free_cells: tuple = ()  # directly measured floor cells, not inferred empty space
    grid_resolution_m: float = .10
    coverage_verified: bool = False
    time_sync_verified: bool = False
    body_transform_verified: bool = False
    target_source: str = 'slow_model'


@dataclass(frozen=True)
class LocalCandidate:
    """A fast local detector/tracker candidate, optionally depth-ranked."""
    candidate_id: str
    kind: str
    attributes: tuple
    bbox: tuple
    timestamp_ns: int
    range_m: Optional[float] = None
    track_age_s: float = 0.0
    local_xy: Optional[tuple] = None
    semantic_verified: bool = False
    verifier_source: str = ""
    reference_request_id: Optional[str] = None
    identity_verified: bool = False

    def __post_init__(self):
        if not isinstance(self.candidate_id, str) or not self.candidate_id or not isinstance(self.kind, str) or not self.kind:
            raise ValueError("invalid_local_candidate_identity")
        if not isinstance(self.attributes, tuple) or any(not isinstance(v, str) or not v.strip() for v in self.attributes):
            raise ValueError('invalid_local_candidate_attributes')
        if type(self.semantic_verified) is not bool or type(self.identity_verified) is not bool:
            raise ValueError('invalid_local_candidate_verification')
        if not isinstance(self.verifier_source, str) or (self.reference_request_id is not None and not isinstance(self.reference_request_id, str)):
            raise ValueError('invalid_local_candidate_provenance')
        if type(self.timestamp_ns) is not int or self.timestamp_ns <= 0:
            raise ValueError("invalid_local_candidate_timestamp")
        if len(self.bbox) != 4 or not all(number(v) and 0 <= v <= 1 for v in self.bbox):
            raise ValueError("invalid_local_candidate_bbox")
        if self.bbox[2] < self.bbox[0] or self.bbox[3] < self.bbox[1]:
            raise ValueError("invalid_local_candidate_bbox_order")
        if self.range_m is not None and (not number(self.range_m) or self.range_m < 0):
            raise ValueError("invalid_local_candidate_range")
        if self.local_xy is not None and (len(self.local_xy) != 2 or not all(number(v) for v in self.local_xy)):
            raise ValueError("invalid_local_candidate_local_xy")
        if not number(self.track_age_s) or self.track_age_s < 0:
            raise ValueError("invalid_local_candidate_age")


@dataclass(frozen=True)
class SemanticAnchor:
    """Slow semantic result that can outlive its source image after local revalidation."""
    request: "Request"
    completed_ns: int
    anchor_kind: str
    target_label: str
    attributes: tuple
    selection_rule: str
    bbox_valid: bool
    reference_bbox: tuple
    confidence: float
    target_id: Optional[str]
    dynamic_target: bool
    provider: str
    reason: str = ""

    def __post_init__(self):
        if self.anchor_kind not in {"target", "destination", "none"}:
            raise ValueError("invalid_anchor_kind")
        if self.selection_rule not in {"attributes", "farthest", "nearest",
                                      "leftmost", "rightmost", "only_match", "none"}:
            raise ValueError("invalid_anchor_selection_rule")
        if not isinstance(self.target_label, str) or len(self.target_label) > 200:
            raise ValueError("invalid_anchor_target_label")
        if not isinstance(self.attributes, tuple) or len(self.attributes) > 12:
            raise ValueError("invalid_anchor_attributes")
        if not all(isinstance(v, str) and 0 < len(v) <= 80 for v in self.attributes):
            raise ValueError("invalid_anchor_attributes")
        if type(self.bbox_valid) is not bool or len(self.reference_bbox) != 4:
            raise ValueError("invalid_anchor_bbox")
        if not all(number(v) and 0 <= v <= 1 for v in self.reference_bbox):
            raise ValueError("invalid_anchor_bbox")
        if self.reference_bbox[2] < self.reference_bbox[0] or self.reference_bbox[3] < self.reference_bbox[1]:
            raise ValueError("invalid_anchor_bbox_order")
        if not number(self.confidence) or not 0 <= self.confidence <= 1:
            raise ValueError("invalid_anchor_confidence")
        if not isinstance(self.dynamic_target, bool) or not isinstance(self.provider, str):
            raise ValueError("invalid_anchor_metadata")
        if type(self.completed_ns) is not int or self.completed_ns < self.request.sent_ns:
            raise ValueError("invalid_anchor_completion_time")


@dataclass(frozen=True)
class Observation:
    stream_id: str
    boot_id: str
    revision: int
    instruction: str
    prediction_index: int
    camera_index: int
    camera_ns: int
    prediction_ns: int
    waypoints: tuple
    context: Context = field(default_factory=Context)
    image_base64: Optional[str] = field(default=None, repr=False)

    @property
    def task_key(self):
        return (self.boot_id, self.stream_id, self.revision, self.instruction)

    @property
    def order(self):
        return (self.camera_ns, self.prediction_index)

    @classmethod
    def from_state(cls, state, context=None):
        r = state["record"]
        return cls(state["stream_id"], state["host_boot_id"], r["instruction_revision"],
                   r["instruction"], r["prediction_index"], r["camera_frame_index"],
                   r["camera_received_monotonic_ns"], r["prediction_available_monotonic_ns"],
                   points(r["waypoints"], count=8), context or Context(), state.get("image_base64"))


@dataclass(frozen=True)
class Request:
    request_id: str
    observation: Observation
    sent_ns: int


@dataclass(frozen=True)
class SlowPlan:
    request: Request
    completed_ns: int
    coordinate_frame: str  # observation_local_m or image_uv
    waypoints: tuple
    confidence: float
    target_id: Optional[str]
    dynamic_target: bool
    provider: str
    reason: str = ""
    metric_verified: bool = False  # set only by local geometry, never model claim
    anchor: Optional[SemanticAnchor] = None


@dataclass(frozen=True)
class FusionConfig:
    """Route-level fusion. One DeepSeek plan per task; VLX supplies the path shape."""
    plan_weight: float = 0.85
    min_confidence: float = 0.50
    pose_skew_s: float = 0.10
    footprint_radius_m: float = 0.45
    max_cross_track_m: float = 1.50
    # No rotation cap: a correction may go to +/-180 deg. The turn RATE is
    # still bounded by the executor's yaw limit.
    max_heading_change_rad: float = math.pi
    plan_step_m: float = 0.10
    plan_lookahead_m: float = 2.0
    # 1.0 aims the heading at the route's END (the target); 0.0 uses only the
    # point plan_lookahead_m ahead. Perspective flattens a visible image curve
    # into a few centimetres, so the early lookahead point is nearly straight.
    plan_endpoint_weight: float = 1.0
    plan_max_age_s: float = 900.0

    def __post_init__(self):
        for value in self.__dict__.values():
            if not number(value) or value <= 0:
                raise ValueError("invalid_fusion_config")
        if not 0 < self.plan_weight <= 0.95 or not 0 < self.min_confidence <= 1:
            raise ValueError("invalid_fusion_weight")

    @classmethod
    def from_mapping(cls, data):
        """Accept older config files that still carry removed depth/anchor keys."""
        if not isinstance(data, dict):
            raise ValueError('invalid_fusion_config')
        fields = cls.__dataclass_fields__
        legacy = {'max_slow_weight': 'plan_weight'}
        values = {}
        for key, value in data.items():
            if key in fields:
                values[key] = value
            elif key in legacy:
                values[legacy[key]] = value
        return cls(**values)


@dataclass(frozen=True)
class TaskConfig:
    """One task = one plan, one motion. No reasoning while the robot moves."""
    arrival_distance_m: float = 0.35
    task_timeout_s: float = 120.0
    plan_timeout_s: float = 90.0
    max_range_m: float = 4.0
    max_travel_m: float = 2.0
    max_plan_calls: int = 1

    def __post_init__(self):
        for value in (self.arrival_distance_m, self.task_timeout_s,
                      self.plan_timeout_s, self.max_range_m, self.max_travel_m):
            if not number(value) or value <= 0:
                raise ValueError('invalid_task_config')
        if not 0.05 <= self.arrival_distance_m <= 2. or not 1. <= self.max_range_m <= 15.:
            raise ValueError('task_config_out_of_bounds')
        if not .2 <= self.max_travel_m <= 10.:
            raise ValueError('task_config_out_of_bounds')
        # Exactly one plan per task: no configuration may enable a second
        # reasoning call while the robot is executing.
        if type(self.max_plan_calls) is not int or self.max_plan_calls != 1:
            raise ValueError('one_plan_per_task_only')
