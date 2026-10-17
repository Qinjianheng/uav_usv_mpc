"""P4 nominal FOLLOW proposals and actual rejection ACK recording; no control publisher."""
from dataclasses import replace
from collections import deque
from statistics import median
import math
import time
import secrets
import resource
import threading
from pathlib import Path
from types import SimpleNamespace

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from uav_usv_interfaces.msg import FollowTrajectory, FollowPlanAck, FollowReceiverState

from uav_control.common.record_snapshot import record_snapshot
from uav_control.controllers.follow_research_shadow_node import FollowResearchShadowNode
from uav_control.guidance.follow_limits import ResearchConfig, constraint_snapshot
from uav_control.guidance.follow_profile import CycleProfile, profiled
from uav_control.controllers.follow_mpc_shadow_node import _serialize
from uav_control.controllers.follow_transport import proposal_from_event, seconds
from uav_control.controllers.follow_mpc_shadow_node import (
    ShadowResearchRunner, completion_rejection,
)


class P4ResearchRunner(ShadowResearchRunner):
    """Math hints may survive mere input waiting; never an accepted reference or permission."""

    def _delivery_enabled(self):
        return (getattr(self, 'successor_scheduling', 'periodic') == 'delivery'
                and getattr(self, 'deadline_scheduling', False))

    def _delivery_window(self, now):
        from uav_control.guidance.successor_delivery import DeliveryWindow
        status = getattr(self, 'execution_status', None) or {}
        if not (self._delivery_enabled() and status.get('active')
                and 0 <= now-status['stamp'] <= .2):
            return None
        identity = (getattr(self, 'current_receiver_epoch', None), status['active'])
        window = getattr(self, 'delivery_window', None)
        if window is None or window.identity != identity:
            self.delivery_window = window = DeliveryWindow(identity)
        return window

    def _invalidate_warm(self):
        hint = getattr(self.solver, 'hint', None)
        if (hint is not None and self._follow()
                and hint[0].mission_id == self.mission['mission_id']
                and hint[0].clock_generation == self.adapter.clock_generation):
            return
        super()._invalidate_warm()

    def _solve_request(self, request):
        started = time.monotonic()
        cpu_started = time.thread_time()
        wall = getattr(self.solver, 'wall_clock', None)
        ros_start = wall() if wall else None
        usage = resource.getrusage(resource.RUSAGE_THREAD)
        scheduler_path = Path(f'/proc/self/task/{threading.get_native_id()}/schedstat')
        scheduler_start = tuple(map(int, scheduler_path.read_text().split()))
        result = self.solver.solve(request)
        cpu_ended, ended = time.thread_time(), time.monotonic()
        usage_end = resource.getrusage(resource.RUSAGE_THREAD)
        scheduler_end = tuple(map(int, scheduler_path.read_text().split()))
        timeline = dict(solve_start_monotonic=started, solve_end_monotonic=ended,
                        solve_start_ros=ros_start, solve_end_ros=wall() if wall else None,
                        solve_start_thread_cpu=cpu_started, solve_end_thread_cpu=cpu_ended,
                        worker_thread_cpu_seconds=cpu_ended-cpu_started,
                        scheduler_runqueue_seconds=(scheduler_end[1]-scheduler_start[1])/1e9,
                        voluntary_context_switches=usage_end.ru_nvcsw-usage.ru_nvcsw,
                        involuntary_context_switches=usage_end.ru_nivcsw-usage.ru_nivcsw)
        plan = getattr(request, 'delivery_plan', None)
        if plan is not None:
            actual_path = ('fallback' if result.metrics.get('follow_guided', {})
                           .get('fallback_used') else 'follow')
            self.solver.delivery_costs.observe(
                actual_path, ended-started,
                request.context.observation_stamp, result.valid, result.solver_status,
                cpu=cpu_ended-cpu_started)
            timeline['delivery_path'] = actual_path
            timeline['delivery_planned_path'] = plan.path
            timeline['math_runtime'] = getattr(self, 'math_runtime', None)
        return replace(result, metrics=dict(result.metrics, worker_timeline=timeline))

    def event(self, *args, **kwargs):
        """Bind each completed solve to the receiver identity observed at dispatch."""
        result = super().event(*args, **kwargs)
        result['preparation_profile'] = (self.pending.get('preparation_profile')
                                         if self.pending else None)
        result['receiver_epoch_at_dispatch'] = (self.pending.get('receiver_epoch')
                                                if self.pending else None)
        result['dispatch_timing'] = (self.pending.get('dispatch_timing')
                                     if self.pending else None)
        result['delivery_budget'] = (self.pending.get('delivery_budget')
                                     if self.pending else None)
        if result['event'] == 'completion':
            self.last_completed_valid = result['output']['valid']
        result['deadline_retry'] = getattr(self, 'deadline_retry', None)
        rejected = getattr(self, 'delivery_rejected_request', None)
        if (self.pending is None and rejected is not None
                and result['admission_status'] == 'SUCCESSOR_AWAITING_FRESH_INPUT'):
            result['request'] = record_snapshot(rejected)
            result['input_provenance']['paired_navigation'] = record_snapshot(
                getattr(self, 'delivery_rejected_pose', None))
            result['input_provenance']['prepared_prediction'] = record_snapshot(self.prediction)
            result['delivery_budget'] = self.delivery_budget
            result['budget_rejection_before_submit'] = True
            result['expires_at_ros_stamp'] = self.delivery_budget['freshness_deadline']
        return result

    def reject(self, reason, now, monotonic_now):
        """Record a budget denial once per acquired image rather than losing new epochs."""
        request = getattr(self, 'delivery_rejected_request', None)
        if reason == 'SUCCESSOR_AWAITING_FRESH_INPUT' and request is not None:
            c = request.context
            epoch = (c.mission_id, c.clock_generation, c.observation_stamp)
            if getattr(self, 'last_delivery_denial', None) != epoch:
                self.last_rejection = None
                self.last_delivery_denial = epoch
        super().reject(reason, now, monotonic_now)

    def _prepare_delivery(self, result, now):
        """Select one path against full original-epoch delivery cost before submission."""
        from uav_control.guidance.fast_follow_minco import freshness
        from uav_control.guidance.follow_delivery_budget import make_plan, tail_reserve
        from uav_control.guidance.follow_minco_execution import delivery_request
        request, pose = result
        c = request.context
        until, _ = freshness(request, now)
        costs = self.solver.delivery_costs.hints(c.observation_stamp)
        if not getattr(self.solver, 'fallback_enabled', False):
            costs.pop('fallback', None)
        current = self.solver.wall_clock()
        samples = getattr(self, 'queue_samples', ())
        queue = max(.020, float(sorted(samples)[-1])*1.2) if samples else .020
        receiver = tail_reserve(getattr(self, 'receiver_cost_samples', ()), current, .020, .005)
        publication = tail_reserve(getattr(self, 'publication_cost_samples', ()),
                                   current, .010, .002)
        status = getattr(self, 'execution_status', None) or {}
        parent = getattr(request, 'prior_curve', None)
        parent_id = parent.plan_id if parent is not None else 0
        parent_deadline = status.get('deadline', 0.) if parent_id else 0.
        epoch = getattr(self, 'current_receiver_epoch', None)
        identity = ((epoch[0], getattr(self, 'planner_boot_id', '')) if epoch else ())
        identity += (c.mission_id, c.clock_generation, c.cycle_id)
        self.delivery_rejected_request = request
        self.delivery_rejected_pose = pose
        self.delivery_budget = dict(
            prepared_ros=current, freshness_deadline=until,
            preparation_seconds=current-now, queue_hint_seconds=queue,
            publication_reserve_seconds=publication, receiver_reserve_seconds=receiver,
            available_worker_seconds=(min(until, c.execution_start_stamp)-current
                                      - queue-publication-receiver), path_costs=costs,
            delivery_cost_evidence=self.solver.delivery_costs.report(c.observation_stamp),
            parent_id=parent_id, parent_deadline=parent_deadline, estimate_is_guarantee=False)
        plan = make_plan(identity, current, until, c.execution_start_stamp,
                         parent_id, parent_deadline, costs, queue, publication, receiver)
        self.delivery_budget.update(
            full_worker_hint_seconds=plan.worker_hint,
            selected_path=plan.path, worker_until=plan.worker_until,
            wire_until=plan.wire_until, accept_until=plan.accept_until)
        self.delivery_rejected_request = None
        return delivery_request(request, plan), pose

    def _prepare(self, now, monotonic_now):
        if self._delivery_enabled() and self.pending is not None:
            return self._completion_inputs(now, monotonic_now)
        self.delivery_rejected_request = None
        with CycleProfile(self.cycle_id+1) as profile:
            result = super()._prepare(now, monotonic_now)
            integrated = self._delivery_enabled() and hasattr(self.solver, 'delivery_costs')
            if integrated:
                result = self._prepare_delivery(result, now)
            waiting = getattr(self, 'retry_waiting_parent', None)
            status = getattr(self, 'execution_status', None) or {}
            if waiting and status.get('active') != waiting:
                self.retry_waiting_parent = None
                waiting = None
            successor = (getattr(self, 'deadline_scheduling', False) and status.get('active'))
            if (not integrated and (waiting or successor) and self.pending is None
                    and getattr(self.solver, 'online_budget_guard', False)):
                from uav_control.guidance.fast_follow_minco import freshness
                from uav_control.guidance.follow_cost_evidence import protect_costs
                until, _ = freshness(result[0], now)
                observation = result[0].context.observation_stamp
                protect_costs(self.solver, observation)
                costs = [self.solver.follow_estimate]
                if self.solver.fallback_enabled:
                    costs.append(self.solver.fallback_estimate)
                available = until-now-self.solver.fast.publication_reserve
                if successor and self._delivery_enabled():
                    # Successful cost evidence already includes model setup and
                    # the full worker. Measure preparation using the real clock;
                    # queue history is a hint, not a relaxation of any deadline.
                    current = self.solver.wall_clock()
                    samples = getattr(self, 'queue_samples', ())
                    # The loaded trace rejected the median-only reserve: its
                    # small queue estimate admitted too many tail-cost workers.
                    # Retain the existing 20ms dispatch/jitter floor, separately
                    # from full worker cost, and account preparation explicitly.
                    queue_hint = max(.020, 1.2*median(samples)) if samples else .020
                    available = until-current-self.solver.fast.publication_reserve-queue_hint
                    epoch = (result[0].context.mission_id,
                             result[0].context.clock_generation, observation)
                    if (available <= min(costs)
                            and getattr(self, 'budget_wait_epoch', None) != epoch):
                        self.budget_wait_epoch = epoch
                        paths = (('follow', 'fallback') if self.solver.fallback_enabled
                                 else ('follow',))
                        protect_costs(self.solver, observation, age_paths=paths)
                        costs = [self.solver.follow_estimate]
                        if self.solver.fallback_enabled:
                            costs.append(self.solver.fallback_estimate)
                        current = self.solver.wall_clock()
                        available = until-current-self.solver.fast.publication_reserve-queue_hint
                    self.delivery_budget = dict(
                        prepared_ros=current, freshness_deadline=until,
                        preparation_seconds=current-now, queue_hint_seconds=queue_hint,
                        full_worker_hint_seconds=min(costs),
                        publication_reserve_seconds=self.solver.fast.publication_reserve,
                        available_worker_seconds=available, estimate_is_guarantee=False)
                    if available <= min(costs):
                        raise ValueError('SUCCESSOR_AWAITING_FRESH_INPUT')
                # Preserve the legacy fixed headroom for periodic cases. The
                # delivery path above separates measured preparation/queue from
                # full worker cost; neither path changes raw TTL or the lease.
                if successor and not self._delivery_enabled() and available <= min(costs)+.020:
                    context = result[0].context
                    epoch = (context.mission_id, context.clock_generation,
                             context.observation_stamp)
                    previous = getattr(self, 'budget_wait_epoch', None)
                    if (previous is None or epoch[:2] != previous[:2]
                            or epoch[2] > previous[2]):
                        # Waiting must not freeze a cost spike forever. Age the
                        # same non-guaranteed hints as the solver, once per new
                        # acquired image; repeated timer ticks cannot age them.
                        self.budget_wait_epoch = epoch
                        paths = (('follow', 'fallback') if self.solver.fallback_enabled
                                 else ('follow',))
                        protect_costs(self.solver, observation, age_paths=paths)
                        costs = [self.solver.follow_estimate]
                        if self.solver.fallback_enabled:
                            costs.append(self.solver.fallback_estimate)
                    if available <= min(costs)+.020:
                        raise ValueError('SUCCESSOR_AWAITING_FRESH_INPUT')
                if waiting and available <= min(costs):
                    raise ValueError('RETRY_AWAITING_FRESH_INPUT')
        self.preparation_profile = profile.report()
        return result

    def _completion_inputs(self, now, monotonic_now):
        """Recheck current inputs and the original handover, without a discarded new solve."""
        if not self._follow():
            raise ValueError('MISSION_NOT_FOLLOW')
        p = self.prediction
        if p is None:
            raise ValueError('PREDICTION_UNAVAILABLE')
        if p.mission_id != self.mission['mission_id']:
            raise ValueError('MISSION_CHANGED')
        if p.source != 'tracking' or p.frame_id != 'local_ned':
            raise ValueError('PREDICTION_SOURCE_OR_FRAME')
        if p.source_stamp > now or p.generated_stamp > now or p.observation_stamp > now:
            raise ValueError('PREDICTION_FUTURE')
        pair = self.adapter.pair_latest(now, monotonic_now, p.observation_stamp)
        if pair.snapshot is None:
            raise ValueError(pair.status)
        if pair.snapshot.clock_generation != self.adapter.clock_generation:
            raise ValueError('CLOCK_GENERATION_CHANGED')
        if pair.snapshot.stamp > now:
            raise ValueError('INPUT_FUTURE')
        from uav_control.guidance.follow_time_contract import input_time_contract
        contract = input_time_contract(self.config.target_time_profile)
        if (now-pair.snapshot.stamp > self.config.maximum_input_age
                or now-p.observation_stamp > contract.observation_age
                or now-p.source_stamp > contract.prediction_age
                or now >= p.valid_until):
            raise ValueError('STALE_INPUT')
        request = self.pending['request']
        admission = getattr(self, 'completion_admission', None)
        if admission is not None:
            admission(request, now)
        return request, pair.snapshot

    def tick(self, now, monotonic_now):
        """Busy work has no queued request; completion alone rebuilds from latest inputs."""
        if self.pending is not None and not self.pending['future'].done():
            return
        # A completed worker must always be consumed, but during the normal
        # dispatch cooldown there is no candidate to prepare or submit.
        # Avoid re-pairing/allocating snapshots on every 10 ms timer tick:
        # those preparations contend with the single active MINCO worker.
        # Freshness, admission and receiver identity are checked again when
        # the next request is actually eligible to dispatch. Explicit invalid
        # prediction/mission messages are rejected in their own callbacks.
        window = self._delivery_window(now)
        status = getattr(self, 'execution_status', None) or {}
        if self.pending is None and window is not None:
            if (status.get('pending') or self.prediction is None
                    or now+getattr(self, 'execution_lead', .15) >= status['deadline']-.05
                    or not window.eligible(self.prediction.observation_stamp, now)):
                return
            self.next_solve = min(self.next_solve, monotonic_now)
        if self.pending is None and monotonic_now < self.next_solve:
            return
        previous = self.pending
        finishing = previous is not None and previous['future'].done()
        status = getattr(self, 'execution_status', None) or {}
        retry = (getattr(self, 'retry_waiting_parent', None) is not None
                 and getattr(self, 'retry_waiting_parent', None) == status.get('active'))
        provenance = getattr(self, 'prediction_provenance', None)
        aligned = (getattr(self, 'deadline_scheduling', False)
                   and getattr(self, 'prediction', None) is not None
                   and provenance is not None and provenance.get('valid'))
        receipt = provenance.get('receipt_monotonic') if aligned else None
        if aligned and not finishing:
            if (not isinstance(receipt, (float, int)) or not math.isfinite(receipt)
                    or not 0 <= receipt <= monotonic_now):
                self.reject('PREDICTION_RECEIPT_INVALID', now, monotonic_now)
                return
            # Select at most one regular input per configured receipt period.
            # A timer cooldown alone can repeatedly select a 50ms-old callback
            # at 5Hz/20Hz. Wait for an input received in the eligible period,
            # then let the original raw TTL, pairing and admission checks run.
            # The sole bounded retry retains its existing freshness budget guard.
            if window is None and not retry and receipt < self.next_solve:
                return
        eligible = self.next_solve
        super().tick(now, monotonic_now)
        if finishing:
            try:
                timeline = previous['future'].result().metrics.get('worker_timeline', {})
            except Exception:
                timeline = {}  # Base completion already records SOLVER_EXCEPTION.
            submitted = previous.get('submitted_monotonic')
            started = timeline.get('solve_start_monotonic')
            if submitted is not None and started is not None and started >= submitted:
                if not hasattr(self, 'queue_samples'):
                    self.queue_samples = deque(maxlen=8)
                self.queue_samples.append(started-submitted)
            if window is not None and previous.get('delivery_identity') == window.identity:
                window.failed = not getattr(self, 'last_completed_valid', False)
        if self._delivery_enabled() and finishing:
            return
        status = getattr(self, 'execution_status', None)
        if (finishing and getattr(self, 'deadline_scheduling', False)
                and not getattr(self, 'last_completed_valid', False) and status
                and 0 <= now-status['stamp'] <= .2 and status['active']
                and 0 < status['deadline']-now <= self.period+.15+.05
                and getattr(self, 'retried_parent', None) != status['active']):
            self.retried_parent = status['active']
            self.retry_waiting_parent = status['active']
            self.next_solve = min(self.next_solve, monotonic_now+.01)
            self.deadline_retry = dict(parent=status['active'], stamp=now,
                                       scheduled_monotonic=self.next_solve,
                                       original_deadline=status['deadline'])
        if self.pending is not None and self.pending is not previous:
            if window is not None:
                window.dispatched(self.pending['request'].context.observation_stamp,
                                  self.cycle_id,
                                  self.pending['request'].context.execution_start_stamp)
                self.pending['delivery_identity'] = window.identity
            elif aligned and not retry:
                self.next_solve = receipt+self.period
            self.pending['dispatch_timing'] = dict(
                policy=('successor_delivery' if window is not None else
                        'bounded_retry' if retry else 'input_aligned' if aligned else 'timer'),
                delivery_attempt=window.attempts if window is not None else None,
                delivery_parent=status.get('active') if window is not None else None,
                eligible_monotonic=eligible if math.isfinite(eligible) else None,
                dispatch_monotonic=monotonic_now,
                prediction_receipt_monotonic=receipt,
                prediction_receipt_ros_stamp=(provenance.get('receipt_ros_stamp')
                                              if provenance else None),
                prediction_sequence_id=self.pending['prediction'].sequence_id,
                prediction_observation_stamp=self.pending['prediction'].observation_stamp,
                input_receipt_to_dispatch_seconds=(monotonic_now-receipt
                                                   if receipt is not None else None))
            self.retry_waiting_parent = None
            self.pending['preparation_profile'] = self.preparation_profile
            self.pending['delivery_budget'] = getattr(self, 'delivery_budget', None)
            self.pending['receiver_epoch'] = getattr(self, 'current_receiver_epoch', None)


class P4FollowPlannerNode(FollowResearchShadowNode):
    """Keep one worker/latest input and all existing completion/serialization TTL checks."""

    node_name = 'p4_follow_planner_node'
    config_class = ResearchConfig

    def __init__(self):
        """Publish a distinct proposal topic; ACK is subscribed only, never synthesized."""
        super().__init__()
        from std_msgs.msg import String
        from uav_control.guidance.p6_follow import configure_p6
        self.p6_runtime = configure_p6(self, self.runner.solver.model)
        factory = self.runner.request_factory

        def p6_request(pose, prediction, *args, **kwargs):
            return self.p6_runtime.prepare(factory(
                pose, self.p6_runtime.planning_prediction(prediction), *args, **kwargs))

        self.runner.request_factory = p6_request
        self.p6_diagnostic_sub = self.create_subscription(
            String, '/planning/imm_diagnostics', self.p6_diagnostic_callback,
            qos_profile_sensor_data)
        self.follow_proposal_pub = self.create_publisher(
            FollowTrajectory, '/planning/follow_trajectory', 1)
        self.follow_ack_sub = self.create_subscription(
            FollowPlanAck, '/control/follow_ack', self.ack_callback, 10)
        self.ack_file = self.log_path.with_name('follow_ack_'+self.log_path.name).open('x')
        self.expected_plans = {}
        self.declare_parameter('prediction_revalidation', 'latest_only')
        self.prediction_policy = self.get_parameter('prediction_revalidation').value
        if self.prediction_policy not in ('latest_only', 'full'):
            raise ValueError('INVALID_PREDICTION_POLICY')
        self.planner_boot_id, self.receiver_epoch = secrets.token_hex(16), None
        self.runner.planner_boot_id = self.planner_boot_id
        if self.runner._delivery_enabled():
            from uav_control.guidance.follow_math_runtime import configure_numerical_threads
            self.runner.math_runtime = configure_numerical_threads(1)
            self.runner.critical_completion = self._publish_typed_completion
        self.receiver_status = None
        epoch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)
        self.receiver_epoch_sub = self.create_subscription(
            FollowReceiverState, '/control/follow_receiver_state', self.epoch_callback, epoch_qos)
        self.destroy_timer(self.timer)
        self.timer = self.create_timer(.01, self.tick)

    def p6_diagnostic_callback(self, message):
        """Cache immutable model evidence; match epochs before attaching it."""
        self.p6_runtime.diagnostic(message)
        if self.runner.prediction is not None:
            self.runner.prediction = self.p6_runtime.planning_prediction(self.runner.prediction)

    def epoch_callback(self, message):
        """Trust the unique actual receiver graph endpoint, never a proposal counter."""
        endpoints = self.get_publishers_info_by_topic('/control/follow_receiver_state')
        now, stamp = self._ros_seconds(), seconds(message.stamp)
        valid = (len(endpoints) == 1 and endpoints[0].node_name == 'trajectory_tracker_node'
                 and message.receiver == 'trajectory_tracker_node' and message.receiver_boot_id
                 and 0 <= now-stamp <= .2)
        if not valid:
            self.receiver_epoch = None
            self.receiver_status = None
            self.runner.current_receiver_epoch = None
            self.runner.execution_status = None
            return
        identity = (message.receiver_boot_id, int(message.clock_generation),
                    int(message.mission_id), self.adapter.clock_generation)
        previous = getattr(self, 'last_receiver_heartbeat', self.receiver_epoch)
        retired = getattr(self, 'retired_receiver_boots', set())
        if identity[0] in retired:
            return
        if (previous is not None and identity[0] == previous[0][0]
                and identity[1:3] < previous[0][1:3]):
            return
        if previous is not None and identity == previous[0] and stamp <= previous[1]:
            return
        floor = getattr(self, 'receiver_ack_floor', None)
        if floor is not None and identity == floor[0] and stamp <= floor[1]:
            return
        if previous is not None and identity != previous[0]:
            if identity[0] != previous[0][0]:
                retired.add(previous[0][0])
                self.retired_receiver_boots = retired
            self.accepted_curve = None
            self.ack_order, self.last_control_ack = {}, None
            self.receiver_ack_floor = None
            self.runner.queue_samples = deque(maxlen=8)
            self.runner.receiver_cost_samples = deque(maxlen=16)
            self.runner.publication_cost_samples = deque(maxlen=16)
        self.last_receiver_heartbeat = (identity, stamp)
        self.receiver_epoch = (identity, stamp)
        self.runner.current_receiver_epoch = identity
        self.receiver_status = dict(stamp=stamp, state=message.research_state,
                                    allowed=message.entry_allowed,
                                    converged=message.follow_converged,
                                    inhibition=getattr(message, 'entry_inhibition_reason', ''),
                                    active=int(message.active_plan_id),
                                    pending=int(message.pending_plan_id),
                                    deadline=message.active_lease_deadline)
        self.runner.execution_status = self.receiver_status

    def make_runner(self, config):
        """Retain the original synchronization factory, with explicitly separate cache policy."""
        old = super().make_runner(config)
        self.declare_parameter('follow_execution_enabled', False)
        self.follow_execution_enabled = bool(self.get_parameter('follow_execution_enabled').value)
        self.declare_parameter('follow_admission_required', True)
        self.follow_admission_required = bool(
            self.get_parameter('follow_admission_required').value)
        self.execution_curves, self.accepted_curve = {}, None
        original_factory = old.request_factory

        def admission(request, now):
            status = getattr(self, 'receiver_status', None)
            if self.follow_execution_enabled and self.follow_admission_required:
                if status is None or not 0 <= now-status['stamp'] <= .2:
                    raise ValueError('RECEIVER_ADMISSION_UNAVAILABLE')
                if status['pending']:
                    raise ValueError('AWAITING_PENDING_ACTIVE')
                curve = self.accepted_curve
                if status['active'] and (curve is None or status['active'] != curve.plan_id):
                    raise ValueError('AWAITING_REAL_ACTIVE_ACK')
                if (status['active']
                        and request.context.execution_start_stamp >= status['deadline']-.05):
                    raise ValueError('NO_SUCCESSOR_ACTIVATION_MARGIN')

        def completion_admission(request, now):
            admission(request, now)
            status = getattr(self, 'receiver_status', None) or {}
            prior = getattr(request, 'prior_curve', None)
            if (prior.plan_id if prior is not None else 0) != status.get('active', 0):
                raise ValueError('ACTIVE_PARENT_CHANGED')

        def factory(*args, **kwargs):
            request = original_factory(*args, **kwargs)
            curve = self.accepted_curve
            start = request.context.execution_start_stamp
            admission(request, request.now_stamp)
            if (self.follow_execution_enabled and curve is not None
                    and self.receiver_epoch is not None
                    and (curve.receiver_boot_id, curve.generation) == self.receiver_epoch[0][:2]
                    and curve.mission_id == request.context.mission_id
                    and curve.start <= start < min(curve.end, curve.start+.45)):
                from uav_control.guidance.follow_minco_execution import accepted_request
                return accepted_request(request, curve)
            return request

        old.solver.profile_enabled = True
        runner = P4ResearchRunner(old.config, old.adapter, old.executor, self._publish_event,
                                  1/old.period, old.solver, factory)
        runner.deadline_scheduling = (self.follow_execution_enabled
                                      and self.follow_admission_required)
        runner.execution_lead = float(self.get_parameter('execution_lead').value)
        runner.completion_admission = completion_admission
        self.declare_parameter('successor_scheduling', 'periodic')
        runner.successor_scheduling = self.get_parameter('successor_scheduling').value
        if runner.successor_scheduling not in ('periodic', 'delivery'):
            raise ValueError('INVALID_SUCCESSOR_SCHEDULING')
        return runner

    def ack_callback(self, message):
        """Log wrong receiver/identity as invalid ACK; REJECTED never grants execution."""
        key = (message.receiver_boot_id, message.planner_boot_id, int(message.mission_id),
               int(message.clock_generation), int(message.plan_id))
        publication = self.expected_plans.get(key)
        now = self._ros_seconds()
        ack_stamp = seconds(message.stamp)
        endpoints = self.get_publishers_info_by_topic('/control/follow_ack')
        one_receiver = len(endpoints) == 1 and endpoints[0].node_name == 'trajectory_tracker_node'
        valid = bool(one_receiver and message.receiver == 'trajectory_tracker_node'
                     and publication is not None and publication <= ack_stamp <= now
                     and now-ack_stamp <= .125
                     and message.mission_id == self.runner.mission['mission_id']
                     and self.receiver_epoch is not None
                     and key[:1] == self.receiver_epoch[0][:1]
                     and key[3] == self.receiver_epoch[0][1]
                     and key[1] == self.planner_boot_id)
        executing = getattr(self, 'follow_execution_enabled', False)
        curve = self.execution_curves.get(key) if executing else None
        if executing and message.state == 'ACTIVE':
            valid = bool(valid and curve is not None
                         and message.active_plan_id == message.plan_id
                         and message.control_owner == 'MINCO_FOLLOW'
                         and curve.start <= ack_stamp < min(curve.end, curve.start+.45)
                         and ack_stamp-curve.start <= .05+1e-7)
        order = getattr(self, 'ack_order', {})
        previous = order.get(key)
        terminal = ('REJECTED', 'REVOKED', 'EXPIRED')
        last = getattr(self, 'last_control_ack', None)
        if valid and (message.state not in ('ACCEPTED', 'ACTIVE', *terminal)
                      or (previous is not None and (
                          ack_stamp < previous[0] or previous[1] in terminal
                          or (previous[1] == 'ACTIVE' and message.state == 'ACCEPTED')))
                      or (message.state == 'ACTIVE' and last is not None and (
                          ack_stamp < last[0] or (ack_stamp == last[0]
                                                  and message.plan_id != last[1])))):
            valid = False
        if valid:
            order[key] = (ack_stamp, message.state)
            self.ack_order = order
            if len(order) > 1024:
                order.pop(next(iter(order)))
        unexpected_accept = not executing and message.state in ('ACCEPTED', 'ACTIVE')
        if executing and valid:
            cost = getattr(message, 'receiver_compute_seconds', 0.)
            if message.state in ('ACCEPTED', 'REJECTED') and math.isfinite(cost) and cost > 0:
                if not hasattr(self.runner, 'receiver_cost_samples'):
                    self.runner.receiver_cost_samples = deque(maxlen=16)
                self.runner.receiver_cost_samples.append((ack_stamp, cost))
            if message.state == 'ACTIVE' and curve is not None:
                self.accepted_curve = curve
                self.last_control_ack = (ack_stamp, message.plan_id)
            elif (message.state in ('REVOKED', 'EXPIRED') and self.accepted_curve is not None
                  and self.accepted_curve.plan_id == message.plan_id):
                self.accepted_curve = None
                self.last_control_ack = (ack_stamp, message.plan_id)
            if getattr(self.runner, 'successor_scheduling', 'periodic') == 'delivery':
                status = getattr(self, 'receiver_status', None)
                if (message.state == 'ACTIVE' and curve is not None and status is not None
                        and ack_stamp >= status['stamp']):
                    self.receiver_status = dict(status, stamp=ack_stamp,
                                                active=curve.plan_id,
                                                pending=int(message.pending_plan_id),
                                                deadline=min(curve.end, curve.start+.45))
                    self.runner.execution_status = self.receiver_status
                    self.receiver_ack_floor = (self.receiver_epoch[0], ack_stamp)
                elif (message.state == 'ACCEPTED' and curve is not None and status is not None
                      and ack_stamp >= status['stamp']
                      and message.pending_plan_id == curve.plan_id
                      and curve.parent_id == status['active']):
                    self.receiver_status = dict(status, stamp=ack_stamp, pending=curve.plan_id)
                    self.runner.execution_status = self.receiver_status
                    self.receiver_ack_floor = (self.receiver_epoch[0], ack_stamp)
                elif (message.state == 'REJECTED' and status is not None
                      and ack_stamp >= status['stamp'] and status['pending'] == message.plan_id):
                    self.receiver_status = dict(status, stamp=ack_stamp, pending=0)
                    self.runner.execution_status = self.receiver_status
                    self.receiver_ack_floor = (self.receiver_epoch[0], ack_stamp)
                window = getattr(self.runner, 'delivery_window', None)
                if message.state == 'REJECTED' and window is not None:
                    if window.plan == message.plan_id:
                        window.failed = True
        self.ack_file.write(_serialize(dict(
            receipt=self._ros_seconds(), stamp=seconds(message.stamp),
            receipt_monotonic=time.monotonic(),
            identity=key, state=message.state, reasons=list(message.reasons),
            active_plan_id=message.active_plan_id, pending_plan_id=message.pending_plan_id,
            control_owner=message.control_owner, replaced=message.replaced,
            identity_valid=valid,
            publish_to_ack_seconds=seconds(message.stamp)-publication if valid else None,
            accepted_by_tracker=bool(executing and valid
                                     and message.state in ('ACCEPTED', 'ACTIVE')),
            unexpected_accept=unexpected_accept))+'\n')
        self.ack_file.flush()

    def _proposal_rejection(self, event, now):
        """Check the complete dispatch-to-proposal budget including new-version validation."""
        start = event.get('cycle_started_monotonic')
        if start is None:
            return 'CYCLE_ORIGIN_MISSING'
        payload = event['request']
        request = SimpleNamespace(context=SimpleNamespace(**payload['context']),
                                  now_stamp=payload['now_stamp'])
        reason = completion_rejection(request, self.runner.config, now,
                                      self.runner.mission['mission_id'],
                                      self.adapter.clock_generation, time.monotonic()-start,
                                      self.runner._follow())
        plan = payload.get('delivery_plan')
        if not reason and plan is not None and now >= plan['wire_until']:
            reason = 'DELIVERY_WIRE_DEADLINE'
        return reason

    @staticmethod
    def _request_view(request):
        """Shallow typed view for wire conversion; full immutable snapshot follows delivery."""
        result = dict(request.__dict__, context=dict(request.context.__dict__))
        for name in ('prior_curve', 'delivery_plan'):
            value = result.get(name)
            if value is not None:
                result[name] = dict(value.__dict__)
        return result

    def _publish_typed_completion(self, request, result, record):
        """Use the original objects before copying large prediction/solver audit trees."""
        self._pending_revalidation_audit = None
        plan = getattr(request, 'delivery_plan', None)
        if plan is not None:
            timeline = result.metrics.get('worker_timeline', {})
            path, finish = timeline.get('delivery_path'), timeline.get('solve_end_ros')
            handoff = self._ros_seconds()
            reason = ''
            if path != plan.path:
                reason = 'DELIVERY_PATH_MISMATCH'
            elif (finish is None or not math.isfinite(finish) or not math.isfinite(handoff)
                  or not plan.accepts(path, finish) or not plan.accepts(path, handoff)):
                reason = 'DELIVERY_WORKER_DEADLINE'
            elif finish > handoff:
                reason = 'DELIVERY_CLOCK_REVERSED'
            if reason:
                return dict(published=False, reason=reason, worker_handoff_ros=handoff)
        event = dict(
            request=self._request_view(request),
            output=dict(result.__dict__, context=dict(result.context.__dict__)),
            _typed_request=request, _typed_result=result,
            cycle_started_monotonic=record['started'],
            receiver_epoch_at_dispatch=record.get('receiver_epoch'))
        with CycleProfile(request.context.cycle_id) as profile:
            outcome = self._publish_proposal(event)
        worker_end = result.metrics.get('worker_timeline', {}).get('solve_end_monotonic')
        if worker_end is not None:
            finish = (record['started']+outcome['wire_cycle_seconds']
                      if outcome.get('wire_cycle_seconds') is not None else time.monotonic())
            if not hasattr(self.runner, 'publication_cost_samples'):
                self.runner.publication_cost_samples = deque(maxlen=16)
            self.runner.publication_cost_samples.append(
                (request.context.observation_stamp, max(0., finish-worker_end)))
        outcome['critical_publication_profile'] = profile.report()
        return outcome

    def _publish_event(self, event):
        """Include prediction revalidation and wire publication in the final cycle audit."""
        if 'critical_outcome' in event:
            outcome = event['critical_outcome']
            with CycleProfile(event['request']['context']['cycle_id']) as profile:
                super()._publish_event(event)
                self._flush_revalidation_audit()
            self.runner.last_completed_valid = bool(
                outcome['published'] and not outcome.get('reason'))
            self._record_publication_final(event, outcome, profile)
            return
        # Per-event audit buffer. A fresh buffer prevents cross-cycle leakage.
        self._pending_revalidation_audit = None
        cycle_id = event['request']['context']['cycle_id'] if event.get('request') else 0
        critical_first = (getattr(self, 'follow_execution_enabled', False)
                          and getattr(self, 'follow_admission_required', False))
        with CycleProfile(cycle_id) as profile:
            if critical_first:
                try:
                    outcome = (self._publish_proposal(event)
                               if event['output'].get('valid') and event.get('request') else
                               dict(published=False, reason=event['output'].get('solver_status')))
                finally:
                    # Revalidation and deadline checks ran before the wire;
                    # only full diagnostic serialization is deferred.
                    self._flush_revalidation_audit()
            # Large research markers and audit serialization cannot consume the
            # critical wire's raw-input budget. Wire/Receiver checks remain complete.
            super()._publish_event(event)
            if not critical_first:
                try:
                    outcome = (self._publish_proposal(event)
                               if event['output'].get('valid') and event.get('request') else
                               dict(published=False, reason=event['output'].get('solver_status')))
                finally:
                    self._flush_revalidation_audit()
            elif event.get('event') == 'completion':
                # The original one-parent retry follows actual critical delivery,
                # not solver feasibility or a later research-marker deadline.
                self.runner.last_completed_valid = bool(
                    outcome['published'] and not outcome.get('reason'))
        self._record_publication_final(event, outcome, profile)

    def _record_publication_final(self, event, outcome, profile):
        """Audit delivery and later research-marker completion without another wire attempt."""
        if event.get('cycle_started_monotonic') is None or not event.get('request'):
            return
        finished = time.monotonic()
        self.log_file.write(_serialize(dict(
            event='follow_publication_final', cycle_id=event['request']['context']['cycle_id'],
            stamp=self._ros_seconds(), whole_cycle_time=finished-event['cycle_started_monotonic'],
            remaining_raw_ttl=event['expires_at_ros_stamp']-self._ros_seconds(),
            preparation_profile=event.get('preparation_profile'),
            worker_profile=(event.get('candidate_output') or event['output'])
            .get('metrics', {}).get('cycle_profile'),
            research_marker_reason=('' if event['output'].get('valid') else
                                    event['output'].get('solver_status')),
            publication_profile=profile.report(), **outcome))+'\n')
        self.log_file.flush()

    def _publish_proposal(self, event):
        """Recheck all epochs after allocation; a published nominal curve grants no holding."""
        before = time.perf_counter()
        reason = self._proposal_rejection(event, self._ros_seconds())
        if reason:
            return dict(published=False, reason=reason)
        policy = self._execution_publication_policy(self._ros_seconds())
        if policy not in ('', 'SHADOW_PROBE'):
            return dict(published=False, reason=policy)
        proposal_event = self._prediction_event(event)
        if proposal_event is None:
            return dict(published=False, reason='PREDICTION_CHANGED_OR_REVALIDATION_FAILED')
        snapshot = constraint_snapshot(self.runner.solver.model)
        proposal_event = dict(proposal_event, output=dict(proposal_event['output'], metrics=dict(
            proposal_event['output']['metrics'], constraint_snapshot=snapshot.payload,
            constraint_fingerprint=snapshot.fingerprint)))
        message = proposal_from_event(proposal_event, self._ros_seconds())
        if message is None:
            return dict(published=False, reason='INPUT_OR_START_EXPIRED')
        plan = proposal_event['request'].get('delivery_plan')
        if plan is not None:
            from uav_control.controllers.follow_transport import stamp
            message.input_valid_until = stamp(min(seconds(message.input_valid_until),
                                                  plan['accept_until']))
        now = self._ros_seconds()
        reason = self._proposal_rejection(proposal_event, now)
        if reason:
            return dict(published=False, reason=reason)
        if self.receiver_epoch is None:
            return dict(published=False, reason='RECEIVER_EPOCH_UNAVAILABLE')
        epoch, epoch_stamp = self.receiver_epoch
        dispatched = event.get('receiver_epoch_at_dispatch')
        if (dispatched is None or tuple(dispatched) != epoch or not 0 <= now-epoch_stamp <= .2
                or epoch[2] != message.mission_id or epoch[3] != self.adapter.clock_generation):
            return dict(published=False, reason='RECEIVER_EPOCH_CHANGED')
        message.receiver_boot_id, message.clock_generation = epoch[:2]
        message.planner_boot_id = self.planner_boot_id
        if getattr(self, 'follow_execution_enabled', False):
            from uav_control.controllers.follow_transport import curve_from_message
            prior = proposal_event['request'].get('prior_curve')
            message.parent_plan_id = int(prior['plan_id']) if prior else 0
            message.planner_type, message.reason = 'rolling_minco', 'NOMINAL_EXECUTION'
            if policy == 'SHADOW_PROBE':
                message.reason = 'SHADOW_PROBE'
        key = (message.receiver_boot_id, message.planner_boot_id, message.mission_id,
               message.clock_generation, message.plan_id)
        if key in self.expected_plans:
            return dict(published=False, reason='PLAN_ALREADY_PUBLISHED')
        self.expected_plans[key] = now
        if getattr(self, 'follow_execution_enabled', False):
            self.execution_curves[key] = curve_from_message(message)
            if len(self.execution_curves) > 1024:
                self.execution_curves.pop(next(iter(self.execution_curves)))
        if len(self.expected_plans) > 1024:
            self.expected_plans.pop(next(iter(self.expected_plans)))
        self._publish_wire_proposal(message)
        wire_finish_ros, wire_finish_monotonic = self._ros_seconds(), time.monotonic()
        crossed = self._proposal_rejection(proposal_event, self._ros_seconds())
        self.log_file.write(_serialize(dict(event='follow_proposal', identity=key,
                                            stamp=now, holding_valid_until=0., authority=False,
                                            parent=message.parent_plan_id,
                                            planned_start=seconds(message.execution_start_stamp),
                                            raw_input_until=seconds(message.input_valid_until),
                                            source_age=now-seconds(message.source_stamp),
                                            observation_age=now-seconds(message.observation_stamp),
                                            publication_monotonic=time.monotonic(),
                                            current_lease_deadline=(
                                                getattr(self, 'receiver_status', None) or {})
                                            .get('deadline'),
                                            proposal_kind=message.reason,
                                            worker_timeline=event['output'].get('metrics', {})
                                            .get('worker_timeline'),
                                            prediction_receipt=self.runner.prediction_provenance,
                                            conversion_publish_seconds=time.perf_counter()-before))
                            + '\n')
        self.log_file.flush()
        return dict(published=True, reason=crossed, publication_crossed_deadline=bool(crossed),
                    wire_finish_ros=wire_finish_ros,
                    wire_finish_input_margin=seconds(message.input_valid_until)-wire_finish_ros,
                    wire_cycle_seconds=wire_finish_monotonic-event['cycle_started_monotonic'])

    def _execution_publication_policy(self, now):
        """Consume the unique actual receiver gate; probes never request execution."""
        if not (getattr(self, 'follow_execution_enabled', False)
                and getattr(self, 'follow_admission_required', False)):
            return ''
        status = getattr(self, 'receiver_status', None)
        if status is None or not 0 <= now-status['stamp'] <= .2:
            return 'RECEIVER_ADMISSION_UNAVAILABLE'
        if status['pending']:
            return 'AWAITING_PENDING_ACTIVE'
        if status['active'] or status['allowed']:
            return ''
        if status.get('inhibition') == 'FOLLOW_ENTRY_LIMIT_REACHED':
            return status['inhibition']
        if (status['state'] == 'FOLLOW_STABLE'
                or (status['state'] == 'FOLLOW_RECOVERY' and status.get('converged'))):
            return 'SHADOW_PROBE'
        return 'FOLLOW_ENTRY_INHIBITED'

    @profiled('ros_publication')
    def _publish_wire_proposal(self, message):
        self.follow_proposal_pub.publish(message)

    @profiled('prediction_revalidation')
    def _prediction_event(self, event):
        """Reject superseded output (A) or fully reassess unchanged coefficients (B)."""
        prediction = self.runner.prediction
        request_data = event.get('request')
        if prediction is None or not request_data or not event['output'].get('valid'):
            return None
        if prediction.sequence_id == request_data['context']['prediction_sequence_id']:
            return event
        report = dict(valid=False, reason='PREDICTION_CHANGED', policy=self.prediction_policy)
        proposed = None
        if self.prediction_policy == 'full':
            from uav_control.controllers.follow_mpc_seed import (
                MpcSeedResult, planning_context_from_payload,
            )
            from uav_control.guidance.follow_revalidation import revalidate_prediction
            payload = dict(request_data)
            payload['context'] = planning_context_from_payload(payload['context'])
            from uav_control.guidance.follow_minco_execution import request_from_payload
            request = event.get('_typed_request') or request_from_payload(request_data)
            from uav_control.guidance.follow_time_contract import context_deadline
            until = min(context_deadline(replace(
                request.context, observation_stamp=prediction.observation_stamp,
                prediction_source_stamp=prediction.source_stamp,
                prediction_valid_until=prediction.valid_until)),
                request.context.execution_start_stamp)
            remaining = until-self._ros_seconds()-.005
            plan = getattr(request, 'delivery_plan', None)
            if plan is not None:
                remaining = min(remaining, plan.wire_until-self._ros_seconds()-.005)
            if event.get('cycle_started_monotonic') is not None:
                remaining = min(remaining, self.runner.config.solve_budget
                                - (time.monotonic()-event['cycle_started_monotonic'])-.005)
            if remaining <= 0:
                return None
            updated, report = revalidate_prediction(
                request, event.get('_typed_result') or MpcSeedResult(**event['output']),
                prediction, self._ros_seconds(),
                self.runner.solver.model, budget=remaining,
                cache=getattr(self.runner.solver, 'invariant_cache', None))
            if report['valid'] and self.runner.prediction is prediction:
                view = (self._request_view(updated) if event.get('_typed_request') is not None
                        else record_snapshot(updated))
                proposed = dict(event, request=view)
                proposed['output'] = dict(
                    event['output'], context=record_snapshot(updated.context), metrics=dict(
                        event['output']['metrics'],
                        minimum_horizontal_margin=report['minimum_horizontal_margin'],
                        minimum_vertical_margin=report['minimum_vertical_margin'],
                        prediction_revalidation=report))
        # Keep the entire research audit, but never serialize a large future
        # prediction before the critical wire has been attempted. This record
        # is flushed even when complete revalidation rejects the candidate.
        self._pending_revalidation_audit = (
            request_data['context']['cycle_id'], self._ros_seconds(), report, prediction)
        return proposed

    def _flush_revalidation_audit(self):
        record = getattr(self, '_pending_revalidation_audit', None)
        self._pending_revalidation_audit = None
        if record is None:
            return
        cycle_id, stamp, report, prediction = record
        self.log_file.write(_serialize(dict(
            event='prediction_revalidation', cycle_id=cycle_id, stamp=stamp,
            report=report, new_prediction=record_snapshot(prediction)))+'\n')
        self.log_file.flush()

    def destroy_node(self):
        """Close the independent receiver log before the existing worker shutdown."""
        if hasattr(self, 'ack_file'):
            self.ack_file.close()
        return super().destroy_node()


def main(args=None):
    """Run proposals; final PX4 authority stays exclusively with original Tracker."""
    rclpy.init(args=args)
    node = None
    try:
        node = P4FollowPlannerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
