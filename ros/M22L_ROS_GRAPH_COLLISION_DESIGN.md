# M22L design gate: ROS diagnostic publisher collision checks

Status: DESIGN-ONLY. No implementation, ROS runtime, motors, or devices validated.

## M22K verified baseline

M22K operator-host acceptance: 97/97 offline PASS and isolated actual
LeroyBridgeNode/MemoryCDC simulation PASS, zero nonzero wheel writes.
PR #91 merged at 8c13323dd6e389de51f8bb986147daeb50fb77eb.

## Objective

Replace the M22K method's caller-supplied `publishers` labels with a
fresh source observation made by `LeroyBridgeNode` itself using ROS 2
`get_publishers_info_by_topic`. This is **diagnostic collision
detection**, not secure DDS publisher authentication.

A ROS graph snapshot must fail closed if any of three synthetic
diagnostic topics is missing, duplicated, mislabeled, of the wrong
message type, or unavailable:

- /leroy_nav2/diagnostics/m22l/signed_motion — expected controller_server
- /leroy_nav2/diagnostics/m22l/signed_rearm — expected m22l_fake_operator
- /leroy_nav2/diagnostics/m22l/synthetic_health — expected m22l_fake_health

A matching graph produces only an ADVISORY source label for the existing
M22K diagnostic supervisor. The real bridge must own the graph API call;
callers must not be able to pass arbitrary source labels into that new
graph-bound method.

## Threat model and limitations

DDS node_name, namespace, endpoint type, and graph presence do NOT prove
operator identity, process integrity, secure key custody, or message
origin. Discovery may race delivery. A malicious publisher may copy an
expected node name. Graph checks must NEVER replace HMAC/session checks.
Even an authenticated HMAC merely proves possession of a provisioned
test key, not human approval. M22L must not introduce a production signed
ROS motion subscriber, a new serial writer, or any actuator output.
Keep the M22D real CDC zero-wheel allowlist unmodified.

## Proposed test matrix (not yet executed)

1. Exactly one expected publisher on every diagnostic topic -> observe
   labels only, physical motion permission remains false.
2. Missing expected motion, rearm, or health publisher -> latch.
3. Two publishers on one topic, including an expected-name duplicate ->
   latch; do not pick one.
4. Same name, wrong namespace -> latch.
5. Wrong type -> latch.
6. Graph discovery raises exception or returns malformed metadata ->
   latch.
7. Source disappears after synthetic diagnostic rearm -> latch with no
   automatic recovery when it returns.
8. Graph conflict during synthetic rearm dwell -> restart authorization
   requirement and dwell.
9. Multiple inconsistent observations -> always re-evaluate and fail
   closed, never cache a prior passing graph snapshot.
10. HMAC forgery/old-session replay remains denied independently of
    healthy-looking graph metadata.
11. Bridge-owned local transport, clock and stop flags override any
    matched graph labels.
12. Exact real bridge against injected in-memory-only `m22f://memory`
    fake CDC; zero nonzero wheel packets and no physical device access.

## Required separation and acceptance

Do not construct the normal bridge on the host; that opens the real
STM32 serial port and may energize actuators. For any later ROS tests,
patch `serial.Serial` before construction and use isolated rootless
Podman with network=none, no device mappings, private IPC/PID, read-only
rootfs and all caps dropped.

Keep all new status reports explicitly labeled `diagnostic_only`,
`publisher_identity_authenticated=false`,
`physical_motion_authorized=false`, and `device_access=false`.

M22L source implementation, test code, run receipts and PR remain
pending. A graph-label match is not a release gate for physical motion.

## M22L Jazzy API review (2026-10-18)

Documentation review confirms the ROS 2 graph API exposes publisher endpoint
metadata through `get_publishers_info_by_topic`. Relevant references:

- https://api.nav2.org/jazzy/html/classrclcpp_1_1Node.html
- https://api.nav2.org/jazzy/html/graph_8h.html

These document the publisher-list contract and available node name,
namespace, topic type, GID, and QoS properties. The source of authority
is still signed envelope verification, not graph endpoint metadata.
Graph inspection must never set a field called `authenticated` to true.

### Required integration boundary

The existing bridge-owned M22K method
`_supervise_signed_diagnostic(supervisor, motion, rearm, now_ns, health,
publishers)` receives a **caller-provided** synthetic source tuple.
A distinct new graph-bound entry point must instead ask the actual bridge
for a fresh observation of the configured diagnostic topic endpoints.
That new method must NOT accept a caller-provided publisher list, a
precomputed `labels_match` boolean, or an injectable `graph_ok`
boolean. Keep the old M22K method as a diagnostic test fixture only;
do not treat it as trusted operator ingress.

Require ALL three named topics to show exactly ONE publisher with the
expected name, namespace `/`, and `std_msgs/msg/String` type. Count
every endpoint even when several endpoints report the same node label.
Use fully-qualified topic names. Treat no endpoints, duplicate
endpoints, incorrect type/namespace/name, invalid metadata and graph
API exceptions as a negative observation. Do not cache success and
do not automatically restore an armed diagnostic state on recovery.

Sample graph labels again for each evaluation and bind the outcome to
a stateful supervisor decision. A simulated publisher disappearing after
the rearm grant must transition the supervisor to `LATCHED`, requiring a
new separately signed rearm grant and full healthy dwell to resume
diagnostic permission. The result must still report
`physical_motion_authorized=false`.

### Negative regression contract — not yet implemented

| Case | Expected diagnostic behavior |
|---|---|
| No publishers present | Inhibit |
| Exactly one expected publisher per topic | Labels match, still no physical motion permission |
| Motion publisher absent | Inhibit |
| Rearm publisher absent | Inhibit |
| Health publisher absent | Inhibit |
| Two or more motion publishers | Latch on observed conflict |
| Two or more rearm publishers | Latch on observed conflict |
| Two or more health publishers | Latch on observed conflict |
| Same expected node label used twice | Latch; do not choose first |
| Correct node name, wrong namespace | Inhibit |
| Correct label, wrong message type | Inhibit |
| Missing or malformed endpoint attribute | Inhibit |
| Graph query exception | Inhibit |
| Publisher disappears after simulated arming | Latch; no automatic rearm |
| Publisher returns after disappearance | Remain latched until fresh rearm and dwell |
| Wrong-key motion or operator grant with healthy graph | Reject independently via HMAC |
| Replayed old-session command with healthy graph | Reject independently via session |
| Bridge-local clock fault with matching graph | Inhibit regardless of observation |
| Bridge-local terminal CDC fault with matching graph | Inhibit and refuse subsequent CDC writes |
| Evaluator stalls beyond M22K limit | Latch |
| Graph observation changes during evaluation | Never claim stable/atomic graph state |
| Any observed graph state | Zero nonzero bridge wheel writes |

### ROS runtime test requirements — not yet implemented

Test in isolated ROS 2 Jazzy with the real `LeroyBridgeNode` instantiated
ONLY after replacing `serial.Serial` with strict in-memory `MemoryCDC`,
using serial URI `m22f://memory`. Do not mount `/dev/ttyACM*`,
`/dev/video*`, host network, or robot launch files.

Use synthetic ROS publishers created and destroyed as separate nodes.
Graph discovery and disappearance are asynchronous, so poll with a
finite test deadline and treat timeout as test failure. Never interpret
the absence of a publisher in a single sample as proof of a durable
hardware security property; it is only an observation.

Preserve M22D command allowlist, M22G serial fault latch, M22J audit,
M22K supervisor and the lack of a raw `/cmd_vel` subscriber. Do not
create a production `LeroyBridgeNode` command subscriber as part of this
milestone. No physical STOP acknowledgement or E-stop claim.

Any eventual machine report should include:
`diagnostic_only=true`,
`publisher_identity_authenticated=false`,
`production_motion_enabled=false`,
`physical_motion_authorized=false`,
`physical_device_access=false`,
and `nonzero_bridge_wheel_writes=0`.

### Acceptance is still blocked

This section is a review and future test contract, not implemented
behavior. A matching ROS publisher name is readily impersonable and
graph discovery is non-atomic relative to message delivery. Proper
operator identity, protected signer keys, independent sensor integrity,
authenticated transport, and physical protective systems remain
unresolved. No test totals or PASS markers may be claimed until the
actual source changes and strict no-device tests have run successfully.

