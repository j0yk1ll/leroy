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
