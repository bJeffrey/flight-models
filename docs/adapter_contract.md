# Existing Adapter Field Contract

This inventory records observed source mappings for the FM contract slice. It
does not approve runtime behavior or close FM-CONTRACT-1. Schema references are
the installed OAC-STD-002 Rev E UCI 2.5 XSD at
`../../uci/03_OAC-STD-002_RevE_UCI_Schema_v2_5/UCI_MessageDefinitions_v2_5_0.xsd`.

## DIS EntityStatePdu

Source: `ams-gra-hello-world-sk-sim-supercell/src/dis.rs`,
`src/entity.rs`, and `tests/dis_pdu.rs`.

| DIS field | Source and transform | Limits / unresolved |
| --- | --- | --- |
| Entity ID | `site_id`, `application_id`, `entity_id` copied to DIS `EntityId` | Stable cross-run identity policy is not established here. |
| Entity type / force / marking | Seven `entity_type` codes and `force_id`; marking filtered to ASCII and truncated to 11 characters | Invalid force falls back to `Other`; code mapping and marking-loss policy need contract-level decision. |
| Entity location | `latitude_deg`, `longitude_deg`, ellipsoid `altitude_m` converted from WGS-84 geodetic to ECEF metres | MSL and terrain fields are not used. |
| Entity velocity | NED north/east/down m/s rotated to ECEF, then f32 | Confirmed by tests for conversion and round-trip; precision is limited by DIS vector f32. |
| Entity orientation | NED yaw/pitch/roll degrees composed with local frame and encoded as ECEF-relative DIS psi/theta/phi radians | Convention is documented in the adapter; no independent consumer-frame acceptance is shown. |
| Dead reckoning | Static entities use static algorithm; moving entities use DRM_RVW with ECEF acceleration and body angular rates | Acceleration fields are ECEF; angular-rate fields are body axes in rad/s. Runtime evolution and lifecycle semantics remain open. |
| Header timestamp | UTC scenario datetime converted to absolute DIS hour-relative units; low bit marks absolute | About 1.676 microsecond units; wraps hourly by format. `EntityState.sim_time_s` is not read by the builder. |

## UCI PositionReportDetailed

Source: `ams-gra-hello-world-sk-sim-supercell/src/owp.rs` and
`route-planning/src/route_planning/adapters/uci_position_report.py`.

| UCI 2.5 field | Observed mapping | Limits / unresolved |
| --- | --- | --- |
| `MessageHeader.SystemID` | Platform UUID: configured UUID for ownship; UUIDv5 of base UUID and DIS site/application/entity for other platforms | UUID derivation and authoritative identity registry require ecosystem agreement. |
| `PositionSource` | SuperCell emits `SubsystemID` for platform EGI; EGI UUID configured for ownship, derived for others | Route-planning now accepts either choice and keys the platform from the preserved header SystemID. The contributor does not become the aircraft identity. |
| `Kinematics.Position.AbsolutePoint` | Latitude/longitude radians; HAE altitude metres; timestamp | Matches UCI WGS-84 angular units and default/explicit HAE semantics. HAE comes from physical `altitude_m` today, so this is not yet a modeled navigation estimate. |
| `Kinematics.Velocity` | NED North/East/Down m/s and same supplied timestamp | Route-planning expects matching optional velocity timestamp, converts NED to ENU. |
| `Kinematics.Orientation` | Omitted by SuperCell | Route-planning requires reported `Orientation.Yaw` and preserves it separately from ground course. Flight-models must emit modeled navigation orientation; the yaw-less SuperCell specimen remains unsupported by this consumer. |
| `NavigationSolutionState` | `BLENDED` | Describes solution state; does not establish an error model. |
| `KinematicsError` | All NED position, position-velocity, and velocity covariance blocks populated | Current SuperCell derives diagonal terms from timing variance and velocity/acceleration; cross terms are zero. This does not establish a validated navigation covariance model. |
| `MessageHeader.Timestamp` and point timestamp | Publisher-supplied absolute UTC string | Header time means creation; component time means navigation validity. The reference may use one supplied value for both, but target scheduling must preserve their distinct meanings. |
| `SimulationTargetNumber` | Packed DIS site/application/entity integer | Explicit UCI simulation-only truth field; must not be used as operational identity or navigation input. |

The reusable nominal JSON fixture is
`tests/fixtures/uci_position_report_detailed_v2_5.json`. It is a consumer-ready
navigation-shaped specimen adapted from route-planning's existing
`uci_position_report_detailed_v1.json`, with mandatory UCI message security
metadata added per SuperCell's wrapper. It deliberately uses `PositionSource.SystemID`
as one standard source choice. Route-planning also has a SubsystemID specimen
and keys both by header SystemID; its canonical planner state does not use the
EGI as vehicle identity. Field names, order,
and NED covariance blocks are checked against the installed UCI 2.5 XSD; this
JSON is not itself an XML instance and XSD validation does not validate JSON.
The test-only Sleet probe republishes the same source UUID in each XSD choice
(`SystemID` and `SubsystemID`) to check schema acceptance. This is not an
identity-equivalence or source-assuredness test: the schema says one identifies
the producing System and the other the producing Subsystem.

## Route plan and execution boundary

`route-planning/src/route_planning/adapters/uci_route_output.py` maps its
canonical detailed route to UCI 2.5 `RoutePlanMDT`: route/command/path/segment
IDs and plan version, applicability, validity window, primary route, projection,
and ordered waypoint segments. Coordinates are radians; altitude and speed are
metres and metres/second. The adapter currently uses a named local ENU frame for
planning and projects to UCI geodetic points. Its generated detailed route is
`Detailed: true`; the SuperCell example route in `src/owp.rs` is `Detailed: false`,
uses MSL altitude, and derives IDs from configuration. These are distinct
producer contracts, not interchangeable fixtures. The route-planning UCI output
and execution-status adapters are the existing examples for route correlation
and feedback; SuperCell's sample route does not define permission or execution
authority.

No standard UCI authorization carrier, execution permission mapping, DIS entity
identity registry, canonical simulation time mapping, validated navigation
error profile, or accepted covariance realism is established by this audit.
Those remain explicit FM-CONTRACT-1 gaps. No emission/transmitter carrier is
selected by this field inventory.

## Validation candidates

The installed UCI XSD is XML, while adapter fixtures are JSON. Use
`xmllint --noout --schema <UCI_2.5_combined_schema.xsd> <generated_xml>` only
after converting a JSON specimen through the repository's Sleet UCI 2.5 types
or an approved serializer. Candidate Rust checks are the SuperCell OWP unit
tests (`cargo test ... owp`) and generated Sleet `PositionReportDetailedMt`
deserialization/serialization; route-planning's Python fixture test is useful
for semantic adapter acceptance but is not XSD validation. No rejection fixture
is included because an XML schema rejection specimen would need a serializer
path and schema validation harness not present in this repository slice.
