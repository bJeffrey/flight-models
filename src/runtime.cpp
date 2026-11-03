#include "flight_models/runtime.h"

#include "flight_models/navigation.h"
#include "math/coordinates/geodetic.hpp"
#include "math/coordinates/geometry.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace flight_models {
namespace {
using Json = nlohmann::json;
constexpr std::int64_t kHourNs = 3600000000000LL;

std::size_t positive_size(const Json& value, const char* name) {
  if (value.is_number_unsigned()) {
    const auto parsed = value.get<std::uint64_t>();
    if (parsed == 0 || parsed > std::numeric_limits<std::size_t>::max())
      throw std::invalid_argument(std::string(name) + " must be a positive representable integer");
    return static_cast<std::size_t>(parsed);
  }
  if (!value.is_number_integer())
    throw std::invalid_argument(std::string(name) + " must be a positive integer");
  const auto parsed = value.get<std::int64_t>();
  if (parsed <= 0 || static_cast<std::uint64_t>(parsed) > std::numeric_limits<std::size_t>::max())
    throw std::invalid_argument(std::string(name) + " must be a positive representable integer");
  return static_cast<std::size_t>(parsed);
}

std::size_t nonnegative_size(const Json& value, const char* name) {
  if (value.is_number_unsigned()) {
    const auto parsed = value.get<std::uint64_t>();
    if (parsed > std::numeric_limits<std::size_t>::max())
      throw std::invalid_argument(std::string(name) + " exceeds size range");
    return static_cast<std::size_t>(parsed);
  }
  if (!value.is_number_integer()) throw std::invalid_argument(std::string(name) + " must be an integer");
  const auto parsed = value.get<std::int64_t>();
  if (parsed < 0 || static_cast<std::uint64_t>(parsed) > std::numeric_limits<std::size_t>::max())
    throw std::invalid_argument(std::string(name) + " must be a nonnegative representable integer");
  return static_cast<std::size_t>(parsed);
}

std::int64_t integer_ns(const Json& value, const char* name) {
  if (value.is_number_unsigned()) {
    const auto parsed = value.get<std::uint64_t>();
    if (parsed > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
      throw std::invalid_argument(std::string(name) + " exceeds int64 range");
    return static_cast<std::int64_t>(parsed);
  }
  if (!value.is_number_integer())
    throw std::invalid_argument(std::string(name) + " must be an integer");
  return value.get<std::int64_t>();
}

double number(const Json& value, const char* name) {
  if (!value.is_number()) throw std::invalid_argument(std::string(name) + " must be numeric");
  const double parsed = value.get<double>();
  if (!std::isfinite(parsed)) throw std::invalid_argument(std::string(name) + " must be finite");
  return parsed;
}

template <typename Integer>
Integer bounded_integer(const Json& value, const char* name) {
  if (value.is_number_unsigned()) {
    const auto parsed = value.get<std::uint64_t>();
    if (parsed > std::numeric_limits<Integer>::max())
      throw std::invalid_argument(std::string(name) + " is out of range");
    return static_cast<Integer>(parsed);
  }
  if (!value.is_number_integer()) throw std::invalid_argument(std::string(name) + " must be an integer");
  const auto parsed = value.get<std::int64_t>();
  if (parsed < 0 || static_cast<std::uint64_t>(parsed) > std::numeric_limits<Integer>::max())
    throw std::invalid_argument(std::string(name) + " is out of range");
  return static_cast<Integer>(parsed);
}

DisEntityMetadata dis_metadata(const Json& value) {
  DisEntityMetadata result;
  result.exercise_id = bounded_integer<std::uint8_t>(value.at("exercise_id"), "exercise_id");
  result.site_id = bounded_integer<std::uint16_t>(value.at("site_id"), "site_id");
  result.application_id = bounded_integer<std::uint16_t>(value.at("application_id"), "application_id");
  result.entity_id = bounded_integer<std::uint16_t>(value.at("entity_id"), "entity_id");
  result.force_id = bounded_integer<std::uint8_t>(value.at("force_id"), "force_id");
  const auto& type = value.at("type");
  result.type = {bounded_integer<std::uint8_t>(type.at("kind"), "type.kind"),
      bounded_integer<std::uint8_t>(type.at("domain"), "type.domain"),
      bounded_integer<std::uint16_t>(type.at("country"), "type.country"),
      bounded_integer<std::uint8_t>(type.at("category"), "type.category"),
      bounded_integer<std::uint8_t>(type.at("subcategory"), "type.subcategory"),
      bounded_integer<std::uint8_t>(type.at("specific"), "type.specific"),
      bounded_integer<std::uint8_t>(type.at("extra"), "type.extra")};
  result.marking = value.at("marking").get<std::string>();
  return result;
}

PrdMetadata prd_metadata(const Json& value) {
  return {value.at("platform_system_id").get<std::string>(),
          value.at("navigation_subsystem_id").get<std::string>(),
          value.at("classification").get<std::string>(),
          value.at("owner_government").get<std::string>(),
          value.at("navigation_solution_state").get<std::string>()};
}

PhysicalState initial_state(const Json& entity) {
  const auto& location = entity;
  const math::coordinates::GeodeticPosition geodetic{
      number(location.at("latitude_rad"), "latitude_rad"),
      number(location.at("longitude_rad"), "longitude_rad"),
      number(location.at("altitude_hae_m"), "altitude_hae_m")};
  const double heading = number(location.at("heading_rad"), "heading_rad");
  const double speed = number(location.at("horizontal_speed_mps"), "horizontal_speed_mps");
  const auto ned_to_ecef = math::coordinates::ned_to_ecef_rotation(geodetic);
  // Body FRD axes are built from heading in local NED, then rotated to ECEF.
  const double c = std::cos(heading), s = std::sin(heading);
  PhysicalState result;
  result.position_ecef_m = math::coordinates::geodetic_to_ecef(geodetic);
  result.velocity_ecef_mps = math::coordinates::rotate(ned_to_ecef, {speed * c, speed * s, 0.0});
  result.body_frd_to_ecef = math::coordinates::compose_rotations(ned_to_ecef,
      math::coordinates::body_frd_to_ned(heading, 0.0, 0.0));
  return result;
}

void require_object(const Json& value, const char* name) {
  if (!value.is_object()) throw std::invalid_argument(std::string(name) + " must be an object");
}

math::Vector3 vector3_from_json(const Json& value, const char* name) {
  if (!value.is_array() || value.size() != 3)
    throw std::invalid_argument(std::string(name) + " must contain exactly three values");
  return {number(value[0], name), number(value[1], name), number(value[2], name)};
}

Json vector3_to_json(const math::Vector3& value) {
  return Json::array({value.x, value.y, value.z});
}

std::array<double, 3> array3_from_json(const Json& value, const char* name) {
  const auto vector = vector3_from_json(value, name);
  return {vector.x, vector.y, vector.z};
}

Json array3_to_json(const std::array<double, 3>& value) {
  return Json::array({value[0], value[1], value[2]});
}

// Restore compatibility uses the admitted values, independently of profile files.
MotionProfile profile_from_json(const Json& value) {
  require_object(value, "profile");
  const auto& navigation = value.at("navigation");
  const auto& envelope = value.at("flight_envelope");
  const auto& integration = value.at("integration");
  const auto& publication = value.at("publication");
  require_object(navigation, "profile.navigation");
  require_object(envelope, "profile.flight_envelope");
  require_object(integration, "profile.integration");
  require_object(publication, "profile.publication");
  NavigationConfig parsed_navigation{
      navigation.at("model").get<std::string>(),
      array3_from_json(navigation.at("position_bias_ecef_m"), "position_bias_ecef_m"),
      array3_from_json(navigation.at("velocity_bias_ecef_mps"), "velocity_bias_ecef_mps"),
      number(navigation.at("position_sigma_m"), "position_sigma_m"),
      number(navigation.at("velocity_sigma_mps"), "velocity_sigma_mps"),
      bounded_integer<std::uint64_t>(navigation.at("seed"), "navigation.seed")};
  FlightEnvelope parsed_envelope{
      number(envelope.at("min_speed_mps"), "min_speed_mps"),
      number(envelope.at("max_speed_mps"), "max_speed_mps"),
      number(envelope.at("max_acceleration_mps2"), "max_acceleration_mps2"),
      number(envelope.at("max_deceleration_mps2"), "max_deceleration_mps2"),
      number(envelope.at("max_turn_rate_radps"), "max_turn_rate_radps"),
      number(envelope.at("max_climb_rate_mps"), "max_climb_rate_mps"),
      number(envelope.at("max_descent_rate_mps"), "max_descent_rate_mps"),
      number(envelope.at("min_altitude_hae_m"), "min_altitude_hae_m"),
      number(envelope.at("max_altitude_hae_m"), "max_altitude_hae_m")};
  MotionProfile result;
  result.profile_id                            = value.at("profile_id").get<std::string>();
  result.model                                 = value.at("model").get<std::string>();
  result.navigation                            = std::move(parsed_navigation);
  result.flight_envelope                       = parsed_envelope;
  result.waypoint_capture_radius_m             = number(value.at("waypoint_capture_radius_m"), "waypoint_capture_radius_m");
  result.integration.max_substep_ns            = integer_ns(integration.at("max_substep_ns"), "max_substep_ns");
  result.integration.max_interval_ns           = integer_ns(integration.at("max_interval_ns"), "max_interval_ns");
  result.integration.max_substeps_per_interval = integer_ns(
      integration.at("max_substeps_per_interval"), "max_substeps_per_interval");
  result.publication.dis_period_ns = integer_ns(publication.at("dis_period_ns"), "dis_period_ns");
  result.publication.prd_period_ns = integer_ns(publication.at("prd_period_ns"), "prd_period_ns");
  return result;
}

Json profile_to_json(const MotionProfile& profile) {
  Json navigation;
  navigation["model"]                  = profile.navigation.model;
  navigation["position_bias_ecef_m"]   = array3_to_json(profile.navigation.position_bias_ecef_m);
  navigation["velocity_bias_ecef_mps"] = array3_to_json(profile.navigation.velocity_bias_ecef_mps);
  navigation["position_sigma_m"]       = profile.navigation.position_sigma_m;
  navigation["velocity_sigma_mps"]     = profile.navigation.velocity_sigma_mps;
  navigation["seed"]                   = profile.navigation.seed;

  Json envelope;
  envelope["min_speed_mps"]         = profile.flight_envelope.min_speed_mps;
  envelope["max_speed_mps"]         = profile.flight_envelope.max_speed_mps;
  envelope["max_acceleration_mps2"] = profile.flight_envelope.max_acceleration_mps2;
  envelope["max_deceleration_mps2"] = profile.flight_envelope.max_deceleration_mps2;
  envelope["max_turn_rate_radps"]   = profile.flight_envelope.max_turn_rate_radps;
  envelope["max_climb_rate_mps"]    = profile.flight_envelope.max_climb_rate_mps;
  envelope["max_descent_rate_mps"]  = profile.flight_envelope.max_descent_rate_mps;
  envelope["min_altitude_hae_m"]    = profile.flight_envelope.min_altitude_hae_m;
  envelope["max_altitude_hae_m"]    = profile.flight_envelope.max_altitude_hae_m;

  Json integration;
  integration["max_substep_ns"]            = profile.integration.max_substep_ns;
  integration["max_interval_ns"]           = profile.integration.max_interval_ns;
  integration["max_substeps_per_interval"] = profile.integration.max_substeps_per_interval;
  Json publication;
  publication["dis_period_ns"] = profile.publication.dis_period_ns;
  publication["prd_period_ns"] = profile.publication.prd_period_ns;

  Json result;
  result["profile_id"]                = profile.profile_id;
  result["model"]                     = profile.model;
  result["navigation"]                = std::move(navigation);
  result["flight_envelope"]           = std::move(envelope);
  result["waypoint_capture_radius_m"] = profile.waypoint_capture_radius_m;
  result["integration"]               = std::move(integration);
  result["publication"]               = std::move(publication);
  return result;
}

// Decode shapes and units here; the driver validates the resulting motion state.
MotionDriverCheckpointRow row_from_json(const Json& value) {
  require_object(value, "driver row");
  const std::string mode_name = value.at("mode").get<std::string>();
  const EntityMode mode = mode_name == "fixed" ? EntityMode::fixed :
      mode_name == "moving" ? EntityMode::moving :
      throw std::invalid_argument("driver row mode must be fixed or moving");
  const auto& physical = value.at("physical");
  const auto& desired = value.at("desired");
  require_object(physical, "driver row physical state");
  require_object(desired, "driver row desired motion");
  const auto& attitude = physical.at("body_frd_to_ecef");
  if (!attitude.is_array() || attitude.size() != 9)
    throw std::invalid_argument("body_frd_to_ecef must contain exactly nine values");
  PhysicalState state;
  state.position_ecef_m        = vector3_from_json(physical.at("position_ecef_m"), "position_ecef_m");
  state.velocity_ecef_mps      = vector3_from_json(physical.at("velocity_ecef_mps"), "velocity_ecef_mps");
  state.acceleration_ecef_mps2 = vector3_from_json(physical.at("acceleration_ecef_mps2"), "acceleration_ecef_mps2");
  for (std::size_t i = 0; i < 9; ++i) state.body_frd_to_ecef[i] = number(attitude[i], "body_frd_to_ecef");
  state.angular_velocity_body_radps = vector3_from_json(
      physical.at("angular_velocity_body_radps"), "angular_velocity_body_radps");
  state.source_time_ns = integer_ns(physical.at("source_time_ns"), "source_time_ns");
  DesiredMotion target{
      number(desired.at("heading_rad"), "heading_rad"),
      number(desired.at("horizontal_speed_mps"), "horizontal_speed_mps"),
      number(desired.at("altitude_hae_m"), "altitude_hae_m")};
  return {value.at("entity_id").get<std::string>(), mode, state, target,
      nonnegative_size(value.at("command_index"), "command_index"),
      nonnegative_size(value.at("waypoint_index"), "waypoint_index")};
}

Json row_to_json(const MotionDriverCheckpointRow& row) {
  Json physical;
  physical["position_ecef_m"]        = vector3_to_json(row.state.position_ecef_m);
  physical["velocity_ecef_mps"]      = vector3_to_json(row.state.velocity_ecef_mps);
  physical["acceleration_ecef_mps2"] = vector3_to_json(row.state.acceleration_ecef_mps2);
  physical["body_frd_to_ecef"]       = Json::array();
  for (const double component : row.state.body_frd_to_ecef) physical["body_frd_to_ecef"].push_back(component);
  physical["angular_velocity_body_radps"] = vector3_to_json(row.state.angular_velocity_body_radps);
  physical["source_time_ns"]              = row.state.source_time_ns;
  Json desired;
  desired["heading_rad"]          = row.desired.heading_rad;
  desired["horizontal_speed_mps"] = row.desired.horizontal_speed_mps;
  desired["altitude_hae_m"]       = row.desired.altitude_hae_m;
  Json result;
  result["entity_id"]      = row.entity_id;
  result["mode"]           = row.mode == EntityMode::fixed ? "fixed" : "moving";
  result["physical"]       = std::move(physical);
  result["desired"]        = std::move(desired);
  result["command_index"]  = row.command_index;
  result["waypoint_index"] = row.waypoint_index;
  return result;
}
}  // namespace

struct Runtime::Impl {
  struct Entity {
    std::string id;
    EntityMode mode{};
    DisEntityMetadata dis;
    bool reports{};
    PrdMetadata prd;
  };
  std::size_t capacity;
  std::int64_t epoch_unix_ns;
  Json configuration;
  MotionProfile profile;
  NavigationModel navigation;
  StateStore store;
  std::unique_ptr<MotionDriver> driver;
  std::unordered_map<std::size_t, Entity> entities;
  bool started{};
  bool failed{};

  explicit Impl(const Json& config, const MotionProfile* frozen_profile = nullptr)
      : capacity(positive_size(config.at("capacity"), "capacity")),
        epoch_unix_ns(integer_ns(config.at("scenario_epoch_unix_ns"), "scenario_epoch_unix_ns")),
        configuration(config),
        profile(frozen_profile ? *frozen_profile :
            load_motion_profile(config.at("profile_path").get<std::string>(),
                                config.at("profile_schema_path").get<std::string>())),
        navigation(profile.navigation),
        store(capacity, 1) {
    if (config.at("entities").size() > capacity) throw std::length_error("entity count exceeds capacity");
    for (const auto& entity_json : config.at("entities")) {
      if (entity_json.contains("commands") || entity_json.contains("waypoints"))
        throw std::invalid_argument("runtime does not support entity commands or waypoints");
      const std::string id = entity_json.at("id").get<std::string>();
      const std::string mode_name = entity_json.at("mode").get<std::string>();
      const EntityMode mode = mode_name == "fixed" ? EntityMode::fixed :
          mode_name == "moving" ? EntityMode::moving : throw std::invalid_argument("entity mode must be fixed or moving");
      const auto state = initial_state(entity_json);
      const DesiredMotion desired{number(entity_json.at("heading_rad"), "heading_rad"),
          number(entity_json.at("horizontal_speed_mps"), "horizontal_speed_mps"),
          number(entity_json.at("altitude_hae_m"), "altitude_hae_m")};
      const Handle handle = store.spawn(id, 0, mode, state, desired);
      const auto& prd = entity_json.at("prd");
      Entity metadata{id, mode, dis_metadata(entity_json.at("dis")), !prd.is_null(), {}};
      if (metadata.reports) {
        metadata.prd = prd_metadata(prd);
      }
      entities.emplace(handle.slot, std::move(metadata));
    }
    const auto& limits = config.at("limits");
    DriverLimits bounds{nonnegative_size(limits.at("max_commands"), "max_commands"),
        nonnegative_size(limits.at("max_waypoints"), "max_waypoints"),
        positive_size(limits.at("max_samples"), "max_samples"),
        positive_size(limits.at("max_transitions"), "max_transitions"),
        positive_size(limits.at("max_events"), "max_events")};
    std::vector<EntityProgram> programs;
    programs.reserve(store.size());
    Snapshot snapshot(store.size());
    store.snapshot(snapshot);
    for (std::size_t row = 0; row < snapshot.size(); ++row) {
      const auto& source = snapshot.rows()[row];
      programs.push_back({source.handle, entities.at(source.handle.slot).reports, {}, {}});
    }
    driver = std::make_unique<MotionDriver>(store, 0, profile, bounds, std::move(programs));
  }
};

Runtime::Runtime(const Json& config) {
  try {
    impl_ = std::make_unique<Impl>(config);
  } catch (const Json::exception& error) {
    throw std::invalid_argument(std::string("invalid runtime configuration: ") + error.what());
  }
}
Runtime::Runtime(std::unique_ptr<Impl> candidate) noexcept : impl_(std::move(candidate)) {}
Runtime::~Runtime() = default;
Runtime::Runtime(Runtime&&) noexcept = default;
Runtime& Runtime::operator=(Runtime&&) noexcept = default;
std::vector<RuntimeOutput> Runtime::advance_to(std::int64_t time_ns) {
  return advance_to(time_ns, time_ns);
}
std::vector<RuntimeOutput> Runtime::advance_to(std::int64_t time_ns, std::int64_t creation_time_ns) {
  if (creation_time_ns < time_ns) throw std::invalid_argument("creation time cannot precede processed boundary");
  if (impl_->failed) throw std::logic_error("runtime is unusable after an output encoding failure");
  if (!impl_->started && time_ns != 0) throw std::invalid_argument("first runtime advance must be time zero");
  impl_->driver->advance_to(time_ns);
  impl_->started = true;
  std::vector<RuntimeOutput> result;
  try {
    result.reserve(impl_->driver->samples().size());
    auto epoch_hour = impl_->epoch_unix_ns % kHourNs;
    if (epoch_hour < 0) epoch_hour += kHourNs;
    for (const auto& sample : impl_->driver->samples()) {
      const auto& entity = impl_->entities.at(sample.entity.slot);
      RuntimeOutput output;
      output.entity_id = entity.id;
      output.kind = sample.kind;
      if (sample.kind == SampleKind::physical) {
        output.dis = encode_entity_state(entity.dis, sample.state, entity.mode, epoch_hour);
      } else if (entity.reports) {
        const auto solution = impl_->navigation.solution(sample.state, entity.id);
        output.prd = encode_position_report_detailed(entity.prd, solution, impl_->epoch_unix_ns, creation_time_ns);
      }
      result.push_back(std::move(output));
    }
  } catch (...) {
    impl_->failed = true;
    throw;
  }
  return result;
}

RuntimeCheckpoint Runtime::checkpoint() const {
  if (!impl_) throw std::logic_error("runtime is moved-from");
  if (impl_->failed) throw std::logic_error("failed runtime cannot be checkpointed");
  RuntimeCheckpoint result;
  result.configuration = impl_->configuration;
  result.profile       = impl_->profile;
  result.driver        = impl_->driver->checkpoint();
  result.started       = impl_->started;
  return result;
}

void Runtime::restore(const RuntimeCheckpoint& checkpoint) {
  if (!impl_) throw std::logic_error("runtime is moved-from");
  if (checkpoint.configuration != impl_->configuration ||
      !same_motion_profile(checkpoint.profile, impl_->profile) ||
      checkpoint.started != checkpoint.driver.initialized) {
    throw std::invalid_argument("runtime checkpoint configuration or profile mismatch");
  }
  // Reuse admitted values: restore must not reload a subsequently edited profile file.
  auto candidate = std::make_unique<Impl>(checkpoint.configuration, &checkpoint.profile);
  candidate->driver->restore(checkpoint.driver);
  candidate->started = checkpoint.started;
  impl_.swap(candidate);
}

std::string Runtime::encode_checkpoint() const {
  const RuntimeCheckpoint saved = checkpoint();
  Json driver;
  driver["initialized"] = saved.driver.initialized;
  driver["time_ns"]     = saved.driver.time_ns;
  driver["rows"]        = Json::array();
  for (const auto& row : saved.driver.rows) {
    Json encoded_row = row_to_json(row);
    driver["rows"].push_back(std::move(encoded_row));
  }

  Json payload;
  payload["format"]        = "flight-models.runtime-checkpoint";
  payload["version"]       = 1;
  payload["configuration"] = saved.configuration;
  payload["profile"]       = profile_to_json(saved.profile);
  payload["driver"]        = std::move(driver);
  try {
    return payload.dump();
  } catch (const Json::exception& error) {
    throw std::invalid_argument(std::string("runtime checkpoint is not JSON-encodable: ") + error.what());
  }
}

Runtime Runtime::prepare_restore(const std::string& payload_text) const {
  if (!impl_) throw std::logic_error("runtime is moved-from");

  Json payload;
  try {
    payload = Json::parse(payload_text);
  } catch (const Json::exception& error) {
    throw std::invalid_argument(std::string("invalid runtime checkpoint JSON: ") + error.what());
  }
  try {
    require_object(payload, "runtime checkpoint");
    if (payload.at("format").get<std::string>() != "flight-models.runtime-checkpoint")
      throw std::invalid_argument("unsupported runtime checkpoint format");
    if (bounded_integer<std::uint32_t>(payload.at("version"), "version") != 1)
      throw std::invalid_argument("unsupported runtime checkpoint version");

    RuntimeCheckpoint parsed;
    parsed.configuration = payload.at("configuration");
    parsed.profile       = profile_from_json(payload.at("profile"));
    const auto& driver = payload.at("driver");
    require_object(driver, "driver");
    if (!driver.at("initialized").is_boolean())
      throw std::invalid_argument("driver.initialized must be a boolean");
    parsed.driver.initialized = driver.at("initialized").get<bool>();
    parsed.driver.time_ns     = integer_ns(driver.at("time_ns"), "driver.time_ns");
    if (!driver.at("rows").is_array())
      throw std::invalid_argument("driver.rows must be an array");
    if (driver.at("rows").size() != impl_->entities.size())
      throw std::invalid_argument("runtime checkpoint cohort size mismatch");
    parsed.driver.rows.reserve(driver.at("rows").size());
    for (const auto& row : driver.at("rows")) {
      MotionDriverCheckpointRow decoded_row = row_from_json(row);
      parsed.driver.rows.push_back(std::move(decoded_row));
    }
    parsed.started = parsed.driver.initialized;

    if (parsed.configuration != impl_->configuration ||
        !same_motion_profile(parsed.profile, impl_->profile)) {
      throw std::invalid_argument("runtime checkpoint configuration or admitted profile mismatch");
    }

    // Build a compatible fresh driver to regenerate its opaque in-process program token.
    auto candidate = std::make_unique<Impl>(impl_->configuration, &impl_->profile);
    const MotionDriverCheckpoint fresh_compatibility = candidate->driver->checkpoint();
    parsed.driver.compatibility_ = fresh_compatibility.compatibility_;
    candidate->driver->restore(parsed.driver);
    candidate->started = parsed.started;
    return Runtime(std::move(candidate));
  } catch (const Json::exception& error) {
    throw std::invalid_argument(std::string("invalid runtime checkpoint structure: ") + error.what());
  }
}
}  // namespace flight_models
