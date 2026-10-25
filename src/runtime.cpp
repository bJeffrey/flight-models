#include "flight_models/runtime.h"

#include "flight_models/navigation.h"
#include "math/coordinates/geodetic.hpp"
#include "math/coordinates/geometry.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <unordered_map>

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
  MotionProfile profile;
  NavigationModel navigation;
  StateStore store;
  std::unique_ptr<MotionDriver> driver;
  std::unordered_map<std::size_t, Entity> entities;
  bool started{};
  bool failed{};

  explicit Impl(const Json& config)
      : capacity(positive_size(config.at("capacity"), "capacity")),
        epoch_unix_ns(integer_ns(config.at("scenario_epoch_unix_ns"), "scenario_epoch_unix_ns")),
        profile(load_motion_profile(config.at("profile_path").get<std::string>(),
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
}  // namespace flight_models
