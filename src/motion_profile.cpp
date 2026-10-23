#include "flight_models/motion_profile.h"

#include <nlohmann/json.hpp>
#include <valijson/adapters/nlohmann_json_adapter.hpp>
#include <valijson/schema.hpp>
#include <valijson/schema_parser.hpp>
#include <valijson/validator.hpp>
#include <valijson/validation_results.hpp>

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace flight_models {
namespace {
using Json = nlohmann::json;

Json read_json(const std::string& path, bool is_schema) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot read " + std::string(is_schema ? "schema: " : "profile: ") + path);
  try {
    return Json::parse(input);
  } catch (const Json::parse_error& error) {
    if (is_schema) throw std::runtime_error("invalid schema JSON " + path + ": " + error.what());
    throw std::invalid_argument("invalid profile JSON " + path + ": " + error.what());
  }
}

double finite_number(const Json& object, const char* key) {
  const double result = object.at(key).get<double>();
  if (!std::isfinite(result)) throw std::invalid_argument(std::string("profile field must be finite: ") + key);
  return result;
}

std::int64_t positive_int64(const Json& object, const char* key) {
  const auto& value = object.at(key);
  // Keep durations/counts exact even near int64 limits; schema comparisons
  // alone may round their bounds through floating-point arithmetic.
  if (value.is_number_unsigned()) {
    const auto unsigned_value = value.get<std::uint64_t>();
    if (unsigned_value > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
      throw std::invalid_argument(std::string("profile integer exceeds int64 range: ") + key);
    if (unsigned_value == 0)
      throw std::invalid_argument(std::string("profile integer must be positive: ") + key);
    return static_cast<std::int64_t>(unsigned_value);
  }
  if (!value.is_number_integer())
    throw std::invalid_argument(std::string("profile field must be an integer: ") + key);
  const auto signed_value = value.get<std::int64_t>();
  if (signed_value <= 0)
    throw std::invalid_argument(std::string("profile integer must be positive: ") + key);
  return signed_value;
}

}  // namespace

MotionProfile load_motion_profile(const std::string& path, const std::string& schema_path) {
  // Parse both inputs before schema validation so file and JSON failures retain their source context.
  const Json document = read_json(path, false);
  const Json schema_document = read_json(schema_path, true);

  valijson::Schema schema;
  try {
    valijson::SchemaParser parser;
    valijson::adapters::NlohmannJsonAdapter schema_adapter(schema_document);
    parser.populateSchema(schema_adapter, schema);
  } catch (const std::runtime_error& error) {
    throw std::runtime_error("cannot construct JSON schema " + schema_path + ": " + error.what());
  }

  valijson::Validator validator;
  valijson::adapters::NlohmannJsonAdapter document_adapter(document);
  valijson::ValidationResults errors;
  if (!validator.validate(schema, document_adapter, &errors)) {
    std::ostringstream message;
    message << "profile " << path << " violates schema " << schema_path;
    valijson::ValidationResults::Error error;
    while (errors.popError(error)) {
      message << ": ";
      for (const auto& component : error.context) message << component;
      message << " " << error.description;
    }
    throw std::invalid_argument(message.str());
  }

  const auto& envelope = document.at("flight_envelope");
  const auto& integration = document.at("integration");
  const auto& publication = document.at("publication");
  // Keep numerical groups inspectable before constructing the complete profile.
  const FlightEnvelope limits{
      finite_number(envelope, "min_speed_mps"), finite_number(envelope, "max_speed_mps"),
      finite_number(envelope, "max_acceleration_mps2"), finite_number(envelope, "max_deceleration_mps2"),
      finite_number(envelope, "max_turn_rate_radps"), finite_number(envelope, "max_climb_rate_mps"),
      finite_number(envelope, "max_descent_rate_mps"), finite_number(envelope, "min_altitude_hae_m"),
      finite_number(envelope, "max_altitude_hae_m")};
  const IntegrationPolicy integration_policy{
      positive_int64(integration, "max_substep_ns"), positive_int64(integration, "max_interval_ns"),
      positive_int64(integration, "max_substeps_per_interval")};
  const PublicationPolicy publication_policy{
      positive_int64(publication, "dis_period_ns"), positive_int64(publication, "prd_period_ns")};
  MotionProfile result{
      document.at("profile_id").get<std::string>(),
      document.at("model").get<std::string>(),
      document.at("navigation").get<std::string>(), limits,
      finite_number(document.at("guidance"), "waypoint_capture_radius_m"),
      integration_policy, publication_policy};

  // The schema enforces positive fields; cross-field ordering remains a typed semantic check.
  if (result.flight_envelope.min_speed_mps > result.flight_envelope.max_speed_mps)
    throw std::invalid_argument("profile minimum speed exceeds maximum speed");
  if (result.flight_envelope.min_altitude_hae_m >= result.flight_envelope.max_altitude_hae_m)
    throw std::invalid_argument("profile minimum altitude must be below maximum altitude");

  const auto max_substep = result.integration.max_substep_ns;
  const auto max_interval = result.integration.max_interval_ns;
  if (max_substep > max_interval)
    throw std::invalid_argument("profile maximum substep exceeds maximum interval");

  // Quotient/remainder computes ceil(interval/substep) without addition overflow.
  const auto required_substeps = max_interval / max_substep + (max_interval % max_substep != 0);
  if (required_substeps > result.integration.max_substeps_per_interval)
    throw std::invalid_argument("profile substep budget cannot cover maximum interval");

  return result;
}

}  // namespace flight_models
