#include "flight_models/process_config.h"
#include "flight_models/position_report.h"

#include <valijson/adapters/nlohmann_json_adapter.hpp>
#include <valijson/schema.hpp>
#include <valijson/schema_parser.hpp>
#include <valijson/validator.hpp>

#include <filesystem>
#include <fstream>
#include <limits>
#include <regex>
#include <set>
#include <stdexcept>

namespace flight_models {
namespace {
using Json = nlohmann::json;

Json read_json(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot read configuration file: " + path);
  try { return Json::parse(input); }
  catch (const Json::parse_error& error) {
    throw std::invalid_argument("invalid JSON in " + path + ": " + error.what());
  }
}

std::int64_t exact_int64(const Json& value, const char* name) {
  if (value.is_number_unsigned() &&
      value.get<std::uint64_t>() > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
    throw std::invalid_argument(std::string(name) + " exceeds signed nanosecond/integer range");
  }
  if (!value.is_number_integer()) throw std::invalid_argument(std::string(name) + " must be an exact integer");
  return value.get<std::int64_t>();
}
}  // namespace

std::string canonical_scenario_epoch(const std::string& text) {
  static const std::regex syntax(R"(^(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2})(?:\.(\d{1,9}))?Z$)");
  std::smatch match;
  if (!std::regex_match(text, match, syntax)) {
    throw std::invalid_argument("scenario epoch must be a Zulu timestamp with at most nine fractional digits");
  }
  std::string fraction = match[2].str();
  fraction.resize(9, '0');
  return match[1].str() + "." + fraction + "Z";
}

Json load_process_config(const std::string& path, const std::string& schema_path) {
  Json config = read_json(path);
  const Json definition = read_json(schema_path);
  valijson::Schema schema;
  valijson::SchemaParser parser;
  valijson::adapters::NlohmannJsonAdapter schema_adapter(definition);
  parser.populateSchema(schema_adapter, schema);
  valijson::Validator validator;
  valijson::adapters::NlohmannJsonAdapter document_adapter(config);
  if (!validator.validate(schema, document_adapter, nullptr)) {
    throw std::invalid_argument("runtime configuration violates schema: " + path);
  }

  // JSON Schema numeric comparisons may round large integers. Keep time-domain
  // checks exact, including epoch+duration at the last publication boundary.
  const auto epoch = exact_int64(config.at("scenario_epoch_unix_ns"), "scenario epoch");
  const auto end = exact_int64(config.at("end_time_ns"), "end time");
  if (end <= 0) throw std::invalid_argument("scenario end time must be positive");
  const auto supplied_epoch = canonical_scenario_epoch(config.at("scenario_epoch_utc"));
  if (format_uci_simulation_time(epoch, 0) != supplied_epoch) {
    throw std::invalid_argument("scenario UTC text and integer origin disagree");
  }
  (void)format_uci_simulation_time(epoch, end);
  (void)exact_int64(config.at("capacity"), "capacity");
  for (const auto& limit : config.at("limits")) (void)exact_int64(limit, "event limit");
  for (const auto* name : {"clock_poll_timeout_ms", "operation_timeout_ms", "pending_message_limit",
                           "pending_byte_limit", "clock_replay_window"}) {
    const auto value = exact_int64(config.at("transport").at(name), name);
    if (value <= 0 || value > std::numeric_limits<int>::max()) {
      throw std::invalid_argument(std::string(name) + " exceeds transport integer range");
    }
  }

  std::set<std::string> ids, systems, dis_ids;
  for (const auto& entity : config.at("entities")) {
    if (!ids.insert(entity.at("id").get<std::string>()).second) {
      throw std::invalid_argument("duplicate scenario entity ID");
    }
    const auto& dis = entity.at("dis");
    const Json identity = {dis.at("exercise_id"), dis.at("site_id"),
                           dis.at("application_id"), dis.at("entity_id")};
    if (!dis_ids.insert(identity.dump()).second) throw std::invalid_argument("duplicate DIS identity");
    if (!entity.at("prd").is_null() &&
        !systems.insert(entity.at("prd").at("platform_system_id").get<std::string>()).second) {
      throw std::invalid_argument("duplicate reporting platform SystemID");
    }
    if (entity.at("mode") == "fixed" && entity.at("horizontal_speed_mps").get<double>() != 0.0) {
      throw std::invalid_argument("fixed entities must start with zero speed");
    }
  }
  if (config.at("entities").size() > config.at("capacity").get<std::uint64_t>()) {
    throw std::invalid_argument("entity count exceeds configured capacity");
  }
  for (const char* name : {"profile_path", "profile_schema_path"}) {
    const std::filesystem::path configured = config.at(name).get<std::string>();
    if (configured.is_relative()) {
      config[name] = (std::filesystem::absolute(path).parent_path() / configured).lexically_normal().string();
    }
  }
  return config;
}
}  // namespace flight_models
