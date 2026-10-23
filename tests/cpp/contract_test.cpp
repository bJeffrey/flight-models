#include <nlohmann/json.hpp>
#include <valijson/adapters/nlohmann_json_adapter.hpp>
#include <valijson/schema.hpp>
#include <valijson/schema_parser.hpp>
#include <valijson/validator.hpp>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>

namespace {
using Json = nlohmann::json;

/** Load a version-controlled specimen or schema; missing inputs fail the test. */
Json read_json(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot read " + path);
  return Json::parse(input);
}

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

/** Check structural contracts separately from interval and frame invariants. */
void validate_schema(const Json& document, const Json& definition) {
  valijson::Schema schema;
  valijson::SchemaParser parser;
  valijson::adapters::NlohmannJsonAdapter schema_adapter(definition);
  parser.populateSchema(schema_adapter, schema);
  valijson::Validator validator;
  valijson::adapters::NlohmannJsonAdapter document_adapter(document);
  require(validator.validate(schema, document_adapter, nullptr), "specimen violates JSON schema");
}

/** Validate a coherent physical-state projection, never a navigation substitute. */
void validate_state(const Json& state, const Json& definition) {
  validate_schema(state, definition);
  const auto& rotation = state.at("body_frd_to_ecef");
  // A proper direction-cosine matrix has orthonormal rows and positive determinant.
  // This fixture tolerance is a numerical check, not a configured motion limit.
  constexpr double rotation_tolerance = 1e-12;
  for (int row = 0; row < 3; ++row) {
    for (int other = 0; other < 3; ++other) {
      double product = 0;
      for (int column = 0; column < 3; ++column)
        product += rotation.at(3 * row + column).get<double>() * rotation.at(3 * other + column).get<double>();
      require(std::abs(product - (row == other ? 1.0 : 0.0)) <= rotation_tolerance, "rotation is not orthonormal");
    }
  }
  const auto value = [&](int index) { return rotation.at(index).get<double>(); };
  const double determinant = value(0) * (value(4) * value(8) - value(5) * value(7)) -
      value(1) * (value(3) * value(8) - value(5) * value(6)) +
      value(2) * (value(3) * value(7) - value(4) * value(6));
  require(std::abs(determinant - 1.0) <= rotation_tolerance, "rotation changes handedness");
}

/**
 * Reference-check existing SimTimeTick projections under explicit lifecycle inputs.
 *
 * This is test-only acceptance logic. Replace it with calls to the production
 * time adapter at FM-RUNTIME-1; retain these specimens as adapter inputs.
 * Restore labels represent completed authorized checkpoint application, not a
 * new network message or permission inferred from an arbitrary clock tick.
 */
unsigned validate_clock(const Json& specimen, const Json& definition) {
  validate_schema(specimen, definition);
  Json previous;
  unsigned advances = 0;
  std::map<std::uint64_t, Json> accepted_intervals;
  for (const auto& event : specimen.at("events")) {
    const auto& tick = event.at("tick");
    const auto operation = event.at("operation").get<std::string>();
    const auto elapsed_us = tick.at("t_us").get<std::uint64_t>();
    const auto dt_us = tick.at("dt_us").get<std::uint64_t>();
    require(elapsed_us <= static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() / 1000), "elapsed nanoseconds overflow");
    const bool advancing = tick.at("advances_time").get<bool>();
    require(advancing ? dt_us > 0 : dt_us == 0, "invalid clock interval");
    require(!advancing || !tick.at("paused").get<bool>(), "paused announcement advances time");

    if (previous.is_null()) {
      require(operation == "initialize" && elapsed_us == 0 && tick.at("tick_id") == 0 && !advancing,
              "startup requires an explicit initial boundary");
    } else {
      require(tick.at("run_id") == previous.at("run_id") &&
              tick.at("scenario_epoch_utc") == previous.at("scenario_epoch_utc"), "run or UTC origin changed");
      if (operation == "restore") {
        require(tick.at("timeline_epoch").get<std::uint64_t>() == previous.at("timeline_epoch").get<std::uint64_t>() + 1 &&
                tick.at("tick_id") == 0 && tick.at("paused").get<bool>() && !advancing &&
                previous.at("paused").get<bool>() && !previous.at("advances_time").get<bool>(),
                "restore requires a paused new epoch boundary");
        accepted_intervals.clear();
      } else {
        require(operation == "announce" && tick.at("timeline_epoch") == previous.at("timeline_epoch"), "uncommitted or stale epoch");
        if (advancing) {
          const auto replay = accepted_intervals.find(tick.at("tick_id").get<std::uint64_t>());
          if (replay != accepted_intervals.end()) {
            require(tick == replay->second, "conflicting replayed interval");
            continue;  // A late duplicate cannot undo a subsequent pause/resume.
          }
        }
        if (tick == previous) continue;  // Replayed delivery is idempotent.
        const auto prior_us = previous.at("t_us").get<std::uint64_t>();
        const auto prior_tick = previous.at("tick_id").get<std::uint64_t>();
        if (advancing) {
          require(elapsed_us > prior_us && elapsed_us - prior_us == dt_us &&
                  tick.at("tick_id").get<std::uint64_t>() == prior_tick + 1, "noncontiguous clock interval");
          ++advances;
          accepted_intervals.emplace(tick.at("tick_id").get<std::uint64_t>(), tick);
        } else {
          require(elapsed_us == prior_us && tick.at("tick_id") == previous.at("tick_id"), "heartbeat changed integration boundary");
        }
      }
    }
    previous = tick;
  }
  return advances;
}

/** A rejected fixture must fail for a contract reason, not a missing test file. */
template <typename Check>
void rejects(Check check, const std::string& reason) {
  try {
    check();
  } catch (const std::runtime_error& error) {
    require(std::string(error.what()).find(reason) != std::string::npos, "unexpected rejection: " + std::string(error.what()));
    return;
  }
  throw std::runtime_error("accepted invalid specimen: " + reason);
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  try {
    const std::string root = argv[1];
    const auto state_schema = read_json(root + "/contracts/physical_state.schema.json");
    const auto state = read_json(root + "/tests/fixtures/physical_state.json");
    validate_state(state, state_schema);
    // At latitude/longitude zero, level east means forward=+Y, right=-Z,
    // down=-X in ECEF. A transpose is also orthonormal but reverses this mapping.
    const auto check_east_axes = [](const Json& fixture) {
      const double expected[] = {0, 0, -1, 1, 0, 0, 0, -1, 0};
      for (int index = 0; index < 9; ++index)
        require(std::abs(fixture.at("body_frd_to_ecef").at(index).get<double>() - expected[index]) <= 1e-12,
                "wrong level-east body basis");
    };
    check_east_axes(state);
    auto bad_state = state;
    bad_state["body_frd_to_ecef"] = {0, 1, 0, 0, 0, -1, -1, 0, 0};
    validate_state(bad_state, state_schema);
    rejects([&] { check_east_axes(bad_state); }, "body basis");
    bad_state = state;
    bad_state["body_frd_to_ecef"][0] = 1;
    rejects([&] { validate_state(bad_state, state_schema); }, "orthonormal");
    bad_state = state;
    bad_state["dis_entity_id"][0] = 65536;
    rejects([&] { validate_state(bad_state, state_schema); }, "JSON schema");

    const auto clock_schema = read_json(root + "/contracts/clock_sequence.schema.json");
    const auto sequence = read_json(root + "/tests/fixtures/clock_sequence.json");
    require(validate_clock(sequence, clock_schema) == 3, "wrong number of integration intervals");
    const auto& source_tick = sequence.at("events").at(1).at("tick");
    require(state.at("run_id") == source_tick.at("run_id") &&
            state.at("timeline_epoch") == source_tick.at("timeline_epoch") &&
            state.at("tick_id") == source_tick.at("tick_id"), "snapshot context differs from source tick");
    // Scheduling remains elapsed-time based even after a restore to 30 seconds.
    require(state.at("elapsed_ns") == sequence.at("events").at(1).at("tick").at("t_us").get<std::int64_t>() * 1000,
            "30-second event was rebased");
    auto changed = sequence;
    changed["events"].insert(changed["events"].begin() + 2, changed["events"][1]);
    require(validate_clock(changed, clock_schema) == 3, "duplicate tick advanced twice");
    changed = sequence;
    changed["events"].insert(changed["events"].begin() + 3, changed["events"][1]);
    require(validate_clock(changed, clock_schema) == 3, "late duplicate undid pause");
    changed["events"][3]["tick"]["dt_us"] = 1;
    rejects([&] { validate_clock(changed, clock_schema); }, "conflicting replay");
    changed = sequence;
    changed["events"][4]["tick"]["dt_us"] = 1;
    rejects([&] { validate_clock(changed, clock_schema); }, "noncontiguous");
    changed = sequence;
    changed["events"][7]["tick"]["timeline_epoch"] = 0;
    rejects([&] { validate_clock(changed, clock_schema); }, "stale epoch");
    changed = sequence;
    changed["events"].erase(changed["events"].begin() + 5);
    rejects([&] { validate_clock(changed, clock_schema); }, "paused new epoch");
    changed = sequence;
    changed["events"][3]["tick"]["t_us"] = static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() / 1000) + 1;
    rejects([&] { validate_clock(changed, clock_schema); }, "overflow");
    std::cout << "physical frame, 30-second scheduling, pause/resume, restore, duplicate and rejection contracts passed\n";
  } catch (const std::exception& error) {
    std::cerr << "flight contract failed: " << error.what() << '\n';
    return 1;
  }
}
