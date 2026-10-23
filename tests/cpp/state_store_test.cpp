#include "flight_models/state_store.h"
#include "flight_models/motion_profile.h"

#include <nlohmann/json.hpp>
#include <valijson/adapters/nlohmann_json_adapter.hpp>
#include <valijson/schema.hpp>
#include <valijson/schema_parser.hpp>
#include <valijson/validator.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <new>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// Replacement allocation operators live in a separate translation unit so
// compiler inlining does not misdiagnose their malloc/free implementation.
namespace test_allocation {
void forbid_heap_allocations(bool forbid) noexcept;
}

namespace {
using namespace flight_models;
using Json = nlohmann::json;

class NoAllocation {
 public:
  NoAllocation() { test_allocation::forbid_heap_allocations(true); }
  ~NoAllocation() { test_allocation::forbid_heap_allocations(false); }
};

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

template <typename Error, typename Function>
void rejects(Function operation, const char* message) {
  try {
    operation();
  } catch (const Error&) {
    return;
  }
  throw std::runtime_error(message);
}

PhysicalState state_at(std::size_t index, std::int64_t time_ns = 0) {
  PhysicalState state;
  state.position_ecef_m = {6379137.0 + 0.01 * index, 0, 0};
  state.velocity_ecef_mps = {0, 100.0 + index % 10, 0};
  state.body_frd_to_ecef = {0, 0, -1, 1, 0, 0, 0, -1, 0};
  state.source_time_ns = time_ns;
  return state;
}

DesiredMotion desired_at(std::size_t index) {
  return {1.5707963267948966, 100.0 + index % 10, 1000.0 + 0.01 * index};
}

/** Read explicit deployment resources; no resource limit is hidden in the store. */
Json policy(const std::string& root) {
  std::ifstream input(root + "/config/storage_policy.json");
  std::ifstream schema_input(root + "/config/storage_policy.schema.json");
  const auto document = Json::parse(input);
  const auto definition = Json::parse(schema_input);
  valijson::Schema schema;
  valijson::SchemaParser parser;
  valijson::adapters::NlohmannJsonAdapter schema_adapter(definition);
  parser.populateSchema(schema_adapter, schema);
  valijson::Validator validator;
  valijson::adapters::NlohmannJsonAdapter document_adapter(document);
  require(validator.validate(schema, document_adapter, nullptr), "invalid storage resource policy");
  for (const auto& path : document.at("profiles")) {
    load_motion_profile(root + "/config/" + path.get<std::string>(), root + "/config/motion_profile.schema.json");
  }
  return document;
}

void lifecycle_and_capacity() {
  rejects<std::invalid_argument>([] { StateStore store(0, 1); }, "zero capacity accepted");
  rejects<std::invalid_argument>([] { StateStore store(2, 0); }, "zero profiles accepted");
  rejects<std::invalid_argument>([] { StateStore store(std::numeric_limits<std::size_t>::max(), 1); },
                                "unrepresentable capacity accepted");
  StateStore store(2, 2);
  rejects<std::invalid_argument>([&] { store.spawn("bad-profile", 2, EntityMode::moving, state_at(0), desired_at(0)); },
                                "unregistered profile accepted");
  auto invalid_time = state_at(0);
  invalid_time.source_time_ns = -1;
  rejects<std::invalid_argument>([&] { store.spawn("bad-time", 0, EntityMode::moving, invalid_time, desired_at(0)); },
                                "negative source time accepted");
  const auto first = store.spawn("ownship", 0, EntityMode::moving, state_at(0), desired_at(0));
  const auto second = store.spawn("target", 1, EntityMode::fixed, state_at(1), desired_at(1));
  rejects<std::invalid_argument>([&] { store.spawn("ownship", 0, EntityMode::moving, state_at(0), desired_at(0)); },
                                "duplicate identity accepted");
  rejects<std::length_error>([&] { store.spawn("overflow", 0, EntityMode::moving, state_at(0), desired_at(0)); },
                            "capacity silently grew");
  require(store.size() == 2, "failed spawn changed entity count");
  require(store.erase(first) && !store.erase(first), "erase did not stale the handle");
  const auto replacement = store.spawn("replacement", 0, EntityMode::moving, state_at(2), desired_at(2));
  require(replacement.slot == first.slot && replacement.generation != first.generation, "slot reuse aliased old handle");
  PhysicalState state;
  DesiredMotion desired;
  require(!store.read_state(first, state, desired) && !store.find("ownship"), "removed identity survived");
  require(store.find("replacement") == replacement, "replacement lookup failed");

  StateBatch batch(2);
  store.gather(&second, 1, batch);
  store.clear();
  rejects<std::invalid_argument>([&] { store.commit(batch); }, "clear did not stale staged work");
  require(!store.read_state(replacement, state, desired), "clear left a current handle");
  const auto restarted = store.spawn("replacement", 0, EntityMode::moving, state_at(0), desired_at(0));
  require(restarted != replacement && store.size() == 1, "fresh reset reused an old generation");
  require(store.columns().active[second.slot] == 0, "clear left an occupied slot");

  StateStore holes(4, 1);
  Handle slots[4];
  for (std::size_t index = 0; index < 4; ++index)
    slots[index] = holes.spawn(std::to_string(index), 0, EntityMode::fixed, state_at(index), desired_at(index));
  holes.erase(slots[3]);
  holes.erase(slots[1]);
  require(holes.spawn("first-hole", 0, EntityMode::fixed, state_at(0), desired_at(0)).slot == 1 &&
          holes.spawn("second-hole", 0, EntityMode::fixed, state_at(0), desired_at(0)).slot == 3,
          "free-slot order depends on erase order");
}

void staged_updates_and_snapshots() {
  StateStore store(5, 2);
  const auto zulu = store.spawn("zulu", 0, EntityMode::moving, state_at(0), desired_at(0));
  const auto alpha = store.spawn("alpha", 0, EntityMode::moving, state_at(1), desired_at(1));
  store.spawn("fixed", 1, EntityMode::fixed, state_at(2), desired_at(2));
  store.spawn("disabled", 0, EntityMode::disabled, state_at(3), desired_at(3));
  StateBatch batch(5);
  store.gather_profile(0, EntityMode::moving, batch);
  require(batch.size() == 2 && batch.handles()[0] == alpha && batch.handles()[1] == zulu,
          "cohort identity order depends on slot order");
  require(batch.profile_indices()[0] == 0 && batch.modes()[0] == EntityMode::moving &&
          store.columns().profile_index[2] == 1 && store.columns().mode[2] == EntityMode::fixed,
          "cohort metadata differs from the registered model");
  Snapshot before(5);
  store.snapshot(before);
  const auto revision = before.revision();
  require(before.rows()[0].id == "alpha" && before.rows()[0].state.source_time_ns == 0,
          "snapshot is not canonical committed state");

  auto columns = batch.columns();
  columns.position_ecef_m.y[0] = 1;
  columns.source_time_ns[0] = 10000000;
  columns.position_ecef_m.y[1] = 2;
  columns.source_time_ns[1] = 10000000;
  columns.velocity_ecef_mps.y[1] = std::numeric_limits<double>::quiet_NaN();
  rejects<std::invalid_argument>([&] { store.commit(batch); }, "nonfinite batch accepted");
  require(store.columns().position_ecef_m.y[alpha.slot] == 0, "failed batch partially committed");
  Snapshot unchanged(5);
  store.snapshot(unchanged);
  require(unchanged.revision() == revision, "failed batch changed committed revision");
  columns.velocity_ecef_mps.y[1] = 102;
  store.commit(batch);
  require(before.rows()[0].state.position_ecef_m.y == 0 && before.revision() == revision,
          "previous snapshot borrowed mutable state");
  require(store.columns().position_ecef_m.y[zulu.slot] == 2, "scatter wrote the wrong slot");
  require(store.columns().source_time_ns[2] == 0, "subset update restamped untouched entities");
  rejects<std::invalid_argument>([&] { store.commit(batch); }, "batch committed twice");

  store.gather(&alpha, 1, batch);
  batch.columns().source_time_ns[0] = 0;
  rejects<std::invalid_argument>([&] { store.commit(batch); }, "source time went backward");
  StateBatch stale(5);
  store.gather(&alpha, 1, stale);
  require(store.set_mode(alpha, EntityMode::disabled), "mode update failed");
  rejects<std::invalid_argument>([&] { store.commit(stale); }, "lifecycle change did not fence staged data");
  store.gather_profile(0, EntityMode::moving, batch);
  require(batch.size() == 1 && batch.handles()[0] == zulu, "disabled entity leaked into moving cohort");

  const Handle duplicates[] = {zulu, zulu};
  rejects<std::invalid_argument>([&] { store.gather(duplicates, 2, batch); }, "duplicate update accepted");
  rejects<std::invalid_argument>([&] { store.commit(batch); }, "failed gather left usable staged data");
  StateStore foreign(1, 1);
  rejects<std::invalid_argument>([&] { foreign.commit(stale); }, "foreign batch accepted");
  Snapshot too_small(1);
  rejects<std::length_error>([&] { store.snapshot(too_small); }, "snapshot capacity silently grew");

  // A failed cohort gather must not leave old prepared rows paired with new handles.
  require(store.set_mode(alpha, EntityMode::moving), "mode restoration failed");
  StateBatch small(1);
  store.gather(&zulu, 1, small);
  rejects<std::length_error>([&] { store.gather_profile(0, EntityMode::moving, small); }, "cohort capacity exceeded");
  rejects<std::invalid_argument>([&] { store.commit(small); }, "failed cohort gather stayed prepared");
}

void scalar_and_batch_equivalence() {
  constexpr std::size_t count = 256;  // Correctness population, not a dispatch crossover.
  StateStore scalar(count, 1);
  StateStore bulk(count, 1);
  std::vector<Handle> scalar_handles;
  scalar_handles.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    std::ostringstream name;
    name << "entity-" << std::setw(4) << std::setfill('0') << index;
    scalar_handles.push_back(scalar.spawn(name.str(), 0, EntityMode::moving, state_at(index), desired_at(index)));
    bulk.spawn(name.str(), 0, EntityMode::moving, state_at(index), desired_at(index));
  }
  StateBatch one(1);
  for (const auto handle : scalar_handles) {
    scalar.gather(&handle, 1, one);
    one.columns().position_ecef_m.y[0] += 1;
    one.columns().source_time_ns[0] = 10000000;
    scalar.commit(one);
  }
  StateBatch all(count);
  bulk.gather_profile(0, EntityMode::moving, all);
  auto columns = all.columns();
  for (std::size_t index = 0; index < count; ++index) {
    columns.position_ecef_m.y[index] += 1;
    columns.source_time_ns[index] = 10000000;
  }
  bulk.commit(all);
  Snapshot a(count), b(count);
  scalar.snapshot(a);
  bulk.snapshot(b);
  for (std::size_t index = 0; index < count; ++index) {
    require(a.rows()[index].id == b.rows()[index].id &&
            a.rows()[index].state.position_ecef_m.x == b.rows()[index].state.position_ecef_m.x &&
            a.rows()[index].state.position_ecef_m.y == b.rows()[index].state.position_ecef_m.y &&
            a.rows()[index].state.source_time_ns == b.rows()[index].state.source_time_ns,
            "scalar and batch scatter differ");
    const auto expected = state_at(index);
    require(b.rows()[index].state.body_frd_to_ecef == expected.body_frd_to_ecef &&
            b.rows()[index].state.velocity_ecef_mps.y == expected.velocity_ecef_mps.y &&
            b.rows()[index].desired.horizontal_speed_mps == desired_at(index).horizontal_speed_mps &&
            b.rows()[index].desired.altitude_hae_m == desired_at(index).altitude_hae_m,
            "batch transfer lost untouched physical or desired columns");
  }
}

void steady_state_allocation(const Json& limits) {
  StateStore store(limits.at("max_entities"), limits.at("profiles").size());
  StateBatch batch(limits.at("max_batch_updates"));
  Snapshot snapshot(limits.at("max_entities"));
  const std::string long_id = "alpha-ownship-with-an-identity-longer-than-small-string-storage";
  store.spawn("zulu-target", 0, EntityMode::moving, state_at(0), desired_at(0));
  store.spawn(long_id, 0, EntityMode::moving, state_at(1), desired_at(1));
  store.snapshot(snapshot);  // Own immutable ID strings before measuring the numerical path.
  const auto* position_column = store.columns().position_ecef_m.x;
  {
    NoAllocation guard;
    for (int iteration = 1; iteration <= 3; ++iteration) {
      store.gather_profile(0, EntityMode::moving, batch);
      auto columns = batch.columns();
      for (std::size_t index = 0; index < batch.size(); ++index) {
        columns.position_ecef_m.y[index] += 1;
        columns.source_time_ns[index] = iteration * 10000000;
      }
      store.commit(batch);
      store.snapshot(snapshot);
    }
  }
  require(store.columns().position_ecef_m.x == position_column, "live columns moved after setup");
  require(snapshot.size() == 2 && snapshot.rows()[0].id == long_id, "warmed snapshot lost identity");
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  try {
    const auto limits = policy(argv[1]);
    lifecycle_and_capacity();
    staged_updates_and_snapshots();
    scalar_and_batch_equivalence();
    steady_state_allocation(limits);
    std::cout << "SoA lifecycle, atomic staging, snapshots, scalar/batch parity and warmed allocation checks passed\n";
  } catch (const std::exception& error) {
    std::cerr << "state store failed: " << error.what() << '\n';
    return 1;
  }
}
