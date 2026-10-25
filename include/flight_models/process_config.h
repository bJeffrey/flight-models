/** @file Deployment configuration for the fresh-run transport process. */
#pragma once
#include <nlohmann/json.hpp>
#include <string>

namespace flight_models {
/** Load and validate versioned deployment input before opening any transport.
 * @param path Generated runtime JSON path.
 * @param schema_path Flight-models runtime JSON Schema path.
 * @return Owning configuration, including explicit queue/time/identity limits.
 * Relative motion profile paths resolve against the generated file's directory.
 * @throws std::runtime_error for unreadable files or invalid schema definitions.
 * @throws std::invalid_argument for malformed/invalid input, identity collisions,
 *         inconsistent epoch strings, or unrepresentable integer times/limits.
 */
[[nodiscard]] nlohmann::json load_process_config(const std::string& path,
                                               const std::string& schema_path);

/** Normalize a supported scenario epoch to nine fractional UTC digits.
 * @param text UTC date-time with zero to nine fractional digits and Z suffix.
 * @return Same date/time text with exactly nine fractional digits.
 * @throws std::invalid_argument for unsupported timestamp syntax.
 * Calendar validity is checked by comparing with the integer-origin formatter.
 */
[[nodiscard]] std::string canonical_scenario_epoch(const std::string& text);
}  // namespace flight_models
