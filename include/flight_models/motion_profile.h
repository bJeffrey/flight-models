/** @file
 * Typed configuration interface for flight-model-owned motion profiles.
 */
#pragma once

#include <cstdint>
#include <string>

namespace flight_models {

/** Flight-model-owned speed, acceleration, turn, and altitude limits. */
struct FlightEnvelope {
  double min_speed_mps;               /**< Minimum horizontal ground speed, m/s. */
  double max_speed_mps;               /**< Maximum horizontal ground speed, m/s. */
  double max_acceleration_mps2;       /**< Maximum speed increase, m/s^2. */
  double max_deceleration_mps2;       /**< Maximum speed decrease, m/s^2. */
  double max_turn_rate_radps;         /**< Maximum heading rate, radians/s. */
  double max_climb_rate_mps;           /**< Maximum HAE increase, m/s. */
  double max_descent_rate_mps;         /**< Maximum HAE decrease magnitude, m/s. */
  double min_altitude_hae_m;           /**< Minimum WGS-84 HAE, m. */
  double max_altitude_hae_m;           /**< Maximum WGS-84 HAE, m. */
};

/** Limits integration work for each advancing simulation interval. */
struct IntegrationPolicy {
  std::int64_t max_substep_ns;             /**< Maximum numerical substep, ns. */
  std::int64_t max_interval_ns;            /**< Maximum admitted interval, ns. */
  std::int64_t max_substeps_per_interval;  /**< Maximum substeps per interval. */
};

/** Independent physical and local-navigation publication cadences. */
struct PublicationPolicy {
  std::int64_t dis_period_ns; /**< DIS sample cadence, simulation ns. */
  std::int64_t prd_period_ns; /**< PRD report cadence, simulation ns. */
};

/** Parsed, schema-checked motion profile; treat its fields as immutable after load. */
struct MotionProfile {
  std::string profile_id;            /**< Stable profile identifier. */
  std::string model;                 /**< Schema-constrained model identifier. */
  std::string navigation;            /**< Schema-constrained navigation identifier. */
  FlightEnvelope flight_envelope;    /**< Configured motion limits. */
  double waypoint_capture_radius_m;  /**< 3D capture radius in ECEF metres. */
  IntegrationPolicy integration;    /**< Configured numerical work bounds. */
  PublicationPolicy publication;     /**< Independent output cadences. */
};

/**
 * Load and validate a motion profile against its JSON Schema.
 *
 * @param path Profile JSON file path.
 * @param schema_path JSON Schema file path.
 * Integer durations/work counts require exact JSON integer tokens.
 * @return Typed profile values, including model and navigation metadata.
 * @throws std::invalid_argument If profile JSON, schema validation, or semantic
 *         profile constraints fail.
 * @throws std::runtime_error If either file cannot be read or the schema cannot
 *         be constructed.
 */
MotionProfile load_motion_profile(const std::string& path, const std::string& schema_path);

}  // namespace flight_models
