/** @file Process-only adapters for clock admission and runtime output delivery. */
#pragma once

#include "flight_models/runtime.h"

#include <nlohmann/json.hpp>

#include <memory>
#include <optional>
#include <vector>
#include <cstdint>

namespace flight_models {

/** Owns the process boundary for the fresh-run runtime.
 * DIS datagrams preserve the encoded PDU bytes and configured host/port.
 * PRD JSON is published to its configured entity topic; entity identity remains
 * in configuration and is not inserted into the report payload. Clock payloads
 * are returned unchanged so the runtime owner controls time interpretation.
 */
class RuntimeTransport {
 public:
  /** Construct from the validated process configuration (including transport/entities). */
  explicit RuntimeTransport(const nlohmann::json& config);
  ~RuntimeTransport();
  RuntimeTransport(RuntimeTransport&&) noexcept;
  RuntimeTransport& operator=(RuntimeTransport&&) noexcept;
  RuntimeTransport(const RuntimeTransport&) = delete;
  RuntimeTransport& operator=(const RuntimeTransport&) = delete;

  /** Connect transports, complete OWP INIT and start continuous error readers.
   * NATS subscription registration is flushed before readiness. No automatic
   * reconnect or replay is attempted after a failure; restoration is a later gate.
   * @throws std::runtime_error for connection, startup or deadline failures.
   */
  void open();
  /** Poll unchanged protobuf bytes; empty means the configured poll timed out.
   * Connection loss, dropped clock messages and background publication errors
   * throw std::runtime_error, including while the clock is idle.
   */
  std::optional<std::vector<std::uint8_t>> next_clock();
  /** Send outputs through per-entity DIS or PRD adapters.
   * PRD pending message and byte limits apply across all configured ASB peers;
   * capacity is reserved before posting payloads to the IO thread. Writes are
   * pipelined rather than blocking the caller for each publication ACK.
   * @throws std::runtime_error for send failure, queue overflow or peer error.
   */
  void publish(const std::vector<RuntimeOutput>& outputs);
  /** Wait up to the configured operation deadline for pending PRD ACKs.
   * An ACK establishes ASB admission, not downstream processing or offboard delivery.
   * @throws std::runtime_error for timeout or transport failure.
   */
  void flush();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace flight_models
