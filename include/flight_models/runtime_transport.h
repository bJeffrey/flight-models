/** @file Process-only adapters for clock admission and runtime output delivery. */
#pragma once

#include "flight_models/runtime.h"

#include <nlohmann/json.hpp>

#include <memory>
#include <optional>
#include <vector>
#include <cstdint>

namespace flight_models {

/** Kinds of ordered events admitted from the simulation bus. */
enum class RuntimeEventKind { clock, checkpoint_request };

/** Original payload for one clock or checkpoint request bus event. */
struct RuntimeEvent {
  RuntimeEventKind kind;             ///< Event kind selected by exact subject match.
  std::vector<std::uint8_t> payload; ///< Original NATS payload bytes.
};

/** Owns the runtime process transport boundary.
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
   * reconnect or replay is attempted after a failure. The optional checkpoint
   * process binding performs explicit coordinated recovery.
   * @throws std::runtime_error for connection, startup or deadline failures.
   */
  void open();
  /** Poll one clock or checkpoint request in bus delivery order.
   * One bounded NATS subscription inbox preserves the order of both subjects;
   * unrelated messages under their shared prefix are discarded before payload
   * copying. Empty means the configured wall poll timed out. Inbox overflow,
   * connection loss, dropped subscription messages and publication errors fail
   * the transport rather than silently losing control events.
   */
  std::optional<RuntimeEvent> next_event();
  /** Poll unchanged clock protobuf bytes for clock-only callers.
   * Empty means the configured poll timed out. Throws if a checkpoint request
   * is received, because discarding it would break event ordering and handling.
   * Connection loss, dropped clock messages and background publication errors
   * throw std::runtime_error, including while the clock is idle.
   */
  std::optional<std::vector<std::uint8_t>> next_clock();
  /** Publish a checkpoint response and wait for NATS to flush it.
   * @param payload Opaque response bytes for the configured checkpoint subject.
   * @throws std::logic_error if checkpoint transport is not configured.
   * @throws std::runtime_error for publication or flush failure.
   */
  void publish_checkpoint_response(const std::vector<std::uint8_t>& payload);
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
